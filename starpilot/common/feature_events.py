"""
Standalone per-event logs for the optional driving features (stop warnings, curve slowdown).

Each event becomes a folder /data/media/0/feature_events/<YYYY-MM-DD_HH-MM-SS>_<kind>/ with
  summary.txt   what fired and the values it saw, route name, segment number and seconds into
                the segment (so the matching qcamera minute can be found later)
  signals.csv   the relevant signals at 10-20 Hz from PRE_SECONDS before the event to
                POST_SECONDS after it, kept in a small in-memory ring buffer

Rules:
  * Nothing is written while the event is going on: the folder is written once, after the
    post window, by a separate writer PROCESS (one per openpilot process, started once). The caller
    never formats or writes a file: each tick it pickles at most PIPE_CHUNK_ROWS rows of one ready
    event (C code, a fraction of a ms) into a non-blocking pipe. History: drive 3b, one 2,000-row
    event written in one go froze card for 0.1-0.3 s ("Communication Issue"); the fix then was a
    writer THREAD with chunks and pauses. Drive 44 (Nov 4): a burst of nine events written by that
    thread still held card's GIL often enough to delay carState by 20-55 ms for seconds, and
    selfdrived raised "System Lagging" (TAKE CONTROL IMMEDIATELY). A thread shares the GIL; a process
    does not. The thread path is kept only as a fallback if the process cannot be started.
  * Events are capped at MAX_ROWS rows (evenly thinned); pruning runs rarely, in the writer.
  * Point events that do not need the signal ring (signals=False) write summary.txt only.
  * Every public method swallows its own exceptions: this must never break the planner or
    the car controller. The worst case is a missing event folder.
  * The folder is outside realdata, so the log deleter never touches it; it is capped at
    MAX_BYTES / MAX_EVENTS by pruning the oldest events.
  * No video is written. tools/feature_events_pack.sh collects the events plus only the
    qcamera.ts files of the referenced segments.

The route name comes from the "CurrentRoute" param (set by loggerd for the current drive).
The segment number and the seconds into it are derived from the newest segment folder of that
route in the realdata directories at write time (directory mtime = when its files were
created = segment start); the wall-clock and monotonic times are logged too, so the pack
script can re-derive the segment from file mtimes if that lookup failed.
"""
import datetime
import os
import pickle
import queue
import shutil
import struct
import subprocess
import sys
import threading
import time
from collections import deque

EVENTS_ROOT = "/data/media/0/feature_events"
LOG_ROOTS = ("/data/media/0/realdata", "/data/media/0/realdata_HD", "/data/media/0/realdata_konik")
MAX_BYTES = 50_000_000
MAX_EVENTS = 300
PRE_SECONDS = 15.0
POST_SECONDS = 5.0
MAX_OPEN_SECONDS = 180.0        # an episode still open after this is written as it is
MAX_ROWS = 1200                 # rows per event file at most (longer events are evenly thinned)
WRITE_CHUNK_ROWS = 20           # rows formatted per step in the writer ...
WRITE_PAUSE = 0.005             # ... then the writer pauses this long (s), so the caller always gets the GIL
PRUNE_EVERY = 600.0             # s between two prunes of the event folder
NEAR_MISS_SUFFIX = "_nearMiss"  # events of this kind are rate limited and pruned first
NEAR_MISS_MIN_GAP = 20.0        # s between two near-miss events of the same kind
USE_WRITER_PROCESS = True       # write in a separate process (see Rules); False = the old writer thread
PIPE_CHUNK_ROWS = 20            # rows pickled per tick by the caller
PUMP_MIN_GAP = 0.004            # s: at most one pipe message per this, per process (several recorders share it)
F_SETPIPE_SZ = 1031             # fcntl: ask for a bigger pipe buffer (Linux)
PIPE_SIZE = 1 << 20


def _fmt(v):
  if isinstance(v, bool):
    return "1" if v else "0"
  if isinstance(v, float):
    return f"{v:.3f}"
  if v is None:
    return ""
  return str(v)


def locate_segment(route, now_wall, roots=LOG_ROOTS):
  """(segment number, seconds into it, log root) of the newest segment folder of `route`, or (None, None, None)."""
  if not route:
    return None, None, None
  best = None
  for root in roots:
    try:
      names = os.listdir(root)
    except OSError:
      continue
    prefix = route + "--"
    for name in names:
      if name.startswith(prefix):
        try:
          num = int(name[len(prefix):])
          start = os.stat(os.path.join(root, name)).st_mtime
        except (ValueError, OSError):
          continue
        if best is None or num > best[0]:
          best = (num, start, root)
  if best is None:
    return None, None, None
  return best[0], max(now_wall - best[1], 0.0), best[2]


class EventRecorder:
  """Ring buffer of signal rows plus event folders written after the post window."""

  def __init__(self, columns, hz, root=EVENTS_ROOT, params=None, pre=PRE_SECONDS, post=POST_SECONDS,
               max_bytes=MAX_BYTES, max_events=MAX_EVENTS, clock=time.time, write_in_thread=True):
    self.columns = list(columns)
    self.root = root
    self.params = params
    self.pre, self.post = pre, post
    self.max_bytes, self.max_events = max_bytes, max_events
    self.clock = clock
    self.write_in_thread = write_in_thread
    self.buffer = deque(maxlen=int((pre + post) * hz) + 2)
    self.open_events = []          # events still collecting rows
    self.pending = []              # closed events waiting for the post window
    self.last_open = {}            # kind -> t of the last event opened (rate limit for near-misses)
    self.written = 0
    self.failed = 0
    self.skipped = 0
    self.write_pause = WRITE_PAUSE if write_in_thread else 0.0
    self.queue = None              # writer thread input (started on the first event)
    self.last_prune = None
    self.writer = None             # the shared writer process (USE_WRITER_PROCESS)
    if write_in_thread and USE_WRITER_PROCESS:
      try:
        self.writer = get_writer()
      except Exception:
        self.writer = None

  # ---- collecting ----
  def add_row(self, t, values):
    """Append one row (t = monotonic seconds, values in the order of `columns`)."""
    try:
      row = (t, tuple(values))
      self.buffer.append(row)
      for ev in self.open_events:
        if ev["signals"]:
          ev["rows"].append(row)
      for ev in self.pending:
        if ev["signals"] and t <= ev["t_end"] + self.post:
          ev["rows"].append(row)
    except Exception:
      pass

  def open(self, kind, t, summary, min_gap=None, signals=True):
    """Start an event at t: keeps the rows from t - pre on until close() (signals=False: summary only).

    Near-miss kinds (name ending in NEAR_MISS_SUFFIX) are rate limited to one per
    NEAR_MISS_MIN_GAP per kind, unless `min_gap` says otherwise; returns None when skipped."""
    try:
      if min_gap is None and kind.endswith(NEAR_MISS_SUFFIX):
        min_gap = NEAR_MISS_MIN_GAP
      if min_gap is not None and kind in self.last_open and t - self.last_open[kind] < min_gap:
        self.skipped += 1
        return None
      self.last_open[kind] = t
      ev = {"kind": kind, "t_start": t, "t_end": None, "wall": self.clock(), "summary": dict(summary),
            "signals": bool(signals), "rows": [r for r in self.buffer if r[0] >= t - self.pre] if signals else []}
      self.open_events.append(ev)
      return ev
    except Exception:
      return None

  def close(self, ev, t, extra=None):
    """End an event at t; it is written once t + post has passed."""
    try:
      if ev is None or ev not in self.open_events:
        return
      ev["t_end"] = t
      if extra:
        ev["summary"].update(extra)
      self.open_events.remove(ev)
      self.pending.append(ev)
    except Exception:
      pass

  def record(self, kind, t, summary, min_gap=None, signals=True):
    """A point event: open and close at the same time. Returns False when rate limited."""
    ev = self.open(kind, t, summary, min_gap, signals)
    self.close(ev, t)
    return ev is not None

  def tick(self, t):
    """Call every frame: writes the events whose post window has elapsed (in a background thread)."""
    try:
      for ev in list(self.open_events):
        if t - ev["t_start"] > MAX_OPEN_SECONDS:
          self.close(ev, t, {"note": "still open after MAX_OPEN_SECONDS"})
      ready = [ev for ev in self.pending if t >= ev["t_end"] + self.post]
      if self.writer is not None:
        ready = ready[:1]          # one hand-off per tick: thinning a long event is the caller's biggest cost
      for ev in ready:
        self.pending.remove(ev)
        # rows after t_end + post are not of interest (the buffer may already hold a few)
        ev["rows"] = [r for r in ev["rows"] if r[0] <= ev["t_end"] + self.post]
        if self.writer is not None:
          if self.writer.broken:
            self.failed += 1
          else:
            self.writer.submit(self, self._header(ev), self._thin(ev["rows"]))
        elif self.write_in_thread:
          self._start_writer()
          self.queue.put(ev)
        else:
          self._write(ev)
      if self.writer is not None:
        self.writer.pump(time.monotonic())
    except Exception:
      pass

  # ---- writing (background) ----
  def _start_writer(self):
    if self.queue is None:
      self.queue = queue.Queue()
      threading.Thread(target=self._writer_loop, daemon=True).start()

  @staticmethod
  def _leave_caller_priority():
    _leave_caller_priority()

  def _writer_loop(self):
    self._leave_caller_priority()
    while True:
      ev = self.queue.get()
      self._write(ev)

  @staticmethod
  def _thin(rows, max_rows=MAX_ROWS):
    """At most max_rows rows, evenly spread over the event (first and last kept)."""
    n = len(rows)
    if n <= max_rows:
      return rows
    step = (n - 1) / (max_rows - 1)
    return [rows[round(i * step)] for i in range(max_rows)]

  def _route(self):
    try:
      if self.params is not None:
        return self.params.get("CurrentRoute", encoding="utf8")
    except Exception:
      pass
    return None

  def _header(self, ev):
    """Everything the writer needs except the rows, as plain picklable data."""
    return {"kind": ev["kind"], "wall": ev["wall"], "t_start": ev["t_start"], "t_end": ev["t_end"],
            "summary": [(k, _fmt(v)) for k, v in ev["summary"].items()], "route": self._route(),
            "signals": ev.get("signals", True), "columns": self.columns, "root": self.root,
            "max_bytes": self.max_bytes, "max_events": self.max_events}

  def _write(self, ev):
    """Thread / synchronous path."""
    ok = _write_event(self._header(ev), self._thin(ev["rows"]), self.write_pause)
    ev["rows"] = []
    if ok:
      self.written += 1
    else:
      self.failed += 1
    now = time.monotonic()
    if self.last_prune is None or now - self.last_prune > PRUNE_EVERY:
      self.last_prune = now
      prune(self.root, self.max_bytes, self.max_events)


def _leave_caller_priority():
  """A writer inherits the caller's real-time priority and core (card: SCHED_FIFO on one core).
  On Linux pid 0 = the calling thread / process: normal priority, the other cores, nice 10."""
  try:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  except (AttributeError, OSError, PermissionError):
    pass
  try:
    cores = os.sched_getaffinity(0)
    others = set(range(os.cpu_count() or 1)) - cores
    if others:
      os.sched_setaffinity(0, others)
  except (AttributeError, OSError):
    pass
  try:
    os.nice(10)
  except OSError:
    pass


def _write_event(h, rows, write_pause=0.0):
  """Write one event folder from its header (EventRecorder._header) and thinned rows. True when written."""
  try:
    wall = h["wall"]
    stamp = datetime.datetime.fromtimestamp(wall).strftime("%Y-%m-%d_%H-%M-%S")
    folder = os.path.join(h["root"], f"{stamp}_{h['kind']}")
    n = 1
    while os.path.exists(folder):
      n += 1
      folder = os.path.join(h["root"], f"{stamp}_{h['kind']}_{n}")
    route = h["route"]
    segment, seconds, log_root = locate_segment(route, wall)
    lines = [
      f"kind: {h['kind']}",
      f"time: {datetime.datetime.fromtimestamp(wall).isoformat(timespec='milliseconds')}",
      f"time_unix: {wall:.3f}",
      f"time_monotonic: {h['t_start']:.3f}",
      f"duration_s: {max(h['t_end'] - h['t_start'], 0.0):.2f}",
      f"route: {route or ''}",
      f"segment: {'' if segment is None else segment}",
      f"segment_seconds: {'' if seconds is None else f'{seconds:.1f}'}",
      f"segment_source: {log_root or ''}",
      f"written_after_s: {time.time() - wall:.1f}",
      f"monotonic_at_write: {time.monotonic():.3f}",
    ]
    lines += [f"{k}: {v}" for k, v in h["summary"]]
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "summary.txt"), "w") as f:
      f.write("\n".join(lines) + "\n")
    if h.get("signals", True):
      t0 = h["t_start"]
      with open(os.path.join(folder, "signals.csv"), "w") as f:
        f.write("t_rel," + ",".join(h["columns"]) + "\n")
        for i in range(0, len(rows), WRITE_CHUNK_ROWS):
          f.write("".join(f"{t - t0:.2f}," + ",".join(_fmt(v) for v in vals) + "\n"
                          for t, vals in rows[i:i + WRITE_CHUNK_ROWS]))
          if write_pause:
            time.sleep(write_pause)
    return True
  except Exception:
    return False


# ---- the writer process ----
class WriterProcess:
  """One per openpilot process. The parent pickles small messages into a non-blocking pipe (stdin of
  a fresh python running this file with --writer); the child formats and writes. Acks (event id,
  ok) come back on its stdout. Nothing here may raise into the caller."""

  def __init__(self, argv=None):
    self.owners = {}               # event id -> EventRecorder (for written / failed)
    self.outbox = deque()          # [id, header, rows, pos]
    self.outbuf = b""
    self.ackbuf = b""
    self.next_id = 1
    self.last_pump = -1.0
    self.broken = False
    self.pid_owner = os.getpid()
    cmd = argv or [sys.executable, "-u", os.path.abspath(__file__), "--writer"]
    self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 close_fds=True, start_new_session=True)
    self.w = self.proc.stdin.fileno()
    self.r = self.proc.stdout.fileno()
    try:
      import fcntl
      fcntl.fcntl(self.w, F_SETPIPE_SZ, PIPE_SIZE)
    except Exception:
      pass
    os.set_blocking(self.w, False)
    os.set_blocking(self.r, False)

  def submit(self, rec, header, rows):
    ev_id = self.next_id
    self.next_id += 1
    self.owners[ev_id] = rec
    self.outbox.append([ev_id, header, rows, -1])

  def pending(self):
    return len(self.outbox) + (1 if self.outbuf else 0)

  def _break(self):
    self.broken = True
    for item in self.outbox:
      rec = self.owners.pop(item[0], None)
      if rec is not None:
        rec.failed += 1
    self.outbox.clear()
    self.outbuf = b""

  def _read_acks(self):
    try:
      data = os.read(self.r, 4096)
    except BlockingIOError:
      return
    except OSError:
      self._break()
      return
    if not data:
      if self.proc.poll() is not None:
        self._break()
      return
    self.ackbuf += data
    while len(self.ackbuf) >= 5:
      ev_id, ok = struct.unpack_from("<IB", self.ackbuf)
      self.ackbuf = self.ackbuf[5:]
      rec = self.owners.pop(ev_id, None)
      if rec is not None:
        if ok:
          rec.written += 1
        else:
          rec.failed += 1

  def pump(self, now, force=False):
    """At most one message per PUMP_MIN_GAP: pickle the next chunk, push what the pipe takes."""
    try:
      if self.broken or (not force and now - self.last_pump < PUMP_MIN_GAP):
        return
      self.last_pump = now
      self._read_acks()
      if not self.outbuf and self.outbox:
        item = self.outbox[0]
        ev_id, header, rows, pos = item
        if pos < 0:
          msg = ("open", ev_id, header)
          item[3] = 0
        elif pos < len(rows):
          msg = ("rows", ev_id, rows[pos:pos + PIPE_CHUNK_ROWS])
          item[3] = pos + PIPE_CHUNK_ROWS
        else:
          msg = ("end", ev_id)
          self.outbox.popleft()
        data = pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL)
        self.outbuf = struct.pack("<I", len(data)) + data
      if self.outbuf:
        try:
          n = os.write(self.w, self.outbuf)
          self.outbuf = self.outbuf[n:]
        except BlockingIOError:
          pass
        except OSError:
          self._break()
    except Exception:
      self._break()

  def close(self):
    try:
      self.proc.stdin.close()
    except Exception:
      pass


_writer = None


def get_writer():
  """The process-wide writer (started on first use; a forked child never reuses its parent's)."""
  global _writer
  if _writer is None or _writer.pid_owner != os.getpid():
    _writer = WriterProcess()
  return _writer


def _writer_main(fin=None, fout=None):
  """The writer process: read length-prefixed pickles from stdin, write folders, ack on stdout."""
  _leave_caller_priority()
  try:
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)
  except Exception:
    pass
  fin = fin if fin is not None else sys.stdin.buffer
  fout = fout if fout is not None else sys.stdout.buffer
  events = {}
  last_prune = None
  while True:
    head = fin.read(4)
    if len(head) < 4:
      return
    n = struct.unpack("<I", head)[0]
    body = fin.read(n)
    if len(body) < n:
      return
    try:
      msg = pickle.loads(body)
      kind, ev_id = msg[0], msg[1]
      if kind == "open":
        events[ev_id] = (msg[2], [])
      elif kind == "rows":
        events[ev_id][1].extend(msg[2])
      elif kind == "end":
        header, rows = events.pop(ev_id)
        ok = _write_event(header, rows)
        fout.write(struct.pack("<IB", ev_id, 1 if ok else 0))
        fout.flush()
        now = time.monotonic()
        if ok and (last_prune is None or now - last_prune > PRUNE_EVERY):
          last_prune = now
          prune(header["root"], header["max_bytes"], header["max_events"])
    except Exception:
      try:
        fout.write(struct.pack("<IB", int(msg[1]), 0))
        fout.flush()
      except Exception:
        pass


def _folder_size(path):
  total = 0
  try:
    with os.scandir(path) as it:
      for e in it:
        try:
          total += e.stat().st_size
        except OSError:
          pass
  except OSError:
    pass
  return total


def prune(root, max_bytes=MAX_BYTES, max_events=MAX_EVENTS):
  """Delete event folders until the count and the total size are within the caps:
  the oldest near-misses first, then the oldest of the rest."""
  try:
    entries = sorted(e for e in os.listdir(root) if os.path.isdir(os.path.join(root, e)))
    sizes = {e: _folder_size(os.path.join(root, e)) for e in entries}
    total = sum(sizes.values())
    removed = 0
    while entries and (len(entries) > max_events or total > max_bytes):
      victim = next((e for e in entries if NEAR_MISS_SUFFIX in e), entries[0])
      entries.remove(victim)
      shutil.rmtree(os.path.join(root, victim), ignore_errors=True)
      total -= sizes[victim]
      removed += 1
    return removed
  except Exception:
    return 0


class NearMissTracker:
  """Turns a per-frame "candidate seen but the trigger did not fire" condition into one near-miss
  event per candidate episode, written at the end of the episode with the conditions that blocked it."""

  def __init__(self, recorder, kind, min_hold=0.3, gap=1.0):
    self.recorder = recorder
    self.kind = kind + NEAR_MISS_SUFFIX
    self.min_hold = min_hold
    self.gap = gap
    self.start = None
    self.last_true = None
    self.fired = False
    self.blockers = []
    self.info = {}

  def update(self, t, candidate, fired, blockers=(), info=None):
    """candidate: the loose condition is true this frame; fired: the real trigger fired this frame."""
    try:
      if candidate:
        if self.start is None:
          self.start, self.fired, self.info = t, False, {}
        self.last_true = t
        self.blockers = list(blockers)
        if info:
          self.info.update(info)
      if fired and self.start is not None:
        self.fired = True
      if self.start is not None and not candidate and t - self.last_true >= self.gap:
        held = self.last_true - self.start
        if not self.fired and held >= self.min_hold:
          summary = {"candidate_start_monotonic": self.start, "candidate_held_s": held,
                     "blocked_by": "; ".join(self.blockers) if self.blockers else "nothing (debounce or rate limit)"}
          summary.update(self.info)
          self.recorder.record(self.kind, self.last_true, summary)
        self.start = None
    except Exception:
      self.start = None


if __name__ == "__main__" and "--writer" in sys.argv:
  _writer_main()
