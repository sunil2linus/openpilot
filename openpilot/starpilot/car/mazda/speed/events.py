"""Feature-event log of the Mazda speed control: one JSON object per line in /data/media/0/mazda_speed/, written by a
background thread at normal priority (card never waits on the disk). Never raises; drops lines when the queue is
full or the file is at its size cap. No GPS position is ever written here (curve ids and distances only)."""
import json
import os
import queue
import threading
import time

LOG_DIR = "/data/media/0/mazda_speed"
_MISSED = set()                 # cereal fields already reported missing in this process (schema_miss)
MAX_BYTES = 20_000_000          # per drive file
KEEP_FILES = 30
QUEUE = 2000


def leave_caller_priority():
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


class EventLog:
  def __init__(self, directory=LOG_DIR, name=None, background=True, clock=time.monotonic):
    self.directory = directory
    self.name = name or time.strftime("events-%Y%m%d-%H%M%S.jsonl")
    self.path = os.path.join(directory, self.name)
    self.clock = clock
    self.q = queue.Queue(maxsize=QUEUE)
    self.dropped = 0
    self.written = 0
    self.background = background
    self.closed = False
    self.thread = None
    if background:
      self.thread = threading.Thread(target=self._run, daemon=True)
      self.thread.start()

  def log(self, kind, **fields):
    if self.closed:
      return
    row = {"t": round(self.clock(), 3), "kind": kind, **fields}
    try:
      self.q.put_nowait(row)
    except queue.Full:
      self.dropped += 1
    if not self.background:
      self.flush()

  def _open(self):
    os.makedirs(self.directory, exist_ok=True)
    try:
      names = sorted(n for n in os.listdir(self.directory) if n.startswith("events-") and n.endswith(".jsonl"))
      for old in names[:-KEEP_FILES]:
        os.remove(os.path.join(self.directory, old))
    except OSError:
      pass
    return open(self.path, "a", buffering=1)

  def flush(self):
    f = None
    try:
      while True:
        try:
          row = self.q.get_nowait()
        except queue.Empty:
          break
        if f is None:
          f = self._open()
        line = json.dumps(row, separators=(",", ":"), default=str) + "\n"
        if f.tell() + len(line) > MAX_BYTES:
          self.dropped += 1
          continue
        f.write(line)
        self.written += 1
    except (OSError, ValueError, TypeError):
      self.dropped += 1
    finally:
      if f is not None:
        try:
          f.close()
        except OSError:
          pass

  def _run(self):
    leave_caller_priority()
    while not self.closed:
      time.sleep(0.5)
      self.flush()

  def close(self):
    self.flush()
    self.closed = True


def schema_miss(warn, what, err=None):
  """A cereal field our code reads is not in this build's schema (a renamed field): never silent. warn: the drive's
  reporter (contributors.Sources.warn: one event-log line per field per drive); None: one cloudlog warning per field
  per process. Never raises."""
  try:
    if warn is not None:
      warn(what, err)
      return
    if what in _MISSED:
      return
    _MISSED.add(what)
    from openpilot.common.swaglog import cloudlog
    cloudlog.warning(f"mazda speed control: cereal field missing: {what} ({str(err)[:160]})")
  except Exception:
    pass
