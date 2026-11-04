"""
Event tags: the owner taps the on-road TAG button when something worth a look happens; each tap appends one
JSON line to /data/media/0/event_tags/tags.jsonl with the route, the segment and the seconds into it, GPS, speed,
cruise / openpilot state, lead, plan and alert. tools/event_tags_pack.py then packs only the qlog + qcamera
of the segments around each tag (10 s before to 20 s after by default) plus the feature_events in that window.

Rules (both learned the hard way on this car):
  * The UI must never crash because of this. The raylib render loop (selfdrive/ui/ui.py) has no try/except, so
    any exception escaping into it kills the UI and the boot logo shows for ~13 s (drive 3e: str() of an
    out-of-range capnp enum raised "Member was null"). So every value is read through a guard, enums are read
    ONLY as .raw ints (never str()), and build_tag_record() always returns a dict.
  * The UI thread does no file I/O. TagWriter.submit() only puts the record on a queue; a background writer
    thread (normal priority, nice 10) looks up the segment folder and appends the line.

Stdlib only (no openpilot imports), so the pack tool and the tests can load this file by path.
"""
import datetime
import json
import os
import queue
import re
import threading
import time

TAGS_ROOT = "/data/media/0/event_tags"
TAGS_FILE = "tags.jsonl"
MARK_FILE = "last_packed.txt"
EVENTS_ROOT = "/data/media/0/feature_events"
LOG_ROOTS = ("/data/media/0/realdata", "/data/media/0/realdata_HD", "/data/media/0/realdata_konik")
LOG_FILES = ("qlog.zst", "qlog", "rlog.zst", "rlog", "qcamera.ts")
LOCK_FILE = "rlog.lock"          # loggerd keeps it in the segment being written and removes it when the segment ends
PRE_S = 10.0                     # pack this much before a tag ...
POST_S = 20.0                    # ... and this much after it
PAD_S = 2.0                      # slack for segment boundary times
SEGMENT_S = 60.0
CHAIN_MAX_S = 90.0               # previous segment end counts as this segment's start only if this close
RECORD_VERSION = 1
GPS_SERVICES = ("gpsLocation", "gpsLocationExternal")   # Konik / comma 3X: qcomgpsd publishes gpsLocation


# ---------- safe readers (never raise) ----------
def _f(x, nd=3):
  try:
    v = float(x)
    return round(v, nd) if v == v else None
  except Exception:
    return None


def _b(x):
  try:
    return bool(x)
  except Exception:
    return None


def _i(x):
  try:
    return int(x)
  except Exception:
    return None


def _s(x):
  try:
    return x if isinstance(x, str) else (x.decode("utf-8", "replace") if isinstance(x, bytes) else None)
  except Exception:
    return None


def _get(obj, field):
  try:
    return getattr(obj, field)
  except Exception:
    return None


def _enum(obj, field):
  """Enum field as its int (.raw). Never str(): pycapnp raises on out-of-range enums."""
  try:
    return int(getattr(obj, field).raw)
  except Exception:
    return None


def _section(fn):
  try:
    return fn()
  except Exception:
    return None


def _msg(sm, name, require_valid=True):
  """sm[name] if it is alive (and valid), else None."""
  try:
    if not sm.alive.get(name, False):
      return None
    if require_valid and not sm.valid.get(name, False):
      return None
    return sm[name]
  except Exception:
    return None


# ---------- record ----------
def _boot_ns():
  try:
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)   # same clock as logMonoTime
  except Exception:
    try:
      return time.monotonic_ns()
    except Exception:
      return None


def _gps(sm, now_mono):
  best = None
  for src in GPS_SERVICES:
    g = _msg(sm, src)
    if g is None:
      continue
    if _get(g, "hasFix"):
      best = (src, g)
      break
    if best is None:
      best = (src, g)
  if best is None:
    return None
  src, g = best
  try:
    age = now_mono - float(sm.recv_time[src])
  except Exception:
    age = None
  return {
    "src": src, "fix": _b(_get(g, "hasFix")),
    "lat": _f(_get(g, "latitude"), 7), "lon": _f(_get(g, "longitude"), 7), "alt": _f(_get(g, "altitude"), 1),
    "bearing": _f(_get(g, "bearingDeg"), 1), "speed": _f(_get(g, "speed"), 2),
    "h_acc": _f(_get(g, "horizontalAccuracy"), 1), "sats": _i(_get(g, "satelliteCount")),
    "unix_ms": _i(_get(g, "unixTimestampMillis")), "age_s": _f(age, 2),
  }


def _car(sm):
  cs = _msg(sm, "carState")
  if cs is None:
    return None
  cruise = _get(cs, "cruiseState")
  return {
    "v_ego": _f(_get(cs, "vEgo"), 2), "a_ego": _f(_get(cs, "aEgo"), 2), "standstill": _b(_get(cs, "standstill")),
    "steer_deg": _f(_get(cs, "steeringAngleDeg"), 1), "steer_pressed": _b(_get(cs, "steeringPressed")),
    "gas": _b(_get(cs, "gasPressed")), "brake": _b(_get(cs, "brakePressed")),
    "left_blinker": _b(_get(cs, "leftBlinker")), "right_blinker": _b(_get(cs, "rightBlinker")),
    "gear": _enum(cs, "gearShifter"),
    "cruise_on": _b(_get(cruise, "available")), "cruise_enabled": _b(_get(cruise, "enabled")),
    "cruise_speed": _f(_get(cruise, "speed"), 2),
    "v_cruise": _f(_get(cs, "vCruise"), 1), "v_cruise_cluster": _f(_get(cs, "vCruiseCluster"), 1),
  }


def _op(sm, state):
  ss = _msg(sm, "selfdriveState")
  out = {}
  if ss is not None:
    out.update({
      "enabled": _b(_get(ss, "enabled")), "active": _b(_get(ss, "active")), "engageable": _b(_get(ss, "engageable")),
      "state": _enum(ss, "state"), "experimental": _b(_get(ss, "experimentalMode")),
      "personality": _enum(ss, "personality"),
    })
  out.update({
    "aol": _b(_get(state, "always_on_lateral_active")), "long_control": _b(_get(state, "has_longitudinal_control")),
    "traffic_mode": _b(_get(state, "traffic_mode_enabled")),
  })
  scs = _msg(sm, "starpilotCarState")
  if scs is not None:
    out.update({
      "aol_enabled": _b(_get(scs, "alwaysOnLateralEnabled")),
      "lat_paused": _b(_get(scs, "pauseLateral")), "long_paused": _b(_get(scs, "pauseLongitudinal")),
      "force_coast": _b(_get(scs, "forceCoast")), "dash_limit": _f(_get(scs, "dashboardSpeedLimit"), 2),
      "curve_offset": _f(_get(scs, "curveSlowdownOffset"), 2), "lkas_off": _b(_get(scs, "mazdaStockLkasOff")),
    })
  return out


def _alert(sm):
  out = {}
  ss = _msg(sm, "selfdriveState", require_valid=False)
  if ss is not None:
    out.update({"text1": _s(_get(ss, "alertText1")), "text2": _s(_get(ss, "alertText2")),
                "type": _s(_get(ss, "alertType")), "size": _enum(ss, "alertSize"), "status": _enum(ss, "alertStatus")})
  sps = _msg(sm, "starpilotSelfdriveState", require_valid=False)
  if sps is not None:
    out.update({"sp_text1": _s(_get(sps, "alertText1")), "sp_text2": _s(_get(sps, "alertText2")),
                "sp_sound": _enum(sps, "alertSound")})
  return out or None


def _lead(sm):
  rs = _msg(sm, "radarState")
  out = {}
  if rs is not None:
    ld = _get(rs, "leadOne")
    out.update({
      "status": _b(_get(ld, "status")), "d_rel": _f(_get(ld, "dRel"), 1), "v_rel": _f(_get(ld, "vRel"), 2),
      "v_lead": _f(_get(ld, "vLead"), 2), "y_rel": _f(_get(ld, "yRel"), 2), "a_lead_k": _f(_get(ld, "aLeadK"), 2),
      "model_prob": _f(_get(ld, "modelProb"), 2),
    })
  srs = _msg(sm, "starpilotRadarState")
  if srs is not None:
    adj = _get(srs, "adjacentStopped")
    out["adj_stopped"] = {"status": _b(_get(adj, "status")), "d_rel": _f(_get(adj, "dRel"), 1),
                          "y_rel": _f(_get(adj, "yRel"), 2)}
  return out or None


def _plan(sm):
  out = {}
  lp = _msg(sm, "longitudinalPlan")
  if lp is not None:
    out.update({"a_target": _f(_get(lp, "aTarget"), 2), "has_lead": _b(_get(lp, "hasLead")),
                "source": _enum(lp, "longitudinalPlanSource"), "should_stop": _b(_get(lp, "shouldStop"))})
  sp = _msg(sm, "starpilotPlan")
  if sp is not None:
    events = []
    try:
      for e in _get(sp, "starpilotEvents") or []:
        events.append(_enum(e, "name"))
    except Exception:
      pass
    out.update({
      "v_cruise": _f(_get(sp, "vCruise"), 2), "red_light": _b(_get(sp, "redLight")),
      "forcing_stop": _b(_get(sp, "forcingStop")), "forcing_stop_len": _f(_get(sp, "forcingStopLength"), 1),
      "approach_stop_len": _f(_get(sp, "approachStopLength"), 1), "stop_sign": _b(_get(sp, "stopSignConfirmed")),
      "tracking_lead": _b(_get(sp, "trackingLead")), "slc_limit": _f(_get(sp, "slcSpeedLimit"), 2),
      "slc_source": _s(_get(sp, "slcSpeedLimitSource")), "csc_speed": _f(_get(sp, "cscSpeed"), 2),
      "csc_on": _b(_get(sp, "cscControllingSpeed")),
      "map_curve": {
        "mode": _i(_get(sp, "mapCurveMode")), "status": _i(_get(sp, "mapCurveStatus")),
        "distance": _f(_get(sp, "mapCurveDistance"), 1), "radius": _f(_get(sp, "mapCurveRadius"), 1),
        "name": _s(_get(sp, "mapCurveName")), "veto": _i(_get(sp, "mapCurveVeto")),
        "source": _i(_get(sp, "mapCurveSource")), "gps_trust": _i(_get(sp, "mapGpsTrust")),
        "gps_acc": _f(_get(sp, "mapGpsAccuracy"), 1),
      },
      "events": events,
    })
  return out or None


def _controls(sm):
  c = _msg(sm, "controlsState")
  if c is None:
    return None
  return {"curvature": _f(_get(c, "curvature"), 5), "desired_curvature": _f(_get(c, "desiredCurvature"), 5),
          "long_state": _enum(c, "longControlState")}


def _map(sm):
  m = _msg(sm, "mapdOut")
  if m is None:
    return None
  return {"road": _s(_get(m, "roadName")), "limit": _f(_get(m, "speedLimit"), 2)}


def _names(record, names):
  """Add *_name strings for a few enums when a {field: {int: name}} map is given (built by the UI widget)."""
  if not names:
    return
  for sec, field, key in (("op", "state", "state"), ("car", "gear", "gear"), ("op", "personality", "personality")):
    try:
      v = record[sec][field]
      if v is not None and v in names.get(key, {}):
        record[sec][field + "_name"] = names[key][v]
    except Exception:
      pass
  try:
    ev = record["plan"]["events"]
    record["plan"]["event_names"] = [names.get("events", {}).get(e, e) for e in ev]
  except Exception:
    pass


def build_tag_record(sm, state, now_wall, now_mono, names=None):
  """One tag as a JSON-ready dict. Never raises; reads only in-memory messages (no file I/O)."""
  record = {"v": RECORD_VERSION, "unix": _f(now_wall, 3), "mono": _f(now_mono, 3), "boot_ns": _boot_ns()}
  try:
    record["time"] = datetime.datetime.fromtimestamp(now_wall).strftime("%Y-%m-%d %H:%M:%S")
  except Exception:
    pass
  try:
    record["gps"] = _section(lambda: _gps(sm, now_mono))
    record["car"] = _section(lambda: _car(sm))
    record["op"] = _section(lambda: _op(sm, state))
    record["alert"] = _section(lambda: _alert(sm))
    record["lead"] = _section(lambda: _lead(sm))
    record["plan"] = _section(lambda: _plan(sm))
    record["controls"] = _section(lambda: _controls(sm))
    record["map"] = _section(lambda: _map(sm))
    _names(record, names)
  except Exception:
    pass
  return record


# ---------- segment lookup ----------
def locate_current_segment(route, now_wall, roots=LOG_ROOTS):
  """(segment, seconds into it, log root, how) for the segment of `route` being written now.
  The segment with rlog.lock is the open one; its lock mtime = segment start. Without a lock, the newest
  segment by number and its folder mtime (a guess: a closed segment's folder mtime is its END, because
  loggerd removes the lock file then)."""
  if not route:
    return None, None, None, None
  newest = None
  prefix = route + "--"
  for root in roots:
    try:
      names = os.listdir(root)
    except OSError:
      continue
    for name in names:
      if not name.startswith(prefix):
        continue
      try:
        n = int(name[len(prefix):])
      except ValueError:
        continue
      path = os.path.join(root, name)
      try:
        lock = os.path.join(path, LOCK_FILE)
        if os.path.exists(lock):
          return n, max(now_wall - os.stat(lock).st_mtime, 0.0), root, "lock"
        if newest is None or n > newest[0]:
          newest = (n, os.stat(path).st_mtime, root)
      except OSError:
        continue
  if newest is None:
    return None, None, None, None
  return newest[0], max(now_wall - newest[1], 0.0), newest[2], "mtime"


# ---------- writer ----------
class TagWriter:
  """submit() is called on the UI thread: no I/O there, just a queue put. The writer thread does the rest."""

  def __init__(self, root=TAGS_ROOT, params=None, clock=time.time, write_in_thread=True, roots=LOG_ROOTS):  # noqa: TID251  (wall clock: matched to log file mtimes)
    self.root = root
    self.params = params
    self.clock = clock
    self.roots = roots
    self.write_in_thread = write_in_thread
    self.written = 0
    self.failed = 0
    self._next = None              # next tag number, known once the writer read the file
    self._lock = threading.Lock()
    self.queue = queue.Queue(maxsize=100)
    self._thread = None
    if write_in_thread:
      try:
        self._thread = threading.Thread(target=self._writer_loop, name="event_tags", daemon=True)
        self._thread.start()
      except Exception:
        self._thread = None

  @property
  def path(self):
    return os.path.join(self.root, TAGS_FILE)

  def submit(self, record):
    """Queue one record; returns the tag number when already known (else None). Never raises, never blocks."""
    try:
      with self._lock:
        n = self._next
        if n is not None:
          self._next = n + 1
      record["n"] = n
      if self.write_in_thread and self._thread is not None:
        self.queue.put_nowait(record)
      elif not self.write_in_thread:
        self._write(record)
      else:
        self.failed += 1
      return n
    except Exception:
      self.failed += 1
      return None

  @staticmethod
  def _leave_caller_priority():
    try:
      os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    except (AttributeError, OSError):
      pass
    try:
      os.nice(10)
    except OSError:
      pass

  def _writer_loop(self):
    self._leave_caller_priority()
    self._load_counter()
    while True:
      try:
        self._write(self.queue.get())
      except Exception:
        self.failed += 1

  def _load_counter(self):
    last = 0
    try:
      for t in read_tags(self.path):
        last = max(last, _i(t.get("n")) or 0)
    except Exception:
      pass
    with self._lock:
      if self._next is None:
        self._next = last + 1
    return last

  def _route(self):
    try:
      r = self.params.get("CurrentRoute") if self.params is not None else None
      return _s(r) if r is not None else None
    except Exception:
      return None

  def _write(self, record):
    try:
      if record.get("n") is None:
        if self._next is None:
          self._load_counter()
        with self._lock:
          record["n"] = self._next
          self._next += 1
      route = self._route()
      wall = record.get("unix") or self.clock()
      seg, seg_s, root, how = locate_current_segment(route, wall, self.roots)
      record.update({"route": route, "segment": seg, "segment_s": _f(seg_s, 1), "log_root": root, "segment_from": how})
      os.makedirs(self.root, exist_ok=True)
      line = json.dumps(record, separators=(",", ":"), default=str)
      with open(self.path, "a") as f:
        f.write(line + "\n")
        f.flush()
        os.fsync(f.fileno())
      self.written += 1
    except Exception:
      self.failed += 1


# ---------- pack side (pure functions; used by tools/event_tags_pack.py) ----------
def read_tags(path):
  out = []
  try:
    with open(path) as f:
      for line in f:
        line = line.strip()
        if not line:
          continue
        try:
          t = json.loads(line)
          if isinstance(t, dict):
            out.append(t)
        except ValueError:
          continue
  except OSError:
    pass
  return out


def filter_tags(tags, last=None, since=None, numbers=None, hours=None, route=None, after_n=None, now=None):
  sel = list(tags)
  if numbers:
    want = set(numbers)
    sel = [t for t in sel if t.get("n") in want]
  if since is not None:
    sel = [t for t in sel if (t.get("n") or 0) >= since]
  if after_n is not None:
    sel = [t for t in sel if (t.get("n") or 0) > after_n]
  if hours is not None:
    now = time.time() if now is None else now  # noqa: TID251  (wall clock: matched to log file mtimes)
    sel = [t for t in sel if (t.get("unix") or 0) >= now - hours * 3600]
  if route:
    sel = [t for t in sel if route in (t.get("route") or "")]
  if last is not None:
    sel = sel[-last:] if last > 0 else []
  return sel


class SegSpan:
  __slots__ = ("path", "start", "end", "in_progress")

  def __init__(self, path, start, end, in_progress=False):
    self.path, self.start, self.end, self.in_progress = path, start, end, in_progress

  def __repr__(self):
    return f"SegSpan({self.path!r}, {self.start:.1f}, {self.end:.1f}, {self.in_progress})"


def scan_segments(roots=LOG_ROOTS, now=None):
  """{(route, n): SegSpan}. Start/end wall times: an open segment (rlog.lock) starts at the lock mtime and runs
  to now; a closed one ends at its newest log file mtime and starts where the previous one ended (or end - 60)."""
  now = time.time() if now is None else now  # noqa: TID251  (wall clock: matched to log file mtimes)
  raw = {}
  for root in roots:
    try:
      names = os.listdir(root)
    except OSError:
      continue
    for name in names:
      m = re.match(r"^(.+)--(\d+)$", name)
      if not m:
        continue
      path = os.path.join(root, name)
      try:
        if not os.path.isdir(path):
          continue
        lock = os.path.join(path, LOCK_FILE)
        if os.path.exists(lock):
          raw[(m.group(1), int(m.group(2)))] = (path, os.stat(lock).st_mtime, now, True)
          continue
        mt = [os.stat(os.path.join(path, f)).st_mtime for f in LOG_FILES if os.path.exists(os.path.join(path, f))]
        end = max(mt) if mt else os.stat(path).st_mtime
        raw[(m.group(1), int(m.group(2)))] = (path, None, end, False)
      except OSError:
        continue
  spans = {}
  for (route, n) in sorted(raw):
    path, start, end, live = raw[(route, n)]
    if start is None:
      prev = raw.get((route, n - 1))
      if prev is not None and 0 < end - prev[2] <= CHAIN_MAX_S:
        start = prev[2]
      else:
        start = end - SEGMENT_S
    spans[(route, n)] = SegSpan(path, start, end, live)
  return spans


def select_segments(tag, spans, pre=PRE_S, post=POST_S, pad=PAD_S):
  """([(route, n) found on disk], [(route, n) wanted but missing]) covering [tag - pre, tag + post]."""
  route = tag.get("route")
  wall = tag.get("unix")
  want = set()
  if wall is not None:
    lo, hi = wall - pre - pad, wall + post + pad
    for (r, n), sp in spans.items():
      if (route is None or r == route) and sp.start <= hi and sp.end >= lo:
        want.add((r, n))
  seg, seg_s = tag.get("segment"), tag.get("segment_s")
  if route and seg is not None:
    want.add((route, seg))
    if seg_s is not None:
      if seg_s < pre + pad and seg > 0:
        want.add((route, seg - 1))
      if seg_s + post + pad > SEGMENT_S:
        want.add((route, seg + 1))
  found = sorted(k for k in want if k in spans)
  missing = sorted(k for k in want if k not in spans)
  return found, missing


def select_feature_events(events_root, tags, pre=PRE_S, post=POST_S, pad=PAD_S):
  """Folders in feature_events whose summary time_unix falls in any tag's window (their own 15 s lead-in counts)."""
  windows = [(t["unix"] - pre - pad - 20.0, t["unix"] + post + pad) for t in tags if t.get("unix") is not None]
  out = []
  try:
    names = sorted(os.listdir(events_root))
  except OSError:
    return out
  for name in names:
    summ = os.path.join(events_root, name, "summary.txt")
    try:
      with open(summ) as f:
        t = None
        for line in f:
          if line.startswith("time_unix:"):
            t = float(line.split(":", 1)[1].strip())
            break
    except (OSError, ValueError):
      continue
    if t is not None and any(lo <= t <= hi for lo, hi in windows):
      out.append(os.path.join(events_root, name))
  return out


def summary_line(tag):
  def g(sec, k):
    try:
      return (tag.get(sec) or {}).get(k)
    except Exception:
      return None
  v = g("car", "v_ego")
  mph = f"{v * 2.23694:.0f} mph" if isinstance(v, (int, float)) else "? mph"
  setk = g("car", "v_cruise_cluster") or g("car", "v_cruise")        # km/h; 255 = not set
  setp = f"{setk * 0.621371:.0f}" if isinstance(setk, (int, float)) and 0 < setk < 250 else "-"
  lead = g("lead", "d_rel") if g("lead", "status") else None
  alert = g("alert", "text1") or g("alert", "sp_text1") or ""
  gps = tag.get("gps") or {}
  acc = gps.get("h_acc")
  parts = [f"#{tag.get('n')}", str(tag.get("time", "")),
           f"{tag.get('route')}--{tag.get('segment')} @ {tag.get('segment_s')} s", mph, f"set {setp} mph",
           f"engaged {g('op', 'enabled')}", f"lead {lead if lead is not None else '-'} m",
           f"gps acc {acc if acc is not None else '-'} m", alert]
  return "  ".join(parts).rstrip()
