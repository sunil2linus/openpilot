"""
Per-curve memory (simplified port of the old fork's CurveMemory) and the curve speed offset rules.

Key: the curve's own (first) OSM way id + our direction through it ("<way>:ab" / "<way>:ba"); the road name is kept for
the maintenance tool. No position is ever stored. File: MEMORY_PATH (JSON, written tmp + fsync + rename in the
background).

A pass is recorded only from openpilot-steered driving (latActive for PASS_STEERED of its samples, at least
PASS_MIN_TIME s, at least PASS_MIN_SPEED). Per pass: speed (median), peak lateral (from the wheel angle + the 0.15
m/s^2 margin it reads low), saturated time (|steer command| >= SAT_COMMAND), a steering alert, his steering or brake.
  * clean: no SAT_TIME of saturation, no alert, no driver steering/brake. A blinker before the apex drops a road pass
    (a lane change, a turn) but not a ramp pass.
  * cap-teaching: cruise on all along, no gas, at the planner's own curve speed (our plan governed at the entry), and
    saturated >= SAT_TIME, or an alert, or his correction. Cap = SAT_FACTOR x the pass speed, never below the
    fastest clean pass (and never below CAP_FLOOR). A cap-teaching pass also marks "no offset" on that curve.
  * a clean pass harder than MARK_LAT also marks "no offset" there.
  Passes with cruise off or the gas pressed never teach a cap. Caps and marks expire after EXPIRE_DAYS.
What they apply to (distance profiles): the curve's TIGHT points only (radius within TIGHT_SHARE of its tightest):
the cap limits their speed and the offset is applied to them; the wider points of the same curve are not touched
(the evidence was taken where the curve governs).
The offset (curve_offset_mph, 0-5) is never above sqrt(curve_offset_lat x r) and needs, on ramps/links,
RAMP_CLEAN_PASSES clean passes in the last EXPIRE_DAYS.
Pure apart from the file (tests/test_curve_memory.py).
"""
import json
import math
import os
import statistics
import threading
import time

MEMORY_PATH = "/data/media/0/mazda_speed/curve_memory.json"
VERSION = 1
MPH = 0.44704
EXPIRE_DAYS = 30.0
DAY = 86400.0
KEEP_PASSES = 6
PASS_MIN_TIME = 2.0
PASS_MIN_SPEED = 8.0
PASS_STEERED = 0.9
SAT_COMMAND = 0.98
SAT_TIME = 1.0
SAT_FACTOR = 0.9
CAP_FLOOR = 25 * MPH
MARK_LAT = 2.6
LAT_MARGIN = 0.15
TIGHT_SHARE = 1.3
RAMP_CLEAN_PASSES = 2
STEER_RATIO = 18.9
WHEELBASE = 2.7
SLIP = 0.001
SAVE_INTERVAL = 120.0


def wheel_lat(v, angle_deg):
  """m/s^2 from the wheel angle (carState.yawRate is 0 on this car), with the margin it reads low in curves."""
  return v * v * math.radians(abs(angle_deg)) / (STEER_RATIO * WHEELBASE * (1 + SLIP * v * v)) + LAT_MARGIN


def curve_key(way, direction):
  return f"{int(way)}:{direction}"


class PassRecorder:
  """Samples while the car is in one curve; finish() -> a pass dict or None (not a usable pass)."""

  def __init__(self, key, name, ramp, ours_at_entry):
    self.key, self.name, self.ramp, self.ours = key, name, ramp, ours_at_entry
    self.n = self.steered = 0
    self.v = []
    self.peak = 0.0
    self.sat = 0.0
    self.alert = self.driver = self.gas = self.cruise_off = self.blinker = False
    self.dt = 0.01

  def sample(self, s, before_apex):
    self.n += 1
    self.steered += 1 if s.get("lat_active") else 0
    self.v.append(s["v_ego"])
    self.peak = max(self.peak, wheel_lat(s["v_ego"], s.get("angle", 0.0)))
    if abs(s.get("torque", 0.0)) >= SAT_COMMAND:
      self.sat += self.dt
    self.alert |= bool(s.get("steer_alert"))
    self.driver |= bool(s.get("steering_pressed") or s.get("brake"))
    self.gas |= bool(s.get("gas"))
    self.cruise_off |= not s.get("cruise_enabled", False)
    if before_apex and s.get("blinker"):
      self.blinker = True

  def finish(self, now):
    t = self.n * self.dt
    if t < PASS_MIN_TIME or not self.v or statistics.median(self.v) < PASS_MIN_SPEED:
      return None
    if self.steered < PASS_STEERED * self.n:
      return None                                 # not openpilot-steered: nothing learned
    if self.blinker and not self.ramp:
      return None
    trouble = self.sat >= SAT_TIME or self.alert or self.driver
    return {"t": round(now, 1), "v": round(statistics.median(self.v), 2), "lat": round(self.peak, 2),
            "sat": round(self.sat, 2), "alert": self.alert, "driver": self.driver, "clean": not trouble,
            "teach": bool(trouble and self.ours and not self.gas and not self.cruise_off)}


class CurveMemory:
  def __init__(self, path=MEMORY_PATH, clock=time.time, background=True):  # noqa: TID251  (wall time: 30-day expiry)
    self.path, self.clock, self.background = path, clock, background
    self.curves = {}
    self.dirty = False
    self.saving = False
    self.last_save = None
    self.load()

  def load(self):
    try:
      with open(self.path) as f:
        d = json.load(f)
      if isinstance(d, dict) and d.get("version") == VERSION and isinstance(d.get("curves"), dict):
        self.curves = d["curves"]
    except (OSError, ValueError):
      self.curves = {}

  def _live(self, t):
    return t is not None and self.clock() - t <= EXPIRE_DAYS * DAY

  def add_pass(self, key, name, ramp, p):
    e = self.curves.setdefault(key, {"name": name, "ramp": bool(ramp), "passes": [], "cap": None, "no_offset": None})
    e["name"] = name or e.get("name", "")
    e["passes"] = (e["passes"] + [p])[-KEEP_PASSES:]
    if p["teach"]:
      cap = max(SAT_FACTOR * p["v"], self.fastest_clean(key) or 0.0, CAP_FLOOR)
      old = e.get("cap")
      if old is None or not self._live(old.get("t")) or cap < old["v"]:
        e["cap"] = {"v": round(cap, 2), "t": p["t"]}
      e["no_offset"] = p["t"]
    elif p["clean"] and p["lat"] >= MARK_LAT:
      e["no_offset"] = p["t"]
    if p["clean"] and e.get("cap") is not None and p["v"] > e["cap"]["v"]:
      e["cap"]["v"] = round(p["v"], 2)          # a cap is never below the fastest clean pass
    self.dirty = True

  def fastest_clean(self, key):
    e = self.curves.get(key)
    vs = [p["v"] for p in (e or {}).get("passes", []) if p.get("clean") and self._live(p.get("t"))]
    return max(vs) if vs else None

  def cap(self, key):
    e = self.curves.get(key)
    c = (e or {}).get("cap")
    return c["v"] if c and self._live(c.get("t")) else None

  def offset_allowed(self, key, ramp):
    e = self.curves.get(key)
    if e is not None and self._live(e.get("no_offset")):
      return False
    if ramp:
      clean = [p for p in (e or {}).get("passes", []) if p.get("clean") and self._live(p.get("t"))]
      return len(clean) >= RAMP_CLEAN_PASSES
    return True

  # ---- file ----
  def _write(self, data):
    try:
      os.makedirs(os.path.dirname(self.path), exist_ok=True)
      tmp = self.path + ".tmp"
      with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
        f.flush()
        os.fsync(f.fileno())
      os.replace(tmp, self.path)
    except OSError:
      pass

  def save_if_due(self, now, force=False):
    if not self.dirty or self.saving or (not force and self.last_save is not None and now - self.last_save < SAVE_INTERVAL):
      return False
    self.last_save, self.dirty = now, False
    data = json.loads(json.dumps({"version": VERSION, "curves": self.curves}))
    if not self.background:
      self._write(data)
      return True
    self.saving = True

    def run():
      try:
        from openpilot.starpilot.car.mazda.speed.events import leave_caller_priority
        leave_caller_priority()
        self._write(data)
      finally:
        self.saving = False
    threading.Thread(target=run, daemon=True).start()
    return True


def apply_memory(points, speeds, key, ramp, memory, offset, offset_lat, base_lat):
  """speeds of the profile points after the memory: the offset on the tight points (if allowed there, never above
  sqrt(offset_lat r)) and the cap on the tight points. points: [(d, r)], speeds: the plain point speeds."""
  if not points:
    return speeds
  r_min = min(r for _, r in points)
  cap = memory.cap(key) if memory is not None else None
  allow = offset > 0 and (memory.offset_allowed(key, ramp) if memory is not None else not ramp)
  out = []
  for (_, r), v in zip(points, speeds, strict=True):
    if r <= TIGHT_SHARE * r_min:
      if allow:
        v = max(v, min(v + offset, math.sqrt(offset_lat * r)))
      if cap is not None:
        v = min(v, cap)
    out.append(v)
  return out
