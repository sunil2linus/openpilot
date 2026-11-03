"""
Per-car profile for the Mazda set speed controller: what the first drives taught us about this car, kept across
drives in a small JSON file (PROFILE_PATH) so that every drive starts tuned instead of learning again.

  * Button answer time: the frames from our SET-/SET+ press to the set speed change (CurveSlowdownLogic.latencies,
    the newest LATENCY_KEEP). The timeout logic uses them from the first press of a drive.
  * Hold overrun: the most 5 mph quanta that ever landed after a SET-/SET+ hold was released, and how many holds
    (ours and the driver's own) measured it. From HOLD_MIN_MEASURED holds on, a drive starts "measured": the SET+
    restore holds keep worst + 1 quanta of margin instead of the cautious default. The margin never goes below
    what was seen: a worse landing raises it at once (CurveSlowdownLogic does that; this file remembers it).
  * Cruise decel: how fast the stock cruise slows when the set speed is a little below the true speed (the pace of
    the 1 mph staircase the map curve slowdown plans with). Measured by DecelLearner over clean episodes, median
    over DECEL_MIN_EPISODES or more, clamped to DECEL_RANGE, and written as "use" so the car side (curve_target /
    map_curve_control) and the planner (starpilot/controls/lib/map_curves.py, another process) read the same number.
    Until then both use their constant (DECEL_DEFAULT = MAP_DECEL = PLAN_DECEL).

Rules: the file is read once when the car controller starts (card, real time: a few KB, never raises), written in
the background (tmp + rename, every SAVE_INTERVAL when something changed, and when cruise goes off), and dropped when
the car fingerprint or the Torque Interceptor flag differs. MazdaResetCarProfile (a settings tile) deletes it.
No openpilot imports: unit tested offline.
"""
import json
import os
import threading
import time
from dataclasses import dataclass

PROFILE_PATH = "/data/media/0/mazda_car_profile.json"
PROFILE_VERSION = 1
MPH = 0.44704

LATENCY_KEEP = 9                 # as CurveSlowdownLogic.latencies
HOLD_MIN_MEASURED = 5            # holds measured (over all drives) before a drive starts as measured
DECEL_MIN_EPISODES = 10
DECEL_KEEP = 40                  # newest episodes kept (the median is over these)
DECEL_RANGE = (0.15, 0.40)       # m/s^2: the learned decel is used within this (map_curves.py clamps the same way)
DECEL_DEFAULT = 0.25             # m/s^2 = curve_target.MAP_DECEL = map_curves.PLAN_DECEL, used until measured
SAVE_INTERVAL = 120.0            # s between background writes while something changed

# DecelLearner: an episode is the set speed sitting GAP_MIN..GAP_MAX below the true speed (dash offset taken out)
# with the stock cruise alone in charge: cruise engaged, no pedal, no car closer than LEAD_CLEAR, not in a camera
# curve slowdown. The decel is the least squares slope of the speed from where the cruise started slowing.
DASH_OFFSET = 2.0 * MPH          # the dash (and so the set speed) reads about this much above the true speed (owner's car)
GAP_MIN = 2.0 * MPH              # m/s: set speed (true) at least this far below the true speed ...
GAP_MAX = 6.0 * MPH              # ... and not more: a big drop (a hold) slows harder, this learns the staircase pace
GAP_END = 1.0 * MPH              # the episode ends once the car is this close to the set speed
LEAD_CLEAR = 80.0                # m: a car closer than this may be what the cruise slows for
MIN_SPEED = 20.0 * MPH           # m/s true speed: below this the cruise behaves differently (and we never act there)
MIN_FIT_TIME = 2.0               # s of slowing in the fit
MIN_DROP = 0.4                   # m/s the speed must have fallen over the fit (about 1 mph)
REACTION_DROP = 0.15             # m/s below the speed at the start of the episode: the cruise has started slowing
SMOOTH_TIME = 0.3                # s: averaging window for that test (GPS/wheel speed noise)
SAMPLE_EVERY = 10                # frames: the speed is kept at 10 Hz (a 40 s episode is 400 points: the fit stays well under a millisecond in card)
MAX_EPISODE_TIME = 40.0          # s: longer than this is not one slowdown
DECEL_SANE = (0.02, 1.5)         # m/s^2: a fit outside this is noise (or the hill), not the cruise


def leave_caller_priority():
  """Background threads inherit card's real-time priority and core; give this one normal priority elsewhere
  (a copy of starpilot/common/feature_events.py's, so this module has no openpilot dependency)."""
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


def clamp_decel(value, default=DECEL_DEFAULT, lo=DECEL_RANGE[0], hi=DECEL_RANGE[1]):
  """The decel to plan with from a stored value: clamped to the safe range, the default when there is none."""
  try:
    v = float(value)
  except (TypeError, ValueError):
    return default
  if v != v:
    return default
  return min(max(v, lo), hi)


def decel_from_episodes(episodes, min_episodes=DECEL_MIN_EPISODES):
  """Median of the episodes, clamped, or None until there are min_episodes."""
  vals = sorted(float(v) for v in episodes if isinstance(v, (int, float)))
  if len(vals) < min_episodes:
    return None
  n = len(vals)
  med = vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])
  return round(clamp_decel(med), 3)


def read_profile_decel(path=PROFILE_PATH, default=DECEL_DEFAULT):
  """The decel another process should plan with: the profile's "use" value (clamped), else the default. Cheap (a
  few KB) and never raises; the planner calls it once per drive."""
  try:
    with open(path) as f:
      data = json.load(f)
    use = data.get("decel", {}).get("use")
  except (OSError, ValueError, AttributeError):
    return default
  return default if use is None else clamp_decel(use, default)


class CarProfile:
  def __init__(self, path=PROFILE_PATH, fingerprint="", ti=False, clock=time.monotonic):
    self.path = path
    self.fingerprint = str(fingerprint or "")
    self.ti = bool(ti)
    self.clock = clock
    self.data = self.empty()
    self.loaded = False            # a saved profile of this car was read
    self.discarded = None          # why a saved profile was not used
    self.dirty = False
    self.saving = False
    self.last_save = None
    self.writes = 0
    self.load()

  # ---- file ----
  def empty(self):
    now = round(time.time(), 1)   # wall time: the file is read by people, across reboots
    return {"version": PROFILE_VERSION, "fingerprint": self.fingerprint, "ti": self.ti, "created": now, "updated": now,
            "drives": 0, "latencies": [], "latency_samples": 0,
            "hold": {"worst_after_release": 0, "holds": 0, "driver_holds": 0, "last": None},
            "decel": {"episodes": [], "count": 0, "use": None}}

  def load(self):
    try:
      with open(self.path) as f:
        data = json.load(f)
    except (OSError, ValueError):
      return
    if not isinstance(data, dict):
      self.discarded = "not a profile"
    elif int(data.get("version", 0)) != PROFILE_VERSION:
      self.discarded = f"version {data.get('version')}"
    elif str(data.get("fingerprint", "")) != self.fingerprint:
      self.discarded = f"other car ({data.get('fingerprint')})"
    elif bool(data.get("ti", False)) != self.ti:
      self.discarded = "torque interceptor changed"
    if self.discarded is not None:
      self.dirty = True            # the fresh profile replaces the old file at the next save
      return
    fresh = self.empty()
    try:
      fresh["created"] = float(data.get("created", fresh["created"]))
      fresh["drives"] = int(data.get("drives", 0))
      fresh["latencies"] = [int(x) for x in data.get("latencies", []) if isinstance(x, (int, float)) and x > 0][-LATENCY_KEEP:]
      fresh["latency_samples"] = int(data.get("latency_samples", 0))
      h = data.get("hold") or {}
      fresh["hold"] = {"worst_after_release": max(0, int(h.get("worst_after_release", 0))), "holds": max(0, int(h.get("holds", 0))),
                       "driver_holds": max(0, int(h.get("driver_holds", 0))), "last": h.get("last")}
      d = data.get("decel") or {}
      episodes = [round(float(x), 3) for x in d.get("episodes", []) if isinstance(x, (int, float))][-DECEL_KEEP:]
      fresh["decel"] = {"episodes": episodes, "count": max(int(d.get("count", 0)), len(episodes)), "use": decel_from_episodes(episodes)}
    except (TypeError, ValueError, AttributeError):
      self.discarded = "unreadable values"
      self.dirty = True
      return
    self.data = fresh
    self.loaded = True

  def save_if_due(self, now=None, force=False):
    """Write in the background when something changed and SAVE_INTERVAL passed (or `force`)."""
    if not self.dirty or self.saving:
      return False
    now = self.clock() if now is None else now
    if not force and self.last_save is not None and now - self.last_save < SAVE_INTERVAL:
      return False
    self.last_save = now
    self.save()
    return True

  def _snapshot(self):
    data = json.loads(json.dumps(self.data))   # a copy the writer owns
    data["updated"] = round(time.time(), 1)
    return data

  def _write(self, data):
    try:
      d = os.path.dirname(self.path)
      if d:
        os.makedirs(d, exist_ok=True)
      tmp = self.path + ".tmp"
      with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
      os.replace(tmp, self.path)
      self.writes += 1
    except OSError:
      pass

  def save(self, in_thread=True):
    """Never raises; the caller's thread only takes the copy and starts the writer. Changes made while the writer
    is busy are written right after (as CurveMemory.save does), so a save asked for during a write is not lost."""
    self.dirty = False
    self.saving = True
    snapshot = self._snapshot()

    def write():
      if in_thread:
        leave_caller_priority()
      data = snapshot
      try:
        while True:
          self._write(data)
          if not self.dirty:
            break
          self.dirty = False
          try:
            data = self._snapshot()
          except Exception:
            self.dirty = True       # the caller was changing it right now: the next save_if_due gets it
            break
      finally:
        self.saving = False
    if in_thread:
      threading.Thread(target=write, daemon=True).start()
    else:
      write()

  def delete(self):
    """Forget everything (the settings tile): the file goes, the profile starts fresh."""
    try:
      os.remove(self.path)
    except OSError:
      pass
    self.data = self.empty()
    self.loaded = False
    self.dirty = False

  # ---- what a drive starts with ----
  @property
  def latencies(self):
    return list(self.data["latencies"])

  def hold_start(self, default_overrun):
    """(hold_overrun, hold_measured) for the start of a drive: measured from HOLD_MIN_MEASURED holds on, and never
    less margin than the worst landing seen."""
    h = self.data["hold"]
    return max(default_overrun, h["worst_after_release"]), h["holds"] >= HOLD_MIN_MEASURED

  @property
  def decel_use(self):
    """The learned cruise decel (m/s^2) to plan with, or None until DECEL_MIN_EPISODES episodes."""
    return self.data["decel"]["use"]

  def decel(self, default=DECEL_DEFAULT):
    use = self.decel_use
    return default if use is None else clamp_decel(use, default)

  @property
  def holds(self):
    return self.data["hold"]["holds"]

  @property
  def decel_episodes(self):
    return self.data["decel"]["count"]

  def summary(self):
    """One line for logs and the settings tile."""
    h, d = self.data["hold"], self.data["decel"]
    lat = sorted(self.data["latencies"])
    lat_s = f"{lat[len(lat) // 2] * 10} ms" if lat else "-"
    decel_s = f"{d['use']:.2f}" if d["use"] is not None else f"{d['count']}/{DECEL_MIN_EPISODES}"
    return f"{self.data['drives']} drives, answer {lat_s}, {h['holds']} holds (worst after release {h['worst_after_release']}), decel {decel_s}"

  # ---- learning ----
  def note_drive(self):
    self.data["drives"] += 1
    self.dirty = True

  def set_latencies(self, latencies):
    new = [int(x) for x in latencies][-LATENCY_KEEP:]
    if new != self.data["latencies"]:
      self.data["latency_samples"] += max(1, len(new) - len(self.data["latencies"]))
      self.data["latencies"] = new
      self.dirty = True

  def add_hold(self, quanta_after, driver=False):
    h = self.data["hold"]
    h["worst_after_release"] = max(h["worst_after_release"], int(quanta_after))
    h["holds"] += 1
    if driver:
      h["driver_holds"] += 1
    h["last"] = {"after": int(quanta_after), "driver": bool(driver), "t": round(time.time(), 1)}
    self.dirty = True

  def add_decel_episode(self, decel):
    d = self.data["decel"]
    d["episodes"] = (d["episodes"] + [round(float(decel), 3)])[-DECEL_KEEP:]
    d["count"] += 1
    d["use"] = decel_from_episodes(d["episodes"])
    self.dirty = True
    return d["use"]


@dataclass
class DecelInputs:
  v_ego: float              # m/s, true speed
  set_speed: float          # m/s, the stock cruise set speed as the dash shows it (0 = none)
  cruise_enabled: bool
  brake_pressed: bool
  gas_pressed: bool
  lead_status: bool         # radarState.leadOne.status
  lead_d: float             # m
  csc_active: bool          # the camera curve slowdown wants a lower speed (in a curve)


class DecelLearner:
  """Measures how fast the stock cruise slows for a set speed a little below the true speed (see the module doc).
  update() returns the decel (m/s^2) of an episode that just ended and qualified, else None."""

  def __init__(self, dt=0.01, dash_offset=DASH_OFFSET, sample_every=SAMPLE_EVERY):
    self.dt = dt
    self.dash_offset = dash_offset
    self.sample_every = max(1, int(sample_every))
    self.frame = 0
    self.active = False
    self.samples = []          # (t, v) while the episode runs, every sample_every frames
    self.t = 0.0
    self.episodes = 0          # qualifying episodes seen
    self.rejected = 0          # episodes that ended without a usable fit
    self.last = None           # decel of the last qualifying episode
    self.prev_gap = None

  @staticmethod
  def gates(i):
    return (i.cruise_enabled and i.set_speed > 0 and not i.brake_pressed and not i.gas_pressed and not i.csc_active and
            not (i.lead_status and 0 < i.lead_d < LEAD_CLEAR) and i.v_ego >= MIN_SPEED)

  def update(self, i: DecelInputs):
    self.t += self.dt
    self.frame += 1
    gap = i.v_ego - (i.set_speed - self.dash_offset)
    ok = self.gates(i)
    prev_gap, self.prev_gap = self.prev_gap, (gap if ok else None)
    if not self.active:
      # the gap must open from below (a staircase of small steps), not close in from a big drop (a hold), whose tail
      # is the cruise easing off a harder slowdown
      if ok and GAP_MIN <= gap <= GAP_MAX and prev_gap is not None and prev_gap < GAP_MIN:
        self.active, self.samples = True, [(self.t, i.v_ego)]
      return None
    if ok and GAP_END <= gap <= GAP_MAX and self.t - self.samples[0][0] <= MAX_EPISODE_TIME:
      if self.frame % self.sample_every == 0:
        self.samples.append((self.t, i.v_ego))
      return None
    # over: the car reached the set speed, or something else took over (pedal, lead, curve, a big drop); what was
    # measured while the cruise alone was slowing still counts
    return self._finish()

  def _finish(self):
    self.active = False
    samples, self.samples = self.samples, []
    # the fit starts where the cruise has clearly begun to slow: the first point whose SMOOTH_TIME average is
    # REACTION_DROP below the speed at the start (single noisy samples must not add a flat lead-in to the fit)
    n_avg = max(1, int(round(SMOOTH_TIME / (self.dt * self.sample_every))))
    if len(samples) < 2 * n_avg:
      self.rejected += 1
      return None
    v0 = sum(v for _, v in samples[:n_avg]) / n_avg
    start = None
    run = sum(v for _, v in samples[:n_avg])
    for k in range(n_avg, len(samples)):
      run += samples[k][1] - samples[k - n_avg][1]
      if run / n_avg <= v0 - REACTION_DROP:
        start = k - n_avg // 2
        break
    pts = samples[start:] if start is not None else []
    if len(pts) < 2 or pts[-1][0] - pts[0][0] < MIN_FIT_TIME or v0 - pts[-1][1] < MIN_DROP:
      self.rejected += 1
      return None
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    mv = sum(v for _, v in pts) / n
    sxx = sum((t - mt) ** 2 for t, _ in pts)
    if sxx <= 0:
      self.rejected += 1
      return None
    slope = sum((t - mt) * (v - mv) for t, v in pts) / sxx
    decel = -slope
    if not (DECEL_SANE[0] <= decel <= DECEL_SANE[1]):
      self.rejected += 1
      return None
    self.episodes += 1
    self.last = round(decel, 3)
    return self.last
