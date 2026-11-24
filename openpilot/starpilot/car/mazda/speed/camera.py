"""
The camera (modelV2) in the speed control, two narrow jobs only:

  * CameraVeto: a map curve whose tight points are within the model's reliable view (about 2 s, VETO_SEE_TIME) while
    the model's path is straight there (its curvature VETO_FACTOR or more gentler) for VETO_CONFIRM s is a phantom
    (a wrong road match, a map error): not slowed for. It governs again when the camera sees it within REARM_FACTOR
    for REARM_CONFIRM s; after VETO_MAX_FLIPS take-backs a curve is never vetoed again this drive. No veto while a
    blinker is on (lane change / turn: the path is not the road).
  * CameraCurve: a careful target for curves the map does not have: the model's curvature between CAM_T0 and CAM_T1 s
    ahead (it under-reads beyond ~2 s) planned at CAM_LAT (min with the road lateral; CAM_NO_GPS_LAT without a GPS
    position) with the same braking envelope; the least demanding value of the last CAM_CONFIRM s (one curved model
    frame does not slow the car). Not at all below CAM_MIN_SPEED, to start a slowdown when the target is more than
    CAM_ABOVE_V above the car's speed, with a blinker on, or where a map curve covers the camera's range (the map
    governs there).
No in-curve taps, no post-apex logic, no RampTail: the map profile does that job now. Pure (tests/test_camera.py).
"""
import math
from collections import deque

from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.planner import Contribution

MPH = M.MPH
MODEL_STALE = 0.3             # s
VETO_SEE_TIME = 2.0           # s of the model's path that is reliable
VETO_MIN_D = 10.0             # m
VETO_SEE_MAX = 110.0          # m
VETO_PAD = 10.0               # m the path must reach past the point
VETO_FACTOR = 2.0
VETO_CONFIRM = 1.0            # s
REARM_FACTOR = 1.3
REARM_CONFIRM = 0.5           # s
VETO_MAX_FLIPS = 2
VETO_MIN_SPEED = 8.0          # m/s
TIGHT_SHARE = 1.3             # a profile point within this factor of the curve's tightest is one of its tight points

CAM_T0, CAM_T1 = 0.5, 2.0     # s ahead
CAM_LAT = 2.0                 # m/s^2 (min with the road's plan_lat)
CAM_NO_GPS_LAT = 1.8          # m/s^2 without a GPS position: more careful
CAM_CONFIRM = 0.5             # s
CAM_MIN_SPEED = 25 * MPH
CAM_MIN_K = 1.0 / 1000.0
# Round W, W1 (Chicago drive 18:08-18:10, 18:31, 18:55): crawling in traffic at 25-35 mph with anchor 80-85 mph,
# gentle bends / path wiggles at CAM_LAT (2.0 m/s^2) give 65-80 mph camera targets, repeatedly tapping SET- far
# above v_ego. Camera lookahead is only CAM_T1 (2 s) ahead, so a camera target more than CAM_ABOVE_V above the
# car's actual speed cannot start a slowdown now (once active, it holds until the curve ends so drifting around
# v_ego + 10 mph does not flip-flop).
CAM_ABOVE_V = 10.0 * MPH


def model_path(model, warn=None):
  """(xs m ahead, kappas 1/m) from a modelV2 reader, or None when it is broken (a field missing from the schema is
  reported once: events.schema_miss)."""
  try:
    xs = list(model.position.x)
    rz = list(model.orientationRate.z)
    vx = list(model.velocity.x)
  except AttributeError as e:
    from openpilot.starpilot.car.mazda.speed.events import schema_miss
    schema_miss(warn, "modelV2.position/orientationRate/velocity", e)
    return None
  n = min(len(xs), len(rz), len(vx))
  if n < 2:
    return None
  ks = [rz[k] / max(vx[k], 1.0) for k in range(n)]
  if not all(math.isfinite(v) for v in xs[:n] + ks):
    return None
  return xs[:n], ks


def path_curvature(path, d0, d1):
  """Peak |kappa| of the model's path between d0 and d1 m ahead, or None when the path does not reach d1."""
  xs, ks = path
  if xs[-1] < d1:
    return None
  vals = [abs(k) for x, k in zip(xs, ks, strict=True) if d0 <= x <= d1]
  return max(vals) if vals else None


class CameraVeto:
  def __init__(self, dt=0.05):
    self.dt = dt
    self.state = {}            # curve id -> {"veto": bool, "t": confirm s, "flips": n}
    self.frame_t = None

  def update(self, ctx):
    """Once per new model frame (ctx["model_new"]): advance every map curve's check."""
    path, v = ctx.get("path"), ctx.get("v_ego", 0.0)
    view = ctx.get("view") or {}
    if not ctx.get("model_new"):
      return
    for c in view.get("curves", []):
      st = self.state.setdefault(c["id"], {"veto": False, "t": 0.0, "flips": 0})
      verdict = self._look(c, path, v, ctx)
      if verdict is None:
        st["t"] = 0.0
        continue
      if not st["veto"]:
        if verdict == "straight" and st["flips"] < VETO_MAX_FLIPS:
          st["t"] += self.dt
          if st["t"] >= VETO_CONFIRM - 1e-6:
            st["veto"], st["t"] = True, 0.0
        else:
          st["t"] = 0.0
      else:
        if verdict == "curve":
          st["t"] += self.dt
          if st["t"] >= REARM_CONFIRM - 1e-6:
            st["veto"], st["t"] = False, 0.0
            st["flips"] += 1
        else:
          st["t"] = 0.0

  def _look(self, c, path, v, ctx):
    if path is None or v < VETO_MIN_SPEED or ctx.get("blinker"):
      return None
    driven = max(v, 0.0) * max(ctx.get("now", 0.0) - (ctx.get("view") or {}).get("t", 0.0), 0.0)
    pts = [(d - driven, r) for d, r in c["pts"]]
    if not pts:
      return None
    r_min = min(r for _, r in pts)
    reach = min(v * VETO_SEE_TIME, VETO_SEE_MAX)
    tight = [(d, r) for d, r in pts if r <= TIGHT_SHARE * r_min and VETO_MIN_D <= d <= reach]
    if not tight:
      return None
    worst = None
    for d, r in tight:
      k = path_curvature(path, d - VETO_PAD, d + VETO_PAD)
      if k is None:
        return None
      ratio = k * r          # 1.0: the camera sees the map's curvature
      worst = ratio if worst is None else max(worst, ratio)
    if worst * VETO_FACTOR <= 1.0:
      return "straight"
    if worst * REARM_FACTOR >= 1.0:
      return "curve"
    return "unsure"

  def vetoed(self, c, ctx=None):
    st = self.state.get(c["id"])
    return bool(st and st["veto"])


class CameraCurve:
  def __init__(self, cfg, presses=True, dt=0.01):
    self.cfg = cfg
    self.presses = presses
    self.window = deque(maxlen=max(1, int(round(CAM_CONFIRM / dt))))
    self.last = None
    self.raw_vmin = None
    self.last_vmin = None

  def raw_target(self, ctx):
    path, v = ctx.get("path"), ctx.get("v_ego", 0.0)
    if path is None or v < CAM_MIN_SPEED or ctx.get("blinker") or ctx.get("model_age", 1e9) > MODEL_STALE:
      return None
    lat = min(CAM_LAT, self.cfg.plan_lat)
    if not ctx.get("gps_ok", False):
      lat = min(lat, CAM_NO_GPS_LAT)
    xs, ks = path
    d0, d1 = v * CAM_T0, v * CAM_T1
    if xs[-1] < d1:
      return None
    best = None
    for x, k in zip(xs, ks, strict=True):
      if d0 <= x <= d1 and abs(k) >= CAM_MIN_K:
        vp = math.sqrt(lat / abs(k))
        e = M.envelope(vp, x, v)
        best = e if best is None else min(best, e)
        self.raw_vmin = vp if self.raw_vmin is None else min(self.raw_vmin, vp)
    return best

  def __call__(self, ctx):
    self.raw_vmin = None
    raw = self.raw_target(ctx)
    if raw is not None and ctx.get("map_covers_camera"):
      raw = None
    if self.last is None and raw is not None and raw > ctx.get("v_ego", 0.0) + CAM_ABOVE_V:
      self.raw_vmin = None
      raw = None
    self.window.append(math.inf if raw is None else raw)
    target = max(self.window) if len(self.window) == self.window.maxlen else math.inf
    target = None if not math.isfinite(target) else target
    self.last = target
    if self.raw_vmin is not None:
      self.last_vmin = self.raw_vmin  # true m/s of the tightest point seen (the notice's "slowing to")
    return Contribution("camera_curve", target, presses=self.presses)
