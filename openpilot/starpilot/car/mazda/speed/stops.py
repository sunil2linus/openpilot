"""
The stop slowdown contributor (part G): takes speed off EARLY for a stopped (or much slower) car ahead and for a stop
the camera's driving model plans (a red light, a stop sign, a queue), and says so (notices.py: "Stopped car ahead",
"Stop ahead"). It only proposes a target / a ceiling to the planner (planner.py); the actuator is the only presser.

Honest limit: the stock cruise brakes at most about 1 m/s^2 (cruise_model) and never below its 30 km/h (19 mph)
minimum set speed. It cannot stop the car, and for a stop seen late it cannot even slow enough: this takes speed off
early and warns; the DRIVER brakes.

Triggers (each from 20 mph up; nothing starts in test mode either, it only logs and warns):
  * G1 "stopped car": radard's lead (on this car from the camera; the stock radar only gives a "has lead" bit,
    ctx["radar_lead"], which lowers the lead's needed probability but is never required: it is weak on stationary
    cars) that is stopped (LEAD_STOPPED) or much slower than us (LEAD_SLOW_SHARE of our speed), closing by at least
    LEAD_CLOSING, within LEAD_MAX_D, for LEAD_CONFIRM without a break (a distance jump of LEAD_JUMP is another car and
    starts over; the lead's speed is the HIGHEST of the window: a short window never looks slow).
    Target: reach the lead's speed (the cruise's floor for a stopped car) MARGIN behind it with the planning decel
    after the cruise's lag (cruise_model.envelope); a stopped car within FLOOR_DISTANCE: the floor at once (one hold).
  * G2 "stop ahead": the old fork's stop-light flag, computed here from modelV2 (the old starpilot planner's CEM
    stop_sign_and_light): the model's planned path (position.x[-1]) is shorter than v x the planner time (8 s, x1.2
    from 45 mph, capped at 9 s), on with ON_MARGIN, off with OFF_MARGIN; not above 75 mph, not in a turn at a
    crawl, not with a lead inside the path (that short path is the lead's: G1's job). Confirmed after SHORT_CONFIRM
    without a break. Target: the floor at the path's end (STOP_MARGIN before it), the floor at once within
    FLOOR_DISTANCE.
    Far guess guard: above GUARD_SPEED with the path's end GUARD_FAR or more ahead, the flag (confirmed or not) only
    gives 1 mph guard taps (from GUARD_START, one per GUARD_EVERY, at most GUARD_MAX under the set where it began);
    the full target only once the stop is nearer than GUARD_FAR. A short flag (< GUARD_START) does nothing at all.
  * G3 "stop sign": the car camera's dashboard stop sign (carstate dashboard_stop_sign from CAM_TRAFFIC_SIGNS, read
    on this car with or without the TI) seen within SIGN_RECENT shortens G2's confirmation to SIGN_CONFIRM; it is
    never a trigger without a short model path. (Neither our curve files nor 7.0's mapdOut have a stop sign node.)
  * The target only goes down during an episode (a hold is never cut by it).
Releasing: G1 when the lead moves away (at least RELEASE_MOVING and not closing for RELEASE_TIME) or has been gone
LOST_RELEASE with the model path long; G2 when the path is long again for LONG_RELEASE. The planner then restores
toward the anchor by its normal raise. His buttons win as everywhere: his SET+ during the slowdown blocks it for
the episode (planner) and the episode is declined until its cause is gone; his gas held GAS_HELD declines it too;
his SET (a new engage) or the main switch off forgets it.
His RES with the set left low by us: the episode ends and the speed comes back at once (the planner's raise: a
hold, then taps), but step by step (a ceiling one step above the set: taps, no hold) while the model path is shorter
than v x RESTORE_PATH_T or a lead within RESTORE_LEAD_D is closing; no new episode for RES_QUIET.
Pure (tests/test_stops.py).
"""
import math
from collections import deque

from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.actuator import CAR_MIN_SET
from openpilot.starpilot.car.mazda.speed.planner import Contribution

MPH, KPH = M.MPH, M.KPH
FLOOR_TRUE = CAR_MIN_SET - M.DASH_OFFSET      # m/s true speed the cruise holds at its minimum set speed
START_SPEED = 20 * MPH        # nothing starts below this (the set is near its floor there)

# G1
LEAD_STOPPED = 2.0            # m/s: a lead this slow is stopped
LEAD_SLOW_SHARE = 0.5         # ... or slower than this share of our speed
LEAD_CLOSING = 3.0            # m/s
LEAD_MAX_D = 160.0            # m
LEAD_MIN_PROB = 0.6           # radard's lead (camera) probability ...
LEAD_MIN_PROB_RADAR = 0.4     # ... with the stock radar's "has lead" bit on
LEAD_CONFIRM = 0.5            # s
LEAD_JUMP = 20.0              # m: another car
LEAD_STALE = 0.5              # s
MARGIN = 15.0                 # m behind the lead the speed is reached
FLOOR_DISTANCE = 130.0        # m: a confirmed stop this close: straight to the floor
RELEASE_MOVING = 1.5          # m/s (walking pace)
RELEASE_TIME = 0.5            # s
LOST_RELEASE = 2.0            # s

# G2 / G3
MODEL_TIME = 8.0              # s (old: PLANNER_TIME - 2)
BOOST = ((0.0, 1.0), (35.0, 1.0), (45.0, 1.2))      # mph -> factor (old LIGHT_BOOSTS, interp 0/35/45)
CAP_TIME = 9.0                # s (old LIGHT_MAX_TIME), fully from 45 mph
ON_MARGIN = 2.5               # m
OFF_MARGIN = 4.0              # m
MAX_SPEED = 75 * MPH
LEAD_BLOCK = 15.0             # m
TURN_ANGLE = 45.0             # deg ...
TURN_SPEED = 15 * MPH         # ... below this: a turn, not a stop
MODEL_STALE = 0.3             # s
SHORT_CONFIRM = 1.0           # s
SIGN_CONFIRM = 0.5            # s with the dashboard stop sign seen
SIGN_RECENT = 2.0             # s
STOP_MARGIN = 5.0             # m before the path's end
LONG_RELEASE = 1.0            # s
GUARD_SPEED = 45 * MPH
GUARD_FAR = 140.0             # m
GUARD_START = 0.5             # s of the flag before the first guard tap
GUARD_EVERY = 0.5             # s per further guard step
GUARD_MAX = 3                 # dash steps

# his inputs / the restore after his RES
GAS_HELD = 1.0                # s
RES_QUIET = 3.0               # s
RESTORE_PATH_T = 4.0          # s
RESTORE_LEAD_D = 80.0         # m
RESTORE_CLOSING = 1.0         # m/s
RESTORE_MAX = 30.0            # s
RES_ENGAGE = 2.0              # s after his RES the cruise may still come on (RES after his brake re-engages it)


def _interp(x, pts):
  if x <= pts[0][0]:
    return pts[0][1]
  for (x0, y0), (x1, y1) in zip(pts, pts[1:], strict=False):
    if x <= x1:
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
  return pts[-1][1]


def stop_threshold(v):
  """m: the old stop-light threshold, v x the planner time (boosted and capped from 45 mph)."""
  mph = v / MPH
  t = MODEL_TIME * _interp(mph, BOOST)
  cap = _interp(mph, ((0.0, 0.0), (35.0, 0.0), (45.0, 1.0)))
  if cap > 0:
    t = min(t, CAP_TIME * cap + MODEL_TIME * (1 - cap))
  return max(v * t, 0.0)


class StopSlowdown:
  def __init__(self, cfg, presses=True, metric=False):
    self.cfg = cfg
    self.presses = presses
    self.metric = bool(metric)
    self.step = KPH if self.metric else MPH
    self.t = None
    self.n = 0                     # episodes this drive
    # G1
    self.lead_t = 0.0              # s the lead has been a candidate
    self.lead_prev = None          # (t, d) of the last fresh lead frame
    self.lead_v = deque()          # (t, vLead) of the candidate window
    self.lead_gone = 0.0
    self.away_t = 0.0
    # G2
    self.short = False             # the model path is short (with the hysteresis)
    self.short_t = 0.0
    self.long_t = 0.0
    self.path_len = None
    self.sign_t = -1e9             # last frame the dashboard showed a stop sign
    self.sign_stamp = None
    # the episode
    self.reset()
    self.res_t = -1e9
    self.restore = None            # (until) after his RES: the raise, step by step while not clear
    self.gas_t = 0.0
    self.last = None               # (state, target) for the log
    self.notice = None             # "stopped_car" / "stop_ahead" once per episode (notices.py reads it)

  def reset(self):
    self.active = None             # None, or the episode: {"id", "lead", "stop", "guard_set", "target", ...}
    self.declined = False          # his SET+ / gas: no new episode until the cause is gone

  # ---------------- inputs ----------------
  def _lead(self, ctx, now):
    lead = ctx.get("lead")
    if lead is None or now - lead.get("t", -1e9) > LEAD_STALE or not lead.get("status"):
      return None
    return lead

  def _lead_candidate(self, ctx, lead, now, dt, v):
    """Advance the G1 window. True once the lead has been a stopped / slow closing car for LEAD_CONFIRM."""
    if lead is None:
      self.lead_t, self.lead_prev = 0.0, None
      self.lead_v.clear()
      return False
    d, vrel = float(lead["d"]), float(lead.get("vrel", 0.0))
    prev = self.lead_prev
    jump = prev is not None and abs(d - (prev[1] + vrel * max(now - prev[0], 0.0))) > LEAD_JUMP
    self.lead_prev = (now, d)
    if jump:
      self.lead_t = 0.0
      self.lead_v.clear()
    min_prob = LEAD_MIN_PROB_RADAR if ctx.get("radar_lead") else LEAD_MIN_PROB
    prob = float(lead.get("prob", 1.0))
    self.lead_v.append((now, float(lead["v"])))
    while self.lead_v and now - self.lead_v[0][0] > LEAD_CONFIRM + 1e-6:
      self.lead_v.popleft()
    v_lead = max(x for _, x in self.lead_v)
    slow = v_lead <= LEAD_STOPPED or v_lead <= LEAD_SLOW_SHARE * v
    ok = bool(prob >= min_prob and d <= LEAD_MAX_D and slow and -vrel >= LEAD_CLOSING)
    self.lead_t = self.lead_t + dt if ok else 0.0
    return self.lead_t >= LEAD_CONFIRM - 1e-6

  def _short_path(self, ctx, lead, v):
    """The model path's length and the short-path flag (hysteresis), None when the model is not usable."""
    path = ctx.get("path")
    if path is None or ctx.get("model_age", 1e9) > MODEL_STALE or not path[0]:
      self.short = False
      return None
    length = float(path[0][-1])
    if not math.isfinite(length):
      self.short = False
      return None
    thr = stop_threshold(v)
    if v > MAX_SPEED or (abs(ctx.get("angle", 0.0)) >= TURN_ANGLE and v < TURN_SPEED):
      self.short = False
      return length
    if lead is not None and float(lead["d"]) < length + LEAD_BLOCK:
      self.short = False           # the short path is the lead's (G1's job)
      return length
    self.short = length < (thr + OFF_MARGIN if self.short else max(thr - ON_MARGIN, 0.0))
    return length

  def _sign(self, ctx, now):
    raw = ctx.get("stop_sign_raw")
    if type(raw) is tuple and len(raw) == 2:
      seen, stamp = raw
      if stamp != self.sign_stamp:
        self.sign_stamp = stamp
        if seen is True:
          self.sign_t = now
    return now - self.sign_t <= SIGN_RECENT

  # ---------------- the episode ----------------
  def _start(self, now, set_speed):
    self.n += 1
    self.active = {"id": f"stop{self.n}", "lead": False, "stop": False, "guard_set": None, "guard_t": 0.0,
                   "target": None, "t": now, "warned": set()}
    self.restore = None
    return self.active

  def _end(self, why):
    self.last = ("released", why)
    self.active = None

  def _his(self, ctx):
    """His inputs: (res, set_plus, engaged)."""
    res = plus = engaged = False
    for e in ctx.get("act_events", ()):
      k = e.get("kind")
      if k == "driver_button" and e.get("button") == "RES":
        res = True
      elif k == "driver" and e.get("direction") == "SET_PLUS":
        plus = True
      elif k == "engaged":
        engaged = True
    return res, plus, engaged

  def __call__(self, ctx):
    now, v = ctx.get("now", 0.0), max(ctx.get("v_ego", 0.0), 0.0)
    dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
    self.t = now
    self.notice = None
    out = Contribution("stop", None, presses=self.presses)
    if not ctx.get("main_on", True):
      self.reset()
      self.restore = None
      self.last = ("main off", None)
      return out
    lead = self._lead(ctx, now)
    lead_ok = self._lead_candidate(ctx, lead, now, dt, v)
    length = self._short_path(ctx, lead, v)
    sign = self._sign(ctx, now)
    self.path_len = length
    if self.short:
      self.short_t += dt
      self.long_t = 0.0
    else:
      self.short_t = 0.0
      self.long_t = self.long_t + dt if length is not None else 0.0
    self.gas_t = self.gas_t + dt if ctx.get("gas") else 0.0
    res, plus, engaged = self._his(ctx)
    set_speed = ctx.get("set_speed", 0.0)
    anchor = ctx.get("anchor")
    ep = self.active

    # ---- his inputs ----
    if engaged:
      self.reset()
      self.restore = None
    if res:
      self.res_t = now
      low = anchor is not None and set_speed < anchor - 0.5 * self.step
      if self.active is not None or low:
        self.active = None
        self.restore = now + RESTORE_MAX if low else None
    if self.active is not None and (plus or self.gas_t >= GAS_HELD - 1e-6):
      self.active, self.declined = None, True
    ep = self.active

    # ---- causes ----
    need = SIGN_CONFIRM if sign else SHORT_CONFIRM
    stop_on = self.short and self.short_t > 0.0
    stop_confirmed = self.short and self.short_t >= need - 1e-6
    lead_away = bool(lead is not None and float(lead["v"]) >= RELEASE_MOVING and float(lead.get("vrel", 0.0)) >= 0.0)
    self.away_t = self.away_t + dt if lead_away else 0.0
    self.lead_gone = self.lead_gone + dt if lead is None else 0.0
    cause = lead_ok or stop_on
    if self.declined and not cause and self.lead_t == 0.0 and not self.short:
      self.declined = False
    quiet = now - self.res_t < RES_QUIET

    if ep is None and not self.declined and not quiet and ctx.get("cruise_enabled", False) and v >= START_SPEED:
      far = length is not None and v > GUARD_SPEED and length - STOP_MARGIN >= GUARD_FAR
      guard_due = stop_on and far and self.short_t >= GUARD_START - 1e-6
      if lead_ok or (stop_confirmed and not far) or guard_due:
        ep = self._start(now, set_speed)

    if ep is not None:
      # ---- release ----
      if ep["lead"] and not lead_ok and (self.away_t >= RELEASE_TIME - 1e-6 or
                                        (self.lead_gone >= LOST_RELEASE - 1e-6 and not self.short)):
        ep["lead"] = False
      if (ep["stop"] or ep["guard_set"] is not None) and not self.short and self.long_t >= LONG_RELEASE - 1e-6:
        ep["stop"], ep["guard_set"] = False, None
      if lead_ok:
        ep["lead"] = True
      far = length is not None and v > GUARD_SPEED and length - STOP_MARGIN >= GUARD_FAR
      if stop_confirmed and not far:
        ep["stop"], ep["guard_set"] = True, None
      elif stop_on and far and not ep["stop"] and self.short_t >= GUARD_START - 1e-6 and ep["guard_set"] is None:
        ep["guard_set"] = set_speed
      if not ep["lead"] and not ep["stop"] and ep["guard_set"] is None:
        self._end("cause gone")
        ep = None

    if ep is None:
      return self._restore(ctx, out, now, v, lead, length, set_speed, anchor)

    # ---- the target ----
    target = None
    if ep["lead"] and lead is not None:
      v_lead = max(x for _, x in self.lead_v) if self.lead_v else float(lead["v"])
      d = float(lead["d"])
      if v_lead <= LEAD_STOPPED and d <= FLOOR_DISTANCE:
        tl = FLOOR_TRUE
      else:
        v_point = max(v_lead if v_lead > LEAD_STOPPED else 0.0, FLOOR_TRUE)
        tl = M.envelope(v_point, max(d - MARGIN, 0.0), v)
      target = tl
      if "stopped_car" not in ep["warned"]:
        ep["warned"].add("stopped_car")
        self.notice = "stopped_car"
    if ep["stop"] and length is not None:
      dist = max(length - STOP_MARGIN, 0.0)
      ts = FLOOR_TRUE if dist <= FLOOR_DISTANCE else M.envelope(FLOOR_TRUE, dist, v)
      target = ts if target is None else min(target, ts)
      if "stop_ahead" not in ep["warned"] and self.notice is None:
        ep["warned"].add("stop_ahead")
        self.notice = "stop_ahead"
    if target is None and (ep["lead"] or ep["stop"]):
      target = ep["target"]                    # the lead / the model frame dropped out: keep the episode's target
    if target is not None:
      target = max(target, FLOOR_TRUE)
      if ep["target"] is not None:
        target = min(target, ep["target"])     # only down during an episode
      ep["target"] = target
    out.target = target
    out.episode = ep["id"]
    why = "lead" if ep["lead"] else ("stop" if ep["stop"] else "guard")
    if ep["guard_set"] is not None and not ep["stop"]:
      steps = (min(GUARD_MAX, 1 + int((self.short_t - GUARD_START) / GUARD_EVERY + 1e-6)) if self.short else
               ep["guard_t"])
      ep["guard_t"] = max(ep["guard_t"], steps)
      out.ceiling = ep["guard_set"] - ep["guard_t"] * self.step
    self.last = (why, target if out.ceiling is None else ("ceiling", round(out.ceiling / self.step, 1)))
    ctx["stop_busy"] = True
    return out

  def _restore(self, ctx, out, now, v, lead, length, set_speed, anchor):
    """After his RES with the set left low: the raise comes from the planner (anchor); while the road ahead is not
    clear only step by step (a ceiling one step above the set: taps, never a hold)."""
    if self.restore is None:
      if self.last is None or self.last[0] != "released":
        self.last = ("idle", None)
      return out
    cruise_lost = not ctx.get("cruise_enabled", False) and now - self.res_t > RES_ENGAGE
    if now > self.restore or anchor is None or set_speed >= anchor - 0.5 * self.step or cruise_lost:
      self.restore = None
      self.last = ("idle", None)
      return out
    short = length is not None and length < v * RESTORE_PATH_T
    closing = lead is not None and float(lead["d"]) <= RESTORE_LEAD_D and float(lead.get("vrel", 0.0)) <= -RESTORE_CLOSING
    if short or closing:
      out.ceiling = set_speed + self.step
      out.episode = f"restore{self.n}"
      self.last = ("restore taps", "short path" if short else "closing lead")
    else:
      self.last = ("restore", None)
    return out
