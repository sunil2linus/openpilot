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
    starts over; the lead's speed is the HIGHEST of the window: a short window never looks slow). (Round 2) Within
    LEAD_NEAR_D and closing by LEAD_NEAR_CLOSING it needs only LEAD_MIN_PROB_NEAR (a stopping car's confidence falls to
    0.53-0.66 as we close); a lead missing up to LEAD_DROPOUT holds the window; a "stopping car" counts too: braking
    (the window's mean aLeadK) at LEAD_BRAKING or harder, LEAD_BRAKING_SLOWER slower than us, with the model planning
    a stop (its end speed at most STOP_END_V): the target is the floor where it will stand.
    Target: reach the lead's speed (the cruise's floor for a stopped car) MARGIN behind it with the planning decel
    after the cruise's lag (cruise_model.envelope); a stopped car within FLOOR_DISTANCE: the floor at once (one hold).
  * G2 "stop ahead": the old fork's stop-light flag, computed here from modelV2 (the old starpilot planner's CEM
    stop_sign_and_light): the model's planned path (position.x[-1]) is shorter than v x the planner time (8 s, x1.2
    from 45 mph, capped at 9 s), on with ON_MARGIN, off with OFF_MARGIN; not above 75 mph, not in a turn at a
    crawl, not with a lead inside the path (that short path is the lead's: G1's job) unless the model plans a stop
    (its end speed velocity.x[-1] at most STOP_END_V: the lead at our speed stops too; r2 rt 416 / 901 had no warning
    before his brake). Above GUARD_SPEED it also needs the model's end speed at most END_SHARE of ours (the path alone
    is ~10 s long and dips under 9 s on 3-6 % of the frames at a steady 50 mph: 22 false 1-3 mph guards on r2 / r3).
    Confirmed after SHORT_CONFIRM without a break. Target: the floor at the path's end (STOP_MARGIN before it), the floor at once within
    FLOOR_DISTANCE.
    Far guess guard: above GUARD_SPEED with the path's end GUARD_FAR or more ahead, the flag (confirmed or not) only
    gives 1 mph guard taps (from GUARD_START, one per GUARD_EVERY, at most GUARD_MAX under the set where it began);
    the full target only once the stop is nearer than GUARD_FAR. A short flag (< GUARD_START) does nothing at all.
    No guard taps while the exit plan is busy or a map / camera curve or the exit plan wants less than v + CURVE_NEAR
    (the short path is theirs: no taking turns with them); a guard within GUARD_REARM of the last one goes on from
    it (its set and its steps: never more than GUARD_MAX under the first one's set, however often the flag flaps).
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
LEAD_MIN_PROB_NEAR = 0.5      # ... or a lead this close, closing this fast (r2 rt 418-420: a stopping car ahead fell to
LEAD_NEAR_D = 60.0            # 0.53-0.66 while we closed at 5-6 m/s; following at our speed it is mostly 0.9-1.0)
LEAD_NEAR_CLOSING = 4.0       # m/s
LEAD_DROPOUT = 0.5            # s: the lead missing this long (one or two radarState frames) holds the window, no restart
LEAD_BRAKING = -1.5           # m/s^2 (aLeadK, the window's mean): a lead braking this hard ...
LEAD_BRAKING_SLOWER = 1.0     # m/s ... and slower than us by this, with the model planning a stop: a stopping car (G1)
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
END_SHARE = 0.75              # above GUARD_SPEED the flag needs the model's end speed at most this share of ours ...
END_SHARE_OFF = 0.80          # ... (and stays on up to this): the path alone is ~10 s long (median 10.0 s) and
                              # dips under 9 s on 3-6 % of the frames at a steady 50 mph (r2 / r3: 22 false guards)
STOP_END_V = 4.5              # m/s: the model's end speed this low is a planned stop (a lead at our speed then stops too)
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
GUARD_REARM = 5.0             # s: a new guard this soon after one ended goes on from it (never re-dips from a raised set)
CURVE_NEAR = 5 * MPH          # m/s: a map / camera curve or the exit plan wanting less than v + this explains a short
                              # path: no guard taps then (the exit plan or the curve governs; no taking turns with it)

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
    self.lead_a = deque()          # (t, aLeadK) of the candidate window
    self.lead_missing = 0.0        # s the lead has been missing (LEAD_DROPOUT)
    self.lead_stopping = False     # the candidate is a stopping car (braking, the model planning a stop)
    self.lead_gone = 0.0
    self.away_t = 0.0
    # G2
    self.short = False             # the model path is short (with the hysteresis)
    self.short_t = 0.0
    self.long_t = 0.0
    self.path_len = None
    self.guard_last = None         # (guard_set, guard_t, t) of the last guard that ended (GUARD_REARM)
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

  @staticmethod
  def _model_stop(ctx):
    """The model plans a stop: its path's end speed (velocity.x[-1]) at most STOP_END_V on a fresh frame."""
    ve = ctx.get("path_v")
    return bool(ve is not None and ctx.get("model_age", 1e9) <= MODEL_STALE and ve <= STOP_END_V)

  def _lead_candidate(self, ctx, lead, now, dt, v):
    """Advance the G1 window. True once the lead has been a stopped / slow closing car (or a stopping one: braking,
    slower than us, the model planning a stop) for LEAD_CONFIRM. A missing lead up to LEAD_DROPOUT holds the window."""
    if lead is None:
      self.lead_missing += dt
      if self.lead_missing <= LEAD_DROPOUT + 1e-6 and self.lead_t > 0.0:
        return self.lead_t >= LEAD_CONFIRM - 1e-6       # one or two missing radarState frames: held, no restart
      self.lead_t, self.lead_prev = 0.0, None
      self.lead_v.clear()
      self.lead_a.clear()
      self.lead_stopping = False
      return False
    self.lead_missing = 0.0
    d, vrel = float(lead["d"]), float(lead.get("vrel", 0.0))
    prev = self.lead_prev
    jump = prev is not None and abs(d - (prev[1] + vrel * max(now - prev[0], 0.0))) > LEAD_JUMP
    self.lead_prev = (now, d)
    if jump:
      self.lead_t = 0.0
      self.lead_v.clear()
      self.lead_a.clear()
    min_prob = LEAD_MIN_PROB_RADAR if ctx.get("radar_lead") else LEAD_MIN_PROB
    if d <= LEAD_NEAR_D and -vrel >= LEAD_NEAR_CLOSING:
      min_prob = min(min_prob, LEAD_MIN_PROB_NEAR)    # close and closing fast: a stopping car's confidence drops
    prob = float(lead.get("prob", 1.0))
    self.lead_v.append((now, float(lead["v"])))
    if lead.get("a") is not None:
      self.lead_a.append((now, float(lead["a"])))
    for q in (self.lead_v, self.lead_a):
      while q and now - q[0][0] > LEAD_CONFIRM + 1e-6:
        q.popleft()
    v_lead = max(x for _, x in self.lead_v)
    slow = v_lead <= LEAD_STOPPED or v_lead <= LEAD_SLOW_SHARE * v
    a_lead = sum(x for _, x in self.lead_a) / len(self.lead_a) if self.lead_a else 0.0
    stopping = bool(a_lead <= LEAD_BRAKING and -vrel >= LEAD_BRAKING_SLOWER and self._model_stop(ctx))
    self.lead_stopping = bool(stopping and not (slow and -vrel >= LEAD_CLOSING))
    ok = bool(prob >= min_prob and d <= LEAD_MAX_D and ((slow and -vrel >= LEAD_CLOSING) or stopping))
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
    if lead is not None and float(lead["d"]) < length + LEAD_BLOCK and not self._model_stop(ctx):
      self.short = False           # the short path is the lead's (G1's job), unless the model plans a stop: then the
      return length                # lead at our speed stops too (r2 rt 416 / 901: no warning before his brake)
    if v > GUARD_SPEED:
      ve = ctx.get("path_v")
      if ve is None or ve > (END_SHARE_OFF if self.short else END_SHARE) * v:
        self.short = False         # a short path with the model's end speed near ours is no stop (false dips)
        return length
    self.short = length < (thr + OFF_MARGIN if self.short else max(thr - ON_MARGIN, 0.0))
    return length

  @staticmethod
  def _explained(ctx, v):
    """A map / camera curve or the exit plan wants less than v + CURVE_NEAR (or the exit plan is busy): the short path
    is theirs; the far guard does not tap (no taking turns with them)."""
    if ctx.get("exit_busy"):
      return True
    for name in ("map_curve", "camera_curve", "exit"):
      c = (ctx.get("contribs") or {}).get(name)
      if c is not None and c.target is not None and c.target < v + CURVE_NEAR:
        return True
    return False

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
    if plus or self.gas_t >= GAS_HELD - 1e-6:
      if self.active is not None:
        self.active, self.declined = None, True
      self.restore = None                      # his SET+ / gas during the step-by-step raise: no ceiling any more
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

    explained = self._explained(ctx, v)
    if ep is None and not self.declined and not quiet and ctx.get("cruise_enabled", False) and v >= START_SPEED:
      far = length is not None and v > GUARD_SPEED and length - STOP_MARGIN >= GUARD_FAR and not sign
      guard_due = stop_on and far and self.short_t >= GUARD_START - 1e-6 and not explained
      if lead_ok or (stop_confirmed and not far) or guard_due:
        ep = self._start(now, set_speed)

    if ep is not None:
      # ---- release ----
      if ep["lead"] and not lead_ok and (self.away_t >= RELEASE_TIME - 1e-6 or
                                        (self.lead_gone >= LOST_RELEASE - 1e-6 and not self.short)):
        ep["lead"] = False
      if (ep["stop"] or ep["guard_set"] is not None) and not self.short and self.long_t >= LONG_RELEASE - 1e-6:
        if ep["guard_set"] is not None:
          ep["guard_was"] = ep["guard_set"]
        ep["stop"], ep["guard_set"] = False, None
      if lead_ok:
        ep["lead"] = True
      far = length is not None and v > GUARD_SPEED and length - STOP_MARGIN >= GUARD_FAR and not sign
      if stop_confirmed and not far:
        ep["stop"], ep["guard_set"] = True, None
      elif (stop_on and far and not ep["stop"] and self.short_t >= GUARD_START - 1e-6 and ep["guard_set"] is None and
            not explained):
        ep["guard_set"] = set_speed
        g = self.guard_last
        if g is not None and now - g[2] <= GUARD_REARM:
          ep["guard_set"], ep["guard_t"] = g[0], g[1]    # goes on from the last guard: never deeper than its bound
      if not ep["lead"] and not ep["stop"] and ep["guard_set"] is None:
        if ep.get("guard_was") is not None:
          self.guard_last = (ep["guard_was"], ep["guard_t"], now)
        self._end("cause gone")
        ep = None

    if ep is None:
      return self._restore(ctx, out, now, v, lead, length, set_speed, anchor)

    # ---- the target ----
    target = None
    if ep["lead"] and lead is not None:
      v_lead = max(x for _, x in self.lead_v) if self.lead_v else float(lead["v"])
      d = float(lead["d"])
      a_lead = sum(x for _, x in self.lead_a) / len(self.lead_a) if self.lead_a else 0.0
      if self.lead_stopping and a_lead < -0.1:
        stop_d = d + float(lead["v"]) ** 2 / (2.0 * -a_lead)   # where the braking car will stand
        tl = FLOOR_TRUE if stop_d <= FLOOR_DISTANCE else M.envelope(FLOOR_TRUE, max(stop_d - MARGIN, 0.0), v)
      elif v_lead <= LEAD_STOPPED and d <= FLOOR_DISTANCE:
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
      out.ceiling = max(ep["guard_set"] - ep["guard_t"] * self.step, CAR_MIN_SET)
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
