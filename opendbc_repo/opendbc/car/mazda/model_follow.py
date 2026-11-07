"""
Model follower for the Mazda Gen1 stock cruise: TEST ONLY. It works out the set speed it WOULD command and logs it;
it never presses a button (there is no "on" mode, and nothing here can reach the CAN messages).

The owner's question: openpilot's trained model already knows when to slow (a red light, a slow queue, a stopped car,
a curve); can its plan be turned into stock cruise set speed changes? openpilot's own planner (longitudinalPlan) is no
good for this in chill mode: on drive 52 it planned +0.9..1.3 m/s^2 toward the set speed until 192 s and only saw the
slow queue at 195 s, while the model's action.desiredAcceleration went negative at 188 s. So this follows the MODEL:

  * inputs, 20 times a second: our speed, the driver's speed (the follower never goes above it), cruise on, the model's
    desired acceleration (modelV2.action.desiredAcceleration, drivingModelData's copy when modelV2 is missing) and,
    when there is one, modelV2's velocity plan (velocity.x over velocity.t).
  * the stock cruise answers a set speed change about a second later, so the follower looks LOOK_AHEAD into the plan:
    the speed planned then, and the decel the plan needs from then on (the steepest average drop to any later point,
    up to PLAN_HORIZON, so a stop planned 5 s out is started early enough). Without a plan: the speed after the look
    ahead at the model's accel, and that accel.
  * the measured cruise (drives 44-50, curve_target.py): it slows by about CRUISE_GAIN (m/s^2) per m/s the set speed is
    below our speed, at most about CRUISE_MAX_DECEL; it speeds up toward a higher set speed by the gap (ACCEL_BY_GAP).
    So when slowing: set = planned speed - required decel / CRUISE_GAIN; when the plan speeds up: set = planned speed
    + the gap that gives the model's accel. Then bounded to [the car's minimum set speed, the driver's speed] and
    rounded to whole mph (km/h).
  * no churn: it only starts when the model wants at least START_DECEL for START_TIME; a change needs the target
    HYST_STEPS past the half step, MIN_CHANGE_GAP since the last change and REVERSE_GAP before going the other way.
    Held low while the model wants about nothing for STEADY_TIME above STEADY_MIN_SPEED, or as soon as the stock radar
    has followed a car for LEAD_RELEASE_TIME and the model wants no real slowing (the radar keeps the gap to that car),
    it goes back to the driver's speed.
  * every change is counted as would-be button presses (1 mph taps, and the press actions with long presses).

EpisodeTracker turns the follower's set speed against the real one into episodes for the logs (see its docstring).
No openpilot dependencies, so it can be unit tested.
"""
import math

MS_PER_MPH = 0.44704
MS_PER_KPH = 1 / 3.6
FLOOR_SPEED = 30.0 / 3.6        # the car's own minimum set speed (30 km/h = 18.64 mph)

OFF, TEST = 0, 1                # MazdaModelFollow: 0 off, 1 test (logs only). There is no "on".
DEFAULT_MODE = TEST             # collect data without the owner doing anything

DT = 0.05                       # s between updates (the card wrapper calls it every 5th frame: 20 Hz)
LOOK_AHEAD = 1.25               # s: the cruise answers ~1 s after a press, and the presses themselves take a little
CRUISE_GAIN = 0.19              # (m/s^2) of cruise decel per m/s the set speed is below our speed (measured)
CRUISE_MAX_DECEL = 1.0          # m/s^2: about the most the cruise slows by itself, however low the set speed
# the cruise's acceleration toward a higher set speed (median by gap, measured): gap mph -> m/s^2
ACCEL_GAP_MPH = (0.0, 2.5, 7.5, 12.5, 17.5, 22.5)
ACCEL_BY_GAP = (0.0, 0.33, 0.51, 0.75, 0.91, 1.06)
ACCEL_TAU = 0.4                 # s: low-pass of the model's accel and the plan's decel (they jitter frame to frame)
START_DECEL = 0.3               # m/s^2: the model must want at least this much slowing ...
START_TIME = 0.5                # s: ... for this long before the follower starts lowering
PLAN_MIN_DT = 1.0               # s: the plan's decel is judged at least this far past the look-ahead point ...
PLAN_HORIZON = 6.0              # s: ... and no further than this (the far end of the plan is the least sure)
# A change needs the target HYST_STEPS past the half step, and UP_HYST_STEPS to go back up while the model still wants to
# slow (the required decel is turned into a gap at 1 / CRUISE_GAIN = 5.3 s: 0.1 m/s^2 of jitter is 1.2 mph).
HYST_STEPS = 0.7
UP_HYST_STEPS = 2.5
MIN_CHANGE_GAP = 1.0            # s between two changes
REVERSE_GAP = 3.0               # s before a change in the other direction
STEADY_ACCEL = 0.2              # m/s^2: the model wants about nothing ...
STEADY_TIME = 4.0               # s: ... this long while held low ...
STEADY_MIN_SPEED = 20.0 * MS_PER_MPH  # m/s: ... at this speed or more: back to the driver's speed (not below: a queue)
# The stock radar has followed a car LEAD_RELEASE_TIME and the model wants no more than STEADY_ACCEL of slowing: back to
# the driver's speed (the radar keeps the gap, the set speed only caps it; holding the set at our speed behind a car only
# cost presses in the qlog replays).
LEAD_RELEASE_TIME = 1.0         # s
HOLD_QUANTUM_MPH = 5            # the car's long press steps (1 mph at once, then multiples of 5 every ~0.6 s)


def mode_from_params(params, key="MazdaModelFollow"):
  """The follower's mode from the params: TEST unless the param says 0. A missing key (an older params build on the
  device), an unset value or garbage all mean TEST, which only logs; any other number is TEST too (there is no on)."""
  try:
    raw = params.get(key, encoding="utf8")
  except Exception:
    return DEFAULT_MODE
  if raw is None or raw == "" or raw == b"":
    return DEFAULT_MODE
  try:
    v = int(raw)
  except (TypeError, ValueError):
    return DEFAULT_MODE
  return OFF if v == OFF else TEST


def _interp(x, xs, ys):
  if x <= xs[0]:
    return ys[0]
  for k in range(1, len(xs)):
    if x <= xs[k]:
      x0, x1 = xs[k - 1], xs[k]
      return ys[k - 1] + (ys[k] - ys[k - 1]) * (x - x0) / (x1 - x0) if x1 > x0 else ys[k]
  return ys[-1]


def gap_for_accel(a):
  """The set speed gap above our speed (m/s) that makes the cruise speed up at `a` (m/s^2), from ACCEL_BY_GAP."""
  if a <= 0.0:
    return 0.0
  return _interp(a, ACCEL_BY_GAP, ACCEL_GAP_MPH) * MS_PER_MPH


def plan_look(plan_t, plan_v, look=LOOK_AHEAD, min_dt=PLAN_MIN_DT, horizon=PLAN_HORIZON):
  """From the model's velocity plan: (the planned speed change from now to `look` s, the decel the plan needs after
  that: the steepest average drop from the look-ahead point to any point min_dt..horizon s later; 0 if none)."""
  v0 = plan_v[0]
  v_look = _interp(look, plan_t, plan_v)
  req = 0.0
  for tk, vk in zip(plan_t, plan_v, strict=True):
    if tk < look + min_dt:
      continue
    if tk > horizon:
      break
    d = (v_look - vk) / (tk - look)
    if d > req:
      req = d
  return v_look - v0, req


class ModelFollower:
  """See the module docstring. update() returns the set speed it would command (m/s, 0.0 with cruise off or no model);
  the other results are attributes: target (unrounded), reason, presses / direction this update, the totals."""

  def __init__(self, is_metric=False, dt=DT):
    self.step = MS_PER_KPH if is_metric else MS_PER_MPH
    self.quantum = int(round(HOLD_QUANTUM_MPH * 1.609344)) if is_metric else HOLD_QUANTUM_MPH
    self.dt = dt
    self.alpha = dt / (ACCEL_TAU + dt)
    self.floor = FLOOR_SPEED
    k = int(math.floor(self.floor / self.step + 1e-6))
    self.first_step = k + 1          # the first whole step above the floor (19 mph; 31 km/h)
    # totals for the drive
    self.taps = 0                    # would-be 1 mph (km/h) steps
    self.actions = 0                 # would-be press actions (a long press counts once)
    self.changes = 0
    self.lowers = 0                  # times it started lowering from the driver's speed
    self.below_time = 0.0            # s with the command below the driver's speed
    self._reset()
    self.reason = "off"

  def _reset(self):
    self.t = 0.0
    self.a_f = None
    self.req_plan_f = 0.0
    self.engaged = False
    self.cmd = 0.0
    self.target = 0.0
    self.req = 0.0
    self.v_look = 0.0
    self.start_timer = 0.0
    self.steady_timer = 0.0
    self.lead_timer = 0.0
    self.last_change_t = -1e9
    self.last_dir = 0
    self.presses = 0
    self.press_actions = 0
    self.direction = 0
    self.used_plan = False

  # ---- the set speed grid ----
  def quantize(self, s):
    """The nearest speed the car can be set to: the floor or a whole step above it."""
    first = self.first_step * self.step
    if s < (self.floor + first) / 2:
      return self.floor
    return max(math.floor(s / self.step + 0.5) * self.step, first)

  def _k(self, s):
    """Step number of a set speed on the car's grid (the floor counts as the step below the first whole step when it
    is not a whole step itself: 18.64 mph is one tap below 19)."""
    if s <= self.floor + 0.01:
      return self.first_step - 1
    return int(math.floor(s / self.step + 0.5))

  def press_estimate(self, a, b):
    """(taps, actions) to go from set speed a to b: taps = whole steps; actions = presses with long presses (one hold:
    1 step at once, then multiples of the quantum; the rest taps), whichever is fewer."""
    ka, kb = self._k(a), self._k(b)
    n = abs(ka - kb)
    if n <= 2:
      return n, n
    q = self.quantum
    if kb < ka:
      x = ka - 1
      floor_k = self.first_step - 1
      while True:
        nx = ((x - 1) // q) * q
        if nx < kb:
          if kb == floor_k and x > kb:
            x = kb                     # the car stops a hold at its minimum
          break
        x = nx
      return n, min(n, 1 + (x - kb))
    x = ka + 1
    while True:
      nx = (x // q + 1) * q
      if nx > kb:
        break
      x = nx
    return n, min(n, 1 + (kb - x))

  # ---- the follower ----
  def update(self, v_ego, driver_speed, cruise_enabled, model_accel, plan_t=None, plan_v=None, radar_lead=False):
    """radar_lead: the stock radar has a car it follows (CS.radar_has_lead)."""
    self.presses = self.press_actions = self.direction = 0
    if (not cruise_enabled or driver_speed <= 0.0 or model_accel is None or not math.isfinite(model_accel) or
        not math.isfinite(v_ego)):
      self._reset()
      self.reason = "off"
      return 0.0
    dt = self.dt
    self.t += dt
    a = min(max(model_accel, -5.0), 5.0)
    self.a_f = a if self.a_f is None else self.a_f + self.alpha * (a - self.a_f)

    # where the plan has us after the look-ahead, and the decel it needs from there
    rel, req_plan = None, 0.0
    self.used_plan = False
    if plan_t is not None and plan_v is not None and len(plan_t) >= 2 and len(plan_t) == len(plan_v):
      try:
        rel, req_plan = plan_look(plan_t, plan_v)
        if not (math.isfinite(rel) and math.isfinite(req_plan)):
          rel, req_plan = None, 0.0
        else:
          self.used_plan = True
      except (TypeError, ValueError, ZeroDivisionError):
        rel, req_plan = None, 0.0
    if rel is None:
      rel = self.a_f * LOOK_AHEAD
    self.req_plan_f += self.alpha * (req_plan - self.req_plan_f)
    req = max(-self.a_f, self.req_plan_f, 0.0)
    v_look = max(v_ego + rel, 0.0)
    if req > 0.0:
      target = v_look - min(req, CRUISE_MAX_DECEL) / CRUISE_GAIN
    else:
      target = v_look + gap_for_accel(self.a_f)
    floor = min(self.floor, driver_speed)
    target = min(max(target, floor), driver_speed)
    self.req, self.v_look, self.target = req, v_look, target

    band = (0.5 + HYST_STEPS) * self.step
    if not self.engaged:
      self.cmd = driver_speed
      self.start_timer = self.start_timer + dt if req >= START_DECEL else 0.0
      if self.start_timer >= START_TIME - 1e-6 and target <= driver_speed - band:
        self.engaged = True
        self.lowers += 1
        self.steady_timer = 0.0
    if not self.engaged:
      self.reason = "idle"
      return self.cmd

    if self.cmd > driver_speed:
      self.cmd = driver_speed          # the driver lowered his speed below ours: his change, not a press
    steady = abs(self.a_f) <= STEADY_ACCEL and self.req_plan_f <= STEADY_ACCEL and v_ego >= STEADY_MIN_SPEED
    self.steady_timer = self.steady_timer + dt if steady else 0.0
    self.lead_timer = self.lead_timer + dt if radar_lead else 0.0
    released = self.steady_timer >= STEADY_TIME or (self.lead_timer >= LEAD_RELEASE_TIME - 1e-6 and req <= STEADY_ACCEL)
    want = driver_speed if released else target

    since = self.t - self.last_change_t
    up_band = band if (req <= 0.0 or want >= driver_speed) else (0.5 + UP_HYST_STEPS) * self.step
    new = None
    if want <= self.floor + 1e-6 and self.cmd > self.floor + 1e-3:
      new = self.floor                 # (the floor is less than a step below the first whole step: no band there)
      d = -1
    elif want < self.cmd - band:
      new = min(self.quantize(want), driver_speed)
      d = -1
    elif want > self.cmd + up_band:
      new = min(self.quantize(want), driver_speed)
      d = 1
    if new is not None and abs(new - self.cmd) > 1e-3 and since >= MIN_CHANGE_GAP - 1e-6 and \
       (self.last_dir in (0, d) or since >= REVERSE_GAP - 1e-6):
      taps, actions = self.press_estimate(self.cmd, new)
      self.presses, self.press_actions, self.direction = taps, actions, d
      self.taps += taps
      self.actions += actions
      self.changes += 1
      self.cmd = new
      self.last_change_t = self.t
      self.last_dir = d
      self.reason = "slowing" if d < 0 else "recovering"
    elif want < self.cmd - band:
      self.reason = "slowing"
    elif want > self.cmd + up_band:
      self.reason = "recovering"
    else:
      self.reason = "holding"

    if self.cmd >= driver_speed - 1e-3 and want >= driver_speed - band:
      self.engaged = False             # back at the driver's speed
      self.cmd = driver_speed
      self.steady_timer = 0.0
      self.start_timer = 0.0
      self.reason = "idle"
    else:
      self.below_time += dt
    return self.cmd


class EpisodeTracker:
  """Episodes for the logs: one starts when the follower's set speed differs from the real set speed by START_DIFF or
  more for START_TIME, and ends when it is back within END_DIFF for END_TIME (or cruise goes off, or MAX_EPISODE). The
  speeds and the driver's brake are watched AFTER more seconds, then the episode is handed back to be written.

  update() returns None, or a list of ("open", summary) / ("close", summary) for the caller's event log, in order. Times in the summaries
  are seconds relative to the episode start (negative: before it). Speeds in mph."""

  START_DIFF = 3.0 * MS_PER_MPH
  START_TIME = 1.0
  END_DIFF = 1.0 * MS_PER_MPH
  END_TIME = 2.0
  AFTER = 8.0
  MAX_EPISODE = 150.0               # s (the event log closes anything open after 180 s)
  MAX_PER_DRIVE = 25                # episodes written per drive; the rest are only counted

  def __init__(self):
    self.state = "idle"               # idle, open, after
    self.cand_t = None
    self.ok_t = None
    self.ep = None
    self.episodes = 0                 # episodes seen this drive
    self.written = 0
    self.skipped = 0
    self.follower_lower = 0           # episodes where the follower was below the real set speed
    self.actual_lower = 0
    self.follow_low_since = None      # when the follower last went below the driver's speed
    self.actual_low_since = None      # when the real set speed last went below the driver's speed

  @staticmethod
  def _rel(t, t0):
    return None if t is None else round(t - t0, 2)

  def update(self, t, cruise, v, a_ego, follow_set, actual_set, driver_speed, model_a, reason, presses, actions,
             lead_status, lead_d, lead_v, lead_prob, red_light, features, brake, gas, driver_btn):
    out = None
    on = cruise and follow_set > 0.0 and actual_set > 0.0
    half = 0.5 * MS_PER_MPH
    if on and follow_set < driver_speed - half:
      if self.follow_low_since is None:
        self.follow_low_since = t
    else:
      self.follow_low_since = None
    if on and actual_set < driver_speed - half:
      if self.actual_low_since is None:
        self.actual_low_since = t
    else:
      self.actual_low_since = None

    ep = self.ep
    if self.state == "after":
      ep["v_min_after"] = min(ep["v_min_after"], v)
      ep["brake_after"] |= bool(brake)
      if t - ep["t_end"] >= self.AFTER:
        out = self._finish()
        out = [out] if out is not None else None

    diff = follow_set - actual_set if on else 0.0
    if self.state == "open":
      self._accumulate(t, v, a_ego, follow_set, actual_set, model_a, presses, actions, lead_status, lead_d, lead_prob,
                       red_light, features, brake, gas, driver_btn)
      if not on:
        self._end(t, "cruise off" if not cruise else "follower off")
      elif t - ep["t0"] >= self.MAX_EPISODE:
        self._end(t, "max length")
      elif abs(diff) <= self.END_DIFF:
        self.ok_t = t if self.ok_t is None else self.ok_t
        if t - self.ok_t >= self.END_TIME - 1e-6:
          self._end(t, "back within 1 mph")
      else:
        self.ok_t = None
      return out

    if on and abs(diff) >= self.START_DIFF:
      self.cand_t = t if self.cand_t is None else self.cand_t
      if t - self.cand_t >= self.START_TIME - 1e-6:
        if self.state == "after":
          done = self._finish()        # (a new one before the last one's after window ended)
          if done is not None:
            out = [done]
        self._open(t, v, a_ego, follow_set, actual_set, driver_speed, model_a, reason, lead_status, lead_d, lead_v, lead_prob,
                   red_light, features)
        if self.written < self.MAX_PER_DRIVE:
          self.written += 1
          self.ep["logged"] = True
          return (out or []) + [("open", self._start_summary())]
        self.skipped += 1
    else:
      self.cand_t = None
    return out

  def _open(self, t, v, a_ego, follow_set, actual_set, driver_speed, model_a, reason, lead_status, lead_d, lead_v, lead_prob,
            red_light, features):
    t0 = self.cand_t
    lower = follow_set < actual_set
    self.episodes += 1
    if lower:
      self.follower_lower += 1
    else:
      self.actual_lower += 1
    self.state = "open"
    self.ok_t = None
    self.ep = {
      "t0": t0, "t_open": t, "logged": False, "kind": "follower_lower" if lower else "actual_lower",
      "v0": v, "set0": actual_set, "driver0": driver_speed, "follow0": follow_set, "model_a0": model_a, "reason0": reason,
      "lead0": (bool(lead_status), lead_d, lead_v, lead_prob), "red0": bool(red_light), "features0": features,
      "follow_low_since": self.follow_low_since, "actual_low_since": self.actual_low_since,
      "v_min": v, "v_end": v, "v_min_after": v, "a_min": a_ego, "follow_min": follow_set, "follow_max": follow_set,
      "set_min": actual_set, "set_max": actual_set, "model_a_min": model_a, "lead_d_min": lead_d if lead_status else None,
      "lead_prob_max": lead_prob if lead_status else 0.0, "red_any": bool(red_light), "features": set(),
      "brake": False, "brake_t": None, "brake_after": False, "gas": False, "driver_btn": False, "taps": 0, "actions": 0,
      "follow_first_lower": None, "actual_first_lower": None, "decel_t": None, "t_end": None, "ended_by": "",
      "diff_min": follow_set - actual_set, "diff_max": follow_set - actual_set,
    }
    if features:
      self.ep["features"].update(features.split("+"))

  def _start_summary(self):
    ep = self.ep
    t0 = ep["t0"]
    lead_status, lead_d, lead_v, lead_prob = ep["lead0"]
    return {"mode": "test", "kind": ep["kind"], "vEgo_mph": ep["v0"] / MS_PER_MPH, "set_mph": ep["set0"] / MS_PER_MPH,
            "driver_mph": ep["driver0"] / MS_PER_MPH, "follow_mph": ep["follow0"] / MS_PER_MPH, "model_accel": ep["model_a0"],
            "follow_reason": ep["reason0"], "lead": lead_status, "lead_d": lead_d if lead_status else None,
            "lead_v_mph": lead_v / MS_PER_MPH if lead_status else None, "lead_prob": lead_prob if lead_status else None,
            "red_light": ep["red0"], "features": ep["features0"] or "none",
            "follow_lower_since_s": self._rel(ep["follow_low_since"], t0),
            "actual_lower_since_s": self._rel(ep["actual_low_since"], t0), "opened_after_s": round(ep["t_open"] - t0, 2)}

  def _accumulate(self, t, v, a_ego, follow_set, actual_set, model_a, presses, actions, lead_status, lead_d, lead_prob, red_light,
                  features, brake, gas, driver_btn):
    ep = self.ep
    ep["v_min"] = min(ep["v_min"], v)
    ep["v_end"] = v
    ep["v_min_after"] = min(ep["v_min_after"], v)
    ep["a_min"] = min(ep["a_min"], a_ego)
    if a_ego < -0.5 and ep["decel_t"] is None:
      ep["decel_t"] = t
    if follow_set > 0.0:
      ep["follow_min"] = min(ep["follow_min"], follow_set)
      ep["follow_max"] = max(ep["follow_max"], follow_set)
      if follow_set < ep["follow0"] - 0.5 * MS_PER_MPH and ep["follow_first_lower"] is None:
        ep["follow_first_lower"] = t
    if actual_set > 0.0:
      ep["set_min"] = min(ep["set_min"], actual_set)
      ep["set_max"] = max(ep["set_max"], actual_set)
      if actual_set < ep["set0"] - 0.5 * MS_PER_MPH and ep["actual_first_lower"] is None:
        ep["actual_first_lower"] = t
      if follow_set > 0.0:
        d = follow_set - actual_set
        ep["diff_min"] = min(ep["diff_min"], d)
        ep["diff_max"] = max(ep["diff_max"], d)
    ep["model_a_min"] = min(ep["model_a_min"], model_a)
    if lead_status:
      ep["lead_d_min"] = lead_d if ep["lead_d_min"] is None else min(ep["lead_d_min"], lead_d)
      ep["lead_prob_max"] = max(ep["lead_prob_max"], lead_prob)
    ep["red_any"] |= bool(red_light)
    if features:
      ep["features"].update(features.split("+"))
    if brake and ep["brake_t"] is None:
      ep["brake_t"] = t
    ep["brake"] |= bool(brake)
    ep["gas"] |= bool(gas)
    ep["driver_btn"] |= bool(driver_btn)
    ep["taps"] += presses
    ep["actions"] += actions

  def _end(self, t, why):
    self.ep["t_end"] = t
    self.ep["ended_by"] = why
    self.state = "after"
    self.ok_t = None
    self.cand_t = None

  def _finish(self):
    ep = self.ep
    self.state = "idle"
    self.ep = None
    if not ep["logged"]:
      return None
    t0 = ep["t0"]
    # the follower's and the real set speed's first drop: before the start when it was already low then
    f_low = ep["follow_low_since"] if ep["follow_low_since"] is not None else ep["follow_first_lower"]
    a_low = ep["actual_low_since"] if ep["actual_low_since"] is not None else ep["actual_first_lower"]
    return ("close", {
      "episode_s": round(ep["t_end"] - t0, 2), "ended_by": ep["ended_by"],
      "v_min_mph": ep["v_min"] / MS_PER_MPH, "v_end_mph": ep["v_end"] / MS_PER_MPH,
      "v_min_after_mph": ep["v_min_after"] / MS_PER_MPH, "aEgo_min": ep["a_min"],
      "follow_min_mph": ep["follow_min"] / MS_PER_MPH, "follow_max_mph": ep["follow_max"] / MS_PER_MPH,
      "set_min_mph": ep["set_min"] / MS_PER_MPH, "set_max_mph": ep["set_max"] / MS_PER_MPH,
      "diff_min_mph": ep["diff_min"] / MS_PER_MPH, "diff_max_mph": ep["diff_max"] / MS_PER_MPH,
      "model_accel_min": ep["model_a_min"], "lead_d_min": ep["lead_d_min"], "lead_prob_max": ep["lead_prob_max"],
      "red_light_any": ep["red_any"], "features_any": "+".join(sorted(ep["features"])) or "none",
      "driver_braked": ep["brake"], "driver_braked_after": ep["brake_after"], "brake_at_s": self._rel(ep["brake_t"], t0),
      "gas": ep["gas"], "driver_set_button": ep["driver_btn"], "would_taps": ep["taps"], "would_actions": ep["actions"],
      "follow_lower_at_s": self._rel(f_low, t0), "actual_lower_at_s": self._rel(a_low, t0),
      "decel_at_s": self._rel(ep["decel_t"], t0)})
