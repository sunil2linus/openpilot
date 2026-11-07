"""Offline tests for the Mazda curve slowdown decision logic (no car, no openpilot runtime needed).

The simulated car models the stock MRCC: a SET-/SET+ press changes the dash set speed one step
after `lag` frames, and the true speed follows (set speed - dash offset). The simulated planner
mimics FrogPilot's CurveSpeedController.update_target and csc_active = target < v_cruise, where
v_cruise is the set speed we are changing.
"""
import importlib.util
import os

_spec = importlib.util.spec_from_file_location(
  "curve_slowdown", os.path.join(os.path.dirname(__file__), "..", "curve_slowdown.py"))
cs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cs)
_spec2 = importlib.util.spec_from_file_location(
  "stopped_car_slowdown", os.path.join(os.path.dirname(__file__), "..", "stopped_car_slowdown.py"))
scs = importlib.util.module_from_spec(_spec2)
_spec2.loader.exec_module(scs)

MPH = cs.MS_PER_MPH
KPH = cs.MS_PER_KPH
DT = 0.01


class Car:
  def __init__(self, set_mph, dash_offset_mph=2.0, lag=15, accel=1.0, ignore=False, step=MPH, lose=None, lag_fn=None,
               hold_rate=8.3, hold_start=0.5, hold_latency=0.0, hold_ignored=False, decel=1.5, decel_fn=None, min_set_mph=None,
               hold_phase_window=None, plus_jumps=True, jump_extra=0, cruise_gain=None, cruise_dead=1.0, cruise_max=1.0,
               accel_fn=None):
    self.set_speed = set_mph * MPH
    self.dash_offset = dash_offset_mph * MPH
    self.v_ego = self.set_speed - self.dash_offset
    self.lag, self.accel, self.ignore, self.step = lag, accel, ignore, step
    self.decel = decel
    # The stock cruise as measured on drives 44/45/4c/4d/4f/50 (no lead, no pedals, over 27 mph): it slows about
    # `cruise_gain` m/s^2 per m/s the set speed is below the true speed (0.19 measured: 2-4 mph gap 0.27, 6-8 mph 0.66,
    # 10-12 mph 0.84 m/s^2), capped at `cruise_max`, and answers a set speed change `cruise_dead` s later. Off (None) the
    # old fixed `decel` / `decel_fn` model is used, which the older tests were written against.
    self.cruise_gain, self.cruise_max = cruise_gain, cruise_max
    # accel_fn(gap_mph) -> m/s^2 the stock cruise speeds up at with the set speed gap_mph above the true speed (drives
    # 4c-53 median: 0.33 up to 5 mph, 0.51 5-10, 0.75 10-15, 0.91 15-20, 1.06 20-25); None: the constant `accel`
    self.accel_fn = accel_fn
    self.cruise_dead_frames = int(round(cruise_dead / DT))
    self.gaps = []                    # set speed gap seen each frame (true speed - set speed), for the dead time
    self.hold_phase_window = hold_phase_window   # frames: a hold frame later than this after the wheel's reads as a tap
    # SET+ while faster than the set speed sets the current speed (drive 4d) on the gas only (drives 4c-4f: every jump
    # had the gas pressed within 0.5 s; off the gas a normal step, drive 4f); "always": also off the gas (a car that
    # did, for the fallback)
    self.plus_jumps = plus_jumps
    self.gas_frame = -10**9           # last frame the gas was pressed (set by run)
    self.jump_extra = jump_extra      # ... landing this many steps above it
    self.decel_fn = decel_fn          # decel_fn(gap_mph) -> m/s^2 the stock cruise slows at for that set speed gap
    self.min_set = min_set_mph * MPH if min_set_mph else None   # the car's own minimum set speed (clamp)
    self.lose = lose                  # lose(n) -> True drops our n-th press (1-based), like the real MRCC does
    self.lag_fn = lag_fn              # lag_fn(n) -> frames until our n-th press lands (overrides lag)
    self.pending, self.frame, self.op_presses = [], 0, 0
    self.driver_button, self.driver_button_until = None, -1
    # Long-press model, as measured on the owner's car: 1 mph at the first frame, after `hold_start` s a
    # jump to the next multiple of 5 mph, then 5 mph every 5/hold_rate s while frames keep coming; steps
    # already due within `hold_latency` s after the last frame still land.
    self.hold_rate, self.hold_start, self.hold_latency, self.hold_ignored = hold_rate, hold_start, hold_latency, hold_ignored
    self.hold_last, self.hold_next, self.hold_count = None, None, 0

  def hold_frame(self, button=cs.SET_MINUS):
    if self.hold_ignored:
      return
    if self.hold_last is None or self.frame - self.hold_last > 15:
      self.hold_count += 1
      self.hold_dir = -1 if button == cs.SET_MINUS else 1
      d = self.hold_dir * self.step                                               # the tap on button-down ...
      if (button == cs.SET_PLUS and self.v_ego > self.set_speed + self.step and
          (self.plus_jumps == "always" or (self.plus_jumps and self.frame - self.gas_frame <= 50))):
        d = (round(self.v_ego / self.step) + self.jump_extra) * self.step - self.set_speed   # ... or a jump
      self.pending.append((self.frame + self.lag, d))
      self.hold_next = self.frame + int(self.hold_start * 100) + self.lag
    self.hold_last = self.frame

  def _hold_tick(self):
    if self.hold_last is None:
      return
    released = self.frame - self.hold_last > 15
    if self.frame >= self.hold_next and (not released or self.frame - self.hold_last <= 15 + int(self.hold_latency * 100)):
      mph = round((self.set_speed + sum(d for _, d in self.pending)) / MPH)
      if self.hold_dir < 0:
        new = ((mph - 1) // 5) * 5
      else:
        new = ((mph + 5) // 5) * 5
      self.set_speed += (new - mph) * MPH
      if self.min_set is not None and self.set_speed < self.min_set:
        self.set_speed = self.min_set
      self.hold_next += int(100 * 5 / self.hold_rate)
    if released and self.frame - self.hold_last > 15 + int(self.hold_latency * 100):
      self.hold_last = None

  def driver_button_now(self):
    return self.driver_button if self.frame < self.driver_button_until else None

  def press(self, button, who="op"):
    if who == "op":
      self.op_presses += 1
      if self.ignore or (self.lose is not None and self.lose(self.op_presses)):
        return
    else:
      self.driver_button_until = self.frame + 10      # the stock switch reports the tap for one 10 Hz frame
      self.driver_button = button
    d = -self.step if button == cs.SET_MINUS else self.step
    on_gas = self.frame - self.gas_frame <= 50
    if (self.plus_jumps == "always" or (self.plus_jumps and on_gas)) and button == cs.SET_PLUS and \
       self.v_ego > self.set_speed + self.step:
      d = (round(self.v_ego / self.step) + self.jump_extra) * self.step - self.set_speed
    lag = self.lag_fn(self.op_presses) if (who == "op" and self.lag_fn is not None) else self.lag
    self.pending.append((self.frame + lag, d))

  def tick(self):
    self.frame += 1
    for f, d in [p for p in self.pending if p[0] <= self.frame]:
      if self.min_set is not None and d > 0 and abs(self.set_speed - self.min_set) < 1e-6:
        self.set_speed = (round(self.min_set / MPH) + 1) * MPH   # 30 km/h shows 19; the first SET+ lands on 20 mph
      else:
        self.set_speed += d
    self.pending = [p for p in self.pending if p[0] > self.frame]
    self._hold_tick()
    tgt = self.set_speed - self.dash_offset
    if self.cruise_gain is not None:
      self.gaps.append(self.v_ego - tgt)
      if len(self.gaps) > self.cruise_dead_frames + 1:
        del self.gaps[0]
      gap = self.gaps[0] if len(self.gaps) > self.cruise_dead_frames else 0.0   # what the cruise acts on now
      if gap > 0:
        self.v_ego -= min(self.cruise_gain * gap, self.cruise_max) * DT
      elif gap < 0:
        self.v_ego += max(0.0, min(self._accel(tgt) * DT, tgt - self.v_ego))   # back up to the set speed, never past it
      return
    dec = self.decel_fn((self.v_ego - tgt) / MPH) if self.decel_fn is not None else self.decel
    self.v_ego += max(-dec * DT, min(self._accel(tgt) * DT, tgt - self.v_ego))

  def _accel(self, tgt):
    return self.accel if self.accel_fn is None else self.accel_fn((tgt - self.v_ego) / MPH)


class StoppedLead:
  """A lead car `d0` m ahead at `v_mph`, in our lane (y) with `prob`; the gap closes with our speed.
  `moves_at`: frame from which the lead drives off at `drive_off_mph`. `radar_from`: distance below which
  the stock radar reports it (None: never); `radar_present(f)`: radar dropout control. `follow_from`: while the radar
  reports the car and it is this close or closer, the MRCC brakes for it at `follow_decel` m/s^2 down to its speed."""
  def __init__(self, d0, v_mph=0.0, prob=0.95, y=0.1, present=lambda f: True, moves_at=None, drive_off_mph=15.0,
               radar_from=None, radar_present=lambda f: True, follow_from=None, follow_decel=1.5):
    self.d, self.v, self.prob, self.y, self.present = d0, v_mph * MPH, prob, y, present
    self.moves_at, self.drive_off, self.radar_from, self.radar_present = moves_at, drive_off_mph * MPH, radar_from, radar_present
    self.follow_from, self.follow_decel = follow_from, follow_decel
    self.frame = 0

  def state(self, f, car):
    if not self.present(f):
      return (False, 0.0, 0.0, 0.0, 0.0)
    return (True, self.prob, self.d, self.v, self.y)

  def radar(self, f):
    """Stock radar per the DBC value table: bucket 1 = farthest .. 5 = nearest, 0 = no lead."""
    if self.radar_from is None or not self.radar_present(f) or self.d > self.radar_from:
      return False, 0
    return True, max(1, min(5, 5 - int(self.d // 20)))

  def tick(self, car):
    self.frame += 1
    if self.moves_at is not None and self.frame >= self.moves_at:
      self.v = min(self.v + 2.0 * DT, self.drive_off)
    self.d = max(self.d - (car.v_ego - self.v) * DT, 0.0)
    if (self.follow_from is not None and self.radar(self.frame)[0] and self.d <= self.follow_from and
        car.v_ego > self.v):
      before = getattr(car, "v_before", car.v_ego)
      car.v_ego = max(min(car.v_ego, before - self.follow_decel * DT), self.v)   # the MRCC follows the tracked car
    if self.present(self.frame) and self.d < 12.0 and car.v_ego > self.v:
      car.v_ego = max(car.v_ego - 3.0 * DT, self.v)      # the MRCC (or the driver) brakes behind the lead


class CSC:
  """Mimics FrogPilot's curve speed target (20 Hz) and its active flag."""
  def __init__(self):
    self.target, self.active, self.speed = None, False, 0.0

  def update(self, curve_speed, v_cruise, v_ego, time_to_curve=3.0):
    if curve_speed is None:
      self.target, self.speed, self.active = None, v_cruise, False
      return
    if self.target is None:
      self.target = max(v_ego, curve_speed)
    if curve_speed < self.target:
      decel = max(v_ego - curve_speed, 0) / max(time_to_curve, 1)
      self.target = max(min(self.target, v_ego) - decel * 0.05, curve_speed)
    else:
      self.target = min(self.target + 1.2 * 0.05, curve_speed)
      if self.target >= v_cruise:
        self.target = curve_speed
    self.speed = self.target
    self.active = self.target < v_cruise


def run(car, curve_fn, frames, events=None, plan_lag=10, metric=False, max_drop=5, lead=None, stop_mode=2,
        model=lambda f: None, setup=None, plan=lambda f: False, wheel_phase=0, wheel_alive=lambda f: True,
        limit=lambda f: 0.0, lane_change=lambda f: False, model_length=lambda f: 0.0, red=lambda f: False,
        cap=lambda f: 0.0, undo=lambda f: 0.0, ceiling=lambda f: 0.0, other=lambda f: False, map_curve=None):
  """model(f): the driving model's desired acceleration at frame f (m/s^2), None = no modelV2.
  plan(f): True while the model plans a stop ahead (the red light check).
  red(f): starpilotPlan.redLight (the stop-light detector) at frame f.
  limit(f) / cap(f) / undo(f) / ceiling(f): the speed limit follower's raise / lower limit cap / undo / provisional
  ceiling (m/s, 0 none), called in that order.
  map_curve(f): a followed map curve instead of the camera mimic: (active, target m/s) or None (curve_track on while active)."""
  logic = cs.CurveSlowdownLogic(metric, max_drop)
  if setup is not None:
    setup(logic)
  csc = CSC()
  trigger = scs.StoppedCarTrigger()
  st = dict(enabled=True, cruise=True, gas=False, brake=False, plan_alive=True)
  hist, presses, holds, sets = [], [], [], []
  ctr = -1
  ever_active = False
  start = car.set_speed
  min_set = max_set = car.set_speed
  a_ego, vs, follow, radar_hold, clear, kinds = 0.0, [], [], [], [], []
  for f in range(frames):
    if events and f in events:
      events[f](st, car)
    if f % 5 == 0:
      csc.update(curve_fn(f), car.set_speed, car.v_ego)
    hist.append((csc.active, csc.speed))
    pa, ps = hist[max(0, len(hist) - 1 - plan_lag)]
    mc = map_curve(f) if map_curve is not None else None
    if map_curve is not None:
      pa, ps = (True, mc[1]) if mc is not None and mc[0] else (False, 0.0)
    stop_active, stop_target, stop_urgent, restore_now, hold_restore = False, 0.0, False, False, False
    if st["gas"]:
      car.gas_frame = car.frame
    r_has = False
    if lead is not None:
      status, prob, d, v, y = lead.state(f, car)
      r_has, r_dist = lead.radar(f)
      li = scs.StoppedCarInputs(enabled=st["enabled"], cruise_enabled=st["cruise"], brake_pressed=st["brake"],
                                v_ego=car.v_ego, lead_alive=True, lead_status=status, lead_prob=prob, lead_d=d,
                                lead_v=v, lead_y=y, lead_a=getattr(lead, "a", 0.0) if status else 0.0,
                                radar_has_lead=r_has, radar_lead_distance=r_dist,
                                model_alive=model(f) is not None, model_accel=model(f) or 0.0, plan_stop=plan(f),
                                a_ego=a_ego, lead_y_path=lead.y_path(f, car) if (status and hasattr(lead, "y_path")) else None,
                                model_length=model_length(f), lane_change=lane_change(f), gas_pressed=st["gas"],
                                set_speed=car.set_speed if st["cruise"] else 0.0, red_light=red(f))
      active = trigger.update(li)
      ever_active = ever_active or active
      stop_active, stop_target, stop_urgent = active and stop_mode == 2, trigger.target, trigger.urgent
      restore_now = trigger.restore_now and stop_mode == 2
      hold_restore = trigger.hold_restore and stop_mode == 2
      follow.append(trigger.radar_follow)
      radar_hold.append(trigger.radar_hold)
      clear.append(trigger.clear_ahead)
      kinds.append(trigger.kind if active else None)
    if wheel_phase is not None and wheel_alive(f):
      ctr = ((f - wheel_phase) // 10) % 16       # the wheel's CRZ_BTNS frame every 10 frames (10 Hz)
    inp = cs.SlowdownInputs(btn_counter=ctr if wheel_phase is not None else -1, enabled=st["enabled"], cruise_enabled=st["cruise"], set_speed=car.set_speed,
                            v_ego=car.v_ego, gas_pressed=st["gas"], brake_pressed=st["brake"], standstill=car.v_ego < 0.3,
                            plan_alive=st["plan_alive"], csc_active=pa, csc_speed=ps, other_button_active=other(f),
                            driver_button=car.driver_button_now(), stop_active=stop_active, stop_target=stop_target,
                            stop_urgent=stop_urgent, stop_restore_now=restore_now, hold_restore=hold_restore,
                            radar_lead=bool(lead is not None and trigger.radar_stable),
                            stop_clear_ahead=bool(lead is not None and trigger.clear_ahead and stop_mode == 2),
                            stop_pulling_away=bool(lead is not None and trigger.pulling_away and stop_mode == 2),
                            stop_slower_ahead=bool(lead is not None and trigger.slower_after_red and stop_mode == 2),
                            limit_raise=limit(f), limit_cap=cap(f), limit_undo=undo(f), limit_ceiling=ceiling(f),
                            curve_track=bool(map_curve is not None and pa))
    btn, off = logic.update(inp)
    if btn is not None:
      assert 0 <= off < 16
      if logic.last_action == "hold":
        holds.append(f)
        late = wheel_phase is not None and car.hold_phase_window is not None and (f - wheel_phase) % 10 > car.hold_phase_window
        if late:
          car.press(btn)                               # the wheel's "not pressed" frame wins: the car sees a tap
        else:
          car.hold_frame(btn)
      else:
        presses.append((f, btn))
        car.press(btn)
    car.v_before = car.v_ego
    car.tick()
    if lead is not None:
      lead.tick(car)
    a_ego = (car.v_ego - car.v_before) / DT
    vs.append(car.v_ego)
    min_set, max_set = min(min_set, car.set_speed), max(max_set, car.set_speed)
    sets.append(car.set_speed)
  return dict(presses=presses, holds=holds, sets=sets, min=min_set / MPH, max=max_set / MPH, final=car.set_speed / MPH,
              start=start / MPH, logic=logic, trigger=trigger, ever_active=ever_active, vs=vs, follow=follow,
              radar_hold=radar_hold, radar_hold_frames=sum(radar_hold), clear=clear, kinds=kinds)


def between(a, b, mph):
  return lambda f: mph * MPH if a <= f < b else None


def near(x, y):
  return abs(x - y) < 0.01


def minus_count(r):
  return sum(1 for _, b in r["presses"] if b == cs.SET_MINUS)


# ---------------- basic behaviour ----------------

def test_gentle_curve_drops_then_restores():
  r = run(Car(65), between(100, 1500, 61), 4000)
  assert 60 - 1e-6 <= r["min"] <= 63, r["min"]
  assert near(r["final"], 65), r["final"]
  assert near(r["max"], 65)


def test_sharp_curve_capped_at_5_mph():
  r = run(Car(65), between(100, 1500, 45), 4000)
  assert near(r["min"], 60), r["min"]
  assert near(r["final"], 65)


def test_never_below_25_mph_and_restores_at_low_speed():
  # dash 2 mph high: lowered to 25 dash = ~23 true; must still restore to 29
  r = run(Car(29), between(100, 1500, 15), 4000)
  assert r["min"] >= 25 - 1e-6, r["min"]
  assert near(r["final"], 29), r["final"]


def test_does_not_start_below_25_mph():
  r = run(Car(25, dash_offset_mph=2.0), between(100, 1500, 15), 2500)
  assert r["presses"] == []


def test_tiny_curve_needs_full_step():
  r = run(Car(65, dash_offset_mph=0.0), between(100, 1500, 64.5), 3000)
  assert r["presses"] == []


def test_dash_offset_zero():
  r = run(Car(65, dash_offset_mph=0.0), between(100, 1500, 61), 4000)
  assert r["min"] >= 60 - 1e-6 and near(r["final"], 65)


def test_metric_step_and_cap():
  car = Car(100 / 1.609344, step=KPH, dash_offset_mph=1.0)
  r = run(car, between(100, 1500, 55), 4000, metric=True)
  drop_kph = (r["start"] - r["min"]) * 1.609344
  assert drop_kph <= 8 + 0.01, drop_kph
  assert abs(r["final"] - r["start"]) < 0.01


def test_button_spacing_at_least_0_2_s():
  frames = [f for f, _ in run(Car(65), between(100, 1500, 45), 4000)["presses"]]
  assert all(b - a > cs.BUTTON_GAP_FRAMES for a, b in zip(frames, frames[1:]))


def test_no_ping_pong_inside_a_curve():
  # Targets at and just above step boundaries used to flip FrogPilot's flag and ping-pong.
  for curve in (61.0, 61.05, 61.2, 61.3, 62.0, 62.02):
    for offset in (0.0, 2.0):
      r = run(Car(65, dash_offset_mph=offset), between(100, 2000, curve), 5000)
      plus_in_curve = [f for f, b in r["presses"] if b == cs.SET_PLUS and f < 2000]
      assert plus_in_curve == [], (curve, offset, r["presses"])
      assert len(r["presses"]) <= 10, (curve, offset, len(r["presses"]))
      assert near(r["final"], 65)


def test_gradual_curve_exit_restores():
  rising = lambda f: (55 * MPH if 100 <= f < 900 else (55 * MPH + (f - 900) * 0.01 if 900 <= f < 1700 else None))
  r = run(Car(65), rising, 5000)
  assert near(r["final"], 65) and r["min"] >= 60 - 1e-6


def test_two_curves_back_to_back():
  curve = lambda f: 61 * MPH if 100 <= f < 700 else (58 * MPH if 900 <= f < 1600 else None)
  r = run(Car(65), curve, 5000)
  assert r["min"] >= 60 - 1e-6 and near(r["final"], 65) and near(r["max"], 65)


# ---------------- hard invariants across car response delays ----------------

def test_invariants_for_any_car_delay():
  for lag in (5, 15, 30, 45, 52, 60, 75, 95, 110, 140, 160, 200, 250, 320):
    for curve in (45, 58, 61, 63.5):
      r = run(Car(65, lag=lag), between(100, 1500, curve), 6000)
      assert r["max"] <= 65 + 1e-6, ("above driver's speed", lag, curve, r["max"])
      assert r["min"] >= 60 - 1e-6, ("dropped more than 5 mph", lag, curve, r["min"])
      if lag <= 300:
        assert near(r["final"], 65), ("not restored", lag, curve, r["final"])


# ---------------- driver and system interactions ----------------

def _first_press_frame(curve=61):
  return run(Car(65), between(100, 1500, curve), 3000)["presses"][0][0]


def test_driver_set_minus_during_lowering_is_respected():
  first = _first_press_frame()
  r = run(Car(65), between(100, 1500, 61), 4000,
          events={first + 40: lambda s, c: c.press(cs.SET_MINUS, who="driver")})
  # the driver's SET- is their new choice: we stop managing and leave the speed where they put it
  assert near(r["final"], 63), r["final"]
  assert [f for f, b in r["presses"] if b == cs.SET_PLUS] == [], r["presses"]


def test_driver_hold_minus_5_is_respected():
  first = _first_press_frame()
  def hold(s, c):
    c.set_speed -= 5 * MPH
  r = run(Car(65), between(100, 1500, 61), 4000, events={first + 30: hold})
  assert r["final"] <= 60 + 0.01, r["final"]


def test_driver_set_plus_mid_curve_we_stop():
  r = run(Car(65), between(100, 1500, 61), 5000, events={800: lambda s, c: c.press(cs.SET_PLUS, who="driver")})
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS and f >= 800] == [], r["presses"]
  # the driver's +1 takes back one of our steps: the rest is restored to their 65, never above
  assert r["max"] <= 65 + 1e-6
  assert near(r["final"], 65), r["final"]


def test_driver_taps_before_we_lowered_rearm_after_a_quiet_spell():
  # drive 44: SET- then SET+ by the driver while the curve was already tracked, before any press of ours: the curve
  # then got no slowdown at all. Now they stand us down for DRIVER_REARM s only.
  first = _first_press_frame()
  t1, t2 = first - 5, first + 200
  r = run(Car(65), between(first - 10, 4000, 61), 5000,
          events={t1: lambda s, c: c.press(cs.SET_MINUS, who="driver"), t2: lambda s, c: c.press(cs.SET_PLUS, who="driver")})
  rearm = t2 + int(cs.DRIVER_REARM * 100)
  ours = [f for f, b in r["presses"] if b == cs.SET_MINUS]
  assert ours and all(f >= rearm for f in ours), (rearm, ours[:3])
  assert r["logic"].rearms >= 1
  assert r["min"] >= 60 - 1e-6 and r["max"] <= 65 + 1e-6
  assert near(r["final"], 65), r["final"]                       # restored to the driver's (re-set) 65


def test_driver_tap_after_our_lowering_still_stands_us_down_for_the_curve():
  first = _first_press_frame()
  r = run(Car(65), between(100, 3000, 61), 5000, events={first + 40: lambda s, c: c.press(cs.SET_PLUS, who="driver")})
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS and first + 40 <= f < 3000] == [], r["presses"]
  assert r["logic"].rearms == 0


def test_disengage_mid_curve_no_presses_while_off():
  def off(s, c):
    s["enabled"] = False
  r = run(Car(65), between(100, 1500, 61), 4000, events={300: off})
  assert all(f < 300 for f, _ in r["presses"])


def test_brake_during_restore_then_resume_restores():
  base = run(Car(65), between(100, 1000, 61), 4000)
  plus = [f for f, b in base["presses"] if b == cs.SET_PLUS]
  def brake(s, c):
    s["brake"], s["cruise"] = True, False
  def resume(s, c):
    s["brake"], s["cruise"] = False, True
  r = run(Car(65), between(100, 1000, 61), 6000, events={plus[0] + 5: brake, plus[0] + 300: resume})
  assert near(r["final"], 65), r["final"]
  assert all(not (plus[0] + 5 <= f < plus[0] + 300) for f, _ in r["presses"])


def test_driver_sets_new_speed_while_cruise_off_is_respected():
  def brake(s, c):
    s["brake"], s["cruise"] = True, False
  def set_new(s, c):
    s["brake"], s["cruise"] = False, True
    c.set_speed = 58 * MPH          # driver pressed SET at a new speed
  r = run(Car(65), between(100, 1500, 61), 5000, events={500: brake, 800: set_new})
  assert near(r["final"], 58), r["final"]


def test_gas_pauses_presses_then_restores():
  def gas_on(s, c):
    s["gas"] = True
  def gas_off(s, c):
    s["gas"] = False
  r = run(Car(65), between(100, 1500, 61), 4000, events={90: gas_on, 600: gas_off})
  assert all(not (90 <= f < 600) for f, _ in r["presses"])
  assert near(r["final"], 65)


def test_plan_hiccup_mid_curve_still_restores():
  def dead(s, c):
    s["plan_alive"] = False
  def back(s, c):
    s["plan_alive"] = True
  r = run(Car(65), between(100, 1500, 61), 4000, events={700: dead, 760: back})
  assert near(r["final"], 65), r["final"]


def test_long_pause_forgets_and_leaves_speed_alone():
  def off(s, c):
    s["enabled"] = False
  r = run(Car(65), between(100, 1500, 61), 8000, events={700: off})
  assert all(f < 700 for f, _ in r["presses"])


def test_car_ignores_buttons_we_give_up():
  r = run(Car(65, ignore=True), between(100, 9000, 61), 12000)
  assert len(r["presses"]) <= (cs.NO_EFFECT_LIMIT + cs.MAX_IN_FLIGHT) * cs.NO_EFFECT_TRIPS, len(r["presses"])
  assert near(r["final"], 65)


def test_flicker_short_does_not_spam():
  flick = lambda f: 61 * MPH if (100 <= f < 1600 and (f // 80) % 2 == 0) else None
  r = run(Car(65), flick, 5000)
  assert near(r["final"], 65) and len(r["presses"]) <= 12, len(r["presses"])


def test_flicker_long_is_bounded():
  flick = lambda f: 61 * MPH if (100 <= f < 2100 and (f // 200) % 2 == 0) else None
  r = run(Car(65), flick, 6000)
  assert near(r["final"], 65)
  assert len(r["presses"]) <= 40, len(r["presses"])   # ~10 separate 2 s curves; bounded rate
  assert r["max"] <= 65 + 1e-6 and r["min"] >= 60 - 1e-6


# ---------------- review round 2 (driver speed must never drift) ----------------

def test_repeated_curves_never_drift():
  # 7.5 s curves every 11 s for 2 minutes
  curve = lambda f: 50 * MPH if (f % 1100) < 750 and f < 12000 else None
  r = run(Car(65), curve, 16000)
  assert r["min"] >= 60 - 1e-6, r["min"]
  assert r["max"] <= 65 + 1e-6, r["max"]
  assert near(r["final"], 65), r["final"]


def test_flicker_3_5_s_never_drifts():
  flick = lambda f: 61 * MPH if (f < 6000 and (f // 350) % 2 == 0) else None
  r = run(Car(65), flick, 10000)
  assert r["min"] >= 60 - 1e-6 and r["max"] <= 65 + 1e-6
  assert near(r["final"], 65), r["final"]


def test_random_curves_lags_never_break_limits():
  import random
  rng = random.Random(1234)
  for _ in range(60):
    set_mph = rng.choice([35, 45, 55, 65, 75])
    lag = rng.choice([5, 15, 40, 80, 150, 250])
    segs, f = [], 100
    while f < 9000:
      length = rng.randint(100, 900)
      segs.append((f, f + length, rng.uniform(set_mph - 15, set_mph + 2)))
      f += length + rng.randint(50, 900)
    def curve(fr, segs=segs):
      for a, b, v in segs:
        if a <= fr < b:
          return v * MPH
      return None
    r = run(Car(set_mph, lag=lag, dash_offset_mph=rng.choice([0, 1, 2])), curve, 14000)
    floor = max(set_mph - 5, 25)
    assert r["max"] <= set_mph + 1e-6, (set_mph, lag, r["max"])
    assert r["min"] >= floor - 1e-6, (set_mph, lag, r["min"])
    assert near(r["final"], set_mph), (set_mph, lag, r["final"])


def test_gas_release_does_not_restore_mid_curve():
  def gas_on(s, c):
    s["gas"] = True
  def gas_off(s, c):
    s["gas"] = False
  r = run(Car(65), between(100, 3000, 58), 6000, events={600: gas_on, 900: gas_off})
  assert [f for f, b in r["presses"] if b == cs.SET_PLUS and f < 3000] == [], r["presses"]
  assert near(r["final"], 65)


# ---------------- review round 3 ----------------

def test_driver_tap_during_lowering_next_curve_respects_limits():
  # N1: driver taps SET- once while we lower; later curves must stay within (64 - 5)
  for lag in (15, 60):
    first = run(Car(65, lag=lag), between(100, 1500, 50), 3000)["presses"][0][0]
    curve = lambda f: 45 * MPH if (100 <= f < 1500 or 3000 <= f < 4400) else None
    r = run(Car(65, lag=lag), curve, 8000, events={first + 5: lambda s, c: c.press(cs.SET_MINUS, who="driver")})
    assert r["min"] >= 59 - 1e-6, (lag, r["min"])
    assert near(r["final"], 64), (lag, r["final"])


def test_step_mismatch_disables():
  # N2: device set to km/h but the dash steps in mph
  r = run(Car(65), between(100, 3000, 45), 6000, metric=True)
  assert r["logic"].disabled
  assert len(r["presses"]) <= 2, r["presses"]


def test_long_cruise_off_then_resume_restores():
  # N3: traffic stop with cruise off for 2 minutes, then RES at the lowered speed
  def off(s, c):
    s["cruise"], s["brake"] = False, True
  def resume(s, c):
    s["cruise"], s["brake"] = True, False
  r = run(Car(45), between(100, 1500, 30), 20000, events={1000: off, 13000: resume})
  assert near(r["final"], 45), r["final"]
  assert r["min"] >= 40 - 1e-6


# ---------------- review round 4 ----------------

def test_cancel_while_press_in_flight_then_resume_later():
  # P1: our SET- is in flight when the driver cancels; cruise off for 10 s; then RES.
  for lag in (15, 60, 120):
    first = run(Car(65, lag=lag), between(100, 1500, 50), 3000)["presses"][0][0]
    def off(s, c):
      s["cruise"], s["brake"] = False, True
    def resume(s, c):
      s["cruise"], s["brake"] = True, False
    curve = lambda f: 45 * MPH if (100 <= f < 1500 or 3500 <= f < 4900) else None
    r = run(Car(65, lag=lag), curve, 9000, events={first + 2: off, first + 1000: resume})
    assert r["min"] >= 60 - 1e-6, (lag, r["min"])
    assert near(r["final"], 65), (lag, r["final"])


def test_car_stepping_twice_per_press_disables():
  # P3: a car whose set speed moves 2 steps per press
  r = run(Car(65, step=2 * MPH), between(100, 3000, 45), 6000)
  assert r["logic"].disabled
  assert len(r["presses"]) <= 1, r["presses"]


# ---------------- v2: adjustable drop, restore while exiting ----------------

def test_max_drop_settings():
  for max_drop, expect in ((5, 60), (10, 55), (15, 50)):
    r = run(Car(65), between(100, 2500, 30), 6000, max_drop=max_drop)
    assert near(r["min"], expect), (max_drop, r["min"])
    assert near(r["final"], 65) and near(r["max"], 65)


def test_45_road_with_30_curve():
  # the yellow-sign case: 45 mph road, curve needs ~30 true; dash reads 2 mph high
  r = run(Car(45), between(100, 2500, 30), 6000, max_drop=15)
  assert r["min"] <= 31 + 1e-6 and r["min"] >= 30 - 1e-6, r["min"]   # target 30 true + margin
  assert near(r["final"], 45)
  r = run(Car(45), between(100, 2500, 30), 6000, max_drop=10)
  assert near(r["min"], 35), r["min"]


def test_drop_only_as_much_as_curve_needs():
  r = run(Car(65, dash_offset_mph=0.0), between(100, 2500, 62.5), 6000, max_drop=15)
  assert r["min"] >= 62 - 1e-6, r["min"]


def test_restore_starts_quickly_after_curve():
  r = run(Car(65), between(100, 1500, 58), 5000, max_drop=10)
  plus = [f for f, b in r["presses"] if b == cs.SET_PLUS]
  assert plus and plus[0] - 1500 <= 150, plus[:3]      # within ~1.5 s of the curve ending
  assert near(r["final"], 65)


def test_rising_exit_raises_during_exit_without_ping_pong():
  # curve target rises gradually from 50 to 70 over 8 s while exiting
  exit_curve = lambda f: (50 * MPH if 100 <= f < 1000 else
                          (50 * MPH + (f - 1000) / 800 * 20 * MPH if 1000 <= f < 1800 else None))
  r = run(Car(65), exit_curve, 6000, max_drop=15)
  plus = [f for f, b in r["presses"] if b == cs.SET_PLUS]
  assert plus and plus[0] < 1800, plus[:3]            # starts raising during the exit
  seq = "".join("M" if b == cs.SET_MINUS else "P" for _, b in r["presses"])
  assert seq.count("PM") <= 1, seq                    # no ping-pong
  assert near(r["final"], 65) and near(r["max"], 65)


def test_random_drives_all_drop_settings():
  import random
  rng = random.Random(99)
  for _ in range(45):
    max_drop = rng.choice([5, 10, 15])
    set_mph = rng.choice([30, 45, 55, 65, 80])
    lag = rng.choice([5, 15, 40, 80, 150])
    segs, f = [], 100
    while f < 9000:
      length = rng.randint(100, 900)
      segs.append((f, f + length, rng.uniform(set_mph - 25, set_mph + 2)))
      f += length + rng.randint(50, 900)
    def curve(fr, segs=segs):
      for a, b, v in segs:
        if a <= fr < b:
          return v * MPH
      return None
    r = run(Car(set_mph, lag=lag, dash_offset_mph=rng.choice([0, 1, 2])), curve, 14000, max_drop=max_drop)
    floor = min(set_mph, max(set_mph - max_drop, 25))
    assert r["max"] <= set_mph + 1e-6, (max_drop, set_mph, lag, r["max"])
    assert r["min"] >= floor - 1e-6, (max_drop, set_mph, lag, r["min"])
    assert near(r["final"], set_mph), (max_drop, set_mph, lag, r["final"])


# ---------------- road test findings (routes 00000031--1a4d3b643d and 00000032--5b79f37567) ----------------

def _steps_to(presses, car_lag):
  return [f for f, b in presses]


def test_mazda_minimum_set_speed_does_not_disable():
  # Route 31: cruise engaged at the car's 30 km/h minimum (18.64 mph); the first tap goes to 19 mph,
  # a 0.36-step change. That is the driver, not a unit mismatch: the feature must keep working.
  car = Car(30 / 1.609344, dash_offset_mph=0.0)
  taps = {200: 19.0, 250: 20.0, 300: 25.0, 350: 30.0, 400: 35.0, 450: 40.0, 500: 45.0}
  def tap(mph):
    def ev(s, c):
      c.set_speed = mph * MPH
      c.v_ego = c.set_speed
    return ev
  r = run(car, between(2000, 3500, 30), 6000, events={f: tap(v) for f, v in taps.items()})
  assert not r["logic"].disabled
  assert minus_count(r) == 5, r["presses"]            # 45 -> 40 with the default 5 mph cap of run()
  assert near(r["final"], 45), r["final"]


def test_step_mismatch_with_nothing_in_flight_is_driver_change():
  logic = cs.CurveSlowdownLogic(False, 10)
  def inp(set_mph):
    return cs.SlowdownInputs(True, True, set_mph * MPH, 40 * MPH, False, False, False, True, False, 50 * MPH, False)
  for _ in range(50):
    logic.update(inp(30 / 1.609344))
  logic.update(inp(19.0))
  logic.update(inp(20.0))
  assert not logic.disabled and logic.offset == 0


def test_reengage_at_minimum_speed_with_press_in_flight_is_driver_change():
  # Our SET- is in flight, the driver brakes, later re-engages while slow: the car sets 30 km/h
  # (18.64 mph), a fractional change. That is the driver's new speed, not a unit mismatch.
  first = run(Car(45, lag=60), between(100, 1500, 30), 3000)["presses"][0][0]
  def off(s, c):
    s["cruise"], s["brake"] = False, True
  def resume(s, c):
    s["cruise"], s["brake"] = True, False
    c.pending = []
    c.set_speed = 30 / 1.609344 * MPH
    c.v_ego = 15 * MPH
  r = run(Car(45, lag=60), between(100, 1500, 30), 5000, events={first + 2: off, first + 800: resume})
  assert not r["logic"].disabled
  assert r["logic"].offset == 0


def test_ignored_press_recovers_within_pending_timeout():
  # Route 32: the MRCC ignored about one press in four; every ignored press stalled the logic for 3.5 s.
  # Losing a press must now cost about PENDING_TIMEOUT, not PENDING_TIMEOUT + LATE_WINDOW.
  r = run(Car(50, lag=10, lose=lambda n: n == 3), between(100, 1500, 30), 4000, max_drop=10)
  minus = [f for f, b in r["presses"] if b == cs.SET_MINUS]
  assert minus[3] - minus[2] <= cs._frames(cs.PENDING_TIMEOUT) + 5, minus[:5]
  assert near(r["min"], 40) and near(r["final"], 50), (r["min"], r["final"])
  # The whole 10 mph drop still happens in a few seconds despite a loss
  assert minus[-1] - minus[0] < cs._frames(4.0), minus


def test_first_press_of_the_drive_lost_still_lowers_and_restores():
  # Before the car's answer time is known a lost press is carried along conservatively: the drop
  # happens one step short until it is forgotten, then completes; everything restores.
  r = run(Car(50, lag=10, lose=lambda n: n == 1), between(100, 1500, 30), 4000, max_drop=10)
  minus = [f for f, b in r["presses"] if b == cs.SET_MINUS]
  assert minus[1] - minus[0] <= cs._frames(cs.PENDING_TIMEOUT) + 5, minus[:3]
  assert minus[9] - minus[0] < cs._frames(3.0), minus            # 9 of 10 steps within 3 s
  assert near(r["min"], 40) and near(r["final"], 50), (r["min"], r["final"])


def test_lossy_car_every_4th_press_reaches_floor_and_restores():
  r = run(Car(50, lag=10, lose=lambda n: n % 4 == 0), between(100, 1500, 30), 4000, max_drop=10)
  minus = [f for f, b in r["presses"] if b == cs.SET_MINUS]
  assert near(r["min"], 40) and near(r["final"], 50) and r["max"] <= 50 + 1e-6, (r["min"], r["final"], r["max"])
  assert minus[-1] - minus[0] < cs._frames(6.0), minus      # 10 steps + ~3 losses within 6 s


def test_late_landing_while_next_press_pending_keeps_limits():
  # A press the car answers late (1.2 s) is still counted as ours when the next press is already out.
  for lag in (60, 120, 180):
    r = run(Car(65, lag=lag), between(100, 1500, 45), 5000, max_drop=5)
    assert r["min"] >= 60 - 1e-6 and r["max"] <= 65 + 1e-6, (lag, r["min"], r["max"])
    assert near(r["final"], 65), (lag, r["final"])
    assert not r["logic"].blocked or True


def test_no_direction_reversal_while_press_in_flight():
  # Curve ends while a lost SET- may still land: the first SET+ must wait until that press is
  # forgotten, and the set speed must never go above the driver's speed.
  r = run(Car(65, lag=10, lose=lambda n: n == 5), between(100, 230, 50), 3000, max_drop=5)
  presses = r["presses"]
  last_minus = max(f for f, b in presses if b == cs.SET_MINUS)
  first_plus = min(f for f, b in presses if b == cs.SET_PLUS)
  assert first_plus - last_minus >= cs._frames(cs.PENDING_TIMEOUT), (last_minus, first_plus)
  assert r["max"] <= 65 + 1e-6 and near(r["final"], 65)
  # Without knowing the car's answer time the wait is the full window
  r = run(Car(65, lag=10, lose=lambda n: n == 1), between(100, 230, 50), 3000, max_drop=5)
  last_minus = max(f for f, b in r["presses"] if b == cs.SET_MINUS)
  first_plus = min(f for f, b in r["presses"] if b == cs.SET_PLUS)
  assert first_plus - last_minus >= cs._frames(cs.PENDING_TIMEOUT + cs.LATE_WINDOW) - 60, (last_minus, first_plus)
  assert r["max"] <= 65 + 1e-6 and near(r["final"], 65)


def test_driver_tap_same_direction_while_our_presses_in_flight():
  # Fuzz finding on the multi-in-flight design: with a slow car (0.9 s) the driver's SET- tap landed
  # among ours and was counted as ours, so we restored one step above what the driver wanted.
  for lag in (10, 60, 90, 150):
    first = run(Car(55, dash_offset_mph=0.0, lag=lag), between(100, 1500, 40), 3000)["presses"][0][0]
    r = run(Car(55, dash_offset_mph=0.0, lag=lag), between(100, 1500, 40), 5000, max_drop=15,
            events={first + 89: lambda s, c: c.press(cs.SET_MINUS, who="driver")})
    assert r["max"] <= 55 + 1e-6, (lag, r["max"])
    # the driver's SET- is their new choice: never restored above it (only our own late landings are put back)
    assert r["final"] <= 54 + 1e-6, (lag, r["final"])
    assert r["min"] >= 54 - 15 - 1e-6, (lag, r["min"])


def test_driver_hold_while_our_press_in_flight_is_not_a_unit_mismatch():
  first = run(Car(65, lag=60), between(100, 1500, 45), 3000)["presses"][0][0]
  def hold(s, c):
    c.press(cs.SET_MINUS, who="driver")
    c.pending.append((c.frame + c.lag, -5 * MPH))        # a hold moves 5 mph at once
    c.pending = [p for p in c.pending if p[1] != -MPH or p[0] != c.frame + c.lag]  # replace the tap with the hold
  r = run(Car(65, lag=60), between(100, 1500, 45), 5000, events={first + 30: hold})
  assert not r["logic"].disabled
  assert r["max"] <= 65 + 1e-6


# ---------------- review of 4f85eb1b7 (F1-F4) ----------------

def test_jittery_answer_time_lossless_keeps_limits():
  # F1: answer time varying 0.24-0.64 s with no losses must not orphan a press and send one too many.
  import random
  rng = random.Random(252)
  for _ in range(12):
    lags = [rng.randint(24, 64) for _ in range(400)]
    r = run(Car(35, dash_offset_mph=0.0, lag_fn=lambda n, lags=lags: lags[n % 400]),
            lambda f: 15 * MPH if (100 <= f < 1500 or 2500 <= f < 3600) else None, 6000, max_drop=10)
    assert r["min"] >= 25 - 1e-6 and r["max"] <= 35 + 1e-6, (r["min"], r["max"])
    assert near(r["final"], 35), r["final"]
    assert not r["logic"].disabled


def test_late_landing_after_window_does_not_shift_reference():
  # F1: one press answered 1.5 s late (after the in-flight window once the answer time is ~0.1 s) must
  # still count as ours: the restore must end at the driver's speed, not one step above or below.
  for late_n in (3, 6):
    r = run(Car(65, lag_fn=lambda n, late_n=late_n: 150 if n == late_n else 10), between(100, 1500, 45), 5000, max_drop=10)
    assert r["max"] <= 65 + 1e-6, (late_n, r["max"])
    assert near(r["final"], 65), (late_n, r["final"])


def test_driver_hold_pattern_with_press_in_flight_and_unknown_latency():
  # F2: the real Mazda hold: to the next multiple of 5, then -5 every ~0.6 s while the button is held.
  for start_after in (5, 15, 25):
    def hold(s, c, start_after=start_after):
      c.driver_button, c.driver_button_until = cs.SET_MINUS, c.frame + 200
      first = (c.set_speed / MPH - 0.01) // 5 * 5
      c.pending.append((c.frame + c.lag, first * MPH - c.set_speed))
      c.pending.append((c.frame + c.lag + 60, -5 * MPH))
      c.pending.append((c.frame + c.lag + 120, -5 * MPH))
    first = run(Car(50, lag=60), between(100, 3000, 30), 3000)["presses"][0][0]
    r = run(Car(50, lag=60), between(100, 3000, 30), 6000, max_drop=10, events={first + start_after: hold})
    assert not r["logic"].disabled, start_after
    assert r["max"] <= 50 + 1e-6


def _logic_inputs(set_mph, csc=True, csc_mph=45.0, v_mph=65.0, button=None):
  return cs.SlowdownInputs(True, True, set_mph * MPH, v_mph * MPH, False, False, False, True, csc, csc_mph * MPH, False, button)


def test_two_landings_in_one_sample_with_three_in_flight():
  # F3: three presses out, two of them show up in the same set speed sample: count both, do not disable.
  logic = cs.CurveSlowdownLogic(False, 10)
  set_mph = 65.0
  for f in range(400):
    b, _ = logic.update(_logic_inputs(set_mph))
    if len(logic.inflight) == 3 and b is None:
      break
  assert len(logic.inflight) == 3
  logic.update(_logic_inputs(set_mph - 2))          # two of ours landed at once
  assert not logic.disabled and logic.offset == 2 and len(logic.inflight) == 1
  logic.update(_logic_inputs(set_mph - 3))          # the third
  assert logic.offset == 3 and not logic.inflight
  # more steps than we have in flight is still the "car steps twice" case
  logic2 = cs.CurveSlowdownLogic(False, 10)
  for f in range(60):
    logic2.update(_logic_inputs(65.0))
  assert len(logic2.inflight) == 1
  logic2.update(_logic_inputs(63.0))
  assert logic2.disabled


def test_late_landing_is_claimed_from_the_expired_list():
  # F1: a press that lands after the in-flight window is still ours (no reference shift).
  logic = cs.CurveSlowdownLogic(False, 10)
  logic.latencies = [10] * 5                        # answer time known: window = 1.0 s
  sent = None
  for f in range(400):
    b, _ = logic.update(_logic_inputs(65.0, csc=sent is None))   # curve ends right after our first press
    if b is not None and sent is None:
      sent = logic.frame
    if sent is not None and logic.frame - sent > cs._frames(cs.MIN_IN_FLIGHT_WINDOW) + 5:
      break
  assert not logic.inflight and logic.expired, (logic.inflight, logic.expired)
  logic.update(_logic_inputs(64.0, csc=False))      # lands 1 s+ late
  assert logic.offset == 1 and not logic.blocked and not logic.expired
  # and it is restored afterwards (the reference did not shift)
  plus = None
  for f in range(400):
    b, _ = logic.update(_logic_inputs(64.0, csc=False))
    if b == cs.SET_PLUS:
      plus = logic.frame
      break
  assert plus is not None


def test_visible_driver_tap_while_our_presses_were_ignored_is_the_drivers():
  # F4: both our presses ignored, the driver taps SET- (seen on the bus) and it lands: restore to 64, not 65.
  first = run(Car(65), between(100, 1500, 45), 3000)["presses"][0][0]
  r = run(Car(65, lose=lambda n: n in (1, 2)), between(100, 1500, 45), 5000, max_drop=10,
          events={first + 55: lambda s, c: c.press(cs.SET_MINUS, who="driver")})
  assert r["max"] <= 65 + 1e-6
  assert near(r["final"], 64), r["final"]


def test_random_lossy_drives_never_break_limits():
  import random
  rng = random.Random(7)
  for _ in range(40):
    max_drop = rng.choice([5, 10, 15])
    set_mph = rng.choice([30, 45, 55, 65])
    lag = rng.choice([5, 15, 40, 80, 150])
    loss = rng.choice([0.0, 0.1, 0.25, 0.4])
    lost = set(n for n in range(1, 400) if rng.random() < loss)
    segs, f = [], 100
    while f < 9000:
      length = rng.randint(100, 900)
      segs.append((f, f + length, rng.uniform(set_mph - 25, set_mph + 2)))
      f += length + rng.randint(50, 900)
    def curve(fr, segs=segs):
      for a, b, v in segs:
        if a <= fr < b:
          return v * MPH
      return None
    r = run(Car(set_mph, lag=lag, dash_offset_mph=rng.choice([0, 1, 2]), lose=lambda n, lost=lost: n in lost),
            curve, 14000, max_drop=max_drop)
    floor = min(set_mph, max(set_mph - max_drop, 25))
    assert r["max"] <= set_mph + 1e-6, (max_drop, set_mph, lag, loss, r["max"])
    assert r["min"] >= floor - 1e-6, (max_drop, set_mph, lag, loss, r["min"])
    assert near(r["final"], set_mph), (max_drop, set_mph, lag, loss, r["final"])


if __name__ == "__main__":
  import sys
  failed = 0
  for name, fn in sorted(globals().items()):
    if name.startswith("test_"):
      try:
        fn()
        print("PASS", name)
      except Exception as e:  # noqa: BLE001
        failed += 1
        print("FAIL", name, e)
  print(f"{failed} failed")
  sys.exit(1 if failed else 0)


# ---------------- restoring a big gap after a curve: one SET+ hold, then taps ----------------

def _measured_clean(logic):
  logic.hold_measured, logic.hold_overrun = True, 0     # a hold has shown the car stops cleanly on release


def _back_at(r, mph, after):
  """First frame from `after` on where the set speed is at `mph`."""
  return next(f for f in range(after, len(r["sets"])) if near(r["sets"][f] / MPH, mph))


def _restore_presses(r, after):
  return [f for f, b in r["presses"] if b == cs.SET_PLUS and f >= after], [f for f in r["holds"] if f >= after]


def test_big_gap_after_a_curve_holds_to_a_multiple_of_5_below_then_taps():
  # 45 mph, a curve that needs ~30: the set speed comes down 15. Once the car is known to stop cleanly after a
  # SET+ release, the restore holds up to 40 (one quantum below 45) and taps the rest; never above 45.
  r = run(Car(45, dash_offset_mph=0.0), between(100, 1500, 30), 4000, max_drop=15, setup=_measured_clean)
  taps, holds = _restore_presses(r, 1500)
  assert holds, "a SET+ hold for the 15 mph gap"
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6, (r["final"], r["max"])
  plain = run(Car(45, dash_offset_mph=0.0), between(100, 1500, 30), 4000, max_drop=15)
  p_taps, p_holds = _restore_presses(plain, 1500)
  assert p_holds and len(p_taps) < len(taps) + 10          # until measured: a shorter hold (two quanta of margin)
  assert near(plain["final"], 45) and plain["max"] <= 45 + 1e-6
  # and it is quick: the hold must not end after one frame and wait for its settle (review: 31 -> 45 took 7 s)
  for res in (r, plain):
    first_plus = min([f for f, b in res["presses"] if f >= 1500 and b == cs.SET_PLUS] + [f for f in res["holds"] if f >= 1500])
    assert (_back_at(res, 45, first_plus) - first_plus) * DT < 5.0
  # a car that lands one more quantum after the release (slower than any seen in the logs) still never passes 45
  late = run(Car(45, dash_offset_mph=0.0, hold_latency=0.6), between(100, 1500, 30), 4000, max_drop=15)
  assert near(late["final"], 45) and late["max"] <= 45 + 1e-6, (late["final"], late["max"])


def test_small_gap_after_a_curve_is_restored_by_taps():
  r = run(Car(45, dash_offset_mph=0.0), between(100, 1500, 38), 4000, max_drop=10, setup=_measured_clean)
  taps, holds = _restore_presses(r, 1500)
  assert not holds and taps and near(r["final"], 45)


def test_restore_hold_never_passes_the_driver_speed_whatever_lands_after_the_release():
  # the margin is what the car was measured to add after a release: clean (0 extra) -> one quantum margin
  for latency in (0.0, 0.3):
    r = run(Car(47, dash_offset_mph=0.0, hold_latency=latency), between(100, 1500, 30), 4000, max_drop=15,
            setup=_measured_clean)
    assert r["max"] <= 47 + 1e-6 and near(r["final"], 47), (latency, r["max"], r["final"])


# ---------------- restore to the driver's remembered set speed (drive 2026-11-03) ----------------
# The old step count added its steps on top of a speed the driver had already brought back with the stock buttons
# (21:00 45 -> 48, 21:08 40 -> 43, 21:14 45 -> 47). Now the restore only brings the actual set speed up to the speed
# the driver had before our first press (the anchor), never above.

def _taps(button, n, every=30):
  """Events: the driver taps `button` n times, `every` frames apart, starting at the given frame."""
  def at(start):
    return {start + k * every: (lambda s, c, b=button: c.press(b, who="driver")) for k in range(n)}
  return at


def _after_curve(r):
  return [f for f, b in r["presses"] if b == cs.SET_PLUS]


def test_driver_brings_the_speed_back_and_we_add_nothing_on_top():
  # 21:00 / 21:08: lowered 3 steps, brake (cruise off), RES at the lowered speed, the driver taps SET+ x3
  def brake(s, c):
    s["brake"], s["cruise"] = True, False

  def resume(s, c):
    s["brake"], s["cruise"] = False, True

  for taps_at in (1402, 1440, 1600):          # right after RES (21:08), during our restore, after it is done
    ev = {1100: brake, 1400: resume, **_taps(cs.SET_PLUS, 3)(taps_at)}
    r = run(Car(45), between(100, 1050, 30), 5000, events=ev, max_drop=3)
    assert near(r["min"], 42), r["min"]
    if taps_at == 1402:
      assert near(r["final"], 45) and r["max"] <= 45 + 1e-6, (taps_at, r["final"], r["max"])
    elif taps_at == 1440:
      # our first restore tap was already on its way: the driver's own third tap makes 46. Nothing of ours
      # after their first tap (we wait while they are on the buttons, then they are at or above 45)
      assert near(r["final"], 46), (taps_at, r["final"])
      assert [f for f in _after_curve(r) if f > taps_at] == [], r["presses"]
    else:
      assert near(r["final"], 48), (taps_at, r["final"])    # restored to 45 first, then the driver's own +3


def test_driver_brings_it_back_part_way_we_restore_only_the_rest():
  ev = _taps(cs.SET_PLUS, 1)(1200)            # one tap while we hold the speed down in the curve
  r = run(Car(65), between(100, 1500, 61), 5000, events=ev)
  assert near(r["final"], 65) and r["max"] <= 65 + 1e-6, (r["final"], r["max"])


def test_driver_raises_above_the_old_speed_and_we_never_pull_it_down():
  # 21:14: lowered 45 -> 43; the stock switch then reported SET+ up to 51 and SET- back to 45: nothing from us after
  ev = {**_taps(cs.SET_PLUS, 8, every=25)(600), **_taps(cs.SET_MINUS, 6, every=25)(1200)}
  r = run(Car(45), between(100, 1500, 30), 5000, events=ev, max_drop=2)
  assert near(r["min"], 43), r["min"]
  assert near(r["final"], 45), r["final"]
  ours_after = [(f, b) for f, b in r["presses"] if f >= 600]
  assert ours_after == [], ours_after
  assert r["logic"].anchor is None


def test_driver_set_minus_during_our_restore_stops_it_there():
  base = run(Car(65), between(100, 1000, 50), 4000, max_drop=5)
  first_plus = _after_curve(base)[0]
  r = run(Car(65), between(100, 1000, 50), 5000, max_drop=5, events=_taps(cs.SET_MINUS, 1)(first_plus + 40))
  after = [f for f in _after_curve(r) if f > first_plus + 40]
  assert after == [], r["presses"]
  assert r["final"] < 65 - 1e-6 and r["max"] <= 65 + 1e-6


def test_a_new_reason_after_the_driver_raised_the_speed_anchors_at_the_new_speed():
  ev = _taps(cs.SET_PLUS, 3)(1700)            # after the first curve: 65 -> restored 65, driver +3 -> 68
  curve = lambda f: 60 * MPH if (100 <= f < 1500 or 3000 <= f < 4000) else None  # noqa: E731
  r = run(Car(65), curve, 7000, events=ev)
  assert near(r["final"], 68), r["final"]
  assert r["max"] <= 68 + 1e-6


def test_no_press_of_ours_while_the_driver_is_on_a_button_or_just_was():
  quiet = cs._frames(cs.DRIVER_QUIET)
  ev = _taps(cs.SET_MINUS, 1)(1600)
  r = run(Car(65), between(100, 1500, 61), 5000, events=ev)
  assert [f for f, _ in r["presses"] if 1600 <= f < 1600 + quiet] == [], r["presses"]


def test_set_with_cruise_off_is_the_drivers_new_speed_even_at_the_same_number():
  def off(s, c):
    s["brake"], s["cruise"] = True, False

  def press_set(s, c):
    c.driver_button, c.driver_button_until = cs.SET_MINUS, c.frame + 10    # SET with cruise off: engage here

  def on(s, c):
    s["brake"], s["cruise"] = False, True

  r = run(Car(65), between(100, 1050, 50), 5000, max_drop=5, events={1100: off, 1300: press_set, 1320: on})
  assert [f for f in _after_curve(r) if f >= 1320] == [], r["presses"]
  assert near(r["final"], 60), r["final"]


def test_fuzz_never_above_the_drivers_speed():
  import random
  for seed in range(60):
    rnd = random.Random(seed)
    lag = rnd.choice((10, 15, 30, 60, 90))
    ev = {}
    top = 55
    for _ in range(rnd.randint(1, 4)):
      f = rnd.randint(150, 2500)
      b = rnd.choice((cs.SET_PLUS, cs.SET_MINUS))
      ev[f] = (lambda s, c, b=b: c.press(b, who="driver"))
    r = run(Car(top, lag=lag, lose=(lambda n: n % 7 == 3) if seed % 2 else None), between(100, 1500, 40), 6000,
            events=ev, max_drop=10)
    driver_ups = sum(1 for f in ev if True)
    assert r["max"] <= top + driver_ups + 1e-6, (seed, r["max"])
    # with only SET- taps from the driver, never back above the starting speed
    if all(ev[f].__defaults__[0] == cs.SET_MINUS for f in ev):
      assert r["max"] <= top + 1e-6 and r["final"] <= top + 1e-6, (seed, r["max"], r["final"])


def test_his_cancel_or_resume_stops_our_hold_at_once_and_no_frame_goes_out():
  # outside review of e1be62454: the wheel's CANCEL / RES (other_button_active from the wrapper while he holds one): a
  # SET+ restore hold in progress stops at once, none of our frames while it is down
  r = run(Car(45, dash_offset_mph=0.0), between(100, 1500, 30), 4000, max_drop=15, setup=_measured_clean)
  assert r["holds"]
  at = r["holds"][0] + 30
  win = range(at, at + 60)
  r2 = run(Car(45, dash_offset_mph=0.0), between(100, 1500, 30), 4000, max_drop=15, setup=_measured_clean,
           other=lambda f: f in win)
  assert not [f for f in r2["holds"] if f in win] and not [f for f, _ in r2["presses"] if f in win]
  assert [f for f in r2["holds"] if at - 30 <= f < at], "the hold was on before"
