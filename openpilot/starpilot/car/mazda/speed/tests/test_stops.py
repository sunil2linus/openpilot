"""The stop slowdown (part G) with the planner (tests/rig.py), and through the actuator for the presses: a stopped car
at 60 / 100 / 150 m, the lead pulling away, the short model path's confirmation, the far guess guard, a false flag
at 53 mph, the dashboard stop sign, his RES / SET+ / gas, test mode never presses, off = before. Not a drive
simulation: each test checks one rule."""
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed import stops as S
from openpilot.starpilot.car.mazda.speed.actuator import CAR_MIN_SET
from openpilot.starpilot.car.mazda.speed.caller import SpeedControl
from openpilot.starpilot.car.mazda.speed.contributors import build
from openpilot.starpilot.car.mazda.speed.tests.mrcc_sim import Mrcc
from openpilot.starpilot.car.mazda.speed.tests.rig import Rig, driver, engaged

MPH = M.MPH
CFG = C.Config(enabled=True, stops="on")


def path(length):
  return ([0.0, length / 2, length], [0.0, 0.0, 0.0])


def lead(d, v=0.0, vrel=None, prob=0.9):
  return lambda t: {"status": True, "d": d(t) if callable(d) else d, "v": v,
                    "vrel": vrel if vrel is not None else v - 20.0, "prob": prob, "t": t}


def rig(presses=True, set_mph=45):
  st = S.StopSlowdown(CFG, presses=presses)
  r = Rig([st])
  r.step(set_mph, set_mph, events=[engaged(set_mph)])
  return r, st


def cmd_mph(r):
  return None if r.out.command is None else r.out.command / MPH


# ---------------- G1: a stopped car ----------------
def test_stopped_lead_150_m_from_45_mph_distance_target_then_100_m_and_60_m_the_floor():
  for d, floor in ((150.0, False), (100.0, True), (60.0, True)):
    r, st = rig()
    r.step(45, n=40, lead=lead(d), path=path(300), model_age=0.0)        # 0.4 s: not yet
    assert r.out.reason == "" and st.active is None, d
    r.step(45, n=15, lead=lead(d), path=path(300), model_age=0.0)        # 0.55 s: confirmed
    assert r.out.reason == "stop" and st.active["lead"], d
    if floor:
      assert abs(r.out.target - S.FLOOR_TRUE) < 1e-6, d                   # straight to the floor (one hold)
    else:
      assert S.FLOOR_TRUE + 3 * MPH < r.out.target < 35 * MPH, (d, r.out.target / MPH)
      assert cmd_mph(r) < 40
    assert r.out.target >= S.FLOOR_TRUE - 1e-9


def test_stopped_car_needs_closing_a_confident_lead_and_the_lead_bit_is_not_required():
  r, st = rig()
  r.step(45, n=100, lead=lead(100.0, vrel=0.0), path=path(300), model_age=0.0)     # not closing
  assert st.active is None
  r.step(45, n=100, lead=lead(100.0, prob=0.5), path=path(300), model_age=0.0)     # unsure camera lead
  assert st.active is None
  r.step(45, n=100, lead=lead(100.0, prob=0.5), radar_lead=True, path=path(300), model_age=0.0)  # + the stock bit
  assert st.active is not None
  r2, st2 = rig()
  r2.step(45, n=100, lead=lead(100.0), radar_lead=False, path=path(300), model_age=0.0)        # no stock bit
  assert st2.active is not None


def test_a_distance_jump_is_another_car_and_starts_the_window_again():
  r, st = rig()
  d = {"x": 120.0}
  r.step(45, n=40, lead=lead(lambda t: d["x"]), path=path(300), model_age=0.0)
  d["x"] = 60.0
  r.step(45, n=40, lead=lead(lambda t: d["x"]), path=path(300), model_age=0.0)
  assert st.active is None                    # 0.4 s with the new car
  r.step(45, n=15, lead=lead(lambda t: d["x"]), path=path(300), model_age=0.0)
  assert st.active is not None


def test_lead_pulling_away_releases_and_the_planner_restores():
  r, st = rig()
  r.step(45, n=60, lead=lead(60.0), path=path(300), model_age=0.0)
  assert r.out.reason == "stop"
  r.step(25, n=30, lead=lead(55.0, v=2.5, vrel=0.5), path=path(300), model_age=0.0)  # moving away < 0.5 s
  assert r.out.reason == "stop"
  r.step(25, n=30, lead=lead(55.0, v=2.5, vrel=0.5), path=path(300), model_age=0.0)
  assert st.active is None and r.out.reason == "" and abs(r.out.command - 45 * MPH) < 1e-6


def test_lead_dropout_keeps_the_target_and_a_lost_lead_releases_only_with_a_long_path():
  r, st = rig()
  r.step(45, n=60, lead=lead(60.0), path=path(300), model_age=0.0)
  t0 = r.out.target
  r.step(40, n=50, lead=None, path=path(40), model_age=0.0)       # lost, the model path short: kept
  assert r.out.reason == "stop" and r.out.target <= t0 + 1e-9
  r.step(40, n=250, lead=None, path=path(300), model_age=0.0)     # lost and the path long: released
  assert st.active is None and r.out.reason == ""


def test_the_target_only_goes_down_in_an_episode():
  r, st = rig()
  targets = []
  for d in (150.0, 145.0, 140.0, 200.0, 150.0):
    r.step(45, n=30, lead=lead(d, v=1.0), path=path(300), model_age=0.0)
    if r.out.target is not None:
      targets.append(r.out.target)
  assert targets == sorted(targets, reverse=True)


# ---------------- G2: the short model path ----------------
def test_the_stop_threshold_is_the_old_definition():
  assert abs(S.stop_threshold(30 * MPH) - 30 * MPH * 8.0) < 1e-6             # v x (PLANNER_TIME - 2)
  assert abs(S.stop_threshold(50 * MPH) - 50 * MPH * 9.0) < 1e-6             # boosted, capped at 9 s
  assert S.stop_threshold(40 * MPH) > 40 * MPH * 8.0


def test_short_path_half_a_second_does_nothing_one_second_acts():
  r, st = rig()
  r.step(35, n=50, path=path(60), model_age=0.0)
  assert r.out.reason == "" and st.active is None
  r.step(35, n=60, path=path(300), model_age=0.0)                 # long again
  r.step(35, n=99, path=path(60), model_age=0.0)
  assert st.active is None                                         # 0.99 s
  r.step(35, n=2, path=path(60), model_age=0.0)
  assert r.out.reason == "stop" and abs(r.out.target - S.FLOOR_TRUE) < 1e-6
  assert st.notice == "stop_ahead" or st.active["warned"] == {"stop_ahead"}


def test_short_path_must_be_continuous_and_a_lead_in_the_path_is_g1s_job():
  r, st = rig()
  for _ in range(5):
    r.step(35, n=80, path=path(60), model_age=0.0)
    r.step(35, n=5, path=path(300), model_age=0.0)
  assert st.active is None
  r.step(35, n=150, path=path(60), model_age=0.0, lead=lead(50.0, v=14.0, vrel=-1.0))   # following a moving car
  assert st.active is None


def test_path_long_again_for_one_second_releases():
  r, st = rig()
  r.step(35, n=120, path=path(60), model_age=0.0)
  assert r.out.reason == "stop"
  r.step(35, n=90, path=path(300), model_age=0.0)
  assert r.out.reason == "stop"
  r.step(35, n=15, path=path(300), model_age=0.0)
  assert st.active is None and r.out.reason == ""


def test_above_45_mph_and_far_only_guard_taps_then_the_full_target_when_near():
  r, st = rig(set_mph=55)
  r.step(55, 55, n=300, path=path(200), path_v=3.0, model_age=0.0)   # 3 s, flagged (a planned stop), 195 m away
  assert r.out.target is None and r.out.reason == "stop"
  assert 55 - 3 - 1e-6 <= cmd_mph(r) <= 54 + 1e-6                  # at most 3 mph of guard taps
  assert "stop_ahead" not in st.active["warned"]
  r.step(55, n=20, path=path(120), path_v=2.0, model_age=0.0)     # near now: the full target
  assert r.out.target is not None and abs(r.out.target - S.FLOOR_TRUE) < 1e-6


def test_false_flag_at_53_mph_never_drops_the_set_more_than_3_mph_before_confirmation():
  for length, frames in ((150.0, 80), (150.0, 400), (120.0, 95)):
    r, st = rig(set_mph=53)
    r.step(53, 53, n=frames, path=path(length), model_age=0.0)
    low = cmd_mph(r)
    r.step(53, n=150, path=path(400), model_age=0.0)
    assert low >= 53 - 3 - 1e-6, (length, frames, low)
    assert r.out.reason == "" and abs(cmd_mph(r) - 53) < 1e-6


def test_never_above_75_mph_and_not_in_a_slow_turn():
  r, st = rig(set_mph=76)
  r.step(76, 76, n=300, path=path(60), model_age=0.0)
  assert st.active is None
  r2, st2 = rig()
  r2.step(12, 45, n=300, path=path(20), model_age=0.0, angle=90.0)
  assert st2.active is None


# ---------------- G3: the dashboard stop sign ----------------
def test_dashboard_stop_sign_shortens_the_confirmation_but_never_acts_alone():
  r, st = rig()
  stamp = {"n": 0}

  def sign(t):
    stamp["n"] += 1
    return (True, stamp["n"])
  r.step(35, n=300, path=path(300), model_age=0.0, stop_sign_raw=sign)   # a sign, a long path: nothing
  assert st.active is None
  r.step(35, n=55, path=path(60), model_age=0.0, stop_sign_raw=sign)     # 0.55 s with the sign: acts
  assert r.out.reason == "stop"
  r2, st2 = rig()
  r2.step(35, n=55, path=path(60), model_age=0.0, stop_sign_raw=(False, 5))
  assert st2.active is None


# ---------------- his buttons ----------------
def test_his_set_plus_cancels_the_episode_and_gas_held_declines_it():
  r, st = rig()
  r.step(45, n=60, lead=lead(100.0), path=path(300), model_age=0.0)
  assert r.out.reason == "stop"
  r.step(45, 40, events=[driver("SET_PLUS", 41)], lead=lead(95.0), path=path(300), model_age=0.0)
  r.step(45, n=50, lead=lead(90.0), path=path(300), model_age=0.0)
  assert st.active is None and r.out.reason != "stop"
  r2, st2 = rig()
  r2.step(45, n=60, lead=lead(100.0), path=path(300), model_age=0.0)
  r2.step(45, n=110, lead=lead(95.0), path=path(300), model_age=0.0, gas=True)
  assert st2.active is None and st2.declined


def res_event():
  return {"kind": "driver_button", "button": "RES"}


def test_his_res_with_the_set_low_raises_at_once_but_step_by_step_toward_a_closing_lead():
  r, st = rig()
  r.step(45, n=60, lead=lead(60.0), path=path(300), model_age=0.0)
  r.step(12, 19, n=10, lead=lead(10.0, v=0.0, vrel=-1.0), path=path(300), model_age=0.0)
  # RES, the road clear: no stop wish, the planner raises to his speed (a hold, then taps: the actuator's own choice)
  r.step(12, 19, events=[res_event()], path=path(300), model_age=0.0)
  r.step(12, n=5, path=path(300), model_age=0.0)
  assert st.active is None and r.out.reason == "" and abs(r.out.command - 45 * MPH) < 1e-6
  # RES with a closing lead within 80 m: one step above the set at a time (taps, never a hold)
  r2, st2 = rig()
  r2.step(45, n=60, lead=lead(60.0), path=path(300), model_age=0.0)
  r2.step(12, 19, events=[res_event()], lead=lead(40.0, v=2.0, vrel=-3.0), path=path(300), model_age=0.0)
  r2.step(12, n=5, lead=lead(40.0, v=2.0, vrel=-3.0), path=path(300), model_age=0.0)
  assert r2.out.reason == "stop" and abs(r2.out.command - 20 * MPH) < 1e-6
  # ... and with a short model path
  r3, st3 = rig()
  r3.step(45, n=60, lead=lead(60.0), path=path(300), model_age=0.0)
  r3.step(12, 19, events=[res_event()], path=path(20), model_age=0.0)
  r3.step(12, n=5, path=path(20), model_age=0.0)
  assert abs(r3.out.command - 20 * MPH) < 1e-6


# ---------------- through the actuator ----------------
class FakeSources:
  want_view = False

  def __init__(self, extra):
    self.extra = extra

  def context(self, d):
    ctx = dict(d)
    ctx.update(self.extra(d))
    return ctx


def drive(cfg, frames, extra, car=None, every=None):
  st = S.StopSlowdown(cfg, presses=cfg.presses("stops"))
  car = car or Mrcc(45, latency=12, v_mph=45)
  ctl = SpeedControl(None, cfg, None, False, contributors=[st], sources=FakeSources(extra))
  sent = []
  for k in range(frames):
    car.tick()
    d = {"cruise_enabled": True, "main_on": True, "set_speed": car.set, "v_ego": car.v, "gas": False, "brake": False,
         "standstill": False, "set_plus": False, "set_minus": False, "resume": False, "cancel": False,
         "wheel_counter": car.counter, "other_button": False, "engaged": True, "lkas_blocked": False, "blinker": False,
         "radar_lead": False}
    if every is not None:
      every(k, d, car)
    r = ctl.step(d, now=k * 0.01)
    if r is not None:
      car.receive(*r)
      sent.append((k, r[0]))
  return sent, ctl, car, st


def stopped_car_ahead(d):
  t = d.get("_t", 0.0)
  return {"now": t, "lead": {"status": True, "d": 90.0, "v": 0.0, "vrel": -20.0, "prob": 0.9, "t": t},
          "path": path(300), "model_age": 0.0}


def every_t(k, d, car):
  d["_t"] = k * 0.01


def test_test_mode_never_presses_but_warns():
  sent, ctl, car, st = drive(C.Config(enabled=True, stops="test"), 800, stopped_car_ahead, every=every_t)
  assert sent == [] and abs(car.set_units - 45) < 1e-6
  assert st.n == 1 and "stopped_car" in st.active["warned"]


def test_on_presses_down_to_the_floor_for_a_stopped_car():
  sent, ctl, car, st = drive(CFG, 1500, stopped_car_ahead, every=every_t)
  assert sent and all(b == "SET_MINUS" for _, b in sent)
  assert abs(car.set - CAR_MIN_SET) < 0.5 * MPH                     # the floor (19 mph), never below


def test_res_restore_holds_without_a_closing_lead_and_only_taps_with_one():
  for closing in (False, True):
    state = {"res": None}

    def extra(d, closing=closing):
      t = d["_t"]
      if t < 12.0:
        return stopped_car_ahead(d)
      ld = {"status": True, "d": 40.0, "v": 2.0, "vrel": -3.0, "prob": 0.9, "t": t} if closing else None
      return {"now": t, "lead": ld, "path": path(300), "model_age": 0.0}

    def every(k, d, car, state=state):
      d["_t"] = k * 0.01
      if 1200 <= k < 1210:
        d["resume"] = True
        state["res"] = k
    holds = []
    car = Mrcc(45, latency=12, v_mph=45)
    st = S.StopSlowdown(CFG, presses=True)
    ctl = SpeedControl(None, CFG, None, False, contributors=[st], sources=FakeSources(extra))
    for k in range(2600):
      car.tick()
      if k >= 1200:
        car.v = 12 * MPH
      d = {"cruise_enabled": True, "main_on": True, "set_speed": car.set, "v_ego": car.v, "gas": False, "brake": False,
           "standstill": False, "set_plus": False, "set_minus": False, "resume": False, "cancel": False,
           "wheel_counter": car.counter, "other_button": False, "engaged": True, "lkas_blocked": False,
           "blinker": False, "radar_lead": False}
      every(k, d, car)
      r = ctl.step(d, now=k * 0.01)
      if r is not None:
        car.receive(*r)
      if k > 1210:
        holds += [e for e in ctl.events if e["kind"] == "hold_start" and e["direction"] == "SET_PLUS"]
    assert car.set_units > 19 + 1                                  # it went back up after his RES
    if closing:
      assert holds == []                                           # taps only, toward a closing lead
    else:
      assert holds and abs(car.set_units - 45) < 1e-6              # a hold, then taps: back to his speed


# ---------------- off ----------------
def test_off_builds_no_stop_slowdown_and_no_radar():
  out, src = build(C.Config(enabled=True, curves="test"), None)
  assert not any(isinstance(c, S.StopSlowdown) for c in out) and not src.want_radar
  out, src = build(C.Config(enabled=False, stops="on"), None)
  assert out == [] and src is None
  out, src = build(C.Config(enabled=True, stops="test"), None)
  assert [type(c).__name__ for c in out] == ["StopSlowdown"] and src.want_radar and not src.want_view
  assert out[0].presses is False


def test_res_that_re_engages_the_cruise_after_his_brake_still_restores_step_by_step():
  r, st = rig()
  r.step(45, n=60, lead=lead(60.0), path=path(300), model_age=0.0)
  r.step(10, 19, n=5, cruise=False, lead=lead(15.0, v=0.0, vrel=-1.0), path=path(300), model_age=0.0)   # he braked
  r.step(10, 19, events=[res_event()], cruise=False, lead=lead(30.0, v=2.0, vrel=-2.0), path=path(300), model_age=0.0)
  r.step(10, 19, n=20, cruise=False, lead=lead(30.0, v=2.0, vrel=-2.0), path=path(300), model_age=0.0)
  r.step(10, 19, n=5, lead=lead(30.0, v=2.0, vrel=-2.0), path=path(300), model_age=0.0)                 # back on
  assert st.restore is not None and r.out.reason == "stop" and abs(r.out.command - 20 * MPH) < 1e-6


# ---------------- review round (Neon) ----------------
def test_his_set_plus_during_the_step_by_step_restore_ends_it_and_the_set_is_not_frozen():
  r, st = rig()
  r.step(45, n=60, lead=lead(60.0), path=path(300), model_age=0.0)
  r.step(10, 19, n=5, cruise=False, lead=lead(15.0, v=0.0, vrel=-1.0), path=path(300), model_age=0.0)
  r.step(10, 19, events=[res_event()], cruise=False, lead=lead(30.0, v=2.0, vrel=-2.0), path=path(300), model_age=0.0)
  r.step(10, 19, n=20, cruise=False, lead=lead(30.0, v=2.0, vrel=-2.0), path=path(300), model_age=0.0)
  r.step(10, 19, n=5, lead=lead(30.0, v=2.0, vrel=-2.0), path=path(300), model_age=0.0)
  assert st.restore is not None and r.out.reason == "stop"
  r.step(10, 21, events=[driver("SET_PLUS", 21)], lead=lead(30.0, v=2.0, vrel=-2.0), path=path(300), model_age=0.0)
  r.step(10, n=50, lead=lead(30.0, v=2.0, vrel=-2.0), path=path(300), model_age=0.0)
  assert st.restore is None and "stop" not in r.p.blocked
  assert not r.out.reason.startswith("his choice") and cmd_mph(r) >= 21 - 1e-6


def test_a_dashboard_stop_sign_is_not_held_back_by_the_far_guard():
  stamp = {"n": 0}

  def sign(t):
    stamp["n"] += 1
    return (True, stamp["n"])
  r, st = rig(set_mph=55)
  r.step(55, 55, n=60, path=path(200), path_v=3.0, model_age=0.0, stop_sign_raw=sign)   # 0.6 s, 195 m, the sign
  assert r.out.reason == "stop" and r.out.target is not None                  # the full target, not guard taps
  r2, st2 = rig(set_mph=55)
  r2.step(55, 55, n=300, path=path(200), path_v=3.0, model_age=0.0)          # no sign: guard taps only (as before)
  assert r2.out.target is None


def test_the_guard_ceiling_is_never_below_the_cars_minimum_set():
  r, st = rig(set_mph=19)
  r.step(50, 19, n=400, path=path(200), model_age=0.0, gas=False)
  assert r.out.command is None or r.out.command >= CAR_MIN_SET - 1e-6


# ---------------- round 2 (item 7) ----------------
def test_round2_above_45_a_short_path_needs_the_models_end_speed_low():
  # (r2 / r3: 22 false 1-3 mph dips) the path ~8.6-9 s long at 50-53 mph with the model ending near our speed: no stop
  r, st = rig(set_mph=53)
  r.step(53, 53, n=400, path=path(205), path_v=46 * MPH, model_age=0.0)
  assert st.active is None and abs(cmd_mph(r) - 53) < 1e-6
  r.step(53, n=400, path=path(205), path_v=None, model_age=0.0)       # no end speed (schema): no far guard either
  assert st.active is None
  r.step(53, n=300, path=path(205), path_v=30 * MPH, model_age=0.0)   # a planned slowdown to 30: the guard taps
  assert st.active is not None and cmd_mph(r) < 53
  # at 45 mph and under the end speed is not needed (as before)
  r, st = rig(set_mph=40)
  r.step(40, 40, n=150, path=path(120), path_v=None, model_age=0.0)
  assert st.active is not None and st.active["stop"]


def test_round2_a_lead_at_our_speed_does_not_hide_a_stop_the_model_plans():
  # (r2 rt 416 / 901) the car ahead at our speed 45 m ahead, the model's path 58 m ending at 2 mph: a stop for both
  r, st = rig(set_mph=30)
  r.step(23, 30, n=150, path=path(58), path_v=0.9, model_age=0.0, lead=lead(45.0, v=21 * MPH, vrel=-0.5, prob=0.6))
  assert st.active is not None and st.active["stop"] and r.out.reason == "stop"
  assert st.notice == "stop_ahead" or "stop_ahead" in st.active["warned"]
  # the same short path ending at 20 mph behind a car at our speed: the lead's gap (G1's job), nothing
  r, st = rig(set_mph=30)
  r.step(23, 30, n=300, path=path(58), path_v=20 * MPH, model_age=0.0, lead=lead(45.0, v=21 * MPH, vrel=-0.5, prob=0.6))
  assert st.active is None


def test_round2_a_close_fast_closing_lead_needs_less_confidence_and_a_dropout_is_held():
  def flicker(t):          # a stopped car 50 m ahead at 0.55, missing for 0.2 s in the middle of the window
    if 0.25 < (t % 10.0) < 0.45:
      return None
    return {"status": True, "d": 50.0, "v": 0.5, "vrel": -11.0, "prob": 0.55, "t": t}
  r, st = rig(set_mph=30)
  r.step(25, 30, n=80, lead=flicker, path=path(300), model_age=0.0)
  assert st.active is not None and st.active["lead"]
  # far away (120 m) the same 0.55 is not enough
  r, st = rig(set_mph=30)
  r.step(25, 30, n=150, lead=lead(120.0, v=0.5, vrel=-11.0, prob=0.55), path=path(300), model_age=0.0)
  assert st.active is None


def test_round2_a_braking_lead_with_a_planned_stop_is_a_stopping_car():
  # a car at 18 mph braking at 2 m/s^2, 40 m ahead, we at 25 mph, the model planning a stop: G1 (not blocked)
  braking = lambda t: {"status": True, "d": 40.0, "v": 8.0, "vrel": -3.2, "prob": 0.9, "a": -2.0, "t": t}  # noqa: E731
  r, st = rig(set_mph=30)
  r.step(25, 30, n=80, lead=braking, path=path(60), path_v=1.0, model_age=0.0)
  assert st.active is not None and st.active["lead"] and r.out.target is not None
  assert abs(r.out.target - S.FLOOR_TRUE) < 1e-6                     # it stands ~56 m ahead: the floor at once
  # braking without the model planning a stop (a turn ahead): not a stopping car
  r, st = rig(set_mph=30)
  r.step(25, 30, n=150, lead=braking, path=path(200), path_v=9.0, model_age=0.0)
  assert st.active is None


def test_round2_no_guard_taps_while_the_exit_plan_or_a_curve_explains_the_short_path():
  from openpilot.starpilot.car.mazda.speed.planner import Contribution
  st = S.StopSlowdown(CFG, presses=True)
  ramp = Contribution("exit", 50 * MPH, presses=True, episode="x1")
  r = Rig([lambda ctx: ramp, st])
  r.step(60, 60, events=[engaged(60)])
  r.step(58, n=300, path=path(205), path_v=30 * MPH, model_age=0.0, exit_busy=True)
  assert st.active is None and r.out.reason == "exit"


def test_round2_a_guard_soon_after_another_goes_on_from_it_never_deeper_than_3_mph():
  r, st = rig(set_mph=55)
  lows = []
  for _ in range(4):                 # the flag flaps: 2 s on, 2 s off (the planner restores in between)
    r.step(55, n=200, path=path(200), path_v=30 * MPH, model_age=0.0)
    lows.append(cmd_mph(r))
    r.set = r.out.command            # (the set follows the command)
    r.step(55, n=150, path=path(400), path_v=55 * MPH, model_age=0.0)
  assert min(lows) >= 55 - 3 - 1e-6, lows
