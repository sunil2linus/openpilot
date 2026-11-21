"""The MRCC button actuator against a small model of the car's set speed (mrcc_sim.py)."""
from openpilot.starpilot.car.mazda.speed import actuator as A
from openpilot.starpilot.car.mazda.speed import car_profile as CPF
from openpilot.starpilot.car.mazda.speed.tests.mrcc_sim import Mrcc, run, events

MPH = A.MPH


def sends(car, button=None):
  return [s[:3] for s in car.sent if button is None or s[1] == button]


def test_taps_down_exactly_to_the_command_with_the_counter_pattern_and_gap():
  act, car = A.Actuator(), Mrcc(45)
  run(act, car, 400, command=42)
  assert round(car.set_units) == 42
  taps = sends(car)
  assert len(taps) == 3 and all(b == A.SET_MINUS for _, b, _ in taps)
  frames = [f for f, _, _ in taps]
  assert all(b - a >= A.BUTTON_GAP for a, b in zip(frames, frames[1:], strict=False))
  run(act, car, 300, command=42)
  assert len(sends(car)) == 3                       # nothing more once there
  assert act.latency is not None


def test_tap_counters_follow_the_pattern_and_skip():
  act, car = A.Actuator(), Mrcc(50, latency=5)
  outs = run(act, car, 3000, command=lambda f: 47 if f < 1500 else 50)
  assert round(car.set_units) == 50
  offsets = [(c - wheel) % 16 for _, _, c, wheel in car.sent]
  assert offsets and set(offsets) <= {0, 1}          # create_button_cmd sends counter + 1: wheel + 2 or wheel + 1
  slots = len(events(outs, "tap"))
  assert slots == 6 and offsets[:3] == [1, 1, 0]      # the 4th slot of the pattern is skipped


def test_ignored_taps_still_reach_the_command_never_past_it():
  act, car = A.Actuator(), Mrcc(50, ignore=4)
  lowest = [99]
  run(act, car, 2000, command=43, every=lambda f, d: lowest.__setitem__(0, min(lowest[0], car.set_units)))
  assert round(car.set_units) == 43 and round(lowest[0]) >= 43


def test_hold_for_a_big_drop_lands_on_the_command_and_never_below():
  act, car = A.Actuator(), Mrcc(60, latency=10)
  lowest = [99]
  outs = run(act, car, 1500, command=40, every=lambda f, d: lowest.__setitem__(0, min(lowest[0], car.set_units)))
  assert events(outs, "hold_start")
  assert round(car.set_units) == 40 and round(lowest[0]) >= 40
  assert len(events(outs, "tap")) <= 2              # a hold, then at most a tap or two


def test_hold_up_then_taps_never_above_the_command():
  act, car = A.Actuator(), Mrcc(30, latency=10, v_mph=28)
  highest = [0]
  outs = run(act, car, 2500, command=52, every=lambda f, d: highest.__setitem__(0, max(highest[0], car.set_units)))
  assert events(outs, "hold_start")
  assert round(car.set_units) == 52 and round(highest[0]) <= 52


def test_measured_overrun_is_learned_and_kept_as_margin():
  act, car = A.Actuator(), Mrcc(70, latency=10, overrun=1)
  lowest = [99]
  run(act, car, 3000, command=40, every=lambda f, d: lowest.__setitem__(0, min(lowest[0], car.set_units)))
  assert act.holds_learned and act.holds_learned[0][0] == 1
  assert act.overrun[A.SET_MINUS] == 1
  assert round(car.set_units) == 40
  # the next drive starts with the margin the profile kept
  act2 = A.Actuator(overrun_down=1)
  car2 = Mrcc(70, latency=10, overrun=1)
  low2 = [99]
  run(act2, car2, 3000, command=40, every=lambda f, d: low2.__setitem__(0, min(low2[0], car2.set_units)))
  assert round(low2[0]) >= 40 and round(car2.set_units) == 40


def test_holds_that_do_nothing_are_switched_off_and_taps_do_the_job():
  act, car = A.Actuator(), Mrcc(60, latency=10, holds_work=False)
  outs = run(act, car, 4000, command=45)
  assert len(events(outs, "hold_miss")) <= A.HOLD_MISS_LIMIT
  assert round(car.set_units) == 45


def test_driver_buttons_win():
  act, car = A.Actuator(), Mrcc(50)

  def driver(f, d):
    if 150 <= f < 160:
      d["set_minus"] = True
      if f == 151:
        car._move(-1)                               # his own tap lands
  outs = run(act, car, 600, command=45, every=driver)
  drv = events(outs, "driver")
  assert drv and drv[0]["direction"] == A.SET_MINUS
  # nothing of ours while he is on the button or within DRIVER_QUIET after
  assert not [s for s in car.sent if 150 <= s[0] < 160 + A.frames(A.DRIVER_QUIET)]


def test_unexplained_change_is_his():
  act, car = A.Actuator(), Mrcc(50)
  run(act, car, 50)
  car._move(+3)
  outs = run(act, car, 5)
  e = events(outs, "driver")
  assert e and e[0]["why"] == "unexplained" and e[0]["direction"] == A.SET_PLUS


def test_no_press_on_gas_brake_cruise_off_standstill_openpilot_button():
  for kw in ({"gas": True}, {"brake": True}, {"cruise": False}, {"standstill": True}, {"other_button": True}):
    act, car = A.Actuator(), Mrcc(50)
    run(act, car, 500, command=40, **kw)
    assert not car.sent, kw


def test_gas_settle_after_release():
  act, car = A.Actuator(), Mrcc(50)
  run(act, car, 100, command=40, gas=True)
  run(act, car, A.frames(A.GAS_SETTLE) - 2, command=40)
  assert not car.sent
  run(act, car, 50, command=40)
  assert car.sent


def test_dry_run_never_presses_but_says_what_it_would_do():
  act, car = A.Actuator(), Mrcc(50)
  outs = run(act, car, 300, command=40, dry_run=True)
  assert not car.sent and any(o.would_press == A.SET_MINUS for o in outs)


def test_fraction_landing_of_ours_disables_for_the_drive():
  act, car = A.Actuator(), Mrcc(50)
  run(act, car, 20, command=50)
  orig = car._move
  car._move = lambda steps: setattr(car, "set", car.set + steps * A.KPH)   # the car steps in km/h
  run(act, car, 400, command=45)
  assert act.disabled
  n = len(car.sent)
  car._move = orig
  run(act, car, 400, command=40)
  assert len(car.sent) == n


def test_car_that_ignores_everything_backs_off_then_gives_up():
  act, car = A.Actuator(), Mrcc(50, ignore=1, holds_work=False)
  outs = run(act, car, int(3 * (A.NO_EFFECT_BACKOFF + 20) / A.DT), command=40)
  assert len(events(outs, "backoff")) == A.NO_EFFECT_TRIPS and act.disabled
  assert len(events(outs, "tap")) <= A.NO_EFFECT_TRIPS * (A.NO_EFFECT_LIMIT + 1)


def test_never_below_the_cars_minimum():
  act, car = A.Actuator(), Mrcc(25)
  run(act, car, 1500, command=10)
  assert car.set >= A.CAR_MIN_SET - 1e-6 and round(car.set_units) == 19


def test_set_max_is_90_and_learned_lower_only_after_spaced_misses():
  act, car = A.Actuator(), Mrcc(88, max_mph=88, v_mph=86)
  outs = run(act, car, 2000, command=95)
  assert act.set_max is not None and abs(act.set_max - 88 * MPH) < 0.01
  assert events(outs, "set_max") and not act.disabled
  n = len(car.sent)
  run(act, car, 1000, command=95)
  assert len(car.sent) == n                        # no more SET+ at the learned top
  act, car = A.Actuator(), Mrcc(89, v_mph=87)
  run(act, car, 1500, command=99)
  assert round(car.set_units) == 90 and act.set_max is None


def test_no_set_plus_while_the_car_is_faster_than_the_set():
  act, car = A.Actuator(), Mrcc(40, v_mph=46)
  run(act, car, 500, command=45)
  assert not sends(car, A.SET_PLUS)


def test_cruise_off_forgets_presses_and_resume_at_the_same_set_is_not_a_change():
  act, car = A.Actuator(), Mrcc(50)
  run(act, car, 100, command=50)
  outs = run(act, car, 200, cruise=False)
  outs += run(act, car, 50, command=50)
  assert events(outs, "resumed") and not events(outs, "driver")
  car._move(+5)                                     # cruise off, then on at another speed (his SET)
  outs = run(act, car, 10, cruise=False) + run(act, car, 10, command=None)
  assert any(e["changed"] for e in events(outs, "engaged"))


def test_profile_latencies_start_the_window():
  p = CPF.CarProfile(path="/nonexistent/x.json", fingerprint="MAZDA_CX5", ti=True)
  p.set_latencies([10, 11, 12])
  act = A.Actuator(latencies=p.latencies)
  assert act.latency == 11 and act.window() == max(A.frames(A.MIN_IN_FLIGHT), 44)
