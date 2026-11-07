"""Offline tests for the stop ahead / stopped car warning logic (no openpilot runtime needed)."""
import importlib.util
import os
import tempfile

_spec = importlib.util.spec_from_file_location(
  "stop_warning", os.path.join(os.path.dirname(__file__), "..", "stop_warning.py"))
sw = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sw)

MPH = sw.MS_PER_MPH
T = [i * 10.0 / 32 for i in range(33)]          # model time points 0..10 s


def model_stop(v0, d_stop):
  """Model plan that decelerates evenly from v0 to a stop at distance d_stop."""
  a = v0 ** 2 / (2 * d_stop)
  t_stop = v0 / a
  v = [max(v0 - a * t, 0.0) for t in T]
  x = [v0 * min(t, t_stop) - a * min(t, t_stop) ** 2 / 2 for t in T]
  return v, x


def model_cruise(v0):
  return [v0] * len(T), [v0 * t for t in T]


def inputs(v_mph=40, a_ego=0.0, brake=False, cruise=True, model=None, lead=None, lead_relevant=False, lead_y=0.0):
  v = v_mph * MPH
  mv, mx = model if model is not None else model_cruise(v)
  i = sw.StopWarningInputs(cruise_enabled=cruise, v_ego=v, a_ego=a_ego, brake_pressed=brake, standstill=False,
                           model_t=T, model_v=mv, model_x=mx, lead_relevant=lead_relevant)
  if lead:
    i.lead_status, i.lead_d, i.lead_v, i.lead_prob = True, lead[0], lead[1] * MPH, lead[2]
    i.lead_y = lead_y
  return i


BOTH_TEST = {sw.STOP_AHEAD: sw.TEST, sw.STOPPED_LEAD: sw.TEST}
BOTH_ON = {sw.STOP_AHEAD: sw.ON, sw.STOPPED_LEAD: sw.ON}


def run(frames_inputs, modes=BOTH_ON):
  logic = sw.StopWarningLogic()
  out = []
  for n, i in enumerate(frames_inputs):
    for det, mode in logic.update(i, modes):
      out.append((n, det.kind, mode))
  return out


def repeat(i, seconds):
  return [i] * int(round(seconds / sw.DT))


# ---------------- stop ahead ----------------

def test_red_light_not_slowing_fires_once():
  i = inputs(40, model=model_stop(40 * MPH, 60))
  out = run(repeat(i, 5))
  assert [k for _, k, _ in out] == [sw.STOP_AHEAD], out
  assert out[0][0] == sw._frames(sw.STOP_DEBOUNCE) - 1          # after the debounce


def test_already_slowing_no_warning():
  assert run(repeat(inputs(40, a_ego=-1.0, model=model_stop(40 * MPH, 60)), 5)) == []


def test_braking_no_warning():
  assert run(repeat(inputs(40, brake=True, model=model_stop(40 * MPH, 60)), 5)) == []


def test_green_light_model_keeps_going_no_warning():
  assert run(repeat(inputs(45), 10)) == []


def test_brief_model_hesitation_shorter_than_debounce_no_warning():
  stop = inputs(40, model=model_stop(40 * MPH, 60))
  go = inputs(40)
  frames = (repeat(go, 2) + repeat(stop, sw.STOP_DEBOUNCE - 0.1) + repeat(go, 2)) * 3
  assert run(frames) == []


def test_gentle_far_stop_no_warning():
  # 25 mph, stop 150 m away: needs only ~0.4 m/s^2 (and beyond 6 s)
  assert run(repeat(inputs(26, model=model_stop(26 * MPH, 150)), 5)) == []


def test_below_25_mph_no_stop_warning():
  assert run(repeat(inputs(22, model=model_stop(22 * MPH, 25)), 5)) == []


def test_lead_relevant_no_stop_ahead():
  assert run(repeat(inputs(40, model=model_stop(40 * MPH, 60), lead_relevant=True), 5)) == []


def test_cruise_off_no_warning():
  assert run(repeat(inputs(40, cruise=False, model=model_stop(40 * MPH, 60)), 5)) == []


# ---------------- stopped car ----------------

def test_stopped_car_closing_fast_fires():
  out = run(repeat(inputs(50, lead=(70, 0, 0.9)), 3))
  assert out[0][0] == sw._frames(sw.LEAD_DEBOUNCE) - 1
  assert [k for _, k, _ in out] == [sw.STOPPED_LEAD], out


def test_stopped_car_but_cruise_braking_no_warning():
  assert run(repeat(inputs(50, a_ego=-2.0, lead=(70, 0, 0.9)), 3)) == []


def test_moving_lead_no_warning():
  assert run(repeat(inputs(50, lead=(40, 30, 0.9)), 3)) == []


def test_low_confidence_lead_no_warning():
  assert run(repeat(inputs(50, lead=(70, 0, 0.3)), 3)) == []
  assert run(repeat(inputs(50, lead=(70, 0, 0.7)), 3)) == []


def test_lead_outside_our_lane_no_warning():
  assert run(repeat(inputs(50, lead=(70, 0, 0.9), lead_y=2.5), 3)) == []


# ---------------- real plans from the 2026-11-01 drive (route 00000032--5b79f37567, Mazda CX-5, stock MRCC) ----------------

REAL_T = [0.0, 0.01, 0.04, 0.09, 0.16, 0.24, 0.35, 0.48, 0.62, 0.79, 0.98, 1.18, 1.41, 1.65, 1.91, 2.2, 2.5, 2.82, 3.16, 3.53,
          3.91, 4.31, 4.73, 5.17, 5.62, 6.1, 6.6, 7.12, 7.66, 8.21, 8.79, 9.38, 10.0]
# t=1050.1: 50 mph, red light 4.5 s before the driver braked; the plan only eases off (min 78 %): no warning yet
PLAN_1050 = (22.45, [22.3, 22.29, 22.29, 22.29, 22.28, 22.28, 22.25, 22.25, 22.24, 22.23, 22.19, 22.21, 22.14, 22.1, 22.07, 22.0, 21.94,
                     21.87, 21.75, 21.63, 21.47, 21.35, 21.19, 20.98, 20.72, 20.46, 20.12, 19.77, 19.37, 18.95, 18.47, 18.02, 17.56],
             [0.0, 0.2, 0.9, 2.0, 3.5, 5.4, 7.8, 10.7, 13.9, 17.6, 21.7, 26.3, 31.2, 36.7, 42.5, 48.7, 55.4, 62.4, 69.8, 77.7, 85.9,
              94.5, 103.5, 112.7, 122.1, 132.1, 142.1, 152.7, 163.3, 174.0, 184.4, 195.8, 206.8])
# t=1052.1: same light 2.5 s before the driver braked (hard, -2.8 m/s^2): plan min 63 % at 7.7 s, 148 m -> warn
PLAN_1052 = (22.67, [22.39, 22.4, 22.4, 22.4, 22.35, 22.34, 22.31, 22.25, 22.18, 22.1, 21.99, 21.9, 21.73, 21.58, 21.42, 21.21, 21.01,
                     20.74, 20.46, 20.13, 19.73, 19.33, 18.87, 18.31, 17.65, 16.88, 16.04, 15.13, 14.19, 13.18, 12.16, 11.22, 10.28],
             [0.0, 0.2, 0.9, 2.0, 3.5, 5.5, 7.9, 10.7, 13.9, 17.6, 21.7, 26.2, 31.1, 36.4, 42.1, 48.0, 54.5, 61.1, 68.1, 75.5, 83.1,
              90.8, 99.0, 107.1, 115.1, 123.7, 131.8, 139.8, 147.8, 155.4, 162.7, 170.0, 177.0])
# t=1054.1: 0.5 s before the brake: plan min 38 %, never below 1 m/s inside the horizon (the old full-stop rule missed this)
PLAN_1054 = (22.65, [21.8, 21.79, 21.76, 21.73, 21.67, 21.62, 21.55, 21.41, 21.26, 21.09, 20.88, 20.67, 20.37, 20.07, 19.73, 19.42,
                     18.97, 18.47, 17.94, 17.31, 16.63, 15.91, 15.1, 14.21, 13.2, 12.11, 10.91, 9.78, 8.63, 7.5, 6.42, 5.49, 4.65],
             [0.0, 0.2, 0.9, 1.9, 3.4, 5.3, 7.6, 10.4, 13.5, 17.0, 20.9, 25.1, 29.7, 34.7, 40.0, 45.5, 51.2, 57.3, 63.4, 69.9, 76.3,
              82.9, 89.3, 95.7, 101.7, 107.9, 113.5, 118.8, 123.8, 128.4, 132.2, 136.1, 139.4])
# t=1152.1: another red light 1.7 s before the brake: min 65 % at 7.7 s, 148 m (borderline deceleration, 1.0 m/s^2)
PLAN_1152 = (22.64, [22.28, 22.25, 22.22, 22.19, 22.14, 22.1, 22.07, 22.01, 21.92, 21.84, 21.72, 21.64, 21.5, 21.36, 21.2, 21.02,
                     20.84, 20.59, 20.31, 20.0, 19.59, 19.23, 18.77, 18.25, 17.66, 17.0, 16.27, 15.53, 14.77, 13.96, 13.17, 12.42, 11.69],
             [0.0, 0.2, 0.9, 2.0, 3.5, 5.4, 7.8, 10.6, 13.8, 17.4, 21.5, 25.9, 30.8, 36.0, 41.6, 47.5, 53.9, 60.5, 67.6, 74.9, 82.4,
              90.0, 98.1, 106.2, 114.3, 122.8, 131.1, 139.3, 147.5, 155.7, 163.3, 171.3, 179.0])


def real_inputs(plan, a_ego=0.0):
  v, mv, mx = plan
  return sw.StopWarningInputs(cruise_enabled=True, v_ego=v, a_ego=a_ego, brake_pressed=False, standstill=False,
                              model_t=REAL_T, model_v=mv, model_x=mx)


def test_real_red_light_plans_warn_before_the_driver_braked():
  for plan in (PLAN_1052, PLAN_1054, PLAN_1152):
    out = run(repeat(real_inputs(plan), 3))
    assert [k for _, k, _ in out] == [sw.STOP_AHEAD], (plan[0], out)


def test_real_red_light_plan_too_early_no_warning():
  assert run(repeat(real_inputs(PLAN_1050), 3)) == []


def test_real_plan_sequence_fires_once_about_a_second_after_the_plan_commits():
  frames = repeat(real_inputs(PLAN_1050), 2) + repeat(real_inputs(PLAN_1052), 2) + repeat(real_inputs(PLAN_1054), 2)
  out = run(frames)
  assert len(out) == 1 and out[0][1] == sw.STOP_AHEAD
  assert abs(out[0][0] - (2 * 20 + sw._frames(sw.STOP_DEBOUNCE) - 1)) <= 1, out


def test_short_plan_dropout_does_not_restart_debounce():
  warn = real_inputs(PLAN_1052)
  go = real_inputs(PLAN_1050)
  frames = repeat(warn, 0.6) + repeat(go, sw.STOP_DROPOUT - 0.05) + repeat(warn, 0.6)
  assert len(run(frames)) == 1
  frames = repeat(warn, 0.6) + repeat(go, sw.STOP_DROPOUT + 0.1) + repeat(warn, 0.6)
  assert run(frames) == []


def test_slowdown_beyond_horizon_ignored():
  # The plan only gets slow after 8 s: not within the horizon
  v = 20.0
  mv = [v if t <= 8.3 else 5.0 for t in REAL_T]
  mx = [v * t for t in REAL_T]
  i = sw.StopWarningInputs(cruise_enabled=True, v_ego=v, a_ego=0.0, brake_pressed=False, standstill=False,
                           model_t=REAL_T, model_v=mv, model_x=mx)
  assert run(repeat(i, 3)) == []


def test_stop_ahead_detection_carries_lead_data():
  # A slowing lead 90 m ahead that is not "relevant" yet: the CSV line must still show it.
  i = inputs(40, model=model_stop(40 * MPH, 60), lead=(90, 20, 0.85), lead_y=0.4)
  logic = sw.StopWarningLogic()
  fired = []
  for _ in range(int(round(5 / sw.DT))):
    fired += logic.update(i, BOTH_TEST)
  kinds = [d.kind for d, _ in fired]
  assert sw.STOP_AHEAD in kinds, kinds
  det = next(d for d, _ in fired if d.kind == sw.STOP_AHEAD)
  assert det.lead_status and abs(det.lead_d - 90) < 1e-6 and abs(det.lead_speed - 20 * MPH) < 1e-6
  assert abs(det.lead_prob - 0.85) < 1e-6 and abs(det.lead_y - 0.4) < 1e-6 and not det.lead_relevant
  line = sw.format_log_line(det, sw.TEST, 40 * MPH, 0.0)
  assert line.strip().split(",")[10:14] == ["1", "90.0", "0.4", "0"], line


def test_far_horizon_single_dip_no_warning():
  v = 40 * MPH
  mv, mx = model_cruise(v)
  mv = list(mv)
  mv[20] = 0.5            # one noisy point ~6 s ahead
  assert run(repeat(inputs(40, model=(mv, mx)), 5)) == []


def test_a_ego_noise_does_not_block_or_reset():
  base = model_stop(40 * MPH, 60)
  frames = [inputs(40, a_ego=(0.6 if n % 2 else -0.6), model=base) for n in range(100)]
  out = run(frames)
  assert [k for _, k, _ in out] == [sw.STOP_AHEAD], out


def test_lead_prob_flicker_debounced():
  good = inputs(50, lead=(70, 0, 0.9))
  bad = inputs(50, lead=(70, 0, 0.5))
  frames = [good if n % 4 else bad for n in range(60)]
  assert run(frames) == []


def test_far_stopped_car_no_warning_yet():
  assert run(repeat(inputs(50, lead=(200, 0, 0.9)), 3)) == []


def test_slow_speed_stopped_car_no_warning():
  assert run(repeat(inputs(15, lead=(20, 0, 0.9)), 3)) == []


# ---------------- repeats, modes, logging ----------------

def test_no_repeat_while_condition_stays():
  out = run(repeat(inputs(50, lead=(70, 0, 0.9)), 30))
  assert len(out) == 1


def test_short_gap_does_not_rearm():
  warn = inputs(50, lead=(70, 0, 0.9))
  clear = inputs(50)
  frames = repeat(warn, 2) + repeat(clear, 1) + repeat(warn, 2)
  assert len(run(frames)) == 1


def test_rearms_after_condition_gone_and_min_gap():
  warn = inputs(50, lead=(70, 0, 0.9))
  clear = inputs(50)
  frames = repeat(warn, 2) + repeat(clear, sw.MIN_GAP + 1) + repeat(warn, 2)
  assert len(run(frames)) == 2


def test_modes():
  i = inputs(40, model=model_stop(40 * MPH, 60))
  assert run(repeat(i, 5), {sw.STOP_AHEAD: sw.OFF, sw.STOPPED_LEAD: sw.ON}) == []
  out = run(repeat(i, 5), BOTH_TEST)
  assert out and out[0][2] == sw.TEST
  # stopped car off, stop ahead on: stopped car scenario gives nothing
  assert run(repeat(inputs(50, lead=(70, 0, 0.9)), 3), {sw.STOP_AHEAD: sw.ON, sw.STOPPED_LEAD: sw.OFF}) == []


def test_log_line_and_rotation():
  det = sw.Detection(sw.STOPPED_LEAD, 70.0, 2.1, 3.1, 0.0, 0.9, True, 70.0, 0.3, False)
  line = sw.format_log_line(det, sw.TEST, 50 * MPH, 0.1, {"latitude": 42.5, "longitude": -83.1})
  parts = line.strip().split(",")
  assert len(parts) == len(sw.LOG_HEADER.strip().split(",")), parts
  assert parts[1] == sw.STOPPED_LEAD and parts[2] == "test" and parts[3] == "50.0"
  assert parts[10] == "1" and parts[11] == "70.0" and parts[12] == "0.3" and parts[13] == "0"
  with tempfile.TemporaryDirectory() as d:
    p = os.path.join(d, "log.csv")
    sw.append_log(line, p)
    sw.append_log(line, p)
    with open(p) as f:
      lines = f.readlines()
    assert lines[0] == sw.LOG_HEADER and len(lines) == 3
    with open(p, "a") as f:
      f.write("x" * (sw.LOG_MAX_BYTES + 10))
    sw.append_log(line, p)
    assert os.path.exists(p + ".old")
    with open(p) as f:
      assert f.readlines()[0] == sw.LOG_HEADER


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
        print("FAIL", name, repr(e))
  print(f"{failed} failed")
  sys.exit(1 if failed else 0)


# ---------------- plan dips (data for a red light slowdown) ----------------

DIP_PLAN = (6.0, 8.0, 90.0, 1.2)     # (t, v, distance, implied decel): 8 m/s at 90 m, i.e. ratio 0.4 at 45 mph


def run_dips(seconds, plan_fn, brake_fn=lambda t: False, cruise_fn=lambda t: True, v_fn=lambda t: 45 * MPH,
             lead_fn=lambda t: False, fired_fn=lambda t: False):
  tr = sw.PlanDipTracker()
  out = []
  for k in range(int(round(seconds / sw.DT))):
    t = k * sw.DT
    i = sw.StopWarningInputs(cruise_enabled=cruise_fn(t), v_ego=v_fn(t), a_ego=0.0, brake_pressed=brake_fn(t),
                             standstill=False, lead_relevant=lead_fn(t))
    for a in tr.update(i, fired_fn(t), plan_fn(t)):
      out.append((round(t, 2),) + a)
  return out


def test_dip_then_driver_brakes():
  # the plan dips from 10 s, the warning fires at 11.5 s, the driver brakes (cruise off) at 13 s
  out = run_dips(30, lambda t: DIP_PLAN if 10 <= t < 13 else None, brake_fn=lambda t: 13 <= t < 14,
                 cruise_fn=lambda t: t < 13, fired_fn=lambda t: abs(t - 11.5) < 1e-6)
  assert [a[1:3] for a in out] == [("open", sw.PLAN_DIP), ("close", sw.PLAN_DIP)], out
  assert out[0][0] == 11.0 - sw.DT and out[0][3]["held_before_open_s"] == 1.0
  extra = out[1][3]
  assert extra["outcome"] == "driver braked or cancelled cruise"
  assert abs(extra["driver_brake_after_dip_s"] - 3.0) < 1e-6 and abs(extra["warning_fired_after_dip_s"] - 1.5) < 1e-6
  assert abs(extra["min_plan_ratio"] - 8.0 / (45 * MPH)) < 1e-3


def test_dip_without_brake_is_the_green_light_class():
  out = run_dips(30, lambda t: DIP_PLAN if 10 <= t < 13 else None)
  assert [a[1] for a in out] == ["open", "close"]
  assert out[1][3]["outcome"].startswith("no brake within 8 s")
  assert out[1][3]["driver_brake_after_dip_s"] == ""
  assert out[1][0] >= 10 + sw.DIP_WATCH - 1e-6           # kept open long enough to see a brake


def test_short_dip_lead_low_speed_ignored():
  assert run_dips(20, lambda t: DIP_PLAN if 10 <= t < 10.6 else None) == []
  assert run_dips(20, lambda t: DIP_PLAN if 10 <= t < 13 else None, lead_fn=lambda t: True) == []
  assert run_dips(20, lambda t: DIP_PLAN if 10 <= t < 13 else None, v_fn=lambda t: 20 * MPH) == []
  # short plan dropouts do not restart the 1 s
  out = run_dips(20, lambda t: DIP_PLAN if (10 <= t < 10.5 or 10.65 <= t < 11.3) else None)
  assert out and out[0][1] == "open"


def test_driver_brake_without_any_dip_is_logged():
  out = run_dips(20, lambda t: None, brake_fn=lambda t: 12 <= t < 13, cruise_fn=lambda t: t < 12)
  assert len(out) == 1 and out[0][1:3] == ("point", sw.NO_DIP_BRAKE) and abs(out[0][0] - 12.0) < 1e-6
  assert out[0][3]["seconds_since_last_dip"] == "none in the drive"
  # not when a dip was seen in the 10 s before, not below 25 mph, not with a relevant lead
  out = run_dips(40, lambda t: DIP_PLAN if 5 <= t < 7 else None, brake_fn=lambda t: 15 <= t < 16, cruise_fn=lambda t: t < 15)
  assert [a[1] for a in out] == ["open", "close"]
  assert run_dips(20, lambda t: None, brake_fn=lambda t: 12 <= t < 13, cruise_fn=lambda t: t < 12, v_fn=lambda t: 15 * MPH) == []
  assert run_dips(20, lambda t: None, brake_fn=lambda t: 12 <= t < 13, cruise_fn=lambda t: t < 12, lead_fn=lambda t: True) == []


def test_driver_brake_counts_once_and_cruise_off_driving_is_quiet():
  # cruise never on: nothing at all
  assert run_dips(20, lambda t: DIP_PLAN, brake_fn=lambda t: 12 <= t < 13, cruise_fn=lambda t: False) == []
  # one brake edge = one event
  out = run_dips(30, lambda t: None, brake_fn=lambda t: 12 <= t < 15, cruise_fn=lambda t: t < 12)
  assert len(out) == 1


def test_long_dip_closes_at_dip_max():
  out = run_dips(sw.DIP_MAX + 10, lambda t: DIP_PLAN)
  assert [a[1] for a in out][:2] == ["open", "close"]
  assert out[1][0] <= sw.DIP_MAX + 1.0 + 1e-6



def test_lead_relevance_matches_frogpilot():
  # 25 m/s, lead at 10 m/s: FrogPilot counts it out to 6 + 25*3 + 15*10 = 231 m (and within the model's path + 10 m)
  assert sw.lead_is_relevant(True, 180.0, 15.0, 0.0, False, 25.0, 200.0)
  assert not sw.lead_is_relevant(True, 180.0, 15.0, 0.0, False, 25.0, 150.0)   # beyond the planned path
  assert not sw.lead_is_relevant(False, 50.0, 0.0, 0.0, False, 25.0, 200.0)
  assert sw.lead_is_relevant(True, 8.0, 0.0, 0.0, True, 0.0, 0.0)
  assert not sw.lead_is_relevant(True, 12.0, 0.0, 0.0, True, 0.0, 0.0)


def test_no_stop_ahead_during_a_lane_change_or_2_s_after():
  # drive 3b: the model's plan dips while the car changes lanes; 2 of 6 warnings came right after one began
  stop = inputs(40, model=model_stop(40 * MPH, 60))
  changing = inputs(40, model=model_stop(40 * MPH, 60))
  changing.lane_change = True
  assert run(repeat(changing, 4)) == []                           # during the lane change: nothing
  out = run(repeat(changing, 2) + repeat(stop, 5))
  assert [k for _, k, _ in out] == [sw.STOP_AHEAD], out           # still warns once the lane change is over ...
  first = out[0][0] - sw._frames(2.0)
  assert first >= sw._frames(sw.LANE_CHANGE_HOLD) + sw._frames(sw.STOP_DEBOUNCE) - 2, first   # ... 2 s + debounce later


def test_our_own_stop_slowdown_does_not_hide_the_warning():
  # Mazda: the set speed is lowered 0.5 s before the warning shows (stopped_car_slowdown.WARN_START_DEBOUNCE); the car
  # slowing for that must not count as "already slowing" (owner: "first start reducing the speed and then display
  # the warning")
  model = model_stop(40 * MPH, 60)
  ours = [inputs(40, a_ego=-1.2, model=model) for _ in range(40)]
  for i in ours:
    i.own_slowdown = True
  assert run(ours) != []
  assert run([inputs(40, a_ego=-1.2, model=model) for _ in range(40)]) == []     # someone else slowing: no warning
  harder = [inputs(40, a_ego=-2.5, model=model) for _ in range(40)]
  for i in harder:
    i.own_slowdown = True
  assert run(harder) == []                     # braking harder than a lowered set speed does: the radar cruise / driver


def test_own_slowdown_still_respects_the_brake_and_reports_the_applied_limit():
  model = model_stop(40 * MPH, 60)
  braked = [inputs(40, a_ego=-1.2, brake=True, model=model) for _ in range(40)]
  for i in braked:
    i.own_slowdown = True
  assert run(braked) == []
  logic = sw.StopWarningLogic()
  i = inputs(40, a_ego=-1.2, model=model)
  i.own_slowdown = True
  for _ in range(40):
    logic.update(i, BOTH_ON)
  assert not any("already slowing" in b for b in logic.common_blockers(i, sw.ON))
  i.own_slowdown = False
  assert any(f"<= {sw.NOT_SLOWING_ACCEL}" in b for b in logic.common_blockers(i, sw.ON))


def test_the_warning_follows_our_start_as_the_car_begins_to_slow():
  # time-sequenced: our set speed drops at 0.5 s (flag up), the car slows from ~1.0 s at -1.2 m/s^2; the warning
  # still shows at 1.0 s
  model = model_stop(40 * MPH, 60)
  seq = []
  for n in range(60):
    t = n * sw.DT
    i = inputs(40, a_ego=-1.2 if t >= 0.8 else 0.0, model=model)
    i.own_slowdown = t >= 0.5
    seq.append(i)
  out = run(seq)
  assert [k for _, k, _ in out] == [sw.STOP_AHEAD] and out[0][0] == sw._frames(sw.STOP_DEBOUNCE) - 1, out


# ---------------- our own stop slowdown under way: the stopped car warning comes as soon as more braking is needed ----------------

def _own(i):
  i.own_slowdown = True
  return i


def test_own_slowdown_warns_as_soon_as_stopping_needs_more_than_the_cruise_gives():
  # 45 mph, a stopped car 95 m ahead: 4.7 s away (the TTC rule says not yet) but 2.2 m/s^2 needed; 65 % a car
  lead = (95, 0, 0.65)
  assert run(repeat(inputs(45, lead=lead), 3)) == []                               # without our slowdown: as before
  out = run(repeat(_own(inputs(45, lead=lead)), 3))
  assert [k for _, k, _ in out] == [sw.STOPPED_LEAD] and out[0][0] == sw._frames(sw.LEAD_DEBOUNCE) - 1, out
  # the other rules stay: the next lane, barely closing, far (little braking needed), too unsure; a car moving at 25 mph
  # 95 m ahead needs only 0.55 m/s^2 (a car at 8 mph there needs 1.5: the slower moving car rule, below)
  for kw in (dict(lead=(95, 25, 0.65)), dict(lead=(95, 0, 0.65), lead_y=1.5), dict(lead=(95, 37, 0.9)), dict(lead=(220, 0, 0.9)),
             dict(lead=(95, 0, 0.5))):
    assert run(repeat(_own(inputs(45, **kw)), 3)) == [], kw
  # the brake and the car already braking hard (the radar cruise / driver) still silence it; a lowered set speed does not
  assert run(repeat(_own(inputs(45, brake=True, lead=lead)), 3)) == []
  assert run(repeat(_own(inputs(45, a_ego=-2.5, lead=lead)), 3)) == []
  assert run(repeat(_own(inputs(45, a_ego=-1.0, lead=lead)), 3)) != []
  # the blockers say why
  logic = sw.StopWarningLogic()
  i = inputs(45, lead=(95, 0, 0.5))
  logic.update(i, BOTH_ON)
  assert any("prob 0.50 < 0.8" in b for b in logic.stopped_lead_blockers(i, sw.ON))
  assert any("ttc" in b for b in logic.stopped_lead_blockers(i, sw.ON))
  _own(i)
  assert any("prob 0.50 < 0.6" in b for b in logic.stopped_lead_blockers(i, sw.ON))
  assert not any("ttc" in b for b in logic.stopped_lead_blockers(i, sw.ON))


def test_a_recent_stop_ahead_warning_also_relaxes_the_stopped_car_rule():
  # the stop ahead showed at 1 s; the queue car (stopped, 5 s away, 90 %) appears 4 s later: warned at once (+ debounce)
  red = inputs(45, model=model_stop(45 * MPH, 90))
  lead = inputs(45, lead=(100, 0, 0.9))
  out = run(repeat(red, 2) + repeat(inputs(45), 3) + repeat(lead, 2))
  kinds = [(n, k) for n, k, _ in out]
  assert [k for _, k in kinds] == [sw.STOP_AHEAD, sw.STOPPED_LEAD], kinds
  assert kinds[1][0] == 5 * 20 + sw._frames(sw.LEAD_DEBOUNCE) - 1, kinds
  # more than OWN_STOP_RECENT after it: the usual rules (5 s away: not yet)
  out = run(repeat(red, 2) + repeat(inputs(45), sw.OWN_STOP_RECENT + 0.5) + repeat(lead, 2))
  assert [k for _, k, _ in out] == [sw.STOP_AHEAD], out


# drive 50 (t = s after redLight at 307.5): our stop ahead start at 7.5 s (own slowdown from there), the stopped queue car
# (radarState leadOne, qlog samples): (t, vEgo mph, aEgo, lead status, dRel m, vLead mph, yRel m, modelProb)
DRIVE_50_LEAD = [
  (9.2, 44.4, -0.4, 1, 110.1, 10.5, 0.32, 0.3), (9.45, 43.9, -0.75, 1, 117.5, 9.4, 0.42, 0.29),
  (9.7, 43.4, -0.88, 1, 117.1, 6.9, 0.79, 0.38), (9.95, 43.0, -0.84, 1, 106.7, 8.6, 0.63, 0.41),
  (10.2, 42.3, -1.13, 1, 108.5, 7.6, 0.52, 0.35), (10.45, 41.7, -0.77, 1, 106.7, 4.6, 1.48, 0.57),
  (10.7, 41.3, -1.05, 1, 101.0, 4.1, 1.42, 0.7), (10.95, 40.8, -0.96, 1, 96.7, 4.5, 0.95, 0.65),
  (11.2, 40.2, -0.99, 1, 98.2, 1.5, 1.85, 0.53), (11.45, 39.6, -1.08, 1, 96.0, 3.8, 1.97, 0.62),
  (11.7, 38.9, -1.23, 1, 100.8, 1.3, 2.66, 0.43), (11.95, 38.6, -0.91, 1, 84.1, 2.4, 1.57, 0.65),
  (12.2, 37.9, -0.95, 1, 80.8, 3.6, 1.24, 0.72), (12.45, 37.5, -0.89, 1, 80.0, 2.6, 1.13, 0.71),
  (12.7, 36.8, -1.0, 1, 77.2, 2.4, 0.95, 0.85), (12.95, 36.6, -0.7, 1, 73.5, 2.2, 0.93, 0.87),
  (13.21, 35.9, -0.98, 1, 72.1, 1.8, 0.66, 0.86), (13.46, 35.6, -0.8, 1, 64.9, 3.0, 0.62, 0.93),
  (13.7, 34.9, -1.0, 1, 63.8, 1.9, 0.31, 0.93), (13.95, 34.5, -1.03, 1, 54.2, 2.1, 0.44, 0.94),
  (14.2, 33.9, -0.99, 1, 52.2, 2.5, 0.3, 0.97), (14.45, 33.5, -0.92, 1, 45.3, 2.0, 0.33, 0.98),
  (14.71, 32.9, -0.8, 1, 40.0, 2.8, 0.18, 0.98), (14.95, 32.5, -1.07, 1, 38.4, 2.7, 0.22, 0.98),
  (15.2, 31.9, -0.98, 1, 30.4, 2.9, 0.17, 0.99), (15.45, 31.0, -2.13, 1, 26.9, 2.5, 0.1, 0.99),
]


def _drive_50(own, a_ego=None):
  """20 Hz from 5 s to 15.5 s, the lead samples held until the next: the frame the stopped car warning fired (t s)."""
  logic = sw.StopWarningLogic()
  fired = []
  for n in range(int(5.0 / sw.DT), int(15.5 / sw.DT)):
    t = n * sw.DT
    row = max((r for r in DRIVE_50_LEAD if r[0] <= t), key=lambda r: r[0], default=None)
    v = row[1] if row else 45.0
    i = sw.StopWarningInputs(cruise_enabled=True, v_ego=v * MPH, a_ego=(a_ego if a_ego is not None else (row[2] if row else 0.0)),
                             brake_pressed=False, standstill=False, own_slowdown=own and t >= 7.5)
    if row:
      i.lead_status, i.lead_d, i.lead_v, i.lead_y, i.lead_prob = True, row[4], row[5] * MPH, row[6], row[7]
    for det, _ in logic.update(i, {sw.STOP_AHEAD: sw.OFF, sw.STOPPED_LEAD: sw.ON}):
      fired.append((round(t, 2), det.kind, round(det.distance, 1), round(det.lead_prob, 2)))
  return fired


def test_drive_50_replay_the_brake_warning_comes_1_4_s_sooner_with_our_slowdown_under_way():
  # on the drive: "STOPPED CAR AHEAD - BRAKE" at 321.8 (14.3 s, 94 %, 54 m), the driver braked at 322.5 at 32.4 mph.
  # Now 320.4 (77 m, 85 %): the car was still 1.4-2.7 m off our line (raw yRel) at 318.7-319.7, hence not 319.5
  fired = _drive_50(own=True)
  assert len(fired) == 1 and fired[0][1] == sw.STOPPED_LEAD, fired
  assert 12.7 <= fired[0][0] <= 13.0 and fired[0][2] >= 73.0, fired
  # without our slowdown the car's own slowing (aEgo -0.8..-1.1) hides it; with the usual rules (the car not slowing) it
  # would have come where it did on the drive, at ~54 m
  assert _drive_50(own=False) == []
  fired = _drive_50(own=False, a_ego=0.0)
  assert len(fired) == 1 and fired[0][0] >= 14.2 and fired[0][2] <= 55.0, fired


# ---------------- a moving but much slower car (SLOW_LEAD_*): "CAR AHEAD - BRAKE" ----------------

# drive 52 (t = s after 190.0, night, a queue at a light that had turned green): our stop ahead slowdown from 1.0 s (own
# slowdown), the set speed at the floor from 3.0 s, no radar lock; the queue car (radarState leadOne, qlog samples):
# (t, vEgo mph, aEgo, lead status, dRel m, vLead mph, yRel m, modelProb). The driver braked at 6.0 s (25.7 mph, 25 m).
DRIVE_52_LEAD = [
  (2.62, 28.8, 0.40, 1, 86.7, 7.4, -1.03, 0.61), (2.89, 29.0, 0.20, 1, 83.5, 12.2, -1.52, 0.59),
  (3.13, 29.0, 0.02, 1, 46.6, 14.3, -0.45, 0.99), (3.37, 28.9, -0.09, 1, 50.4, 7.2, -0.53, 0.98),
  (3.63, 28.8, -0.30, 1, 46.9, 13.9, -0.59, 0.98), (3.88, 28.4, -0.65, 1, 43.1, 9.1, -0.68, 0.98),
  (4.13, 28.2, -0.60, 1, 39.8, 8.4, -0.73, 0.98), (4.38, 27.8, -0.51, 1, 39.0, 8.1, -0.77, 0.99),
  (4.63, 27.5, -0.60, 1, 42.5, 5.7, -0.93, 0.99), (4.88, 27.1, 0.00, 1, 31.5, 8.2, -0.74, 1.00),
  (5.13, 26.8, -0.42, 1, 32.2, 7.5, -0.76, 0.99), (5.38, 26.6, -0.48, 1, 29.9, 8.2, -0.75, 1.00),
  (5.63, 26.2, -0.50, 1, 31.2, 4.4, -0.66, 0.99), (5.88, 26.0, -0.61, 1, 24.9, 6.6, -0.49, 1.00),
]


def _drive_52(own=True, a_ego=None):
  """20 Hz from 2.6 s to 6.0 s, the samples held until the next: the stopped car warnings (t, moving, d, required)."""
  logic = sw.StopWarningLogic()
  fired = []
  for n in range(int(2.6 / sw.DT), int(6.0 / sw.DT)):
    t = n * sw.DT
    row = max((r for r in DRIVE_52_LEAD if r[0] <= t), key=lambda r: r[0], default=None)
    if row is None:
      continue
    i = sw.StopWarningInputs(cruise_enabled=True, v_ego=row[1] * MPH, a_ego=row[2] if a_ego is None else a_ego,
                             brake_pressed=False, standstill=False, own_slowdown=own)
    i.lead_status, i.lead_d, i.lead_v, i.lead_y, i.lead_prob = True, row[4], row[5] * MPH, row[6], row[7]
    for det, _ in logic.update(i, {sw.STOP_AHEAD: sw.OFF, sw.STOPPED_LEAD: sw.ON}):
      fired.append((round(t, 2), det.kind, det.moving, round(det.distance, 1), round(det.required_decel, 2)))
  return fired


def test_drive_52_replay_a_much_slower_moving_car_warns_at_194_5_to_195():
  # the car ahead at 6-9 mph (above the stopped car's 2 m/s): no warning on the drive; matching its speed needed 1.11
  # m/s^2 at 194.5 and 1.32 at 195.0 while the car slowed 0.4-0.7
  fired = _drive_52()
  assert len(fired) == 1 and fired[0][1] == sw.STOPPED_LEAD and fired[0][2], fired
  assert 4.5 <= fired[0][0] <= 5.0 and fired[0][4] >= sw.SLOW_LEAD_DECEL, fired
  orig = sw.SLOW_LEAD_ENABLED
  sw.SLOW_LEAD_ENABLED = False                     # the old rule: nothing
  try:
    assert _drive_52() == []
  finally:
    sw.SLOW_LEAD_ENABLED = orig
  # without our slowdown: the usual time to collision rule (4 s), and the car not slowing (-0.5): 195.4 s, 30 m
  fired = _drive_52(own=False, a_ego=0.0)
  assert len(fired) == 1 and 5.3 <= fired[0][0] <= 5.6, fired
  # the alert: "CAR AHEAD - BRAKE" for a moving car, as before for a stopped one
  assert sw.lead_warning_text(8.2 * MPH) == "CAR AHEAD - BRAKE" and sw.lead_warning_text(0.5) == "STOPPED CAR AHEAD - BRAKE"


def test_slower_moving_car_rules():
  # 40 mph, a car 50 m ahead at 15 mph, 95 %, in our lane: 11.2 m/s closing, 1.39 m/s^2 to match it, 4.5 s: with our
  # slowdown on it warns (moving), without it the time to collision rule waits
  lead = (50, 15, 0.95)
  out = run(repeat(_own(inputs(40, lead=lead)), 3))
  assert [k for _, k, _ in out] == [sw.STOPPED_LEAD] and out[0][0] == sw._frames(sw.LEAD_DEBOUNCE) - 1, out
  assert run(repeat(inputs(40, lead=lead), 3)) == []
  assert run(repeat(inputs(40, lead=(40, 15, 0.95)), 3)) != []                    # 3.6 s: the usual rule
  # not for: the next lane, a car only 3.6 m/s slower, little braking needed (1.0 < 1.1), below 20 mph, too unsure
  for kw in (dict(lead=lead, lead_y=1.5), dict(lead=(30, 32, 0.95)), dict(lead=(70, 15, 0.95)), dict(lead=(60, 10, 0.95), v_mph=19),
             dict(lead=(50, 15, 0.55))):
    v = kw.pop("v_mph", 40)
    assert run(repeat(_own(inputs(v, **kw)), 3)) == [], kw
  # already slowing nearly as hard as needed (the radar cruise braking for the car it tracks, or the driver): silent
  assert run(repeat(_own(inputs(40, a_ego=-1.2, lead=lead)), 3)) == []          # 1.2 of the 1.39 needed
  assert run(repeat(_own(inputs(40, a_ego=-0.9, lead=lead)), 3)) != []          # the lowered set speed's ~0.9: warn
  # the detection is marked moving; a stopped car is not; the blockers say why
  logic = sw.StopWarningLogic()
  i = _own(inputs(40, lead=lead))
  logic.update(i, BOTH_ON)
  assert logic.stopped_lead(i).moving and not logic.stopped_lead(_own(inputs(40, lead=(60, 0, 0.95)))).moving
  i = _own(inputs(40, a_ego=-1.2, lead=lead))
  for _ in range(40):
    logic.update(i, BOTH_ON)
  assert any("slower car: already slowing" in b for b in logic.stopped_lead_blockers(i, sw.ON))


def test_radar_cruise_following_a_slower_car_down_is_not_warned():
  # 45 mph closing on a car at 15 mph from 75 m (1.28 m/s^2 to match it), our slowdown on. The radar cruise tracking it
  # brakes at what is needed (5 % more): no warning. Only our lowered set speed (0.6 m/s^2): the need grows, it warns.
  def approach(radar):
    v, d, vl, seq = 45 * MPH, 75.0, 15 * MPH, []
    for _ in range(int(8.0 / sw.DT)):
      closing = v - vl
      need = closing ** 2 / (2 * max(d - sw.LEAD_BUFFER, 1.0)) if closing > 0 else 0.0
      a = (-1.05 * need if radar else -0.6) if closing > 0 else 0.0
      seq.append(_own(sw.StopWarningInputs(cruise_enabled=True, v_ego=v, a_ego=a, brake_pressed=False, standstill=False,
                                           lead_status=True, lead_d=d, lead_v=vl, lead_prob=0.95, lead_y=0.1)))
      v = max(v + a * sw.DT, vl)
      d -= (v - vl) * sw.DT
    return run(seq)
  assert approach(radar=True) == []
  assert [k for _, k, _ in approach(radar=False)] == [sw.STOPPED_LEAD]
