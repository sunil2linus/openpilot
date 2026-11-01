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
  det = sw.Detection(sw.STOPPED_LEAD, 70.0, 2.1, 3.1, 0.0, 0.9)
  line = sw.format_log_line(det, sw.TEST, 50 * MPH, 0.1, {"latitude": 42.5, "longitude": -83.1})
  parts = line.strip().split(",")
  assert len(parts) == len(sw.LOG_HEADER.strip().split(",")), parts
  assert parts[1] == sw.STOPPED_LEAD and parts[2] == "test" and parts[3] == "50.0"
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
