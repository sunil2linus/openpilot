"""The model follower (model_follow.py, TEST ONLY: logs the set speed the model would want) and its card wiring."""
import os
import py_compile
import random

import pytest

from opendbc.car.mazda import model_follow as mf

MPH = mf.MS_PER_MPH
KPH = mf.MS_PER_KPH
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
T_IDXS = [0.0, 0.0098, 0.039, 0.088, 0.156, 0.244, 0.352, 0.479, 0.625, 0.791, 0.977, 1.18, 1.406, 1.65, 1.914, 2.197, 2.5,
          2.82, 3.164, 3.525, 3.906, 4.307, 4.727, 5.166, 5.625, 6.104, 6.602, 7.119, 7.656, 8.213, 8.789, 9.385, 10.0]


def run(f, n, v, driver, accel, cruise=True, plan=None, dv=None):
  """n updates; accel: a number or a function of the step; dv: integrate the speed with this accel (None: constant)."""
  out = []
  for k in range(n):
    a = accel(k) if callable(accel) else accel
    pt, pv = (T_IDXS, plan(v)) if plan else (None, None)
    out.append((f.update(v, driver, cruise, a, pt, pv), f.reason, f.presses, f.direction))
    if dv is not None:
      v = max(v + dv(k) * f.dt, 0.0)
  return out


def test_never_above_the_drivers_speed_nor_below_the_floor():
  random.seed(7)
  for metric in (False, True):
    f = mf.ModelFollower(metric)
    driver = 45 * MPH
    v = 20.0
    for k in range(6000):
      if k % 700 == 0:
        driver = random.choice([25, 35, 45, 60]) * MPH
      a = random.uniform(-4.0, 2.5)
      plan = [max(v + random.uniform(-3, 3) * t, 0.0) for t in T_IDXS] if k % 3 else None
      s = f.update(v, driver, True, a, T_IDXS if plan else None, plan)
      assert mf.FLOOR_SPEED - 1e-6 <= s <= driver + 1e-6, (k, s, driver)
      v = min(max(v + random.uniform(-1, 1), 0.0), 35.0)


def test_floor_and_low_driver_speed():
  f = mf.ModelFollower()
  out = run(f, 100, 9.0, 45 * MPH, -3.0)
  assert abs(out[-1][0] - mf.FLOOR_SPEED) < 1e-6            # a hard stop: the car's own minimum, not lower
  f = mf.ModelFollower()
  out = run(f, 100, 9.0, mf.FLOOR_SPEED, -3.0)                 # the driver at the minimum: nothing to lower
  assert all(abs(s - mf.FLOOR_SPEED) < 1e-6 and p == 0 for s, _, p, _ in out)


def test_no_churn_on_noisy_input():
  random.seed(3)
  f = mf.ModelFollower()
  out = run(f, 4000, 45 * MPH, 45 * MPH, lambda k: random.gauss(0.0, 0.15))   # 200 s cruising, a jittery model
  assert f.changes == 0 and f.taps == 0 and all(r == "idle" for _, r, _, _ in out)
  # held low for 60 s with a noisy model (the qlogs: the accel moves a median 0.04-0.07 m/s^2 per 0.5 s): the first
  # drop and a step or two; a much noisier model (0.25 every 50 ms) still never goes back and forth quicker than
  # REVERSE_GAP nor changes quicker than MIN_CHANGE_GAP
  for sigma, most in ((0.1, 3), (0.25, 20)):
    f = mf.ModelFollower()
    out = run(f, 1200, 15.0, 45 * MPH, lambda k, s=sigma: random.gauss(-0.6, s))
    dirs = [(k, d) for k, (_, _, p, d) in enumerate(out) if p]
    assert 0 < len(dirs) <= most, (sigma, dirs)
    for (k0, d0), (k1, d1) in zip(dirs, dirs[1:], strict=False):
      assert (k1 - k0) * f.dt >= mf.MIN_CHANGE_GAP - 1e-6
      if d0 != d1:
        assert (k1 - k0) * f.dt >= mf.REVERSE_GAP - 1e-6


def test_slows_with_the_model_then_recovers_to_the_drivers_speed():
  f = mf.ModelFollower()
  v = 40 * MPH
  out = run(f, 9, v, 45 * MPH, -1.0)
  assert all(r == "idle" for _, r, _, _ in out)                # START_TIME not reached yet
  out = run(f, 40, v, 45 * MPH, -1.0)
  s = out[-1][0]
  # planned speed after the look-ahead minus the gap that makes the cruise slow at 1 m/s^2
  want = v - 1.0 * mf.LOOK_AHEAD - 1.0 / mf.CRUISE_GAIN
  assert abs(s - f.quantize(want)) < 1e-6 and abs(s / MPH - round(s / MPH)) < 1e-6
  assert any(r == "slowing" for _, r, _, _ in out) and f.lowers == 1
  assert sum(p for _, _, p, _ in out) == round((45 * MPH - s) / MPH)
  # the model speeds up again: the follower goes up, by the gap the cruise needs for that accel, to the driver's speed
  out = run(f, 200, 25 * MPH, 45 * MPH, 1.2)
  assert any(r == "recovering" for _, r, _, _ in out) and out[-1][1] == "idle" and abs(out[-1][0] - 45 * MPH) < 1e-6


def test_the_plan_leads_a_stop_the_accel_does_not_show_yet():
  def stop_plan(v):
    return [v if t < 3.0 else max(v - 3.0 * (t - 3.0), 0.0) for t in T_IDXS]   # a hard stop planned from 3 s out
  f = mf.ModelFollower()
  run(f, 30, 20.0, 45 * MPH, 0.0, plan=stop_plan)
  assert f.engaged and f.used_plan and f.cmd < 45 * MPH - 5 * MPH
  f = mf.ModelFollower()
  run(f, 30, 20.0, 45 * MPH, 0.0)
  assert not f.engaged and f.reason == "idle"


def test_lets_go_when_the_model_is_steady_above_20_mph():
  f = mf.ModelFollower()
  run(f, 40, 30 * MPH, 45 * MPH, -0.8)
  assert f.engaged
  out = run(f, int(mf.STEADY_TIME / f.dt) + 40, 30 * MPH, 45 * MPH, 0.0)
  assert out[-1][1] == "idle" and abs(out[-1][0] - 45 * MPH) < 1e-6
  # in a slow queue it keeps the low set speed
  f = mf.ModelFollower()
  run(f, 40, 10 * MPH, 45 * MPH, -0.8)
  out = run(f, 200, 10 * MPH, 45 * MPH, 0.0)
  assert f.engaged and out[-1][0] < 20 * MPH


def test_metric_grid_and_presses():
  f = mf.ModelFollower(is_metric=True)
  out = run(f, 60, 60 * KPH, 70 * KPH, -1.0)
  s = out[-1][0]
  assert abs(s * 3.6 - round(s * 3.6)) < 1e-6 and s < 70 * KPH
  assert f.quantum == 8 and f.press_estimate(70 * KPH, 30 * KPH) == (40, 1)
  imp = mf.ModelFollower()
  assert imp.press_estimate(45 * MPH, 20 * MPH) == (25, 1)        # one long press 44, 40, ... 20
  assert imp.press_estimate(45 * MPH, 22 * MPH) == (23, 4)        # a long press to 25, then 3 taps
  assert imp.press_estimate(19 * MPH, mf.FLOOR_SPEED) == (1, 1)   # 19 -> 18.64: one tap
  assert imp.press_estimate(20 * MPH, 45 * MPH) == (25, 1)
  assert imp.press_estimate(43 * MPH, 45 * MPH) == (2, 2)


def test_cruise_off_or_no_model_is_nothing():
  f = mf.ModelFollower()
  out = run(f, 50, 20.0, 45 * MPH, -2.0, cruise=False)
  assert all(s == 0.0 and r == "off" and p == 0 for s, r, p, _ in out)
  run(f, 40, 20.0, 45 * MPH, -2.0)
  assert f.engaged
  assert f.update(20.0, 45 * MPH, False, -2.0) == 0.0 and not f.engaged and f.reason == "off"
  assert f.update(20.0, 45 * MPH, True, None) == 0.0 and f.reason == "off"
  assert f.update(20.0, 45 * MPH, True, float("nan")) == 0.0
  assert f.update(20.0, 0.0, True, -2.0) == 0.0
  # a malformed plan falls back to the accel
  assert f.update(20.0, 45 * MPH, True, 0.0, [0.0, 1.0], [20.0]) == 45 * MPH and not f.used_plan


class _Params:
  def __init__(self, v=None, missing=False):
    self.v, self.missing = v, missing

  def get(self, key, encoding=None):
    if self.missing:
      raise KeyError(key)        # UnknownKeyName on a device whose params build does not have the key yet
    return self.v


def test_param_default_is_test_and_nothing_can_turn_it_on(tmp_path):
  assert mf.mode_from_params(_Params(missing=True)) == mf.TEST
  for v, want in ((None, 1), ("", 1), ("0", 0), (0, 0), ("1", 1), (1, 1), (2, 1), ("2", 1), ("x", 1), (b"0", 0)):
    assert mf.mode_from_params(_Params(v)) == want, v
  keys = open(os.path.join(ROOT, "common", "params_keys.h")).read()
  line = next(ln for ln in keys.splitlines() if '"MazdaModelFollow"' in ln)
  assert '{PERSISTENT, INT, "1"' in line
  ui = os.path.join(ROOT, "selfdrive", "ui", "layouts", "settings", "starpilot", "vehicle.py")
  py_compile.compile(ui, doraise=True, cfile=str(tmp_path / "vehicle.pyc"))
  text = open(ui).read()
  assert 'choice("MazdaModelFollow", (tr_noop("Off"), tr_noop("Test")), 1)' in text


def test_the_module_cannot_press():
  src = open(mf.__file__).read()
  for word in ("Buttons", "mazdacan", "create_button_cmd", "packer", "can_sends"):
    assert word not in src


# ---- episodes ----
def ep_update(tr, t, follow, actual, cruise=True, v=20.0, brake=False, driver=45 * MPH, features=""):
  return tr.update(t, cruise, v, 0.0, follow, actual, driver, -1.0, "slowing", 0, 0, True, 40.0, 5.0, 0.9, False, features,
                   brake, False, False)


def test_episode_open_close_and_after_window():
  tr = mf.EpisodeTracker()
  acts = []
  t = 0.0
  for _ in range(20):                                          # 2 mph apart: nothing
    acts += ep_update(tr, t, 43 * MPH, 45 * MPH) or []
    t += 0.05
  assert not acts
  t_low = t
  while t < t_low + 1.2:                                       # 10 mph apart for over a second: open
    acts += ep_update(tr, t, 35 * MPH, 45 * MPH, features="stop:stopped car") or []
    t += 0.05
  assert [a[0] for a in acts] == ["open"]
  s = acts[0][1]
  assert s["kind"] == "follower_lower" and abs(s["follow_mph"] - 35) < 1e-6 and s["features"] == "stop:stopped car"
  assert s["follow_lower_since_s"] == -1.0 and s["lead"] and abs(s["lead_d"] - 40.0) < 1e-6
  for _ in range(20):
    acts += ep_update(tr, t, 30 * MPH, 45 * MPH, brake=True) or []
    t += 0.05
  while t < t_low + 6.0:                                       # back within 1 mph: ends 2 s later, written AFTER s on
    acts += ep_update(tr, t, 45 * MPH, 45 * MPH, v=8.0) or []
    t += 0.05
  assert [a[0] for a in acts] == ["open"]
  while t < t_low + 20.0:
    acts += ep_update(tr, t, 45 * MPH, 45 * MPH, v=5.0) or []
    t += 0.05
  assert [a[0] for a in acts] == ["open", "close"]
  c = acts[1][1]
  assert c["ended_by"] == "back within 1 mph" and c["driver_braked"] and abs(c["follow_min_mph"] - 30) < 1e-6
  assert abs(c["v_min_after_mph"] - 5.0 / MPH) < 1e-6 and c["follow_lower_at_s"] == -1.0 and c["actual_lower_at_s"] is None
  assert tr.episodes == 1 and tr.follower_lower == 1 and tr.written == 1


def test_episode_ends_with_cruise_off_and_is_capped_per_drive():
  tr = mf.EpisodeTracker()
  tr.MAX_PER_DRIVE = 2
  acts = []
  t = 0.0
  for _ in range(4):
    for _ in range(30):
      acts += ep_update(tr, t, 45 * MPH, 30 * MPH) or []        # our real set speed lower: "actual_lower"
      t += 0.05
    for _ in range(10):
      acts += ep_update(tr, t, 0.0, 0.0, cruise=False) or []
      t += 0.05
    for _ in range(200):
      acts += ep_update(tr, t, 45 * MPH, 45 * MPH) or []
      t += 0.05
  assert [a[0] for a in acts] == ["open", "close", "open", "close"]
  assert acts[0][1]["kind"] == "actual_lower" and acts[1][1]["ended_by"] == "cruise off"
  assert tr.episodes == 4 and tr.written == 2 and tr.skipped == 2 and tr.actual_lower == 4


# ---- the card wrapper (needs the native-module stubs: run with -p stub_native) ----
def _wrapper(tmp_path, monkeypatch, follow_mode):
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  import importlib.util
  from types import SimpleNamespace
  import time
  import cereal.messaging as messaging
  from opendbc.car.mazda import curve_slowdown as cs
  from opendbc.car.mazda import car_profile as cp
  from opendbc.car.mazda import mazdacan
  from opendbc.car.mazda.values import MazdaSafetyFlags
  spec = importlib.util.spec_from_file_location("tcpw_mf", os.path.join(os.path.dirname(__file__), "test_car_profile_wrapper.py"))
  W = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(W)
  sm = W.FakeSM()
  monkeypatch.setattr(messaging, "SubMaster", lambda services: sm)
  monkeypatch.setattr(cp, "PROFILE_PATH", str(tmp_path / "profile.json"))
  monkeypatch.setattr(cp.CarProfile.__init__, "__defaults__", (str(tmp_path / "profile.json"), "", False, time.monotonic))
  monkeypatch.setattr(mazdacan, "create_button_cmd", lambda *a, **k: "button")
  monkeypatch.setattr(cs.CurveSlowdown, "_learning_dir", classmethod(lambda cls: str(tmp_path / "events")))
  CP = SimpleNamespace(flags=MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR, carFingerprint="MAZDA_CX5", maxLateralAccel=1.09)
  c = cs.CurveSlowdown(CP, W.FakeParams({"IsMetric": False}), curve_enabled=False, stop_mode=2, learning=False, map_mode=0,
                       limit_mode=0, follow_mode=follow_mode)
  opened, closed = [], []
  if c.recorder is not None:
    c.recorder.root = str(tmp_path / "events")
  if getattr(c, "follow_rec", None) is not None:
    c.follow_rec.root = str(tmp_path / "events")
    monkeypatch.setattr(c.follow_rec, "open", lambda kind, t, s, **kw: (opened.append((kind, dict(s))), "ev")[1])
    monkeypatch.setattr(c.follow_rec, "close", lambda ev, t, s=None: closed.append(dict(s or {})))
  return c, sm, W, opened, closed


def _scenario(c, sm, W):
  """Cruise at 40 under 45, the model wants -1.2 m/s^2 for 3 s, then nothing; then cruise off for 10 s."""
  out = []
  for k in range(1800):
    sm.msgs["modelV2"].action.desiredAcceleration = -1.2 if 100 <= k < 400 else 0.0
    out.append(c.update(W.CC, W.car_state(40, 45, cruise=k < 800), None, None, False))
  return out


def test_wrapper_logs_episodes_and_never_changes_what_is_sent(tmp_path, monkeypatch):
  c0, sm0, W, _, _ = _wrapper(tmp_path, monkeypatch, follow_mode=0)
  assert c0.follow is None
  base = _scenario(c0, sm0, W)
  c, sm, W, opened, closed = _wrapper(tmp_path, monkeypatch, follow_mode=1)
  assert c.follow is not None
  out = _scenario(c, sm, W)
  assert out == base                                           # Test only: the buttons sent are the same
  assert c.follow.lowers == 1 and c.follow.taps > 0
  assert [k for k, _ in opened] == ["modelFollow"] and opened[0][1]["kind"] == "follower_lower"
  assert opened[0][1]["src"] == "v2" and opened[0][1]["mode"] == "test"
  assert len(closed) == 1 and closed[0]["ended_by"] in ("back within 1 mph", "cruise off")
  path = tmp_path / "events" / c.FOLLOW_SUMMARY_CSV           # the drive summary line, written at cruise off
  for _ in range(200):
    if path.exists():
      break
    import time
    time.sleep(0.01)
  lines = path.read_text().splitlines()
  assert lines[0].split(",") == c.FOLLOW_SUMMARY_COLUMNS and len(lines) == 2
  row = dict(zip(c.FOLLOW_SUMMARY_COLUMNS, lines[1].split(","), strict=True))
  assert row["mode"] == "test" and int(row["lowers"]) == 1 and int(row["episodes"]) == 1 and float(row["call_avg_us"]) > 0


def test_wrapper_survives_an_exception_in_the_follower(tmp_path, monkeypatch):
  c0, sm0, W, _, _ = _wrapper(tmp_path, monkeypatch, follow_mode=0)
  base = _scenario(c0, sm0, W)
  c, sm, W, _, _ = _wrapper(tmp_path, monkeypatch, follow_mode=1)
  records = []
  monkeypatch.setattr(c.recorder, "record", lambda kind, t, data, **kw: records.append((kind, data)))
  calls = []

  def boom(*a, **k):
    calls.append(1)
    raise ValueError("bad model")
  monkeypatch.setattr(c.follow, "update", boom)
  out = _scenario(c, sm, W)
  assert out == base and c.follow is None and len(calls) == 1
  assert [k for k, _ in records if k == "modelFollowError"] == ["modelFollowError"]
  assert "bad model" in c.follow_error


def test_follower_alone_never_sends_a_button(tmp_path, monkeypatch):
  # the default on a car with every pressing feature off: the wrapper exists only for the follower's logs
  c, sm, W, opened, _ = _wrapper(tmp_path, monkeypatch, follow_mode=1)
  c.stop_mode = 0
  c.curve_enabled = False
  sent = []
  for k in range(3000):
    sm.msgs["modelV2"].action.desiredAcceleration = -2.0 if (k // 300) % 2 else 0.5
    set_mph = 45 if k < 1500 else 40
    sent += c.update(W.CC, W.car_state(38, set_mph, cruise=(k % 1000) < 900, set_minus=1500 <= k < 1510, brake=2900 <= k),
                     None, None, False)
  assert sent == [] and c.follow is not None and c.follow.lowers > 0 and opened
