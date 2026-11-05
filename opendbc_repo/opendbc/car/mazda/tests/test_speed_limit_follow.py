"""Speed limit follow (drive 4c): the follower's offers and the set speed controller raising to a higher limit."""
import importlib.util
import os

_here = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location("speed_limit_follow", os.path.join(_here, "..", "speed_limit_follow.py"))
slf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(slf)
_spec_t = importlib.util.spec_from_file_location("test_curve_slowdown", os.path.join(_here, "test_curve_slowdown.py"))
T = importlib.util.module_from_spec(_spec_t)
_spec_t.loader.exec_module(T)
cs, MPH = T.cs, T.MPH
NO_CURVE = lambda f: None  # noqa: E731
STABLE = int(slf.LIMIT_STABLE_TIME / slf.DT_CTRL)


def _li(limit_mph, driver_mph=45.0, source="Map Data", gps_trust=0, cruise=True):
  return slf.LimitInputs(limit=limit_mph * MPH, source=source, gps_trust=gps_trust,
                         driver_speed=driver_mph * MPH if cruise else 0.0, cruise_enabled=cruise)


def _settled(limit_mph=45.0, **kw):
  f = slf.SpeedLimitFollower(False)
  for _ in range(STABLE + 5):
    assert f.update(_li(limit_mph, **kw)) == 0.0
  return f


def _feed(f, n, *a, **kw):
  return [f.update(_li(*a, **kw)) for _ in range(n)]


def test_limit_must_be_stable_for_3_s():
  f = _settled(45)
  out = _feed(f, STABLE + 10, 50)
  assert all(x == 0.0 for x in out[:STABLE - 1])
  assert abs(out[-1] / MPH - 50) < 1e-6


def test_only_when_the_drivers_speed_matches_the_old_limit():
  f = _settled(45, driver_mph=47)
  assert _feed(f, STABLE + 10, 50, driver_mph=47)[-1] == 0.0
  assert f.events and "not the old limit" in f.events[-1][1]["reason"]
  f = _settled(45)
  assert abs(_feed(f, STABLE + 10, 50)[-1] / MPH - 50) < 1e-6


def test_raise_only_and_plausibility():
  for old, new, kw in ((50, 45, {}), (45, 80, {}), (45, 48, {}), (50, 85, {}), (45, 50, dict(source="None")),
                       (45, 50, dict(source="Vision")), (45, 50, dict(source="Previous Limit")),
                       (45, 50, dict(gps_trust=2))):
    f = _settled(old, **{k: v for k, v in kw.items() if k == "gps_trust"})
    assert _feed(f, STABLE + 10, new, **kw)[-1] == 0.0, (old, new, kw)
  f = _settled(45, source="Dashboard", gps_trust=3)            # the dashboard does not depend on the GPS
  assert _feed(f, STABLE + 10, 50, source="Dashboard", gps_trust=3)[-1] > 0
  fm = slf.SpeedLimitFollower(True)
  kph = 1 / 3.6
  for _ in range(STABLE + 5):
    fm.update(slf.LimitInputs(80 * kph, "Map Data", 0, 80 * kph, True))
  out = [fm.update(slf.LimitInputs(100 * kph, "Map Data", 0, 80 * kph, True)) for _ in range(STABLE + 10)]
  assert abs(out[-1] / kph - 100) < 1e-6
  fm = slf.SpeedLimitFollower(True)
  for _ in range(STABLE + 5):
    fm.update(slf.LimitInputs(80 * kph, "Map Data", 0, 80 * kph, True))
  out = [fm.update(slf.LimitInputs(95 * kph, "Map Data", 0, 80 * kph, True)) for _ in range(STABLE + 10)]
  assert out[-1] == 0.0


def test_offer_ends_when_adopted_declined_or_old():
  f = _settled(45)
  _feed(f, STABLE + 2, 50)
  f.mark_adopted()
  assert _feed(f, 500, 50, driver_mph=45)[-1] == 0.0             # no second offer for the same change
  f = _settled(45)
  _feed(f, STABLE + 2, 50)
  assert _feed(f, 5, 50, driver_mph=47)[-1] == 0.0               # the driver chose 47: theirs
  assert _feed(f, 50, 50, driver_mph=45)[-1] == 0.0              # ... for good, even back at 45
  f = _settled(45)
  _feed(f, STABLE + 2, 50, cruise=False)
  out = _feed(f, int(slf.LIMIT_OFFER_TIME / slf.DT_CTRL) + 10, 50, cruise=False)
  assert out[-1] == 0.0 and f.offer is None
  assert _feed(f, 10, 50)[-1] == 0.0                              # too late


def test_old_limit_forgotten_after_30_s_unknown():
  f = _settled(45)
  _feed(f, int(slf.LIMIT_OLD_MAX_AGE / slf.DT_CTRL) + 100, 0, source="None")
  assert _feed(f, STABLE + 10, 50)[-1] == 0.0


def test_flapping_limits_never_raise():
  f = _settled(45)
  for k in range(30):
    out = _feed(f, 100, 50 if k % 2 == 0 else 45)
    assert all(x == 0.0 for x in out)


def test_cruise_off_waits_for_the_drivers_speed():
  # drive 4c: the limit changed 2 s before cruise came back (driver's speed 45 = the old limit)
  f = _settled(45)
  assert all(x == 0.0 for x in _feed(f, STABLE + 200, 50, cruise=False))
  assert abs(_feed(f, 1, 50)[-1] / MPH - 50) < 1e-6
  f = _settled(45)
  _feed(f, STABLE + 200, 50, cruise=False)
  assert _feed(f, 5, 50, driver_mph=30)[-1] == 0.0                # back at a speed the driver set: not ours
  assert _feed(f, 5, 50)[-1] == 0.0


# ---- the set speed controller (simulator) ----

def _limit_from(f0, mph):
  return lambda f: mph * MPH if f >= f0 else 0.0


def test_limit_raise_of_5_mph_is_restored_by_taps():
  r = T.run(T.Car(45, dash_offset_mph=0.0), NO_CURVE, 2000, limit=_limit_from(200, 50))
  assert r["holds"] == [] and len(r["presses"]) == 5 and all(b == cs.SET_PLUS for _, b in r["presses"])
  assert abs(r["final"] - 50) < 0.01 and r["max"] <= 50 + 1e-6
  assert r["logic"].anchor is None


def test_limit_raise_of_25_mph_holds_to_5_below_then_taps():
  r = T.run(T.Car(45, dash_offset_mph=0.0), NO_CURVE, 2000, limit=_limit_from(200, 70))
  ups = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS]
  assert len(ups) == 1 and abs(ups[0]["bound_mph"] - 65) < 1e-6
  s = r["sets"]
  changes = [round(s[k] / MPH, 1) for k in range(1, len(s)) if abs(s[k] - s[k - 1]) > 1e-6]
  assert changes[-6:] == [65.0, 66.0, 67.0, 68.0, 69.0, 70.0], changes
  back = next(k for k, x in enumerate(s) if abs(x / MPH - 70) < 0.01)
  assert (back - 200) * T.DT <= 8.0 and r["max"] <= 70 + 1e-6


def test_drive_4c_restore_after_the_brake_continues_to_the_new_limit():
  spec = importlib.util.spec_from_file_location("tscs", os.path.join(_here, "test_stopped_car_slowdown.py"))
  S = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(S)
  r = T.run(T.Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, events={330: S._brake, 600: S._resume},
            lead=S._Switch(330, 100), max_drop=10, limit=_limit_from(700, 50))
  after = [f for f, b in r["presses"] if b == cs.SET_MINUS and f > 600]
  assert after == []
  assert abs(r["final"] - 50) < 0.01 and r["max"] <= 50 + 1e-6
  back = next(k for k, x in enumerate(r["sets"]) if k > 600 and abs(x / MPH - 50) < 0.01)
  assert (back - 600) * T.DT <= 9.0, back


def test_driver_buttons_win_over_a_limit_raise():
  def driver_minus(s, c):
    c.press(cs.SET_MINUS, who="driver")
  # (the follower closes its offer once taken, so the raise is offered for a moment only)
  window = lambda f: 70 * MPH if 200 <= f < 260 else 0.0  # noqa: E731
  r = T.run(T.Car(45, dash_offset_mph=0.0), NO_CURVE, 2500, events={330: driver_minus}, limit=window)
  assert r["max"] <= 70 + 1e-6
  ours_after = [f for f, b in r["presses"] if f > 345] + [f for f in r["holds"] if f > 345]
  assert ours_after == [], ours_after
  assert r["logic"].anchor is None and r["final"] < 70
  # a driver tap just before: the raise waits LIMIT_DRIVER_QUIET

  def driver_plus(s, c):
    c.press(cs.SET_PLUS, who="driver")
  r = T.run(T.Car(45, dash_offset_mph=0.0), NO_CURVE, 2000, events={150: driver_plus}, limit=_limit_from(200, 51))
  first = min([f for f, _ in r["presses"]] + r["holds"] or [10_000])
  assert first >= 150 + int(cs.LIMIT_DRIVER_QUIET / T.DT) - 5, first


def test_no_raise_while_a_curve_is_active():
  r = T.run(T.Car(45, dash_offset_mph=0.0), T.between(100, 900, 38), 3500, limit=_limit_from(300, 50), max_drop=10)
  ups_during = [f for f, b in r["presses"] if b == cs.SET_PLUS and f < 900]
  assert ups_during == []
  assert abs(r["final"] - 50) < 0.01 and r["max"] <= 50 + 1e-6


def test_limit_raise_never_above_the_limit_for_any_overrun():
  for latency in (0.0, 0.3, 0.6):
    for phase in (0, None):
      r = T.run(T.Car(45, dash_offset_mph=0.0, hold_latency=latency, lose=lambda n: n % 4 == 0), NO_CURVE, 3000,
                limit=_limit_from(200, 70), wheel_phase=phase)
      assert r["max"] <= 70 + 1e-6, (latency, phase, r["max"])
      assert abs(r["final"] - 70) < 0.01, (latency, phase, r["final"])
