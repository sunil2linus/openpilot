"""Speed limit follow (drives 4c, 4d, 52): the follower's offers (a higher limit, a first limit, a lower limit's cap) and
the set speed controller taking them."""
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
RAISE = int(slf.LIMIT_RAISE_STABLE_TIME / slf.DT_CTRL)        # a higher limit (rules 1, 2)
QUICK = int(slf.LIMIT_RISE_QUICK_TIME / slf.DT_CTRL)          # a one-band rise with GPS trust 0 (F)


def _li(limit_mph, driver_mph=45.0, source="Map Data", gps_trust=0, cruise=True):
  return slf.LimitInputs(limit=limit_mph * MPH, source=source, gps_trust=gps_trust,
                         driver_speed=driver_mph * MPH if cruise else 0.0, cruise_enabled=cruise)


def _settled(limit_mph=45.0, **kw):
  """The limit known before cruise comes on (so not a first limit to follow), then cruise on at the driver's speed."""
  f = slf.SpeedLimitFollower(False)
  for _ in range(STABLE + 5):
    assert f.update(_li(limit_mph, **dict(kw, cruise=False))) == 0.0
  for _ in range(5):
    assert f.update(_li(limit_mph, **kw)) == 0.0
  f.events = []
  return f


def _feed(f, n, *a, **kw):
  return [f.update(_li(*a, **kw)) for _ in range(n)]


def test_a_higher_limit_must_be_stable_for_5_s():
  # (F, owner after drives 54 / 55) a one-band rise with GPS trust 0 on every frame: 2.5 s
  f = _settled(45)
  out = _feed(f, RAISE + 10, 50)
  assert all(x == 0.0 for x in out[:QUICK - 1]) and abs(out[QUICK + 5] / MPH - 50) < 1e-6
  assert abs(out[-1] / MPH - 50) < 1e-6
  # trust 1 on one frame of it, a big rise, a first limit: 5 s
  f = _settled(45)
  out = _feed(f, 1, 50, gps_trust=1) + _feed(f, RAISE + 10, 50)
  assert all(x == 0.0 for x in out[:RAISE - 1]) and abs(out[-1] / MPH - 50) < 1e-6
  f = _settled(45)
  out = _feed(f, RAISE + 10, 60)
  assert all(x == 0.0 for x in out[:RAISE - 1]) and abs(out[-1] / MPH - 60) < 1e-6
  f = _fresh(2000)
  out = _feed(f, RAISE + 10, 45, driver_mph=35)
  assert all(x == 0.0 for x in out[:RAISE - 1]) and abs(out[-1] / MPH - 45) < 1e-6


def test_only_when_the_drivers_speed_is_at_the_old_limit_or_up_to_5_above():
  # rule 1's band: old .. old + 5 mph; always to the new limit itself
  for driver, follows in ((45, True), (47, True), (50, True), (44, False), (52, False), (40, False)):
    f = _settled(45, driver_mph=driver)
    out = _feed(f, RAISE + 10, 55, driver_mph=driver)[-1]
    assert (abs(out / MPH - 55) < 1e-6) if follows else out == 0.0, (driver, out)
    if not follows:
      assert f.events and "not the old limit or up to 5 above" in f.events[-1][1]["reason"], driver
  # metric: 10 km/h
  kph = 1 / 3.6
  for driver, follows in ((80, True), (90, True), (95, False)):
    fm = slf.SpeedLimitFollower(True)
    for _ in range(STABLE + 5):
      fm.update(slf.LimitInputs(80 * kph, "Map Data", 0, driver * kph, True))
    out = [fm.update(slf.LimitInputs(100 * kph, "Map Data", 0, driver * kph, True)) for _ in range(RAISE + 10)][-1]
    assert (abs(out / kph - 100) < 1e-6) if follows else out == 0.0, (driver, out)


def test_drive_52_tag_318_at_50_on_a_45_road_the_55_is_followed_to_55():
  # he set 50 himself on the 45 road (t 230); the limit went 45 -> 55 (t 310.3); he raised 50 -> 55 by hand (t 321)
  f = _settled(45, driver_mph=50)
  out = _feed(f, RAISE + 10, 55, driver_mph=50)
  assert all(x == 0.0 for x in out[:QUICK - 1])
  assert abs(out[-1] / MPH - 55) < 1e-6                           # the new limit, not 55 + his 5
  assert f.offer.old == 45 and f.offer.kind == "raise"
  # once he is at 55 (our raise, or his own SET+ first) the offer is closed, quietly
  assert _feed(f, 5, 55, driver_mph=55)[-1] == 0.0 and f.offer is None and not f.events


def test_a_change_of_the_drivers_speed_inside_the_band_while_offered_is_his():
  f = _settled(45)
  _feed(f, RAISE + 2, 55)
  assert _feed(f, 5, 55, driver_mph=48)[-1] == 0.0 and f.offer is None
  assert "changed his speed to 48" in f.events[-1][1]["reason"]


def test_raise_only_and_plausibility():
  for old, new, kw in ((50, 45, {}), (45, 80, {}), (45, 48, {}), (50, 85, {}), (45, 50, dict(source="None")),
                       (45, 50, dict(source="Vision")), (45, 50, dict(source="Previous Limit")),
                       (45, 50, dict(gps_trust=2))):
    f = _settled(old, **{k: v for k, v in kw.items() if k == "gps_trust"})
    assert _feed(f, RAISE + 10, new, **kw)[-1] == 0.0, (old, new, kw)
  f = _settled(45, source="Dashboard", gps_trust=3)            # the dashboard does not depend on the GPS
  assert _feed(f, RAISE + 10, 50, source="Dashboard", gps_trust=3)[-1] > 0
  fm = slf.SpeedLimitFollower(True)
  kph = 1 / 3.6
  for _ in range(STABLE + 5):
    fm.update(slf.LimitInputs(80 * kph, "Map Data", 0, 80 * kph, True))
  out = [fm.update(slf.LimitInputs(100 * kph, "Map Data", 0, 80 * kph, True)) for _ in range(RAISE + 10)]
  assert abs(out[-1] / kph - 100) < 1e-6
  fm = slf.SpeedLimitFollower(True)
  for _ in range(STABLE + 5):
    fm.update(slf.LimitInputs(80 * kph, "Map Data", 0, 80 * kph, True))
  out = [fm.update(slf.LimitInputs(95 * kph, "Map Data", 0, 80 * kph, True)) for _ in range(RAISE + 10)]
  assert out[-1] == 0.0


def test_offer_ends_when_adopted_declined_or_old():
  f = _settled(45)
  _feed(f, RAISE + 2, 50)
  f.mark_adopted()
  assert _feed(f, 500, 50, driver_mph=45)[-1] == 0.0             # no second offer for the same change
  f = _settled(45)
  _feed(f, RAISE + 2, 50)
  assert _feed(f, 5, 50, driver_mph=47)[-1] == 0.0               # the driver chose 47: theirs
  assert _feed(f, 50, 50, driver_mph=45)[-1] == 0.0              # ... for good, even back at 45
  f = _settled(45)
  _feed(f, RAISE + 2, 50)                                        # offered, then cruise off for longer than the offer
  out = _feed(f, int(slf.LIMIT_OFFER_TIME / slf.DT_CTRL) + 10, 50, cruise=False)
  assert out[-1] == 0.0 and f.offer is None and "not taken in time" in f.events[-1][1]["reason"]
  assert _feed(f, 10, 50)[-1] == 0.0                              # too late


def test_drive_4d_a_cross_road_limit_while_waiting_at_the_light_does_not_hide_the_old_one():
  # 45 (the driver's speed) -> 40 for 28 s at the light (the cross road) -> 55 after the turn: offered 45 -> 55
  f = _settled(45)
  _feed(f, 2800, 40)
  out = _feed(f, RAISE + 10, 55)
  assert abs(out[-1] / MPH - 55) < 1e-6
  assert f.offer.old == 45
  # ... but not when the driver's speed was last at 45 too long ago (and is not in the band of the cross road's 35)
  f = _settled(45)
  _feed(f, int(slf.LIMIT_MATCH_MAX_AGE / slf.DT_CTRL) + 100, 35)
  assert _feed(f, RAISE + 10, 55)[-1] == 0.0
  # (with the cross road at 40, 45 is in 40's band: after 90 s that is the old limit, and the 55 is followed from it)
  f = _settled(45)
  _feed(f, int(slf.LIMIT_MATCH_MAX_AGE / slf.DT_CTRL) + 100, 40)
  assert abs(_feed(f, RAISE + 10, 55)[-1] / MPH - 55) < 1e-6 and f.offer.old == 40
  # and a driver who set 40 on the 40 road is matched to 40: a 40 -> 55 rise is offered from 40
  f = _settled(45)
  _feed(f, 2800, 40, driver_mph=40)
  assert abs(_feed(f, RAISE + 10, 55, driver_mph=40)[-1] / MPH - 55) < 1e-6


def test_old_limit_forgotten_after_30_s_unknown():
  # at 50 on a 45 road, 31 s with no limit, then 70: as a rise from 45 it would be followed (band, jump 25); after 30 s
  # the 45 is forgotten and the 70 is a first limit, 20 above the driver's speed: not followed
  f = _settled(45, driver_mph=50)
  _feed(f, int(slf.LIMIT_OLD_MAX_AGE / slf.DT_CTRL) + 100, 0, source="None", driver_mph=50)
  assert _feed(f, RAISE + 10, 70, driver_mph=50)[-1] == 0.0
  assert "more than 15 above" in f.events[-1][1]["reason"]


def test_flapping_limits_never_raise():
  f = _settled(45)
  for k in range(30):
    out = _feed(f, 100, 50 if k % 2 == 0 else 45)
    assert all(x == 0.0 for x in out)


def test_cruise_off_waits_for_the_drivers_speed():
  # drive 4c: the limit changed 2 s before cruise came back (driver's speed 45 = the old limit), steady with cruise on;
  # offered, then cruise off a while (a stop slowdown, his brake): taken once he is back at the speed it was offered for
  f = _settled(45)
  _feed(f, 200, 50, cruise=False)
  out = _feed(f, QUICK - 200 + 5, 50)
  assert all(x == 0.0 for x in out[:-6]) and abs(out[-1] / MPH - 50) < 1e-6 and abs(f.offer.driver - 45) < 1e-6
  assert all(x == 0.0 for x in _feed(f, 400, 50, cruise=False))
  assert abs(_feed(f, 1, 50)[-1] / MPH - 50) < 1e-6
  f = _settled(45)
  _feed(f, RAISE + 5, 50)
  _feed(f, 400, 50, cruise=False)
  assert _feed(f, 5, 50, driver_mph=30)[-1] == 0.0                # back at a speed the driver set: not ours
  assert _feed(f, 5, 50)[-1] == 0.0


def test_a_higher_limit_steady_while_cruise_is_off_is_not_offered():
  # review: the speed from the last time cruise was on is not the driver's speed now (RES would take it 0.3 s later)
  f = _settled(45)
  assert all(x == 0.0 for x in _feed(f, RAISE + 200, 50, cruise=False))
  assert f.offer is None and "cruise off" in f.events[-1][1]["reason"]
  assert all(x == 0.0 for x in _feed(f, 100, 50))                  # RES at the old speed: nothing


# ---- rule 2: a first limit after none ----

def _fresh(n, driver_mph=35.0, **kw):
  """A drive with no limit known for n frames (residential streets) at the driver's speed."""
  f = slf.SpeedLimitFollower(False)
  _feed(f, n, 0, source="None", driver_mph=driver_mph, **kw)
  return f


def test_drive_52_tag_211_a_first_limit_10_above_the_drivers_speed_is_followed():
  # slc 0 from t 5.7 to 187.3 (set 35 on the unmapped street), then a 45 road; he held SET+ 35 -> 45 at t 212.8
  f = _fresh(18_000)
  out = _feed(f, RAISE + 10, 45, driver_mph=35)
  assert all(x == 0.0 for x in out[:RAISE - 1])
  assert abs(out[-1] / MPH - 45) < 1e-6 and f.offer.kind == "first" and f.offer.old is None
  # since the drive started counts too
  f = slf.SpeedLimitFollower(False)
  assert abs(_feed(f, RAISE + 10, 45, driver_mph=35)[-1] / MPH - 45) < 1e-6
  # offered (cruise on at 187.3 and while it became steady), the cruise-off wait (a stop slowdown, his brake: t
  # 196-200.2) and back at his speed
  f = _fresh(18_000)
  _feed(f, RAISE + 2, 45, driver_mph=35)
  assert all(x == 0.0 for x in _feed(f, 400, 45, cruise=False))
  assert abs(_feed(f, 1, 45, driver_mph=35)[-1] / MPH - 45) < 1e-6
  # ... but back at another speed (he set it with cruise off): his
  f = _fresh(18_000)
  _feed(f, RAISE + 2, 45, driver_mph=35)
  _feed(f, 400, 45, cruise=False)
  assert _feed(f, 5, 45, driver_mph=38)[-1] == 0.0 and f.offer is None
  # metric: up to 25 km/h above
  kph = 1 / 3.6
  for new, follows in ((70, True), (80, False)):
    fm = slf.SpeedLimitFollower(True)
    for _ in range(2000):
      fm.update(slf.LimitInputs(0.0, "None", 0, 50 * kph, True))
    out = [fm.update(slf.LimitInputs(new * kph, "Map Data", 0, 50 * kph, True)) for _ in range(RAISE + 10)][-1]
    assert (abs(out / kph - new) < 1e-6) if follows else out == 0.0, (new, out)


def test_first_limit_not_followed():
  # 20 above the driver's speed
  f = _fresh(2000)
  assert _feed(f, RAISE + 10, 55, driver_mph=35)[-1] == 0.0 and "more than 15 above" in f.events[-1][1]["reason"]
  # 15 above is the most
  f = _fresh(2000)
  assert abs(_feed(f, RAISE + 10, 50, driver_mph=35)[-1] / MPH - 50) < 1e-6
  # a speed he chose after the new limit appeared, even inside 15
  f = _fresh(2000)
  _feed(f, 50, 45, driver_mph=35)
  assert _feed(f, RAISE + 10, 45, driver_mph=40)[-1] == 0.0 and "set after the limit appeared" in f.events[-1][1]["reason"]
  # ... and one he chose after it became stable, while it was offered
  f = _fresh(2000)
  _feed(f, RAISE + 2, 45, driver_mph=35)
  assert _feed(f, 5, 45, driver_mph=40)[-1] == 0.0 and f.offer is None
  # no driver's speed yet (cruise never on)
  f = _fresh(2000, cruise=False)
  assert _feed(f, RAISE + 10, 45, cruise=False)[-1] == 0.0 and _feed(f, 10, 45, driver_mph=35)[-1] == 0.0
  # the usual checks: GPS trust (0 for a first limit), not a multiple of 5, above 80 mph, and the offer's timeout
  for new, kw in ((45, dict(gps_trust=2)), (45, dict(gps_trust=1)), (42, {}), (85, dict(driver_mph=75))):
    kw = dict(dict(driver_mph=35), **kw)
    f = _fresh(2000, driver_mph=kw["driver_mph"])
    assert _feed(f, RAISE + 10, new, **kw)[-1] == 0.0, (new, kw)
  f = _fresh(2000)
  _feed(f, RAISE + 2, 45, driver_mph=35)
  _feed(f, int(slf.LIMIT_OFFER_TIME / slf.DT_CTRL) + 10, 45, cruise=False)
  assert _feed(f, 10, 45, driver_mph=35)[-1] == 0.0 and "not taken in time" in f.events[-1][1]["reason"]
  # at or below the driver's speed: nothing to raise, nothing logged
  f = _fresh(2000, driver_mph=50)
  assert _feed(f, RAISE + 10, 45, driver_mph=50)[-1] == 0.0 and not f.events


def test_a_first_limit_right_after_a_junction_flap_is_a_normal_change():
  # at 40 on a 45 road, the limit gone for 2 s (a junction), then 50: the 45 was seen 2 s ago, so this is a rise from
  # 45 (rule 1: 40 is not in its band), not a first limit (rule 2 would follow 40 -> 50)
  f = _settled(45, driver_mph=40)
  _feed(f, 200, 0, source="None", driver_mph=40)
  assert _feed(f, RAISE + 10, 50, driver_mph=40)[-1] == 0.0
  assert "not the old limit" in f.events[-1][1]["reason"]
  # with the 45 not seen for 30 s, but another limit flapping 5 s before: not quiet for 10 s, not a first limit either
  f = _settled(45, driver_mph=40)
  _feed(f, 3200, 0, source="None", driver_mph=40)
  _feed(f, 100, 35, driver_mph=40)
  _feed(f, 500, 0, source="None", driver_mph=40)
  assert _feed(f, RAISE + 10, 50, driver_mph=40)[-1] == 0.0
  assert "old limit not seen recently" in f.events[-1][1]["reason"]
  # ... while the same limit seen briefly before (the new road's own flap) does not count against it
  f = _fresh(2000, driver_mph=40)
  _feed(f, 100, 50, driver_mph=40)
  _feed(f, 200, 0, source="None", driver_mph=40)
  assert abs(_feed(f, RAISE + 10, 50, driver_mph=40)[-1] / MPH - 50) < 1e-6


def test_drive_4d_band_matched_keeps_the_limit_the_driver_is_closest_to():
  # at 50 on a 45 road (rule 1's band), the cross road 40 at the light (50 is outside its band), then 55
  f = _settled(45, driver_mph=50)
  _feed(f, 2800, 40, driver_mph=50)
  assert abs(_feed(f, RAISE + 10, 55, driver_mph=50)[-1] / MPH - 55) < 1e-6 and f.offer.old == 45


# ---- rule 3: a lower limit caps a pending restore ----

def test_drive_52_tag_450_a_lower_limit_offers_a_cap_until_taken():
  # at 50 on a 50 road (the driver's speed: our anchor 50, the set lowered to 45), cruise off, the limit 50 -> 45 (t 440.3),
  # cruise back at 442.4: the cap is decided with his speed then (never with the one from before cruise went off)
  f = _settled(50, driver_mph=50)
  caps = []
  for _ in range(STABLE + 50):
    f.update(_li(45, driver_mph=50, cruise=False))
    caps.append(f.cap_speed)
  assert not any(caps) and f.cap is not None and f.cap.old == 50 and f.cap.driver is None   # waits, nothing to cap yet
  f.update(_li(45, driver_mph=50))
  assert abs(f.cap_speed / MPH - 45) < 1e-6 and f.cap.driver == 50                         # open from the first frame back
  assert all(x == 0.0 for x in _feed(f, 100, 45, driver_mph=50)) and f.cap is not None     # never a raise
  _feed(f, 1, 45, driver_mph=45)                                                          # taken: 45 now
  assert f.cap is None and all(x == 0.0 for x in _feed(f, 100, 45, driver_mph=45)) and f.cap is None
  # a new limit closes it too (the cross road's limit gone again)
  f = _settled(50, driver_mph=50)
  _feed(f, STABLE + 5, 45, driver_mph=50, cruise=False)
  assert f.cap is not None
  _feed(f, STABLE + 5, 50, driver_mph=50, cruise=False)
  assert f.cap is None and f.offer is None
  # back at a speed outside the old limit's band (he set it with cruise off): no cap
  f = _settled(50, driver_mph=50)
  _feed(f, STABLE + 5, 45, driver_mph=50, cruise=False)
  _feed(f, 1, 45, driver_mph=57)
  assert f.cap is None and f.cap_speed == 0.0


def test_a_lower_limit_with_the_driver_outside_the_band_is_left_alone():
  for driver in (57, 47):
    f = _settled(50, driver_mph=driver)
    caps = []
    for _ in range(STABLE + 50):
      f.update(_li(45, driver_mph=driver))
      caps.append(f.cap_speed)
    assert not any(caps) and not f.events, driver
  # nor a cap for an implausible one (GPS trust low)
  f = _settled(50, driver_mph=50)
  caps = []
  for _ in range(STABLE + 50):
    f.update(_li(45, driver_mph=50, gps_trust=2))
    caps.append(f.cap_speed)
  assert not any(caps) and "GPS trust" in f.events[-1][1]["reason"]


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


# ---- the follower and the controller together (the wrapper's wiring, curve_slowdown.CurveSlowdown) ----

class Coupled:
  """Every frame the follower sees the posted limit slc(f) (mph, 0 none; trust(f) its GPS trust) and the controller's
  driver's speed (the anchor, else the set speed; 0 with cruise off); the controller gets its raise, cap and undo. An
  adoption closes the offer (the wrapper's _limit_bookkeeping, here on the next frame, before the follower runs).
  road(f, x): the map fields at frame f, x m driven (road_class, ramp_to_highway, next_limit in mph, next_distance);
  the driver's SET presses (the simulated car's driver button) are passed on."""
  def __init__(self, car, slc, metric=False, trust=lambda f: 0, road=None, street_mode=0):
    self.car, self.slc, self.trust, self.road = car, slc, trust, road
    # (the legal limit: these tests are of the rules themselves; the Street Speed offset: test_street_speed_*)
    self.f = slf.SpeedLimitFollower(metric, street_mode=street_mode)
    self.logic, self.st, self.frame = None, None, -1
    self.raise_, self.cap_, self.undo_, self.ceiling_ = 0.0, 0.0, 0.0, 0.0
    self.raises, self.caps, self.undos, self.kinds = [], [], [], []
    self.x = 0.0                                    # m driven

  def setup(self, logic, anchor_mph=None):
    self.logic = logic
    if anchor_mph is not None:                      # a restore pending from earlier (a stop, a curve)
      logic.anchor, logic.last_set_speed = anchor_mph * MPH, self.car.set_speed

  def events(self, extra=None):
    ev = dict(extra or {})
    first = ev.get(0)

    def grab(s, c):
      self.st = s
      if first is not None:
        first(s, c)
    ev[0] = grab
    return ev

  def _step(self, f):
    if f == self.frame:
      return
    self.frame = f
    lg = self.logic
    self.x += self.car.v_ego * slf.DT_CTRL
    if lg.limit_adopted is not None:
      self.kinds.append(self.f.offer.kind if self.f.offer is not None else "")
      self.f.mark_adopted()
      self.raises.append((f - 1, lg.limit_adopted[1] / MPH))
    if lg.limit_capped is not None:
      self.caps.append((f - 1, lg.limit_capped[1] / MPH))
    if self.f.cap is not None and self.f.cap.driver is not None and lg.limit_cap is None:
      self.f.mark_capped()                          # the controller holds no cap (any more): nothing to keep open
    if self.f.undo is not None:
      self.undos.append((f - 1, lg.limit_undone))   # dealt with on the frame it was offered (None: nothing of ours)
      self.f.mark_undone()
    cruise = self.st["cruise"] if self.st is not None else True
    drv = (lg.anchor if lg.anchor is not None else self.car.set_speed) if cruise else 0.0
    kw = {}
    if self.road is not None:
      kw = dict(self.road(f, self.x))
      kw["next_limit"] = kw.get("next_limit", 0.0) * MPH
    kw.update(v_ego=self.car.v_ego, driver_set=self.car.driver_button_now() is not None)
    self.raise_ = self.f.update(slf.LimitInputs(self.slc(f) * MPH, "Map Data", self.trust(f), drv, cruise, **kw))
    self.cap_ = self.f.cap_speed
    self.undo_ = self.f.undo_speed
    self.ceiling_ = self.f.ceiling_speed

  def limit(self, f):
    self._step(f)
    return self.raise_

  def cap(self, f):
    self._step(f)
    return self.cap_

  def undo(self, f):
    self._step(f)
    return self.undo_

  def ceiling(self, f):
    self._step(f)
    return self.ceiling_


def _brake_held(s, c):
  s["brake"] = True                                 # cruise on, nothing of ours goes out


def _cruise_off(s, c):
  s["cruise"], s["brake"] = False, True


def _resume_now(s, c):
  s["cruise"], s["brake"] = True, False


def _sim(car, slc, frames, events=None, anchor_mph=None, trust=lambda f: 0, road=None, **kw):
  c = Coupled(car, slc, trust=trust, road=road)
  r = T.run(car, kw.pop("curve", NO_CURVE), frames, events=c.events(events), limit=c.limit, cap=c.cap, undo=c.undo,
            ceiling=c.ceiling, setup=lambda lg: c.setup(lg, anchor_mph), **kw)
  return r, c


def test_drive_52_tag_318_end_to_end_50_to_55_by_taps():
  r, c = _sim(T.Car(50, dash_offset_mph=0.0), lambda f: 45 if f < 500 else 55, 2500)
  assert c.raises and abs(c.raises[0][1] - 55) < 1e-6
  assert r["holds"] == [] and len(r["presses"]) == 5 and all(b == cs.SET_PLUS for _, b in r["presses"])
  assert abs(r["final"] - 55) < 0.01 and r["max"] <= 55 + 1e-6


def test_drive_52_tag_211_end_to_end_first_limit_after_a_stop_slowdown():
  # no limit, set 35; a 45 road; a stopped car ahead lowers the set, his brake (cruise off), resume: the restore goes on
  # to 45 (the old code stopped at 35)
  spec = importlib.util.spec_from_file_location("tscs", os.path.join(_here, "test_stopped_car_slowdown.py"))
  S = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(S)
  # (drive: the 45 at 187.3, steady by 192.3 with cruise on, the stop slowdown from 191.0, his brake 196.0-200.2)
  r, c = _sim(T.Car(35, dash_offset_mph=0.0), lambda f: 0 if f < 20 else 45, 4000,
              events={870: S._brake, 1290: S._resume}, lead=S._Switch(870, 150, present=lambda f: f >= 390), max_drop=10)
  assert r["min"] < 35 - 1e-6                                     # the stop slowdown ran
  assert c.raises and abs(c.raises[0][1] - 45) < 1e-6 and c.raises[0][0] > 1290
  assert abs(r["final"] - 45) < 0.01 and r["max"] <= 45 + 1e-6
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS and f > 1290] == []


def test_drive_52_tag_450_end_to_end_the_restore_stops_at_the_lower_limit():
  # the set lowered 50 -> 45 (the red light head start; our anchor 50), his brake, cruise off, the limit 50 -> 45
  # while off, resume at 45: no restore back up to 50 (drive 52: 45 -> 50 at t 444-446, his SET- at t 448)
  car = T.Car(45, dash_offset_mph=0.0)
  r, c = _sim(car, lambda f: 50 if f < 500 else 45, 2500, events={0: _brake_held, 400: _cruise_off, 1000: _resume_now},
              anchor_mph=50)
  assert c.caps and abs(c.caps[0][1] - 45) < 1e-6
  assert r["presses"] == [] and r["holds"] == [] and abs(r["final"] - 45) < 0.01
  assert r["logic"].anchor is None
  # without the lower limit the same restore goes back to 50 (what the drive did)
  r0, _ = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 50, 2500,
               events={0: _brake_held, 400: _cruise_off, 1000: _resume_now}, anchor_mph=50)
  assert abs(r0["final"] - 50) < 0.01


def test_a_cap_with_our_presses_in_flight_stops_there_never_a_set_minus():
  # the restore 45 -> 50 already going (taps in flight) when the lower limit becomes stable
  for cap_at in (630, 650, 670, 690):
    car = T.Car(45, dash_offset_mph=0.0)
    r, c = _sim(car, lambda f, at=cap_at: 50 if f < at - slf.LIMIT_STABLE_TIME * 100 else 45, 3000,
                events={0: _brake_held, 600: _resume_now}, anchor_mph=50)
    capped = [f for f, _ in c.caps]
    assert capped, cap_at
    assert [f for f, b in r["presses"] if b == cs.SET_MINUS] == [], cap_at
    assert [f for f, b in r["presses"] if f > capped[0]] == [], (cap_at, r["presses"], capped)
    # what was in flight may still land ("leave it"); nothing more
    assert 45 - 1e-6 <= r["final"] <= 50 + 1e-6 and r["max"] <= r["final"] + 1e-6, (cap_at, r["final"])
    assert r["logic"].anchor is None and r["logic"].limit_cap is None


def test_a_cap_never_presses_set_minus():
  # the set already above the new limit (a curve left it at 48 of 55): it stays at 48
  r, c = _sim(T.Car(48, dash_offset_mph=0.0), lambda f: 55 if f < 500 else 45, 2000,
              events={0: _brake_held, 1000: _resume_now}, anchor_mph=55)
  assert c.caps and r["presses"] == [] and abs(r["final"] - 48) < 0.01
  # nothing pending (the set at the driver's speed): a lower limit does nothing at all
  r, c = _sim(T.Car(50, dash_offset_mph=0.0), lambda f: 50 if f < 500 else 45, 2000)
  assert c.caps == [] and r["presses"] == [] and abs(r["final"] - 50) < 0.01


def test_a_lower_limit_with_the_driver_outside_the_band_leaves_the_restore_alone():
  # the driver's speed 57 on a 50 road (old + 7): his own; the restore goes back to 57 after the limit falls to 45
  r, c = _sim(T.Car(50, dash_offset_mph=0.0), lambda f: 50 if f < 500 else 45, 2500,
              events={0: _brake_held, 1000: _resume_now}, anchor_mph=57)
  assert c.caps == [] and abs(r["final"] - 57) < 0.01


def test_a_cap_during_a_curve_waits_for_it_to_end():
  # max drop 5 from the driver's 50: the curve may go down to 45. Capped to 45 at once, the floor would have dropped to
  # 40 (SET- because of the lower limit); it waits: the curve does exactly what it does without the cap
  curve = T.between(500, 2000, 38)
  r, c = _sim(T.Car(50, dash_offset_mph=0.0), lambda f: 50 if f < 700 else 45, 5000, curve=curve)
  r0, _ = _sim(T.Car(50, dash_offset_mph=0.0), lambda f: 50, 5000, curve=curve)
  assert c.caps
  assert [p for p in r["presses"] if p[1] == cs.SET_MINUS] == [p for p in r0["presses"] if p[1] == cs.SET_MINUS]
  assert abs(r["min"] - r0["min"]) < 0.01 and abs(r["min"] - 45) < 0.01
  assert abs(r["final"] - 45) < 0.01 and abs(r0["final"] - 50) < 0.01


def test_drives_4c_4d_the_cross_road_matched_at_a_light_does_not_keep_the_restore_low():
  # 45 road, stopped at a light (set lowered to 35, our anchor 45, cruise off); the map matches the cross road's 40 for
  # 7 s, then the 45 again (drive 4c: 40 for 41 s at t 614-655 and 35 s at t 1704-1739, cruise back at t 654.6 and
  # 1738.7). Back before the resume: the cap is lifted, the restore goes to 45.
  slc = lambda f: 40 if 500 <= f < 1200 else 45  # noqa: E731
  r, c = _sim(T.Car(35, dash_offset_mph=0.0), slc, 4000, events={0: _brake_held, 400: _cruise_off, 1800: _resume_now},
              anchor_mph=45)
  assert c.caps == [] and c.raises == []                         # (it waited for cruise, and went before it was back)
  assert abs(r["final"] - 45) < 0.01 and r["max"] <= 45 + 1e-6
  # back only after the resume: the restore stops at 40 first, then the 45 (rule 1 from 40) takes it on to 45
  r, c = _sim(T.Car(35, dash_offset_mph=0.0), slc, 4000, events={0: _brake_held, 400: _cruise_off, 1000: _resume_now},
              anchor_mph=45)
  assert c.caps and c.raises and abs(c.raises[0][1] - 45) < 1e-6
  assert abs(r["final"] - 45) < 0.01 and r["max"] <= 45 + 1e-6
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS] == []


# ---- review of drive 52's rules: cruise off, map glitches, the undo (rule 4) ----

def _reasons(c):
  return [e["reason"] for _, e in c.f.events]


def test_rule_2_a_first_limit_that_came_with_cruise_off_is_not_raised_on_res():
  # review l3: cruise off at 5 s, a first limit 60 appears while it is off, RES 10 s later at the stored 45: it was offered
  # for his speed from before cruise went off and RES (not a SET edge: no LIMIT_DRIVER_QUIET) took it 0.3 s later
  for off_s in (20, 120):
    t_lim = (5 + off_s) * 100
    r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f, t=t_lim: 0 if f < t else 60, t_lim + 2000,
                events={500: _cruise_off, t_lim + 1000: _resume_now})
    assert c.raises == [] and r["max"] <= 45 + 1e-6 and abs(r["final"] - 45) < 0.01, (off_s, c.raises, r["final"])
    assert any("cruise was not on" in x for x in _reasons(c)), _reasons(c)
  # cruise off for a moment before it was steady (then RES at the same speed): not followed either
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 0 if f < 2000 else 55, 4000,
              events={2200: _cruise_off, 2300: _resume_now})
  assert c.raises == [] and abs(r["final"] - 45) < 0.01
  # cruise on throughout (drive 52, tag 211): followed
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 0 if f < 2000 else 55, 4000)
  assert c.raises and abs(r["final"] - 55) < 0.01


def test_a_map_glitch_shorter_than_5_s_never_raises():
  # review l1: a wrong road matched for 3.5 s (60 on a 45 road) took the set to 59-60, whatever the driver's speed in the
  # band; 4.9 s too
  for dur in (3.5, 4.9):
    for driver in (45, 50):
      r, c = _sim(T.Car(driver, dash_offset_mph=0.0), lambda f, d=dur: 60 if 1000 <= f < 1000 + d * 100 else 45, 3000)
      assert c.raises == [] and r["max"] <= driver + 1e-6 and r["presses"] == [] and r["holds"] == [], (dur, driver)
  # a flap 45 / 55 every 2.4 s neither
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 55 if (f // 240) % 2 else 45, 6000)
  assert c.raises == [] and r["max"] <= 45 + 1e-6
  # (F: a one-band rise with trust 0 is offered after 2.5 s) every 4 s: one raise, taken back by rule 4 at once, then
  # LIMIT_RAISE_STABLE_TIME again (no churn)
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 55 if (f // 400) % 2 else 45, 6000)
  assert len(c.raises) == 1 and len(c.undos) == 1 and abs(r["final"] - 45) < 0.01, (c.raises, c.undos, r["final"])


def test_a_first_limit_or_a_rise_of_more_than_10_mph_needs_gps_trust_0():
  # trust 1 (shaky) all the time: a 10 mph rise is followed, a 15 mph rise or a first limit is not
  for new, follows in ((55, True), (60, False)):
    f = _settled(45)
    out = _feed(f, RAISE + 10, new, gps_trust=1)[-1]
    assert (abs(out / MPH - new) < 1e-6) if follows else out == 0.0, (new, out)
  assert "GPS trust 1" in f.events[-1][1]["reason"]
  f = _fresh(2000)
  assert _feed(f, RAISE + 10, 45, driver_mph=35, gps_trust=1)[-1] == 0.0 and "GPS trust 1" in f.events[-1][1]["reason"]
  # trust 1 for a moment while the new limit was reported, 0 when it became steady: the worst counts
  f = _settled(45)
  _feed(f, 100, 60, gps_trust=1)
  assert _feed(f, RAISE, 60)[-1] == 0.0 and "GPS trust 1" in f.events[-1][1]["reason"]
  # trust 0 throughout (drive 52, tags 211 and 318): followed
  f = _settled(45)
  assert abs(_feed(f, RAISE + 10, 60)[-1] / MPH - 60) < 1e-6
  # metric: more than 16 km/h
  kph = 1 / 3.6
  for new, follows in ((90, True), (100, False)):
    fm = slf.SpeedLimitFollower(True)
    for _ in range(STABLE + 5):
      fm.update(slf.LimitInputs(80 * kph, "Map Data", 0, 80 * kph, True))
    out = [fm.update(slf.LimitInputs(new * kph, "Map Data", 1, 80 * kph, True)) for _ in range(RAISE + 10)][-1]
    assert (abs(out / kph - new) < 1e-6) if follows else out == 0.0, (new, out)


def _undos(c):
  return [u for _, u in c.undos]


def test_a_raise_from_a_glitch_is_undone_when_the_old_limit_comes_back():
  # review: 60 for 5.5-8 s on a 45 road (steady 5 s, trust 0) was raised and never taken back; now the 45 back within
  # 30 s of our raise takes the set speed back to the driver's 45 (or his 50, in the band), our own SET- taps only
  for driver, dur in ((45, 5.5), (45, 8.0), (50, 8.0)):
    r, c = _sim(T.Car(driver, dash_offset_mph=0.0), lambda f, d=dur: 60 if 1000 <= f < 1000 + d * 100 else 45, 5000)
    assert c.raises and abs(c.raises[0][1] - 60) < 1e-6, (driver, dur)
    assert [u[2] for u in _undos(c)] == ["undo"], (driver, dur, c.undos)
    assert abs(r["final"] - driver) < 0.01 and r["min"] >= driver - 1e-6, (driver, dur, r["final"], r["min"])
    assert r["logic"].anchor is None and r["logic"].limit_cap is None and r["logic"].undo_cap is None
  # a first limit (no limit before): any lower one takes it back
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 0 if f < 2000 else (60 if f < 2800 else 45), 6000)
  assert c.raises and [u[2] for u in _undos(c)] == ["undo"] and abs(r["final"] - 45) < 0.01
  # the lower limit 30 s or more after our raise: a real change, no undo (never a SET- for a lower limit)
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 60 if 1000 <= f < 4600 else 45, 7000)
  assert c.raises and c.undos == [] and abs(r["final"] - 60) < 0.01
  assert [p for p in r["presses"] if p[1] == cs.SET_MINUS] == []
  # a lower limit between the two (50): rule 3, no undo and no SET-
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 45 if f < 1000 else (60 if f < 1800 else 50), 5000)
  assert c.raises and c.undos == [] and [p for p in r["presses"] if p[1] == cs.SET_MINUS] == []


def test_an_undo_needs_gps_trust_0():
  # review (rule 4 had no trust check): the 45 back with trust 1 or 2 over its steady time is not trusted to take our
  # raise back by SET-: the raise stays; with trust 0 it is undone (above)
  for trust in (1, 2):
    r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 60 if 1000 <= f < 1600 else 45, 5000,
                trust=lambda f, t=trust: t if f >= 1600 else 0)
    assert c.raises and c.undos == [] and abs(r["final"] - 60) < 0.01, (trust, c.undos, r["final"])
    assert [p for p in r["presses"] if p[1] == cs.SET_MINUS] == []
    assert any("an undo needs 0" in x for x in _reasons(c)), _reasons(c)


def test_no_undo_after_the_driver_changed_his_speed_braked_or_a_slowdown_owns_it():
  def drv(btn):
    return lambda s, c: c.press(btn, who="driver")
  glitch = lambda f: 60 if 1000 <= f < 1800 else 45  # noqa: E731
  # his SET+ after our raise: his speed, nothing undone, nothing pressed by us after it (the 45 back at frame 1800
  # paused our restore at 57 at once, the provisional ceiling: his tap makes it 58)
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), glitch, 5000, events={1950: drv(cs.SET_PLUS)})
  assert c.raises and abs(r["final"] - 58) < 0.01, r["final"]
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS or f >= 1950] == []
  # his SET- during our restore: his
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), glitch, 5000, events={1550: drv(cs.SET_MINUS)})
  assert c.raises and [f for f, b in r["presses"] if b == cs.SET_MINUS] == [] and r["final"] < 60
  # his brake (cruise off) during our restore, the 45 back before (or after) his RES: the restore stops at his old 45 or
  # where the set speed is, never a SET-
  for res in (1950, 2300):
    r, c = _sim(T.Car(45, dash_offset_mph=0.0), glitch, 5000, events={1530: _cruise_off, res: _resume_now})
    assert c.raises and [u[2] for u in _undos(c)] == ["cap"], (res, c.undos)
    assert [f for f, b in r["presses"] if b == cs.SET_MINUS] == [] and r["final"] < 60 - 1e-6, (res, r["final"])
    assert r["logic"].anchor is None
  # a stopped car slowdown owns the set speed when the 45 comes back: its restore goes back to 45, not 60
  spec = importlib.util.spec_from_file_location("tscs", os.path.join(_here, "test_stopped_car_slowdown.py"))
  S = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(S)
  r, c = _sim(T.Car(45, dash_offset_mph=0.0), lambda f: 60 if 1000 <= f < 2050 else 45, 7000, max_drop=10,
              lead=_StoppedFrom(2200, 150.0, 2550), events={2550: S._brake, 2650: S._resume})
  assert c.raises and r["min"] < 45 - 1e-6                            # the stop slowdown ran
  assert [u[2] for u in _undos(c)] == ["cap"], c.undos
  assert r["max"] <= 60 + 1e-6 and abs(r["final"] - 45) < 0.01, r["final"]


class _StoppedFrom(T.StoppedLead):
  """Nothing ahead until frame `at`, then a stopped car `d_at` m ahead; from frame `gone` the only car ahead is 79 m
  away at 36 mph (the driver braked, the queue moves)."""
  def __init__(self, at, d_at, gone):
    super().__init__(d_at, present=lambda f: f >= at)
    self.at, self.gone = at, gone

  def tick(self, car):
    if self.frame + 1 < self.at:
      self.frame += 1
      return
    if self.frame + 1 == self.gone:
      self.d, self.v = 79.0, 36 * MPH
    super().tick(car)


def test_a_cap_never_cancels_a_set_minus_correcting_our_own_overshoot():
  # our SET+ landed one step too many (set 52, the driver's speed 50) and a lower limit's cap (45) comes: the correction
  # still goes back down to 50 (the cap only limits SET+), never to 45
  car = T.Car(52, dash_offset_mph=0.0)

  def setup(lg):
    lg.anchor, lg.last_set_speed = 50 * MPH, 52 * MPH
  r = T.run(car, NO_CURVE, 1500, setup=setup, cap=lambda f: 45 * MPH if f >= 5 else 0.0)
  assert abs(r["final"] - 50) < 0.01, r["final"]
  assert [b for _, b in r["presses"]] == [cs.SET_MINUS, cs.SET_MINUS] and r["min"] >= 50 - 1e-6


# ---- drive 53: the merge onto a highway (rule 5), the street after an exit (rule 6), a lower limit not steady yet ----

def _accel53(gap_mph):
  """The stock cruise's speed-up by the set speed gap in mph (drives 4c-53, medians)."""
  for top, a in ((5, 0.33), (10, 0.51), (15, 0.75), (20, 0.91)):
    if gap_mph < top:
      return a
  return 1.06


def _car53(set_mph, v_mph=None):
  car = T.Car(set_mph, dash_offset_mph=0.0, accel_fn=_accel53, cruise_gain=0.19)
  if v_mph is not None:
    car.v_ego = v_mph * MPH
  return car


class _Ramp:
  """A street (class 7, 45 mph) until frame `at`, then a link to a highway (class 2, mapRampToHighway) whose merge point
  is `length` m on; mapd's next limit (70) and its distance from `known` m into the link; SLC's own limit is none on the
  link and the highway's from 88 m before the merge (its lookahead, drive 53: t 333.2); class 1 after the merge. GPS
  trust 0, 2 from 200 m before the merge (drive 53: t 326.2)."""
  def __init__(self, at=500, length=650.0, known=36.0, to_highway=True, next_mph=70, street=45, after=1, trust_low=200.0):
    self.at, self.length, self.known, self.to_hwy, self.next, self.street, self.after = at, length, known, to_highway, next_mph, street, after
    self.trust_low = trust_low
    self.x0 = None
    self.x = 0.0

  def left(self, f):
    return self.length - (self.x - self.x0) if (self.x0 is not None and f >= self.at) else None

  def road(self, f, x):
    self.x = x
    if f < self.at:
      return dict(road_class=7)
    if self.x0 is None:
      self.x0 = x
    rest = self.length - (x - self.x0)
    if rest <= 0:
      return dict(road_class=self.after)
    out = dict(road_class=2, ramp_to_highway=self.to_hwy)
    if x - self.x0 >= self.known:
      out.update(next_limit=self.next, next_distance=rest)
    return out

  def slc(self, f):
    rest = self.left(f)
    if rest is None:
      return self.street
    return self.next if rest <= 88.0 else 0

  def trust(self, f):
    rest = self.left(f)
    return 2 if (rest is not None and self.trust_low is not None and rest <= self.trust_low) else 0


def _merge_sim(ramp, car, frames, **kw):
  r, c = _sim(car, ramp.slc, frames, road=ramp.road, trust=ramp.trust, **kw)
  merged = next((k for k in range(len(r["vs"])) if c.x >= 0 and k >= ramp.at and _at(r, ramp, k) >= ramp.length), None)
  return r, c, merged


def _at(r, ramp, k):
  # m driven along the link by frame k (the car's speeds since the link began)
  return sum(v * slf.DT_CTRL for v in r["vs"][ramp.at:k])


def test_drive_53_ramp_1_the_highway_limit_ahead_becomes_the_drivers_speed_in_time_for_the_merge():
  # tag 310: set 45 (his speed on the 45 road), onto the ramp at 18 mph (t 298.2), mapd's next limit 70 at 614 m (t 302.2),
  # no limit on the link; he held SET+ 45 -> 70 himself at t 320.7 (305-345 m before the merge) and was at 64 mph at
  # the merge. Before: nothing (the set stayed 45).
  ramp = _Ramp()
  car = _car53(45, 45)
  r, c, merged = _merge_sim(ramp, car, 6000, events={ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)})
  assert c.raises and abs(c.raises[0][1] - 70) < 1e-6 and c.kinds[0] == "merge", (c.raises, _reasons(c))
  f_raise = c.raises[0][0]
  left = ramp.length - _at(r, ramp, f_raise)
  # (owner, drive 54: "we can do it way before that") as soon as the window is ready: mapd's 70 known 36 m into the
  # link, MERGE_STABLE_TIME on (was timed to arrive 5 mph under the limit at the merge: 250-420 m before it)
  assert 560 < left < 610, left
  assert merged is not None and r["vs"][merged] / MPH >= 64, (merged, r["vs"][merged] / MPH if merged else None)
  assert abs(r["final"] - 70) < 0.01 and r["max"] <= 70 + 1e-6    # the highway's limit itself, not 70 + anything
  assert r["holds"] and [b for _, b in r["presses"] if b == cs.SET_MINUS] == []
  assert len(c.raises) == 1                                        # once per link (the 70 itself then is nothing new)
  # GPS trust 2 from 450 m before the merge (after the 5 s of trust 0 that start it, before it is due): still in time
  ramp = _Ramp(trust_low=450.0)
  r, c, merged = _merge_sim(ramp, _car53(45, 45), 6000, events={ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)})
  assert c.raises and c.kinds == ["merge"] and r["vs"][merged] / MPH >= 60


def test_drive_53_ramp_2_the_merge_waits_for_the_loops_curve_then_goes_to_the_highway_limit():
  # tag 1059.7: the loop's curve held the set at 29 (his speed 50) to the merge, 412 m (t 1020) .. 5 m (t 1049.8); then
  # the restore went back to his 50 only. Now the merge is offered at once (too short to wait) and taken when the curve
  # is over: the restore goes on to 70.
  ramp = _Ramp(at=500, length=430.0, known=18.0, street=50, trust_low=None)
  car = _car53(29, 29)
  end = ramp.at + 3100                                               # the curve ends at the merge (31 s at 29-32 mph)
  r, c, merged = _merge_sim(ramp, car, end + 3000, anchor_mph=50, curve=T.between(0, end, 27), max_drop=15,
                            events={0: _brake_held, 400: _resume_now})
  assert c.raises and c.kinds == ["merge"] and abs(c.raises[0][1] - 70) < 1e-6, (c.raises, _reasons(c))
  assert c.raises[0][0] >= end                                      # (the curve's lower target wins until it ends)
  assert max(r["sets"][:end]) / MPH <= 29 + 1e-6
  assert abs(r["final"] - 70) < 0.01 and r["max"] <= 70 + 1e-6


def test_drive_59_a_merge_from_27_mph_is_not_refused_for_its_jump():
  # drive 59, both merges refused: "merge: jump 43 > 30 mph" (set 27 after a light) and "jump 35 > 30 mph"; he held
  # SET+ to 70 himself. Any street speed onto the highway's limit is a merge (no jump limit)
  for street in (27, 35):
    ramp = _Ramp(street=street)
    r, c, _ = _merge_sim(ramp, _car53(street, street), 6000,
                         events={ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)})
    assert c.raises and abs(c.raises[0][1] - 70) < 1e-6 and c.kinds[0] == "merge", (street, c.raises, _reasons(c))
    assert not any("jump" in x for x in _reasons(c))
    assert abs(r["final"] - 70) < 0.01 and r["max"] <= 70 + 1e-6, (street, r["final"])


def test_merge_negatives():
  car = lambda: _car53(45, 45)  # noqa: E731
  ev = lambda ramp: {ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)}  # noqa: E731
  # a link to a road that is not a highway: nothing
  ramp = _Ramp(to_highway=False, after=5)
  r, c, _ = _merge_sim(ramp, car(), 6000, events=ev(ramp))
  assert c.raises == [] and abs(r["final"] - 45) < 0.01
  # the next limit is not above his speed (40 ahead), or not a highway's (45): nothing
  for nxt in (40, 45):
    ramp = _Ramp(next_mph=nxt)
    r, c, _ = _merge_sim(ramp, car(), 6000, events=ev(ramp))
    assert c.raises == [] and r["max"] <= 45 + 1e-6, nxt
  # GPS trust not 0 on the link: nothing
  ramp = _Ramp()
  ramp.trust = lambda f: 1
  r, c, _ = _merge_sim(ramp, car(), 6000, events=ev(ramp))
  assert c.raises == [] and r["max"] <= 45 + 1e-6
  # from a highway (an interchange link): the merge is not ours (his highway speed, the curve's restore)
  ramp = _Ramp(street=70)
  ramp.road = (lambda orig: lambda f, x: dict(orig(f, x), road_class=1) if f < ramp.at else orig(f, x))(ramp.road)
  r, c, _ = _merge_sim(ramp, _car53(55, 55), 6000)
  assert c.raises == []
  # his SET+ on the link before it starts: his (he chose 50; no raise to 70 on top)
  ramp = _Ramp()
  e = dict(ev(ramp))
  e[ramp.at + 300] = lambda s, cr: cr.press(cs.SET_PLUS, who="driver")
  r, c, _ = _merge_sim(ramp, car(), 6000, events=e)
  assert c.raises == [] and abs(r["final"] - 46) < 0.01, (c.raises, r["final"])
  assert any("the driver set his speed on the link" in x for x in _reasons(c))


def test_merge_the_driver_touching_his_speed_during_it_is_his():
  # our hold to 65 is going; his SET- at once: his speed, nothing of ours after it
  ramp = _Ramp()
  base, _, _ = _merge_sim(ramp, _car53(45, 45), 6000, events={ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)})
  f0 = base["holds"][0]
  for btn in (cs.SET_MINUS, cs.SET_PLUS):
    ramp = _Ramp()
    ev = {ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH), f0 + 150: lambda s, cr, b=btn: cr.press(b, who="driver")}
    r, c, _ = _merge_sim(ramp, _car53(45, 45), 6000, events=ev)
    after = [f for f, _ in r["presses"] if f > f0 + 150] + [f for f in r["holds"] if f > f0 + 150]
    if btn == cs.SET_MINUS:
      assert after == [] and r["final"] < 70 - 1e-6, (btn, r["final"], after)
    else:
      assert r["final"] <= 70 + 1e-6 and r["max"] <= 70 + 1e-6, (btn, r["final"])   # SET+: the rest still restored to 70
  # his brake (cruise off) on the ramp before the merge is taken: no raise after it
  ramp = _Ramp()
  ev = {ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH), f0 - 300: _cruise_off, f0 - 200: _resume_now}
  r, c, _ = _merge_sim(ramp, _car53(45, 45), 6000, events=ev)
  assert c.raises == [] and r["max"] <= 45 + 1e-6, (c.raises, _reasons(c))


def test_merge_taken_back_when_the_link_ends_on_a_street():
  # the map matched the wrong link: after our raise the car is on a street (class 5) for 3 s: back to his 45 (our own
  # SET- taps only), as rule 4
  ramp = _Ramp(after=5)
  ramp.slc = (lambda orig: lambda f: 45 if (orig(f) == 70) else orig(f))(ramp.slc)
  r, c, merged = _merge_sim(ramp, _car53(45, 45), 8000, events={ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)})
  assert c.raises and c.kinds == ["merge"]
  assert [u[2] for u in _undos(c)] == ["undo"], c.undos
  assert abs(r["final"] - 45) < 0.01


class _GlitchAfterMerge(_Ramp):
  """review: a few s after the merge the street's 45 is reported for `glitch` s, GPS trust 2 for 15 s after the merge
  (the street next to the highway)."""
  def __init__(self, after_s=5.0, glitch=4.0, **kw):
    super().__init__(**kw)
    self.after_s, self.glitch, self.merge_f = after_s, glitch, None

  def road(self, f, x):
    out = super().road(f, x)
    if out.get("road_class") == self.after and self.merge_f is None:
      self.merge_f = f
    return out

  def _in(self, f):
    return self.merge_f is not None and self.merge_f + self.after_s * 100 <= f < self.merge_f + (self.after_s + self.glitch) * 100

  def slc(self, f):
    return self.street if self._in(f) else super().slc(f)

  def trust(self, f):
    return 2 if (self.merge_f is not None and f < self.merge_f + 1500) else super().trust(f)


def test_merge_is_never_undone_once_on_the_highway():
  # review (merge2.py): the street's 45 reported 3-4 s with trust 2 a few s after the merge undid our merge raise on the
  # highway, 25 SET- 70 -> 45. Once the highway is matched the merge is done; an undo needs trust 0 too
  for after_s in (3.0, 5.0):
    ramp = _GlitchAfterMerge(after_s=after_s)
    r, c, _ = _merge_sim(ramp, _car53(45, 45), 9000, events={ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)})
    assert c.raises and c.kinds == ["merge"] and c.undos == [], (after_s, c.undos)
    assert [p for p in r["presses"] if p[0] >= ramp.merge_f and p[1] == cs.SET_MINUS] == []
    assert abs(r["final"] - 70) < 0.01
  # the same with trust 0 (a glitch the map is sure of): still not undone on the highway
  ramp = _GlitchAfterMerge()
  ramp.trust = lambda f: 0
  r, c, _ = _merge_sim(ramp, _car53(45, 45), 9000, events={ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)})
  assert c.raises and c.undos == [] and abs(r["final"] - 70) < 0.01


def test_merge_refused_after_his_set_or_cruise_off_any_time_on_the_link():
  # review (merge1.py): his SET, cancel or brake on the link only ended the merge once its window was open
  ev0 = lambda ramp: {ramp.at: lambda s, cr: setattr(cr, "v_ego", 18 * MPH)}  # noqa: E731

  def engage_set40(s, c):     # SET with cruise off: engages at the current speed, 40
    c.v_ego = c.set_speed = 40 * MPH
    c.driver_button, c.driver_button_until = cs.SET_MINUS, c.frame + 10

  def on(s, c):
    s["brake"], s["cruise"] = False, True
  for known in (36.0, 250.0):
    ramp = _Ramp(known=known)
    ev = dict(ev0(ramp))
    ev.update({ramp.at - 200: _cruise_off, ramp.at + 300: engage_set40, ramp.at + 310: on})
    r, c, _ = _merge_sim(ramp, _car53(45, 45), 7000, events=ev)
    assert c.raises == [] and r["max"] <= 45 + 1e-6 and abs(r["final"] - 40) < 0.01, (known, c.raises, r["final"])
  # his brake on the link before mapd's next limit is known, then RES at the stored 45
  ramp = _Ramp(known=200.0)
  ev = dict(ev0(ramp))
  ev.update({ramp.at + 50: _cruise_off, ramp.at + 400: _resume_now})
  r, c, _ = _merge_sim(ramp, _car53(45, 45), 7000, events=ev)
  assert c.raises == [] and r["max"] <= 45 + 1e-6, (c.raises, _reasons(c))
  assert any("cruise off on the link" in x for x in _reasons(c)), _reasons(c)
  # cruise already off when the link began, RES on it (drive 53, ramp 1: off from t 290.0, RES at 298.5, 0.3 s after the
  # link was matched): not his choice of speed, the merge goes on
  ramp = _Ramp()
  ev = dict(ev0(ramp))
  ev.update({ramp.at - 300: _cruise_off, ramp.at + 30: _resume_now})
  r, c, _ = _merge_sim(ramp, _car53(45, 45), 7000, events=ev)
  assert c.kinds == ["merge"] and abs(r["final"] - 70) < 0.01, (c.raises, _reasons(c))


def test_a_planned_stop_ahead_holds_the_merge_raise_and_the_fast_restore():
  # review (holdr.py): on a ramp to a highway (fast_restore) with a stopped car latched ahead (hold_restore), a merge
  # offer of 70 went 45 -> 70 by SET+ holds; the fast restore also ignored hold_restore
  def run(hold_restore, fast=True, frames=1500, set_mph=45, anchor=None):
    car = T.Car(set_mph, dash_offset_mph=0.0, lag=10, accel=0.6, step=MPH)
    logic = cs.CurveSlowdownLogic(False, 10)
    if anchor is not None:
      logic.anchor, logic.last_set_speed = anchor * MPH, set_mph * MPH
    plus = 0
    for f in range(frames):
      i = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego, gas_pressed=False,
                            brake_pressed=False, standstill=False, plan_alive=True, csc_active=False, csc_speed=100.0,
                            other_button_active=False, driver_button=None, fast_restore=fast, hold_restore=hold_restore(f),
                            limit_raise=(70 * MPH if f >= 300 and anchor is None else 0.0))
      btn, _ = logic.update(i)
      if btn is not None:
        plus += btn == cs.SET_PLUS
        car.hold_frame(btn) if logic.last_action == "hold" else car.press(btn)
      car.tick()
    return car, logic, plus
  car, logic, plus = run(lambda f: True)
  assert plus == 0 and abs(car.set_speed / MPH - 45) < 1e-6 and logic.anchor is None, (plus, car.set_speed / MPH)
  # the stop released at frame 800: the raise is taken then
  car, logic, plus = run(lambda f: f < 800)
  assert plus > 0 and abs(car.set_speed / MPH - 70) < 1e-6
  # a restore pending (his 60, set 45) on the ramp with the stop latched: no fast restore while it holds
  car, logic, plus = run(lambda f: True, anchor=60)
  assert plus == 0 and abs(car.set_speed / MPH - 45) < 1e-6, (plus, car.set_speed / MPH)


def test_merge_metric():
  # a 60 km/h street, a ramp to a 100 km/h highway: 100
  kph = 1 / 3.6
  ramp = _Ramp(next_mph=100 * kph / MPH, street=60 * kph / MPH)
  car = T.Car(60, dash_offset_mph=0.0, step=kph, accel_fn=_accel53, cruise_gain=0.19)
  car.set_speed = car.v_ego = 60 * kph
  c = Coupled(car, ramp.slc, metric=True, trust=ramp.trust, road=ramp.road)
  ev = c.events({ramp.at: lambda s, cr: setattr(cr, "v_ego", 30 * kph)})
  r = T.run(car, NO_CURVE, 7000, events=ev, limit=c.limit, cap=c.cap, undo=c.undo, ceiling=c.ceiling, metric=True,
            setup=lambda lg: c.setup(lg))
  assert c.raises and c.kinds == ["merge"] and abs(c.raises[0][1] * MPH / kph - 100) < 1e-6, c.raises
  assert abs(r["final"] * MPH / kph - 100) < 1.0          # (the simulated car's long press steps in mph)


def test_merge_need_follows_the_cruise_table():
  f = slf.SpeedLimitFollower(False)
  need = f._merge_need(45 * MPH, 70) - 45 * MPH * slf.MERGE_LEAD_TIME
  assert 290 < need < 310, need                                     # 45 -> 65 mph: ~11.9 s, ~300 m
  assert f._merge_need(70 * MPH, 70) == 70 * MPH * slf.MERGE_LEAD_TIME


class _Exit:
  """Drive 53, tag 457: the highway (class 1, 70; his 75) until frame `link`, the exit (class 2, no limit) until
  `street`, then a street (class 0 for 1 s, then 5) whose limit 50 comes at `limit`."""
  def __init__(self, link=1000, street=3000, limit=3400, street_mph=50):
    self.link, self.street, self.limit, self.street_mph = link, street, limit, street_mph

  def road(self, f, x):
    return dict(road_class=1 if f < self.link else (2 if f < self.street else (0 if f < self.street + 100 else 5)))

  def slc(self, f):
    return 70 if f < self.link else (self.street_mph if f >= self.limit else 0)


def test_drive_53_tag_457_after_an_exit_the_streets_limit_is_his_speed():
  # our stop slowdown left the set at the floor 19 on the exit (his 75); off the ramp the map's street rule made 19 his
  # speed (t 443.2), the street's 50 came at t 448.2: rule 2 refused 31 mph and he held SET+ himself at t 457
  ex = _Exit()

  def forget(s, car):
    s["brake"] = False
    c.logic._forget()                                            # (map_curve_control's adopt: off an exit onto a street)
  car = _car53(19, 19)
  c = Coupled(car, ex.slc, road=ex.road)
  ev = c.events({0: _brake_held, 2000: _cruise_off, 2700: _resume_now, ex.street + 50: forget})
  r = T.run(car, NO_CURVE, 7000, events=ev, limit=c.limit, cap=c.cap,
            undo=c.undo, ceiling=c.ceiling, setup=lambda lg: c.setup(lg, 75))
  assert c.raises and c.kinds == ["exit"] and abs(c.raises[0][1] - 50) < 1e-6, (c.raises, _reasons(c))
  assert c.raises[0][0] >= ex.limit + slf.EXIT_STEADY_TIME * 100      # (drive 69: 2 s on the street after an exit)
  assert abs(r["final"] - 50) < 0.01 and r["max"] <= 50 + 1e-6
  # the restore still pending (no street rule: his 75 kept): it stops at the street's 50, never 75 on the street
  car = _car53(19, 19)
  c = Coupled(car, ex.slc, road=ex.road)
  ev = c.events({0: _brake_held, 2000: _cruise_off, ex.street + 300: _resume_now})
  r = T.run(car, NO_CURVE, 7000, events=ev, limit=c.limit, cap=c.cap, undo=c.undo, ceiling=c.ceiling,
            setup=lambda lg: c.setup(lg, 75))
  assert abs(r["final"] - 50) < 0.01 and r["max"] <= 50 + 1e-6, (r["final"], r["max"], c.caps)
  # his own SET press since the exit began: his speed, no raise
  car = _car53(19, 19)
  c = Coupled(car, ex.slc, road=ex.road)
  ev = c.events({0: _brake_held, 2000: _cruise_off, 2700: _resume_now, 2800: lambda s, cr: cr.press(cs.SET_PLUS, who="driver"),
                 ex.street + 50: forget})
  r = T.run(car, NO_CURVE, 7000, events=ev, limit=c.limit, cap=c.cap,
            undo=c.undo, ceiling=c.ceiling, setup=lambda lg: c.setup(lg, 75))
  assert c.raises == [] and r["final"] < 50 - 1e-6, (c.raises, r["final"])
  assert any("set his speed" in x for x in _reasons(c)), _reasons(c)


def _exit_wrong_first(first, street_mph, wrong_s, trust=lambda f: 0, extra=None, floor_mph=25):
  """After an exit at the floor (his speed, map_curve_control's street adopt): mapd gives `first` for `wrong_s` s from
  the street's first limit, then the street's own."""
  ex = _Exit(limit=3200, street_mph=street_mph)
  slc = lambda f: first if ex.limit <= f < ex.limit + wrong_s * 100 else ex.slc(f)  # noqa: E731
  car = _car53(floor_mph, floor_mph)
  c = Coupled(car, slc, road=ex.road, trust=trust)

  def forget(s, cr):
    s["brake"] = False
    c.logic._forget()
  ev = {0: _brake_held, ex.street + 50: forget}
  ev.update(extra or {})
  r = T.run(car, NO_CURVE, 9000, events=c.events(ev), limit=c.limit, cap=c.cap, undo=c.undo, ceiling=c.ceiling,
            setup=lambda lg: c.setup(lg, 75))
  return r, c


def test_after_an_exit_a_wrong_first_limit_is_taken_back_by_the_streets_lower_one():
  # review (exit6.py): a wrong 65 steady 6 s raised his speed, then the street's 35 only capped the restore (rule 3) at
  # the set speed it had reached (50 on a 35 street). A lower limit within LIMIT_UNDO_TIME of a rule 6 raise brings his
  # speed down to it, our own SET- taps (as rule 4's undo)
  for wrong_s, first in ((10, 65), (15, 65), (15, 50)):
    r, c = _exit_wrong_first(first, 35, wrong_s)
    assert c.kinds == ["exit"] and abs(c.raises[0][1] - first) < 1e-6, (wrong_s, c.raises)
    assert [u[2] for u in _undos(c)] == ["undo"], (wrong_s, c.undos)
    assert abs(r["final"] - 35) < 0.01 and r["max"] <= first + 1e-6, (wrong_s, r["final"])
    assert [p for p in r["presses"] if p[1] == cs.SET_MINUS], "our own raise taken back by SET-"
  # never below his speed before our raise (45 from our restore, adopted off the exit: exit6.py): back to 45
  r, c = _exit_wrong_first(65, 35, 15, floor_mph=45)
  assert [u[2] for u in _undos(c)] == ["undo"] and abs(r["final"] - 45) < 0.01, (c.undos, r["final"])
  # the lower limit with GPS trust 2: not taken back (and no cap from our own raise)
  r, c = _exit_wrong_first(65, 35, 15, trust=lambda f: 2 if f >= 4700 else 0)
  assert c.undos == [] and abs(r["final"] - 65) < 0.01 and [p for p in r["presses"] if p[1] == cs.SET_MINUS] == []
  # 30 s or more after the raise: the street's own change, no undo, no SET-
  r, c = _exit_wrong_first(50, 35, 40)
  assert c.kinds == ["exit"] and c.undos == [] and [p for p in r["presses"] if p[1] == cs.SET_MINUS] == []


def test_after_an_exit_the_offer_ends_with_cruise_off():
  # review: a rule 6 offer survived cruise off and was taken 0.3 s after RES; cruise must stay on from when the limit
  # appeared until it is taken (his brake held the offer at 3350, cancel at 3450, RES at 3650; offered at 3400, the
  # street's limit steady EXIT_STEADY_TIME)
  r, c = _exit_wrong_first(50, 50, 6, extra={3350: _brake_held, 3450: _cruise_off, 3650: _resume_now})
  assert any("after the exit: cruise off" in x for x in _reasons(c)), _reasons(c)
  # (rule 8, owner after drives 54 / 55: his RES on the known 50 street matches its limit: a "match", not rule 6's)
  assert c.kinds == ["match"] and abs(r["final"] - 50) < 0.01, (c.kinds, r["final"])
  # cruise on throughout: taken
  r, c = _exit_wrong_first(50, 50, 6)
  assert c.kinds == ["exit"] and abs(r["final"] - 50) < 0.01


def test_drive_53_tag_1324_a_lower_limit_at_the_resume_holds_the_restore_at_once():
  # his 50 on a 45 road (inside the band), set 39 after a curve, his brake (cruise off t 1279-1317.8); the 45 -> 40 came at
  # t 1317.7, 0.1 s before cruise was back: the restore went 39 -> 50 by t 1323 (rule 3's cap, 3 s later: ~47)
  slc = lambda f: 45 if f < 1000 else 40  # noqa: E731
  r, c = _sim(_car53(39, 39), slc, 3000, events={0: _brake_held, 400: _cruise_off, 1010: _resume_now}, anchor_mph=50)
  assert r["max"] <= 40 + 1e-6 and abs(r["final"] - 40) < 0.01, (r["max"], r["final"])
  assert c.caps and r["logic"].anchor is None
  assert [b for _, b in r["presses"] if b == cs.SET_MINUS] == []


def test_drive_53_tags_1324_1355_the_40_road_then_the_45_again():
  # t 1279 his brake (set 39, his 50), the 40 at the resume (t 1317.8), our curve slowdown at t 1332.8, his brake again
  # (t 1339.1-1351.1), the 45 back at t 1354.2: never above 40 on the 40 road; on the 45 road his speed is the 45 (rule 1
  # from 40). Before: 50 by t 1323 and again at t 1352.7 (tags 1324.5, 1355.9).
  t0 = 1279.0

  def fr(t):
    return int(round((t - t0) * 100)) + 500
  slc = lambda f: 45 if f < fr(1317.7) or f >= fr(1354.2) else 40  # noqa: E731
  ev = {0: _brake_held, 400: _cruise_off, fr(1317.8): _resume_now, fr(1339.1): _cruise_off, fr(1351.1): _resume_now}
  r, c = _sim(_car53(39, 39), slc, fr(1375), events=ev, anchor_mph=50, curve=T.between(fr(1332.8), fr(1337.0), 36))
  on40 = r["sets"][fr(1317.8):fr(1354.2)]
  assert max(on40) / MPH <= 40 + 1e-6, max(on40) / MPH
  assert abs(r["final"] - 45) < 0.01 and r["max"] <= 45 + 1e-6, (r["final"], r["max"])


def test_a_short_lower_limit_while_restoring_only_pauses_it():
  # a junction's flap (35 for 1.5 s) while the restore 40 -> 50 runs on a 50 road: no SET+ while it is reported, then
  # on to 50; never a SET-, no cap (it never was steady)
  slc = lambda f: 35 if 1100 <= f < 1250 else 50  # noqa: E731
  r, c = _sim(_car53(40, 40), slc, 3500, events={0: _brake_held, 1000: _resume_now}, anchor_mph=50)
  assert c.caps == [] and abs(r["final"] - 50) < 0.01 and r["max"] <= 50 + 1e-6
  during = [f for f, b in r["presses"] if 1110 <= f < 1250] + [f for f in r["holds"] if 1110 <= f < 1250]
  assert during == [], during
  assert [b for _, b in r["presses"] if b == cs.SET_MINUS] == []
  # the same flap with the driver outside the band (57 on a 50 road): his, not paused
  r, c = _sim(_car53(40, 40), slc, 3500, events={0: _brake_held, 1000: _resume_now}, anchor_mph=57)
  assert abs(r["final"] - 57) < 0.01


# ---------------- Street Speed (MazdaStreetSpeedOffset), rule 7 (after a turn), owner after drives 54 / 55 ----------------

def _lis(limit_mph, driver_mph=45.0, road_class=5, name="A Road", trust=0, cruise=True, source="Map Data", turned=False,
         pending=False):
  return slf.LimitInputs(limit=limit_mph * MPH, source=source, gps_trust=trust,
                         driver_speed=driver_mph * MPH if cruise else 0.0, cruise_enabled=cruise, road_class=road_class,
                         road_name=name, turned=turned, pending=pending)


def _on_street(mode=2, limit=45, driver=50, road_class=5, name="A Road"):
  f = slf.SpeedLimitFollower(False, street_mode=mode)
  for _ in range(STABLE + 5):
    f.update(_lis(limit, driver, road_class=road_class, name=name, cruise=False))
  for _ in range(5):
    f.update(_lis(limit, driver, road_class=road_class, name=name))
  f.events = []
  f.engage_frame, f.offer, f.lower = None, None, None   # (settled: his speed set long ago, not a fresh engage: rule 8)
  return f


def _run(f, n, *a, **kw):
  return [f.update(_lis(*a, **kw)) for _ in range(n)]


def test_street_speed_setting_options_and_metric():
  for m, add in ((0, 0), (1, 3), (2, 5), (3, 8), (9, 5)):
    assert slf.SpeedLimitFollower(False, street_mode=m).street_add == add, m
  for m, add in ((0, 0), (1, 5), (2, 8), (3, 13)):
    assert slf.SpeedLimitFollower(True, street_mode=m).street_add == add, m
  assert slf.SpeedLimitFollower(False).street_add == 5                    # the default: +5 mph


def test_street_speed_rule_1_raise_and_band_on_a_street_never_on_a_highway():
  # at 50 on a 45 street (+5): the 55 road -> 60
  f = _on_street(2, 45, 50)
  out = _run(f, RAISE + 10, 55, 50)
  assert abs(out[-1] / MPH - 60) < 1e-6 and f.offer.kind == "raise" and f.offer.new == 55
  # the legal limit setting: 55 itself
  f = _on_street(0, 45, 50)
  assert abs(_run(f, RAISE + 10, 55, 50)[-1] / MPH - 55) < 1e-6
  # +8: at 53 on the 45 street he is "at" its speed (the band is the offset when larger): 55 -> 63; with the legal
  # limit setting 53 is outside rule 1's band (45 .. 50)
  f = _on_street(3, 45, 53)
  assert abs(_run(f, RAISE + 10, 55, 53)[-1] / MPH - 63) < 1e-6
  f = _on_street(0, 45, 53)
  assert _run(f, RAISE + 10, 55, 53)[-1] == 0.0
  # a highway (class 1): the limit itself whatever the setting (70 -> 75 at his 70)
  f = _on_street(3, 70, 70, road_class=1)
  assert abs(_run(f, RAISE + 10, 75, 70, road_class=1)[-1] / MPH - 75) < 1e-6
  # the road class unknown (Map Curves off): no offset
  f = _on_street(2, 45, 45, road_class=0)
  assert abs(_run(f, RAISE + 10, 50, 45, road_class=0)[-1] / MPH - 50) < 1e-6


def test_street_speed_rule_3_cap_and_rule_2_first_limit():
  # at 50 (45 + 5) the 35 road: a pending restore stops at 40
  f = _on_street(2, 45, 50)
  _run(f, STABLE + 5, 35, 50)
  assert f.cap is not None and abs(f.cap_speed / MPH - 40) < 1e-6
  # a first limit on a street: the limit + 5 (35 -> 50 for a 45 road)
  f = slf.SpeedLimitFollower(False)
  _run(f, 2000, 0, 35, source="None")
  out = _run(f, RAISE + 10, 45, 35)
  assert abs(out[-1] / MPH - 50) < 1e-6 and f.offer.kind == "first"


def _turned(mode=2, old=45, new=50, driver=75, trust=lambda k: 0, cruise_off=False, turned=True, frames=RAISE + 600):
  """On A Road (old) at his `driver`, a turn onto B Road (new): the name changes, the new limit from the same moment;
  turned: the heading changed (HeadingWatch) through it."""
  f = _on_street(mode, old, driver)
  outs = []
  for k in range(frames):
    outs.append(f.update(_lis(new, driver, name="B Road", trust=trust(k), cruise=not cruise_off, turned=turned and k < 500)))
  return f, outs


def test_rule_7_after_a_turn_a_restore_never_goes_back_above_the_new_roads_limit():
  # drive 55: Dobry Drive (45), right turn onto Mound Road (50), his highway 75 still the restore's target: capped at 55
  f, outs = _turned(2, 45, 50, 75)
  assert all(x == 0.0 for x in outs) and f.cap is not None and abs(f.cap_speed / MPH - 55) < 1e-6
  assert f.cap.band is False and f.exit_kind == "turn"
  # the same limit on the new road (45 -> 45): its limit once the name has changed
  f, outs = _turned(2, 45, 45, 75)
  assert f.cap is not None and abs(f.cap_speed / MPH - 50) < 1e-6
  # GPS trust 2 when its limit became steady (drive 55, t 633-639), 0 again later: taken then
  f, outs = _turned(2, 45, 50, 75, trust=lambda k: 2 if k < 900 else 0)
  assert f.cap is not None and abs(f.cap_speed / MPH - 55) < 1e-6
  # no name change (the same road's new limit): rule 3 as before (his 75 is outside the band: nothing)
  f = _on_street(2, 45, 75)
  _run(f, RAISE + 600, 50, 75)
  assert f.cap is None and f.offer is None


def test_rule_7_needs_a_heading_change_and_the_trust_retry_raises():
  # review of merge-f: the name changing along the same road (or the highway's name becoming the street's after an exit)
  # is no turn: without the heading change rule 7 does not open
  f, outs = _turned(2, 45, 50, 75, turned=False)
  assert f.cap is None and f.exit_kind != "turn" and all(x == 0.0 for x in outs)
  # review of merge-f (drive 55, Mound Road): the new road's 50 steady while GPS trust was 2, then 0: at 40 it raises to
  # 55 on a fresh steady time with trust 0 (the retry kept the old trust 2 and never raised)
  f, outs = _turned(2, 45, 50, 40, trust=lambda k: 2 if k < 900 else 0, frames=900 + RAISE + 300)
  got = [k for k, x in enumerate(outs) if x > 0]
  assert got and abs(outs[got[0]] / MPH - 55) < 1e-6 and got[0] >= 900 + RAISE - 5, (got[:1], f.events)
  # the heading watch: 90 deg in 6 s is a turn, a lane change's few degrees are not, an old turn is forgotten
  h = slf.HeadingWatch()
  for _ in range(600):
    h.update(0.01, 15.0)
  assert h.turned()
  for _ in range(1100):
    h.update(0.01, 0.0)
  assert not h.turned()
  for k in range(400):
    h.update(0.01, 3.0 if k < 200 else -3.0)
  assert not h.turned()
  # (second review) 45 deg within 10 s and 150 m: a 90 deg right turn at 6 m/s (drive 55, Mound Road) is one; 40 deg is
  # not; 50 deg swept over 10 s at 20 m/s (200 m, a gentle curve) is not; the same 50 deg within 100 m is
  for rate, v, n, want in ((30.0, 6.0, 300, True), (8.0, 6.0, 500, False), (5.0, 20.0, 1000, False),
                           (10.0, 20.0, 500, True)):
    h = slf.HeadingWatch()
    for _ in range(n):
      h.update(0.01, rate, v)
    assert h.turned() is want, (rate, v, n)
  assert slf.TURN_HEADING == 45.0 and slf.TURN_HEADING_DIST == 150.0
  # mapd's new name comes seconds after the turn (drive 55, Mound Road: the name steady 0.2 s after the heading's 10 s
  # window ended): a turn if the heading said so at any time since the new name appeared
  f = _on_street(2, 45, 75)
  for k in range(RAISE + 600):
    f.update(_lis(50, 75, name="B Road", turned=k < 50))
  assert f.exit_kind == "turn" and f.cap is not None and abs(f.cap_speed / MPH - 55) < 1e-6
  # ... not if it said so only before the new name appeared (a curve on the old road)
  f = _on_street(2, 45, 75)
  _run(f, 100, 45, 75, turned=True)
  _run(f, RAISE + 600, 50, 75, name="B Road", turned=False)
  assert f.exit_kind != "turn" and f.cap is None


def test_review2_rule_9_undo_brings_back_his_own_speed_not_a_pending_restores():
  # a 45 street, the controller's anchor 70 (his highway speed, a restore pending: the set under it), the limit 35:
  # slowed to 40; the limit back to 45 soon after (a wrong lower limit): back to at most 45 + 5 (was: 70 on the street)
  for pending, back in ((True, 50), (False, 70)):
    f = _on_street(2, 45, 70)
    f.slow_lower = True
    _run(f, int(slf.LOWER_STEADY / slf.DT_CTRL) + 50, 35, 70, pending=pending)
    assert f.lower is not None and f.lower.target == 40 and f.lower.own is (not pending), f.events
    f.lower_finished("done", 40)
    _run(f, RAISE + 50, 45, 40)
    assert f.offer is not None and f.offer.kind == "unlower" and f.offer.target == back, (pending, f.offer, f.events)


def test_rule_7_a_first_known_limit_on_the_new_road_raises_him_never_slows_him():
  # at 40 on A Road, a turn onto the 50 road: his speed 55 (+5) once steady 5 s
  f, outs = _turned(2, 45, 50, 40)
  got = [x for x in outs if x > 0]
  assert got and abs(got[-1] / MPH - 55) < 1e-6 and f.offer.kind == "turn", (got[-1:] and got[-1] / MPH, f.events)
  assert all(x == 0.0 for x in outs[:RAISE - 1])
  # more than LIMIT_FIRST_MAX above his speed: not followed
  f, outs = _turned(2, 45, 50, 30)
  assert all(x == 0.0 for x in outs) and any("more than 15" in e[1]["reason"] for e in f.events), f.events
  # his speed above the new road's: never a SET- (a cap of a pending restore only)
  f, outs = _turned(0, 50, 35, 50)
  assert all(x == 0.0 for x in outs) and f.undo is None


def _exit69(limit_at=None, street_mph=45, set_mph=27, trust=lambda f: 0, extra=None, street_mode=1):
  """Drive 69 t 658-700: the highway (70), its exit (class 2) until 3000, then a street (class 0 for 1 s, then 7) with the
  set speed adopted at 27 off the ramp; the street's 45 comes at `limit_at` (None: never). Street Speed +3."""
  ex = _Exit(link=1000, street=3000, limit=limit_at if limit_at is not None else 10 ** 9, street_mph=street_mph)
  car = _car53(set_mph, set_mph)
  c = Coupled(car, ex.slc, road=ex.road, trust=trust, street_mode=street_mode)

  # (the set speed already his: the ramp's slowdown adopted, map_curve_control's street rule)
  r = T.run(car, NO_CURVE, 7000, events=c.events(extra), limit=c.limit, cap=c.cap, undo=c.undo, ceiling=c.ceiling,
            setup=lambda lg: c.setup(lg))
  return r, c


def test_drive_69_after_an_exit_the_streets_limit_is_raised_to_2_s_after_it_is_steady():
  # the street's 45 came 4.6 s after the turn onto it and was taken 5 s later (693.3), the driver pressed SET+ himself
  r, c = _exit69(limit_at=3560)
  ex = [(f, v) for (f, v), k in zip(c.raises, c.kinds, strict=True) if k == "exit"]
  assert ex and abs(ex[-1][1] - 48) < 1e-6, (c.raises, c.kinds, _reasons(c))
  assert ex[-1][0] <= 3560 + slf.EXIT_STEADY_TIME * 100 + 20, ex
  assert abs(r["final"] - 48) < 0.01


def test_drive_69_after_an_exit_with_no_limit_the_street_fallback_after_2_s():
  r, c = _exit69()
  assert c.kinds == ["exit"] and abs(c.raises[0][1] - slf.STREET_FALLBACK[0]) < 1e-6, (c.raises, _reasons(c))
  assert c.raises[0][0] <= 3100 + slf.EXIT_STEADY_TIME * 100 + 20, c.raises      # (class 7 from 3100)
  assert abs(r["final"] - 43) < 0.01 and r["max"] <= 43 + 1e-6
  # a limit that comes after it: rule 6 from there (43 -> 48)
  r, c = _exit69(limit_at=3600)
  assert abs(r["final"] - 48) < 0.01, (c.raises, c.kinds, r["final"])
  # never from above it, never with GPS trust above 0, never after his own SET since the exit began
  r, c = _exit69(set_mph=45)
  assert c.raises == [] and abs(r["final"] - 45) < 0.01
  r, c = _exit69(trust=lambda f: 1)
  assert c.raises == []
  r, c = _exit69(extra={2500: lambda s, cr: cr.press(cs.SET_MINUS, who="driver")})
  assert c.raises == [] and any("set his speed" in x for x in _reasons(c)), (c.raises, _reasons(c))
  # metric: 70 km/h
  f = slf.SpeedLimitFollower(True)
  assert f.fallback == 70
