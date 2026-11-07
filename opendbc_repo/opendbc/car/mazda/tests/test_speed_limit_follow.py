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


def test_limit_must_be_stable_for_3_s():
  f = _settled(45)
  out = _feed(f, STABLE + 10, 50)
  assert all(x == 0.0 for x in out[:STABLE - 1])
  assert abs(out[-1] / MPH - 50) < 1e-6


def test_only_when_the_drivers_speed_is_at_the_old_limit_or_up_to_5_above():
  # rule 1's band: old .. old + 5 mph; always to the new limit itself
  for driver, follows in ((45, True), (47, True), (50, True), (44, False), (52, False), (40, False)):
    f = _settled(45, driver_mph=driver)
    out = _feed(f, STABLE + 10, 55, driver_mph=driver)[-1]
    assert (abs(out / MPH - 55) < 1e-6) if follows else out == 0.0, (driver, out)
    if not follows:
      assert f.events and "not the old limit or up to 5 above" in f.events[-1][1]["reason"], driver
  # metric: 10 km/h
  kph = 1 / 3.6
  for driver, follows in ((80, True), (90, True), (95, False)):
    fm = slf.SpeedLimitFollower(True)
    for _ in range(STABLE + 5):
      fm.update(slf.LimitInputs(80 * kph, "Map Data", 0, driver * kph, True))
    out = [fm.update(slf.LimitInputs(100 * kph, "Map Data", 0, driver * kph, True)) for _ in range(STABLE + 10)][-1]
    assert (abs(out / kph - 100) < 1e-6) if follows else out == 0.0, (driver, out)


def test_drive_52_tag_318_at_50_on_a_45_road_the_55_is_followed_to_55():
  # he set 50 himself on the 45 road (t 230); the limit went 45 -> 55 (t 310.3); he raised 50 -> 55 by hand (t 321)
  f = _settled(45, driver_mph=50)
  out = _feed(f, STABLE + 10, 55, driver_mph=50)
  assert all(x == 0.0 for x in out[:STABLE - 1])
  assert abs(out[-1] / MPH - 55) < 1e-6                           # the new limit, not 55 + his 5
  assert f.offer.old == 45 and f.offer.kind == "raise"
  # once he is at 55 (our raise, or his own SET+ first) the offer is closed, quietly
  assert _feed(f, 5, 55, driver_mph=55)[-1] == 0.0 and f.offer is None and not f.events


def test_a_change_of_the_drivers_speed_inside_the_band_while_offered_is_his():
  f = _settled(45)
  _feed(f, STABLE + 2, 55)
  assert _feed(f, 5, 55, driver_mph=48)[-1] == 0.0 and f.offer is None
  assert "changed his speed to 48" in f.events[-1][1]["reason"]


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


def test_drive_4d_a_cross_road_limit_while_waiting_at_the_light_does_not_hide_the_old_one():
  # 45 (the driver's speed) -> 40 for 28 s at the light (the cross road) -> 55 after the turn: offered 45 -> 55
  f = _settled(45)
  _feed(f, 2800, 40)
  out = _feed(f, STABLE + 10, 55)
  assert abs(out[-1] / MPH - 55) < 1e-6
  assert f.offer.old == 45
  # ... but not when the driver's speed was last at 45 too long ago (and is not in the band of the cross road's 35)
  f = _settled(45)
  _feed(f, int(slf.LIMIT_MATCH_MAX_AGE / slf.DT_CTRL) + 100, 35)
  assert _feed(f, STABLE + 10, 55)[-1] == 0.0
  # (with the cross road at 40, 45 is in 40's band: after 90 s that is the old limit, and the 55 is followed from it)
  f = _settled(45)
  _feed(f, int(slf.LIMIT_MATCH_MAX_AGE / slf.DT_CTRL) + 100, 40)
  assert abs(_feed(f, STABLE + 10, 55)[-1] / MPH - 55) < 1e-6 and f.offer.old == 40
  # and a driver who set 40 on the 40 road is matched to 40: a 40 -> 55 rise is offered from 40
  f = _settled(45)
  _feed(f, 2800, 40, driver_mph=40)
  assert abs(_feed(f, STABLE + 10, 55, driver_mph=40)[-1] / MPH - 55) < 1e-6


def test_old_limit_forgotten_after_30_s_unknown():
  # at 50 on a 45 road, 31 s with no limit, then 70: as a rise from 45 it would be followed (band, jump 25); after 30 s
  # the 45 is forgotten and the 70 is a first limit, 20 above the driver's speed: not followed
  f = _settled(45, driver_mph=50)
  _feed(f, int(slf.LIMIT_OLD_MAX_AGE / slf.DT_CTRL) + 100, 0, source="None", driver_mph=50)
  assert _feed(f, STABLE + 10, 70, driver_mph=50)[-1] == 0.0
  assert "more than 15 above" in f.events[-1][1]["reason"]


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


# ---- rule 2: a first limit after none ----

def _fresh(n, driver_mph=35.0, **kw):
  """A drive with no limit known for n frames (residential streets) at the driver's speed."""
  f = slf.SpeedLimitFollower(False)
  _feed(f, n, 0, source="None", driver_mph=driver_mph, **kw)
  return f


def test_drive_52_tag_211_a_first_limit_10_above_the_drivers_speed_is_followed():
  # slc 0 from t 5.7 to 187.3 (set 35 on the unmapped street), then a 45 road; he held SET+ 35 -> 45 at t 212.8
  f = _fresh(18_000)
  out = _feed(f, STABLE + 10, 45, driver_mph=35)
  assert all(x == 0.0 for x in out[:STABLE - 1])
  assert abs(out[-1] / MPH - 45) < 1e-6 and f.offer.kind == "first" and f.offer.old is None
  # since the drive started counts too
  f = slf.SpeedLimitFollower(False)
  assert abs(_feed(f, STABLE + 10, 45, driver_mph=35)[-1] / MPH - 45) < 1e-6
  # the cruise-off wait (a stop slowdown, his brake: t 196-200.2) and back at his speed
  f = _fresh(18_000)
  _feed(f, STABLE + 2, 45, cruise=False)
  assert all(x == 0.0 for x in _feed(f, 400, 45, cruise=False))
  assert abs(_feed(f, 1, 45, driver_mph=35)[-1] / MPH - 45) < 1e-6
  # ... but back at another speed (he set it with cruise off): his
  f = _fresh(18_000)
  _feed(f, STABLE + 2, 45, cruise=False)
  assert _feed(f, 5, 45, driver_mph=38)[-1] == 0.0 and f.offer is None
  # metric: up to 25 km/h above
  kph = 1 / 3.6
  for new, follows in ((70, True), (80, False)):
    fm = slf.SpeedLimitFollower(True)
    for _ in range(2000):
      fm.update(slf.LimitInputs(0.0, "None", 0, 50 * kph, True))
    out = [fm.update(slf.LimitInputs(new * kph, "Map Data", 0, 50 * kph, True)) for _ in range(STABLE + 10)][-1]
    assert (abs(out / kph - new) < 1e-6) if follows else out == 0.0, (new, out)


def test_first_limit_not_followed():
  # 20 above the driver's speed
  f = _fresh(2000)
  assert _feed(f, STABLE + 10, 55, driver_mph=35)[-1] == 0.0 and "more than 15 above" in f.events[-1][1]["reason"]
  # 15 above is the most
  f = _fresh(2000)
  assert abs(_feed(f, STABLE + 10, 50, driver_mph=35)[-1] / MPH - 50) < 1e-6
  # a speed he chose after the new limit appeared, even inside 15
  f = _fresh(2000)
  _feed(f, 50, 45, driver_mph=35)
  assert _feed(f, STABLE + 10, 45, driver_mph=40)[-1] == 0.0 and "set after the limit appeared" in f.events[-1][1]["reason"]
  # ... and one he chose after it became stable, while it was offered
  f = _fresh(2000)
  _feed(f, STABLE + 2, 45, driver_mph=35)
  assert _feed(f, 5, 45, driver_mph=40)[-1] == 0.0 and f.offer is None
  # no driver's speed yet (cruise never on)
  f = _fresh(2000, cruise=False)
  assert _feed(f, STABLE + 10, 45, cruise=False)[-1] == 0.0 and _feed(f, 10, 45, driver_mph=35)[-1] == 0.0
  # the usual checks: GPS trust, not a multiple of 5, above 80 mph, and the offer's timeout
  for new, kw in ((45, dict(gps_trust=2)), (42, {}), (85, dict(driver_mph=75))):
    kw = dict(dict(driver_mph=35), **kw)
    f = _fresh(2000, driver_mph=kw["driver_mph"])
    assert _feed(f, STABLE + 10, new, **kw)[-1] == 0.0, (new, kw)
  f = _fresh(2000)
  _feed(f, STABLE + 2, 45, cruise=False)
  _feed(f, int(slf.LIMIT_OFFER_TIME / slf.DT_CTRL) + 10, 45, cruise=False)
  assert _feed(f, 10, 45, driver_mph=35)[-1] == 0.0 and "not taken in time" in f.events[-1][1]["reason"]
  # at or below the driver's speed: nothing to raise, nothing logged
  f = _fresh(2000, driver_mph=50)
  assert _feed(f, STABLE + 10, 45, driver_mph=50)[-1] == 0.0 and not f.events


def test_a_first_limit_right_after_a_junction_flap_is_a_normal_change():
  # at 40 on a 45 road, the limit gone for 2 s (a junction), then 50: the 45 was seen 2 s ago, so this is a rise from
  # 45 (rule 1: 40 is not in its band), not a first limit (rule 2 would follow 40 -> 50)
  f = _settled(45, driver_mph=40)
  _feed(f, 200, 0, source="None", driver_mph=40)
  assert _feed(f, STABLE + 10, 50, driver_mph=40)[-1] == 0.0
  assert "not the old limit" in f.events[-1][1]["reason"]
  # with the 45 not seen for 30 s, but another limit flapping 5 s before: not quiet for 10 s, not a first limit either
  f = _settled(45, driver_mph=40)
  _feed(f, 3200, 0, source="None", driver_mph=40)
  _feed(f, 100, 35, driver_mph=40)
  _feed(f, 500, 0, source="None", driver_mph=40)
  assert _feed(f, STABLE + 10, 50, driver_mph=40)[-1] == 0.0
  assert "old limit not seen recently" in f.events[-1][1]["reason"]
  # ... while the same limit seen briefly before (the new road's own flap) does not count against it
  f = _fresh(2000, driver_mph=40)
  _feed(f, 100, 50, driver_mph=40)
  _feed(f, 200, 0, source="None", driver_mph=40)
  assert abs(_feed(f, STABLE + 10, 50, driver_mph=40)[-1] / MPH - 50) < 1e-6


def test_drive_4d_band_matched_keeps_the_limit_the_driver_is_closest_to():
  # at 50 on a 45 road (rule 1's band), the cross road 40 at the light (50 is outside its band), then 55
  f = _settled(45, driver_mph=50)
  _feed(f, 2800, 40, driver_mph=50)
  assert abs(_feed(f, STABLE + 10, 55, driver_mph=50)[-1] / MPH - 55) < 1e-6 and f.offer.old == 45


# ---- rule 3: a lower limit caps a pending restore ----

def test_drive_52_tag_450_a_lower_limit_offers_a_cap_until_taken():
  # at 50 on a 50 road (the driver's speed: our anchor 50, the set lowered to 45), cruise off, the limit 50 -> 45
  f = _settled(50, driver_mph=50)
  caps = []
  for _ in range(STABLE + 50):
    f.update(_li(45, driver_mph=50, cruise=False))
    caps.append(f.cap_speed)
  assert caps[STABLE - 2] == 0.0 and all(abs(x / MPH - 45) < 1e-6 for x in caps[STABLE + 1:])   # open, cruise off
  assert f.cap.old == 50 and f.cap.driver == 50
  assert all(x == 0.0 for x in _feed(f, 100, 45, driver_mph=50)) and f.cap is not None     # never a raise
  _feed(f, 1, 45, driver_mph=45)                                                          # taken: 45 now
  assert f.cap is None and all(x == 0.0 for x in _feed(f, 100, 45, driver_mph=45)) and f.cap is None
  # a new limit closes it too (the cross road's limit gone again)
  f = _settled(50, driver_mph=50)
  _feed(f, STABLE + 5, 45, driver_mph=50, cruise=False)
  assert f.cap is not None
  _feed(f, STABLE + 5, 50, driver_mph=50, cruise=False)
  assert f.cap is None and f.offer is None


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
  """Every frame the follower sees the posted limit slc(f) (mph, 0 none) and the controller's driver's speed (the
  anchor, else the set speed; 0 with cruise off); the controller gets its raise and cap. An adoption closes the offer
  (the wrapper's _limit_bookkeeping, here on the next frame, before the follower runs)."""
  def __init__(self, car, slc, metric=False):
    self.car, self.slc = car, slc
    self.f = slf.SpeedLimitFollower(metric)
    self.logic, self.st, self.frame = None, None, -1
    self.raise_, self.cap_ = 0.0, 0.0
    self.raises, self.caps = [], []

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
    if lg.limit_adopted is not None:
      self.f.mark_adopted()
      self.raises.append((f - 1, lg.limit_adopted[1] / MPH))
    if lg.limit_capped is not None:
      self.caps.append((f - 1, lg.limit_capped[1] / MPH))
    if self.f.cap is not None and lg.limit_cap is None:
      self.f.mark_capped()                          # the controller holds no cap (any more): nothing to keep open
    cruise = self.st["cruise"] if self.st is not None else True
    drv = (lg.anchor if lg.anchor is not None else self.car.set_speed) if cruise else 0.0
    self.raise_ = self.f.update(slf.LimitInputs(self.slc(f) * MPH, "Map Data", 0, drv, cruise))
    self.cap_ = self.f.cap_speed

  def limit(self, f):
    self._step(f)
    return self.raise_

  def cap(self, f):
    self._step(f)
    return self.cap_


def _brake_held(s, c):
  s["brake"] = True                                 # cruise on, nothing of ours goes out


def _cruise_off(s, c):
  s["cruise"], s["brake"] = False, True


def _resume_now(s, c):
  s["cruise"], s["brake"] = True, False


def _sim(car, slc, frames, events=None, anchor_mph=None, **kw):
  c = Coupled(car, slc)
  r = T.run(car, kw.pop("curve", NO_CURVE), frames, events=c.events(events), limit=c.limit, cap=c.cap,
            setup=lambda lg: c.setup(lg, anchor_mph), **kw)
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
  r, c = _sim(T.Car(35, dash_offset_mph=0.0), lambda f: 0 if f < 20 else 45, 4000,
              events={330: S._brake, 600: S._resume}, lead=S._Switch(330, 100), max_drop=10)
  assert r["min"] < 35 - 1e-6                                     # the stop slowdown ran
  assert c.raises and abs(c.raises[0][1] - 45) < 1e-6 and c.raises[0][0] > 600
  assert abs(r["final"] - 45) < 0.01 and r["max"] <= 45 + 1e-6
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS and f > 600] == []


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
  assert c.caps and c.raises == []
  assert abs(r["final"] - 45) < 0.01 and r["max"] <= 45 + 1e-6
  # back only after the resume: the restore stops at 40 first, then the 45 (rule 1 from 40) takes it on to 45
  r, c = _sim(T.Car(35, dash_offset_mph=0.0), slc, 4000, events={0: _brake_held, 400: _cruise_off, 1000: _resume_now},
              anchor_mph=45)
  assert c.caps and c.raises and abs(c.raises[0][1] - 45) < 1e-6
  assert abs(r["final"] - 45) < 0.01 and r["max"] <= 45 + 1e-6
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS] == []
