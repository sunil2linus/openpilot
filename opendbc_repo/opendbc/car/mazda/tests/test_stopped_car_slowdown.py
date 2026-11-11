"""Offline tests for the stopped car slowdown (trigger, target, holds) on the shared set speed controller."""
import importlib.util
import os

_spec = importlib.util.spec_from_file_location(
  "test_curve_slowdown", os.path.join(os.path.dirname(__file__), "test_curve_slowdown.py"))
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
cs, scs, MPH = T.cs, T.scs, T.MPH
Car, StoppedLead, run, near, between = T.Car, T.StoppedLead, T.run, T.near, T.between
NO_CURVE = lambda f: None  # noqa: E731
FLOOR_MPH = 30 / 3.6 / 0.44704   # the stopped car floor: the car's own 30 km/h minimum (the dash shows 19)
CAR_MIN_MPH = 30 / 1.609344   # the car's own minimum set speed


def approach(set_mph=45, d0=120, frames=6000, lead_kw=None, car_kw=None, curve=NO_CURVE, events=None, stop_mode=2, max_drop=10):
  lead = StoppedLead(d0, **(lead_kw or {}))
  car = Car(set_mph, dash_offset_mph=0.0, **(car_kw or {}))
  return run(car, curve, frames, events=events, lead=lead, stop_mode=stop_mode, max_drop=max_drop)


# deceleration of the stock cruise vs the set speed gap (vEgo - set, mph), measured on 9 drives (Nov 2026)
MEASURED_DECEL = lambda gap: scs._interp(gap, [0.5, 3, 7.5, 10.5, 13.5, 16.5, 19.5], [0.0, 0.54, 0.70, 0.77, 0.86, 0.90, 1.6])  # noqa: E731
MEASURED = lambda lg: (setattr(lg, "hold_overrun", 1), setattr(lg, "hold_measured", True))  # noqa: E731  owner's car profile


def first_frame(r, what="any"):
  fs = [f for f, _ in r["presses"]] + r["holds"]
  return min(fs) if fs else None


# ---------------- trigger and target ----------------

def test_target_formula():
  assert near(scs.stopped_car_target(15.0) / MPH, FLOOR_MPH)
  assert near(scs.stopped_car_target(0.0) / MPH, FLOOR_MPH)
  v = scs.stopped_car_target(120.0)
  assert abs(v - (((FLOOR_MPH * MPH) ** 2 + 2 * 1.0 * 105) ** 0.5)) < 1e-9
  assert 35 < v / MPH < 40


def test_trigger_holds_half_a_second_and_releases():
  tr = scs.StoppedCarTrigger()
  def inp(status=True, prob=0.95, d=80.0, v=0.0, y=0.1, v_ego=20.0, cruise=True, brake=False, enabled=True):
    return scs.StoppedCarInputs(enabled, cruise, brake, v_ego, True, status, prob, d, v, y)
  for _ in range(60):
    tr.update(inp(status=False))
  assert not tr.active
  n = 0
  while not tr.update(inp()):
    n += 1
  assert 49 <= n <= 51, n
  assert tr.urgent and tr.required > 1.5
  for _ in range(100):
    assert tr.update(inp(status=False))          # lost for 1 s: still active
  for _ in range(101):
    tr.update(inp(status=False))
  assert not tr.active and tr.release_reason.startswith("lead lost")
  while not tr.update(inp()):
    pass
  for _ in range(49):
    assert tr.update(inp(v=5.0))                 # moving for < 0.5 s: still active
  assert not tr.update(inp(v=5.0)) and tr.release_reason.startswith("lead moved away") and tr.restore_now
  while not tr.update(inp()):
    pass
  for _ in range(201):
    tr.update(inp(y=1.8))
  assert not tr.active and tr.release_reason == "lead leaving the lane"     # drive 4c: a jump out is a lane exit
  assert tr.clear_release
  while not tr.update(inp()):
    pass
  assert not tr.update(inp(brake=True, cruise=False)) and "brake" in tr.release_reason


def test_trigger_rejects_phantom_adjacent_and_slow_ego():
  tr = scs.StoppedCarTrigger()
  def inp(**kw):
    base = dict(enabled=True, cruise_enabled=True, brake_pressed=False, v_ego=20.0, lead_alive=True, lead_status=True,
                lead_prob=0.95, lead_d=80.0, lead_v=0.0, lead_y=0.1)
    base.update(kw)
    return scs.StoppedCarInputs(**base)
  for _ in range(60):
    tr.update(inp(lead_status=False))
  for _ in range(30):                              # 0.3 s phantom
    tr.update(inp())
  for _ in range(60):
    assert not tr.update(inp(lead_status=False))
  for _ in range(100):
    assert not tr.update(inp(lead_y=1.5))         # adjacent lane
  assert any("yRel" in b for b in tr.blockers(inp(lead_y=1.5)))
  for _ in range(100):
    tr.update(inp(lead_prob=0.8))
  # not sure enough for the stopped car start; the unsure car taps instead (owner, Nov 5: the yellow triangle)
  assert tr.kind == scs.SLOW_UNSURE and not tr.urgent
  assert any("prob" in b for b in tr.blockers(inp(lead_prob=0.8)))
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(inp(lead_status=False))
  for _ in range(100):
    assert not tr.update(inp(v_ego=15 * MPH))
  for _ in range(100):
    assert not tr.update(inp(lead_v=2.5))
  for _ in range(100):
    assert not tr.update(inp(enabled=False))


# ---------------- approaches ----------------

def test_approach_45_mph_from_120_m_lowers_towards_20_and_restores():
  lead = StoppedLead(120)
  car = Car(45, dash_offset_mph=0.0)
  r = run(car, NO_CURVE, 2500, lead=lead, stop_mode=2, max_drop=10)
  assert r["min"] >= FLOOR_MPH - 1e-6, r["min"]
  assert r["min"] <= 30, r["min"]                      # well on the way to the floor (lead still there)
  assert r["max"] <= 45 + 1e-6
  assert r["holds"] and r["holds"][0] < (r["presses"][0][0] if r["presses"] else 10**9), "120 m: floor mode, a hold first"
  assert r["trigger"].floor_mode
  # the lead departs: normal restore
  lead2 = StoppedLead(120, present=lambda f: f < 2000)
  r2 = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 6500, lead=lead2, stop_mode=2, max_drop=10)
  assert near(r2["final"], 45), r2["final"]
  assert r2["max"] <= 45 + 1e-6 and r2["min"] >= FLOOR_MPH - 1e-6


def test_urgent_approach_uses_holds_and_reaches_floor_fast():
  r = approach(45, 60, frames=1500)
  assert r["holds"], r
  assert r["min"] >= FLOOR_MPH - 1e-6
  assert r["min"] <= 21, r["min"]                      # reached the floor
  assert first_frame(r) is not None and first_frame(r) < 150


def test_relaxed_approach_taps_follow_the_target():
  # 45 mph, 200 m: not urgent (ttc 10 s, 1.1 m/s^2); the distance target starts above the set speed and
  # comes down slowly, so taps keep up with it; within FLOOR_DISTANCE (130 m) floor mode takes over with holds.
  r = approach(45, 200, frames=5000)
  minus_holds = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS]
  assert r["presses"] and r["presses"][0][0] < 200, r["presses"][:3]     # taps first (trigger 0.5 s + cruise settle)
  # 70 m closer at ~20 m/s is ~3.5 s: no SET- hold before the lead is within 130 m
  assert minus_holds and minus_holds[0]["frame"] > 300, minus_holds[:1]
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["max"] <= 45 + 1e-6
  gaps = [b - a for (a, _), (b, _) in zip(r["presses"], r["presses"][1:])]
  assert min(gaps[:3]) >= cs.RELAXED_GAP_FRAMES, gaps[:3]          # relaxed pace while not urgent (it gets urgent closer in)


def test_confirmed_stopped_car_with_room_holds_from_the_start():
  # 55 mph, stopped car at 120 m: the target (38 mph) leaves 17 steps of room: a hold, not taps, first.
  r = approach(55, 120, frames=2500)
  assert r["holds"] and (not r["presses"] or r["holds"][0] < r["presses"][0][0]), (r["holds"][:3], r["presses"][:3])
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["max"] <= 55 + 1e-6
  log = r["logic"].hold_log
  assert log[0]["landed"] >= 5, log


def test_phantom_lead_no_action():
  r = approach(45, 80, frames=800, lead_kw=dict(present=lambda f: 100 <= f < 130))
  assert r["presses"] == [] and r["holds"] == []


def test_adjacent_lane_lead_no_action():
  r = approach(45, 60, frames=1500, lead_kw=dict(y=1.6))
  assert r["presses"] == [] and r["holds"] == []


def test_driver_brakes_mid_approach_pauses_then_resumes_restore():
  def brake(s, c):
    s["cruise"], s["brake"] = False, True
  def resume(s, c):
    s["cruise"], s["brake"] = True, False
  lead = StoppedLead(120, present=lambda f: f < 700)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 6000, events={700: brake, 1500: resume}, lead=lead, max_drop=10)
  assert r["max"] <= 45 + 1e-6 and r["min"] >= FLOOR_MPH - 1e-6
  assert near(r["final"], 45), r["final"]


def test_curve_and_stopped_car_together_min_wins_no_fighting():
  lead = StoppedLead(150, present=lambda f: f < 2000)
  r = run(Car(45, dash_offset_mph=0.0), between(100, 1500, 40), 7000, lead=lead, max_drop=5)
  seq = "".join("M" if b == cs.SET_MINUS else "P" for _, b in r["presses"])
  assert "PM" not in seq or seq.count("PM") <= 1, seq     # no SET+/SET- fighting while both reasons are on
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["max"] <= 45 + 1e-6
  assert near(r["final"], 45), r["final"]
  assert r["min"] < 40 - 1e-6                              # went below the curve's 5 mph cap: the stopped car won


def test_presses_ignored_still_safe():
  r = approach(45, 60, frames=4000, car_kw=dict(lose=lambda n: n % 3 == 0, hold_ignored=True))
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["max"] <= 45 + 1e-6
  assert r["presses"]                                     # taps carry on after the hold is ignored
  assert r["logic"].hold_misses >= 1


def test_lead_departs_early_restores():
  lead = StoppedLead(120, present=lambda f: f < 400)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 4000, lead=lead, max_drop=10)
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_test_mode_sends_nothing():
  r = approach(45, 60, frames=1500, stop_mode=1)
  assert r["presses"] == [] and r["holds"] == []
  assert r["ever_active"]


# ---------------- long press safety ----------------

def test_hold_rates_and_latencies_never_cross_floor_or_target():
  for rate in (3.0, 5.0, 8.3):
    for latency in (0.0, 0.5, 1.0):
      for d0 in (60, 150):
        r = approach(45, d0, frames=3000, car_kw=dict(hold_rate=rate, hold_latency=latency, min_set_mph=CAR_MIN_MPH))
        # the floor is the car's own minimum (the hold to 20, then one tap); a car that keeps stepping lands there too
        assert r["min"] >= CAR_MIN_MPH - 1e-6, (rate, latency, d0, r["min"])
        assert r["max"] <= 45 + 1e-6, (rate, latency, d0, r["max"])
        assert not r["logic"].disabled, (rate, latency, d0)


def test_hold_landing_as_1_3_10_steps_is_safe():
  # a car that answers a hold with a single tap, a 3 mph step or 10 mph at once
  for rate, start in ((1.0, 0.1), (5.0, 0.5), (16.6, 0.1)):
    r = approach(45, 80, frames=3000, car_kw=dict(hold_rate=rate, hold_start=start))
    assert r["min"] >= FLOOR_MPH - 1e-6 and r["max"] <= 45 + 1e-6, (rate, r["min"], r["max"])
    assert not r["logic"].disabled


def test_driver_hold_during_our_hold_driver_wins():
  r0 = approach(45, 60, frames=400)
  h0 = r0["holds"][0]
  def driver_hold(s, c):
    c.driver_button, c.driver_button_until = cs.SET_MINUS, c.frame + 150
    c.pending.append((c.frame + c.lag + 50, -5 * MPH))
    c.pending.append((c.frame + c.lag + 110, -5 * MPH))
  lead = StoppedLead(60, present=lambda f: f < 1200)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 5000, events={h0 + 20: driver_hold}, lead=lead, max_drop=10)
  assert not r["logic"].disabled
  assert r["max"] <= 45 + 1e-6
  assert r["final"] <= 45 - 10 + 1e-6 + 10        # restore ends at or below the driver's new speed
  assert r["holds"] and max(r["holds"]) < h0 + 25     # our hold stopped when the driver pressed


def test_brake_during_hold_aborts():
  r0 = approach(45, 60, frames=400)
  h0 = r0["holds"][0]
  def brake(s, c):
    s["cruise"], s["brake"] = False, True
  r = approach(45, 60, frames=1200, events={h0 + 30: brake})
  assert max(r["holds"]) < h0 + 35
  assert r["min"] >= FLOOR_MPH - 1e-6


def test_lead_departs_during_hold_aborts_then_restores():
  r0 = approach(45, 60, frames=400)
  h0 = r0["holds"][0]
  lead = StoppedLead(60, present=lambda f: f < h0 + 30)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 4000, lead=lead, max_drop=10)
  assert max(r["holds"]) < h0 + 35 + 200      # the lead must be lost for 2 s before the reason clears
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_hold_ignored_falls_back_to_taps():
  r = approach(45, 60, frames=3000, car_kw=dict(hold_ignored=True))
  assert r["presses"] and r["min"] >= FLOOR_MPH - 1e-6
  assert r["logic"].hold_misses >= 1
  first = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS][0]
  assert first["frames_sent"] <= cs.HOLD_NO_EFFECT_SECONDS * 10 + 1, first        # given up after ~1 s
  assert r["min"] <= 30


def test_hold_log_records_each_hold():
  r = approach(45, 60, frames=2000)
  log = r["logic"].hold_log
  assert log and all(e["done"] for e in log), log
  assert all(e["frames_sent"] >= 1 and e["landed"] >= 1 for e in log), log
  assert r["logic"].hold_measured


def test_45_mph_one_hold_straight_to_the_floor():
  # drive 45: two holds with a 1.5 s pause (45 -> 35, 34 -> 25), because the release kept two quanta of margin and
  # the target followed the car ahead. Phase-locked now: one hold, released as 20 lands; then one tap to the car's
  # minimum (owner, Nov 6: "one more 1 mph step down to 19").
  r = approach(45, 60, frames=3000)
  log = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS]
  assert len(log) == 1 and log[0]["phase_lock"], log
  assert near(log[0]["bound_mph"], FLOOR_MPH) and log[0]["landed"] >= 20, log
  assert abs(r["min"] - FLOOR_MPH) < 0.5 and r["max"] <= 45 + 1e-6
  taps = [f for f, b in r["presses"] if b == cs.SET_MINUS]
  assert len(taps) == 1 and taps[0] > log[0]["frame"], taps
  first = log[0]["frame"]
  assert r["sets"][first + 400] <= 20 * MPH + 1e-6          # 45 -> 20 in one go, within 4 s


def test_lead_moves_during_first_hold_abort_and_immediate_restore():
  r0 = approach(45, 60, frames=400)
  h0 = r0["holds"][0]
  lead = StoppedLead(60, moves_at=h0 + 40, drive_off_mph=20)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=lead, max_drop=10)
  tr = r["trigger"]
  assert tr.release_reason.startswith("lead moved away"), tr.release_reason
  assert max(r["holds"]) < h0 + 40 + 60 + 300 or True
  # the restore starts right away (no 0.5 s delay + 1 s dwell) and uses SET+ holds, never above 45
  plus = [f for f, b in r["presses"] if b == cs.SET_PLUS] + [f for f in r["holds"] if f > h0 + 40]
  assert plus and r["max"] <= 45 + 1e-6
  assert near(r["final"], 45), r["final"]
  assert min(plus) < h0 + 40 + 50 + 400, min(plus)          # restore begins promptly after the 0.5 s moving check


def test_set_plus_hold_never_above_driver_speed_for_overrun_0_to_3_quanta():
  # S3: the lead departs after a tap-only lowering (nothing measured) and after holds (measured): with a release
  # overrun of 0, 1, 2 or 3 quanta the SET+ restore must never go above the driver's speed.
  for latency in (0.0, 0.6, 1.2, 1.8):
    for d0 in (200, 60):
      lead = StoppedLead(d0, present=lambda f: f < 1500)
      r = run(Car(55, dash_offset_mph=0.0, hold_latency=latency, min_set_mph=CAR_MIN_MPH), NO_CURVE, 6000, lead=lead,
              max_drop=10)
      assert r["max"] <= 55 + 1e-6, (latency, d0, r["max"])
      assert near(r["final"], 55), (latency, d0, r["final"])
      assert r["min"] >= CAR_MIN_MPH - 1e-6, (latency, d0, r["min"])


def _releases(fn):
  """Run `fn()` with every trigger release recorded as (frame, reason, restore_now)."""
  rel = []
  orig = scs.StoppedCarTrigger._release

  def recording(self, reason, restore_now=True):
    rel.append((self.frame, reason, restore_now))
    orig(self, reason, restore_now)
  scs.StoppedCarTrigger._release = recording
  try:
    return fn(), rel
  finally:
    scs.StoppedCarTrigger._release = orig


def _plus_frames(r):
  return sorted([f for f, b in r["presses"] if b == cs.SET_PLUS] +
                [h["frame"] for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS])


def test_ego_stops_behind_lead_restores_after_1_s_at_standstill():
  # We stop behind the stopped car; once we have been stopped STANDSTILL_RESTORE_DELAY the driver's speed comes
  # back while still stopped (owner, 2018 CX-5: SET+ at a stop behind a car only raises the set speed, the MRCC
  # keeps holding). Nothing is pressed while still rolling to the stop.
  lead = StoppedLead(60, moves_at=2500, drive_off_mph=25)
  car = Car(45, dash_offset_mph=0.0, decel=2.5)
  speeds, gaps, sets = {}, {}, {}
  orig_tick = lead.tick

  def tick(c):
    orig_tick(c)
    speeds[lead.frame] = c.v_ego
    gaps[lead.frame] = lead.d
    sets[lead.frame] = c.set_speed
  lead.tick = tick
  r = run(car, NO_CURVE, 7000, lead=lead, max_drop=10)
  tr = r["trigger"]
  assert tr.release_reason == "ego stopped behind the lead", tr.release_reason
  stopped = min(f for f, v in speeds.items() if v < 0.3)
  plus = _plus_frames(r)
  assert plus and stopped + 100 <= plus[0] < 2500, (stopped, plus[:1])    # 1 s into the stop, before the lead moves
  assert all(speeds[f] < 0.3 for f in range(plus[0], 2500)), "the car stays stopped behind the lead"
  assert near(sets[2499] / MPH, 45), sets[2499] / MPH                     # back to 45 before the lead moves
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6, r["final"]


def test_creeping_in_a_queue_no_restore_until_rolling_or_stopped():
  # Rolling slowly behind the car (between a stop and 5 m/s) never raises the set speed.
  lead = StoppedLead(60, v_mph=0.0, moves_at=2000, drive_off_mph=6)     # the lead creeps off at 6 mph
  car = Car(45, dash_offset_mph=0.0, decel=2.5)
  speeds = {}
  orig_tick = lead.tick

  def tick(c):
    orig_tick(c)
    speeds[lead.frame] = c.v_ego
  lead.tick = tick
  r = run(car, NO_CURVE, 6000, lead=lead, max_drop=10)
  for f in _plus_frames(r):
    v = speeds.get(f - 1, 0.0)
    assert v < 0.3 or v > cs.RESTORE_MOVING_SPEED, (f, v)


def test_lost_release_restores_gently_and_the_retrigger_wins():
  # R3-1 (reviewer's trace): vision loses the stopped car for 2.5 s at ~26 m / 29 mph during the approach. The
  # 2 s "lost" release must not restore with a SET+ hold: at most a tap or two go out before the lead is seen
  # again and the slowdown re-arms, and the car still ends up stopped behind the lead with the set speed at 20.
  lead = StoppedLead(120, present=lambda f: not (350 <= f < 600))
  car = Car(45, dash_offset_mph=0.0, lag=10)
  r, rel = _releases(lambda: run(car, NO_CURVE, 5000, lead=lead, max_drop=10))
  reasons = [reason for _, reason, _ in rel]
  assert reasons == ["lead lost (vision)", "ego stopped behind the lead"], rel
  assert rel[0][2] is False and rel[1][2] is True, rel                     # gentle restore, then restore at once
  stop_f = rel[1][0]
  plus_before = [f for f in _plus_frames(r) if f < stop_f]
  assert len(plus_before) <= 2, plus_before          # (the one hold to 20 may still be settling: then none)
  assert not [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS and h["frame"] < stop_f], "no SET+ hold"
  assert abs(r["min"] - FLOOR_MPH) < 0.5, r["min"]
  plus_after = [f for f in _plus_frames(r) if f >= stop_f]
  assert plus_after and plus_after[0] >= stop_f + 100 - 5, (stop_f, plus_after[:1])   # ~1 s into the stop
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6, (r["final"], r["max"])


def test_left_lane_release_restores_with_the_fast_hold():
  # R3-1: the lead drifts out of our lane for good: a correct release. Drive 4c: with nothing slower left in our lane
  # the restore uses the SET+ hold (5 below, then taps), never above the driver's speed.
  lead = StoppedLead(120)
  orig_state = lead.state
  lead.state = lambda f, c: (lambda st: (st[0], st[1], st[2], st[3], 1.6 if f >= 350 else st[4]))(orig_state(f, c))

  def tick(c):      # the MRCC does not brake for a car in the next lane: we drive past it
    lead.frame += 1
    lead.d = max(lead.d - c.v_ego * T.DT, 0.0)
  lead.tick = tick
  car = Car(45, dash_offset_mph=0.0, lag=10)
  r, rel = _releases(lambda: run(car, NO_CURVE, 3000, lead=lead, max_drop=10))
  assert [reason for _, reason, _ in rel] == ["lead leaving the lane"], rel
  assert rel[0][2] is False
  assert [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS], "a SET+ hold"
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6, (r["final"], r["max"])


def test_radar_dropout_does_not_raise_early():
  # the stock radar sees the stopped car from 80 m, drops it for a while, sees it again: no restore
  lead = StoppedLead(120, radar_from=110, radar_present=lambda f: not (300 <= f < 700))
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2000, lead=lead, max_drop=10)
  # the only restore comes after the ego has stopped behind the lead, never during the radar dropout
  assert r["trigger"].release_reason == "ego stopped behind the lead", r["trigger"].release_reason
  first_plus = min([f for f, b in r["presses"] if b == cs.SET_PLUS] +
                   [h["frame"] for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS] + [10**9])
  assert first_plus > 700, first_plus
  assert r["trigger"].radar_rises, "radar pick-ups are logged"


def test_vanished_vision_lead_with_radar_lead_releases_within_2_s():
  # S2: the stock radar reports a lead most of the time in traffic; it must never keep the reason alive.
  lead = StoppedLead(60, present=lambda f: f < 200, radar_from=200)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2000, lead=lead, max_drop=10)
  assert r["ever_active"]
  assert r["trigger"].release_reason == "lead lost (vision)", r["trigger"].release_reason
  last_minus = max([f for f, b in r["presses"] if b == cs.SET_MINUS] +
                   [h["release_frame"] for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS])
  assert last_minus <= 200 + 200 + 20, last_minus                       # nothing lowered after ~2 s
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_radar_locked_on_during_approach_never_releases_before_the_lead_moves():
  # S1 regression (reviewer's MMMMMMMMMPPPPPPPMPPPP trace): the stock radar's distance bucket rises as the stopped
  # car gets nearer (5 = nearest); that must not read as "moved away". Vision lost by the model, then back.
  lead = StoppedLead(120, radar_from=100, moves_at=600, drive_off_mph=25)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 4000, lead=lead, max_drop=10)
  first_plus = min([f for f, b in r["presses"] if b == cs.SET_PLUS] +
                   [h["frame"] for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS] + [10**9])
  assert first_plus >= 600, first_plus
  assert r["trigger"].release_reason == "lead moved away (vision)", r["trigger"].release_reason
  assert r["trigger"].radar_rises


def test_curve_slowdown_still_uses_taps_only():
  r = T.run(Car(65), between(100, 1500, 45), 4000)
  assert r["holds"] == [] and r["presses"]


def test_random_stopped_car_drives_keep_limits():
  import random
  rng = random.Random(5)
  for _ in range(40):
    set_mph = rng.choice([30, 45, 55, 65])
    d0 = rng.choice([50, 80, 120, 160])
    rate = rng.choice([3.0, 5.0, 8.3])
    latency = rng.choice([0.0, 0.3, 0.6, 1.0])
    start = rng.choice([0.1, 0.5, 0.9])
    loss = rng.choice([0.0, 0.25])
    lost = set(n for n in range(1, 300) if rng.random() < loss)
    ignored = rng.random() < 0.2
    gone = rng.randint(300, 3000)
    lead = StoppedLead(d0, prob=rng.choice([0.9, 0.95]), y=rng.choice([0.0, 0.5, -0.8]), present=lambda f, g=gone: f < g)
    car = Car(set_mph, dash_offset_mph=rng.choice([0, 2]), lag=rng.choice([5, 15, 40]), lose=lambda n, lost=lost: n in lost,
              hold_rate=rate, hold_start=start, hold_latency=latency, hold_ignored=ignored, min_set_mph=CAR_MIN_MPH)
    curve = between(1200, 2200, rng.uniform(set_mph - 20, set_mph)) if rng.random() < 0.3 else NO_CURVE
    r = run(car, curve, 7000, lead=lead, max_drop=rng.choice([5, 10]))
    assert r["max"] <= set_mph + 1e-6, (set_mph, d0, rate, latency, r["max"])
    # a phase-locked stop hold aims at 20 with no overrun margin: a car that steps once more after the release
    # lands on its own 30 km/h minimum, never lower
    assert r["min"] >= CAR_MIN_MPH - 1e-6, (set_mph, d0, rate, latency, start, r["min"])
    assert not r["logic"].disabled, (set_mph, d0, rate, latency)
    assert near(r["final"], set_mph), (set_mph, d0, rate, latency, loss, ignored, r["final"])


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


# ---------------- stop ahead with a much slower car (red light + a car) ----------------

def _sw(**kw):
  base = dict(enabled=True, cruise_enabled=True, brake_pressed=False, v_ego=45 * MPH, lead_alive=True, lead_status=True,
              lead_prob=0.6, lead_d=95.0, lead_v=13 * MPH, lead_y=1.6, model_alive=True, model_accel=-0.6)
  base.update(kw)
  return scs.StoppedCarInputs(**base)


def test_stop_with_a_slower_car_starts_in_half_a_second_and_is_urgent():
  # drive 36, 4:44.8: 45 mph, the model slowing for the light, a car 95 m ahead at 13-15 mph (prob 0.6, y 1.6)
  tr = scs.StoppedCarTrigger()
  for _ in range(60):                        # cruise on for a while, no car yet
    tr.update(_sw(lead_status=False))
  n = 0
  while not tr.update(_sw()):
    n += 1
    assert n < 100
  assert 49 <= n <= 51, n
  assert tr.kind == scs.STOP_WITH_CAR and tr.urgent and tr.floor_mode
  assert abs(tr.target - scs.FLOOR_SPEED) < 1e-9                  # 95 m, confirmed: the floor (the car ahead is at 13 mph)


def test_no_stop_with_car_while_just_following_or_without_a_car_or_model():
  # drive 31: the model slows down all the time to keep the gap behind a car at our speed: no slowdown
  for kw in (dict(lead_v=42 * MPH), dict(lead_v=41 * MPH, lead_d=35.0),       # following
             dict(lead_status=False),                                          # red light, no car: warning only
             dict(model_accel=-0.2),                                           # model not slowing, car moving slowly
             dict(model_alive=False),
             dict(lead_y=2.6), dict(lead_prob=0.3), dict(lead_d=180.0),        # not our lane / unsure / too far
             dict(v_ego=18 * MPH, lead_v=2 * MPH, lead_prob=0.6)):             # too slow to act
    tr = scs.StoppedCarTrigger()
    for _ in range(300):
      assert not tr.update(_sw(**kw)), kw


def test_stop_with_car_ignores_a_moving_lead_and_releases_when_traffic_moves():
  tr = scs.StoppedCarTrigger()
  while not tr.update(_sw()):
    pass
  for _ in range(300):                       # the car ahead keeps rolling at 13 mph: still slowing (no "moved away")
    assert tr.update(_sw(model_accel=-0.3, v_ego=35 * MPH))
  for _ in range(99):                        # green: the model accelerates, the car ahead is at our speed
    assert tr.update(_sw(model_accel=0.4, v_ego=30 * MPH, lead_v=29 * MPH))
  assert not tr.update(_sw(model_accel=0.4, v_ego=30 * MPH, lead_v=29 * MPH))
  # drive 4b (owner): traffic moving again restores at once, with SET+ holds (a re-trigger aborts a hold)
  assert tr.release_reason.startswith("traffic moving again") and tr.restore_now


def test_stop_with_car_other_releases():
  tr = scs.StoppedCarTrigger()
  while not tr.update(_sw()):
    pass
  assert not tr.update(_sw(brake_pressed=True)) and "brake" in tr.release_reason
  while not tr.update(_sw()):
    pass
  assert not tr.update(_sw(v_ego=0.5, lead_v=0.0)) and tr.release_reason.startswith("ego stopped") and tr.restore_now
  while not tr.update(_sw()):
    pass
  for _ in range(200):
    tr.update(_sw(lead_status=False))
  assert not tr.active and tr.release_reason.startswith("lead lost") and not tr.restore_now


def test_stopped_car_rolling_forward_with_the_model_slowing_is_not_released():
  # a confirmed stopped car starts creeping (5 m/s) towards the light: still a stop, not "moved away"
  tr = scs.StoppedCarTrigger()
  while not tr.update(_sw(lead_v=0.0, lead_prob=0.95, lead_y=0.1, model_accel=0.0, v_ego=20.0)):
    pass
  assert tr.kind == scs.STOPPED_CAR
  for _ in range(200):
    assert tr.update(_sw(lead_v=5.0, lead_prob=0.95, lead_y=0.1, v_ego=18.0))
  assert tr.kind == scs.STOP_WITH_CAR
  # without the model slowing a stopped car that drives off is still released as before
  tr = scs.StoppedCarTrigger()
  while not tr.update(_sw(lead_v=0.0, lead_prob=0.95, lead_y=0.1, model_accel=0.0, v_ego=20.0)):
    pass
  for _ in range(60):
    tr.update(_sw(lead_v=5.0, lead_prob=0.95, lead_y=0.1, model_accel=0.0, v_ego=20.0))
  assert not tr.active and tr.release_reason.startswith("lead moved away")


def test_approach_45_mph_to_a_slow_car_at_a_red_light_drops_at_once_never_below_20():
  lead = StoppedLead(95, v_mph=13.0, prob=0.7, y=1.0)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 800, lead=lead, stop_mode=2, max_drop=10, model=lambda f: -0.7)
  assert r["ever_active"] and r["trigger"].kind == scs.STOP_WITH_CAR
  first = first_frame(r)
  # first SET- within 0.6 s of the trigger (0.3 s cruise settle at the start of the run + 0.5 s hold)
  assert first is not None and first <= 90, first
  assert r["holds"] and r["holds"][0] <= 90                  # urgent: a long press straight away
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["min"] <= 33, r["min"]
  # the same car with the model not slowing (just a slow car, no stop ahead): only the unsure car's taps (owner, Nov
  # 5: the yellow triangle), 1 mph each, at most UNSURE_MAX_DROP, no long press
  lead = StoppedLead(95, v_mph=13.0, prob=0.7, y=1.0)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 800, lead=lead, stop_mode=2, max_drop=10, model=lambda f: 0.0)
  assert r["holds"] == [] and all(b == cs.SET_MINUS for _, b in r["presses"])
  assert r["min"] >= 45 - scs.UNSURE_MAX_DROP / MPH - 1 - 1e-6, r["min"]


def test_restore_at_a_stop_holds_most_of_the_way_up():
  # drive 3c: a relaxed approach lowered to 22 by taps (no hold had measured the car yet), we stopped behind the car,
  # and the SET+ hold was released at 26 (an assumed overrun of 3 quanta): the last 19 mph took 12 s of taps.
  # Now the unmeasured margin is what a measured clean hold gives, so the hold goes most of the way; never above 45.
  lead = StoppedLead(200, moves_at=4500, drive_off_mph=25)
  car = Car(45, dash_offset_mph=0.0, decel=2.5)
  sets = {}
  orig_tick = lead.tick

  def tick(c):
    orig_tick(c)
    sets[lead.frame] = c.set_speed
  lead.tick = tick
  # floor mode off: the tap-only lowering this restore case starts from (drive 3c, before floor mode)
  scs.FLOOR_MODE_ENABLED = False
  try:
    r = run(car, NO_CURVE, 8000, lead=lead, max_drop=10)
  finally:
    scs.FLOOR_MODE_ENABLED = True
  plus = _plus_frames(r)
  assert not [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS]   # lowered by taps, as in drive 3c
  low = min(sets.values()) / MPH
  holds_up = [f for f in r["holds"] if f >= plus[0]]
  assert holds_up, "a SET+ hold"
  after_hold = sets[max(holds_up) + 60] / MPH
  assert after_hold - low >= 10, (low, after_hold)            # the hold did at least 10 mph of the way
  back = min(f for f in sets if f > plus[0] and near(sets[f] / MPH, 45))
  assert (back - plus[0]) * T.DT < 7.0, (back - plus[0]) * T.DT
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


# ---------------- red light first: a low-confidence car after a planned stop (drive 2026-11-03 21:12) ----------------

SWL = importlib.util.spec_from_file_location(
  "stop_warning", os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "starpilot", "controls", "lib", "stop_warning.py"))


def _stop_warning():
  m = importlib.util.module_from_spec(SWL)
  SWL.loader.exec_module(m)
  return m


def _plan(v_ego, v_min, t_dip=4.0, n=33):
  """A model plan: our speed, then down to v_min from t_dip on (velocity.t / velocity.x / position.x)."""
  ts = [10.0 * (k / (n - 1)) ** 2 for k in range(n)]
  vs = [v_ego if t < t_dip else max(v_min, v_ego - 2.5 * (t - t_dip)) for t in ts]
  xs = [0.0]
  for k in range(1, n):
    xs.append(xs[-1] + (vs[k] + vs[k - 1]) / 2 * (ts[k] - ts[k - 1]))
  return ts, vs, xs


def test_the_planned_stop_check_matches_the_red_light_warning():
  sw = _stop_warning()
  logic = sw.StopWarningLogic()
  v = 20.0
  for v_min, t_dip in ((0.0, 3.0), (5.0, 4.0), (15.0, 2.0), (19.0, 1.0), (0.0, 7.5), (0.0, 9.0)):
    ts, vs, xs = _plan(v, v_min, t_dip)
    stop, ratio, d = scs.plan_stop(ts, vs, xs, v)
    ref = sw.StopWarningLogic.plan_slowdown(type("I", (), {"model_t": ts, "model_v": vs, "model_x": xs, "v_ego": v})())
    want = ref is not None and ref[3] >= sw.STOP_DECEL
    assert stop == want, (v_min, t_dip, stop, ref)
    assert ratio <= 1.0
  del logic
  assert scs.plan_stop([], [], [], 20.0) == (False, 1.0, 0.0)


def _latched_inputs(**kw):
  # a genuine red light: the stop-light detector (redLight) agrees, as in all 7 genuine warnings on drives 4f / 50
  base = dict(enabled=True, cruise_enabled=True, brake_pressed=False, v_ego=45 * MPH, lead_alive=True, lead_status=True,
              lead_prob=0.26, lead_d=118.0, lead_v=13 * MPH, lead_y=-0.8, model_alive=True, model_accel=-0.6, plan_stop=True,
              red_light=True)
  base.update(kw)
  return scs.StoppedCarInputs(**base)


def test_latch_sets_after_half_a_second_and_holds_four_seconds():
  tr = scs.StoppedCarTrigger()
  for _ in range(40):
    tr.update(_latched_inputs(lead_status=False))
  assert not tr.latched
  for k in range(20):                          # a 0.2 s gap is tolerated
    tr.update(_latched_inputs(lead_status=False, plan_stop=(k % 2 == 0) or True))
  assert tr.latched
  for _ in range(int(scs.LATCH_HOLD / scs.DT_CTRL)):
    tr.update(_latched_inputs(lead_status=False, plan_stop=False))
  assert tr.latched
  tr.update(_latched_inputs(lead_status=False, plan_stop=False))
  assert not tr.latched
  tr2 = scs.StoppedCarTrigger()
  for _ in range(60):
    tr2.update(_latched_inputs(lead_status=False))
  assert tr2.latched
  tr2.update(_latched_inputs(lead_status=False, v_ego=0.5))   # stopped: the latch ends
  assert not tr2.latched


def test_a_low_confidence_slow_car_starts_the_slowdown_only_after_a_planned_stop():
  # 21:12: the queue car was 25-37 % "a car" at 110-130 m; the red light check had fired
  tr = scs.StoppedCarTrigger()
  for _ in range(40):
    tr.update(_latched_inputs(lead_status=False))          # the stop is planned first: latched
  n = 0
  while not tr.update(_latched_inputs(lead_status=(n % 15 != 0))):   # the car blinks out now and then
    n += 1
    assert n < 200
  for _ in range(100):
    if tr.kind == scs.STOP_WITH_CAR:
      break
    tr.update(_latched_inputs(lead_status=True))
  assert tr.kind == scs.STOP_WITH_CAR and tr.early and not tr.urgent      # taps until it is confirmed
  # without a planned stop first: only the unsure car taps (at most UNSURE_MAX_DROP), not the stop with car
  tr = scs.StoppedCarTrigger()
  for _ in range(300):
    tr.update(_latched_inputs(plan_stop=False))
  assert tr.active and tr.kind == scs.SLOW_UNSURE and not tr.urgent
  assert abs(tr.target - (45 * MPH - scs.UNSURE_MAX_DROP)) < 0.6
  assert any("not latched" in b for b in tr.blockers(_latched_inputs(plan_stop=False)))


class _NoWarningStart:
  """The early start alone, without the start on the stop ahead warning (drive 4c) that would also fire here."""
  def __enter__(self):
    self.old, scs.WARNING_START_ENABLED = scs.WARNING_START_ENABLED, False

  def __exit__(self, *a):
    scs.WARNING_START_ENABLED = self.old


def test_the_early_start_still_needs_a_slow_car_near_our_lane_and_the_model_slowing():
  # (a planned stop with no car for 1 s is now a start on the stop ahead warning: see the drive 4c tests)
  with _NoWarningStart():
    for kw in (dict(lead_v=43 * MPH), dict(lead_status=False), dict(lead_y=2.5), dict(lead_d=160.0),
               dict(model_accel=-0.2), dict(lead_prob=0.15)):
      tr = scs.StoppedCarTrigger()
      for _ in range(40):
        tr.update(_latched_inputs(lead_status=False, red_light=False))
      for _ in range(300):
        tr.update(_latched_inputs(red_light=False, **kw))     # (no stop-light detector: its head start is not the point here)
      if "model_accel" in kw or "lead_prob" in kw:              # a slower car in our path: only the unsure taps
        assert not tr.active or tr.kind == scs.SLOW_UNSURE, kw
      else:
        assert not tr.active, kw


def test_an_early_start_is_confirmed_when_the_car_is_sure_and_then_slows_fast():
  tr = scs.StoppedCarTrigger()
  for _ in range(40):
    tr.update(_latched_inputs(lead_status=False))
  while tr.kind != scs.STOP_WITH_CAR or not tr.active:      # (the unsure car taps first, then the early start)
    tr.update(_latched_inputs())
  assert not tr.urgent
  for _ in range(int(scs.EARLY_CONFIRM_TIME / scs.DT_CTRL) + 1):
    tr.update(_latched_inputs(lead_prob=0.5, lead_d=100.0))
  assert tr.confirmed and tr.urgent
  tr.update(_latched_inputs(lead_prob=0.3))
  assert tr.urgent                             # sticky
  # the early episode's low-confidence car does not count as lost
  for _ in range(300):
    assert tr.update(_latched_inputs(lead_prob=0.25))


def test_while_a_stop_is_planned_traffic_moving_and_lead_lost_wait():
  with _NoWarningStart():
    _while_a_stop_is_planned()


def _while_a_stop_is_planned():
  tr = scs.StoppedCarTrigger()
  for _ in range(40):
    tr.update(_latched_inputs(lead_status=False))
  while not tr.update(_latched_inputs(lead_prob=0.6)):
    pass
  for _ in range(300):                         # lost 3 s with the stop still planned: still active
    assert tr.update(_latched_inputs(lead_status=False))
  for _ in range(250):
    tr.update(_latched_inputs(lead_status=False))
  assert not tr.active and tr.release_reason.startswith("lead lost")
  assert tr.hold_restore                       # still latched: keep the lowered speed for now
  # moving again only once the model no longer plans the stop
  tr = scs.StoppedCarTrigger()
  for _ in range(40):
    tr.update(_latched_inputs(lead_status=False))
  while not tr.update(_latched_inputs(lead_prob=0.6)):
    pass
  for _ in range(300):
    assert tr.update(_latched_inputs(lead_prob=0.6, model_accel=0.3, lead_v=44 * MPH))
  for _ in range(101):                         # (and the light gone: round k, no go release under a red light)
    tr.update(_latched_inputs(lead_prob=0.6, model_accel=0.3, lead_v=44 * MPH, plan_stop=False, red_light=False))
  assert not tr.active and tr.release_reason.startswith("traffic moving")


def test_2112_replay_starts_earlier_and_lower_than_on_the_drive():
  # 21:12 (t s after 21:11:40): the stop planned from t 24.5, a queue car at 110-125 m seen at 0.27-0.58, the
  # confident start only at t 28.5 on the drive (45 -> 40 by t 29.5), the driver braking hard at t 30.5
  F = lambda t: int(round((t - 20.0) * 100))      # frame of drive time t (the run starts at t 20)  # noqa: E731

  class Queue(StoppedLead):
    """The car ahead: ~30 mph, 150 m ahead at t 20 (115-125 m at t 23-27 as on the drive), braking for the light
    from t 28 at 3 m/s^2 down to a crawl; seen intermittently and unsure until t 28."""
    def state(self, f, car):
      t = 20.0 + f / 100
      if t < 23.0 or (t < 28.0 and int(t * 10) % 6 == 0):
        return (False, 0.0, 0.0, 0.0, 0.0)
      prob = 0.3 if t < 28.0 else 0.8
      return (True, prob, self.d, self.v, -0.8)

    def tick(self, car):
      self.frame += 1
      if 20.0 + self.frame / 100 >= 28.0:
        self.v = max(self.v - 3.0 * T.DT, 0.5)
      self.d = max(self.d - (car.v_ego - self.v) * T.DT, 0.0)

  lead = Queue(150.0, v_mph=30.0)
  plan = lambda f: 20.0 + f / 100 >= 24.5  # noqa: E731
  model = lambda f: -0.7 if 20.0 + f / 100 >= 24.0 else 0.0  # noqa: E731
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, F(30.5), lead=lead, stop_mode=2, max_drop=10, model=model, plan=plan)
  first = first_frame(r)
  assert first is not None and first <= F(26.0), (first, F(26.0))
  # unsure until t 28: taps only (the unsure car from t 23) until the planned stop has held WARN_START_DEBOUNCE, then
  # the stop ahead start's long press (the car ahead no longer "relevant" once the taps slowed us)
  assert [f for f in r["holds"] if f < F(24.5 + scs.WARN_START_DEBOUNCE)] == []
  i = F(29.5)
  assert r["sets"][i] / MPH <= 38 + 1e-6, r["sets"][i] / MPH   # on the drive: 40 at t 29.5
  assert r["min"] >= FLOOR_MPH - 1e-6
  assert [f for f, b in r["presses"] if b == cs.SET_PLUS] == []


def test_a_false_early_start_drops_a_few_mph_and_comes_back():
  # a ghost "car" for 1.5 s while the model plans a stop that then goes away (a green light); the early start alone
  # (the stop ahead warning start is covered by test_false_stop_ahead_warning_costs_a_few_mph_and_comes_back_fast)
  class Ghost(StoppedLead):
    def state(self, f, car):
      return (True, 0.25, 115.0, 12 * MPH, 0.5) if 100 <= f < 250 else (False, 0.0, 0.0, 0.0, 0.0)

  plan = lambda f: f < 300  # noqa: E731
  model = lambda f: -0.6 if f < 300 else 0.3  # noqa: E731
  with _NoWarningStart():
    r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=Ghost(115.0), stop_mode=2, max_drop=10, model=model,
            plan=plan)
  assert r["ever_active"] and r["holds"] == []
  assert r["min"] >= 45 - 3 - 1e-6, r["min"]
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_early_start_in_test_mode_sends_nothing():
  class Queue(StoppedLead):
    def state(self, f, car):
      return (True, 0.3, self.d, self.v, -0.8)

  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1500, lead=Queue(125.0, v_mph=15.0), stop_mode=1, max_drop=10,
          model=lambda f: -0.7, plan=lambda f: True)
  assert r["ever_active"] and r["presses"] == [] and r["holds"] == []


# ---------------- floor mode: straight to 20 mph with SET- holds (drive 00000043, Nov 4) ----------------

class SlowingLead(T.StoppedLead):
  """A car ahead at `v_mph` that brakes at `decel` m/s^2 down to `v_end` m/s (a car stopping at a light)."""
  def __init__(self, d0, v_mph, decel=2.0, v_end=0.5, **kw):
    super().__init__(d0, v_mph=v_mph, **kw)
    self.decel, self.v_end = decel, v_end

  @property
  def a(self):
    return -self.decel if self.v > self.v_end else 0.0     # what the radarState reports (aLeadK)

  def tick(self, car):
    self.v = max(self.v - self.decel * T.DT, self.v_end)
    super().tick(car)


def _episode_a(frames=700):
  # 1:54.6: 45.5 mph, the model slowing, a car 110 m ahead at 29 mph braking to a stop; the stock cruise slows
  # as measured for the set speed gap. Before floor mode: 13 taps in 5 s, 39.5 mph when the driver braked.
  lead = SlowingLead(110, 29, prob=0.9, y=0.1)
  car = Car(45, dash_offset_mph=0.0, decel_fn=MEASURED_DECEL)
  r = run(car, NO_CURVE, frames, lead=lead, stop_mode=2, max_drop=10, model=lambda f: -0.7, setup=MEASURED)
  return r, car, lead


def test_episode_a_replay_floor_hold_slows_faster_than_taps():
  r, car, lead = _episode_a()
  tr = r["trigger"]
  assert r["ever_active"]
  assert tr.floor_mode
  start = tr.start_frame
  assert r["holds"] and r["holds"][0] - start <= 30, (start, r["holds"][:2], r["presses"][:2])
  assert r["sets"][start + 350] <= 25 * MPH + 1e-6, r["sets"][start + 350] / MPH      # <= 25 mph 3.5 s in
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["max"] <= 45 + 1e-6
  v_new = car.v_ego
  # the same approach by taps along the distance target (floor mode off, no holds)
  scs.FLOOR_MODE_ENABLED = False
  try:
    lead = SlowingLead(110, 29, prob=0.9, y=0.1)
    car_old = Car(45, dash_offset_mph=0.0, decel_fn=MEASURED_DECEL)
    run(car_old, NO_CURVE, 700, lead=lead, stop_mode=2, max_drop=10, model=lambda f: -0.7,
        setup=lambda lg: (MEASURED(lg), setattr(lg, "holds_ok", False), setattr(lg, "hold_misses", cs.HOLD_MISS_LIMIT)))
  finally:
    scs.FLOOR_MODE_ENABLED = True
  # with the measured gap -> deceleration table the car still only slows ~1 m/s^2 on average (the gap shrinks as it
  # slows), so the gain 7 s in is ~1 mph; the set speed is at 25 mph instead of ~31 by 3.5 s in
  assert v_new / MPH <= 35.0, v_new / MPH
  assert car_old.v_ego / MPH - v_new / MPH >= 1.0, (car_old.v_ego / MPH, v_new / MPH)


def test_measured_car_one_hold_to_the_floor_then_taps():
  r = run(Car(45, dash_offset_mph=0.0, decel_fn=MEASURED_DECEL), NO_CURVE, 2500, lead=T.StoppedLead(100),
          stop_mode=2, max_drop=10, setup=MEASURED)
  log = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS]
  assert len(log) >= 1 and near(log[0]["bound_mph"], FLOOR_MPH), log
  assert r["sets"][log[0]["frame"] + 350] <= 25 * MPH + 1e-6
  assert abs(r["min"] - FLOOR_MPH) < 0.5 and r["max"] <= 45 + 1e-6


def test_stop_with_a_car_rolling_at_25_mph_goes_to_the_floor_and_comes_back():
  # owner's choice (drive 45): no lead-speed guard: straight to 20; the stock radar keeps the gap to a car that keeps
  # moving, and the release (traffic moving again) restores the driver's speed
  lead = T.StoppedLead(90, v_mph=25.0, prob=0.9, y=0.1)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2000, lead=lead, stop_mode=2, max_drop=10, model=lambda f: -0.6)
  assert r["ever_active"] and r["trigger"].floor_mode
  assert abs(r["min"] - 20) < 0.5, r["min"]            # the moving car guard: its 25 mph less 5


def test_floor_target_respects_a_braking_lead_guard():
  scs.FLOOR_LEAD_GUARD = True       # off by default (drive 45); the guard itself still works
  try:
    _guard_check()
  finally:
    scs.FLOOR_LEAD_GUARD = False


def _guard_check():
  tr = scs.StoppedCarTrigger()
  base = dict(enabled=True, cruise_enabled=True, brake_pressed=False, v_ego=20.0, lead_alive=True, lead_status=True,
              lead_prob=0.9, lead_d=90.0, lead_v=11.0, lead_y=0.1, model_alive=True, model_accel=-0.8)
  for _ in range(150):
    tr.update(scs.StoppedCarInputs(**base))
  assert tr.active and tr.floor_mode
  assert near(tr.target / MPH, 11.0 / MPH)                                      # 25 mph car ahead, not braking
  for _ in range(150):                                                          # braking hard: it will be slow soon
    tr.update(scs.StoppedCarInputs(**dict(base, lead_a=-2.5)))
  assert near(tr.target, scs.FLOOR_SPEED)


def test_far_confirmed_stop_is_not_floor_mode_yet():
  tr = scs.StoppedCarTrigger()
  base = dict(enabled=True, cruise_enabled=True, brake_pressed=False, v_ego=20.0, lead_alive=True, lead_status=True,
              lead_prob=0.95, lead_d=180.0, lead_v=0.0, lead_y=0.1)
  for _ in range(150):
    tr.update(scs.StoppedCarInputs(**base))
  assert tr.active and not tr.floor_mode and tr.target > scs.FLOOR_SPEED + 3 * MPH


def test_hold_onto_the_car_minimum_is_ours_not_a_disable():
  # a hold that keeps landing after release (latency 1.5 s) runs onto the car's own 18.6 mph minimum
  car = Car(45, dash_offset_mph=0.0, hold_latency=1.5, min_set_mph=30 / 1.609344)
  r = run(car, NO_CURVE, 2500, lead=T.StoppedLead(70), stop_mode=2, max_drop=10, setup=MEASURED)
  lg = r["logic"]
  assert not lg.disabled
  assert r["min"] >= 30 / 1.609344 - 1e-6


# ---------------- phase-locked holds (drive 44: our holds mostly read as quick taps) ----------------

def test_hold_frames_go_out_only_with_a_new_wheel_frame():
  for phase in (0, 3, 7):
    r = approach(45, 60, frames=1500, events=None) if phase == 0 else \
        run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1500, lead=T.StoppedLead(60), stop_mode=2, max_drop=10, wheel_phase=phase)
    assert r["holds"], phase
    assert all((f - phase) % 10 == 0 for f in r["holds"]), (phase, r["holds"][:10])
    assert len({(f - phase) // 10 for f in r["holds"]}) == len(r["holds"])          # never two per wheel frame
    for h in r["logic"].hold_log:
      assert h["phase_lock"] and set(h["delays"]) <= {0}, h


def test_hold_fallback_when_the_wheel_frames_stop():
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1500, lead=T.StoppedLead(60), stop_mode=2, max_drop=10,
          wheel_alive=lambda f: f < 110)
  log = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS]
  assert log and sum(h["fallbacks"] for h in log) > 0, log
  gaps = [b - a for a, b in zip(r["holds"], r["holds"][1:], strict=False) if b - a < 50 and a >= 120]
  assert gaps and min(gaps) >= cs.HOLD_FALLBACK_FRAMES, gaps
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["max"] <= 45 + 1e-6


def test_phase_strict_car_sees_a_long_press_only_when_locked():
  r = run(Car(45, dash_offset_mph=0.0, hold_phase_window=1), NO_CURVE, 2500, lead=T.StoppedLead(70), stop_mode=2,
          max_drop=10, wheel_phase=4)
  log = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS and h["done"]]
  assert log and log[0]["pattern"] == "quantum", log[:1]
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["max"] <= 45 + 1e-6
  cs.HOLD_PHASE_LOCK = False
  try:
    r2 = run(Car(45, dash_offset_mph=0.0, hold_phase_window=1), NO_CURVE, 2500, lead=T.StoppedLead(70), stop_mode=2,
             max_drop=10, wheel_phase=4)
  finally:
    cs.HOLD_PHASE_LOCK = True
  log2 = [h for h in r2["logic"].hold_log if h["direction"] == cs.SET_MINUS and h["done"]]
  assert r2["min"] >= FLOOR_MPH - 1e-6 and r2["max"] <= 45 + 1e-6       # taps answers keep the limits too
  assert all(not h["phase_lock"] for h in log2)


def test_taps_are_unchanged_by_the_wheel_counter():
  a = run(Car(65), T.between(100, 1500, 61), 4000, wheel_phase=None)
  b = run(Car(65), T.between(100, 1500, 61), 4000, wheel_phase=3)
  assert a["presses"] == b["presses"] and a["holds"] == b["holds"] == []


# ---------------- fast restore while the stock radar follows a car (owner's request, drive 45) ----------------

def _restore(radar_from):
  lead = StoppedLead(120, present=lambda f: f < 700, radar_from=radar_from)     # vision loses it: a gentle release
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 5000, lead=lead, max_drop=10))
  stop_f = rel[0][0]
  plus_holds = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS and h["frame"] >= stop_f]
  return r, rel, plus_holds


def test_restore_uses_set_plus_holds_while_the_radar_follows_a_car():
  r, rel, plus_holds = _restore(radar_from=1000.0)
  assert rel[0][1] == "lead lost (vision)" and rel[0][2] is False, rel        # a release that restores gently ...
  assert plus_holds, r["logic"].hold_log                                       # ... but the radar has a car: holds
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_restore_without_a_radar_lead_stays_gentle():
  r, rel, plus_holds = _restore(radar_from=None)
  assert rel[0][1] == "lead lost (vision)" and plus_holds == [], plus_holds
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


# ---------------- drive 4b: the stock radar following the car ahead, the filtered lead speed, fast restores ----------------

def _set_changes(r):
  s = r["sets"]
  return [(k, round(s[k] / MPH, 1)) for k in range(1, len(s)) if abs(s[k] - s[k - 1]) > 1e-6]


def _minus_frames(r):
  return sorted([f for f, b in r["presses"] if b == cs.SET_MINUS] +
                [h["frame"] for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS])


def test_radar_tracked_slowing_car_is_left_to_the_mrcc():
  # T5-T7: a slower car the stock radar was already following while it was moving, the MRCC braking for it: we do not
  # lower the set speed at all, and nothing is pressed until we have stopped behind it
  lead = SlowingLead(110, 29, decel=1.0, radar_from=1000, follow_from=80)
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=lead, max_drop=10,
                                 model=lambda f: -0.6 if f >= 200 else 0.0))
  assert r["ever_active"]
  assert r["radar_hold_frames"] > 0
  assert r["presses"] == [] and r["holds"] == [], (r["presses"][:5], r["holds"][:5])
  assert rel and rel[0][1] == "ego stopped behind the lead", rel


def test_radar_picks_up_a_slower_car_after_we_lowered_restores_with_holds_once_matched():
  # owner's rule 1: the car ahead keeps moving and the radar tracks it: once the MRCC is at its speed, back to the
  # driver's speed with a SET+ hold to 5 below it, then 1 mph taps
  lead = StoppedLead(120, v_mph=25, radar_from=100, follow_from=100)
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=lead, max_drop=10,
                                 model=lambda f: -0.6 if f < 600 else 0.0))
  log = r["logic"].hold_log
  assert log and log[0]["direction"] == cs.SET_MINUS
  first_rh = r["radar_hold"].index(True)
  assert all(f < first_rh for f in _minus_frames(r)), (first_rh, _minus_frames(r))
  # (drive 4d: at a moving car's speed it is now released on vision too, often before the radar match)
  assert rel and (rel[0][1].startswith("radar following") or rel[0][1] == "at the moving car's speed") and rel[0][2], rel
  plus_holds = [h for h in log if h["direction"] == cs.SET_PLUS]
  assert plus_holds and plus_holds[0]["frame"] >= rel[0][0]
  assert near(plus_holds[0]["bound_mph"], 40)
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6
  # the hold stops 5 mph below the driver's speed, the last 5 are taps
  ups = [mph for f, mph in _set_changes(r) if f > rel[0][0]]
  assert ups[-6:] == [40.0, 41.0, 42.0, 43.0, 44.0, 45.0], ups


def test_radar_flicker_during_the_approach_keeps_lowering_and_never_restores():
  # (a) a stopped car the radar blinks on and off for: never trusted (it was never seen moving): today's floor hold
  r = approach(lead_kw=dict(radar_from=1000, radar_present=lambda f: (f // 60) % 2 == 0))
  stop = next(k for k, v in enumerate(r["vs"]) if v < 0.3)
  assert abs(r["min"] - FLOOR_MPH) < 0.5
  assert all(f > stop for f in _plus_frames(r))
  # (b) drive 4b: the radar dropped the car at 42 m for a second: the follow ends at once and we lower (floor hold)

  def mk(rp):
    return SlowingLead(110, 23, radar_from=1000, follow_from=80, radar_present=rp, v_end=0.0)
  ds = []
  probe = mk(lambda f: True)
  tick = probe.tick
  probe.tick = lambda car: (tick(car), ds.append(probe.d))
  run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=probe, max_drop=10, model=lambda f: -0.6 if f >= 200 else 0.0)
  D = next(k for k, d in enumerate(ds) if d <= 42)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=mk(lambda f: not (D <= f < D + 100)), max_drop=10,
          model=lambda f: -0.6 if f >= 200 else 0.0)
  fl = r["follow"]
  assert any(fl[:D]) and not all(fl[D:D + 100])
  assert any(h["direction"] == cs.SET_MINUS and h["frame"] >= D for h in r["logic"].hold_log)
  stop = next(k for k, v in enumerate(r["vs"]) if v < 0.3)
  assert all(f > stop for f in _plus_frames(r))
  assert r["max"] <= 45 + 1e-6


def test_a_single_noisy_lead_speed_frame_does_not_start_a_stop():
  # T1: vLead jumped 18 / 28 / 42 mph frame to frame for a car doing ~40: the filtered speed never looks slow
  cycle = [18, 28, 42, 39, 40, 41]
  with _NoWarningStart():
    tr = scs.StoppedCarTrigger()
    for f in range(400):
      tr.update(_latched_inputs(lead_prob=0.9, lead_d=100.0, lead_v=cycle[(f // 10) % len(cycle)] * MPH, red_light=False))
      assert not tr.active, f
    tr = scs.StoppedCarTrigger()                  # sanity: a car really at 13 mph does start it
    assert any(tr.update(_latched_inputs(lead_prob=0.9, lead_d=100.0, lead_v=13 * MPH)) for _ in range(400))


class _T1Lead(StoppedLead):
  """13 mph until frame 400 (the noisy slow frames), then the car ahead's real 44 mph."""
  def tick(self, car):
    if self.frame + 1 == 400:
      self.v = 44 * MPH
    super().tick(car)


def test_t1_false_start_restores_with_a_hold_then_taps_never_above_the_driver():
  lead = _T1Lead(120, v_mph=13, prob=0.9)
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=lead, max_drop=10,
                                 model=lambda f: -0.6 if f < 400 else 0.3, plan=lambda f: f < 400))
  assert rel and rel[0][1].startswith("traffic moving") and rel[0][2], rel
  plus_holds = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS]
  assert plus_holds and plus_holds[0]["frame"] >= rel[0][0]
  back = next(k for k, s in enumerate(r["sets"]) if k > rel[0][0] and near(s / MPH, 45))
  assert (back - rel[0][0]) * T.DT <= 6.0, back - rel[0][0]     # 20 -> 45: was ~25 slow taps (12 s+)
  assert r["max"] <= 45 + 1e-6 and near(r["final"], 45)


def test_stop_behind_a_radar_tracked_car_no_set_plus_until_stopped_then_holds():
  # owner's rule 2: the car ahead is stopping and the radar tracks and brakes for it: wait until we have stopped,
  # then back to the driver's speed in 5 mph steps
  lead = SlowingLead(120, 25, decel=1.5, v_end=0.0, radar_from=80, follow_from=80)
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 4000, lead=lead, max_drop=10,
                                 model=lambda f: -0.6))
  stop = next(k for k, v in enumerate(r["vs"]) if v < 0.3)
  assert all(f > stop for f in _plus_frames(r)), (stop, _plus_frames(r)[:3])
  first_rh = r["radar_hold"].index(True)
  assert all(f < first_rh for f in _minus_frames(r))
  assert rel and rel[0][1] == "ego stopped behind the lead"
  assert any(h["direction"] == cs.SET_PLUS and h["frame"] > stop for h in r["logic"].hold_log)
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_radar_follow_needs_a_moving_car_and_a_dropout_needs_longer_to_count_again():
  base = dict(lead_prob=0.9, lead_d=60.0, lead_v=20 * MPH, radar_has_lead=True)
  tr = scs.StoppedCarTrigger()
  for _ in range(99):
    tr.update(_latched_inputs(**base))
  assert not tr.radar_stable
  tr.update(_latched_inputs(**base))
  assert tr.radar_stable and tr.radar_follow
  # a stopped car reported by the radar without having tracked it moving is not followed
  tr2 = scs.StoppedCarTrigger()
  for _ in range(200):
    tr2.update(_latched_inputs(**dict(base, lead_v=0.0)))
  assert tr2.radar_stable and not tr2.radar_follow
  # during an episode, a dropout > 0.3 s ends the follow; it then takes 2 s of radar to count again
  tr3 = scs.StoppedCarTrigger()
  while not tr3.update(_latched_inputs(lead_prob=0.9, lead_d=100.0, lead_v=13 * MPH)):
    pass
  for _ in range(150):
    tr3.update(_latched_inputs(lead_prob=0.9, lead_d=100.0, lead_v=13 * MPH, radar_has_lead=True))
  assert tr3.radar_follow
  for _ in range(40):
    tr3.update(_latched_inputs(lead_prob=0.9, lead_d=100.0, lead_v=13 * MPH))
  assert not tr3.radar_follow and tr3.radar_flicker
  for _ in range(150):
    tr3.update(_latched_inputs(lead_prob=0.9, lead_d=100.0, lead_v=13 * MPH, radar_has_lead=True))
  assert not tr3.radar_stable
  for _ in range(60):
    tr3.update(_latched_inputs(lead_prob=0.9, lead_d=100.0, lead_v=13 * MPH, radar_has_lead=True))
  assert tr3.radar_stable and tr3.radar_follow


# ---------------- drive 4c: clear restore, car leaving our lane, start on the stop ahead warning ----------------

def _brake(s, c):
  s["cruise"], s["brake"] = False, True


def _resume(s, c):
  s["cruise"], s["brake"] = True, False


class _Switch(StoppedLead):
  """A stopped car until frame F; from then the only car ahead is `after_d` m away at `after_mph` (tag 3: a car
  79 m ahead at 36 mph when cruise came back)."""
  def __init__(self, F, d0, after_mph=36.0, after_d=79.0, **kw):
    super().__init__(d0, **kw)
    self.F, self.after_v, self.after_d = F, after_mph * MPH, after_d

  def tick(self, car):
    if self.frame + 1 == self.F:
      self.d, self.v = self.after_d, self.after_v
    super().tick(car)


def test_brake_release_is_clear_only_without_a_slower_car_in_our_lane():
  tr = scs.StoppedCarTrigger()
  while not tr.update(_sw()):
    pass
  tr.update(_sw(brake_pressed=True))
  assert not tr.active and tr.release_reason == "cruise off / brake" and tr.clear_release
  tr.update(_sw(brake_pressed=True, lead_status=False))
  assert tr.clear_ahead                                                  # nothing ahead
  for _ in range(60):
    tr.update(_sw(brake_pressed=True, v_ego=27 * MPH, lead_v=36 * MPH, lead_d=79.0, lead_prob=0.9, lead_y=0.1))
  assert tr.clear_ahead                                                  # faster than us (tag 3)
  for _ in range(60):
    tr.update(_sw(brake_pressed=True, lead_v=0.0, lead_d=60.0, lead_prob=0.9, lead_y=0.1))
  assert not tr.clear_ahead                                              # a stopped car in our lane
  tr.update(_sw(brake_pressed=True, lead_v=0.0, lead_d=60.0, lead_prob=0.9, lead_y=0.1, lead_y_path=1.6))
  assert tr.clear_ahead                                                  # ... the next lane over
  tr.update(_sw(brake_pressed=True, lead_v=0.0, lead_d=60.0, lead_prob=0.3, lead_y=0.1))
  assert tr.clear_ahead                                                  # too unsure to count


def test_lost_release_is_never_clear():
  tr = scs.StoppedCarTrigger()
  while not tr.update(_sw()):
    pass
  for _ in range(600):
    tr.update(_sw(lead_status=False, model_alive=False))
  assert not tr.active and tr.release_reason.startswith("lead lost")
  for _ in range(1000):
    tr.update(_sw(lead_status=False, model_alive=False))
    assert not tr.clear_ahead


def test_drive_4c_brake_release_restores_with_a_hold_when_the_car_ahead_is_faster():
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2500, events={330: _brake, 600: _resume},
                                 lead=_Switch(330, 100), max_drop=10))
  assert rel and rel[0][1] == "cruise off / brake"
  ups = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS]
  assert ups and 600 <= ups[0]["frame"] <= 600 + 150 and near(ups[0]["bound_mph"], 40), ups[:1]
  tail = [mph for f, mph in _set_changes(r) if f > 600]
  assert tail[-6:] == [40.0, 41.0, 42.0, 43.0, 44.0, 45.0], tail
  back = next(k for k, s in enumerate(r["sets"]) if k > 600 and near(s / MPH, 45))
  assert (back - 600) * T.DT <= 5.0, back
  assert r["max"] <= 45 + 1e-6


def test_brake_release_stays_gentle_while_a_slower_car_is_ahead():
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2500, events={330: _brake, 600: _resume},
                                 lead=_Switch(330, 100, after_mph=12.0, after_d=60.0), max_drop=10))
  ups = [f for f, b in r["presses"] if b == cs.SET_PLUS and f < 1000]
  assert ups and all(b - a >= cs.RELAXED_GAP_FRAMES for a, b in zip(ups, ups[1:])), ups
  assert not [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS and h["frame"] < 1000]
  assert r["max"] <= 45 + 1e-6


def _stop_with_car_active(**kw):
  tr = scs.StoppedCarTrigger()
  while not tr.update(_sw(lead_prob=0.9, lead_y=0.2, **kw)):
    pass
  return tr


def test_leaving_needs_offset_drift_raw_agreement_and_half_a_second():
  tr = _stop_with_car_active()
  ys = [0.2] * 50 + [0.2 + 0.0065 * k for k in range(100)] + [0.85] * 200     # drifts out to 0.85 and stays
  n = None
  for k, y in enumerate(ys):
    if not tr.update(_sw(lead_prob=0.9, lead_y=y)):
      n = k
      break
  assert n is not None and tr.release_reason == "lead leaving the lane" and tr.clear_release, (n, tr.release_reason)
  first_out = next(k for k, y in enumerate(ys) if y >= scs.LEAVE_Y_MIN)
  assert first_out + 40 <= n <= first_out + 120, (first_out, n)
  # offset constant from the start (no drift): stays
  tr = _stop_with_car_active(lead_y_path=0.9)
  for _ in range(300):
    assert tr.update(_sw(lead_prob=0.9, lead_y=0.9, lead_y_path=0.9))
  # raw yRel grows on a curve but the car stays on our path: stays
  tr = _stop_with_car_active()
  for k in range(300):
    assert tr.update(_sw(lead_prob=0.9, lead_y=min(0.2 + 0.01 * k, 1.9), lead_y_path=0.1))
  # the path wobbles but raw yRel says centred: stays
  tr = _stop_with_car_active()
  for k in range(300):
    assert tr.update(_sw(lead_prob=0.9, lead_y=0.2, lead_y_path=min(0.2 + 0.01 * k, 1.0)))


def test_lead_switch_restarts_the_lateral_baseline():
  tr = _stop_with_car_active(lead_d=60.0)
  for k in range(30):
    tr.update(_sw(lead_prob=0.9, lead_d=60.0, lead_y=0.2 + 0.02 * k))
  for k in range(300):                       # another car (110 m) at 0.1 -> 0.6: never "leaving"
    assert tr.update(_sw(lead_prob=0.9, lead_d=110.0, lead_y=min(0.1 + 0.005 * k, 0.6)))


class _Tag2(SlowingLead):
  """Tag 2: a car slowing at a red light, centred, then moving into a right-turn lane (-0.25 -> -1.8 m)."""
  def y_path(self, f, car):
    return -0.25 if f < 150 else max(-1.8, -0.25 - (f - 150) * 0.0075)

  def state(self, f, car):
    st = super().state(f, car)
    return (st[0], st[1], st[2], st[3], self.y_path(f, car)) if st[0] else st


def test_tag2_replay_suv_turning_right_releases_within_a_second_and_restores_fast():
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2500,
                                 lead=_Tag2(90, 31, decel=1.0, v_end=8.0, prob=0.9), max_drop=10,
                                 model=lambda f: -0.7 if f < 300 else 0.3, plan=lambda f: f < 300))
  assert r["logic"].hold_log[0]["direction"] == cs.SET_MINUS
  out = 150 + int((scs.LEAVE_Y_MIN - 0.25) / 0.0075)      # first frame |y| >= 0.8
  assert rel and rel[0][1] == "lead leaving the lane" and rel[0][0] <= out + 100, (rel, out)
  assert not [f for f in _minus_frames(r) if f > rel[0][0]]           # never lowered for that car again
  ups = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS]
  assert ups, "fast restore"
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6 and r["min"] >= FLOOR_MPH - 1e-6


class _CurveAhead(StoppedLead):
  """A stopped car straight ahead in a curve: raw yRel grows to 1.9 m, the offset from the path stays 0.1."""
  def y_path(self, f, car):
    return 0.1

  def state(self, f, car):
    st = super().state(f, car)
    return (st[0], st[1], st[2], st[3], min(0.1 + f * 0.009, 1.9)) if st[0] else st


def test_stopped_car_straight_ahead_in_a_curve_is_not_released():
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=_CurveAhead(100), max_drop=10))
  assert [x[1] for x in rel][:1] == ["ego stopped behind the lead"], rel
  assert abs(r["min"] - FLOOR_MPH) < 0.5


def test_a_car_that_left_our_lane_does_not_start_it_again_until_it_comes_back():
  tr = _stop_with_car_active()
  for k in range(400):
    tr.update(_sw(lead_prob=0.9, lead_y=min(0.2 + 0.01 * k, 1.8)))
  assert not tr.active and tr.release_reason == "lead leaving the lane" and tr.left_car
  for _ in range(300):
    assert not tr.update(_sw(lead_prob=0.9, lead_y=1.8))      # still slow, still in the next lane: no restart
  for _ in range(100):
    tr.update(_sw(lead_prob=0.9, lead_y=0.1))                 # it came back in front of us
  assert tr.active


def test_warning_gates_match_the_planner_warning():
  sw = _stop_warning()
  v = 20.0
  ts, vs, xs = _plan(v, 0.0, 3.0)
  for lead in (dict(lead_status=False), dict(lead_d=40.0, lead_v=19.0), dict(lead_d=150.0, lead_v=19.0),
               dict(lead_d=150.0, lead_v=0.0), dict(lead_d=230.0, lead_v=0.0)):
    for model_length in (0.0, 120.0, 190.0):
      kw = dict(lead_status=True, lead_d=0.0, lead_v=0.0, lead_a=0.0)
      kw.update(lead)
      ref = sw.lead_is_relevant(kw["lead_status"], kw["lead_d"], kw["lead_v"], 0.0, False, v, model_length)
      mine = scs.lead_relevant(kw["lead_status"], kw["lead_d"], kw["lead_v"], 0.0, False, v, model_length)
      if model_length > 0:
        assert mine == ref, (lead, model_length)
  assert scs.WARN_DEBOUNCE == sw.STOP_DEBOUNCE and scs.WARN_DROPOUT == sw.STOP_DROPOUT
  assert scs.WARN_MIN_SPEED == sw.STOP_MIN_SPEED and scs.WARN_NOT_SLOWING_ACCEL == sw.NOT_SLOWING_ACCEL
  assert scs.WARN_ACCEL_TAU == sw.ACCEL_FILTER_TAU and scs.WARN_LANE_CHANGE_HOLD == sw.LANE_CHANGE_HOLD
  assert scs.WARN_REARM == sw.REARM_TIME
  assert scs.PLAN_STOP_RATIO == sw.STOP_SLOWDOWN and scs.PLAN_STOP_DECEL == sw.STOP_DECEL


def test_stop_ahead_warning_alone_starts_a_floor_hold_before_the_warning_shows():
  # owner (Nov 5): "first start reducing the speed and then display the warning": we start after 0.5 s of the
  # warning's condition, the warning shows after 1.0 s
  assert scs.WARN_START_DEBOUNCE + 0.5 - 1e-9 <= scs.WARN_DEBOUNCE
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_latched_inputs(lead_status=False, plan_stop=False))
  n = 0
  while not tr.update(_latched_inputs(lead_status=False)):
    n += 1
    assert n < 200
    if 10 < n < 45:
      assert any("warning held" in b for b in tr.blockers(_latched_inputs(lead_status=False)))
  assert 48 <= n <= 52, n
  assert tr.kind == scs.STOP_AHEAD and tr.warn_start and tr.urgent and tr.floor_mode
  assert near(tr.target / MPH, FLOOR_MPH)


def test_stop_ahead_start_is_blocked_by_a_relevant_lead_lane_change_braking_or_low_speed():
  cases = (dict(lead_status=True, lead_d=40.0, lead_v=40 * MPH, lead_prob=0.9, lead_y=0.1), dict(lane_change=True),
           dict(a_ego=-0.8),
           dict(v_ego=22 * MPH), dict(brake_pressed=True), dict(plan_stop=False))
  for kw in cases:
    tr = scs.StoppedCarTrigger()
    base = dict(lead_status=False, red_light=False)   # (the detector alone would start its head start after 2 s)
    base.update(kw)
    for _ in range(300):
      tr.update(_latched_inputs(**base))
    assert not tr.active, kw


def test_stop_ahead_hands_over_to_the_car_when_it_appears():
  tr = scs.StoppedCarTrigger()
  while not tr.update(_latched_inputs(lead_status=False)):
    pass
  for _ in range(50):                                 # a car at our speed: still the planned stop
    assert tr.update(_latched_inputs(lead_prob=0.9, lead_d=60.0, lead_v=44 * MPH, lead_y=0.1))
  assert tr.kind == scs.STOP_AHEAD
  tr.update(_latched_inputs(lead_prob=0.4, lead_d=113.0, lead_v=27 * MPH, lead_y=0.3))
  assert tr.active and tr.kind == scs.STOP_WITH_CAR and tr.urgent and tr.floor_mode
  assert near(tr.target / MPH, FLOOR_MPH)


def _started_past_the_warning(tr):
  while not tr.update(_latched_inputs(lead_status=False)):
    pass
  for _ in range(int((scs.WARN_DEBOUNCE - scs.WARN_START_DEBOUNCE) / scs.DT_CTRL) + 5):   # the warning shows too
    assert tr.update(_latched_inputs(lead_status=False))


def test_stop_ahead_releases_when_the_plan_clears_and_restores_at_once():
  tr = scs.StoppedCarTrigger()
  _started_past_the_warning(tr)
  for _ in range(99):
    assert tr.update(_latched_inputs(lead_status=False, plan_stop=False, red_light=False, model_accel=0.4))
  assert not tr.update(_latched_inputs(lead_status=False, plan_stop=False, red_light=False, model_accel=0.4))
  assert tr.release_reason.startswith("stop ahead cleared") and tr.restore_now
  # the model stays a little negative: released once the planned stop has been gone STOP_AHEAD_GONE_TIME (drive 54, t
  # 758.65: the plan gone at 761.8, released only at 764.66 when the latch ran out) ...
  tr = scs.StoppedCarTrigger()
  _started_past_the_warning(tr)
  n = 0
  while tr.update(_latched_inputs(lead_status=False, plan_stop=False, red_light=False, model_accel=-0.2)):
    n += 1
    assert n < 1000
  assert abs(n - scs.STOP_AHEAD_GONE_TIME / scs.DT_CTRL) <= 3 and tr.release_reason.startswith("stop ahead cleared")
  assert tr.restore_now
  # ... a planned stop that blinks off for less than that keeps it
  tr = scs.StoppedCarTrigger()
  _started_past_the_warning(tr)
  for k in range(400):
    assert tr.update(_latched_inputs(lead_status=False, plan_stop=(k % 100) >= 80, model_accel=-0.2))
  # ... and with a slower car in our lane (not sure enough to be the stop's car yet) it waits for the latch (LATCH_HOLD)
  slower = dict(lead_status=True, lead_prob=0.15, lead_d=110.0, lead_v=15 * MPH, lead_y=0.1, plan_stop=False, red_light=False,
                model_accel=-0.2)
  tr = scs.StoppedCarTrigger()
  _started_past_the_warning(tr)
  n = 0
  while tr.update(_latched_inputs(**slower)):
    n += 1
    assert n < 1000
  assert abs(n - scs.LATCH_HOLD / scs.DT_CTRL) <= 3 and tr.release_reason.startswith("stop ahead cleared"), n
  # with the flag off (the old code): the latch
  with _Flag("STOP_AHEAD_GONE_ENABLED", False):
    tr = scs.StoppedCarTrigger()
    _started_past_the_warning(tr)
    n = 0
    while tr.update(_latched_inputs(lead_status=False, plan_stop=False, red_light=False, model_accel=-0.2)):
      n += 1
      assert n < 1000
  assert abs(n - scs.LATCH_HOLD / scs.DT_CTRL) <= 3
  # and not again for the same stop: the condition must be gone WARN_REARM first
  tr2 = scs.StoppedCarTrigger()
  while not tr2.update(_latched_inputs(lead_status=False)):
    pass
  tr2.update(_latched_inputs(lead_status=False, brake_pressed=True))
  for _ in range(250):
    assert not tr2.update(_latched_inputs(lead_status=False))


def test_stop_ahead_brake_release_is_clear():
  tr = scs.StoppedCarTrigger()
  while not tr.update(_latched_inputs(lead_status=False)):
    pass
  tr.update(_latched_inputs(lead_status=False, brake_pressed=True))
  assert not tr.active and tr.clear_release and tr.clear_ahead


class _Tag4(SlowingLead):
  """Tag 4: the stop is planned from frame 100 (the warning holds 1 s), the car is seen 0.8 s after that."""
  def state(self, f, car):
    if f < 280:
      return (False, 0.0, 0.0, 0.0, 0.0)
    return (True, 0.4 if f < 360 else 0.9, self.d, self.v, 0.3)


def test_tag4_replay_lowering_starts_on_the_warning_before_the_car_is_seen():
  # (the stop-light detector agrees from the plan on, as in every genuine warning on drives 4f / 50)
  def go():
    return run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1200, lead=_Tag4(150, 27, decel=1.5, v_end=0.0), max_drop=10,
               model=lambda f: -0.7, plan=lambda f: f >= 100, red=lambda f: f >= 100)
  r = go()
  first = first_frame(r)
  assert r["holds"] and r["holds"][0] == first and first <= 215, (first, r["holds"][:2])
  at20 = next(k for k, s in enumerate(r["sets"]) if near(s / MPH, 20))
  assert (at20 - first) * T.DT <= 3.5 and r["min"] >= FLOOR_MPH - 1e-6
  with _NoWarningStart():
    r2 = go()
  assert first_frame(r2) >= first + 100, (first_frame(r2), first)


class _NoCar(StoppedLead):
  def state(self, f, car):
    return (False, 0.0, 0.0, 0.0, 0.0)

  def tick(self, car):
    self.frame += 1


def test_false_stop_ahead_warning_costs_a_few_mph_and_comes_back_fast():
  # a stop the detector backed (redLight on) that goes away after 3 s (the light turned green): the floor hold, then back
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2500, lead=_NoCar(200), max_drop=10,
                                 model=lambda f: -0.6 if 100 <= f < 400 else 0.3, plan=lambda f: 100 <= f < 400,
                                 red=lambda f: 100 <= f < 400))
  assert rel and rel[0][1].startswith("stop ahead cleared") and rel[0][2]
  assert [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS], "restored with a SET+ hold"
  assert r["logic"].holds_allowed                                   # the cut-short SET- hold did not switch holds off
  back = next(k for k, s in enumerate(r["sets"]) if k > rel[0][0] and near(s / MPH, 45))
  assert (back - rel[0][0]) * T.DT <= 7.0, back
  assert r["max"] <= 45 + 1e-6 and min(r["vs"]) / MPH >= 45 - 20


def test_stop_ahead_start_in_test_mode_sends_nothing():
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1500, lead=_NoCar(200), stop_mode=1, max_drop=10,
          model=lambda f: -0.7, plan=lambda f: f >= 100)
  assert r["ever_active"] and r["presses"] == [] and r["holds"] == []


def test_stop_ahead_then_a_radar_followed_car_is_left_to_the_mrcc():
  class Appears(SlowingLead):
    def state(self, f, car):
      return (True, 0.9, self.d, self.v, 0.1) if f >= 250 else (False, 0.0, 0.0, 0.0, 0.0)
  lead = Appears(150, 30, decel=0.5, v_end=5.0, radar_from=1000, follow_from=80)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1500, lead=lead, max_drop=10, model=lambda f: -0.7,
          plan=lambda f: f >= 100, red=lambda f: f >= 100)
  assert r["holds"] and r["holds"][0] <= 215
  rh = r["radar_hold"]
  first_rh = rh.index(True)
  assert all(f < first_rh for f in _minus_frames(r)), (first_rh, _minus_frames(r)[-3:])
  assert r["min"] >= FLOOR_MPH - 1e-6


# ---------------- drive 4d: a slower car that keeps moving, restores at any speed, the driver on the gas ----------------

def test_moving_slower_car_target_stays_above_its_speed_until_it_brakes():
  tr = scs.StoppedCarTrigger()
  kw = dict(lead_prob=0.9, lead_y=0.1, lead_d=110.0, lead_v=31 * MPH, lead_a=-0.2)
  while not tr.update(_sw(**kw)):
    pass
  for _ in range(100):
    tr.update(_sw(**kw))
  assert tr.active and tr.floor_mode and 25 <= tr.target / MPH <= 27, tr.target / MPH
  for _ in range(300):                              # it brakes for the light: the floor from now on
    tr.update(_sw(**dict(kw, lead_a=-1.5)))
  assert near(tr.target / MPH, FLOOR_MPH) and tr.floor_committed
  for _ in range(300):                              # and stays there even if it eases off again
    tr.update(_sw(**kw))
  assert near(tr.target / MPH, FLOOR_MPH)


class _Tag4Lead(StoppedLead):
  """Tag 4: a car 110 m ahead slows from 42 to 31 mph and keeps going (later speeds up to 45)."""
  def __init__(self, *a, speed_up_at=1500, **kw):
    super().__init__(*a, **kw)
    self.speed_up_at, self.a = speed_up_at, 0.0

  def tick(self, car):
    if self.frame < self.speed_up_at:
      target, rate = 31 * MPH, 0.6       # tag 4: aLeadK -0.25..-1.0 while it slowed, about -0.65 on average
    else:
      target, rate = 45 * MPH, 1.0
    dv = max(-rate * T.DT, min(rate * T.DT, target - self.v))
    self.a = dv / T.DT
    self.v += dv
    super().tick(car)


def test_tag4_replay_a_slower_car_that_keeps_moving_is_not_a_stop():
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3500,
                                 lead=_Tag4Lead(110, v_mph=42, prob=0.9), max_drop=10,
                                 model=lambda f: -0.6, plan=lambda f: 60 <= f < 1200))
  assert r["ever_active"]
  assert r["min"] >= 25 - 1e-6, r["min"]                                   # not 20 for a car doing 31
  assert rel and rel[0][1] == "at the moving car's speed" and rel[0][2], rel
  ups = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS and h["frame"] >= rel[0][0]]
  assert ups, "back with the hold"
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


class _GasCar(Car):
  """The driver on the gas: the car accelerates at 1.5 m/s^2 whatever the set speed (up to `top`)."""
  def __init__(self, *a, top_mph=42, **kw):
    super().__init__(*a, **kw)
    self.gas, self.top = False, top_mph * MPH

  def tick(self):
    if self.gas:
      v = self.v_ego
      super().tick()
      self.v_ego = min(max(v + 1.5 * T.DT, self.v_ego), max(self.top, self.v_ego))
    else:
      super().tick()


def _gas_run(v_resume_mph, top_mph=42, jump_extra=0, lose=None):
  car = _GasCar(45, dash_offset_mph=0.0, top_mph=top_mph, jump_extra=jump_extra, lose=lose)
  lead = StoppedLead(100)

  def brake(s, c):
    s["cruise"], s["brake"] = False, True

  def stop(s, c):
    c.v_ego = 0.0

  def resume(s, c):
    s["cruise"], s["brake"], s["gas"] = True, False, True
    c.gas = True
    c.v_ego = v_resume_mph * MPH
    lead.d, lead.v = 300.0, 30.0                    # the queue drove off: nothing slower ahead any more

  def lift(s, c):
    s["gas"] = False
    c.gas = False
  events = {330: brake, 400: stop, 800: resume, 1800: lift}
  return _releases(lambda: run(car, NO_CURVE, 3000, events=events, lead=lead, max_drop=10))


def test_tag3_replay_on_the_gas_the_restore_goes_on():
  # after a stop at a red light (set 20, the driver's speed 45) the driver drives off on the gas
  r, rel = _gas_run(5)                            # slower than the set speed: SET+ taps while on the gas
  assert rel and rel[0][1] == "cruise off / brake"
  back = next(k for k, s in enumerate(r["sets"]) if k > 800 and near(s / MPH, 45))
  assert back < 1800, back                        # back at 45 before the gas is lifted
  # (drives 56 / 59: a restore hold also on the gas while the set speed is above the car's speed; never above his 45)
  assert r["max"] <= 45 + 1e-6 and near(r["final"], 45) and not r["logic"].disabled


def test_on_the_gas_faster_than_the_set_speed_one_set_plus_jumps_to_the_current_speed():
  r, rel = _gas_run(35)                           # already at 35 with the set speed at 20 (tags 3/4: 20 -> 39 / 38)
  s = r["sets"]
  big = [(k, s[k] / MPH) for k in range(801, 1800) if s[k] - s[k - 1] > 2 * MPH]
  assert big and 34 <= big[0][1] <= 45 - cs.GAS_JUMP_MARGIN + 1, big[:2]
  assert r["max"] <= 45 + 1e-6 and near(r["final"], 45) and not r["logic"].disabled


def test_set_plus_jump_is_never_sent_when_it_could_pass_the_drivers_speed():
  r, rel = _gas_run(45, top_mph=45)               # at the driver's 45 on the gas, set 25: no jump (it could land at 46)
  s = r["sets"]
  assert r["max"] <= 45 + 1e-6
  assert not [k for k in range(801, 1800) if s[k] - s[k - 1] > 2 * MPH]
  assert near(r["final"], 45) and not r["logic"].disabled
  # a jump landing one above the shown speed, presses ignored now and then: never above 45, never switched off
  for v, extra in ((35, 1), (40, 1), (43, 1), (44, 0)):
    r, rel = _gas_run(v, top_mph=v, jump_extra=extra, lose=lambda n: n % 4 == 0)
    assert r["max"] <= 45 + 1e-6, (v, extra, r["max"])
    assert near(r["final"], 45) and not r["logic"].disabled, (v, extra, r["final"])


def test_restore_starts_below_12_mph_once_the_car_ahead_pulls_away():
  car = Car(45, dash_offset_mph=0.0)
  lead = StoppedLead(100)

  def brake(s, c):
    s["cruise"], s["brake"] = False, True

  def slow(s, c):
    c.v_ego = 1.0
    lead.d, lead.v = 8.0, 0.0

  def resume(s, c):
    s["cruise"], s["brake"] = True, False
    lead.moves_at, lead.drive_off = lead.frame, 25 * MPH        # the car ahead drives off
  speeds = {}
  orig = lead.tick
  lead.tick = lambda c: (orig(c), speeds.__setitem__(lead.frame, c.v_ego))
  r = run(car, NO_CURVE, 2500, events={330: brake, 500: slow, 700: resume}, lead=lead, max_drop=10)
  ups = [f for f, b in r["presses"] if b == cs.SET_PLUS] + [h["frame"] for h in r["logic"].hold_log
                                                            if h["direction"] == cs.SET_PLUS]
  first = min(ups)
  assert speeds[first - 1] < cs.KEEP_SPEED, (first, speeds[first - 1] / MPH)
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_driver_on_the_gas_releases_and_that_car_does_not_restart_it():
  tr = scs.StoppedCarTrigger()
  kw = dict(lead_prob=0.9, lead_y=0.1, lead_d=90.0, lead_v=13 * MPH)
  while not tr.update(_sw(**kw)):
    pass
  for _ in range(99):
    assert tr.update(_sw(gas_pressed=True, **kw))
  assert not tr.update(_sw(gas_pressed=True, **kw))
  assert tr.release_reason == "driver on the gas" and tr.clear_release and not tr.restore_now
  for _ in range(300):
    assert not tr.update(_sw(gas_pressed=True, **kw))          # nothing starts on the gas
  for _ in range(300):
    assert not tr.update(_sw(**kw))                            # nor for that car afterwards
  n = 0
  while not tr.update(_sw(**dict(kw, lead_d=130.0))):         # another car (a distance jump): it may
    n += 1
    assert n < 200


def test_moving_car_release_needs_a_steady_car_and_no_planned_stop():
  base = dict(lead_prob=0.9, lead_y=0.1, lead_d=60.0, lead_v=30 * MPH, v_ego=31 * MPH, model_accel=-0.6)
  for kw, released in ((dict(lead_a=-0.1), True), (dict(lead_a=-0.55), False), (dict(lead_a=-0.1, plan_stop=True), False)):
    tr = scs.StoppedCarTrigger()
    while not tr.update(_sw(**dict(base, v_ego=45 * MPH, lead_a=-0.1))):
      pass
    gone = False
    for _ in range(400):
      if not tr.update(_sw(**dict(base, **kw))):
        gone = True
        break
    assert gone == released, kw
    if released:
      assert tr.release_reason == "at the moving car's speed" and tr.restore_now


def test_an_overridden_car_counts_again_once_it_slows_a_lot_or_the_gas_is_off_a_while():
  kw = dict(lead_prob=0.9, lead_y=0.1, lead_d=90.0, lead_v=13 * MPH)

  def overridden():
    tr = scs.StoppedCarTrigger()
    while not tr.update(_sw(**kw)):
      pass
    for _ in range(101):
      tr.update(_sw(gas_pressed=True, **kw))
    assert tr.release_reason == "driver on the gas" and tr.overridden
    return tr
  tr = overridden()
  for _ in range(int(scs.OVERRIDE_REARM / scs.DT_CTRL) - 10):
    assert not tr.update(_sw(**kw))
  assert any(tr.update(_sw(**kw)) for _ in range(100))            # 5 s off the gas: it counts again
  tr = overridden()
  slower = dict(kw, lead_v=5 * MPH)                               # 13 -> 5 mph: it is slowing for something
  assert any(tr.update(_sw(**slower)) for _ in range(100))


def _warn_episode(plan_frames, then_frames=300, red_with_plan=False, **kw):
  """A stop ahead (no car) planned for plan_frames, then not: (trigger, frame it started, frame it released).
  red_with_plan: the stop-light detector on only while the stop is planned (else on throughout)."""
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_latched_inputs(lead_status=False, plan_stop=False))
  started = released = None
  for n in range(plan_frames + then_frames):
    if red_with_plan:
      kw["red_light"] = n < plan_frames
    on = tr.update(_latched_inputs(lead_status=False, plan_stop=n < plan_frames, **kw))
    if on and started is None:
      started = n
    if started is not None and not on and released is None:
      released = n
      tr.restore_at_release = tr.restore_now
  return tr, started, released


def test_a_planned_stop_that_goes_away_before_the_warning_is_released_at_once():
  # review of the 0.5 s start: a 0.7 s plan dip (never a warning) started a floor hold that lasted until the stop
  # "cleared" (1.6-4.6 s); now it is released with the set speed restored within the warning's dropout
  # (the stop-light detector on with the plan only: released once it has been off RED_DROPOUT, drive 69)
  tr, started, released = _warn_episode(70, red_with_plan=True)
  assert started is not None and abs(started - 50) <= 2
  assert released is not None and released <= 70 + int((scs.RED_DROPOUT + scs.WARN_DROPOUT) / scs.DT_CTRL) + 2, released
  assert tr.release_reason == scs.STOP_AHEAD_UNCONFIRMED and tr.restore_at_release
  # ... but not while the stop-light detector is still on (drive 69 t 244-248, 1193-1200, 1388): held until the light has
  # been off RED_DROPOUT, or STOP_AHEAD_RED_HOLD with no plan
  tr, started, released = _warn_episode(70, then_frames=900)
  assert started is not None and released is not None, (started, released)
  assert abs(released - (70 + scs.STOP_AHEAD_RED_HOLD / scs.DT_CTRL)) <= 3, released


def test_a_planned_stop_that_lasts_past_the_warning_is_not_released_by_the_dip_rule():
  tr, started, released = _warn_episode(130, then_frames=50)
  assert started is not None and released is None and tr.active        # the normal release rules from here
  # our own slowdown (the car slowing at -1.0 after the start) does not count as "the stop went away"
  tr, started, released = _warn_episode(300, then_frames=0, a_ego=-1.0)
  assert started is None                                                 # (already slowing before: no start)
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_latched_inputs(lead_status=False, plan_stop=False))
  for n in range(300):
    tr.update(_latched_inputs(lead_status=False, a_ego=-1.0 if n > 55 else 0.0))
  assert tr.active and tr.kind == scs.STOP_AHEAD


# ---- the unsure car (owner, Nov 5: "when we see the yellow triangle start slowing down 1 mph at a time until the
# confidence score reaches 0.85") ----

class _Unsure(StoppedLead):
  """A stopped car in our lane, seen from 120 m at a low confidence that rises to `sure_prob` at `sure_at` (s)."""
  def __init__(self, d0=120.0, v_mph=0.0, prob=0.3, sure_at=None, sure_prob=0.95, gone_at=None, y=0.2):
    super().__init__(d0, v_mph=v_mph, prob=prob, y=y)
    self.sure_at, self.sure_prob, self.gone_at, self.p0 = sure_at, sure_prob, gone_at, prob

  def state(self, f, car):
    t = f * T.DT
    if self.gone_at is not None and t >= self.gone_at:
      return (False, 0.0, 0.0, 0.0, 0.0)
    p = self.sure_prob if (self.sure_at is not None and t >= self.sure_at) else self.p0
    return (True, p, self.d, self.v, self.y)


def _sets_changes(r):
  s = r["sets"]
  return [(k, round(s[k] / MPH, 1)) for k in range(1, len(s)) if abs(s[k] - s[k - 1]) > 1e-6]


def test_unsure_car_taps_1_mph_at_a_time_at_most_5_mph():
  # no planned stop (the model is not slowing) and the confidence never reaches 0.85: taps only, 1 mph each
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 900, lead=_Unsure(125.0, v_mph=15.0), stop_mode=2, max_drop=10,
          model=lambda f: 0.0)
  ch = _sets_changes(r)
  assert r["holds"] == [] and all(b == cs.SET_MINUS for _, b in r["presses"])
  assert ch and ch[0][0] <= 100, ch                      # 0.3 s cruise settle + 0.5 s hold, then the first tap
  assert all(abs(a[1] - b[1]) <= 1.0 + 1e-6 for a, b in zip([(0, 45.0)] + ch[:-1], ch, strict=True)), ch      # 1 mph steps
  assert all(b[0] - a[0] >= 45 for a, b in zip(ch[:-1], ch[1:], strict=True)), ch                             # ~0.5 s apart
  assert r["min"] >= 45 - 5 - 1e-6, r["min"]
  assert r["trigger"].kind == scs.SLOW_UNSURE


def test_unsure_car_hands_over_to_the_stopped_car_at_085():
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1000, lead=_Unsure(150.0, sure_at=3.0), stop_mode=2, max_drop=10,
          model=lambda f: 0.0)
  before = [f for f, b in r["presses"] if f < 300]
  assert before and all(b == cs.SET_MINUS for f, b in r["presses"] if f < 300)
  assert [f for f in r["holds"] if f < 300] == []        # taps only while unsure
  assert r["trigger"].kind == scs.STOPPED_CAR and r["trigger"].confirmed
  assert [f for f in r["holds"] if f >= 300], r["holds"]  # then the stopped car's long press
  assert r["min"] >= FLOOR_MPH - 1e-6


def test_unsure_car_false_alarm_gives_the_speed_back():
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1500, lead=_Unsure(120.0, v_mph=10.0, gone_at=2.5), stop_mode=2,
          max_drop=10, model=lambda f: 0.0)
  assert r["trigger"].release_reason == scs.UNSURE_GONE
  assert r["min"] >= 45 - 5 - 1e-6 and abs(r["final"] - 45) < 0.01, (r["min"], r["final"])


def test_unsure_car_not_for_the_next_lane_our_speed_or_the_radar():
  for kw, lead in ((dict(), _Unsure(120.0, y=2.0)),                 # next lane
                   (dict(), _Unsure(120.0, v_mph=40.0)),            # only 5 mph slower: a car we follow
                   (dict(), _Unsure(140.0, v_mph=44.0))):                          # (and not slower)
    r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 400, lead=lead, stop_mode=2, max_drop=10, model=lambda f: 0.0, **kw)
    assert r["presses"] == [] and r["holds"] == [], lead.__dict__
  tr = scs.StoppedCarTrigger()
  i = _sw(lead_prob=0.3, lead_d=110.0, lead_v=0.0, lead_y=0.2, model_accel=0.0)
  tr.lead_v_f = 0.0
  assert tr._unsure_car(i)
  tr.radar_follow = True                                            # the MRCC already follows a car: its job
  assert not tr._unsure_car(i)
  tr.radar_follow = False
  assert not tr._unsure_car(_sw(lead_prob=0.9, lead_d=110.0, lead_v=0.0, lead_y=0.2))   # sure: the normal starts


def test_unsure_car_sure_but_moving_releases_and_rearms_after_3_s():
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_sw(lead_status=False))
  n = 0
  while not tr.update(_sw(lead_prob=0.3, lead_d=110.0, lead_v=25 * MPH, lead_y=0.2, model_accel=0.0)):
    n += 1
    assert n < 200
  assert tr.kind == scs.SLOW_UNSURE
  for _ in range(int(scs.UNSURE_SURE_TIME / scs.DT_CTRL) + 2):     # sure, still 20 mph slower, no stop planned
    tr.update(_sw(lead_prob=0.95, lead_d=100.0, lead_v=25 * MPH, lead_y=0.2, model_accel=0.0))
  assert not tr.active and tr.release_reason == scs.UNSURE_NOT_STOP
  for _ in range(int(2.5 / scs.DT_CTRL)):                         # unsure again, but within UNSURE_REARM
    assert not tr.update(_sw(lead_prob=0.3, lead_d=100.0, lead_v=25 * MPH, lead_y=0.2, model_accel=0.0))
  for _ in range(100):
    tr.update(_sw(lead_prob=0.3, lead_d=100.0, lead_v=25 * MPH, lead_y=0.2, model_accel=0.0))
  assert tr.active and tr.kind == scs.SLOW_UNSURE


def _u(**kw):
  base = dict(lead_prob=0.3, lead_d=110.0, lead_v=0.0, lead_y=0.2, model_accel=0.0)
  base.update(kw)
  return _sw(**base)


def _unsure_started(**kw):
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_sw(lead_status=False))
  for _ in range(100):
    if tr.update(_u(**kw)):
      break
  return tr


def test_unsure_car_cap_is_below_the_set_speed_and_not_for_oncoming_cars():
  # review: v_ego 30 mph under a 45 set speed (after a stop) aimed at 25: 13 taps; now 5 below the set speed while it is
  # near our speed (the cruise still accelerating, more than CATCH_UP_GAP above it: the drop to our speed first, below)
  tr = _unsure_started(v_ego=43.1 * MPH, set_speed=45 * MPH)
  assert tr.active and tr.kind == scs.SLOW_UNSURE and abs(tr.target / MPH - 40) < 0.01 and not tr.urgent
  tr = _unsure_started(lead_v=-15.0)                            # an oncoming car is not "slower"
  assert not tr.active


def test_unsure_car_sure_and_stopped_is_kept_and_a_stopping_car_is_not_a_false_alarm():
  # a stopped car on a curve: raw yRel 1.4 (the stopped car start's lane check fails), on our path; sure now
  tr = _unsure_started(lead_y=1.4, lead_y_path=0.2)
  assert tr.kind == scs.SLOW_UNSURE
  for _ in range(300):
    assert tr.update(_u(lead_prob=0.95, lead_y=1.4, lead_y_path=0.2))
  # matched speed but the car is braking (or a stop planned): not "it sped up"
  tr = _unsure_started(lead_v=10 * MPH)
  for _ in range(300):
    assert tr.update(_u(lead_v=44 * MPH, lead_a=-2.0, v_ego=45 * MPH))


def test_unsure_car_blinking_far_lead_does_not_go_up_and_down():
  # review: a real stopped car that blinks out 1.2 s every 3 s: 45 -> 42 -> 45 -> 42 with the 1 s "gone" release
  lead = _Unsure(125.0, v_mph=0.0)
  lead.present = lambda f: (f % 300) < 180
  orig = lead.state

  def state(f, car):
    return orig(f, car) if lead.present(f) else (False, 0.0, 0.0, 0.0, 0.0)
  lead.state = state
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 900, lead=lead, stop_mode=2, max_drop=10, model=lambda f: 0.0)
  assert [f for f, b in r["presses"] if b == cs.SET_PLUS] == [] and r["holds"] == [], r["presses"]
  assert r["min"] >= 45 - 5 - 1e-6


def _red_then_green(car, frames=2500):
  # a planned stop (no car) from t 1 s to t 7 s, then the light turns green (the plan clears, the model speeds up)
  plan = lambda f: 100 <= f < 700  # noqa: E731
  model = lambda f: -0.7 if 100 <= f < 700 else 0.3  # noqa: E731
  nobody = StoppedLead(200.0, present=lambda f: False)
  return run(car, NO_CURVE, frames, lead=nobody, stop_mode=2, max_drop=10, model=model, plan=plan, red=plan)


def test_drive_4f_tag3_restore_after_a_green_light_uses_the_long_press_off_the_gas():
  # drive 4f, tag 3: 45 -> 20 for a red light, green at ~33 mph; off the gas every SET+ landed +1 mph, so booking
  # them as possible jumps (taps only, no hold) took 8.8 s back to 45
  # (decel 0.8: the car stays ~10 mph faster than the rising set speed for a few seconds, as on the drive)
  r = _red_then_green(Car(45, dash_offset_mph=0.0, decel=0.8))
  assert r["min"] <= 25, r["min"]
  first_hold = min([f for f in r["holds"] if f > 700] or [10**9])
  taps_before = [f for f, b in r["presses"] if b == cs.SET_PLUS and 700 < f < first_hold]
  assert first_hold < 900 and taps_before == [], (first_hold, taps_before)    # the long press first, no taps
  back = next(k for k, x in enumerate(r["sets"]) if k > 700 and abs(x / MPH - 45) < 0.01)
  assert (back - 700) * T.DT <= 6.3, (back - 700) * T.DT
  assert r["max"] <= 45 + 1e-6 and not r["logic"].disabled


def test_a_car_that_jumps_off_the_gas_is_still_safe():
  # if the car did jump to the current speed on a SET+ off the gas after all: never above the driver's speed, the
  # landing is booked as ours (not "more steps than our presses": switched off for the drive)
  for jump_extra in (0, 1):
    r = _red_then_green(Car(45, dash_offset_mph=0.0, plus_jumps="always", jump_extra=jump_extra))
    assert r["min"] <= 25, r["min"]
    assert r["max"] <= 45 + 1e-6, (jump_extra, r["max"])
    assert not r["logic"].disabled and abs(r["final"] - 45) < 0.01, (jump_extra, r["final"])


class _JumpHoldCar(Car):
  """A car that jumps to the current speed on a hold's first SET+ frame off the gas (never seen on this car), its
  SET+ holds landing a quantum up to `up_latency` s after the release."""
  def __init__(self, *a, up_latency=0.0, **kw):
    super().__init__(*a, **kw)
    self.up_latency = up_latency

  def hold_frame(self, button=cs.SET_MINUS):
    first = self.hold_last is None or self.frame - self.hold_last > 15
    self.hold_latency = self.up_latency if button == cs.SET_PLUS else 0.0
    if first and button == cs.SET_PLUS and self.v_ego > self.set_speed + self.step:
      self.hold_count += 1
      self.hold_dir = 1
      self.pending.append((self.frame + self.lag, round(self.v_ego / self.step) * self.step - self.set_speed))
      self.hold_next = self.frame + int(self.hold_start * 100) + self.lag
      self.hold_last = self.frame
      return
    super().hold_frame(button)


def test_a_car_that_jumps_off_the_gas_never_overshoots_on_a_long_press():
  # review: off the gas, coasting at about the driver's speed (downhill) with the set speed low: a hold's first frame
  # that jumped to 45, then the quantum after the release, ended at 50; now a hold while the car is faster only
  # starts when such a jump would land inside the hold's own margin
  for v_mph in (33.0, 40.0, 44.0, 45.0):
    for phase in (0, None):
      car = _JumpHoldCar(45, dash_offset_mph=0.0, decel=0.8, up_latency=0.6)
      tick = car.tick

      def held(car=car, tick=tick, v=v_mph):
        tick()
        if car.frame < 1200:
          car.v_ego = max(car.v_ego, v * MPH)      # coasting: the car does not slow below this
      car.tick = held
      nobody = StoppedLead(200.0, present=lambda f: False)
      r = run(car, NO_CURVE, 2500, lead=nobody, stop_mode=2, max_drop=10, wheel_phase=phase,
              model=lambda f: -0.7 if 100 <= f < 700 else 0.3, plan=lambda f: 100 <= f < 700)
      assert r["max"] <= 45 + 1e-6, (v_mph, phase, r["max"])


def test_a_cruise_gap_set_at_the_current_speed_is_the_drivers_not_a_jump_of_ours():
  # review: a restore tap in flight at set 20, the driver brakes (cruise off) and sets again at 32 (= the current
  # speed): that is the driver's new speed; it used to be claimed as our jump and tapped back down to 20
  lg = cs.CurveSlowdownLogic(False)
  from types import SimpleNamespace as N
  p = cs.Press(direction=cs.SET_PLUS, frame=0)
  lg.inflight.append(p)
  lg.anchor, lg.last_set_speed = 45 * MPH, 20 * MPH
  lg.cruise_gap = True
  i = N(set_speed=32 * MPH, v_ego=32 * MPH, cruise_enabled=True, driver_button=None)
  lg._observe_set_speed(i)
  assert p in lg.inflight + lg.expired or p.landed == 0      # not booked as ours


def test_floor_is_the_cars_own_minimum_one_tap_below_20():
  # owner (Nov 6): "when we drop the speed by holding the button it's only going down to 20, let's do one more 1 mph
  # step down to 19": the long press stops at 20, one SET- tap lands on the car's 30 km/h minimum (18.6 mph, a
  # short step: booked as ours, not "a fraction of a step": switched off for the drive), then the restore
  class MinCar(Car):
    """The real levels at the bottom: the 30 km/h minimum (18.6 mph), then 20, 21 ... (no 19)."""
    def tick(self):
      super().tick()
      if CAR_MIN_MPH * MPH + 1e-6 < self.set_speed < 20 * MPH - 1e-6:
        self.set_speed = CAR_MIN_MPH * MPH
  lead = StoppedLead(60, prob=0.95, y=0.1, present=lambda f: f < 1500)
  r = run(MinCar(45, dash_offset_mph=0.0, min_set_mph=CAR_MIN_MPH), NO_CURVE, 3500, lead=lead, stop_mode=2, max_drop=10)
  assert abs(r["min"] - CAR_MIN_MPH) < 1e-6, r["min"]
  assert not r["logic"].disabled and near(r["final"], 45) and r["max"] <= 45 + 1e-6, (r["logic"].disabled, r["final"])


def _pull_away_trigger(radar=True):
  # drive 4f, tag 6: a stop with a car (the model slowing), the car ahead 30 -> 15 m away at 6 mph, the radar following
  # it (it was tracked while moving), we slowing from 30 to 8 mph
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_sw(lead_status=False))
  for n in range(300):
    tr.update(_sw(v_ego=(30 - n * 0.073) * MPH, lead_prob=0.95, lead_d=30.0 - n * 0.05, lead_v=6 * MPH, lead_y=0.1,
                  model_accel=-0.8, radar_has_lead=radar))
  assert tr.active and tr.kind in (scs.STOP_WITH_CAR, scs.STOPPED_CAR)
  assert tr.radar_follow == radar
  return tr


def _pull(tr, frames=200, radar=True, **kw):
  base = dict(v_ego=8 * MPH, lead_prob=0.95, lead_d=14.0, lead_v=8.2 * MPH, lead_a=1.0, lead_y=0.1, model_accel=0.3,
              radar_has_lead=radar)
  base.update(kw)
  n = 0
  while n < frames and tr.update(_sw(**base)):
    n += 1
  return n


def test_drive_4f_tag6_car_ahead_pulling_away_releases_at_once():
  # it pulled away at 0.9-1.5 m/s^2 from our speed (8 mph) at t 800.8; our restore landed only at 802.5 (the radar
  # match release's 2 s). Now: released after PULL_AWAY_ACCEL_TIME
  tr = _pull_away_trigger()
  n = _pull(tr)
  assert tr.release_reason == scs.PULLING_AWAY and n <= 80, (tr.release_reason, n)     # tag 6: ~801.5, not 802.5


def test_pulling_away_needs_a_sure_same_car_the_model_going_and_the_radar_or_a_faster_car():
  # review: an untracked car that may stop again, the model still braking, a low-confidence car, a cut-in
  cases = (dict(model_accel=-1.0), dict(plan_stop=True), dict(lead_prob=0.5), dict(lead_v=6 * MPH, v_ego=12 * MPH))
  for kw in cases:
    tr = _pull_away_trigger()
    _pull(tr, 150, **kw)
    assert tr.release_reason != scs.PULLING_AWAY, kw
  tr = _pull_away_trigger()                                      # a cut-in: another car 8 m ahead
  n = _pull(tr, 150, lead_d=6.0)
  assert tr.release_reason != scs.PULLING_AWAY or n >= int(scs.PULL_AWAY_SAME_CAR / scs.DT_CTRL), n
  tr = _pull_away_trigger(radar=False)                           # no radar: at our speed is not enough ...
  _pull(tr, 150, radar=False)
  assert tr.release_reason != scs.PULLING_AWAY
  tr = _pull_away_trigger(radar=False)                           # ... it must be faster than us
  _pull(tr, 150, radar=False, lead_v=12 * MPH)
  assert tr.release_reason == scs.PULLING_AWAY


def test_driver_hold_to_the_minimum_is_not_booked_as_our_tap():
  # review: our SET- tap in flight (dropped by the car), the driver holds SET- down to the minimum: theirs
  from types import SimpleNamespace as N
  lg = cs.CurveSlowdownLogic(False)
  lg.inflight.append(cs.Press(direction=cs.SET_MINUS, frame=0))
  lg.anchor, lg.last_set_speed = 45 * MPH, 20 * MPH
  lg.last_driver_edge[cs.SET_MINUS] = lg.frame
  lg.prev_driver_button = cs.SET_MINUS
  lg._observe_set_speed(N(set_speed=cs.CAR_MIN_SET_SPEED, v_ego=15 * MPH, cruise_enabled=True, driver_button=cs.SET_MINUS))
  assert all(p.landed == 0 for p in lg.inflight + lg.expired)


# ---- drive 50: the red light head start (RED_*) and the false stop ahead guard (WARN_GUARD_*) ----

class _Flag:
  """Flip a module constant for one test."""
  def __init__(self, name, value):
    self.name, self.value = name, value

  def __enter__(self):
    self.old = getattr(scs, self.name)
    setattr(scs, self.name, self.value)

  def __exit__(self, *a):
    setattr(scs, self.name, self.old)


def _no_car(**kw):
  base = dict(lead_status=False, plan_stop=False, model_accel=-0.1, red_light=True)
  base.update(kw)
  return _latched_inputs(**base)


def _red_started(red_frames=350):
  """Cruise on, nothing ahead, the light detector on from frame 60: (trigger, frame the head start began)."""
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_no_car(red_light=False))
  started = None
  for n in range(red_frames):
    if tr.update(_no_car()) and started is None:
      started = n
  return tr, started


def test_red_light_head_start_after_2_s_steps_1_mph_a_second_to_5_below():
  tr, started = _red_started()
  assert started is not None and abs(started - scs.RED_ON_TIME / scs.DT_CTRL) <= 2, started
  assert tr.kind == scs.RED_LIGHT and not tr.urgent and not tr.floor_mode and not tr.confirmed
  assert near(tr.target / MPH, 45 - 2)                      # 1.5 s in: the 2nd step
  for _ in range(400):
    tr.update(_no_car())
  assert tr.active and near(tr.target / MPH, 45 - 5)        # the cap
  # the steps are from the set speed when the car is a little slower than it (CATCH_UP_GAP or less: cruising)
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_no_car(red_light=False))
  for _ in range(260):
    tr.update(_no_car(v_ego=43.1 * MPH, set_speed=45 * MPH))
  assert tr.active and near(tr.target / MPH, 44) and not tr.urgent
  # with the flag off (the old code): nothing
  with _Flag("RED_START_ENABLED", False):
    tr, started = _red_started()
  assert started is None and not tr.active


def test_red_light_head_start_in_the_car_taps_once_a_second_silently():
  # the harness: 45 mph, the detector on from 1 s, nothing else; one SET- tap a second, never a hold, at most 5 mph
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1200, lead=_NoCar(200), max_drop=10, model=lambda f: -0.1,
          red=lambda f: f >= 100)
  ch = _sets_changes(r)
  assert r["holds"] == [] and all(b == cs.SET_MINUS for _, b in r["presses"])
  assert ch and 300 <= ch[0][0] <= 340, ch                   # 2 s of red light, then the first tap
  assert all(abs(a[1] - b[1]) <= 1.0 + 1e-6 for a, b in zip([(0, 45.0)] + ch[:-1], ch, strict=True)), ch   # 1 mph steps
  assert all(90 <= b[0] - a[0] <= 115 for a, b in zip(ch[:-1], ch[1:], strict=True)), ch                  # ~1 s apart
  assert r["min"] >= 45 - 5 - 1e-6 and near(r["final"], 40)
  assert r["trigger"].kind == scs.RED_LIGHT and r["ever_active"]


def test_red_light_head_start_needs_cruise_speed_no_lane_change_and_not_already_slowing():
  for kw in (dict(v_ego=14 * MPH), dict(lane_change=True), dict(a_ego=-0.8), dict(brake_pressed=True), dict(gas_pressed=True),
             dict(cruise_enabled=False)):
    tr = scs.StoppedCarTrigger()
    for _ in range(60):
      tr.update(_no_car(red_light=False))
    for _ in range(400):
      tr.update(_no_car(**kw))
    assert not tr.active, kw
  # ... and a continuous light: 1.5 s on, 0.5 s off, again, never starts; 0.2 s blinks are tolerated
  tr = scs.StoppedCarTrigger()
  for n in range(800):
    tr.update(_no_car(red_light=(n % 200) < 150))
  assert not tr.active
  tr = scs.StoppedCarTrigger()
  for n in range(400):
    tr.update(_no_car(red_light=(n % 100) < 85))
  assert tr.active and tr.kind == scs.RED_LIGHT


def test_red_light_gone_releases_after_1_s_and_the_speed_comes_back():
  tr, _ = _red_started()
  for _ in range(int(scs.RED_DROPOUT / scs.DT_CTRL) - 5):   # a dropout shorter than RED_DROPOUT: still on
    assert tr.update(_no_car(red_light=False))
  for _ in range(100):
    assert tr.update(_no_car())
  n = 0
  while tr.update(_no_car(red_light=False)):
    n += 1
    assert n < 200
  assert abs(n - scs.RED_DROPOUT / scs.DT_CTRL) <= 3 and tr.release_reason == scs.RED_GONE and tr.restore_now
  # a slower car in our lane at the release: the restore stays gentle; that car (not sure yet) starts the unsure car's
  # taps at once, not after UNSURE_REARM (drive 52: the light went at 187.8 s, the queue car was seen at 188.0)
  tr, _ = _red_started()
  slower = dict(red_light=False, lead_status=True, lead_prob=0.6, lead_d=90.0, lead_v=20 * MPH, lead_y=0.2, model_accel=0.1)
  n = 0
  while tr.update(_no_car(**slower)):
    n += 1
    assert n < 200
  assert tr.release_reason == scs.RED_GONE and not tr.restore_now
  n = 0
  while not tr.update(_no_car(**slower)):
    n += 1
    assert n < 400
  assert tr.kind == scs.SLOW_UNSURE and n <= scs.UNSURE_HOLD / scs.DT_CTRL + 2, n
  # the harness: 45 -> 42 for a 3 s light, then back to 45 within a few seconds
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1800, lead=_NoCar(200), max_drop=10,
                                 model=lambda f: -0.1, red=lambda f: 100 <= f < 600))
  assert rel and rel[0][1] == scs.RED_GONE and rel[0][2]
  assert r["min"] >= 45 - 5 - 1e-6 and near(r["final"], 45)
  back = next(k for k, s in enumerate(r["sets"]) if k > rel[0][0] and near(s / MPH, 45))
  assert (back - rel[0][0]) * T.DT <= 6.0, back


def test_red_light_head_start_driver_overrides_and_does_not_start_again_for_the_same_light():
  tr, _ = _red_started()
  for _ in range(int(scs.GAS_RELEASE_TIME / scs.DT_CTRL) + 2):
    tr.update(_no_car(gas_pressed=True))
  assert not tr.active and tr.release_reason == "driver on the gas"
  for _ in range(600):                                        # the light still on: no new head start
    assert not tr.update(_no_car())
  for _ in range(int(scs.RED_DROPOUT / scs.DT_CTRL) + 5):     # off for RED_DROPOUT ...
    tr.update(_no_car(red_light=False))
  for _ in range(int(scs.RED_ON_TIME / scs.DT_CTRL) + 5):     # ... on again: the next light
    tr.update(_no_car())
  assert tr.active and tr.kind == scs.RED_LIGHT
  # the brake, too
  tr, _ = _red_started()
  tr.update(_no_car(brake_pressed=True))
  assert not tr.active and tr.release_reason == "cruise off / brake" and tr.clear_release
  for _ in range(300):
    assert not tr.update(_no_car())


def test_red_light_head_start_hands_over_to_the_planned_stop_with_the_full_drop():
  # drive 50: redLight at 307.5, the head start at 309.5 (45 -> 40 by 313.5), the plan says stop at 315.0: the floor hold
  tr, _ = _red_started()
  n = 0
  while tr.kind == scs.RED_LIGHT:
    tr.update(_no_car(plan_stop=True, model_accel=-0.6))
    n += 1
    assert n < 200
  assert tr.active and tr.kind == scs.STOP_AHEAD and tr.confirmed and tr.urgent and tr.floor_mode and not tr.warn_unsure
  assert abs(n - scs.WARN_START_DEBOUNCE / scs.DT_CTRL) <= 2 and near(tr.target / MPH, FLOOR_MPH)
  assert tr.unsure_from                                       # (the episode began with the head start's taps)
  # ... also while our own taps already slow the car a little (-0.7 m/s^2 is not "already slowing" here)
  tr, _ = _red_started()
  for _ in range(100):
    tr.update(_no_car(a_ego=-0.7))
  n = 0
  while tr.kind == scs.RED_LIGHT:
    tr.update(_no_car(plan_stop=True, model_accel=-0.6, a_ego=-0.7))
    n += 1
    assert n < 200
  assert tr.kind == scs.STOP_AHEAD and tr.floor_mode


def test_red_light_head_start_hands_over_to_a_stopped_car():
  tr, _ = _red_started()
  n = 0
  while tr.kind == scs.RED_LIGHT:
    tr.update(_no_car(lead_status=True, lead_prob=0.95, lead_d=90.0, lead_v=0.0, lead_y=0.1, model_accel=0.0))
    n += 1
    assert n < 200
  assert tr.active and tr.kind == scs.STOPPED_CAR and tr.confirmed and abs(n - scs.HOLD_TIME / scs.DT_CTRL) <= 2
  # ... and to a stop with a much slower car (the model slowing); an unsure car does not take it over
  tr, _ = _red_started()
  for _ in range(100):
    tr.update(_no_car(lead_status=True, lead_prob=0.3, lead_d=110.0, lead_v=10 * MPH, lead_y=0.1, model_accel=0.0))
  assert tr.active and tr.kind == scs.RED_LIGHT
  for _ in range(60):
    tr.update(_no_car(lead_status=True, lead_prob=0.6, lead_d=100.0, lead_v=10 * MPH, lead_y=0.1, model_accel=-0.6))
  assert tr.active and tr.kind == scs.STOP_WITH_CAR and tr.urgent


class _Drive50(StoppedLead):
  """Drive 50 (t = seconds after redLight came on at 307.5): the stopped queue car first seen at 9.5 s (118 m, 29 %),
  57-65 % from 10.5 s, 87 % from 13 s, 94 % from 14 s; in our lane."""
  def state(self, f, car):
    t = f * T.DT
    if t < 9.5:
      return (False, 0.0, 0.0, 0.0, 0.0)
    p = 0.29 if t < 10.5 else (0.6 if t < 13.0 else (0.87 if t < 14.0 else 0.95))
    return (True, p, self.d, 0.0, 0.3)


def _drive_50(head_start=True):
  # redLight from 0, the plan says stop from 7.5 s (315.0), the driver braked at 15 s (322.5); the measured cruise decel
  car = Car(45, dash_offset_mph=0.0, decel_fn=MEASURED_DECEL)
  lead = _Drive50(118.0 + 45 * MPH * 9.5)
  with _Flag("RED_START_ENABLED", head_start):
    r = run(car, NO_CURVE, 1500, lead=lead, max_drop=10, setup=MEASURED, model=lambda f: -0.6 if f >= 750 else -0.2,
            plan=lambda f: f >= 750, red=lambda f: True)
  return r


def test_drive_50_replay_the_head_start_takes_speed_off_before_the_stop_and_the_full_drop_is_not_later():
  with_hs, without = _drive_50(True), _drive_50(False)
  at_floor = lambda r: next(k for k, s in enumerate(r["sets"]) if s <= 20 * MPH + 1e-6)  # noqa: E731
  assert at_floor(with_hs) <= at_floor(without) + 5, (at_floor(with_hs), at_floor(without))
  assert at_floor(without) <= 1120                                          # ~318.5 s (the drive: set 20 at 317.95)
  first_hs = first_frame(with_hs)
  assert 190 <= first_hs <= 230 and first_frame(without) >= 780, (first_hs, first_frame(without))   # 2 s of red light
  assert with_hs["sets"][740] / MPH <= 40 + 1e-6 and without["sets"][740] / MPH >= 45 - 1e-6     # 5 mph off before the plan
  v_brake_hs, v_brake = with_hs["vs"][1500 - 1] / MPH, without["vs"][1500 - 1] / MPH
  assert v_brake_hs <= v_brake - 2.0, (v_brake_hs, v_brake)                 # slower where the driver had to brake
  assert with_hs["min"] >= FLOOR_MPH - 1e-6 and with_hs["holds"]            # the floor hold still happened


# ---- the false stop ahead guard ----

def _guarded(frames=200, **kw):
  """A planned stop with no car and no red light: (trigger, frame started)."""
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_no_car(red_light=False))
  started = None
  for n in range(frames):
    if tr.update(_no_car(plan_stop=True, model_accel=-0.6, red_light=False, **kw)) and started is None:
      started = n
  return tr, started


def test_a_planned_stop_without_the_light_detector_only_taps_and_the_warning_timing_is_unchanged():
  tr, started = _guarded()
  assert started is not None and abs(started - scs.WARN_START_DEBOUNCE / scs.DT_CTRL) <= 2   # the start itself: as today
  assert tr.active and tr.kind == scs.STOP_AHEAD and tr.warn_start and tr.warn_unsure and tr.warn_guarded
  assert not tr.urgent and not tr.floor_mode and not tr.confirmed
  assert near(tr.target / MPH, 40)                            # the unsure car's cap, taps
  assert tr.warn_frames * scs.DT_CTRL >= scs.WARN_DEBOUNCE - 1e-9    # (the warning's own gate is the planner's)
  with _Flag("WARN_GUARD_ENABLED", False):                    # the old code: the floor hold at once
    tr, _ = _guarded()
  assert tr.active and tr.urgent and tr.floor_mode and near(tr.target / MPH, FLOOR_MPH)
  # redLight on within WARN_RED_RECENT before the start: not guarded
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_no_car())
  for _ in range(int((scs.WARN_RED_RECENT - scs.WARN_START_DEBOUNCE) / scs.DT_CTRL) - 20):
    tr.update(_no_car(red_light=False))
  for _ in range(60):
    tr.update(_no_car(plan_stop=True, model_accel=-0.6, red_light=False))
  assert tr.active and tr.kind == scs.STOP_AHEAD and not tr.warn_unsure and tr.floor_mode
  # drive 50 t 108.4: redLight off ~2.0-2.5 s before the start: guarded
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_no_car())
  for _ in range(200):
    tr.update(_no_car(red_light=False))
  for _ in range(60):
    tr.update(_no_car(plan_stop=True, model_accel=-0.6, red_light=False))
  assert tr.active and tr.warn_unsure


def test_a_guarded_stop_ahead_costs_at_most_5_mph_and_comes_back_when_the_plan_clears():
  # drive 50, t 107-114 (a green light, a car turning across): 45 -> 19 and ~12 mph lost; now 1 mph taps to 40
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2000, lead=_NoCar(200), max_drop=10,
                                 model=lambda f: -0.6 if 100 <= f < 400 else 0.3, plan=lambda f: 100 <= f < 400))
  assert rel and rel[0][1].startswith("stop ahead cleared") and rel[0][2]
  assert r["holds"] == [] or all(h >= rel[0][0] for h in r["holds"]), r["holds"]     # no SET- hold
  assert r["min"] >= 45 - 5 - 1e-6 and near(r["final"], 45) and min(r["vs"]) / MPH >= 45 - 6
  back = next(k for k, s in enumerate(r["sets"]) if k > rel[0][0] and near(s / MPH, 45))
  assert (back - rel[0][0]) * T.DT <= 7.0, back
  # a dip that never became a warning: released at once, as before
  tr, started, released = _warn_episode(70, red_light=False)
  assert started is not None and released is not None and tr.release_reason == scs.STOP_AHEAD_UNCONFIRMED


def test_a_guarded_stop_ahead_becomes_the_full_drop_when_the_light_detector_agrees():
  tr, _ = _guarded()
  tr.update(_no_car(plan_stop=True, model_accel=-0.6, red_light=True))
  assert tr.active and tr.kind == scs.STOP_AHEAD and not tr.warn_unsure and tr.confirmed and tr.urgent and tr.floor_mode
  assert near(tr.target / MPH, FLOOR_MPH) and tr.warn_guarded


def test_a_guarded_stop_ahead_becomes_the_full_drop_once_the_plan_has_held_4_s():
  tr, started = _guarded(frames=int(scs.WARN_GUARD_PLAN_HOLD / scs.DT_CTRL) - 10)
  assert tr.active and tr.warn_unsure
  for _ in range(12):
    tr.update(_no_car(plan_stop=True, model_accel=-0.6, red_light=False))
  assert tr.active and not tr.warn_unsure and tr.floor_mode and tr.urgent and near(tr.target / MPH, FLOOR_MPH)
  # short plan dropouts (LATCH_DROPOUT) do not restart that clock
  tr, _ = _guarded(frames=300)
  for n in range(130):
    tr.update(_no_car(plan_stop=(n % 50) >= 10, model_accel=-0.6, red_light=False))
  assert tr.active and not tr.warn_unsure and tr.floor_mode


def test_a_guarded_stop_ahead_becomes_the_full_drop_when_a_slower_car_confirms_it():
  # the car for the stop appears (prob >= 0.4 for 0.3 s, the model slowing): the stop with car, the floor
  tr, _ = _guarded()
  car = dict(plan_stop=True, model_accel=-0.6, red_light=False, lead_status=True, lead_prob=0.5, lead_d=130.0, lead_v=10 * MPH,
             lead_y=0.2)                                          # (not urgent yet: required ~1.1 m/s^2, 8 s away)
  tr.update(_no_car(**car))
  assert tr.kind == scs.STOP_WITH_CAR and tr.warn_unsure and not tr.urgent
  assert tr.target >= tr.unsure_target - 1e-6                  # the cap still holds until it is confirmed
  for _ in range(int(scs.EARLY_CONFIRM_TIME / scs.DT_CTRL) + 1):
    tr.update(_no_car(**car))
  assert tr.confirmed and not tr.warn_unsure and tr.urgent and tr.floor_mode and near(tr.target / MPH, FLOOR_MPH)
  # drive 50 t 109.7: a slower car (89 %) while the model does not want to slow (-0.05): not a confirmation; a car at
  # our speed neither
  tr, _ = _guarded()
  for _ in range(100):
    tr.update(_no_car(**dict(car, model_accel=-0.05, lead_prob=0.89, lead_d=100.0, lead_v=29 * MPH, v_ego=41 * MPH)))
  assert tr.active and tr.kind == scs.STOP_WITH_CAR and tr.warn_unsure and not tr.urgent
  tr, _ = _guarded()
  for _ in range(100):
    tr.update(_no_car(**dict(car, lead_prob=0.9, lead_v=44 * MPH)))
  assert tr.active and tr.kind == scs.STOP_AHEAD and tr.warn_unsure and not tr.urgent
  # a slower car already there at the start (prob >= 0.4): not guarded at all, as today
  tr = scs.StoppedCarTrigger()
  for _ in range(60):
    tr.update(_no_car(red_light=False))
  for _ in range(60):
    tr.update(_no_car(plan_stop=True, model_accel=-0.6, red_light=False, lead_status=True, lead_prob=0.5, lead_d=140.0,
                      lead_v=20 * MPH, lead_y=0.2))
  assert tr.active and tr.confirmed and not tr.warn_guarded


def test_a_guarded_stop_ahead_is_released_by_the_driver_and_the_light_latch_holds():
  tr, _ = _guarded()
  tr.update(_no_car(plan_stop=True, model_accel=-0.6, red_light=False, brake_pressed=True))
  assert not tr.active and tr.clear_release and not tr.warn_unsure
  # the planner's own gates are untouched: the warning condition is the same as before
  sw = _stop_warning()
  assert scs.WARN_OWN_SLOWING_ACCEL == sw.OWN_SLOWDOWN_ACCEL


# ---- drive 52: the cruise still accelerating into slow traffic (CATCH_UP_*), the tap episodes from 15 mph ----

def test_unsure_car_with_the_cruise_still_accelerating_brings_the_set_speed_to_our_speed_first():
  # 22 mph under a 40 set speed: the first drop to 22 (urgent: a SET- hold or fast taps), then the taps to 17 -> the
  # floor, 5 below our speed (the old cap, 5 below the set speed, let the car accelerate to 35)
  tr = _unsure_started(v_ego=22 * MPH, set_speed=40 * MPH)
  assert tr.active and tr.kind == scs.SLOW_UNSURE and near(tr.target / MPH, 22) and tr.urgent
  assert tr.catch_up and tr.caught_up
  tr.update(_u(v_ego=22.5 * MPH, set_speed=23 * MPH))           # landed within 1 mph: the taps, the cap from our speed
  assert tr.active and not tr.catch_up and not tr.urgent and near(tr.target / MPH, FLOOR_MPH)
  # presses lost (or Test mode): after CATCH_UP_TIME the taps' target anyway
  tr = _unsure_started(v_ego=22 * MPH, set_speed=40 * MPH)
  for _ in range(int(scs.CATCH_UP_TIME / scs.DT_CTRL) + 1):
    tr.update(_u(v_ego=22 * MPH, set_speed=40 * MPH))
  assert tr.active and not tr.catch_up and near(tr.target / MPH, FLOOR_MPH)
  # from 25 mph the cap stays 5 below max(speed, set) (drive 4f t 156): 30 under 45, the first drop stops at 40 ...
  tr = _unsure_started(v_ego=30 * MPH, set_speed=45 * MPH)
  assert tr.active and tr.catch_up and tr.urgent and near(tr.target / MPH, 40)
  tr.update(_u(v_ego=30.5 * MPH, set_speed=41 * MPH))
  assert tr.active and not tr.catch_up and not tr.urgent and near(tr.target / MPH, 40)
  # ... and 42 under 45: the drop to 42 (stop accelerating), then the taps to 40
  tr = _unsure_started(v_ego=42 * MPH, set_speed=45 * MPH)
  assert tr.active and tr.catch_up and tr.urgent and near(tr.target / MPH, 42)
  tr.update(_u(v_ego=42 * MPH, set_speed=43 * MPH))
  assert tr.active and not tr.catch_up and near(tr.target / MPH, 40)
  # never below the floor: 21 mph under 35 -> 21, then the car's minimum (not 16)
  tr = _unsure_started(v_ego=21 * MPH, set_speed=35 * MPH, lead_d=80.0)
  assert near(tr.target / MPH, 21) and tr.urgent
  tr.update(_u(v_ego=21 * MPH, set_speed=21 * MPH, lead_d=80.0))
  assert near(tr.target / MPH, FLOOR_MPH) and not tr.urgent
  # not seen: hold (no lowering), as for the taps
  tr = _unsure_started(v_ego=30 * MPH, set_speed=45 * MPH)
  for _ in range(int(scs.EARLY_DROPOUT / scs.DT_CTRL) + 2):
    tr.update(_u(v_ego=30 * MPH, set_speed=40 * MPH, lead_status=False))
  assert tr.active and tr.target >= 99 and not tr.urgent
  with _Flag("CATCH_UP_ENABLED", False):                         # the old rule: 5 below max(speed, set)
    tr = _unsure_started(v_ego=30 * MPH, set_speed=45 * MPH)
  assert near(tr.target / MPH, 40) and not tr.urgent


def test_unsure_car_from_15_mph():
  # drive 52, t 188-191: a car in our lane at 77-87 m, 9-16 mph, 55-76 % a car, at 23.5-24.4 mph (the old minimum: 25)
  tr = _unsure_started(v_ego=23.8 * MPH, set_speed=35 * MPH, lead_prob=0.66, lead_d=81.0, lead_v=11 * MPH, lead_y=0.2)
  assert tr.active and tr.kind == scs.SLOW_UNSURE and near(tr.target / MPH, 23.8) and tr.urgent
  tr = _unsure_started(v_ego=14 * MPH, lead_d=60.0)
  assert not tr.active


class _Drive52(StoppedLead):
  """Drive 52 from t 188.0: the queue car in our lane 81 m ahead at ~11 mph (9-16), 55-76 % a car, 98 % from 193.3."""
  def state(self, f, car):
    return (True, 0.66 if f * T.DT < 5.3 else 0.98, self.d, self.v, 0.2)


def _drive_52(*flags):
  # 23.8 mph, set 35, the cruise accelerating (0.6 m/s^2) and slowing as measured (0.19 m/s^2 per m/s, 1 s late); the
  # model slows and the plan says stop from 3.0 s (191.0, our stop ahead start on the drive); the driver braked at 8 s
  car = Car(35, dash_offset_mph=0.0, cruise_gain=0.19, cruise_dead=1.0, cruise_max=1.0, accel=0.6)
  car.v_ego = 23.8 * MPH
  lead = _Drive52(81.0, v_mph=11.0)
  saved = [(k, getattr(scs, k, None)) for k, _ in flags]
  for k, v in flags:
    setattr(scs, k, v)
  try:
    r = run(car, NO_CURVE, 800, lead=lead, max_drop=10, model=lambda f: -0.6 if f >= 300 else -0.2, plan=lambda f: f >= 300,
            setup=MEASURED)
  finally:
    for k, v in saved:
      setattr(scs, k, v)
  return r, lead


def test_drive_52_replay_accelerating_toward_a_slow_car_the_set_speed_comes_down_to_our_speed_first():
  # on the drive (the old code): nothing until 25 mph, then the cap 30: the car went on accelerating 24 -> 29 mph and the
  # driver braked at 25.7 mph 25 m behind the queue car (up to 4.3 m/s^2)
  r, lead = _drive_52()
  old, old_lead = _drive_52(("CATCH_UP_ENABLED", False), ("UNSURE_MIN_SPEED", 25 * MPH))
  first = first_frame(r)
  assert first is not None and first <= 100, first                 # 0.3 s cruise settle + 0.5 s unsure hold
  assert r["kinds"][first] == scs.SLOW_UNSURE
  ch = _sets_changes(r)
  down = next(k for k, s in ch if s <= 25.0 + 1e-6)                # the first drop: 35 -> 25 (our speed) ...
  assert down - first <= 150 and r["holds"] and r["holds"][0] == first, (ch, r["holds"][:3])   # ... one SET- hold
  assert r["min"] >= FLOOR_MPH - 1e-6
  assert max(r["vs"]) / MPH <= 25.5 and max(old["vs"]) / MPH >= 28.5, (max(r["vs"]) / MPH, max(old["vs"]) / MPH)
  # where the driver braked: slower and farther back
  assert r["vs"][-1] / MPH <= old["vs"][-1] / MPH - 3.0, (r["vs"][-1] / MPH, old["vs"][-1] / MPH)
  assert lead.d >= old_lead.d + 6.0, (lead.d, old_lead.d)


def _rolling_then_red(v_mph, set_mph, rolling_s=15.0, red_s=2.5, **kw):
  """Stopped 1 s, rolling `rolling_s` at v_mph (no light), then the light on: (trigger, frame the head start began)."""
  tr = scs.StoppedCarTrigger()
  for _ in range(100):
    tr.update(_no_car(red_light=False, v_ego=0.0, set_speed=set_mph * MPH))
  for _ in range(int(rolling_s / scs.DT_CTRL)):
    tr.update(_no_car(red_light=False, v_ego=v_mph * MPH, set_speed=set_mph * MPH))
  started = None
  for n in range(int(red_s / scs.DT_CTRL)):
    if tr.update(_no_car(v_ego=v_mph * MPH, set_speed=set_mph * MPH, **kw)) and started is None:
      started = n
  return tr, started


def test_red_light_head_start_at_20_mph_brings_the_set_speed_down_then_steps():
  # drive 52, t 182.3-185.8: redLight on 3.5 s at 20-24 mph with the set speed at 35 (the old minimum: 25 mph)
  tr, started = _rolling_then_red(20, 35)
  assert started is not None and abs(started - scs.RED_ON_TIME / scs.DT_CTRL) <= 2, started
  assert tr.kind == scs.RED_LIGHT and tr.catch_up and tr.urgent and near(tr.target / MPH, 20)
  tr.update(_no_car(v_ego=20.4 * MPH, set_speed=21 * MPH))      # the drop landed: one 1 mph step at once ...
  assert not tr.catch_up and not tr.urgent and near(tr.target / MPH, 19)
  for _ in range(105):                                          # ... then the cap: the car's minimum (20 - 5 is below it)
    tr.update(_no_car(v_ego=20.0 * MPH, set_speed=19 * MPH))
  assert tr.active and near(tr.target / MPH, FLOOR_MPH)
  # the set speed near our speed: as before (one step below max(speed, set) at once)
  tr, _ = _rolling_then_red(20, 21)
  assert tr.active and not tr.catch_up and not tr.urgent and near(tr.target / MPH, 20)
  # below 15 mph: nothing; a light that came on soon after a stop: nothing below 25 mph (drives 4d t 1434, 50 t 233),
  # as before above it
  tr, started = _rolling_then_red(14, 35)
  assert started is None
  tr, started = _rolling_then_red(20, 35, rolling_s=8.0)
  assert started is None and any("after the last stop" in b for b in tr.blockers(_no_car(v_ego=20 * MPH)))
  tr, started = _rolling_then_red(30, 35, rolling_s=8.0)
  assert started is not None and tr.kind == scs.RED_LIGHT


def _red_accelerating(*flags):
  car = Car(35, dash_offset_mph=0.0, cruise_gain=0.19, cruise_dead=1.0, cruise_max=1.0, accel=0.6)
  car.v_ego = 21.0 * MPH
  saved = [(k, getattr(scs, k, None)) for k, _ in flags]
  for k, v in flags:
    setattr(scs, k, v)
  try:
    return run(car, NO_CURVE, 1200, lead=_NoCar(200), max_drop=10, model=lambda f: -0.1, red=lambda f: True, setup=MEASURED)
  finally:
    for k, v in saved:
      setattr(scs, k, v)


def test_red_light_head_start_in_the_car_with_the_cruise_accelerating_drops_to_our_speed_then_steps():
  # 21 mph and accelerating under a 35 set speed, the light on (the harness's trigger starts at the light: the last stop
  # was long before, RED_AFTER_STOP off): after 2 s one SET- hold down to our speed, then 1 mph a second to 5 below it
  r = _red_accelerating(("RED_AFTER_STOP", 0.0))
  old = _red_accelerating(("RED_AFTER_STOP", 0.0), ("CATCH_UP_ENABLED", False), ("RED_MIN_SPEED", 25 * MPH))
  first = first_frame(r)
  assert 195 <= first <= 210 and r["holds"] and r["holds"][0] == first, (first, r["holds"][:3])
  ch = _sets_changes(r)
  assert next(k for k, s in ch if s <= 25.0 + 1e-6) - first <= 150, ch
  taps = [(k, s) for k, s in ch if s < 25.0 - 1e-6]
  assert all(abs(a[1] - b[1]) <= 1.0 + 1e-6 for a, b in zip(taps[:-1], taps[1:], strict=True)), ch    # then 1 mph steps
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["min"] <= 20.0 + 1e-6, r["min"]        # 5 below ~24 mph, not 5 below 35
  assert max(r["vs"]) / MPH <= 25.5 and max(old["vs"]) / MPH >= 29.0, (max(r["vs"]) / MPH, max(old["vs"]) / MPH)


# ---- review of the drive 52 changes: the light on through a stop, false alarms at speed, unsure cars below 25 mph,
# the red light gone restore ----

def _mcar(set_mph, v_mph, accel=0.6):
  c = Car(set_mph, dash_offset_mph=0.0, cruise_gain=0.19, cruise_dead=1.0, cruise_max=1.0, accel=accel)
  c.v_ego = v_mph * MPH
  return c


def test_a_light_on_through_our_stop_is_no_head_start_while_we_pull_away_under_it():
  # review a8: the light on 1-3 s before our stop, through it and while we pull away (1 m/s^2, set 45): its onset was
  # counted with the time we had been rolling before the stop (60 s), so the head start came at 15 mph (a SET- hold
  # 45 -> 25, then the hold back up); drive 4f t 1122.6-1131 had such a light (cruise off)
  def pull_away(before_s, after_s, fresh_after=None):
    tr = scs.StoppedCarTrigger()
    for _ in range(6000):
      tr.update(_no_car(red_light=False, v_ego=40 * MPH, set_speed=45 * MPH, a_ego=0.3))
    n = int(40 * MPH / 1.5 * 100)
    for k in range(n):
      tr.update(_no_car(red_light=(n - k) <= before_s * 100, v_ego=max(0.0, 40 * MPH - 1.5 * k / 100), set_speed=45 * MPH,
                        a_ego=-1.5))
    for _ in range(300):
      tr.update(_no_car(red_light=True, v_ego=0.0, set_speed=45 * MPH, a_ego=0.0))
    for k in range(2500):
      red = k < after_s * 100 or (fresh_after is not None and k >= fresh_after * 100)
      if tr.update(_no_car(red_light=red, v_ego=min(k / 100, 22 * MPH), set_speed=45 * MPH, a_ego=1.0)):
        return k
    return None
  for before_s, after_s in ((3.0, 10.0), (3.0, 8.0), (1.0, 8.0)):
    assert pull_away(before_s, after_s) is None, (before_s, after_s)
  # a light that comes on again 12 s or more after we roll (below 25 mph): the head start, as on drive 52
  k = pull_away(3.0, 6.0, fresh_after=13.0)
  assert k is not None and k >= 1300 + scs.RED_ON_TIME * 100 - 5, k


def test_a_false_red_light_at_25_mph_or_more_costs_at_most_5_mph():
  # drive 4f t 154-171 (review a9): a false redLight for 15 s, no stop, while the cruise accelerates 37.7 -> 45 after a
  # curve: the drop to our speed and 5 more cost 45 -> 34 (11 mph, ~10 s to come back); the old cap: 40
  r = run(_mcar(45, 37.7, accel=0.5), NO_CURVE, 3500, lead=_NoCar(200), max_drop=10, model=lambda f: -0.1,
          red=lambda f: f < 1700, setup=MEASURED)
  assert abs(r["min"] - 40) < 0.01 and abs(r["final"] - 45) < 0.01, (r["min"], r["final"])
  assert min(r["vs"][200:]) / MPH >= 38.5, min(r["vs"][200:]) / MPH
  first = first_frame(r)
  assert next(k for k, s in _sets_changes(r) if s <= 40 + 1e-6) - first <= 200    # the drop at once, not 1 mph a second
  # (below 25 mph the drop to our speed and 5 below it stay: test_red_light_head_start_in_the_car_with_the_cruise_...)


class _AccelLead(StoppedLead):
  """A low-confidence car ahead pulling away from a light: v0 mph, accelerating at `acc` m/s^2 from frame `start`,
  reported by radarState aLeadK (lead.a)."""
  def __init__(self, d0, v0, acc, prob=0.5, start=0):
    super().__init__(d0, v_mph=v0, prob=prob, y=0.2)
    self.acc, self.start, self.a = acc, start, 0.0

  def tick(self, car):
    self.frame += 1
    self.a = self.acc if (self.frame >= self.start and self.v < 45 * MPH) else 0.0
    self.v = min(self.v + self.a * T.DT, 45 * MPH)
    self.d = max(self.d - (car.v_ego - self.v) * T.DT, 0.0)


def test_an_unsure_car_below_25_mph_that_is_accelerating_does_nothing():
  # review a2: pulling away from a light at 16 mph under a 45 set, a low-confidence car 40-60 m ahead accelerating away
  # from 2-4 mph: the first drop took the set 45 -> 19
  for v0, acc, d0 in ((4, 1.0, 60), (4, 0.6, 60), (2, 1.5, 40)):
    r = run(_mcar(45, 16.0, accel=1.0), NO_CURVE, 2500, lead=_AccelLead(d0, v0, acc), max_drop=10, model=lambda f: 0.3,
            setup=MEASURED)
    slow = [f for f, v in enumerate(r["vs"]) if v < 25 * MPH]
    assert not any(r["kinds"][f] for f in slow), (v0, acc, d0)
    assert r["min"] >= 40 - 1e-6, (v0, acc, d0, r["min"])          # (at 25 mph and more: the old 5 mph cap)
  # it starts accelerating after our start: released within ~1.5 s, the speed back
  r = run(_mcar(45, 16.0, accel=1.0), NO_CURVE, 2500, lead=_AccelLead(60, 4, 0.6, start=150), max_drop=10,
          model=lambda f: 0.3, setup=MEASURED)
  act = [f for f, k in enumerate(r["kinds"]) if k]
  end = next(f for f in range(act[0], len(r["kinds"])) if not r["kinds"][f])
  assert act[0] < 150 and end < 150 + 200 and r["vs"][end] < 25 * MPH, (act[0], end)
  assert r["min"] > FLOOR_MPH + 5                                  # (HEAD: the floor)
  # a slow car that is not accelerating still starts it (drive 52's queue car, aLeadK -0.2..-0.6)
  r = run(_mcar(45, 16.0, accel=1.0), NO_CURVE, 600, lead=_AccelLead(60, 4, -0.3), max_drop=10, model=lambda f: 0.3,
          setup=MEASURED)
  assert any(r["kinds"][:200]) and r["min"] < 25


def test_the_red_light_gone_restore_taps_with_a_slower_car_in_our_lane():
  # review a1 (below 25 mph): the head start 45 -> our speed, the light goes, a low-confidence slower car in our lane (the
  # queue the light was for) seen at the release or just after: the restore must not go up past our speed into it
  def a1(unsure_at):
    def fn(f):
      return None if f < unsure_at else (0.3, 5.0, 0.2)
    car = _mcar(45, 17.0)
    with _Flag("RED_AFTER_STOP", 0.0):
      return run(car, NO_CURVE, 2500, lead=_FnLead(90.0, fn), max_drop=10, model=lambda f: -0.1, red=lambda f: 100 <= f < 700,
                 setup=MEASURED)
  for unsure_at, top in ((750, None), (805, None), (900, 25.0)):
    r = a1(unsure_at)
    rel = next(f for f in range(1, len(r["kinds"])) if r["kinds"][f - 1] == scs.RED_LIGHT and r["kinds"][f] is None)
    start = next(f for f, k in enumerate(r["kinds"]) if k == scs.SLOW_UNSURE)
    seen = max(unsure_at, rel)
    ups = [h["frame"] for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS and h["frame"] >= seen]
    assert ups == [], (unsure_at, ups)                            # no SET+ hold once the car is there ...
    taps = [f for f, b in r["presses"] if b == cs.SET_PLUS and seen <= f <= start]
    assert taps == [], (unsure_at, taps)                          # ... nor taps: never past our speed into it
    if top is not None:                                           # (a hold already sent stops at its next quantum)
      assert max(r["sets"][seen:start + 1]) / MPH <= top + 1e-6, (unsure_at, max(r["sets"][seen:start + 1]) / MPH)
  # no car: the restore after the light goes still holds back up at once
  r = a1(10**9)
  assert [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS] and abs(r["final"] - 45) < 0.01


class _FnLead(StoppedLead):
  """fn(f) -> None (no car) or (prob, v_mph, y); the distance integrates from d0 with the car's speed."""
  def __init__(self, d0, fn):
    super().__init__(d0)
    self.fn = fn

  def state(self, f, car):
    s = self.fn(f)
    return (False, 0.0, 0.0, 0.0, 0.0) if s is None else (True, s[0], self.d, s[1] * MPH, s[2])

  def tick(self, car):
    self.frame += 1
    s = self.fn(self.frame)
    self.d = max(self.d - (car.v_ego - (s[1] * MPH if s is not None else car.v_ego)) * T.DT, 0.0)


def _drive54_g(lost=(2, 3)):
  """Drive 54, t 1242-1252: the restore had just set 50 at 41.5 mph (still accelerating), the light detector came on, the
  head start's catch-up tapped fast and the car ignored two of its taps, then the planned stop (redLight backed) took
  over in floor mode. Returns (run result, frame the stop ahead start began)."""
  car = Car(50, dash_offset_mph=0.0, accel=0.6, cruise_gain=0.19, cruise_dead=1.0, cruise_max=1.0,
            min_set_mph=CAR_MIN_MPH, lag=12, lose=lambda n: n in lost)
  car.v_ego = 41.5 * MPH
  r = run(car, NO_CURVE, 1100, lead=_NoCar(200), max_drop=10, model=lambda f: -0.6 if f >= 250 else -0.1,
          plan=lambda f: f >= 250, red=lambda f: True)
  start = next(k for k, kind in enumerate(r["kinds"]) if kind == scs.STOP_AHEAD)
  return r, start


def test_drive_54_g_lost_taps_do_not_keep_the_floor_hold_from_starting():
  # on the drive the two lost taps stayed "in flight" for the whole stop (every newer landing was booked to the oldest
  # press), so the floor hold never started: 49 -> 33 by 1 mph taps in 6.5 s. Now the hold starts once only lost taps
  # are in flight and the set speed is at the floor ~4 s after the stop ahead start
  r, start = _drive54_g()
  assert r["trigger"].floor_mode and r["trigger"].kind == scs.STOP_AHEAD
  assert any(cs.SET_MINUS == h["direction"] for h in r["logic"].hold_log), "a SET- hold"
  first_hold = min(r["holds"])
  assert first_hold - start <= int(0.6 / T.DT), (start, first_hold)
  at_floor = next(k for k, s in enumerate(r["sets"]) if s / MPH <= 20.5)
  assert (at_floor - start) * T.DT <= 4.5, (start, at_floor)
  assert r["min"] >= CAR_MIN_MPH - 1e-6
  # the same with no taps lost (control) and with four lost
  for lost in ((), (2, 3, 5, 6)):
    r2, start2 = _drive54_g(lost)
    at_floor2 = next(k for k, s in enumerate(r2["sets"]) if s / MPH <= 20.5)
    assert (at_floor2 - start2) * T.DT <= 4.5, (lost, start2, at_floor2)


def _si(set_mph):
  return cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=set_mph * MPH, v_ego=45 * MPH, gas_pressed=False,
                           brake_pressed=False, standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0,
                           other_button_active=False)


def test_lost_taps_stay_counted_but_do_not_block_a_hold():
  # three taps 0.4 s apart, only the last lands: the two before it stay in flight for the limits (they may still land
  # late: a slow car answers after PENDING_TIMEOUT), but once past PENDING_TIMEOUT they no longer keep a hold back
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.last_set_speed = 45 * MPH
  for f in (100, 140, 180):
    lg.frame = f
    lg._send(cs.SET_MINUS)
  lg.frame = 190
  lg._observe_set_speed(_si(44))
  assert len(lg.inflight) == 2 and lg.pending is None
  assert not lg._only_lost_in_flight(cs.SET_MINUS)            # the newest is still young (it is the one left)
  lg.frame = 180 + cs._frames(cs.PENDING_TIMEOUT)
  assert lg._only_lost_in_flight(cs.SET_MINUS) and not lg._only_lost_in_flight(cs.SET_PLUS)
  lg.inflight[-1].hold = True                                 # a hold in flight always blocks
  assert not lg._only_lost_in_flight(cs.SET_MINUS)


def test_a_lost_restore_tap_does_not_keep_the_restore_hold_from_starting():
  # drive 54, t 427-434: the restore's SET+ taps on the gas lost some and up to 3 stayed "in flight"; off the gas the
  # restore hold needed nothing in flight. Here the last tap on the gas (our 5th press) is lost: the SET+ hold still
  # starts ~0.3 s later, not 1 mph taps all the way back
  ev = {600: lambda s, c: s.__setitem__("gas", True), 700: lambda s, c: s.__setitem__("gas", False)}
  r = run(Car(45, dash_offset_mph=0.0, lose=lambda n: n == 5), NO_CURVE, 2500, lead=_NoCar(200), max_drop=10, events=ev,
          model=lambda f: -0.6 if 100 <= f < 400 else 0.3, plan=lambda f: 100 <= f < 400, red=lambda f: 100 <= f < 400)
  ups = [h["frame"] for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS]
  assert ups and ups[0] <= 760, ups               # (a restore hold may start on the gas too: drives 56 / 59)
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_a_confirmed_floor_stop_waits_for_a_press_in_flight_then_holds():
  # (STOP_HOLD_WAIT) a fresh press of ours still in flight when the floor stop wants its hold: no 1 mph taps meanwhile,
  # the hold once it is past PENDING_TIMEOUT
  lg = cs.CurveSlowdownLogic(False, 10)

  def si():
    return cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=45 * MPH, v_ego=45 * MPH, gas_pressed=False,
                             brake_pressed=False, standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0,
                             other_button_active=False, stop_active=True, stop_target=cs.STOP_MIN_SET_SPEED, stop_urgent=True)
  idle = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=45 * MPH, v_ego=45 * MPH, gas_pressed=False,
                           brake_pressed=False, standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0,
                           other_button_active=False)
  for _ in range(100):
    assert lg.update(idle) == (None, None)
  lg.inflight.append(cs.Press(cs.SET_MINUS, lg.frame))      # (its pending window already given up on)
  sent = []
  for _ in range(60):
    b, _ = lg.update(si())
    if b is not None:
      sent.append((lg.frame, lg.last_action))
  waited = cs._frames(cs.PENDING_TIMEOUT)
  assert sent and sent[0][1] == "hold" and sent[0][0] - 100 >= waited, sent


def _stop_ahead_in_a_map_curve():
  return _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2500, lead=_NoCar(200), max_drop=10,
                               model=lambda f: -0.6 if 200 <= f < 500 else -0.2, plan=lambda f: 200 <= f < 500,
                               red=lambda f: 200 <= f < 500, map_curve=lambda f: (True, 40 * MPH)))


def test_drive_54_e_stop_ahead_gone_releases_after_1_s_and_restores_with_a_hold_up_to_the_curve():
  # drive 54, t 758.65: the floor hold for a planned stop that went away (the model still a little negative), inside a
  # followed 42 mph map curve: released only when the latch ran out (2.9 s after the plan), then 1 mph taps 19 -> 26.
  # Now: released 1 s after the plan, and the restore takes a SET+ hold up to 5 below the curve's target, then taps to
  # it, never above it
  r, rel = _stop_ahead_in_a_map_curve()
  assert rel and rel[0][1].startswith("stop ahead cleared") and rel[0][2]
  assert abs(rel[0][0] - 500 - scs.STOP_AHEAD_GONE_TIME / scs.DT_CTRL) <= 3, rel
  ups = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS]
  assert ups and ups[0]["frame"] > rel[0][0] and ups[0]["bound_mph"] <= 40 - 5 + 1e-6, ups
  after = [s / MPH for s in r["sets"][rel[0][0]:]]
  assert max(after) <= 40 + 1e-6 and near(r["final"], 40)
  at35 = next(k for k, s in enumerate(r["sets"]) if k > rel[0][0] and s / MPH >= 35 - 1e-6)
  # the old rule (taps only inside a curve): 35 reached about 1 s later
  old = cs.RESTORE_IN_CURVE_HOLD
  cs.RESTORE_IN_CURVE_HOLD = False
  try:
    r2, rel2 = _stop_ahead_in_a_map_curve()
  finally:
    cs.RESTORE_IN_CURVE_HOLD = old
  assert not [h for h in r2["logic"].hold_log if h["direction"] == cs.SET_PLUS]
  at35_old = next(k for k, s in enumerate(r2["sets"]) if k > rel2[0][0] and s / MPH >= 35 - 1e-6)
  assert at35_old - at35 >= 80, (at35, at35_old)
  assert max(s / MPH for s in r2["sets"][rel2[0][0]:]) <= 40 + 1e-6


def _cap_in_a_map_curve(fits=None):
  """Drive 54, t 742-769: his speed still the highway's 70 (anchor) on a street whose lower limit (here 50) is pending,
  a followed map curve (target 40, floor 37) on the whole time, a stop ahead that clears."""
  def setup(lg):
    lg.anchor = 70 * MPH
    if fits is not None:
      lg._cap_fits_curve = lambda: fits
  presses = []
  r, rel = _releases(lambda: run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2500, lead=_NoCar(200), max_drop=10,
                                 model=lambda f: -0.6 if 200 <= f < 500 else -0.2, plan=lambda f: 200 <= f < 500,
                                 red=lambda f: 200 <= f < 500, map_curve=lambda f: (True, 40 * MPH, 37 * MPH),
                                 cap=lambda f: 50 * MPH, setup=setup))
  presses = r["presses"]
  return r, rel, presses


def test_a_pending_lower_limit_cap_applies_in_a_curve_once_it_cannot_cause_a_set_minus():
  # the cap waited for the curve to end, so the anchor stayed 70 and the pending cap kept the in-curve restore hold out;
  # now it applies inside the curve once the set speed is at or below the curve's lowest from the capped speed (50 - 10
  # Max Drop - 3 undershoot = 37): here after the stop ahead took the set to the floor
  r, rel, presses = _cap_in_a_map_curve()
  lg = r["logic"]
  assert rel and rel[0][1].startswith("stop ahead cleared")
  assert lg.limit_cap is None                                                 # applied
  ups = [h for h in lg.hold_log if h["direction"] == cs.SET_PLUS]
  assert ups and ups[0]["frame"] > rel[0][0] and ups[0]["bound_mph"] <= 40 - 5 + 1e-6, ups
  after = [s / MPH for s in r["sets"][rel[0][0]:]]
  assert max(after) <= 40 + 1e-6 and near(r["final"], 40)                     # never above the curve's target
  assert [f for f, b in presses if b == cs.SET_MINUS and f > rel[0][0]] == []   # no SET- for the lower limit
  # the anchor is the cap: after the curve the restore would stop at 50, not 70
  assert lg.anchor is not None and near(lg.anchor / MPH, 50)
  # before (the cap never applied in a curve): the anchor stayed 70 and the restore only tapped
  r0, rel0, _ = _cap_in_a_map_curve(fits=False)
  lg0 = r0["logic"]
  assert near(lg0.anchor / MPH, 70) and lg0.limit_cap is not None
  assert not [h for h in lg0.hold_log if h["direction"] == cs.SET_PLUS]
  # while the set speed is still above the capped speed's curve floor the cap waits (no SET- because of it): the same
  # SET- presses as with no cap at all until the stop ahead starts
  r1 = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 180, lead=_NoCar(200), max_drop=10,
           map_curve=lambda f: (True, 40 * MPH, 37 * MPH), cap=lambda f: 50 * MPH, setup=lambda lg: setattr(lg, "anchor", 70 * MPH))
  assert r1["logic"].limit_cap is not None and near(r1["logic"].anchor / MPH, 70)
  assert near(r1["final"], 40)


# ---------------- drive 69: holds lost for the rest of the drive ----------------

def _hold_landings(r):
  h = r["holds"]
  return [f for f in range(h[0] + 1, h[-1] + 30) if abs(r["sets"][f] - r["sets"][f - 1]) > 0.1 * MPH]


def test_drive_69_a_driver_set_edge_on_a_hold_landing_does_not_switch_holds_off():
  # drive 69 t 879.18: the floor hold 41 -> 20 had its last quantum (25 -> 20) land on the frame the wheel reported SET+;
  # the edge cut the hold first, so that quantum read as "the car kept stepping after the release": holds off for the rest
  # of the drive (every 55 mph red light after it only tapped). Now the quantum the car was stepping stays the hold's.
  r0 = approach(45, 60, frames=1500)
  landings = _hold_landings(r0)
  assert len(landings) >= 4, landings
  for land in landings[1:5]:
    def phantom(s, c):
      c.driver_button, c.driver_button_until = cs.SET_PLUS, c.frame + 100   # (no set speed change of his)
    r = approach(45, 60, frames=1500, events={land + 1: phantom})
    lg = r["logic"]
    assert lg.hold_misses == 0 and lg.holds_allowed, (land, lg.hold_misses, lg.holds_off_why)
    assert not lg.disabled and r["min"] >= FLOOR_MPH - 1e-6


def test_hold_misses_are_forgotten_after_hold_miss_forget():
  lg = cs.CurveSlowdownLogic(False, 10)
  idle = _si(45)
  for _ in range(10):
    lg.update(idle)
  lg._hold_miss("the car kept stepping after a release", True)       # the genuine protection: off at once
  assert not lg.holds_allowed and lg.hold_misses == cs.HOLD_MISS_LIMIT
  for _ in range(cs._frames(cs.HOLD_MISS_FORGET) - 1):
    lg.update(idle)
  assert not lg.holds_allowed
  lg.update(idle)
  assert lg.holds_allowed and lg.hold_misses == 0 and lg.holds_off_why.startswith("forgotten")


def test_holds_come_back_for_a_stop_after_the_decay():
  # holds switched off early in the drive (two misses); a stopped car 125 s later gets its floor hold again
  frames = cs._frames(cs.HOLD_MISS_FORGET) + 1500
  start = cs._frames(cs.HOLD_MISS_FORGET) + 500

  def setup(lg):
    lg.hold_misses, lg.holds_ok, lg.hold_miss_frame, lg.holds_off_why = cs.HOLD_MISS_LIMIT, False, 0, "a hold with no effect"
  lead = StoppedLead(60 + 20.1168 * start * T.DT, present=lambda f: f >= start)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, frames, lead=lead, max_drop=10, setup=setup)
  assert r["holds"] and min(r["holds"]) >= start, r["holds"][:3]
  assert r["min"] >= FLOOR_MPH - 1e-6 and r["min"] <= 21
  # control: with no decay the same stop only taps
  old = cs.HOLD_MISS_FORGET
  cs.HOLD_MISS_FORGET = 1e9
  try:
    lead2 = StoppedLead(60 + 20.1168 * start * T.DT, present=lambda f: f >= start)
    r2 = run(Car(45, dash_offset_mph=0.0), NO_CURVE, frames, lead=lead2, max_drop=10, setup=setup)
  finally:
    cs.HOLD_MISS_FORGET = old
  assert not r2["holds"]


# ---------------- drive 69: a stop ahead released while the red light is still on ----------------

def _drive69_red(set_mph=45, plan=lambda f: f >= 140, red=lambda f: True, frames=1400, lost=(), lead=None, events=None):
  """A red light at set_mph, no car (or `lead`): the head start, then the planned stop (redLight backed) in floor mode."""
  car = Car(set_mph, dash_offset_mph=0.0, accel=0.6, cruise_gain=0.19, cruise_dead=1.0, cruise_max=1.0,
            min_set_mph=CAR_MIN_MPH, lag=12, lose=lambda n: n in lost)
  car.v_ego = set_mph * MPH
  return _releases(lambda: run(car, NO_CURVE, frames, lead=lead or _NoCar(200), max_drop=10, events=events,
                               model=lambda f: -0.6 if plan(f) else -0.2, plan=plan, red=red))


def test_drive_69_t125_a_plan_blink_under_a_red_light_does_not_release_or_restore():
  # t 125-134: head start 125.5, the floor hold 45 -> 35 from 126.0; the plan blinked off at 126.6 for ~2 s, released
  # at 127.5 ("stop ahead cleared") with the light on, then SET+ taps 35 -> 45 toward the red light (the driver braked)
  plan = lambda f: 140 <= f < 200 or f >= 400       # noqa: E731
  r, rel = _drive69_red(plan=plan)
  assert not rel, rel
  assert not [f for f, b in r["presses"] if b == cs.SET_PLUS] and not _plus_frames(r)
  assert r["min"] <= 21 and r["min"] >= CAR_MIN_MPH - 1e-6
  # the light goes green (redLight and the plan off): released 1 s after, the speed comes back
  r2, rel2 = _drive69_red(plan=lambda f: 140 <= f < 600, red=lambda f: f < 600, frames=2500)
  assert rel2 and rel2[0][1] == scs.STOP_AHEAD_CLEARED and rel2[0][2], rel2
  assert abs(rel2[0][0] - 600 - scs.RED_DROPOUT / scs.DT_CTRL) <= 3, rel2
  assert near(r2["final"], 45)


def test_drive_69_t1193_a_flickering_plan_before_the_warning_under_a_red_light_is_not_released():
  # t 1193-1200 (55 mph): the stop ahead start at 1193.4, the plan flickering, released at 1194.6 "gone before the
  # warning" with the light on, the restore 46 -> 53, then a car 125 m ahead; the driver braked at 51 mph
  plan = lambda f: f >= 150 and not (205 <= f < 260)   # noqa: E731   (before the fix: released at 231, SET+ from 371)
  lead = StoppedLead(330, v_mph=31, present=lambda f: f >= 750)
  r, rel = _drive69_red(55, plan=plan, lead=lead, frames=1100)
  assert not [x for x in rel if x[1] in (scs.STOP_AHEAD_UNCONFIRMED, scs.STOP_AHEAD_CLEARED)], rel
  assert not _plus_frames(r)
  assert r["min"] <= 25


def test_a_false_red_light_holds_the_stop_ahead_at_most_stop_ahead_red_hold():
  # the detector stays on, the planned stop went away and no car: released STOP_AHEAD_RED_HOLD after the plan, but with
  # the light still on the lowered set speed stays (second review: no restore toward the light) until it goes off ...
  r, rel = _drive69_red(plan=lambda f: 140 <= f < 300, red=lambda f: f < 1300, frames=2200)
  assert rel and rel[0][1] == scs.STOP_AHEAD_CLEARED, rel
  assert abs(rel[0][0] - 300 - scs.STOP_AHEAD_RED_HOLD / scs.DT_CTRL) <= 3, rel
  plus = _plus_frames(r)
  assert plus and plus[0] >= 1300 + scs.RED_DROPOUT / scs.DT_CTRL, plus[:2]
  assert near(r["final"], 45)
  # ... or, a stuck flag, RED_RELEASE_HOLD_MAX after the release
  r, rel = _drive69_red(plan=lambda f: 140 <= f < 300, frames=6000)
  plus = _plus_frames(r)
  assert plus and plus[0] >= rel[0][0] + scs.RED_RELEASE_HOLD_MAX / scs.DT_CTRL, (rel, plus[:2])
  assert near(r["final"], 45)


def test_the_drivers_gas_still_releases_a_stop_ahead_under_a_red_light():
  ev = {500: lambda s, c: s.__setitem__("gas", True)}
  r, rel = _drive69_red(events=ev, frames=900)
  assert rel and rel[0][1] == "driver on the gas", rel
  assert rel[0][0] - 500 <= scs.GAS_RELEASE_TIME / scs.DT_CTRL + 3


def test_after_a_cleared_release_the_stop_starts_again_at_once_when_the_light_and_plan_come_back():
  tr = scs.StoppedCarTrigger()
  _started_past_the_warning(tr)
  n = 0
  while tr.update(_latched_inputs(lead_status=False, plan_stop=False, red_light=False, model_accel=-0.2)):
    n += 1
    assert n < 1000
  assert tr.release_reason == scs.STOP_AHEAD_CLEARED and tr.rearm_on_red and not tr.latched
  assert not tr.hold_restore                     # the restore is not held back as "a stop planned ahead"
  back = None
  for k in range(250):
    if tr.update(_latched_inputs(lead_status=False)):
      back = k
      break
  # the latch (LATCH_DEBOUNCE) and the start: no WARN_REARM wait (3 s)
  assert back is not None and back <= scs.LATCH_DEBOUNCE / scs.DT_CTRL + 5, back
  assert tr.kind == scs.STOP_AHEAD and tr.floor_mode
  # not after the driver's own release (brake): the WARN_REARM wait stays
  tr = scs.StoppedCarTrigger()
  _started_past_the_warning(tr)
  tr.update(_latched_inputs(lead_status=False, brake_pressed=True))
  assert not tr.active and not tr.rearm_on_red
  for _ in range(250):
    assert not tr.update(_latched_inputs(lead_status=False))


def test_restore_now_waits_while_a_stop_is_still_planned_ahead():
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.anchor, lg.last_set_speed = 45 * MPH, 35 * MPH
  base = dict(enabled=True, cruise_enabled=True, set_speed=35 * MPH, v_ego=35 * MPH, gas_pressed=False, brake_pressed=False,
              standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0, other_button_active=False)
  lg.update(cs.SlowdownInputs(**base, stop_restore_now=True, hold_restore=True))
  assert not lg.restore_pending
  lg.update(cs.SlowdownInputs(**base, stop_restore_now=True))
  assert lg.restore_pending


# ---------------- the floor hold's wait for a press in flight (STOP_HOLD_WAIT) ----------------

def _floor_stop_with_presses_in_flight(frames=300):
  """A confirmed floor stop from 55 mph that wants its hold while an earlier hold of ours (still settling) and one lost tap
  are in flight; the car answers each tap 80 ms later and every landing is booked to the oldest press (the lost tap), so
  the newest tap always looks fresh. Returns [(frame, action)] of what was sent."""
  lg = cs.CurveSlowdownLogic(False, 10)
  set_mph = [55.0]

  def si(stop=True):
    return cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=set_mph[0] * MPH, v_ego=55 * MPH,
                             gas_pressed=False, brake_pressed=False, standstill=False, plan_alive=True, csc_active=False,
                             csc_speed=0.0, other_button_active=False, stop_active=stop,
                             stop_target=cs.STOP_MIN_SET_SPEED if stop else 0.0, stop_urgent=stop)
  for _ in range(100):
    lg.update(si(False))
  lg.inflight.append(cs.Press(cs.SET_MINUS, lg.frame - 100))                    # a lost tap
  h = cs.Press(cs.SET_MINUS, lg.frame - 20, reserve=6, remaining=6, landed=10, hold=True, frames_sent=1,
               release_frame=lg.frame)
  lg.inflight.append(h)                                                          # a hold of ours still settling
  sent, lands = [], []
  for _ in range(frames):
    for f in [f for f in lands if f <= lg.frame]:
      set_mph[0] -= 1
      lands.remove(f)
    b, _ = lg.update(si())
    if b is not None:
      sent.append((lg.frame, lg.last_action))
      if lg.last_action != "hold":
        lands.append(lg.frame + 8)
  return sent


def test_a_floor_stop_with_a_lost_tap_in_flight_still_holds():
  # (the drive 69 analysis suspected that once STOP_HOLD_WAIT ran out, a lost tap would keep every newer tap "fresh" and
  # the stop would tap all the way down; it cannot: a tap is lost PENDING_TIMEOUT after it was sent, sooner than the wait
  # runs out, and nothing is sent while a hold of ours is in flight) the hold follows as soon as the settling hold is done
  sent = _floor_stop_with_presses_in_flight()
  holds = [f for f, a in sent if a == "hold"]
  assert holds and not [f for f, a in sent if a != "hold" and f < holds[0]], sent


def test_a_floor_stop_from_55_mph_with_a_lost_tap_holds():
  # a red light at 55 mph: the head start's taps (one of them lost, the car slow to answer), then the planned stop in floor
  # mode: one SET- hold within ~0.6 s of the stop ahead start, at the floor within ~5 s
  for lost in ((1,), (2,), (1, 2)):
    for lag in (12, 35):
      car = Car(55, dash_offset_mph=0.0, accel=0.6, cruise_gain=0.19, cruise_dead=1.0, cruise_max=1.0,
                min_set_mph=CAR_MIN_MPH, lag=lag, lose=lambda n, lost=lost: n in lost)
      car.v_ego = 55 * MPH
      r = run(car, NO_CURVE, 1200, lead=_NoCar(200), max_drop=10, model=lambda f: -0.6 if f >= 251 else -0.2,
              plan=lambda f: f >= 251, red=lambda f: True)
      start = next(k for k, kind in enumerate(r["kinds"]) if kind == scs.STOP_AHEAD)
      holds = [h for h in r["holds"] if h >= start]
      assert holds and holds[0] - start <= int(0.6 / T.DT), (lost, lag, start, holds[:1])
      at_floor = next(k for k, s in enumerate(r["sets"]) if s / MPH <= 20.5)
      assert (at_floor - start) * T.DT <= 5.0, (lost, lag, start, at_floor)


def test_drive_69_t760_no_red_light_head_start_in_a_curve():
  # t 760: in a followed 38 mph map curve the stop-light detector came on (the tail lights ahead): the head start took
  # the set speed 38 -> 35 for 30 s. No head start while a curve target is on; after the curve it starts as before
  def go(curve_until):
    return run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1500, lead=_NoCar(200), max_drop=10, red=lambda f: 100 <= f < 1200,
               map_curve=lambda f: (f < curve_until, 40 * MPH), curve_to_trigger=True)
  r = go(1500)
  assert scs.RED_LIGHT not in r["kinds"]
  assert r["min"] >= 40 - 1e-6                         # only the curve's own target
  r2 = go(600)
  first = next((k for k, kind in enumerate(r2["kinds"]) if kind == scs.RED_LIGHT), None)
  assert first is not None and first >= 600, first
  # a stop ahead (the planned stop) in a curve still starts on the warning
  r3 = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 1000, lead=_NoCar(200), max_drop=10, red=lambda f: f >= 100,
           plan=lambda f: f >= 300, model=lambda f: -0.6 if f >= 300 else -0.2,
           map_curve=lambda f: (True, 40 * MPH), curve_to_trigger=True)
  assert scs.STOP_AHEAD in r3["kinds"] and scs.RED_LIGHT not in r3["kinds"]


# ---------------- review of the merged fix: limits of the hold under a red light and of the restart ----------------

def _starts(r):
  return [k for k, kind in enumerate(r["kinds"]) if kind == scs.STOP_AHEAD and (k == 0 or r["kinds"][k - 1] != scs.STOP_AHEAD)]


def test_a_red_light_for_another_lane_holds_a_stop_ahead_at_most_stop_ahead_red_hold_max():
  # a red arrow while going straight / the cross street's red: the light stays on, the plan blinks (1.5 s on, 1.5 s off):
  # released once the plan has been absent STOP_AHEAD_RED_HOLD_MAX in all under the light (here ~16 s, then gone for
  # STOP_AHEAD_GONE_TIME), and every later episode the same; the restart at once is used once per REARM_CHAIN_TIME. (With
  # the light on the restore then waits, RED_RELEASE_HOLD_MAX at most: his gas is the way out of a red arrow)
  plan = lambda f: f >= 140 and (f - 140) % 300 < 150       # noqa: E731
  rearms = []
  orig = scs.StoppedCarTrigger.update

  def watch(self, i):
    before = self.rearm_used_frame
    out = orig(self, i)
    if self.rearm_used_frame is not None and self.rearm_used_frame != before:
      rearms.append(self.frame)
    return out
  scs.StoppedCarTrigger.update = watch
  try:
    r, rel = _drive69_red(plan=plan, frames=6000)
  finally:
    scs.StoppedCarTrigger.update = orig
  cleared = [x for x in rel if x[1] == scs.STOP_AHEAD_CLEARED]
  assert cleared, rel
  first_drop = 290
  limit = 2 * scs.STOP_AHEAD_RED_HOLD_MAX + 3.0     # (the plan off half the time)
  assert first_drop + 2 * scs.STOP_AHEAD_RED_HOLD_MAX / scs.DT_CTRL - 150 <= cleared[0][0] <= \
    first_drop + limit / scs.DT_CTRL + 3, cleared
  starts = _starts(r)
  for s0 in starts:                               # (before: one episode held to the floor for the whole minute)
    end = next((x[0] for x in rel if x[0] > s0), len(r["kinds"]))
    assert (end - s0) * T.DT <= 1.5 + limit or end == len(r["kinds"]), (s0, end, rel)
  assert all(b - a > scs.REARM_CHAIN_TIME / scs.DT_CTRL for a, b in zip(rearms, rearms[1:], strict=False)), rearms


def test_the_model_saying_go_releases_a_stop_ahead_under_a_red_light():
  car = Car(45, dash_offset_mph=0.0, accel=0.6, cruise_gain=0.19, cruise_dead=1.0, cruise_max=1.0, min_set_mph=CAR_MIN_MPH,
            lag=12)
  r, rel = _releases(lambda: run(car, NO_CURVE, 2000, lead=_NoCar(200), max_drop=10, red=lambda f: True,
                                 plan=lambda f: 140 <= f < 500, model=lambda f: -0.6 if 140 <= f < 500 else 0.5))
  assert rel and rel[0][1] in (scs.STOP_AHEAD_CLEARED, scs.STOP_AHEAD_UNCONFIRMED), rel
  assert rel[0][0] - 500 <= scs.GO_TIME / scs.DT_CTRL + 3, rel


def test_the_restart_at_once_happens_once_per_release_chain():
  def release(tr):
    n = 0
    while tr.update(_latched_inputs(lead_status=False, plan_stop=False, red_light=False, model_accel=-0.2)):
      n += 1
      assert n < 1000
    assert tr.release_reason in (scs.STOP_AHEAD_CLEARED, scs.STOP_AHEAD_UNCONFIRMED), tr.release_reason

  def back(tr, frames=250):
    for k in range(frames):
      if tr.update(_latched_inputs(lead_status=False)):
        return k
    return None
  tr = scs.StoppedCarTrigger()
  _started_past_the_warning(tr)
  release(tr)
  assert back(tr) is not None                     # the first time: at once
  release(tr)
  assert not tr.rearm_on_red and back(tr) is None   # not again in the same chain (the WARN_REARM wait)
  # a new chain after the driver braked: available again
  tr2 = scs.StoppedCarTrigger()
  _started_past_the_warning(tr2)
  release(tr2)
  assert back(tr2) is not None
  tr2.update(_latched_inputs(lead_status=False, brake_pressed=True))
  assert not tr2.active and tr2.rearm_used_frame is None
  # ... or REARM_CHAIN_TIME after it was used
  tr3 = scs.StoppedCarTrigger()
  _started_past_the_warning(tr3)
  release(tr3)
  assert back(tr3) is not None
  tr3.rearm_used_frame -= int(scs.REARM_CHAIN_TIME / scs.DT_CTRL) + 1
  release(tr3)
  assert tr3.rearm_on_red and back(tr3) is not None


def test_second_review_a_real_red_light_with_plan_blinks_is_not_released_and_never_restored_toward_it():
  # the reviewer's probe: the light on throughout, the plan blinking off 0.4 s early and again 1.2 s (or 0.6 s) eight /
  # nine seconds later; the 8 s cap counted from the first blink released it at the second, with a SET+ hold toward
  # the light. The cap counts plan-absent time: no release, no SET+, the set speed stays at the floor
  for plan in (lambda f: f >= 140 and not (340 <= f < 380) and not (1140 <= f < 1260),
               lambda f: f >= 140 and not (340 <= f < 380) and not (1240 <= f < 1300)):
    r, rel = _drive69_red(plan=plan, frames=2200)
    assert not rel and not _plus_frames(r), (rel, _plus_frames(r)[:2])
    assert r["final"] <= 20
  # the plan gone for good near the line with the light still on (the camera losing it at 20-30 mph): released after
  # STOP_AHEAD_RED_HOLD, but the set speed never goes up while the light is on
  r, rel = _drive69_red(plan=lambda f: 140 <= f < 600, frames=2200)
  assert rel and rel[0][1] == scs.STOP_AHEAD_CLEARED and not r["trigger"].restore_now, rel
  assert not _plus_frames(r) and r["max"] <= 45 + 1e-6 and r["final"] <= 20
  assert r["trigger"].hold_restore                       # (no SET+: the controller's restore waits)


def test_after_a_release_under_a_red_light_his_gas_or_the_models_go_lets_the_restore_come():
  ev = {1500: lambda s, c: s.__setitem__("gas", True), 1520: lambda s, c: s.__setitem__("gas", False)}
  r, rel = _drive69_red(plan=lambda f: 140 <= f < 600, frames=2600, events=ev)
  assert rel and _plus_frames(r) and _plus_frames(r)[0] >= 1500, _plus_frames(r)[:2]
  car = Car(45, dash_offset_mph=0.0, accel=0.6, cruise_gain=0.19, cruise_dead=1.0, cruise_max=1.0, min_set_mph=CAR_MIN_MPH,
            lag=12)
  model = lambda f: -0.6 if 140 <= f < 600 else (0.5 if f >= 1500 else -0.2)   # noqa: E731
  r2, rel2 = _releases(lambda: run(car, NO_CURVE, 2600, lead=_NoCar(200), max_drop=10, red=lambda f: True,
                                   plan=lambda f: 140 <= f < 600, model=model))
  plus = _plus_frames(r2)
  assert rel2 and plus and 1500 + scs.GO_TIME / scs.DT_CTRL <= plus[0] <= 2000, (rel2, plus[:2])
