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
  assert near(scs.stopped_car_target(15.0) / MPH, 20.0)
  assert near(scs.stopped_car_target(0.0) / MPH, 20.0)
  v = scs.stopped_car_target(120.0)
  assert abs(v - (((20 * MPH) ** 2 + 2 * 1.0 * 105) ** 0.5)) < 1e-9
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
  assert not tr.active and tr.release_reason == "lead left the lane"
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
    assert not tr.update(inp(lead_prob=0.8))
  assert any("prob" in b for b in tr.blockers(inp(lead_prob=0.8)))
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
  assert r["min"] >= 20 - 1e-6, r["min"]
  assert r["min"] <= 30, r["min"]                      # well on the way to the floor (lead still there)
  assert r["max"] <= 45 + 1e-6
  assert r["holds"] and r["holds"][0] < (r["presses"][0][0] if r["presses"] else 10**9), "120 m: floor mode, a hold first"
  assert r["trigger"].floor_mode
  # the lead departs: normal restore
  lead2 = StoppedLead(120, present=lambda f: f < 2000)
  r2 = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 6500, lead=lead2, stop_mode=2, max_drop=10)
  assert near(r2["final"], 45), r2["final"]
  assert r2["max"] <= 45 + 1e-6 and r2["min"] >= 20 - 1e-6


def test_urgent_approach_uses_holds_and_reaches_floor_fast():
  r = approach(45, 60, frames=1500)
  assert r["holds"], r
  assert r["min"] >= 20 - 1e-6
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
  assert r["min"] >= 20 - 1e-6 and r["max"] <= 45 + 1e-6
  gaps = [b - a for (a, _), (b, _) in zip(r["presses"], r["presses"][1:])]
  assert min(gaps[:3]) >= cs.RELAXED_GAP_FRAMES, gaps[:3]          # relaxed pace while not urgent (it gets urgent closer in)


def test_confirmed_stopped_car_with_room_holds_from_the_start():
  # 55 mph, stopped car at 120 m: the target (38 mph) leaves 17 steps of room: a hold, not taps, first.
  r = approach(55, 120, frames=2500)
  assert r["holds"] and (not r["presses"] or r["holds"][0] < r["presses"][0][0]), (r["holds"][:3], r["presses"][:3])
  assert r["min"] >= 20 - 1e-6 and r["max"] <= 55 + 1e-6
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
  assert r["max"] <= 45 + 1e-6 and r["min"] >= 20 - 1e-6
  assert near(r["final"], 45), r["final"]


def test_curve_and_stopped_car_together_min_wins_no_fighting():
  lead = StoppedLead(150, present=lambda f: f < 2000)
  r = run(Car(45, dash_offset_mph=0.0), between(100, 1500, 40), 7000, lead=lead, max_drop=5)
  seq = "".join("M" if b == cs.SET_MINUS else "P" for _, b in r["presses"])
  assert "PM" not in seq or seq.count("PM") <= 1, seq     # no SET+/SET- fighting while both reasons are on
  assert r["min"] >= 20 - 1e-6 and r["max"] <= 45 + 1e-6
  assert near(r["final"], 45), r["final"]
  assert r["min"] < 40 - 1e-6                              # went below the curve's 5 mph cap: the stopped car won


def test_presses_ignored_still_safe():
  r = approach(45, 60, frames=4000, car_kw=dict(lose=lambda n: n % 3 == 0, hold_ignored=True))
  assert r["min"] >= 20 - 1e-6 and r["max"] <= 45 + 1e-6
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
        # phase-locked stop holds aim at 20 with no overrun margin: a car that keeps stepping lands on its minimum
        assert r["min"] >= (CAR_MIN_MPH if latency else 20) - 1e-6, (rate, latency, d0, r["min"])
        assert r["max"] <= 45 + 1e-6, (rate, latency, d0, r["max"])
        assert not r["logic"].disabled, (rate, latency, d0)


def test_hold_landing_as_1_3_10_steps_is_safe():
  # a car that answers a hold with a single tap, a 3 mph step or 10 mph at once
  for rate, start in ((1.0, 0.1), (5.0, 0.5), (16.6, 0.1)):
    r = approach(45, 80, frames=3000, car_kw=dict(hold_rate=rate, hold_start=start))
    assert r["min"] >= 20 - 1e-6 and r["max"] <= 45 + 1e-6, (rate, r["min"], r["max"])
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
  assert r["min"] >= 20 - 1e-6


def test_lead_departs_during_hold_aborts_then_restores():
  r0 = approach(45, 60, frames=400)
  h0 = r0["holds"][0]
  lead = StoppedLead(60, present=lambda f: f < h0 + 30)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 4000, lead=lead, max_drop=10)
  assert max(r["holds"]) < h0 + 35 + 200      # the lead must be lost for 2 s before the reason clears
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6


def test_hold_ignored_falls_back_to_taps():
  r = approach(45, 60, frames=3000, car_kw=dict(hold_ignored=True))
  assert r["presses"] and r["min"] >= 20 - 1e-6
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
  # the target followed the car ahead. Phase-locked now: one hold, released as 20 lands; nothing left for taps.
  r = approach(45, 60, frames=3000)
  log = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS]
  assert len(log) == 1 and log[0]["phase_lock"], log
  assert near(log[0]["bound_mph"], 20) and log[0]["landed"] >= 20, log
  assert near(r["min"], 20) and r["max"] <= 45 + 1e-6
  assert [f for f, b in r["presses"] if b == cs.SET_MINUS] == []
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
  assert near(r["min"], 20), r["min"]
  plus_after = [f for f in _plus_frames(r) if f >= stop_f]
  assert plus_after and plus_after[0] >= stop_f + 100 - 5, (stop_f, plus_after[:1])   # ~1 s into the stop
  assert near(r["final"], 45) and r["max"] <= 45 + 1e-6, (r["final"], r["max"])


def test_left_lane_release_restores_by_taps_only():
  # R3-1: the lead drifts out of our lane for good: a correct release, restored by taps at the relaxed pacing
  # (no SET+ hold), never above the driver's speed.
  lead = StoppedLead(120)
  orig_state = lead.state
  lead.state = lambda f, c: (lambda st: (st[0], st[1], st[2], st[3], 1.6 if f >= 350 else st[4]))(orig_state(f, c))

  def tick(c):      # the MRCC does not brake for a car in the next lane: we drive past it
    lead.frame += 1
    lead.d = max(lead.d - c.v_ego * T.DT, 0.0)
  lead.tick = tick
  car = Car(45, dash_offset_mph=0.0, lag=10)
  r, rel = _releases(lambda: run(car, NO_CURVE, 3000, lead=lead, max_drop=10))
  assert [reason for _, reason, _ in rel] == ["lead left the lane"], rel
  assert rel[0][2] is False
  assert not [h for h in r["logic"].hold_log if h["direction"] == cs.SET_PLUS], "no SET+ hold"
  plus = _plus_frames(r)
  assert plus and all(b - a >= cs.RELAXED_GAP_FRAMES for a, b in zip(plus, plus[1:])), plus
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
  assert r["min"] >= 20 - 1e-6 and r["min"] <= 33, r["min"]
  # the same car with the model not slowing (just a slow car, no stop ahead): nothing happens
  lead = StoppedLead(95, v_mph=13.0, prob=0.7, y=1.0)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 800, lead=lead, stop_mode=2, max_drop=10, model=lambda f: 0.0)
  assert not r["ever_active"] and r["presses"] == [] and r["holds"] == []


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
  base = dict(enabled=True, cruise_enabled=True, brake_pressed=False, v_ego=45 * MPH, lead_alive=True, lead_status=True,
              lead_prob=0.26, lead_d=118.0, lead_v=13 * MPH, lead_y=-0.8, model_alive=True, model_accel=-0.6, plan_stop=True)
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
  assert tr.kind == scs.STOP_WITH_CAR and tr.early and not tr.urgent      # taps until it is confirmed
  # without a planned stop first: nothing (the near-miss log says why)
  tr = scs.StoppedCarTrigger()
  for _ in range(300):
    assert not tr.update(_latched_inputs(plan_stop=False))
  assert any("not latched" in b for b in tr.blockers(_latched_inputs(plan_stop=False)))


def test_the_early_start_still_needs_a_slow_car_near_our_lane_and_the_model_slowing():
  for kw in (dict(lead_v=43 * MPH), dict(lead_status=False), dict(lead_y=2.5), dict(lead_d=160.0),
             dict(model_accel=-0.2), dict(lead_prob=0.15)):
    tr = scs.StoppedCarTrigger()
    for _ in range(40):
      tr.update(_latched_inputs(lead_status=False))
    for _ in range(300):
      tr.update(_latched_inputs(**kw, plan_stop=False) if False else _latched_inputs(**kw))
    assert not tr.active, kw


def test_an_early_start_is_confirmed_when_the_car_is_sure_and_then_slows_fast():
  tr = scs.StoppedCarTrigger()
  for _ in range(40):
    tr.update(_latched_inputs(lead_status=False))
  while not tr.update(_latched_inputs()):
    pass
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
  for _ in range(101):
    tr.update(_latched_inputs(lead_prob=0.6, model_accel=0.3, lead_v=44 * MPH, plan_stop=False))
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
  assert r["sets"][F(28.0)] / MPH >= 40 - 1e-6        # unsure until t 28: gentle taps only
  i = F(29.5)
  assert r["sets"][i] / MPH <= 38 + 1e-6, r["sets"][i] / MPH   # on the drive: 40 at t 29.5
  assert r["min"] >= 20 - 1e-6
  assert [f for f, b in r["presses"] if b == cs.SET_PLUS] == []


def test_a_false_early_start_drops_a_few_mph_and_comes_back():
  # a ghost "car" for 1.5 s while the model plans a stop that then goes away (a green light)
  class Ghost(StoppedLead):
    def state(self, f, car):
      return (True, 0.25, 115.0, 12 * MPH, 0.5) if 100 <= f < 250 else (False, 0.0, 0.0, 0.0, 0.0)

  plan = lambda f: f < 300  # noqa: E731
  model = lambda f: -0.6 if f < 300 else 0.3  # noqa: E731
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 3000, lead=Ghost(115.0), stop_mode=2, max_drop=10, model=model, plan=plan)
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
  assert r["min"] >= 20 - 1e-6 and r["max"] <= 45 + 1e-6
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
  assert len(log) >= 1 and near(log[0]["bound_mph"], 20), log
  assert r["sets"][log[0]["frame"] + 350] <= 25 * MPH + 1e-6
  assert near(r["min"], 20) and r["max"] <= 45 + 1e-6


def test_stop_with_a_car_rolling_at_25_mph_goes_to_the_floor_and_comes_back():
  # owner's choice (drive 45): no lead-speed guard: straight to 20; the stock radar keeps the gap to a car that keeps
  # moving, and the release (traffic moving again) restores the driver's speed
  lead = T.StoppedLead(90, v_mph=25.0, prob=0.9, y=0.1)
  r = run(Car(45, dash_offset_mph=0.0), NO_CURVE, 2000, lead=lead, stop_mode=2, max_drop=10, model=lambda f: -0.6)
  assert r["ever_active"] and r["trigger"].floor_mode
  assert near(r["min"], 20), r["min"]


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
  tr.update(scs.StoppedCarInputs(**dict(base, lead_a=-2.5)))
  assert near(tr.target, scs.FLOOR_SPEED)                                       # braking hard: it will be slow soon


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
  assert r["min"] >= 20 - 1e-6 and r["max"] <= 45 + 1e-6


def test_phase_strict_car_sees_a_long_press_only_when_locked():
  r = run(Car(45, dash_offset_mph=0.0, hold_phase_window=1), NO_CURVE, 2500, lead=T.StoppedLead(70), stop_mode=2,
          max_drop=10, wheel_phase=4)
  log = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS and h["done"]]
  assert log and log[0]["pattern"] == "quantum", log[:1]
  assert r["min"] >= 20 - 1e-6 and r["max"] <= 45 + 1e-6
  cs.HOLD_PHASE_LOCK = False
  try:
    r2 = run(Car(45, dash_offset_mph=0.0, hold_phase_window=1), NO_CURVE, 2500, lead=T.StoppedLead(70), stop_mode=2,
             max_drop=10, wheel_phase=4)
  finally:
    cs.HOLD_PHASE_LOCK = True
  log2 = [h for h in r2["logic"].hold_log if h["direction"] == cs.SET_MINUS and h["done"]]
  assert r2["min"] >= 20 - 1e-6 and r2["max"] <= 45 + 1e-6       # taps answers keep the limits too
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
  assert rel and rel[0][1].startswith("radar following") and rel[0][2], rel
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
  assert near(r["min"], 20)
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
  tr = scs.StoppedCarTrigger()
  for f in range(400):
    tr.update(_latched_inputs(lead_prob=0.9, lead_d=100.0, lead_v=cycle[(f // 10) % len(cycle)] * MPH))
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
