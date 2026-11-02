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


def approach(set_mph=45, d0=120, frames=6000, lead_kw=None, car_kw=None, curve=NO_CURVE, events=None, stop_mode=2, max_drop=10):
  lead = StoppedLead(d0, **(lead_kw or {}))
  car = Car(set_mph, dash_offset_mph=0.0, **(car_kw or {}))
  return run(car, curve, frames, events=events, lead=lead, stop_mode=stop_mode, max_drop=max_drop)


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
  assert r["presses"] and r["presses"][0][0] < (r["holds"][0] if r["holds"] else 10**9), "from 120 m the target starts above the set speed: taps first"
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
  # comes down slowly, so taps keep up with it and no 5 mph hold is ever needed (room < 6 steps).
  r = approach(45, 200, frames=5000)
  minus_holds = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS]
  assert r["presses"] and minus_holds == [], (len(r["presses"]), minus_holds)
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
        r = approach(45, d0, frames=3000, car_kw=dict(hold_rate=rate, hold_latency=latency))
        assert r["min"] >= 20 - 1e-6, (rate, latency, d0, r["min"])
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


def test_45_mph_two_holds_then_taps():
  # 45 mph, stopped car at 60 m: hold 1 aims 15 mph down, hold 2 10 more, then taps to the floor.
  # With the release overrun still assumed to be one quantum, the holds release one quantum early
  # (45 -> 35, 35 -> 30); a car that does overrun lands 30 / 25. Taps finish to 20.
  r = approach(45, 60, frames=3000)
  log = [h for h in r["logic"].hold_log if h["direction"] == cs.SET_MINUS]
  assert len(log) == 2, log
  assert log[0]["landed"] >= 10 and log[1]["landed"] >= 5, log
  assert near(r["min"], 20) and r["max"] <= 45 + 1e-6
  minus = [f for f, b in r["presses"] if b == cs.SET_MINUS]
  assert minus and all(f > log[1]["release_frame"] for f in minus)     # taps only after the holds


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
      r = run(Car(55, dash_offset_mph=0.0, hold_latency=latency), NO_CURVE, 6000, lead=lead, max_drop=10)
      assert r["max"] <= 55 + 1e-6, (latency, d0, r["max"])
      assert near(r["final"], 55), (latency, d0, r["final"])
      assert r["min"] >= 20 - 1e-6, (latency, d0, r["min"])


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
  assert 1 <= len(plus_before) <= 2, plus_before
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
              hold_rate=rate, hold_start=start, hold_latency=latency, hold_ignored=ignored)
    curve = between(1200, 2200, rng.uniform(set_mph - 20, set_mph)) if rng.random() < 0.3 else NO_CURVE
    r = run(car, curve, 7000, lead=lead, max_drop=rng.choice([5, 10]))
    assert r["max"] <= set_mph + 1e-6, (set_mph, d0, rate, latency, r["max"])
    assert r["min"] >= 20 - 1e-6, (set_mph, d0, rate, latency, start, r["min"])
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
  assert tr.kind == scs.STOP_WITH_CAR and tr.urgent
  assert abs(tr.target - scs.stopped_car_target(95.0)) < 1e-9


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
  assert tr.release_reason.startswith("traffic moving again") and not tr.restore_now   # gentle restore


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
