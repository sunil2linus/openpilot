"""Offline tests for map curves: the planner-side lookahead (starpilot/controls/lib/map_curves.py), the car-side
control (map_curve_control.py) and the controller's ramp rules, on curve files built by tools/curve_map."""
import importlib.util
import json
import math
import os
import random
import sys
import tempfile

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))   # curve_target for map_curve_control's offline import


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


MC = _load("map_curves", os.path.join(ROOT, "starpilot", "controls", "lib", "map_curves.py"))
CTL = _load("map_curve_control", os.path.join(HERE, "..", "map_curve_control.py"))
G = _load("tbcm", os.path.join(ROOT, "tools", "curve_map", "test_build_curve_map.py"))
B = G.B
T = _load("tcs", os.path.join(HERE, "test_curve_slowdown.py"))
cs = T.cs
MPH = 0.44704


def write_files(ways):
  """Build curve/ramp/chain records for synthetic ways (as the generator does) into a curve_map folder."""
  root = tempfile.mkdtemp()
  nidx = B.node_index(ways)
  curves, ramps, chains = [], [], []
  for ch in B.build_chains(ways):
    ramp = B.ramp_record(ch, nidx, ways)
    found = B.find_curves(ch, ways, ramp)
    curves += found
    if ramp:
      ramps.append(ramp)
    if found or ramp:
      chains.append(B.chain_record(ch))
  d = os.path.join(root, "42", "-84")
  os.makedirs(d)
  roads = [r for lst in B.road_shapes(ways).values() for r in lst]
  with open(os.path.join(d, MC.tile_name(G.LAT0, G.LON0) + ".json"), "w") as f:
    json.dump({"version": 4, "curves": curves, "ramps": ramps, "chains": chains, "roads": roads}, f)
  return root, curves, ramps


def pos(x, y):
  return G.LAT0 + y * G.M_LAT, G.LON0 + x * G.M_LON


def planner(root):
  return MC.MapCurves(MC.MapCurveStore(root, background=False))


# ---------------- planner side ----------------

def test_curve_ahead_on_the_road_and_inside_it():
  root, curves, _ = write_files(G.curve_road(r=115, turn=40, split=True))
  assert len(curves) == 1
  m = planner(root)
  la, lo = pos(0, -250)
  m.update(la, lo, 0.0, 1, True)
  assert m.status == MC.STATUS_CURVE and abs(m.radius - curves[0]["r"]) < 1e-6
  assert 200 <= m.distance <= 300, m.distance
  la, lo = pos(0, -60)
  m.update(la, lo, 0.0, 1, True)
  assert 20 <= m.distance <= 110, m.distance
  # in the curve (on the arc, heading ~20 deg left)
  ang = math.radians(20)
  la, lo = pos(-115 + 115 * math.cos(ang), 115 * math.sin(ang))
  m.update(la, lo, 340.0, 2, True)
  assert m.curve is not None and m.distance == 0.0
  # well past the curve, on the straight after it: nothing ahead
  la, lo = pos(-115 + 115 * math.cos(math.radians(40)) - math.sin(math.radians(40)) * 250,
               115 * math.sin(math.radians(40)) + math.cos(math.radians(40)) * 250)
  m.update(la, lo, 320.0, 2, True)
  assert m.curve is None and m.status == MC.STATUS_NONE


def test_other_direction_finds_the_curve_from_its_far_end():
  root, curves, _ = write_files(G.curve_road(r=115, turn=40, split=True))
  m = planner(root)
  ex = -115 + 115 * math.cos(math.radians(40)) - math.sin(math.radians(40)) * 200
  ey = 115 * math.sin(math.radians(40)) + math.cos(math.radians(40)) * 200
  la, lo = pos(ex, ey)
  m.update(la, lo, 140.0, 2, False)     # driving back towards the curve on way 2, against its node order
  assert m.curve is not None and 100 <= m.distance <= 260, m.distance


def test_unknown_way_or_no_match_gives_nothing():
  root, _, _ = write_files(G.curve_road(r=115, turn=40))
  m = planner(root)
  la, lo = pos(0, -250)
  m.update(la, lo, 0.0, 999, True)
  assert m.curve is None and m.data_here
  m.update(la, lo, 0.0, 0, True)
  assert m.curve is None


def test_on_an_exit_ramp_we_know_where_it_leads():
  root, _, ramps = write_files(G.ramp_scene())
  m = planner(root)
  ex = [r for r in ramps if r["wdir"][0][0] == 11][0]
  la, lo = ex["start"]
  m.update(la + 30 * G.M_LAT * math.cos(math.radians(20)), lo + 30 * G.M_LON * math.sin(math.radians(20)), 20.0, 11, True)
  assert m.on_ramp and m.ramp["type"] == "exit" and m.ramp["to"]["name"] == "Main Street"
  assert m.status == MC.STATUS_RAMP_CURVE and 40 <= m.radius <= 60
  assert m.ramp_end_dist > 50


def test_navigation_exit_is_matched_before_the_split():
  root, _, ramps = write_files(G.ramp_scene())
  m = planner(root)
  ex = [r for r in ramps if r["wdir"][0][0] == 11][0]
  la, lo = pos(0, -300)                             # on the motorway, heading north, the exit ~ 340 m ahead
  d = MC.dist_m(la, lo, *ex["start"])
  m.update(la, lo, 0.0, 10, True, ("off ramp", "slight right", d + 40))
  assert m.nav_exit == MC.NAV_EXIT_CURVES and m.nav_ramp["chain"] == ex["chain"]
  assert 40 <= m.nav_curve["r"] <= 60 and m.nav_curve_dist > d
  m.update(la, lo, 0.0, 10, True, ("turn", "right", d))        # not an exit
  assert m.nav_exit == MC.NAV_NONE
  m.update(la, lo, 0.0, 10, True, ("off ramp", "slight right", d + 900))   # nothing there at that distance
  assert m.nav_exit == MC.NAV_EXIT_NO_DATA


def test_no_files_here_means_no_data():
  root = tempfile.mkdtemp()
  m = planner(root)
  la, lo = pos(0, 0)
  m.update(la, lo, 0.0, 10, True, ("off ramp", "right", 500.0))
  assert not m.data_here and m.nav_exit == MC.NAV_EXIT_NO_DATA


# ---------------- car side ----------------

def F(**kw):
  f = CTL.MapFields(mode=CTL.ON)
  for k, v in kw.items():
    setattr(f, k, v)
  return f


def test_curve_far_away_waits_then_acts():
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(status=1, radius=115, distance=900), 45 * MPH, False)
  assert not out.active
  out = c.update(F(status=1, radius=115, distance=40), 50 * MPH, False)
  assert out.active and out.why == "curve" and out.floor == 0
  assert abs(out.speed - math.sqrt(2.2 * 115)) < 1e-6      # 40 m out: already the curve's own speed (2.2 m/s^2)


HOLD = int(CTL.RAMP_HOLD / CTL.DT_CTRL)


def on_ramp_for(c, frames, speed, **kw):
  out = None
  for _ in range(frames):
    out = c.update(F(**kw), speed, 0)
  return out


def test_ramp_curve_may_pass_the_max_drop_down_to_25():
  c = CTL.MapCurveControl(3.0)
  out = on_ramp_for(c, HOLD + 1, 70 * MPH, status=2, radius=45, distance=80, on_ramp=True, ramp_type=1)
  assert out.active and out.why == "ramp" and abs(out.floor - 25 * MPH) < 1e-9 and out.budget == CTL.RAMP_BUDGET


def test_a_short_wrong_ramp_match_does_nothing():
  # passing an exit in the right lane: the road match says "ramp" for less than RAMP_HOLD
  c = CTL.MapCurveControl(3.0)
  ramp = dict(status=2, radius=45, distance=60, on_ramp=True, ramp_type=1, ramp_to_highway=False,
              ramp_to_speed=35 * MPH, ramp_end_distance=150)
  for _ in range(HOLD - 1):
    out = c.update(F(**ramp), 70 * MPH, 0)
    assert not out.active and not out.fast_restore and not out.adopt
  out = c.update(F(status=0), 70 * MPH, 0)                      # back on the freeway
  assert not out.adopt and not out.active


def test_street_rules_need_progress_along_the_exit():
  c = CTL.MapCurveControl(3.0)
  ramp = dict(on_ramp=True, ramp_type=1, ramp_to_highway=False, ramp_to_speed=45 * MPH, ramp_end_distance=150)
  out = on_ramp_for(c, HOLD * 3, 70 * MPH, **ramp)              # matched for 3 s but its end is not getting closer
  assert not out.active
  assert not c.update(F(on_ramp=False), 70 * MPH, 0).adopt


def nav(d, side=1, **kw):
  """Navigation's exit d m ahead; its 45 m curve 1500 m past the exit point (a long ramp: the split speed is the setting's)"""
  return F(nav_exit=1, nav_exit_distance=d, nav_curve_radius=45, nav_curve_distance=d + 1500, nav_exit_side=side, **kw)


def nav_drive(c, d0, d1, driver, v, signal=lambda d: 0, hw=None, step=10.0):
  """Navigation's exit from d0 to d1 m ahead in steps (each one the frames the car needs for it at v, the odometer
  agreeing): [(d, out of its last frame)]."""
  hw = hw or {}
  out = []
  d = d0
  while d >= d1:
    o = None
    for _ in range(max(int(round(step / (v * CTL.DT_CTRL))), 1)):
      o = c.update(nav(d, **hw), driver, signal(d), v_ego=v)
    out.append((d, o))
    d -= step
  return out


def test_navigation_exit_is_planned_without_a_signal_slowed_for_only_in_an_exit_lane():
  # owner: navigation is the base (he often stays in the lane that becomes the exit lane without signalling); before
  # the split only the gentle slowdown to the exit speed at the split (the highway's limit here), only in an exit lane
  hw = dict(road_class=1, limit=70 * MPH)
  c = CTL.MapCurveControl(3.0)
  assert not any(o.active for _, o in nav_drive(c, 1500, 100, 80 * MPH, 80 * MPH, hw=hw))   # not in an exit lane
  assert c.plan is not None and not c.plan.confirmed
  c = CTL.MapCurveControl(3.0)
  c.lane.edge[1] = True                                    # the camera: next to the right road edge
  on = [o for _, o in nav_drive(c, 1500, 100, 80 * MPH, 80 * MPH, hw=hw) if o.active]
  assert on and all(o.why == "exit split" and o.speed >= 70 * MPH - 1e-6 for o in on), [o.speed / MPH for o in on][:5]
  assert abs(c.plan.limit - 70 * MPH) < 1e-6


def test_an_early_signal_is_remembered_after_it_goes_off():
  hw = dict(road_class=1, limit=70 * MPH)
  c = CTL.MapCurveControl(3.0)
  run = nav_drive(c, 1500, 10, 80 * MPH, 80 * MPH, signal=lambda d: 1 if 1300 >= d > 1280 else 0, hw=hw)
  acted = [(d, o.active) for d, o in run]
  assert not any(a for d, a in acted if d >= 1280) and c.plan.signalled      # 0.8 mile before: too far to slow yet
  first = next(d for d, a in acted if a)
  assert 350 <= first <= 650, first                         # only as far out as the gentle plan 80 -> 70 needs (~550 m)


def test_the_other_signal_near_the_exit_or_navigation_dropping_it_far_away_ends_the_plan():
  hw = dict(road_class=1, limit=70 * MPH)
  c = CTL.MapCurveControl(3.0)
  c.update(nav(800, **hw), 80 * MPH, -1, v_ego=33.0)       # a lane change to the left far from it: not a decision yet
  assert c.plan is not None and not c.plan.declined
  c.update(nav(250, **hw), 80 * MPH, 1, v_ego=33.0)
  assert c.plan.signalled
  c.update(nav(240, **hw), 80 * MPH, -1, v_ego=33.0)       # changed his mind near it
  assert c.plan.declined and not c.update(nav(230, **hw), 80 * MPH, 0, v_ego=33.0).active
  c = CTL.MapCurveControl(3.0)
  c.update(nav(800, **hw), 80 * MPH, 1, v_ego=33.0)
  for _ in range(int(CTL.NAV_GRACE / CTL.DT_CTRL) + 5):    # rerouted: navigation no longer shows it, far from it
    out = c.update(F(nav_exit=3, **hw), 80 * MPH, 0, v_ego=1.0)
  assert c.plan is None and not out.active


def test_unknown_side_a_signal_says_nothing_and_the_ramp_takes_over():
  # review: with the exit's side unknown any signal counted as being in its lane; now only a known side counts (and the
  # camera needs the side too)
  c = CTL.MapCurveControl(3.0)
  c.update(nav(500, side=0, road_class=1, limit=70 * MPH), 80 * MPH, -1, v_ego=33.0)
  assert not c.plan.signalled
  c.update(nav(490, side=0, road_class=1, limit=70 * MPH), 80 * MPH, 1, v_ego=33.0)
  assert not c.plan.signalled
  assert c.update(F(nav_exit=0, road_class=1), 80 * MPH, 0, v_ego=33.0) is not None and c.plan is not None   # cleared: kept
  c.odo = c.plan.exit_at + 5.0
  for _ in range(int(CTL.EXIT_CONFIRM_SHAPE_ONLY / CTL.DT_CTRL) + 5):   # the shapes alone (no model path): the link
    c.update(F(road_class=1, exit_shape=2, exit_curve_radius=45, exit_curve_distance=150), 60 * MPH, 0, v_ego=10.0)
  assert c.plan.confirmed
  out = on_ramp_for(c, HOLD + 1, 60 * MPH, status=2, radius=45, distance=140, on_ramp=True, ramp_type=1, road_class=2)
  assert c.plan is None and c.last_plan.why == "on the ramp" and out.active and out.why == "ramp"


def test_planner_reads_the_exit_side():
  assert MC.nav_side("slight right") == 1 and MC.nav_side("left") == -1 and MC.nav_side("") == 0


def test_exit_onto_a_street_caps_then_keeps_the_street_speed():
  c = CTL.MapCurveControl(3.0)
  ramp = dict(on_ramp=True, ramp_type=1, ramp_to_highway=False, ramp_to_speed=45 * MPH)
  for k in range(HOLD + 1):                                   # driving along the exit for a second
    assert not c.update(F(ramp_end_distance=500 - k * 0.3, **ramp), 70 * MPH, False).active
  out = on_ramp_for(c, 10, 70 * MPH, ramp_end_distance=150, **ramp)
  # (the Street Speed setting's default: the street's limit + 5 mph)
  assert out.active and out.why == "street cap" and abs(out.speed - 50 * MPH) < 1e-9
  out = c.update(F(on_ramp=False), 70 * MPH, False)          # off the ramp onto the street
  assert out.adopt
  assert not c.update(F(on_ramp=False), 50 * MPH, False).adopt
  # the legal limit setting: 45
  c = CTL.MapCurveControl(3.0, street_mode=0)
  for k in range(HOLD + 1):
    c.update(F(ramp_end_distance=500 - k * 0.3, **ramp), 70 * MPH, False)
  out = on_ramp_for(c, 10, 70 * MPH, ramp_end_distance=150, **ramp)
  assert out.active and out.why == "street cap" and abs(out.speed - 45 * MPH) < 1e-9


def test_ramp_onto_a_highway_restores_fast():
  c = CTL.MapCurveControl(3.0)
  out = on_ramp_for(c, HOLD, 70 * MPH, on_ramp=True, ramp_type=2, ramp_to_highway=True, ramp_end_distance=100)
  assert out.fast_restore and not out.active
  assert not c.update(F(on_ramp=True, ramp_type=2, ramp_to_highway=False, ramp_to_speed=0), 70 * MPH, False).fast_restore


def test_off_does_nothing():
  c = CTL.MapCurveControl(3.0)
  f = F(status=2, radius=45, distance=50, on_ramp=True)
  f.mode = CTL.OFF
  out = c.update(f, 70 * MPH, True)
  assert not out.active and not out.fast_restore


# ---------------- controller: ramp floor and fast restore ----------------

def drive(frames, fn, set_mph=70, lag=10):
  car = T.Car(set_mph, dash_offset_mph=0.0, lag=lag)
  logic = cs.CurveSlowdownLogic(False, 10)
  low = car.set_speed
  for f in range(frames):
    kw = fn(f, car)
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego, gas_pressed=False,
                            brake_pressed=False, standstill=False, plan_alive=True, other_button_active=False,
                            driver_button=None, **kw)
    btn, off = logic.update(inp)
    if btn is not None:
      if logic.last_action == "hold":
        car.hold_frame(btn)
      else:
        car.press(btn)
    car.tick()
    low = min(low, car.set_speed)
  return car, logic, low


def test_ramp_goes_past_the_max_drop_but_not_below_25():
  car, logic, low = drive(3000, lambda f, c: dict(csc_active=True, csc_speed=20 * MPH, csc_floor_unused=None)
                          if False else dict(csc_active=True, csc_speed=20 * MPH, curve_floor=25 * MPH, curve_budget=60))
  assert 24.5 <= low / MPH <= 26, low / MPH
  car, logic, low = drive(3000, lambda f, c: dict(csc_active=True, csc_speed=20 * MPH))
  assert 59.5 <= low / MPH <= 61, low / MPH        # a normal curve keeps the 10 mph max drop


def drive_lossy(frames, fn, set_mph, lose):
  car = T.Car(set_mph, dash_offset_mph=0.0, lag=10, lose=lose)
  logic = cs.CurveSlowdownLogic(False, 10)
  low = car.set_speed
  for f in range(frames):
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego, gas_pressed=False,
                            brake_pressed=False, standstill=False, plan_alive=True, other_button_active=False,
                            driver_button=None, **fn(f, car))
    btn, _ = logic.update(inp)
    if btn is not None:
      if logic.last_action == "hold":
        car.hold_frame(btn)
      else:
        car.press(btn)
    car.tick()
    low = min(low, car.set_speed)
  return car, logic, low


def test_drive_56_an_exit_from_80_with_half_the_taps_ignored_still_reaches_the_ramp_speed():
  # drive 56 t 880-910: from 80 mph, 60 SET- presses sent, about half ignored, the budget (presses) used up at 47 mph
  # for a 30 mph exit curve. The ramp budget counts landed steps; taps (not firm) and a firm follow both get there.
  ramp = lambda f, c: dict(csc_active=True, csc_speed=30 * MPH, curve_floor=25 * MPH, curve_budget=60, curve_track=True)  # noqa: E731
  car, logic, low = drive_lossy(6000, ramp, 80, lambda n: n % 2 == 0)
  assert low / MPH <= 30.5, low / MPH
  assert low / MPH >= 24.5
  firm = lambda f, c: dict(csc_active=True, csc_speed=30 * MPH, curve_floor=25 * MPH, curve_budget=60, curve_track=True,  # noqa: E731
                           curve_hold=True)
  car, logic, low = drive_lossy(3000, firm, 80, lambda n: n % 2 == 0)
  assert low / MPH <= 30.5 and low / MPH >= 24.5, low / MPH
  assert any(h["direction"] == cs.SET_MINUS for h in logic.hold_log), "a firm exit holds"
  # a car that answers no press at all: the presses stop (CURVE_BUDGET_SENT_FACTOR x the budget)
  car, logic, low = drive_lossy(20000, ramp, 80, lambda n: True)
  assert logic.lower_presses <= cs.CURVE_BUDGET_SENT_FACTOR * 60


def test_fast_restore_raises_with_holds_and_never_above_the_driver_speed():
  def fn(f, c):
    if f < 2500:
      return dict(csc_active=True, csc_speed=30 * MPH, curve_floor=25 * MPH, curve_budget=60)
    return dict(csc_active=False, csc_speed=100.0, fast_restore=True)
  car, logic, low = drive(6000, fn)
  assert low / MPH <= 31
  assert abs(car.set_speed / MPH - 70) < 0.6, car.set_speed / MPH
  assert any(h["direction"] == cs.SET_PLUS for h in logic.hold_log), "restored with SET+ holds"


# ---------------- yellow advisory signs ----------------

def test_advisory_sign_caps_the_curve_speed():
  # West Utica Road: 117 m radius, 45 mph limit, yellow 30 mph sign. Shape alone at 2.2 m/s^2 allows ~36 mph,
  # below the sign + 10 mph, so the sign changes nothing there; a 20 mph sign would cap it at 30 mph.
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(status=1, radius=116.6, distance=0), 45 * MPH, 0)
  assert out.active and 35.5 <= out.speed / MPH <= 36.2, out.speed / MPH
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(status=1, radius=116.6, distance=0, advisory=30 * MPH), 45 * MPH, 0)
  assert out.active and 35.5 <= out.speed / MPH <= 36.2
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(status=1, radius=116.6, distance=0, advisory=20 * MPH), 45 * MPH, 0)
  assert out.active and abs(out.speed / MPH - 30) < 0.01           # 20 + 10 mph
  # a sign above what the shape allows changes nothing (the shape is lower)
  c = CTL.MapCurveControl(3.0)
  out = on_ramp_for(c, HOLD, 70 * MPH, status=2, radius=45, distance=0, on_ramp=True, ramp_type=1, advisory=35 * MPH)
  assert abs(out.speed - math.sqrt(2.2 * 45)) < 1e-6


def test_planner_uses_the_advisory_to_choose_and_announce():
  assert abs(MC.curve_speed_limit(116.6, 20 * MPH) / MPH - 30) < 0.01
  assert MC.curve_speed_limit(116.6) > MC.curve_speed_limit(116.6, 20 * MPH)
  # an easy-looking curve with a 25 mph sign governs over a sharper unsigned one at the same distance
  assert MC.plan_speed(300, 200, 25 * MPH) < MC.plan_speed(150, 200)


def test_builder_keeps_a_signed_curve_that_looks_easy():
  assert B.ADVISORY_OVER == 10 * MPH


# ---------------- own road matching (StarPilot: no MapMatchedWay from its map program) ----------------

def matcher(root):
  store = MC.MapCurveStore(root, background=False)
  la, lo = pos(0, 0)
  store.update(la, lo)
  return MC.RoadMatcher(store)


def test_road_matcher_finds_the_way_and_the_direction():
  root, _, _ = write_files(G.curve_road(r=115, turn=40, split=True))
  rm = matcher(root)
  la, lo = pos(4, -250)                                # 4 m beside way 1, heading north (its node order)
  assert rm.update(la, lo, 3.0) == (1, True)
  assert rm.update(la, lo, 181.0) == (1, False)        # the other way along it
  assert rm.update(la, lo, 90.0) == (0, True)          # across it: not on this road
  la, lo = pos(60, -250)
  assert rm.update(la, lo, 0.0) == (0, True)           # 60 m away: no road here
  ang = math.radians(30)
  la, lo = pos(-115 + 115 * math.cos(ang), 115 * math.sin(ang))
  wid, fwd = rm.update(la, lo, 330.0)                  # on the arc, past the split onto way 2
  assert wid in (1, 2) and fwd


def test_road_matcher_keeps_the_main_road_at_an_exit_split():
  root, _, _ = write_files(G.ramp_scene())
  rm = matcher(root)
  assert rm.update(*pos(0, -100), 0.0)[0] == 10        # motorway
  assert rm.update(*pos(0, 20), 0.0)[0] == 10          # past the split, still on the motorway (ramp ~7 m away)
  h = math.radians(20)
  assert rm.update(*pos(15 * math.sin(h), 15 * math.cos(h)), 20.0)[0] == 10   # on the ramp, too close to call
  assert rm.update(*pos(45 * math.sin(h), 45 * math.cos(h)), 20.0) == (11, True)   # clearly on the exit ramp
  assert rm.update(*pos(0, 120), 0.0)[0] == 10         # a car that stayed on the motorway


def test_planner_uses_its_own_matching_when_the_map_program_gives_none():
  root, curves, _ = write_files(G.curve_road(r=115, turn=40, split=True))

  class P:
    def __init__(self, vals):
      self.vals = vals

    def get(self, key, encoding=None):
      return self.vals.get(key)

  mp = MC.MapCurvesPlanner(P({"MazdaMapCurves": "1"}), P({}))
  mp.mc = MC.MapCurves(MC.MapCurveStore(root, background=False))
  mp.matcher = MC.RoadMatcher(mp.mc.store)

  class SM(dict):
    alive = {"navInstruction": False}
    valid = {"navInstruction": False}

  la, lo = pos(0, -250)
  gps = {"latitude": la, "longitude": lo, "bearing": 0.0}
  mp.update(SM(), gps, 18.0, 18.0)        # loads the files, then matches way 1
  mp.update(SM(), gps, 18.0, 18.0)
  assert mp.mc.curve is not None and abs(mp.mc.radius - curves[0]["r"]) < 1e-6
  assert mp.notice is None                 # 250 m out at 40 mph the curve does not need slowing yet
  la, lo = pos(0, -90)
  mp.update(SM(), {"latitude": la, "longitude": lo, "bearing": 0.0}, 18.0, 18.0)
  assert mp.notice == "curve"


def test_off_a_short_exit_onto_a_street_keeps_the_lowered_speed():
  # own road matching: the road after the exit is known (a primary road), so no progress along the exit is needed
  c = CTL.MapCurveControl(3.0)
  ramp = dict(on_ramp=True, ramp_type=1, ramp_to_highway=False, ramp_to_speed=0.0, ramp_end_distance=40, road_class=2)
  on_ramp_for(c, HOLD + 1, 70 * MPH, **ramp)
  assert c.update(F(on_ramp=False, road_class=5), 70 * MPH, 0).adopt


def test_never_keeps_a_lowered_speed_when_the_next_road_is_a_freeway():
  c = CTL.MapCurveControl(3.0)
  ramp = dict(on_ramp=True, ramp_type=1, ramp_to_highway=False, ramp_to_speed=45 * MPH, road_class=2)
  for k in range(HOLD * 3):
    c.update(F(ramp_end_distance=300 - k * 0.5, **ramp), 70 * MPH, 0)
  assert not c.update(F(on_ramp=False, road_class=1), 70 * MPH, 0).adopt


# ---------------- map curve follower (drive 50: West Utica Road, 45 mph, the 91 m curve) ----------------
# The old plan (0.25 m/s^2 staircase) began 380-395 m before the curve and the car crawled down 1 mph at a time. Now
# the plan is firm (MAP_DECEL) and the follower asks the cruise for a set speed below the plan; the simulated car is the
# stock cruise as measured (Car(cruise_gain=0.19): 0.19 m/s^2 per m/s of set speed gap, ~1 s dead time).

CT = _load("curve_target", os.path.join(HERE, "..", "curve_target.py"))
WEST_UTICA_R = 91.0
MEASURED_CRUISE = dict(cruise_gain=0.19, cruise_dead=1.0)


def drive_map(radius=WEST_UTICA_R, set_mph=45, max_drop=10, start_d=600.0, length=80.0, follow=True, decel=None,
              metric=False, events=None, frames=6000, second=None, camera=None, stop=None, car_kw=None, closed=True):
  """Drive at the driver's speed towards a map curve `start_d` m ahead (`length` m long), the map control and the
  controller in the loop. follow=False: the old plan-only staircase (no true speed, no max drop in the plan).
  second: (radius, gap m) of a curve starting that far after the first one's exit. camera(x) / stop(x): the camera
  curve target / stopped car target (m/s) at x m before the entry, or None. Returns the run's numbers."""
  step = cs.MS_PER_KPH if metric else MPH
  car = T.Car(set_mph * MPH / step, dash_offset_mph=0.0, lag=10, accel=0.6, step=step, **dict(MEASURED_CRUISE, **(car_kw or {})))
  car.set_speed = car.v_ego = set_mph * MPH
  logic = cs.CurveSlowdownLogic(metric, max_drop)
  ctl = CTL.MapCurveControl(3.0, decel if decel is not None else CTL.MAP_DECEL)
  mix = CTL.CurveMix(cs.CURVE_MARGIN)
  md = logic.max_drop_steps * logic.step
  x = start_d
  st = dict(gas=False, brake=False, enabled=True, cruise=True)
  presses, sets, vs, xs, gaps = [], [], [], [], []
  first = v_entry = v_entry2 = None
  v_prev = car.v_ego
  a_hist = []
  for f in range(frames):
    if events and f in events:
      events[f](st, car)
    if x > -length:
      fld = F(status=1, radius=radius, distance=max(x, 0.0))
    elif second is not None and x > -length - second[1] - length:
      fld = F(status=1, radius=second[0], distance=max(second[1] + length + x, 0.0))
    else:
      fld = F(status=0)
    driver = car.set_speed + logic.offset * logic.step if st["cruise"] else 0.0
    mo = ctl.update(fld, driver, 0, v_ego=car.v_ego if follow else 0.0, max_drop=md if follow else 0.0,
                    a_ego=(a_hist[-1] if a_hist else 0.0) if (follow and closed) else None)
    # the wrapper's combination with the camera curve target (curve_slowdown.CurveSlowdown.update)
    cam = camera(x) if camera is not None else None
    active, speed, floor, budget, track, raise_ok = mix.update(cam is not None, cam if cam is not None else CTL.NO_CURVE, mo,
                                                               drop_floor=max(driver - md, cs.MIN_SET_SPEED))
    stop_t = stop(x) if stop is not None else None
    inp = cs.SlowdownInputs(enabled=st["enabled"], cruise_enabled=st["cruise"], set_speed=car.set_speed, v_ego=car.v_ego,
                            gas_pressed=st["gas"], brake_pressed=st["brake"], standstill=False, plan_alive=True,
                            csc_active=active, csc_speed=min(speed, 100.0), other_button_active=False,
                            driver_button=car.driver_button_now(), curve_floor=floor, curve_budget=budget,
                            curve_track=track, curve_raise=raise_ok, fast_restore=mo.fast_restore,
                            stop_active=stop_t is not None, stop_target=stop_t or 0.0, stop_urgent=False)
    btn, off = logic.update(inp)
    if btn is not None:
      if logic.last_action == "hold":
        car.hold_frame(btn)
      else:
        car.press(btn)
      presses.append((f, round(x), btn, logic.last_action))
      if first is None and btn == cs.SET_MINUS:
        first = x
    if st["gas"]:
      car.gas_frame = car.frame
    car.tick()
    a_hist.append((car.v_ego - v_prev) / T.DT)
    v_prev = car.v_ego
    x_new = x - car.v_ego * T.DT
    if x > 0 >= x_new:
      v_entry = car.v_ego
    if second is not None and x > -length - second[1] >= x_new:
      v_entry2 = car.v_ego
    x = x_new
    sets.append(car.set_speed)
    vs.append(car.v_ego)
    xs.append(x)
    if 50 < x < 150:
      gaps.append(car.v_ego - car.set_speed)
  n = 50                                      # peak decel over half a second
  peak = max(-sum(a_hist[i:i + n]) / n for i in range(len(a_hist) - n)) if len(a_hist) > n else 0.0
  in_curve = [v for v, xx in zip(vs, xs, strict=True) if -length < xx <= 0]
  return dict(first=first, v_entry=v_entry / MPH if v_entry else None, v_entry2=v_entry2 / MPH if v_entry2 else None,
              min_v=min(vs) / MPH, min_in_curve=min(in_curve) / MPH if in_curve else None, peak=peak, presses=presses,
              n_minus=sum(1 for p in presses if p[2] == cs.SET_MINUS), n_plus=sum(1 for p in presses if p[2] == cs.SET_PLUS),
              min_set=min(sets) / MPH, final_set=sets[-1] / MPH, max_gap=max(gaps) / MPH if gaps else 0.0,
              logic=logic, sets=sets, vs=vs, xs=xs)


def test_west_utica_follower_starts_later_and_slows_firmer():
  # drive 50: the slowdown began 380 m (17 s) before the entry, 1 mph SET- steps every 1-2 s, the car at 35.5 ~50 m
  # before the entry (max drop 10: the curve's 32 mph held at 35). The old plan reproduced on the measured cruise:
  old = drive_map(max_drop=10, follow=False, decel=0.25)
  assert old["first"] >= 350 and old["max_gap"] <= 4.0, (old["first"], old["max_gap"])
  for max_drop, v_curve in ((10, 35.0), (15, math.sqrt(2.2 * WEST_UTICA_R) / MPH)):
    r = drive_map(max_drop=max_drop)
    # starts about half as far out (the plan is firm and aims for the max drop floor, not below it) ...
    assert 170 <= r["first"] <= 260 and r["first"] <= old["first"] - 120, (max_drop, r["first"])
    # ... asks the cruise for a real gap over the last 150 m (the old staircase sat 1-2 mph below the true speed on
    # drive 50, 3.7 on the simulated cruise) ...
    assert r["max_gap"] >= 5.0 and r["max_gap"] >= old["max_gap"] + 2.0, (max_drop, r["max_gap"], old["max_gap"])
    # ... and is at the curve's speed at the entry, never more than ~1 mph below it, with a firm but gentle slowdown
    assert abs(r["v_entry"] - v_curve) <= 1.0, (max_drop, r["v_entry"], v_curve)
    assert r["min_v"] >= v_curve - 1.0 and r["min_in_curve"] >= v_curve - 1.0, (max_drop, r["min_v"], r["min_in_curve"])
    assert r["peak"] <= 0.7, (max_drop, r["peak"])
    # the temporary undershoot stays within MAP_ROAD_UNDERSHOOT of the curve's speed (so of the max drop floor; drive
    # 54: 3 mph, it was 5), the set speed the car settles at is the curve's speed (>= the floor), and the driver's speed
    # comes back after the curve
    road_under = CT.MAP_ROAD_UNDERSHOOT / MPH
    assert r["min_set"] >= v_curve - road_under - 1e-6 and r["min_set"] <= v_curve - 1, (max_drop, r["min_set"])
    in_curve_sets = [sp / MPH for sp, xx in zip(r["sets"], r["xs"], strict=True) if -80 < xx <= -20]
    assert min(in_curve_sets) >= 45 - max_drop - 1e-6 and max(in_curve_sets) <= math.ceil(v_curve) + 1e-6, (max_drop, min(in_curve_sets))
    assert abs(r["final_set"] - 45) < 1e-6
    # a reasonable number of presses: down to the undershoot, up to the curve's speed, and the restore
    assert r["n_minus"] <= 20 and r["n_plus"] <= 25, (max_drop, r["n_minus"], r["n_plus"])
    raises = [p for p in r["presses"] if p[3] == "raise"]
    assert raises and all(0 < p[1] < 60 for p in raises), raises       # the step back up comes just before the entry
    seq = "".join("M" if b == cs.SET_MINUS else "P" for _, _, b, _ in r["presses"])
    assert seq.count("PM") == 0, seq                                   # no ping-pong


def test_follower_works_for_a_slower_or_quicker_cruise():
  # the measured gain is a median; a cruise answering 0.15 or 0.25 m/s^2 per m/s still arrives at the curve's speed
  for gain in (0.15, 0.25):
    r = drive_map(max_drop=10, car_kw=dict(cruise_gain=gain))
    assert abs(r["v_entry"] - 35) <= 1.0 and r["min_in_curve"] >= 34.0 and r["min_set"] >= 30 - 1e-6, (gain, r["v_entry"], r["min_in_curve"])


def test_follower_hands_control_back_to_the_driver():
  # a driver SET- tap mid-slowdown: their new speed, as today: we stop managing (nothing of ours while the curve the
  # tap came in is on), and whatever the map asks for later is relative to that speed, which also comes back after
  r0 = drive_map(max_drop=10)
  mid = next(f for f, x, b, _ in r0["presses"] if x < 150)
  r = drive_map(max_drop=10, events={mid + 60: lambda st, car: car.press(cs.SET_MINUS, who="driver")}, frames=mid + 3500)
  theirs = r["sets"][mid + 90] / MPH                       # where the driver's tap put the set speed
  assert theirs <= 45 - 2 and all(abs(sp / MPH - theirs) < 1e-6 for sp in r["sets"][mid + 90:mid + 360]), theirs
  assert [p for p in r["presses"] if mid + 60 < p[0] <= mid + 360] == []        # nothing of ours for the next 3 s
  assert r["logic"].anchor is None or r["logic"].anchor <= theirs * MPH + 1e-6  # never managing from the old 45 again
  assert abs(r["final_set"] - theirs) < 1e-6, (r["final_set"], theirs)
  # a driver SET+ tap mid-slowdown stands us down for the curve: no SET- of ours after it
  r = drive_map(max_drop=10, events={mid + 60: lambda st, car: car.press(cs.SET_PLUS, who="driver")}, frames=mid + 3500)
  assert [p for p in r["presses"] if p[0] > mid + 60 and p[2] == cs.SET_MINUS] == []
  assert abs(r["final_set"] - 45) < 1e-6
  # the brake mid-slowdown: nothing while braking; the gas: nothing on the gas
  for pedal in ("brake", "gas"):
    def on(st, car, pedal=pedal):
      st[pedal] = True
    r = drive_map(max_drop=10, events={mid + 60: on}, frames=mid + 400)
    assert all(p[0] <= mid + 60 for p in r["presses"]), (pedal, r["presses"])


def test_follower_second_curve_right_after_the_first():
  # a 120 m curve (36.3 mph) 150 m after West Utica's exit: the set speed goes from 35 up to the second curve's speed,
  # not back to 45 in between, and the car enters it at that speed; the driver's speed comes back after it
  v2 = math.sqrt(2.2 * 120.0) / MPH
  r = drive_map(max_drop=10, second=(120.0, 150.0), frames=9000)
  assert abs(r["v_entry"] - 35) <= 1.0 and abs(r["v_entry2"] - v2) <= 1.0, (r["v_entry"], r["v_entry2"], v2)
  between = [sp / MPH for sp, xx in zip(r["sets"], r["xs"], strict=True) if -230 < xx <= -80]
  assert max(between) <= math.ceil(v2) + 1e-6, max(between)
  assert abs(r["final_set"] - 45) < 1e-6
  assert r["n_minus"] <= 20, r["n_minus"]


def test_follower_in_km_h():
  # 72 km/h (45 mph), max drop 10 mph = 16 km/h: the curve held at 56 km/h; 1 km/h steps
  r = drive_map(max_drop=10, metric=True, set_mph=72 / 1.609344)
  v_curve = 56 / 1.609344
  assert abs(r["v_entry"] - v_curve) <= 1.0 and r["min_in_curve"] >= v_curve - 1.0, (r["v_entry"], r["min_in_curve"])
  assert r["min_set"] >= v_curve - 5 - 1e-6, r["min_set"]
  assert 170 <= r["first"] <= 260, r["first"]
  assert abs(r["final_set"] * 1.609344 - 72) < 1e-6
  assert r["n_minus"] <= 30, r["n_minus"]


def test_follower_lowest_target_wins_camera_and_stopped_car():
  # max drop 15 (floor 30): the camera sees the curve a little tighter (29.8 mph true, 30.5 with its 0.3 m/s margin)
  # near it: the step back up stops at the highest step not above the camera's target (30), never at the map's 32
  # (it used to round up to 31); the undershoot still happened first
  def cam(x):
    return 29.8 * MPH if x < 120 else None
  r = drive_map(max_drop=15, camera=cam)
  in_curve_sets = [sp / MPH for sp, xx in zip(r["sets"], r["xs"], strict=True) if -80 < xx <= -20]
  assert max(in_curve_sets) <= 30 + 1e-6 and min(in_curve_sets) >= 30 - 1e-6, (min(in_curve_sets), max(in_curve_sets))
  assert 28 <= r["min_set"] <= 29, r["min_set"]       # (the road undershoot: 3 mph, drive 54)

  # max drop 10 (floor 35): a camera target below the floor is held at the floor, as today
  def cam35(x):
    return 33.5 * MPH if x < 120 else None
  r = drive_map(max_drop=10, camera=cam35)
  in_curve_sets = [sp / MPH for sp, xx in zip(r["sets"], r["xs"], strict=True) if -80 < xx <= -20]
  assert max(in_curve_sets) <= 35 + 1e-6 and min(in_curve_sets) >= 35 - 1e-6, (min(in_curve_sets), max(in_curve_sets))

  # a stopped car ahead wants 25 mph while the follower would step back up: the stop's lower target wins, no step up
  def stop(x):
    return 25 * MPH if x < 60 else None
  r = drive_map(max_drop=10, stop=stop, frames=4500)
  assert not any(p[3] == "raise" for p in r["presses"]), [p for p in r["presses"] if p[3] == "raise"]
  assert r["min_set"] <= 25 + 1e-6, r["min_set"]


def test_follower_invariants_over_curves_speeds_and_settings():
  rng = random.Random(7)
  checked = 0
  while checked < 14:
    set_mph, max_drop = rng.choice([35, 45, 55, 65]), rng.choice([5, 10, 15])
    radius, gain = rng.choice([60, 91, 120, 160, 220, 300]), rng.choice([0.15, 0.19, 0.25])
    v_c = max(math.sqrt(2.2 * radius) / MPH, set_mph - max_drop)        # the speed the curve is taken at
    if v_c > set_mph - 2:
      continue
    checked += 1
    r = drive_map(radius=radius, set_mph=set_mph, max_drop=max_drop, start_d=700.0, frames=9000, car_kw=dict(cruise_gain=gain))
    case = (set_mph, max_drop, radius, gain)
    assert max(r["sets"]) / MPH <= set_mph + 1e-6, case                                   # never above the driver's speed
    assert r["min_set"] >= max(v_c - 5, 25) - 1e-6, (case, r["min_set"])                 # undershoot bounded
    in_curve = [sp / MPH for sp, xx in zip(r["sets"], r["xs"], strict=True) if -80 < xx <= -20]
    assert min(in_curve) >= set_mph - max_drop - 1e-6 and max(in_curve) <= math.ceil(v_c) + 1e-6, (case, in_curve[0])
    assert abs(r["v_entry"] - v_c) <= 1.5 and r["min_in_curve"] >= v_c - 1.5, (case, r["v_entry"], r["min_in_curve"])
    assert abs(r["final_set"] - set_mph) < 1e-6, (case, r["final_set"])                   # restored after the curve
    seq = "".join("M" if b == cs.SET_MINUS else "P" for _, _, b, _ in r["presses"])
    assert seq.count("PM") <= 1, (case, seq)                                              # no ping-pong
    assert r["peak"] <= (0.7 if gain <= 0.19 else 0.9), (case, r["peak"])


def test_follower_command_and_plan_shapes():
  # the plan: firm, aims for the max drop floor, starts about 200 m out for West Utica at 45 (old plan: ~390 m)
  c = CTL.MapCurveControl(3.0)
  md = 10 * MPH
  start = next(d for d in range(500, -1, -2) if c.update(F(status=1, radius=WEST_UTICA_R, distance=d), 45 * MPH, 0, 20.0, md).active)
  assert 190 <= start <= 215, start
  c = CTL.MapCurveControl(3.0)
  start15 = next(d for d in range(500, -1, -2) if c.update(F(status=1, radius=WEST_UTICA_R, distance=d), 45 * MPH, 0, 20.0, 15 * MPH).active)
  assert 240 <= start15 <= 265, start15
  c = CTL.MapCurveControl(3.0, 0.25)
  old = next(d for d in range(500, -1, -2) if c.update(F(status=1, radius=WEST_UTICA_R, distance=d), 45 * MPH, 0).active)
  assert old >= 380, old
  # the follower: the gap for the required decel, bounded by the undershoot, the curve's speed once nearly there
  v_c = 35 * MPH
  required = ((45 * MPH) ** 2 - v_c ** 2) / (2 * (400 - v_c - 45 * MPH))        # over the room after the lead and dead times
  assert abs(CT.map_follow_speed(45 * MPH, v_c, 400.0) - (45 * MPH - required / CT.MAP_FOLLOW_GAIN)) < 1e-9
  # nearly there with no room left: the gap is capped (MAP_FOLLOW_MAX_DECEL), then bounded by the undershoot
  assert abs(CT.map_follow_speed(45 * MPH, v_c, 20.0) - (45 * MPH - CT.MAP_FOLLOW_MAX_DECEL / CT.MAP_FOLLOW_GAIN)) < 1e-9
  assert abs(CT.map_follow_speed(v_c + 2.5 * MPH, v_c, 20.0) - (v_c - CT.MAP_UNDERSHOOT)) < 1e-9
  assert CT.map_follow_speed(v_c + CT.MAP_SETTLE - 0.01, v_c, 100.0) == v_c
  f = CT.MapCurveFollower()
  assert f.update(20.0, v_c, 200.0, 19.5) <= 19.5                      # never above the plan
  low = f.update(19.0, v_c, 150.0, 19.0)
  assert f.update(19.0, v_c, 170.0, 19.0) == low                       # more room again (a GPS step): the command does not rise
  assert f.update(v_c + 0.5, v_c, 20.0, v_c + 0.1) == v_c              # settled: the curve's speed
  assert f.update(v_c + CT.MAP_SETTLE + 0.5 * CT.MAP_SETTLE_HYST, v_c, 10.0, v_c + 2.0) == v_c   # within the hysteresis: still
  assert f.update(v_c + CT.MAP_SETTLE + CT.MAP_SETTLE_HYST + 0.1, v_c, 10.0, v_c + 2.0) < v_c    # clearly too fast: a gap again
  f.update(19.0, v_c + 1.0, 150.0, 19.0)                               # another curve speed: starts over
  assert f.cmd is not None and not f.settled
  # ramp curves: without the true speed the plan alone; with it their own follower since drive 53, the ramp floor and
  # budget (never the Max Drop, never under 25 mph)
  c = CTL.MapCurveControl(3.0)
  out = on_ramp_for(c, HOLD + 1, 70 * MPH, status=2, radius=45, distance=80, on_ramp=True, ramp_type=1)
  assert out.active and not out.track and abs(out.floor - 25 * MPH) < 1e-9
  for _ in range(3):
    out = c.update(F(status=2, radius=45, distance=80, on_ramp=True, ramp_type=1), 70 * MPH, 0, v_ego=30.0, max_drop=md)
  assert out.active and out.track and out.why == "ramp" and abs(out.floor - 25 * MPH) < 1e-9 and out.budget == CTL.RAMP_BUDGET
  assert out.speed <= c.curve.speed + 1e-9 and abs(out.curve_speed - math.sqrt(2.2 * 45)) < 1e-9


def test_a_road_curve_stops_governing_after_its_apex_when_the_camera_slowdown_is_on():
  # slow in, speed up from the apex: past the apex the camera curve slowdown holds the speed for what is left
  root, curves, _ = write_files(G.curve_road(r=115, turn=40, split=True))
  c = curves[0]

  def on_arc(deg):
    a = math.radians(deg)
    return pos(-115 + 115 * math.cos(a), 115 * math.sin(a)), 360.0 - deg

  for release in (False, True):
    m = MC.MapCurves(MC.MapCurveStore(root, background=False), release_after_apex=release)
    (la, lo), brg = on_arc(8)                 # before the apex (the middle of the arc, ~20 deg)
    m.update(la, lo, brg, 2, True)
    assert m.curve is not None and m.distance == 0.0
    (la, lo), brg = on_arc(34)                # ~28 m past the apex, still on the arc
    m.update(la, lo, brg, 2, True)
    assert (m.curve is None) == release, (release, m.curve)
  assert "apex" in c


# ---------------- the follower with the stopped car trigger and the camera (review of the follower) ----------------
# The real stopped car trigger (red light head start, false stop guard, unsure car) and the camera curve target in the
# loop with the map control and the controller, on the measured cruise. Each case broke before the fix: a restore
# asked for at once ping-ponged through the curve and climbed past it, the step back up ran during a stop episode,
# followed the camera's noisy target up, jumped after the gas, rounded above the curve's speed and ran after a
# follower restart.

def drive_follow(radius=WEST_UTICA_R, set_mph=45, max_drop=10, start_d=600.0, length=80.0, frames=6000, fields=None,
                 camera=None, red=None, plan_stop=None, model=None, lead=None, lead_from=0, events=None):
  """drive_map with the stopped car trigger: red(f) / plan_stop(f) / model(f): the stop-light detector, the model's
  planned stop and desired acceleration at frame f; lead: a StoppedLead that starts moving at frame lead_from;
  fields(f, x): the map fields instead of one curve of `radius`. Returns presses as (frame, x, button, action, set
  mph before, v mph, trigger active, trigger kind) and the run's set speeds / speeds / positions."""
  car = T.Car(set_mph, dash_offset_mph=0.0, lag=10, accel=0.6, step=MPH, **MEASURED_CRUISE)
  car.set_speed = car.v_ego = set_mph * MPH
  logic = cs.CurveSlowdownLogic(False, max_drop)
  ctl = CTL.MapCurveControl(3.0, CTL.MAP_DECEL)
  mix = CTL.CurveMix(cs.CURVE_MARGIN)
  trig = T.scs.StoppedCarTrigger()
  md = logic.max_drop_steps * logic.step
  x, a_ego = start_d, 0.0
  st = dict(gas=False, brake=False, enabled=True, cruise=True)
  presses, sets, vs, xs = [], [], [], []
  for f in range(frames):
    if events and f in events:
      events[f](st, car)
    if fields is not None:
      fld = fields(f, x)
    else:
      fld = F(status=1, radius=radius, distance=max(x, 0.0)) if x > -length else F(status=0)
    driver = car.set_speed + logic.offset * logic.step
    mo = ctl.update(fld, driver, 0, v_ego=car.v_ego, max_drop=md)
    cam = camera(f, x) if camera is not None else None
    active, speed, floor, budget, track, raise_ok = mix.update(cam is not None, cam if cam is not None else CTL.NO_CURVE, mo,
                                                               drop_floor=max(driver - md, cs.MIN_SET_SPEED))
    ls, lp, ld, lv, ly = lead.state(f, car) if (lead is not None and f >= lead_from) else (False, 0.0, 0.0, 0.0, 0.0)
    si = T.scs.StoppedCarInputs(enabled=True, cruise_enabled=True, brake_pressed=st["brake"], v_ego=car.v_ego, lead_alive=True,
                                lead_status=ls, lead_prob=lp, lead_d=ld, lead_v=lv, lead_y=ly, model_alive=True,
                                model_accel=model(f) if model else 0.0, plan_stop=plan_stop(f) if plan_stop else False,
                                a_ego=a_ego, model_length=200.0, gas_pressed=st["gas"], set_speed=car.set_speed,
                                red_light=red(f) if red else False)
    stop = trig.update(si)
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego, gas_pressed=st["gas"],
                            brake_pressed=st["brake"], standstill=car.v_ego < 0.3, plan_alive=True, csc_active=active,
                            csc_speed=min(speed, 100.0), other_button_active=False, driver_button=car.driver_button_now(),
                            curve_floor=floor, curve_budget=budget, curve_track=track, curve_raise=raise_ok,
                            fast_restore=mo.fast_restore, stop_active=stop, stop_target=trig.target, stop_urgent=trig.urgent,
                            stop_restore_now=trig.restore_now, hold_restore=trig.hold_restore, radar_lead=trig.radar_stable,
                            stop_clear_ahead=trig.clear_ahead, stop_pulling_away=trig.pulling_away)
    btn, _ = logic.update(inp)
    if btn is not None:
      presses.append((f, round(x), btn, logic.last_action, round(car.set_speed / MPH), car.v_ego / MPH,
                      stop or trig.hold_restore, trig.kind))
      car.hold_frame(btn) if logic.last_action == "hold" else car.press(btn)
    if st["gas"]:
      car.gas_frame = car.frame
    v_before = car.v_ego
    car.tick()
    if lead is not None and f >= lead_from:
      lead.tick(car)
    a_ego = (car.v_ego - v_before) / T.DT
    x -= car.v_ego * T.DT
    sets.append(car.set_speed / MPH)
    vs.append(car.v_ego / MPH)
    xs.append(x)
  seq = "".join("M" if p[2] == cs.SET_MINUS else "P" for p in presses)
  return dict(presses=presses, seq=seq, sets=sets, vs=vs, xs=xs, trig=trig, logic=logic)


def _max_set(r, lo, hi):
  return max(sp for sp, xx in zip(r["sets"], r["xs"], strict=True) if lo < xx <= hi)


def test_red_light_gone_inside_a_followed_curve_restores_only_up_to_the_curve():
  # settled at 35 in a long 35 mph curve (max drop 10 from 45): the red light head start taps once (or four times),
  # then the light goes and the trigger asks for the speed back at once. It came back only to 35 and the curve kept
  # governing; before the fix the restore went 35 <-> 37 every 0.2 s through the curve (~50 presses) and, once the
  # curve's 40 SET- were spent, on up to 44 inside it (review sB)
  for lo, hi in ((3000, 3350), (3000, 3600)):
    r = drive_follow(length=600.0, frames=9000, red=lambda f, lo=lo, hi=hi: lo <= f < hi)
    assert r["trig"].release_reason == T.scs.RED_GONE, r["trig"].release_reason
    assert _max_set(r, -600, 0) <= 35 + 1e-6, (lo, hi, _max_set(r, -600, 0))
    in_curve = "".join("M" if p[2] == cs.SET_MINUS else "P" for p in r["presses"] if -600 < p[1] <= 0)
    assert in_curve.count("PM") == 0 and len(in_curve) <= 10, (lo, hi, in_curve)
    assert abs(r["sets"][-1] - 45) < 1e-6, r["sets"][-1]        # the curve's own restore after it


def test_red_light_gone_while_slowing_for_a_followed_curve_never_ping_pongs():
  # the head start before the curve, the light gone while the follower slows for it (review sG): before the fix the
  # restore and the follower's SET- took turns all the way into the curve
  for lo, hi in ((1600, 2100), (1700, 2400), (1800, 2600)):
    r = drive_follow(frames=5000, red=lambda f, lo=lo, hi=hi: lo <= f < hi)
    assert r["trig"].release_reason == T.scs.RED_GONE
    assert r["seq"].count("PM") == 0 and r["seq"].count("P") <= 16, (lo, hi, r["seq"])
    assert _max_set(r, -80, 0) <= 35 + 1e-6, (lo, hi, _max_set(r, -80, 0))


def test_a_guarded_stop_ahead_clearing_inside_a_followed_curve_stays_at_the_curve():
  # the model plans a stop for 2.5 s with no red light (the false stop guard: taps), then clears; settled in a 35 mph
  # curve. Before the fix the restore asked for at once ping-ponged and then climbed to 44-45 in the curve (review sI)
  r = drive_follow(length=700.0, frames=6000, plan_stop=lambda f: 3100 <= f < 3350,
                   model=lambda f: 0.5 if f >= 3350 else 0.0)
  assert any(p[7] == T.scs.STOP_AHEAD and p[6] for p in r["presses"]), r["presses"]
  assert _max_set(r, -700, -60) <= 35 + 1e-6, _max_set(r, -700, -60)
  after = "".join("M" if p[2] == cs.SET_MINUS else "P" for p in r["presses"] if p[0] > 3350)
  assert after.count("PM") == 0 and len(after) <= 6, after


def test_an_unsure_car_holding_the_speed_inside_a_followed_curve_sees_no_set_plus():
  # settled in a long 35 mph curve; an unsure (60 %) slower car at 125 m: 1 mph taps, and while it is not seen the
  # trigger publishes target 100 ("hold, do not lower further"), leaving the curve the lowest target. The follower's
  # step back up must not run then: a stop episode never sees our SET+ (review sH: 31 -> 33 / 34 while closing on it)
  for gaps in ((), ((3700, 3800),), ((3700, 3800), (4000, 4150))):
    lead = T.StoppedLead(125.0, v_mph=15.0, prob=0.6, y=0.1, present=lambda f, gaps=gaps: not any(a <= f < b for a, b in gaps))
    r = drive_follow(length=900.0, frames=4600, lead=lead, lead_from=3100)
    episode = [p for p in r["presses"] if p[6]]
    assert any(p[7] == T.scs.SLOW_UNSURE for p in episode), (gaps, r["presses"])
    assert not any(p[2] == cs.SET_PLUS for p in episode), (gaps, [p for p in episode if p[2] == cs.SET_PLUS])
    assert not any(p[2] == cs.SET_PLUS for p in r["presses"] if p[0] > 3100), gaps


def test_camera_target_noise_inside_a_followed_curve_never_raises_the_set_speed():
  # the camera sees the curve tighter (26.5 mph true) from 120 m out, but its least demanding curvature of the last
  # second pops up to 38 for one second every three: before the fix the step back up followed it (3-4 SET+, then 3-4
  # SET- again, over and over: review sC). The camera's target counts at its lowest: down only, as before the follower
  def cam(f, x):
    if x >= 120 or x < -400:
      return None
    return 38 * MPH if (f // 100) % 3 == 2 else 26.5 * MPH
  r = drive_follow(length=400.0, max_drop=20, frames=6000, camera=cam)
  in_view = [p for p in r["presses"] if -400 < p[1] < 120]
  assert in_view and not any(p[2] == cs.SET_PLUS for p in in_view), in_view
  assert _max_set(r, -400, 100) <= 28 + 1e-6, _max_set(r, -400, 100)     # 26.5 + its margin, the step at or below
  assert len(r["presses"]) <= 20, r["seq"]


def test_the_step_back_up_is_taps_never_a_jump_after_the_gas():
  # the driver on the gas for 0.4-0.6 s just before the follower steps back up: a SET+ then jumps to the current speed
  # (31 -> 37 in a 35 mph curve, review sF). It waits out the gas jump window and taps
  def on(st, car):
    st["gas"] = True

  def off(st, car):
    st["gas"] = False
  for g0, g1 in ((2690, 2730), (2700, 2760)):
    r = drive_follow(frames=5000, events={g0: on, g1: off})
    assert not any(p[3] == "jump" for p in r["presses"]), (g0, [p for p in r["presses"] if p[3] == "jump"])
    assert max(sp for sp, xx in zip(r["sets"], r["xs"], strict=True) if -80 < xx <= 60) <= 35 + 1e-6
    assert r["seq"].count("PM") == 0, r["seq"]


def test_the_step_back_up_stops_at_the_highest_step_not_above_the_curve_speed():
  # a 120 m curve is 36.3 mph at 2.2 m/s^2 (max drop 15): the step back up ends at 36, never 37 (it used to round up,
  # review sE), and not a step lower either
  r = drive_map(radius=120.0, max_drop=15)
  in_curve = [sp / MPH for sp, xx in zip(r["sets"], r["xs"], strict=True) if -80 < xx <= -20]
  assert max(in_curve) <= 36 + 1e-6 and min(in_curve) >= 36 - 1e-6, (min(in_curve), max(in_curve))
  assert any(p[3] == "raise" for p in r["presses"])


def test_the_step_back_up_waits_for_the_follower_to_settle():
  # the curve's radius re-matched mid-approach (91 -> 100 m 120 m out, max drop 15): the follower starts over and its
  # fresh command is above the one it had; that is no reason to raise (review s4c: 32 -> 33 at 38 mph, then three
  # SET- again). Only once the car is nearly at the curve's speed does the set speed step back up
  v_c = math.sqrt(2.2 * 100.0) / MPH

  def fields(f, x):
    return F(status=1, radius=91.0 if x > 120 else 100.0, distance=max(x, 0.0)) if x > -80 else F(status=0)
  r = drive_follow(max_drop=15, frames=5000, fields=fields)
  raises = [p for p in r["presses"] if p[3] == "raise"]
  assert raises and all(p[5] <= v_c + (CT.MAP_SETTLE + CT.MAP_SETTLE_HYST) / MPH for p in raises), raises
  assert r["seq"].count("PM") == 0, r["seq"]


# ---------------- drive 53: exits, ramps and a ramp loop at night with navigation ----------------
# Closed loop on the measured cruise with the map control, the camera curve mix and the controller, as the wrapper
# runs them (curve_no_accel from CurveMix). x is the distance driven from the start; fields(x) the map fields there.

def drive_53(fields, frames, set_mph, v_mph=None, max_drop=10, signal=None, camera=None, gas=None, lead=None):
  """signal(x): -1 / 0 / 1; camera(x): the camera curve target (m/s) or None; gas(x): the speed (m/s) the driver's foot
  holds, None off the gas; lead(x): the most the car ahead lets us drive (m/s), None for no car ahead."""
  car = T.Car(set_mph, dash_offset_mph=0.0, lag=10, accel=0.6, step=MPH, **MEASURED_CRUISE)
  car.set_speed, car.v_ego = set_mph * MPH, (v_mph or set_mph) * MPH
  logic = cs.CurveSlowdownLogic(False, max_drop)
  ctl = CTL.MapCurveControl(3.0, CTL.MAP_DECEL)
  mix = CTL.CurveMix(cs.CURVE_MARGIN)
  md = logic.max_drop_steps * logic.step
  x = 0.0
  presses, sets, vs, xs = [], [], [], []
  for f in range(frames):
    fld = fields(x)
    driver = car.set_speed + logic.offset * logic.step
    foot = gas(x) if gas else None
    mo = ctl.update(fld, driver, signal(x) if signal else 0, v_ego=car.v_ego, max_drop=md, gas=foot is not None)
    cam = camera(x) if camera else None
    active, speed, floor, budget, track, raise_ok = mix.update(cam is not None, cam if cam is not None else CTL.NO_CURVE, mo,
                                                               drop_floor=max(driver - md, cs.MIN_SET_SPEED))
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego, gas_pressed=foot is not None,
                            brake_pressed=False, standstill=False, plan_alive=True, csc_active=active, csc_speed=min(speed, 100.0),
                            other_button_active=False, driver_button=None, curve_floor=floor, curve_budget=budget,
                            curve_track=track, curve_raise=raise_ok, curve_no_accel=mix.no_accel, curve_hold=mix.firm,
                            fast_restore=mo.fast_restore)
    btn, _ = logic.update(inp)
    if btn is not None:
      presses.append((f, round(x), btn, logic.last_action, round(car.set_speed / MPH), round(car.v_ego / MPH, 1)))
      car.hold_frame(btn) if logic.last_action == "hold" else car.press(btn)
    if foot is not None:
      car.gas_frame = car.frame
    car.tick()
    if foot is not None:
      car.v_ego = max(car.v_ego, foot)
    cap = lead(x) if lead else None
    if cap is not None:
      car.v_ego = min(car.v_ego, cap)
    x += car.v_ego * T.DT
    sets.append(car.set_speed / MPH)
    vs.append(car.v_ego / MPH)
    xs.append(x)
  return dict(presses=presses, sets=sets, vs=vs, xs=xs, logic=logic, ctl=ctl)


def _at(r, x_at, key="vs"):
  return next(v for v, xx in zip(r[key], r["xs"], strict=True) if xx >= x_at)


def _plus_between(r, lo, hi):
  """our SET+ between lo and hi m driven, other than a follower's step back up to its curve's speed"""
  return [p for p in r["presses"] if lo <= p[1] < hi and p[2] == cs.SET_PLUS and p[3] != "raise"]


# Drive 53's exits with the owner's exit design (navigation's plan, the slowdown to the exit speed at the split, the
# shape confirmation, the missed exit, his decisions, the brake chime): test_exit_plan.py.


def test_after_a_missed_exit_navigations_next_exit_is_planned_on_its_own():
  # (was: a new exit after a carried one needs a new signal; no signal is needed for any exit now.) Exit A missed (the
  # shapes say "highway" past its exit point), navigation reroutes to exit B 1600 m on: B is its own plan, A is never
  # taken on again
  exit_a = 352.0
  exit_b = exit_a + 1600

  def fields(x):
    hw = dict(road_class=1, limit=70 * MPH)
    if x < exit_a - 46:
      return F(nav_exit=1, nav_exit_distance=exit_a - x, nav_curve_radius=95.0, nav_curve_distance=exit_a + 270 - x,
               nav_exit_side=1, exit_shape=1, **hw)
    if x < exit_a + 300:
      return F(exit_shape=1 if x > exit_a + 40 else 0, **hw)
    if x < exit_b:
      return F(nav_exit=1, nav_exit_distance=exit_b - x, nav_curve_radius=60.0, nav_curve_distance=exit_b + 200 - x,
               nav_exit_side=1, exit_shape=1, **hw)
    return F(**hw)
  r = drive_53(fields, 5500, 70)
  ctl = r["ctl"]
  assert ctl.plan is not None and abs(ctl.plan.exit_at - exit_b) < 2, (ctl.plan and ctl.plan.exit_at)
  assert ctl.last_plan.why == "missed" and abs(ctl.last_plan.exit_at - exit_a) < 2
  assert min(v for v, xx in zip(r["vs"], r["xs"], strict=True) if xx < exit_b - 600) >= 69   # (not in an exit lane: no slowing)


def test_a_planner_gap_does_not_stretch_the_exit_plan():
  # the exit plan is counted by the control's own odometer: vEgo x dt in update, and through a planner gap the wrapper
  # counts it on by the car's own speed (MapCurveControl.drive)
  ctl = CTL.MapCurveControl(3.0, CTL.MAP_DECEL)
  ctl.update(nav(300.0, road_class=1), 30.0, 0, v_ego=20.0)
  rem = ctl.exit_rem
  for _ in range(100):
    ctl.drive(20.0 * CTL.DT_CTRL)
  assert abs(ctl.exit_rem - (rem - 20.0)) < 1e-6, ctl.exit_rem


def test_a_street_slip_lane_keeps_the_max_drop_floor():
  # review: the ramp floor and CURVE_NO_ACCEL applied on any link, also a street to street slip lane (primary /
  # secondary link, classes 6 and 8): there the camera curve slowdown keeps the Max Drop floor (as on the street). On a
  # link connected to a highway (entered from one, or leading to one) the ramp rules stay
  cam = lambda x: 18 * MPH  # noqa: E731
  for cls in (6, 8):
    fields = lambda x, c=cls: F(road_class=5 if x < 50 else c)  # noqa: E731
    r = drive_53(fields, 2500, 50, v_mph=30, camera=cam, lead=lambda x: 30 * MPH)
    assert abs(min(r["sets"]) - 40) < 1e-6, (cls, min(r["sets"]))
  for fields in (lambda x: F(road_class=1 if x < 50 else 6),
                 lambda x: F(road_class=5) if x < 50 else F(road_class=6, on_ramp=True, ramp_type=2, ramp_to_highway=True),
                 lambda x: F(road_class=5 if x < 50 else 2)):
    r = drive_53(fields, 2500, 50, v_mph=30, camera=cam, lead=lambda x: 30 * MPH)
    assert 25 - 1e-6 <= min(r["sets"]) <= 26 + 1e-6, min(r["sets"])
  # behind a slower car (35) in a camera curve that allows 44: on the slip lane no step down toward the car's speed
  r = drive_53(lambda x: F(road_class=5 if x < 50 else 6), 2500, 50, v_mph=35, camera=lambda x: 44 * MPH,
               lead=lambda x: 35 * MPH)
  assert min(r["sets"]) >= 44 - 1e-6, min(r["sets"])


def _loop_53(link=True, gas=False):
  """Drive 53, t 1015-1060: driver's speed 50 (Max Drop 10), behind a car at 27-28 mph onto the on-ramp loop to M-59
  (71 m, ~330 m long, camera target 32.6 mph at 3 m/s^2). The map matched the link 15 m before the loop, held its curve,
  matched it 8 m before the loop and lost it inside (status 0, still on the ramp). The car ahead speeds up to 33 mph
  in the loop. gas: after the loop the driver accelerates onto the motorway (27 -> 45 mph, lifts for 40 m, then on to
  49 mph)."""
  loop_x, loop_len = 120.0, 330.0
  end_x = loop_x + loop_len

  def fields(x):
    if x >= end_x + 20:
      return F(road_class=1)
    cls = 2 if (link and x >= loop_x - 15) else 5
    if x < loop_x - 30:
      return F(road_class=cls)
    if x < loop_x - 8:
      return F(status=5, radius=71.0, distance=loop_x - x, road_class=cls)
    ramp = dict(on_ramp=link, ramp_type=2, ramp_to_highway=True, ramp_end_distance=end_x + 20 - x, road_class=cls)
    if x < loop_x + 5:
      return F(status=2 if link else 0, radius=71.0 if link else 0.0, distance=max(loop_x - x, 0.0), **ramp)
    return F(**ramp)

  def camera(x):
    return math.sqrt(3.0 * 71.0) if loop_x - 60 <= x < end_x + 30 else None    # the loop's end straightens out

  def lead(x):
    if x < loop_x:
      return 27.5 * MPH
    if x < end_x:
      return min(27.5 + 6.0 * (x - loop_x) / 200.0, 33.0) * MPH
    return None

  def foot(x):
    g0 = end_x + 32                  # on the gas right as the curve ends (drive 53: steering straight at ~1050, gas 1050.3)
    if not gas or x < g0:
      return None
    v = math.sqrt((27.0 * MPH) ** 2 + 2 * 1.6 * (x - g0))
    if v < 45 * MPH:
      return v
    g1 = g0 + ((45 * MPH) ** 2 - (27 * MPH) ** 2) / (2 * 1.6)
    if x < g1 + 40:
      return None                    # lifted for 40 m
    return min(math.sqrt((44 * MPH) ** 2 + 2 * 0.7 * (x - g1 - 40)), 49 * MPH)
  return fields, camera, lead, foot, loop_x, end_x


def test_drive_53_e3_in_a_ramp_loop_behind_a_car_the_set_speed_never_lets_the_car_speed_up():
  # the camera curve slowdown stopped at the Max Drop floor (40 mph with the driver's 50) while the car went 27-28 into
  # the loop: the cruise followed the car ahead up to 32 mph in it ("Turn Exceeds Steering Limit" twice). On a link the
  # camera takes the ramp floor, and in the curve the set speed is never left above the car's speed
  fields, camera, lead, _, loop_x, end_x = _loop_53()
  r = drive_53(fields, 3800, 50, v_mph=31, signal=None, camera=camera, lead=lead)
  v_in = _at(r, loop_x)
  loop = [(sp, v) for sp, v, xx in zip(r["sets"], r["vs"], r["xs"], strict=True) if loop_x + 40 <= xx < end_x - 30]
  assert max(v for _, v in loop) <= v_in + 1.5, (v_in, max(v for _, v in loop))       # no speeding up in the loop
  assert all(sp <= v + 1.5 for sp, v in loop), max(sp - v for sp, v in loop)            # was 11-12 mph above it
  assert min(r["sets"]) >= 25 - 1e-6


def test_drive_53_e3_negatives_the_max_drop_still_holds_on_a_normal_road_and_ramps_keep_their_floor():
  # the same curve and car ahead on a normal road (no link): the camera slowdown stops at the Max Drop floor as before
  fields, camera, lead, _, loop_x, end_x = _loop_53(link=False)
  r = drive_53(fields, 3800, 50, v_mph=31, camera=camera, lead=lead)
  assert min(r["sets"]) >= 40 - 1e-6 and min(r["sets"]) <= 40 + 1e-6, min(r["sets"])
  # behind a slower car (35 mph) in a camera curve that allows 44 (driver's 50, Max Drop 10): on a normal road the set
  # speed only goes to the camera's target (no step down toward the car's speed); on a link it is not left above it
  r = drive_53(lambda x: F(road_class=5), 2500, 50, v_mph=35, camera=lambda x: 44 * MPH, lead=lambda x: 35 * MPH)
  assert min(r["sets"]) >= 44 - 1e-6 and len(r["presses"]) <= 7, (min(r["sets"]), r["presses"])
  r = drive_53(lambda x: F(road_class=2), 2500, 50, v_mph=35, camera=lambda x: 44 * MPH, lead=lambda x: 35 * MPH)
  assert 35 - 1e-6 <= min(r["sets"]) <= 36 + 1e-6, min(r["sets"])
  # a link with a camera target of 18 mph, the car behind a car at 30 mph: never under the ramp floor
  r = drive_53(lambda x: F(road_class=2), 2500, 50, v_mph=30, camera=lambda x: 18 * MPH, lead=lambda x: 30 * MPH)
  assert 25 - 1e-6 <= min(r["sets"]) <= 26 + 1e-6, min(r["sets"])
  # a followed ramp curve tighter than 25 mph (45 m): never under the ramp floor either
  r = drive_53(lambda x: F(status=2, radius=45.0, distance=max(600.0 - x, 0.0), on_ramp=True, ramp_type=1, road_class=2)
               if x < 700 else F(on_ramp=True, ramp_type=1, road_class=2), 5000, 60)
  assert 25 - 1e-6 <= min(r["sets"]) <= 26 + 1e-6, min(r["sets"])
  assert _at(r, 600.0) <= 26.5, _at(r, 600.0)


def test_drive_53_e4_after_the_ramp_loop_the_restore_runs_on_the_gas():
  # after the loop the set speed stayed 29 with the driver's 50 while he accelerated onto M-59 on the gas (the curve's
  # "over" timer was frozen on the gas): when he lifted the car slowed toward 29. Now the restore goes on the gas too:
  # SET+ jumps to the current speed (never one that could land above the driver's speed), so lifting holds the speed
  fields, camera, lead, foot, loop_x, end_x = _loop_53(gas=True)
  r = drive_53(fields, 6500, 50, v_mph=31, camera=camera, lead=lead, gas=foot)
  g0 = end_x + 32
  lift = next(xx for xx in r["xs"] if xx > g0 and foot(xx) is None)
  assert _at(r, lift, "sets") >= _at(r, lift) - 3.0, (_at(r, lift, "sets"), _at(r, lift))
  after_lift = [v for v, xx in zip(r["vs"], r["xs"], strict=True) if lift <= xx < lift + 40]
  assert min(after_lift) >= _at(r, lift) - 1.0, (min(after_lift), _at(r, lift))       # no slowing toward 29 (was -0.77 m/s^2)
  assert max(r["sets"]) <= 50 + 1e-6 and abs(r["sets"][-1] - 50) < 1e-6, (max(r["sets"]), r["sets"][-1])
  on_gas = [p for p in r["presses"] if g0 + 5 < p[1] < lift]
  assert len(on_gas) >= 3 and all(p[2] == cs.SET_PLUS for p in on_gas), on_gas          # the restore ran on the gas


def test_on_the_gas_a_flickering_camera_curve_waits_the_flicker_delay():
  # review: the restore on the gas (CURVE_GAS_RESTORE) waited RESTORE_DELAY (0.5 s), so a camera curve target that
  # flickered off for 0.8-1.6 s mid-curve made a SET+ jump to the current speed and taps there. A camera curve on the
  # gas waits FLICKER_RESTORE_DELAY (a followed map curve keeps 0.5 s: the E4 test above)
  for off_s in (0.8, 1.6):
    def camera(x, off_s=off_s):
      return 33 * MPH if (100 <= x < 1000 and not (600 <= x < 600 + off_s * 19.0)) else None
    r = drive_53(lambda x: F(road_class=1), 9000, 50, camera=camera, gas=lambda x: 42 * MPH if 500 <= x < 1300 else None)
    mid = [p for p in r["presses"] if 600 <= p[1] < 1000]
    assert mid == [], (off_s, mid)
    assert abs(r["sets"][-1] - 50) < 1e-6                         # the restore on the gas once the curve is over
  # off for longer than the flicker delay: the curve is over, the restore goes on the gas
  camera = lambda x: 33 * MPH if (100 <= x < 1000 and not (600 <= x < 600 + 2.5 * 19.0)) else None  # noqa: E731
  r = drive_53(lambda x: F(road_class=1), 9000, 50, camera=camera, gas=lambda x: 42 * MPH if 500 <= x < 1300 else None)
  assert [p for p in r["presses"] if 600 <= p[1] < 700 and p[2] == cs.SET_PLUS]


def _loop_scene():
  """An on-ramp leaving a street, a right loop of 71 m turning 243 deg, joining a motorway (drive 53, onto M-59)."""
  st = G.way(20, G.straight(0, -300, 0, 300), name="South Rochester Road", cls=5, speed_mph=50)
  loop = G.arc(71.0, 60, 71.0, 180, 180 - 243.0)
  ex, ey = loop[-1]
  pts = G.straight(0, 0, 0, 60)[:-1] + loop + G.straight(ex, ey, 243.0, 80)[1:]
  ramp = G.way(21, pts, name="", cls=2, speed_mph=45, oneway=True)
  h = math.radians(243.0)
  mw = G.way(22, G.straight(pts[-1][0] - math.sin(h) * 200, pts[-1][1] - math.cos(h) * 200, 243.0, 1200), name="", cls=1, speed_mph=70)
  mw.ref = "M 59"
  ramp.lat[-1], ramp.lon[-1] = mw.lat[10], mw.lon[10]
  ramp.end = B.key(mw.lat[10], mw.lon[10])
  ramp.lat[0], ramp.lon[0] = st.lat[-1], st.lon[-1]
  ramp.start = st.end = B.key(st.lat[-1], st.lon[-1])
  return {w.id: w for w in (st, ramp, mw)}, pts


def test_drive_53_a_ramp_loop_is_the_curve_we_are_in_until_driven_and_never_ahead_again():
  # the straight-line "ahead" test lost the loop 1 s after its entry (its exit point beside / behind the car: status 0
  # for 9 s in the loop) and, after ~250 deg, found its own entry "ahead" again (173 m: a curve behind us kept the set
  # speed at 29 at the end of the ramp). With the distance driven (the planner's speed x time) it is the one we are in
  # from its entry until its length is driven, then never again on this ramp
  ways, pts = _loop_scene()
  root, curves, _ = write_files(ways)
  loop = [c for c in curves if c.get("ramp")]
  assert len(loop) == 1 and abs(loop[0]["r"] - 71) < 3 and loop[0]["len"] > 250
  m = planner(root)
  travelled, seen = 0.0, []
  for (x0, y0), (x1, y1) in zip(pts, pts[1:], strict=False):
    seg = math.hypot(x1 - x0, y1 - y0)
    brg = (math.degrees(math.atan2(x1 - x0, y1 - y0)) + 360.0) % 360.0
    for i in range(max(1, int(seg / 0.66))):
      la, lo = pos(x0 + (x1 - x0) * i * 0.66 / seg, y0 + (y1 - y0) * i * 0.66 / seg)
      m.update(la, lo, brg, 21, True, gps={"dt": 0.05, "v_ego": 13.2})
      travelled += 0.66
      seen.append((travelled, m.status, m.distance if m.curve is not None else None))
  entry = next(t for t, st, d in seen if st == MC.STATUS_RAMP_CURVE and d == 0.0)
  inside = [(st, d) for t, st, d in seen if entry <= t < entry + loop[0]["len"] - 20]
  assert all(st == MC.STATUS_RAMP_CURVE and d == 0.0 for st, d in inside), sorted({(st, d) for st, d in inside if d != 0.0 or st != 2})[:5]
  after = [(st, d) for t, st, d in seen if t >= entry + loop[0]["len"] + 15]
  assert after and all(st == MC.STATUS_NONE for st, _ in after), after[:5]          # not "ahead" again
  # without the distance driven (no gps speed: the offline callers) the old geometric rules are unchanged
  m = planner(root)
  la, lo = pos(0, 30)
  m.update(la, lo, 0.0, 21, True)
  assert m.status == MC.STATUS_RAMP_CURVE and m.distance > 15 and not m.entered


# ---------------- curve memory ----------------

def _pass(r, seconds=6.0, v=18.0, lat=True, driver=False, cmd=0.5):
  return [(v, 1.0 / r if r else 0.0, lat, driver, cmd)] * int(seconds / 0.05)


def test_a_clean_pass_measures_the_radius_the_car_drove():
  assert abs(MC.curve_pass_radius(_pass(150)) - 150) < 0.5
  assert MC.curve_pass_radius(_pass(0)) == MC.MEMORY_MAX_R          # did not really turn
  assert MC.curve_pass_radius(_pass(150, seconds=1.0)) is None        # too short
  # passes that do not count: the steering at its limit (the car goes wide), the driver steering, lateral off
  assert MC.curve_pass_radius(_pass(150)[:60] + _pass(150, cmd=1.0)[:5] + _pass(150)[:60]) is None
  assert MC.curve_pass_radius(_pass(150, driver=True)) is None
  assert MC.curve_pass_radius(_pass(150, lat=False)) is None


def test_curve_memory_needs_two_agreeing_passes_and_survives_a_reload(tmp_path):
  path = str(tmp_path / "mem.json")
  mem = MC.CurveMemory(path)
  c = {"apex": (42.6, -83.1), "r": 129.0, "name": "Test Road"}
  mem.add_pass(c, 390.0, t=1000.0)
  assert mem.radius(c) is None                                         # one pass is not enough
  mem.add_pass(c, 425.0, t=1010.0)
  assert mem.radius(c) is None                                         # 10 s later: the same pass (divided road)
  mem.add_pass(c, 420.0, t=5000.0)
  assert mem.radius(c) in (420.0, 425.0), mem.radius(c)               # the 390 pass was replaced by its repeat
  moved = {"apex": (42.6 + 10 / 111320, -83.1), "r": 131.0}            # the same curve after a rebuild (10 m off)
  assert mem.radius(moved) is not None
  other = {"apex": (42.6 + 60 / 111320, -83.1), "r": 131.0}
  assert mem.radius(other) is None                                     # 60 m away: another curve
  import time as _t
  for _ in range(100):
    if not mem.saving:
      break
    _t.sleep(0.01)
  again = MC.CurveMemory(path)
  assert again.radius(c) is not None
  # passes that disagree are not used; the result is clamped to a range around the map's radius
  c2 = {"apex": (42.7, -83.2), "r": 100.0}
  mem.add_pass(c2, 150.0, t=1.0)
  mem.add_pass(c2, 300.0, t=500.0)
  assert mem.radius(c2) is None
  c3 = {"apex": (42.8, -83.3), "r": 100.0}
  mem.add_pass(c3, 2000.0, t=1.0)
  mem.add_pass(c3, 2000.0, t=500.0)
  assert mem.radius(c3) == 500.0                                       # at most 5x the map's


def test_the_planner_uses_the_learned_radius(tmp_path):
  root, curves, _ = write_files(G.curve_road(r=115, turn=40, split=True))
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  for k, r in enumerate((160.0, 165.0)):
    mem.add_pass(curves[0], r, t=1000.0 * (k + 1))
  m = MC.MapCurves(MC.MapCurveStore(root, background=False), memory=mem)
  la, lo = pos(0, -250)
  m.update(la, lo, 0.0, 1, True)
  assert m.curve is not None and m.radius in (160.0, 165.0), m.radius


def test_a_pass_with_the_steering_maxed_out_caps_the_curve_speed(tmp_path):
  # drive 3d, the 30 mph West Utica curve: the steering stayed at its limit for ~5 s at 37.5 mph
  maxed = _pass(133, seconds=3.0, v=16.7) + _pass(133, seconds=2.0, v=16.7, cmd=1.0) + _pass(133, seconds=2.0, v=16.7)
  assert abs(MC.curve_pass_saturation(maxed) - 16.7) < 1e-6
  # the driver steering against openpilot's line also maxes the command: that says nothing about the curve
  assert MC.curve_pass_saturation(_pass(133, seconds=3.0) + _pass(133, seconds=2.0, cmd=1.0, driver=True)) is None
  # the speed comes from the longest run at the limit, not from a short blip on entry
  blip = _pass(133, seconds=1.0, v=18.8) + _pass(133, seconds=0.2, v=18.8, cmd=1.0) + _pass(133, seconds=1.0, v=15.2) + \
    _pass(133, seconds=1.5, v=15.2, cmd=1.0)
  assert abs(MC.curve_pass_saturation(blip) - 15.2) < 1e-6
  assert MC.curve_pass_saturation(_pass(133)) is None                                   # never maxed out
  assert MC.curve_pass_saturation(_pass(133)[:60] + _pass(133, cmd=1.0)[:5] + _pass(133)[:60]) is None   # a blip
  assert MC.curve_pass_saturation(_pass(133, cmd=1.0, lat=False)) is None              # openpilot not steering
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  c = {"apex": (42.6, -83.1), "r": 117.0, "adv": 30 * MC.MPH}
  assert mem.speed_cap(c) is None
  mem.add_saturated_pass(c, 37.5 * MC.MPH, t=1000.0)
  assert abs(mem.speed_cap(c) - 0.9 * 37.5 * MC.MPH) < 0.01                           # ~33.8 mph next time
  mem.add_saturated_pass(c, 37.5 * MC.MPH, t=1010.0)
  assert abs(mem.speed_cap(c) - 0.9 * 37.5 * MC.MPH) < 0.01                           # same pass (other carriageway)
  mem.add_saturated_pass(c, 32.0 * MC.MPH, t=5000.0)
  assert abs(mem.speed_cap(c) - 30 * MC.MPH) < 0.01                                     # never below the yellow sign
  mem.add_pass(c, 140.0, t=9000.0, v=28.2 * MC.MPH)
  assert abs(mem.speed_cap(c) - 30.5 * MC.MPH) < 0.01         # clean at the cap (true speed ~2 mph under the dash): up
  mem.add_pass(c, 140.0, t=13000.0, v=25.0 * MC.MPH)
  assert abs(mem.speed_cap(c) - 30.5 * MC.MPH) < 0.01                                   # clean but much slower: no news
  mem.add_pass(c, 140.0, t=17000.0, v=40.0 * MC.MPH)
  assert abs(mem.speed_cap(c) - 35.5 * MC.MPH) < 0.01         # clean well above the cap (no slowdown): up to 5 mph
  unsigned = {"apex": (42.7, -83.2), "r": 60.0}
  mem.add_saturated_pass(unsigned, 26.0 * MC.MPH, t=1.0)
  assert abs(mem.speed_cap(unsigned) - 25 * MC.MPH) < 0.01                              # floor: 25 mph


def test_the_planner_slows_more_for_a_capped_curve(tmp_path):
  root, curves, _ = write_files(G.curve_road(r=115, turn=40, split=True))
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  m = MC.MapCurves(MC.MapCurveStore(root, background=False), memory=mem)
  la, lo = pos(0, -250)
  m.update(la, lo, 0.0, 1, True)
  assert m.curve is not None and abs(m.radius - curves[0]["r"]) < 1e-6
  before = MC.curve_speed_limit(m.radius, m.advisory)
  mem.add_saturated_pass(curves[0], before, t=1.0)
  m.update(la, lo, 0.0, 1, True)
  assert abs(MC.curve_speed_limit(m.radius, m.advisory) - 0.9 * before) < 0.01, (m.radius, before)


def test_tiles_are_parsed_in_steps_with_the_same_result():
  import json
  import threading
  import time as _t
  tile = {"version": 3, "curves": [{"apex": [42.6, -83.1], "r": 120.5, "name": "A, [b] ]}", "adv": 0.0}] * 900,
          "ramps": [], "chains": [{"w": [[1, True, 0.0]], "len": 12.5}] * 700,
          "roads": [{"w": 1, "c": 2, "k": 9, "p": [1, -2, 3, 4]}] * 3000, "meta": {"built": "x"}, "n": None}
  for text in (json.dumps(tile), json.dumps(tile, indent=1), json.dumps({}), json.dumps({"curves": []})):
    assert MC.load_json_in_steps(text) == json.loads(text)
  # other threads get to run while a big tile is parsed (the planner loop must never stall for the whole parse). A tile
  # big enough that one parse without yielding would stall the other thread far longer than 50 ms; a stall that long
  # may still happen now and then on a loaded machine (the scheduler, the garbage collector), so a few are allowed
  # (the parser not yielding would give one per parse)
  big = {k: (v * 10 if isinstance(v, list) else v) for k, v in tile.items()}
  text, ticks, done = json.dumps(big), [], threading.Event()
  t0 = _t.perf_counter()
  json.loads(text)
  assert _t.perf_counter() - t0 > 0.005                  # (the plain parse takes a while: the check below means something)

  def other():
    while not done.is_set():
      ticks.append(_t.perf_counter())
      _t.sleep(0.001)
  th = threading.Thread(target=other)
  th.start()
  parses = 8
  for _ in range(parses):
    MC.load_json_in_steps(text)
  done.set()
  th.join()
  gaps = [b - a for a, b in zip(ticks, ticks[1:], strict=False)]
  long = [round(g, 3) for g in gaps if g >= 0.05]
  assert len(long) <= parses // 4, long


def test_the_background_loader_uses_the_stepped_parser(tmp_path):
  root, curves, _ = write_files(G.curve_road(r=115, turn=40, split=True))
  calls = []
  orig = MC.load_json_in_steps
  MC.load_json_in_steps = lambda text: calls.append(1) or orig(text)
  try:
    store = MC.MapCurveStore(root, background=True)
    la, lo = pos(0, -250)
    store.update(la, lo)
    import time as _t
    for _ in range(200):
      if not store.loading:
        break
      _t.sleep(0.01)
  finally:
    MC.load_json_in_steps = orig
  assert calls and store.loaded


# ---------------- camera check of map curves (CurveVeto) and one trusted gentle pass ----------------

def _view(straight_m, r_cam, length=160.0, ystd=0.3, **kw):
  """The model's path: straight for straight_m, then an arc of r_cam to the left (None: straight on)."""
  s = [5.0 * i for i in range(int(length / 5.0) + 1)]
  psi = [0.0 if (r_cam is None or x <= straight_m) else (x - straight_m) / r_cam for x in s]
  kw.setdefault("v_ego", 18.0)
  return MC.CameraView(s, psi, [ystd] * len(s), **kw)


def _false_kink(memory=None):
  """A 130 m map curve (the map's radius), 40 m ahead of the car, apex ~85 m ahead."""
  root, curves, _ = write_files(G.curve_road(r=130, turn=40, split=True))
  m = MC.MapCurves(MC.MapCurveStore(root, background=False), memory=memory)
  la, lo = pos(0, -40)
  return m, curves, (la, lo, 0.0, 1, True)


def _frames(m, where, view, n):
  for _ in range(n):
    m.update(*where, camera=view)


def test_the_camera_ignores_a_map_curve_it_sees_much_gentler_after_one_second():
  m, curves, where = _false_kink()
  gentle = _view(40.0, 400.0)
  _frames(m, where, gentle, MC.VETO_CONFIRM - 1)
  assert m.curve is not None and m.veto_status == 0                   # 19 frames: still slowing for it
  _frames(m, where, gentle, 1)
  assert m.curve is None and m.status == MC.STATUS_NONE and m.veto_status == 1
  assert 300 <= m.veto_radius <= 500, m.veto_radius
  ev = m.veto.events[-1]
  assert ev["what"] == "veto" and 300 <= ev["r_cam"] <= 500 and abs(ev["r_map"] - curves[0]["r"]) < 1e-6
  # the car side sees the curve gone: the map reason ends
  f = CTL.MapFields(mode=CTL.ON, status=m.status, distance=m.distance, radius=m.radius)
  assert f.status == 0
  # without a camera view nothing is ignored (and the old call still works)
  m2, _, where2 = _false_kink()
  for _ in range(40):
    m2.update(*where2)
  assert m2.curve is not None and m2.veto_status == 0


def test_the_camera_check_waits_when_it_cannot_see_well():
  m, _, _ = _false_kink()
  far = (*pos(0, -150), 0.0, 1, True)                                  # apex ~195 m ahead
  _frames(m, far, _view(150.0, 400.0, length=250.0), 100)
  assert m.curve is not None and m.veto_status == 0
  for blind in (_view(40.0, 400.0, ystd=2.0), _view(40.0, 400.0, red=True), _view(40.0, 400.0, lead_d=60.0),
                _view(40.0, 400.0, blinker=True), _view(40.0, 400.0, lane_change=True), _view(40.0, 400.0, v_ego=6.0),
                _view(40.0, 400.0, length=80.0)):                      # a short path (a hill crest)
    m, _, where = _false_kink()
    _frames(m, where, blind, 100)
    assert m.curve is not None and m.veto_status == 0
    assert all(st["n"] == 0 for st in m.veto.state.values())          # blind frames neither count nor reset
  # blind frames in between hold the count: 10 seeing + 50 blind + 10 seeing = ignored
  m, _, where = _false_kink()
  _frames(m, where, _view(40.0, 400.0), 10)
  _frames(m, where, _view(40.0, 400.0, red=True), 50)
  _frames(m, where, _view(40.0, 400.0), 10)
  assert m.curve is None and m.veto_status == 1


def test_no_veto_when_the_camera_agrees_or_is_not_sure_enough():
  for r_cam in (130.0, 1.8 * 130.0):
    m, _, where = _false_kink()
    _frames(m, where, _view(40.0, r_cam), 60)
    assert m.curve is not None and m.veto_status == 0, r_cam
  # one frame that disagrees starts the count again
  m, _, where = _false_kink()
  _frames(m, where, _view(40.0, 400.0), 15)
  _frames(m, where, _view(40.0, 130.0), 1)
  _frames(m, where, _view(40.0, 400.0), 15)
  assert m.curve is not None


def test_an_ignored_curve_seen_sharp_again_governs_again_and_two_flips_end_it():
  m, _, where = _false_kink()
  gentle, sharp = _view(40.0, 400.0), _view(40.0, 130.0)
  _frames(m, where, gentle, MC.VETO_CONFIRM)
  assert m.curve is None
  _frames(m, where, _view(40.0, 200.0), 30)                           # between the two factors: stays ignored
  assert m.curve is None
  _frames(m, where, sharp, MC.REARM_CONFIRM - 1)
  assert m.curve is None
  _frames(m, where, sharp, 1)
  assert m.curve is not None and m.veto_status == 2 and m.veto.events[-1]["what"] == "rearm"
  _frames(m, where, gentle, MC.VETO_CONFIRM)
  assert m.curve is None
  _frames(m, where, sharp, MC.REARM_CONFIRM)
  assert m.curve is not None and [e["what"] for e in m.veto.events[-2:]] == ["rearm", "give_up"]
  _frames(m, where, gentle, 100)                                       # never ignored again this drive
  assert m.curve is not None and m.veto_status == 2
  m.veto.reset_drive()                                                 # the next drive checks it afresh
  _frames(m, where, gentle, MC.VETO_CONFIRM)
  assert m.curve is None


def test_ramps_signed_capped_and_learned_curves_are_never_ignored(tmp_path):
  gentle = _view(40.0, 400.0)
  for change in ("ramp", "adv"):
    m, _, where = _false_kink()
    m.update(*where)
    m.curve[change] = True if change == "ramp" else 35 * MPH
    _frames(m, where, gentle, 60)
    assert m.curve is not None and m.veto_status == 0, change
  mem = MC.CurveMemory(str(tmp_path / "cap.json"))
  m, curves, where = _false_kink(memory=mem)
  mem.add_saturated_pass(curves[0], 13.0, t=1.0)                     # capped below the map's speed
  _frames(m, where, gentle, 60)
  assert m.curve is not None and m.source == 2
  mem2 = MC.CurveMemory(str(tmp_path / "learned.json"))
  m, curves, where = _false_kink(memory=mem2)
  mem2.add_pass(curves[0], 150.0, t=1.0, v=15.0)
  mem2.add_pass(curves[0], 155.0, t=500.0, v=15.0)
  _frames(m, where, gentle, 60)
  assert m.curve is not None and m.source == 1


def _two_curves():
  """Straight north, a 110 m left curve (35 deg), 200 m straight, a 150 m right curve (40 deg), straight."""
  pts = G.straight(0, -300, 0, 300)
  a = G.arc(-110, 0, 110, 0, 35)
  pts += a[1:]
  b = G.straight(*a[-1], -35, 200)[1:]
  pts += b
  bx, by = b[-1]
  cx, cy = bx + 150 * math.sin(math.radians(55)), by + 150 * math.cos(math.radians(55))
  a0 = math.degrees(math.atan2(by - cy, bx - cx))
  c = G.arc(cx, cy, 150, a0, a0 - 40)
  pts += c[1:]
  pts += G.straight(*c[-1], 5, 300)[1:]
  return {1: G.way(1, pts)}


def test_when_the_first_curve_is_ignored_the_next_one_governs_and_the_pass_is_still_measured():
  root, curves, _ = write_files(_two_curves())
  assert len(curves) == 2, curves
  first = min(curves, key=lambda c: c["s_a"])
  m = MC.MapCurves(MC.MapCurveStore(root, background=False))
  where = (*pos(0, -40), 0.0, 1, True)
  _frames(m, where, _view(40.0, 400.0), MC.VETO_CONFIRM)
  assert m.curve is not None and m.curve["s_a"] > first["s_a"] and m.veto_status == 1   # the real one after it governs
  # inside the ignored curve: still the curve being driven (curve memory measures the pass)
  ang = math.radians(15)
  la, lo = pos(-110 + 110 * math.cos(ang), 110 * math.sin(ang))
  m.update(la, lo, 345.0, 1, True, camera=_view(0.0, 400.0))
  assert m.inside is not None and m.inside["s_a"] == first["s_a"] and m.veto.is_vetoed((m.chain_id, first["s_a"]))


def test_a_curve_that_starts_gentle_and_tightens_is_not_judged_before_its_apex_is_seen():
  root, curves, _ = write_files(G.curve_road(r=130, turn=80, split=True))    # apex ~90 m into the curve
  m = MC.MapCurves(MC.MapCurveStore(root, background=False))
  where = (*pos(0, -20), 0.0, 1, True)
  _frames(m, where, _view(20.0, 400.0), 60)                           # the model sees only the gentle start
  assert m.curve is not None and m.veto_status == 0


def test_camera_view_from_the_model_message():
  from types import SimpleNamespace as N

  class Enum(int):
    @property
    def raw(self):
      return int(self)

  xs = [5.0 * i for i in range(33)]
  md = N(position=N(x=xs, y=[0.0] * 33, yStd=[0.2] * 33), orientation=N(z=[0.0] * 33), confidence=Enum(2),
         meta=N(laneChangeState=Enum(0)))

  class SM(dict):
    alive = {"modelV2": True}
    valid = {"modelV2": True}

  sm = SM(modelV2=md, carState=N(leftBlinker=False, rightBlinker=False, vEgo=20.0),
          radarState=N(leadOne=N(status=True, dRel=55.0)))
  v = MC.camera_view(sm)
  assert v is not None and v.end == 160.0 and not v.red and v.lead_d == 55.0 and v.radius(0, 150) == float("inf")
  md.confidence = Enum(0)
  assert MC.camera_view(sm).red
  sm.alive = {"modelV2": False}
  assert MC.camera_view(sm) is None
  assert MC.camera_view(None) is None


def test_one_trusted_gentle_pass_corrects_a_map_curve(tmp_path):
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  c = {"apex": (42.6, -83.1), "r": 129.0}
  mem.add_pass(c, 400.0, t=1000.0, v=18.0, flags=["full"])
  assert mem.radius(c) == 400.0                                        # one full, fast pass well over the map's
  for name, kw in (("gentler but close", dict(r=180.0)), ("slow", dict(v=8.0)), ("never turned", dict(r=2000.0)),
                   ("not full", dict(flags=[]))):
    m2 = MC.CurveMemory(str(tmp_path / f"{name}.json"))
    args = dict(r=400.0, v=18.0, flags=["full"])
    args.update(kw)
    m2.add_pass(c, args["r"], t=1000.0, v=args["v"], flags=args["flags"])
    assert m2.radius(c) is None, name
  mem.add_pass(c, 150.0, t=5000.0, v=18.0, flags=["full"])
  assert mem.radius(c) is None                                         # a sharper pass: back to the map's radius
  mem.add_pass(c, 390.0, t=9000.0, v=18.0, flags=["full"])
  assert mem.radius(c) == 390.0                                        # a gentle one again: trusted again
  far = {"apex": (42.7, -83.1), "r": 60.0}
  mem.add_pass(far, 1000.0, t=1.0, v=18.0, flags=["full"])
  assert mem.radius(far) == 300.0                                      # still at most 5x the map's


def test_an_old_memory_file_still_loads(tmp_path):
  path = tmp_path / "old.json"
  c = {"apex": (42.6, -83.1), "r": 129.0}
  path.write_text(json.dumps({"version": 1, "curves": [{"lat": 42.6, "lon": -83.1, "r_map": 129.0, "name": "",
                                                         "passes": [[160.0, 1000.0], [165.0, 5000.0]]}]}))
  mem = MC.CurveMemory(str(path))
  assert mem.radius(c) in (160.0, 165.0)
  mem.add_pass(c, 170.0, t=9000.0, v=15.0, flags=["full"])
  assert len(mem._find(42.6, -83.1)["passes"]) == 3 and mem.radius(c) in (165.0, 170.0)


# ---------------- curve memory per direction of travel (drive 50) ----------------

def _wait_saved(mem):
  import time as _t
  for _ in range(200):
    if not mem.saving:
      break
    _t.sleep(0.01)


def _two_way_curve(dx=0.0, dy=0.0, reversed_=False):
  """A curve record as the files hold it (one for both ways): entry a 40 m south-west of the apex, exit b 40 m
  north-east of it, shifted by (dx, dy) m (a rebuild) and with a and b swapped (a reversed chain)."""
  def p(x, y):
    return (42.6 + (y + dy) / 111195.0, -83.1 + (x + dx) / (111195.0 * math.cos(math.radians(42.6))))
  c = {"a": p(-30, -30), "b": p(30, 30), "apex": p(0, 0), "r": 91.0, "len": 95.0, "name": "West Utica", "s_a": 0.0}
  if reversed_:
    c["a"], c["b"] = c["b"], c["a"]
  return c


def test_the_direction_of_travel_is_the_curves_entry_to_exit_bearing():
  c = _two_way_curve()
  fwd, back = MC.curve_direction(c, True), MC.curve_direction(c, False)
  assert abs(fwd - 45.0) < 1.0 and abs(back - 225.0) < 1.0, (fwd, back)         # a -> b north-east, b -> a south-west
  assert MC.same_direction(fwd, 10.0) and MC.same_direction(fwd, 100.0) and not MC.same_direction(fwd, back)
  assert MC.same_direction(350.0, 10.0) and not MC.same_direction(90.0, 271.0)  # around north; just over 90 deg apart
  # a rebuilt file: the points a few m off and the chain the other way round give the same direction
  rebuilt = _two_way_curve(dx=4.0, dy=-3.0, reversed_=True)
  assert abs(MC.curve_direction(rebuilt, False) - fwd) < 10.0 and abs(MC.curve_direction(rebuilt, True) - back) < 10.0
  # unknown: no road match, a curve dict without its ends (the offline tests), a full loop (entry on its exit)
  assert MC.curve_direction(c, None) is None and MC.curve_direction({"apex": c["apex"], "r": 91.0}, True) is None
  assert MC.curve_direction({**c, "b": c["a"]}, True) is None


def test_drive_50_a_cap_from_one_direction_does_not_limit_the_other(tmp_path):
  # West Utica (map 91 m), ~35.4 mph both ways: the right turn (one way) maxed the steering for 3.5 s; the left turn
  # (the usual way) kept up. One shared record capped the usual way too, and the next clean pass that way lifted the
  # cap the right turn had earned.
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  c = _two_way_curve()
  usual, other = MC.curve_direction(c, True), MC.curve_direction(c, False)
  mem.add_saturated_pass(c, 35.4 * MPH, t=1000.0, direction=other)
  cap = mem.speed_cap(c, other)
  assert abs(cap - 0.9 * 35.4 * MPH) < 0.01
  assert mem.speed_cap(c, usual) is None                                # the usual way is not capped
  mem.add_pass(c, 119.0, t=5000.0, v=35.0 * MPH, direction=usual)      # a clean pass the usual way ...
  mem.add_pass(c, 121.0, t=9000.0, v=35.0 * MPH, direction=usual)
  assert mem.speed_cap(c, other) == cap                                 # ... leaves the other way's cap alone
  assert mem.radius(c, usual) in (119.0, 121.0) and mem.radius(c, other) is None   # and its radius is its own
  mem.add_pass(c, 102.0, t=13000.0, v=0.9 * 35.4 * MPH, direction=other)   # clean at the cap the other way: searching up
  assert abs(mem.speed_cap(c, other) - (cap + MC.CAP_RISE)) < 0.01
  assert mem.radius(c, usual) in (119.0, 121.0)
  # the planner looks the curve up the same way MapCurves plans it: a rebuilt file (apex 4 m off, chain reversed)
  rebuilt = _two_way_curve(dx=4.0, dy=-3.0, reversed_=True)
  assert mem.radius(rebuilt, MC.curve_direction(rebuilt, False)) in (119.0, 121.0)
  assert abs(mem.speed_cap(rebuilt, MC.curve_direction(rebuilt, True)) - (cap + MC.CAP_RISE)) < 0.01
  assert mem.speed_cap(rebuilt, MC.curve_direction(rebuilt, False)) is None
  assert len(mem.entries) == 2 and sorted(e["dir"] for e in mem.entries) == sorted((usual, other))


def test_a_record_without_a_direction_stands_in_until_that_way_is_driven_and_is_never_changed(tmp_path):
  path = tmp_path / "old.json"
  c = _two_way_curve()
  usual, other = MC.curve_direction(c, True), MC.curve_direction(c, False)
  old = {"lat": c["apex"][0], "lon": c["apex"][1], "r_map": 91.0, "name": "West Utica",
         "passes": [[118.0, 1000.0, 15.7, ["full"]], [104.0, 5000.0, 15.8, ["full"]], [120.0, 9000.0, 15.7, ["full"]]],
         "cap": 14.15, "cap_t": 3000.0, "cap_hits": 1}
  path.write_text(json.dumps({"version": 1, "curves": [old]}))
  mem = MC.CurveMemory(str(path))
  assert mem.radius(c, usual) == 118.0 and mem.radius(c, other) == 118.0 and mem.speed_cap(c, usual) == 14.15
  mem.add_pass(c, 119.0, t=13000.0, v=35.0 * MPH, direction=usual)        # the first pass kept per direction
  # the old radius (it can only make the curve faster) stops at once: one pass of its own is nothing yet; the old cap
  # (it can only make the curve slower) stays on the safe side until this way has two clean passes or a cap of its own
  assert mem.radius(c, usual) is None and mem.speed_cap(c, usual) == 14.15
  assert mem.radius(c, other) == 118.0 and mem.speed_cap(c, other) == 14.15   # the old record still stands in the other way
  mem.add_pass(c, 121.0, t=17000.0, v=35.0 * MPH, direction=usual)        # two clean passes: this way decides alone
  assert mem.radius(c, usual) in (119.0, 121.0) and mem.speed_cap(c, usual) is None
  mem.add_saturated_pass(c, 35.0 * MPH, t=21000.0, direction=other)       # a cap of its own: the old one steps back
  assert abs(mem.speed_cap(c, other) - 0.9 * 35.0 * MPH) < 0.01 and mem.radius(c, other) is None
  mem.add_pass(c, 101.0, t=25000.0, v=25.0 * MPH, direction=other)
  assert abs(mem.speed_cap(c, other) - 0.9 * 35.0 * MPH) < 0.01       # one pass, well under its own cap: unchanged
  mem.note_veto(c, usual)
  legacy = [e for e in mem.entries if "dir" not in e]
  assert len(legacy) == 1 and legacy[0] == old                           # never written to, still in the file
  assert len(mem.entries) == 3
  _wait_saved(mem)
  data = json.loads(path.read_text())
  assert data["version"] == 2 and old in data["curves"]
  assert all("lat" in e and "lon" in e and isinstance(e.get("passes", []), list) for e in data["curves"])  # old readers
  again = MC.CurveMemory(str(path))
  assert again.radius(c, other) is None and abs(again.speed_cap(c, other) - 0.9 * 35.0 * MPH) < 0.01
  assert again.radius(c, usual) in (119.0, 121.0) and again._find(*c["apex"], usual)["veto_hits"] == 1
  assert again._find(*c["apex"]) == old                                  # no direction (the tools): the old record
  # an old record with a cap but no usable radius: the cap alone carries over to the first directional pass
  path2 = tmp_path / "cap_only.json"
  path2.write_text(json.dumps({"version": 1, "curves": [{"lat": c["apex"][0], "lon": c["apex"][1], "passes": [],
                                                          "cap": 14.15, "cap_t": 3000.0, "cap_hits": 1}]}))
  mem2 = MC.CurveMemory(str(path2))
  assert mem2.speed_cap(c, usual) == 14.15 and mem2.radius(c, usual) is None
  mem2.add_pass(c, 119.0, t=13000.0, v=33.0 * MPH, direction=usual)
  assert mem2.speed_cap(c, usual) == 14.15 and len(mem2.entries) == 2
  mem2.add_pass(c, 121.0, t=17000.0, v=33.0 * MPH, direction=usual)
  assert mem2.speed_cap(c, usual) is None and mem2.radius(c, usual) in (119.0, 121.0)


def test_a_divided_road_is_still_one_pass_per_direction(tmp_path):
  # two curve records ~15 m apart, one per carriageway; the car's pass matches both in turn. Both are driven the
  # car's way (the road matcher never puts the car on the oncoming one-way carriageway; a two-way one is matched by
  # heading), so both carry the car's direction and the second is the same pass again
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  east, west = _two_way_curve(), _two_way_curve(dy=15.0)
  d = MC.curve_direction(east, True)
  assert abs(MC.curve_direction(west, True) - d) < 1.0
  mem.add_pass(east, 120.0, t=1000.0, v=15.0, direction=d)
  mem.add_pass(west, 125.0, t=1010.0, v=15.0, direction=MC.curve_direction(west, True))
  assert len(mem.entries) == 1 and len(mem.entries[0]["passes"]) == 1 and mem.entries[0]["passes"][0][0] == 125.0
  mem.add_saturated_pass(east, 16.0, t=2000.0, direction=d)
  mem.add_saturated_pass(west, 15.0, t=2010.0, direction=d)
  assert len(mem.entries) == 1 and mem.entries[0]["cap_hits"] == 1
  # coming back the other way is another record, 15 m from the first: not this one
  back = MC.curve_direction(west, False)
  mem.add_pass(west, 110.0, t=3000.0, v=15.0, direction=back)
  assert len(mem.entries) == 2 and len(mem._find(*west["apex"], back)["passes"]) == 1
  assert len(mem._find(*east["apex"], d)["passes"]) == 1


def test_the_planner_plans_and_records_a_curve_under_the_same_direction(tmp_path):
  ways = G.curve_road(r=115, turn=40, split=True)
  root, curves, _ = write_files(ways)
  c = curves[0]
  fwd, back = MC.curve_direction(c, True), MC.curve_direction(c, False)
  assert fwd is not None and not MC.same_direction(fwd, back)
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  for k, r in enumerate((160.0, 165.0)):
    mem.add_pass(c, r, t=1000.0 * (k + 1), direction=fwd)              # learned northbound (way 1 forward)
  m = MC.MapCurves(MC.MapCurveStore(root, background=False), memory=mem)
  la, lo = pos(0, -250)
  m.update(la, lo, 0.0, 1, True)
  assert m.curve is not None and m.radius in (160.0, 165.0) and m.source == 1 and m.direction(m.curve) == fwd
  # southbound (way 2 against its node order, as test_other_direction...): the map's radius, nothing learned that way
  ex = -115 + 115 * math.cos(math.radians(40)) - math.sin(math.radians(40)) * 200
  ey = 115 * math.sin(math.radians(40)) + math.cos(math.radians(40)) * 200
  m.update(*pos(ex, ey), 140.0, 2, False)
  assert m.curve is not None and abs(m.radius - c["r"]) < 1e-6 and m.source == 0 and m.direction(m.curve) == back
  # in the curve northbound: the pass is recorded under the direction it is planned with
  ang = math.radians(20)
  m.update(*pos(-115 + 115 * math.cos(ang), 115 * math.sin(ang)), 340.0, 2, True)
  assert m.inside is not None and m.inside_dir == fwd == m.direction(m.inside)
  # the curve files rebuilt with the road's nodes the other way round (and 3 m east): the same record northbound
  pts = [((lo_ - G.LON0) / G.M_LON + 3.0, (la_ - G.LAT0) / G.M_LAT) for w in (ways[1], ways[2]) for la_, lo_ in zip(w.lat, w.lon, strict=True)]
  pts = pts[:len(ways[1].lat)] + pts[len(ways[1].lat) + 1:]             # way 2 starts on way 1's last node
  root2, curves2, _ = write_files({1: G.way(1, pts[::-1])})
  assert len(curves2) == 1 and abs(MC.curve_direction(curves2[0], False) - fwd) < 15.0, (curves2[0], fwd)
  m2 = MC.MapCurves(MC.MapCurveStore(root2, background=False), memory=mem)
  m2.update(la, lo, 0.0, 1, False)                                       # northbound is now against the node order
  assert m2.curve is not None and m2.radius in (160.0, 165.0) and m2.source == 1
  # a pass recorded through the planner's own path carries the planned direction
  mp = MC.MapCurvesPlanner(None, None)
  mp.mc, mp.pass_curve, mp.pass_samples, mp.pass_dir = m, c, _pass(150.0), back
  mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed, mp.pass_shaky = False, 5.0, 16.0, False, False
  mp._finish_pass()
  assert len(mem._find(*c["apex"], back)["passes"]) == 1 and len(mem._find(*c["apex"], fwd)["passes"]) == 2


def test_a_pass_driven_one_way_is_not_planned_the_other_way(tmp_path):
  # the whole path, as on the road: MapCurves finds the curve we are in, the planner measures the pass and records
  # it, and the next plan uses it northbound only (the old shared record made the curve gentler both ways)
  from types import SimpleNamespace as N
  r_map, turn = 115.0, 40.0
  root, curves, _ = write_files(G.curve_road(r=r_map, turn=turn))
  mem = MC.CurveMemory(str(tmp_path / "mem.json"))
  m = MC.MapCurves(MC.MapCurveStore(root, background=False), memory=mem)
  mp = MC.MapCurvesPlanner(None, None)
  mp.mc, mp.trust, mp.pass_curve, mp.pass_samples = m, N(shaky=False), None, []
  arc = math.radians(turn) * r_map
  v, dt, s, frame = 18.0, MC.PASS_DT, 0.0, 0
  while s < 300 + arc + 200:                                              # north up the straight, through the curve, on
    if s < 300:
      x, y, bearing, k = 0.0, -300 + s, 0.0, 0.0
    elif s < 300 + arc:
      a = (s - 300) / r_map
      x, y, bearing, k = -r_map + r_map * math.cos(a), r_map * math.sin(a), (360.0 - math.degrees(a)) % 360.0, 1 / 200.0
    else:
      ex, ey = -r_map + r_map * math.cos(math.radians(turn)), r_map * math.sin(math.radians(turn))
      d = s - 300 - arc
      x, y, bearing, k = ex - math.sin(math.radians(turn)) * d, ey + math.cos(math.radians(turn)) * d, 360.0 - turn, 0.0
    la, lo = pos(x, y)
    m.update(la, lo, bearing, 1, True)
    sm = {"carState": N(vEgo=v, steeringPressed=False, leftBlinker=False, rightBlinker=False),
          "carControl": N(latActive=True, actuators=N(torque=0.4)), "controlsState": N(curvature=k)}
    mp.counter = frame
    mp._measure_pass(sm, m.inside, la, lo, dt=dt)                        # the car drove the curve at 200 m (gentler)
    s, frame = s + v * dt, frame + 1
  assert len(mem.entries) == 1 and len(mem.entries[0]["passes"]) == 1, (mem.entries, getattr(mp, "pass_drops", []))
  assert abs(mem.entries[0]["passes"][0][0] - 200.0) < 10.0
  m.update(*pos(0, -250), 0.0, 1, True)                                  # northbound again: the gentler radius
  assert m.curve is not None and abs(m.radius - 200.0) < 10.0 and m.source == 1, (m.radius, m.source)
  ex = -r_map + r_map * math.cos(math.radians(turn)) - math.sin(math.radians(turn)) * 200
  ey = r_map * math.sin(math.radians(turn)) + math.cos(math.radians(turn)) * 200
  m.update(*pos(ex, ey), 180.0 - turn, 1, False)                         # southbound: never driven that way: the map's
  assert m.curve is not None and abs(m.radius - curves[0]["r"]) < 1e-6 and m.source == 0, (m.radius, m.source)


def test_a_pass_with_the_turn_signal_or_off_the_apex_counts_for_nothing(tmp_path):
  class P:
    def get(self, key, encoding=None):
      return None

  def run(blinker=False, apex=5.0, exit_=16.0):
    mp = MC.MapCurvesPlanner(P(), P())
    mp.mc = MC.MapCurves(MC.MapCurveStore(tempfile.mkdtemp(), background=False),
                         memory=MC.CurveMemory(str(tmp_path / f"m{blinker}{apex}{exit_}.json")))
    c = {"apex": (42.6, -83.1), "r": 129.0, "s_a": 0.0}
    mp.pass_curve, mp.pass_samples = c, _pass(400.0)
    mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed = blinker, apex, exit_, True
    mp._finish_pass()
    e = mp.mc.memory._find(*c["apex"])
    return e["passes"] if e else []

  ps = run()
  assert len(ps) == 1 and ps[0][3] == ["full", "vetoed"]
  assert run(blinker=True) == []                                       # a lane change in the curve
  assert run(apex=30.0) == []                                          # never near the apex: another road
  assert run(exit_=200.0)[0][3] == ["vetoed"]                          # lost the road before the exit: not full


# ---------------- GPS trust: a GPS that is off the road, or a road match that may be the wrong road ----------------

QS = _load("qcom_structs", os.path.join(ROOT, "system", "qcomgpsd", "structs.py"))


def _gps(v=20.0, ambiguous=False, acc=0.0, rel=0, dt=0.05):
  return {"dt": dt, "v_ego": v, "acc": acc, "rel": rel, "ambiguous": ambiguous}


def _drive(m, rm, trust, x, y, bearing, frames, v=20.0, acc=0.0, rel=0):
  la, lo = pos(x, y)
  for _ in range(frames):
    w, f = rm.update(la, lo, bearing, acc)
    m.update(la, lo, bearing, w, f, trust=trust, gps=_gps(v, rm.ambiguous, acc, rel))
  return w


def _with_trust(ways):
  root, curves, ramps = write_files(ways)
  store = MC.MapCurveStore(root, background=False)
  return MC.MapCurves(store), MC.RoadMatcher(store), MC.GpsTrust(), curves, ramps


def test_the_gps_chips_own_accuracy_is_kept():
  report = {"q_FltEllipseSemimajorAxis": 7.5, "u_NumGpsSvsUsed": 9, "u_NumGloSvsUsed": 6, "u_NumBdsSvsUsed": 4,
            "u_HorizontalReliability": 3, "u_EllipseConfidence": 39, "q_SensorAidMask": 0x4, "q_SensorDataUsageMask": 3}
  acc, sats, flags = QS.position_quality(report)
  assert acc == 7.5 and sats == 19
  assert MC.gps_flags(flags) == {"reliability": 3, "sensorAided": True, "sensorData": True, "confidence": 39}
  for bad in (float("nan"), -1.0, 0.0, 9999.0):                          # unknown, never a huge number
    assert QS.position_quality({**report, "q_FltEllipseSemimajorAxis": bad})[0] == 0.0
  from types import SimpleNamespace as N
  q = MC.gps_quality(N(horizontalAccuracy=acc, satelliteCount=sats, flags=flags))
  assert q == {"accuracy": 7.5, "satellites": 19, "reliability": 3, "confidence": 39, "sensorAided": True}
  assert MC.gps_quality(N())["accuracy"] == 0.0                          # another GPS source without these fields


def test_a_gps_far_off_the_road_pauses_the_map_after_ten_seconds():
  m, rm, trust, curves, _ = _with_trust(G.curve_road(r=115, turn=40, split=True))
  frames = int(MC.LOST_TIME / 0.05)
  w = _drive(m, rm, trust, 100, -250, 0.0, frames - 1)                   # drive 3f: the track 100 m beside the road
  assert w == 0 and not trust.paused and 90 <= rm.nearest_d <= 110, rm.nearest_d
  _drive(m, rm, trust, 100, -250, 0.0, 1)
  assert trust.paused and trust.reason == "lost" and m.paused and m.trust_status == MC.TRUST_PAUSED_LOST
  assert [e[0] for e in trust.events] == ["pause"]
  # slow (a residential street, not in the files) or no curve data here: never paused
  m2, rm2, t2, _, _ = _with_trust(G.curve_road(r=115, turn=40, split=True))
  _drive(m2, rm2, t2, 100, -250, 0.0, 3 * frames, v=10.0)
  assert not t2.paused
  t3 = MC.GpsTrust()
  for _ in range(3 * frames):
    t3.update(0.05, 20.0, 0.0, 0, False, False, False)
  assert not t3.paused


def test_after_a_pause_the_road_must_match_for_five_seconds():
  m, rm, trust, curves, _ = _with_trust(G.curve_road(r=115, turn=40, split=True))
  _drive(m, rm, trust, 100, -250, 0.0, int(MC.LOST_TIME / 0.05))
  assert trust.paused
  resume = int(MC.RESUME_TIME / 0.05)
  _drive(m, rm, trust, 0, -250, 0.0, resume // 2)                        # back on the road ...
  _drive(m, rm, trust, 100, -250, 0.0, 1)                                # ... one frame off it again: starts over
  _drive(m, rm, trust, 0, -250, 0.0, resume - 1)
  assert trust.paused and m.curve is None
  _drive(m, rm, trust, 0, -250, 0.0, 1)
  assert not trust.paused and m.curve is not None and m.status == MC.STATUS_CURVE
  assert trust.shaky                                                     # trusted again, but not "steady" for a minute
  _drive(m, rm, trust, 0, -250, 0.0, int(MC.STEADY_TIME / 0.05))
  assert not trust.shaky and m.trust_status == MC.TRUST_OK


def _parallel(gap, curve_on_b=True):
  """Road A straight north; road B (another name) beside it, `gap` m east, with a 115 m left curve 300 m ahead."""
  a = G.way(1, G.straight(0, -400, 0, 1400), name="Main Road")
  if curve_on_b:
    pts = G.straight(gap, -400, 0, 700)
    c = G.arc(gap - 115, 300, 115, 0, 40)
    pts += c[1:]
    pts += G.straight(*c[-1], -40, 300)[1:]
  else:
    pts = G.straight(gap, -400, 0, 1400)
  b = G.way(2, pts, name="Service Drive")
  return {1: a, 2: b}


def test_a_parallel_road_close_by_makes_the_match_doubtful_so_its_curve_is_held():
  # the owner's case: the GPS lands on a parallel road that has a curve; the road the car is on has none
  m, rm, trust, curves, _ = _with_trust(_parallel(20))
  assert any(c["name"] == "Service Drive" for c in curves)
  _drive(m, rm, trust, 12, -50, 0.0, 5)                                  # 8 m from B, 12 m from A
  assert rm.ambiguous and m.curve is None and m.status == MC.STATUS_HELD and m.held is not None
  assert m.held[0]["name"] == "Service Drive" and m.trust_status == MC.TRUST_LOW
  # the same with a larger GPS error estimate: a road 30 m away is still too close to call
  m, rm, trust, _, _ = _with_trust(_parallel(30))
  _drive(m, rm, trust, 25, -50, 0.0, 5, acc=12.0)
  assert rm.ambiguous and m.curve is None
  # documented limit: with the other road farther than the ambiguity distance, the match is taken
  m, rm, trust, _, _ = _with_trust(_parallel(50))
  _drive(m, rm, trust, 47, -50, 0.0, 5)
  assert not rm.ambiguous and m.curve is not None


def test_a_road_that_only_starts_here_or_runs_the_other_way_does_not_make_the_match_doubtful():
  m, rm, trust, _, _ = _with_trust(G.curve_road(r=115, turn=40, split=True))
  _drive(m, rm, trust, 2, -250, 0.0, 3)
  assert not rm.ambiguous and m.curve is not None                        # one road, two ways of the same chain
  # the other carriageway of a divided road: one-way the other direction, 20 m away
  a = G.way(1, G.straight(0, -400, 0, 1400), name="Big Road", oneway=True)
  pts = G.straight(20, 1000, 180, 700)
  c = G.arc(20 + 115, 300, 115, 180, 220)
  b = G.way(2, pts + c[1:], name="Big Road", oneway=True)
  m, rm, trust, _, _ = _with_trust({1: a, 2: b})
  _drive(m, rm, trust, 3, -50, 0.0, 3)
  assert rm.way_id == 1 and not rm.ambiguous


def test_at_an_exit_split_a_ramp_curve_waits_until_the_ramp_is_clearly_apart():
  m, rm, trust, _, ramps = _with_trust(G.ramp_scene())
  h = math.radians(20)
  _drive(m, rm, trust, 0, -200, 0.0, 3)
  for along, held in ((40, True), (60, False)):                          # ramp 13.7 m / 20.5 m from the motorway
    _drive(m, rm, trust, along * math.sin(h), along * math.cos(h), 20.0, 3)
    assert rm.way_id == 11, (along, rm.way_id)
    assert (m.status == MC.STATUS_HELD) == held, (along, m.status)
  assert m.status == MC.STATUS_RAMP_CURVE and m.on_ramp
  # with a GPS error of 10 m the ramp must be 25 m apart
  m, rm, trust, _, _ = _with_trust(G.ramp_scene())
  _drive(m, rm, trust, 0, -200, 0.0, 3, acc=10.0)
  _drive(m, rm, trust, 60 * math.sin(h), 60 * math.cos(h), 20.0, 3, acc=10.0)
  assert m.status == MC.STATUS_HELD and not m.on_ramp


def test_the_gps_error_estimate_pauses_or_holds():
  t = MC.GpsTrust()
  confirm = int(MC.BAD_CONFIRM / 0.05)
  for _ in range(confirm - 1):
    t.update(0.05, 20.0, 80.0, 3, True, True, False)
  assert not t.paused
  t.update(0.05, 20.0, 80.0, 3, True, True, False)
  assert t.paused and t.reason == "accuracy" and t.status == MC.TRUST_PAUSED_ACCURACY
  for _ in range(int(MC.RESUME_TIME / 0.05)):
    t.update(0.05, 20.0, 10.0, 3, True, True, False)
  assert not t.paused
  t.update(0.05, 20.0, 30.0, 3, True, True, False)
  assert t.low and not t.paused                                          # 20-50 m: held, not paused
  t2 = MC.GpsTrust()
  for _ in range(confirm):
    t2.update(0.05, 20.0, 0.0, 1, True, True, False)                     # reliability "very low"
  assert t2.paused
  t3 = MC.GpsTrust()
  for _ in range(confirm - 1):
    t3.update(0.05, 20.0, 80.0, 3, True, True, False)
  t3.update(0.05, 20.0, 5.0, 3, True, True, False)                       # a blip: starts over
  for _ in range(confirm - 1):
    t3.update(0.05, 20.0, 80.0, 3, True, True, False)
  assert not t3.paused


def test_the_car_side_treats_paused_and_held_as_no_curve():
  c = CTL.MapCurveControl(3.0)
  assert c.update(F(status=1, radius=115, distance=40), 50 * MPH, False).active
  for status in (MC.STATUS_PAUSED, MC.STATUS_HELD):
    out = c.update(F(status=status, radius=115, distance=40), 50 * MPH, False)
    assert not out.active, status
  assert c.update(F(status=1, radius=115, distance=40), 50 * MPH, False).active
  from types import SimpleNamespace as N
  plan = N(mapCurveMode=2, mapCurveStatus=4, mapCurveDistance=0.0, mapCurveRadius=0.0, mapOnRamp=False, mapRampType=0,
           mapRampToSpeed=0.0, mapRampToHighway=False, mapRampEndDistance=0.0, mapNavExit=0, mapNavExitDistance=0.0,
           mapNavCurveRadius=0.0, mapNavCurveDistance=0.0, mapGpsTrust=4)
  assert CTL.MapFields.from_plan(plan).gps_trust == 4


def test_a_pass_that_did_not_turn_or_was_not_trusted_is_not_learned(tmp_path):
  class P:
    def get(self, key, encoding=None):
      return None

  def run(samples, shaky=False):
    mp = MC.MapCurvesPlanner(P(), P())
    mp.mc = MC.MapCurves(MC.MapCurveStore(tempfile.mkdtemp(), background=False),
                         memory=MC.CurveMemory(str(tmp_path / f"t{len(samples)}{shaky}{id(samples)}.json")))
    c = {"apex": (42.6, -83.1), "r": 115.0, "s_a": 0.0, "turn": 40.0, "name": "Wrong Road"}
    mp.pass_curve, mp.pass_samples = c, samples
    mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed, mp.pass_shaky = False, 5.0, 16.0, False, shaky
    mp._finish_pass()
    e = mp.mc.memory._find(*c["apex"])
    return (e["passes"] if e else []), getattr(mp, "pass_drops", [])

  turn_time = math.radians(40) * 115 / 18.0                              # s to turn 40 deg on a 115 m arc at 18 m/s
  real = _pass(115, seconds=turn_time)
  passes, drops = run(real)
  assert len(passes) == 1 and not drops
  passes, drops = run(_pass(0, seconds=6.0))                             # straight on (a parallel road): 2000 m
  assert passes == [] and drops[0]["reason"] == "turn" and drops[0]["driven_turn"] < 1
  passes, drops = run(real, shaky=True)                                  # the match was not trusted in the pass
  assert passes == [] and drops[0]["reason"] == "shaky"
  assert abs(MC.driven_turn(real) - 40) < 2


def test_while_the_match_is_shaky_the_camera_may_end_a_slowdown_for_any_curve():
  m, curves, where = _false_kink()
  m.update(*where)
  m.curve["adv"] = 35 * MPH                                              # a signed curve: normally never ignored
  t = MC.GpsTrust()                                                      # drive start: shaky
  for _ in range(MC.VETO_CONFIRM):
    m.update(*where, camera=_view(40.0, 400.0), trust=t, gps=_gps())
  assert m.curve is None and m.veto_status == 1


def test_publish_says_paused_held_or_no_data():
  from types import SimpleNamespace as N
  mp = MC.MapCurvesPlanner(None, None)
  mp.mode = 2
  m, rm, trust, _, _ = _with_trust(_parallel(20))
  mp.mc = m
  _drive(m, rm, trust, 12, -50, 0.0, 3)
  plan = N()
  mp.publish(plan)
  assert plan.mapCurveStatus == MC.STATUS_HELD and plan.mapCurveRadius > 0 and plan.mapGpsTrust == MC.TRUST_LOW
  m, rm, trust, _, _ = _with_trust(G.curve_road(r=115, turn=40, split=True))
  mp.mc = m
  _drive(m, rm, trust, 100, -250, 0.0, int(MC.LOST_TIME / 0.05))
  plan = N()
  mp.publish(plan)
  assert plan.mapCurveStatus == MC.STATUS_PAUSED and plan.mapCurveRadius == 0 and not plan.mapOnRamp and plan.mapNavExit == 0


def test_the_planner_says_map_curves_paused_once_per_drive(monkeypatch):
  import time as _time
  root, curves, _ = write_files(G.curve_road(r=115, turn=40, split=True))

  class P:
    def __init__(self, vals):
      self.vals = vals

    def get(self, key, encoding=None):
      return self.vals.get(key)

    def get_bool(self, key):
      return False

  class SM(dict):
    alive = {"navInstruction": False}
    valid = {"navInstruction": False}

  clock = [1000.0]
  monkeypatch.setattr(_time, "monotonic", lambda: clock[0])
  mp = MC.MapCurvesPlanner(P({"MazdaMapCurves": "2"}), P({}))
  mp.mc = MC.MapCurves(MC.MapCurveStore(root, background=False))
  mp.matcher = MC.RoadMatcher(mp.mc.store)
  mp.pass_curve, mp.pass_samples = None, []

  def drive(x, y, seconds):
    notices = []
    la, lo = pos(x, y)
    for _ in range(int(seconds / 0.05)):
      clock[0] += 0.05
      mp.update(SM(), {"latitude": la, "longitude": lo, "bearing": 0.0}, 20.0, 20.0)
      notices.append(mp.notice)
    return [n for n in notices if n]

  assert drive(100, -400, MC.LOST_TIME + 1) == ["paused"]
  drive(0, -250, MC.RESUME_TIME + 1)                                     # back on the road
  assert not mp.trust.paused
  assert drive(100, -400, MC.LOST_TIME + 1) == []                        # once per drive
  clock[0] += 60.0                                                       # next drive
  assert drive(100, -400, MC.LOST_TIME + 1) == ["paused"]


# ---------------- the pass window (drive 44, Dobry Drive) ----------------

class _FakeMC:
  """What _measure_pass reads from MapCurves: the curve we are in and the metres to its exit, as MapCurves finds
  them (from NEAR past the start to NEAR before the end) on a GPS position that moves once a second and lags."""

  def __init__(self, memory):
    from types import SimpleNamespace as N
    self.memory, self.chain_id, self.inside, self.inside_exit = memory, 1, None, float("inf")
    self.next_entry, self.next_curve = float("inf"), None
    self.veto = N(is_vetoed=lambda key: False)


def _curve(k, start, length, turn, r_map):
  return {"apex": (42.6 + 0.01 * k, -83.1), "r": r_map, "s_a": float(start), "len": float(length), "turn": float(turn),
          "name": f"c{k}", "x0": float(start), "x1": float(start + length)}


def _drive_road(road, curves, tmp_path, v=18.0, lag_m=None, gap_at=None, speed=None, rebuild_at=None, bad_fix_at=None,
                stall_at=None):
  """road: [(metres, signed curvature)]; curves: map curves placed at x0..x1 (m along the road). speed(x): m/s (else
  v); rebuild_at: the curve files are rebuilt there (new dicts, same curves); bad_fix_at: one GPS fix 60 m off there;
  stall_at: one 0.5 s pause of the planner there. Returns the planner (its memory has the passes)."""
  from types import SimpleNamespace as N

  class P:
    def get(self, key, encoding=None):
      return None

  mp = MC.MapCurvesPlanner(P(), P())
  mp.mc = _FakeMC(MC.CurveMemory(str(tmp_path / f"pw{len(curves)}{lag_m}{gap_at}{rebuild_at}{bad_fix_at}{stall_at}.json")))
  mp.trust = N(shaky=False)
  mp.pass_curve, mp.pass_samples = None, []
  lag_m = v * MC.PASS_GPS_LAG if lag_m is None else lag_m
  x, frame, gps_x = 0.0, 0, -1e9
  segs, s0 = [], 0.0
  for length, k in road:
    segs.append((s0, s0 + length, k))
    s0 += length
  done = set()
  while x < s0:
    k = next(kk for a, b, kk in segs if a <= x < b)
    vx = speed(x) if speed is not None else v
    if rebuild_at is not None and x >= rebuild_at and "rebuild" not in done:
      done.add("rebuild")
      curves = [dict(c) for c in curves]
    if frame % 20 == 0:
      gps_x = x - lag_m                                   # a new fix, already lagging
      if bad_fix_at is not None and x >= bad_fix_at and "bad" not in done:
        done.add("bad")
        gps_x += 60.0
    inside = None
    for c in curves:
      if c["x0"] + MC.NEAR < gps_x < c["x1"] - MC.NEAR:
        inside = c
        break
    mp.mc.inside = inside
    mp.mc.inside_exit = (inside["x1"] - gps_x) if inside is not None else float("inf")
    ahead = [c for c in curves if c is not inside and (c["x0"] - gps_x > 0 or abs(c["x0"] - gps_x) < MC.NEAR)]
    nxt = min(ahead, key=lambda c: c["x0"], default=None)
    mp.mc.next_curve, mp.mc.next_entry = nxt, (abs(nxt["x0"] - gps_x) if nxt is not None else float("inf"))
    near = min(curves, key=lambda c: abs((c["x0"] + c["x1"]) / 2 - gps_x))
    lat = near["apex"][0] + (gps_x - (near["x0"] + near["x1"]) / 2) / 111320.0
    mp.counter = frame + (5 if gap_at is not None and x >= gap_at else 0)
    sm = {"carState": N(vEgo=vx, steeringPressed=False, leftBlinker=False, rightBlinker=False),
          "carControl": N(latActive=True, actuators=N(torque=0.4)), "controlsState": N(curvature=k)}
    dt = MC.PASS_DT
    if stall_at is not None and x >= stall_at and "stall" not in done:
      done.add("stall")
      dt = 0.5
      x += vx * (dt - MC.PASS_DT)
    mp._measure_pass(sm, inside, lat, near["apex"][1], dt=dt)
    x += vx * MC.PASS_DT
    frame += 1
  return mp


def _learned(mp, c):
  e = mp.mc.memory._find(*c["apex"])
  return [p[0] for p in e["passes"]] if e else []


def test_drive_44_a_short_curve_is_measured_from_before_its_start_to_past_its_end(tmp_path):
  # Dobry Drive: the map says 131 m, 64 m long, 20 deg; the road is ~165 m and turns 21 deg. Counting only the
  # stretch the car was "inside" (15 m in from each end, on a once-a-second GPS 13-18 m behind) measured 6-8 deg
  # and dropped every pass as "did not turn".
  arc = math.radians(21) * 165
  for lag in (8.0, 12.6, 18.0):
    c = _curve(0, 150, 64, 20, 131.0)
    mp = _drive_road([(150 - (arc - 64) / 2, 0.0), (arc, 1 / 165), (300, 0.0)], [c], tmp_path, lag_m=lag)
    got = _learned(mp, c)
    assert len(got) == 1 and abs(got[0] - 165) < 8, (lag, got, getattr(mp, "pass_drops", []))
    assert not getattr(mp, "pass_drops", [])


def test_a_parallel_road_still_does_not_count(tmp_path):
  c = _curve(0, 150, 64, 20, 131.0)
  mp = _drive_road([(500, 0.0)], [c], tmp_path)                       # the map's curve, but the car drove straight
  assert _learned(mp, c) == [] and mp.pass_drops[0]["reason"] == "turn"


def test_back_to_back_curves_do_not_share_their_stretch(tmp_path):
  # a gentle left (400 m, 20 deg) straight into a tight right (100 m, 35 deg): the gentle one must not take the tight
  # one's start (its 1 s tightest stretch would read ~100 m), nor the tight one the gentle one's end
  la, lb = math.radians(20) * 400, math.radians(35) * 100
  a, b = _curve(0, 150, la, 20, 380.0), _curve(1, 150 + la, lb, 35, 100.0)
  mp = _drive_road([(150, 0.0), (la, 1 / 400), (lb, -1 / 100), (300, 0.0)], [a, b], tmp_path)
  ra, rb = _learned(mp, a), _learned(mp, b)
  assert len(ra) == 1 and ra[0] > 300, ra
  assert len(rb) == 1 and abs(rb[0] - 100) < 5, rb


def test_a_gap_in_the_calls_starts_the_pass_over(tmp_path):
  arc = math.radians(21) * 165
  c = _curve(0, 150, 64, 20, 131.0)
  # the calls stop for a moment in the middle of the curve (no GPS): the distance travelled is unknown, nothing learned
  mp = _drive_road([(150 - (arc - 64) / 2, 0.0), (arc, 1 / 165), (300, 0.0)], [c], tmp_path, gap_at=180.0)
  assert _learned(mp, c) == []


def test_a_one_frame_steering_blip_does_not_spoil_a_clean_pass():
  blip = int(MC.PASS_SAT_BLIP / 0.05)
  assert abs(MC.curve_pass_radius(_pass(150)[:60] + _pass(150, cmd=0.99)[:blip] + _pass(150)[:60]) - 150) < 0.5
  assert MC.curve_pass_radius(_pass(150)[:60] + _pass(150, cmd=0.99)[:blip + 1] + _pass(150)[:60]) is None


def _dobry(tmp_path, **kw):
  arc = math.radians(21) * 165
  c = _curve(0, 150, 64, 20, 131.0)
  return c, _drive_road([(150 - (arc - 64) / 2, 0.0), (arc, 1 / 165), (300, 0.0)], [c], tmp_path, **kw)


def test_a_slow_pass_through_a_sharp_curve_is_not_read_from_the_road_around_it(tmp_path):
  # review: a 40 m, 90 deg bend at 7 m/s with a 12 m/s approach; only the straight road before and after it was fast
  # enough to measure, which read as "never turned" (2000 m): the curve would have been planned at 47 mph, not 21
  arc = math.radians(90) * 40
  c = _curve(0, 150, arc, 90, 40.0)
  mp = _drive_road([(150, 0.0), (arc, 1 / 40), (300, 0.0)], [c], tmp_path,
                   speed=lambda x: 7.0 if 140 <= x <= 160 + arc else 12.0)
  assert _learned(mp, c) == []


def test_curve_files_rebuilt_in_the_middle_of_a_pass_do_not_cut_it(tmp_path):
  # a curve that starts gentle (100 m at 400 m) and tightens (40 m at 120 m); the files are rebuilt in the gentle part
  # (new dicts for the same curves): cut there, the pass would read the gentle part only (~400 m, too gentle)
  c = _curve(0, 150, 140, 33, 150.0)
  mp = _drive_road([(150, 0.0), (100, 1 / 400), (40, 1 / 120), (300, 0.0)], [c], tmp_path, rebuild_at=215.0)
  got = _learned(mp, c)
  assert len(got) == 1 and abs(got[0] - 120) < 6, (got, getattr(mp, "pass_drops", []))


def test_one_gps_fix_far_off_does_not_move_the_pass(tmp_path):
  c, mp = _dobry(tmp_path, bad_fix_at=160.0)
  got = _learned(mp, c)
  assert len(got) == 1 and abs(got[0] - 165) < 8, (got, getattr(mp, "pass_drops", []))


def test_a_stalled_planner_starts_the_pass_over(tmp_path):
  c, mp = _dobry(tmp_path, stall_at=180.0)                            # half a second without a planner cycle, mid-curve
  assert _learned(mp, c) == []
  c, mp = _dobry(tmp_path, stall_at=20.0)                             # well before the curve: the pass is still whole
  assert len(_learned(mp, c)) == 1


def _near(a, b):
  return abs(a - b) < 1e-9


def _at_curve_speed(r, v_curve):
  """m before the entry where the car was first within 1 mph of the curve's speed."""
  return next(x for v, x in zip(r["vs"], r["xs"], strict=True) if v / MPH <= v_curve + 1.0)


def test_drive_54_road_curves_slow_later_and_less_ramps_and_exits_unchanged():
  # drive 54, West Utica Road (35 by the memory cap / Max Drop) at 45.4 mph: the set speed went 3 under the curve, the
  # car slowed up to 0.82 m/s^2 and was at the curve's speed 43-60 m before the entry ("slow down a little later, it
  # brakes a little too much"). Road curves now: 3 mph undershoot, at the curve's speed 0.5 s before the entry, and no
  # further SET- while the car already slows as hard as the room left needs
  ctl = CTL.MapCurveControl(3.0)
  assert _near(ctl.road_follower.undershoot, 3 * MPH) and _near(ctl.road_follower.lead_time, 0.5) and ctl.road_follower.closed_loop
  for fol in (ctl.follower, ctl.nav_follower):                  # ramp curves and exits keep theirs
    assert _near(fol.undershoot, CT.MAP_UNDERSHOOT) and _near(fol.lead_time, CT.MAP_LEAD_TIME) and not fol.closed_loop
  assert _near(CT.MAP_UNDERSHOOT, 5 * MPH) and _near(CT.MAP_LEAD_TIME, 1.0)
  # the car side's road plan and the planner's agree (the notice and the slowdown start together)
  assert _near(MC.PLAN_ROAD_LEAD_TIME, CT.MAP_ROAD_LEAD_TIME) and _near(MC.PLAN_LEAD_TIME, CT.MAP_LEAD_TIME)
  _, sp = ctl.curve.update(91.0, 300.0, 20.0, 0.0, ctl.decel, floor=15.6, lead_time=CT.MAP_ROAD_LEAD_TIME)
  assert abs(MC.plan_speed(91.0, 300.0, 0.0, 20.0, decel=ctl.decel, floor=15.6, lead_time=MC.PLAN_ROAD_LEAD_TIME) - sp) < 1e-9
  # the measured cruise: at the curve's speed closer to the entry than with the old 1 s lead (46 m there), still at it
  r = drive_map(max_drop=10)
  assert 15 <= _at_curve_speed(r, 35.0) <= 40 and abs(r["v_entry"] - 35.0) <= 1.0 and r["min_set"] >= 32 - 1e-6
  # a cruise that slows harder than the model (this stretch of West Utica: ~0.55 m/s^2 at a 1-2 m/s gap): the closed
  # loop keeps the set speed from going further down once the car slows hard enough, so the car gets to the curve's
  # speed nearer the entry than open loop
  for gain in (0.28, 0.4):
    closed = drive_map(max_drop=10, car_kw=dict(cruise_gain=gain))
    open_ = drive_map(max_drop=10, car_kw=dict(cruise_gain=gain), closed=False)
    assert _at_curve_speed(closed, 35.0) <= _at_curve_speed(open_, 35.0) - 10, (gain, _at_curve_speed(closed, 35.0),
                                                                                 _at_curve_speed(open_, 35.0))
    assert abs(closed["v_entry"] - 35.0) <= 1.0 and closed["min_in_curve"] >= 34.0, (gain, closed["v_entry"])
  # a slower cruise still gets there
  slow = drive_map(max_drop=10, car_kw=dict(cruise_gain=0.15))
  assert abs(slow["v_entry"] - 35.0) <= 1.0 and slow["min_in_curve"] >= 34.0, slow["v_entry"]


def test_the_curve_notice_shows_the_speed_the_car_takes_the_curve_to():
  # drive 54: "Map curve in 550 ft · Slowing to 32 mph" for West Utica, which the car took to 35 (the Max Drop floor
  # of his 45): the planner publishes the curve's speed with the floor (mapCurveTarget) and the notice shows that
  from types import SimpleNamespace as N
  mc = N(data_here=True, paused=False, curve={"name": "West Utica Road"}, held=None, status=MC.STATUS_CURVE, distance=190.0,
         radius=93.08, advisory=0.0, floor=35 * MPH, on_ramp=False, ramp=None, nav_ramp=None, veto_status=0, veto_radius=0.0,
         source=2, trust_status=0, nav_exit=0, nav_exit_dist=0.0, nav_curve=None, nav_curve_dist=0.0, nav_side=0,
         exit_shape=0, exit_link=(0.0, 0.0), exit_hwy=(0.0, 0.0), exit_fork=0, exit_curve=None, exit_curve_dist=0.0,
         exit_start=0.0, exit_bends=(None, None), ramp_end_dist=0.0)
  mp = MC.MapCurvesPlanner(None, None)
  mp.mode, mp.mc, mp.road_class, mp.gps_acc = 2, mc, 0, 0.0
  plan = N()
  mp.publish(plan)
  assert abs(plan.mapCurveTarget / MPH - 35.0) < 1e-6 and abs(math.sqrt(2.2 * plan.mapCurveRadius) / MPH - 32.0) < 0.05
  mc.floor = 25 * MPH                                            # a lower floor (Max Drop 20 of his 45): the curve's own
  mp.publish(plan)
  assert abs(plan.mapCurveTarget - math.sqrt(2.2 * 93.08)) < 1e-6
  mc.status, mc.floor = MC.STATUS_RAMP_CURVE, 35 * MPH            # a ramp curve: never the road's floor
  mp.publish(plan)
  assert abs(plan.mapCurveTarget - math.sqrt(2.2 * 93.08)) < 1e-6
