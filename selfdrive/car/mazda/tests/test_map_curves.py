"""Offline tests for map curves: the planner-side lookahead (frogpilot/controls/lib/map_curves.py), the car-side
control (map_curve_control.py) and the controller's ramp rules, on curve files built by tools/curve_map."""
import importlib.util
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))   # curve_target for map_curve_control's offline import


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


MC = _load("map_curves", os.path.join(ROOT, "frogpilot", "controls", "lib", "map_curves.py"))
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
  with open(os.path.join(d, MC.tile_name(G.LAT0, G.LON0) + ".json"), "w") as f:
    json.dump({"version": 3, "curves": curves, "ramps": ramps, "chains": chains}, f)
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
  out = c.update(F(status=1, radius=115, distance=60), 50 * MPH, False)
  assert out.active and out.why == "curve" and out.floor == 0
  assert abs(out.speed - math.sqrt(3.0 * 115 + 2 * 0.6 * 30)) < 1e-6


def test_ramp_curve_may_pass_the_max_drop_down_to_25():
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(status=2, radius=45, distance=80, on_ramp=True, ramp_type=1), 70 * MPH, False)
  assert out.active and out.why == "ramp" and abs(out.floor - 25 * MPH) < 1e-9 and out.budget == CTL.RAMP_BUDGET


def nav(d, side=1, **kw):
  return F(nav_exit=1, nav_exit_distance=d, nav_curve_radius=45, nav_curve_distance=d + 100, nav_exit_side=side, **kw)


def test_navigation_exit_needs_a_turn_signal():
  c = CTL.MapCurveControl(3.0)
  for d in range(1400, 0, -10):           # navigation alone never slows
    assert not c.update(nav(d), 70 * MPH, 0).active
  out = c.update(nav(400), 70 * MPH, 1)
  assert out.active and out.why == "nav exit" and out.floor > 0


def test_early_signal_is_remembered_after_it_goes_off():
  c = CTL.MapCurveControl(3.0)
  assert not c.update(nav(1300), 70 * MPH, 1).active    # signal 0.8 mile before: too far to slow yet
  for d in range(1290, 900, -10):                          # moved into the exit lane, signal off
    assert not c.update(nav(d), 70 * MPH, 0).active
  acted = [c.update(nav(d), 70 * MPH, 0).active for d in range(900, 0, -10)]
  assert any(acted) and c.exit_intent
  first = 900 - 10 * acted.index(True)
  assert 400 <= first <= 700, first                        # starts only as early as the 45 m curve needs


def test_signal_the_wrong_way_or_passing_the_exit_forgets_it():
  c = CTL.MapCurveControl(3.0)
  c.update(nav(800), 70 * MPH, -1)                         # lane change to the left: not this right exit
  assert not c.exit_intent
  c.update(nav(800), 70 * MPH, 1)
  c.update(nav(790), 70 * MPH, -1)                         # changed their mind
  assert not c.exit_intent
  c.update(nav(400), 70 * MPH, 1)
  assert c.exit_intent
  for _ in range(int(CTL.NAV_GRACE / CTL.DT_CTRL) + 5):    # passed the exit: navigation no longer shows it
    out = c.update(F(nav_exit=3), 70 * MPH, 0)
  assert not c.exit_intent and not out.active
  assert not c.update(nav(300), 70 * MPH, 0).active        # the next exit needs its own signal


def test_unknown_side_accepts_either_signal_and_ramp_takes_over():
  c = CTL.MapCurveControl(3.0)
  c.update(nav(500, side=0), 70 * MPH, -1)
  assert c.exit_intent
  assert c.update(F(nav_exit=0), 70 * MPH, 0).active       # split: map match not on the ramp yet, plan kept
  out = c.update(F(status=2, radius=45, distance=150, on_ramp=True, ramp_type=1), 60 * MPH, 0)
  assert not c.exit_intent and out.active and out.why == "ramp"


def test_planner_reads_the_exit_side():
  assert MC.nav_side("slight right") == 1 and MC.nav_side("left") == -1 and MC.nav_side("") == 0


def test_exit_onto_a_street_caps_then_keeps_the_street_speed():
  c = CTL.MapCurveControl(3.0)
  ramp = dict(on_ramp=True, ramp_type=1, ramp_to_highway=False, ramp_to_speed=45 * MPH)
  assert not c.update(F(ramp_end_distance=500, **ramp), 70 * MPH, False).active
  out = c.update(F(ramp_end_distance=150, **ramp), 70 * MPH, False)
  assert out.active and out.why == "street cap" and abs(out.speed - 45 * MPH) < 1e-9
  out = c.update(F(on_ramp=False), 70 * MPH, False)          # off the ramp onto the street
  assert out.adopt
  assert not c.update(F(on_ramp=False), 45 * MPH, False).adopt


def test_ramp_onto_a_highway_restores_fast():
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(on_ramp=True, ramp_type=2, ramp_to_highway=True, ramp_end_distance=100), 70 * MPH, False)
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
  # West Utica Road: 117 m radius, 45 mph limit, yellow 30 mph sign. Shape alone at 3.0 m/s^2 allows ~42 mph.
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(status=1, radius=116.6, distance=0), 45 * MPH, 0)
  assert out.active and 41.5 <= out.speed / MPH <= 42.5, out.speed / MPH
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(status=1, radius=116.6, distance=0, advisory=30 * MPH), 45 * MPH, 0)
  assert out.active and abs(out.speed / MPH - 40) < 0.01           # 30 + 10 mph
  # a sign above what the shape allows changes nothing (the shape is lower)
  c = CTL.MapCurveControl(3.0)
  out = c.update(F(status=2, radius=45, distance=0, on_ramp=True, ramp_type=1, advisory=35 * MPH), 70 * MPH, 0)
  assert abs(out.speed - math.sqrt(3.0 * 45)) < 1e-6


def test_planner_uses_the_advisory_to_choose_and_announce():
  assert abs(MC.curve_speed_limit(116.6, 30 * MPH) / MPH - 40) < 0.01
  assert MC.curve_speed_limit(116.6) > MC.curve_speed_limit(116.6, 30 * MPH)
  # an easy-looking curve with a 25 mph sign governs over a sharper unsigned one at the same distance
  assert MC.plan_speed(300, 200, 25 * MPH) < MC.plan_speed(150, 200)


def test_builder_keeps_a_signed_curve_that_looks_easy():
  assert B.ADVISORY_OVER == 10 * MPH
