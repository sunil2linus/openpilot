"""Offline tests for map curves: the planner-side lookahead (starpilot/controls/lib/map_curves.py), the car-side
control (map_curve_control.py) and the controller's ramp rules, on curve files built by tools/curve_map."""
import importlib.util
import json
import math
import os
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
  assert 400 <= first <= 850, first                        # starts only as early as the 45 m curve needs


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
  out = c.update(F(status=2, radius=45, distance=150, on_ramp=True, ramp_type=1), 70 * MPH, 0)
  assert c.exit_intent and out.active and out.why == "nav exit"   # just matched to the ramp: the exit plan still holds
  out = on_ramp_for(c, HOLD, 60 * MPH, status=2, radius=45, distance=140, on_ramp=True, ramp_type=1)
  assert not c.exit_intent and out.active and out.why == "ramp"


def test_planner_reads_the_exit_side():
  assert MC.nav_side("slight right") == 1 and MC.nav_side("left") == -1 and MC.nav_side("") == 0


def test_exit_onto_a_street_caps_then_keeps_the_street_speed():
  c = CTL.MapCurveControl(3.0)
  ramp = dict(on_ramp=True, ramp_type=1, ramp_to_highway=False, ramp_to_speed=45 * MPH)
  for k in range(HOLD + 1):                                   # driving along the exit for a second
    assert not c.update(F(ramp_end_distance=500 - k * 0.3, **ramp), 70 * MPH, False).active
  out = on_ramp_for(c, 10, 70 * MPH, ramp_end_distance=150, **ramp)
  assert out.active and out.why == "street cap" and abs(out.speed - 45 * MPH) < 1e-9
  out = c.update(F(on_ramp=False), 70 * MPH, False)          # off the ramp onto the street
  assert out.adopt
  assert not c.update(F(on_ramp=False), 45 * MPH, False).adopt


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
  mp.update(SM(), gps, 20.0, 20.0)        # loads the files, then matches way 1
  mp.update(SM(), gps, 20.0, 20.0)
  assert mp.mc.curve is not None and abs(mp.mc.radius - curves[0]["r"]) < 1e-6
  assert mp.notice is None                 # 250 m out at 45 mph the curve does not need slowing yet
  la, lo = pos(0, -90)
  mp.update(SM(), {"latitude": la, "longitude": lo, "bearing": 0.0}, 20.0, 20.0)
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


def test_west_utica_slows_before_the_curve_not_in_it():
  # route 36: 45 mph, the 117 m curve. Start early enough that the stock cruise (1.5 s to react, then the planned
  # 0.5 m/s^2) is at the curve's speed (~36 mph) just before the curve starts (1 s), coming down ~1 mph a second.
  c = CTL.MapCurveControl(3.0)
  start = None
  for d in range(400, -1, -2):
    if c.update(F(status=1, radius=116.6, distance=d), 45 * MPH, 0).active:
      start = d
      break
  assert start is not None and 165 <= start <= 195, start
  # the cruise from 45 mph, reacting then slowing at its measured 0.6 m/s^2, starting at `start`
  v, x, t = 45 * MPH, float(start), 0.0
  v_curve = math.sqrt(2.2 * 116.6)
  while x > 0:
    if t >= 1.5:
      v = max(v - 0.6 * 0.01, v_curve)
    x -= v * 0.01
    t += 0.01
    if v <= v_curve + 1e-6:
      break
  assert x >= v_curve * 1.0 - 1.0, x                          # at the curve's speed ~1 s before the curve
  # gradual: the planned speed comes down by about 1 mph per second, never in jumps
  prev, steps = None, []
  for d in range(start, -1, -1):
    sp = CTL.MapCurveControl(3.0).update(F(status=1, radius=116.6, distance=d), 45 * MPH, 0).speed
    if prev is not None:
      steps.append((prev - sp) / MPH / (1.0 / 18.0))          # mph per second at ~40 mph (18 m/s)
    prev = sp
  assert max(steps) < 1.3, max(steps)
