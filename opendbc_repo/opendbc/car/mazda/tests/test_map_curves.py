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


def test_west_utica_slows_before_the_curve_not_in_it():
  # route 36: 45 mph, the 117 m curve (~36 mph). The set speed comes down in 1 mph steps that the stock cruise can
  # follow: drive 38 showed it follows small steps at only ~0.28 m/s^2, ~1.5 s after each press.
  c = CTL.MapCurveControl(3.0)
  start = None
  for d in range(500, -1, -2):
    if c.update(F(status=1, radius=116.6, distance=d), 45 * MPH, 0).active:
      start = d
      break
  assert start is not None and 280 <= start <= 340, start
  # the set speed (whole mph, following the plan at the car's position) and the cruise following it like on drive 38
  v_curve = math.sqrt(2.2 * 116.6)
  ctl = CTL.MapCurveControl(3.0)
  v, x, t, sets, max_gap = 45 * MPH, 500.0, 0.0, [], 0.0
  while x > 0:
    out = ctl.update(F(status=1, radius=116.6, distance=x), 45 * MPH, 0)
    target = math.ceil(out.speed / MPH - 1e-9) * MPH if out.active else 45 * MPH
    sets.append((t, min(45 * MPH, target)))
    lagged = [sp for tt, sp in sets if tt <= t - 1.5]
    set_now = lagged[-1] if lagged else 45 * MPH
    if v > set_now + 0.05:
      v = max(v - 0.28 * 0.01, set_now)
    max_gap = max(max_gap, (v - sets[-1][1]) / MPH)
    x -= v * 0.01
    t += 0.01
  assert v <= v_curve + 0.6 * MPH, v / MPH                     # at the curve's speed (within ~0.5 mph) at its entry
  assert max_gap <= 2.0, max_gap                               # the cruise never falls far behind the set speed
  # gradual: the planned speed comes down by well under 1 mph per second, never in jumps
  prev, steps = None, []
  for d in range(start, -1, -1):
    sp = CTL.MapCurveControl(3.0).update(F(status=1, radius=116.6, distance=d), 45 * MPH, 0).speed
    if prev is not None:
      steps.append((prev - sp) / MPH / (1.0 / 18.0))          # mph per second at ~40 mph (18 m/s)
    prev = sp
  assert max(steps) < 0.7, max(steps)


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
  # other threads get to run while a big tile is parsed (the planner loop must never stall for the whole parse)
  text, ticks, done = json.dumps(tile), [], threading.Event()

  def other():
    while not done.is_set():
      ticks.append(_t.perf_counter())
      _t.sleep(0.001)
  th = threading.Thread(target=other)
  th.start()
  for _ in range(20):
    MC.load_json_in_steps(text)
  done.set()
  th.join()
  gaps = [b - a for a, b in zip(ticks, ticks[1:], strict=False)]
  assert max(gaps) < 0.05, max(gaps)


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
