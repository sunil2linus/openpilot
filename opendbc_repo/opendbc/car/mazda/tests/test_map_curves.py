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
