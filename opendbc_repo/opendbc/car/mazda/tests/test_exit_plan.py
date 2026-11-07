"""Navigation exits, the owner's design (map_curve_control.ExitPlan / LaneView, map_curves.MapCurves._watch_update):
navigation is the base, a gentle slowdown to the exit speed at the split only in an exit lane, the ramp confirmed by
the map's shapes, the full (firm) slowdown only after that, a missed exit restored at once, his decisions, the early
brake chime, forks after the split. Closed loop on the measured cruise as test_map_curves.drive_53 runs it."""
import importlib.util
import json
import math
import os
import tempfile

HERE = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location("tmc_exit", os.path.join(HERE, "test_map_curves.py"))
TM = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(TM)
T, cs, CTL, MC, G, B = TM.T, TM.cs, TM.CTL, TM.MC, TM.G, TM.B
F, MPH = TM.F, TM.MPH
KPH = 1 / 3.6


# ---------------- planner: the shape check and forks on synthetic curve files ----------------

def write_files(ways):
  """As test_map_curves.write_files, with the road shapes' chain ids as the builder writes them (a way's chain)."""
  root = tempfile.mkdtemp()
  nidx = B.node_index(ways)
  curves, ramps, chains = [], [], []
  chain_of = {}
  for ch in B.build_chains(ways):
    for w, _rev in ch:
      chain_of[w.id] = int(ch[0][0].id)
    ramp = B.ramp_record(ch, nidx, ways)
    found = B.find_curves(ch, ways, ramp)
    curves += found
    if ramp:
      ramps.append(ramp)
    if found or ramp:
      chains.append(B.chain_record(ch))
  d = os.path.join(root, "42", "-84")
  os.makedirs(d)
  roads = [r for lst in B.road_shapes(ways, chain_of).values() for r in lst]
  with open(os.path.join(d, MC.tile_name(G.LAT0, G.LON0) + ".json"), "w") as f:
    json.dump({"version": 4, "curves": curves, "ramps": ramps, "chains": chains, "roads": roads}, f)
  return root, curves, ramps


def _join(a, b):
  """b starts where a ends (the same node)."""
  b.lat[0], b.lon[0] = a.lat[-1], a.lon[-1]
  b.start = a.end = B.key(a.lat[-1], a.lon[-1])


def _path(x0, y0, heading, legs):
  """Points from (x0, y0) along legs: ("s", m) straight, ("a", r, deg) an arc (deg > 0 right, < 0 left)."""
  pts, x, y, h = [(x0, y0)], x0, y0, heading
  for leg in legs:
    if leg[0] == "s":
      n = max(1, int(leg[1] // 10))
      for _ in range(n):
        x, y = x + math.sin(math.radians(h)) * leg[1] / n, y + math.cos(math.radians(h)) * leg[1] / n
        pts.append((x, y))
    else:
      r, turn = leg[1], leg[2]
      n = max(2, int(abs(turn) / 4))
      for _ in range(n):
        step = turn / n
        x += 2 * r * math.sin(math.radians(abs(step) / 2)) * math.sin(math.radians(h + step / 2))
        y += 2 * r * math.sin(math.radians(abs(step) / 2)) * math.cos(math.radians(h + step / 2))
        h += step
        pts.append((x, y))
  return pts


def fork_scene(r_cont=60.0, r_side=150.0, fork=True):
  """A motorway north; a 2-lane exit leaves it at (0, 0) to the right (8 deg), 200 m on it forks: the continuing branch
  goes on straight and turns left (radius r_cont), the side branch turns off 30 deg right and then right (r_side).
  fork=False: the exit without the side branch."""
  mw = G.way(10, G.straight(0, -1000, 0, 2500), name="", cls=1, speed_mph=70, oneway=True)
  mw.ref = "I 75"
  k = 50                                                  # the motorway node at y = 0
  trunk = G.way(21, _path(0, 0, 8, [("s", 200)]), name="", cls=2, speed_mph=45, oneway=True)
  trunk.lat[0], trunk.lon[0] = mw.lat[k], mw.lon[k]
  trunk.start = B.key(mw.lat[k], mw.lon[k])
  cont = G.way(22, _path(0, 0, 8, [("s", 120), ("a", r_cont, -80), ("s", 120)]), name="", cls=2, speed_mph=45, oneway=True)
  cont.lat += trunk.lat[-1] - cont.lat[0]
  cont.lon += trunk.lon[-1] - cont.lon[0]
  _join(trunk, cont)
  ways = [mw, trunk, cont]
  if fork:
    side = G.way(23, _path(0, 0, 38, [("s", 120), ("a", r_side, 60), ("s", 120)]), name="", cls=2, speed_mph=45, oneway=True)
    side.lat += trunk.lat[-1] - side.lat[0]
    side.lon += trunk.lon[-1] - side.lon[0]
    _join(trunk, side)
    ways.append(side)
  return {w.id: w for w in ways}


def _gps(v=25.0):
  return {"dt": 0.05, "v_ego": v, "acc": 0.0, "rel": 0, "ambiguous": False}


class _Car:
  """A car driven along road points through the planner (matcher, trust, MapCurves)."""

  def __init__(self, root):
    self.store = MC.MapCurveStore(root, background=False)
    self.m, self.rm, self.trust = MC.MapCurves(self.store), MC.RoadMatcher(self.store), MC.GpsTrust()

  def at(self, x, y, heading, nav=None, frames=1, v=25.0):
    la, lo = TM.pos(x, y)
    for _ in range(frames):
      w, f = self.rm.update(la, lo, heading)
      self.m.update(la, lo, heading, w, f, nav, trust=self.trust, gps=_gps(v))
    return self.m


def _along(pts, s):
  """(x, y, heading) s m along a polyline of points."""
  for (x0, y0), (x1, y1) in zip(pts, pts[1:], strict=False):
    d = math.hypot(x1 - x0, y1 - y0)
    if s <= d:
      f = s / d if d else 0.0
      return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0
    s -= d
  (x0, y0), (x1, y1) = pts[-2], pts[-1]
  return x1, y1, math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0


def _ramp_pts(r_first_heading, legs, start=(0.0, 0.0)):
  return _path(start[0], start[1], r_first_heading, legs)


def _announce(car, d=400.0):
  """On the motorway, navigation announces the exit d m ahead (also a few frames for the trust to settle)."""
  return car.at(0, -d, 0.0, nav=("off ramp", "slight right", d), frames=120)


def test_planner_watches_the_exit_and_tells_the_link_from_the_highway():
  root, _, ramps = write_files(fork_scene(fork=False))
  car = _Car(root)
  m = _announce(car)
  assert m.nav_exit == MC.NAV_EXIT_CURVES and len(m.watches) == 1 and m.exit_shape == MC.EXIT_SHAPE_HIGHWAY
  # navigation clears at the split (its next maneuver): the exit is still watched
  m = car.at(0, -40, 0.0, frames=5)
  assert m.nav_exit == MC.NAV_NONE and m.watch is not None and m.exit_curve is not None
  # on the link 60 m past the split, heading along it (8 deg): the link's shape
  pts = _ramp_pts(8, [("s", 200)])
  x, y, h = _along(pts, 60)
  m = car.at(x + 2.5, y, h, frames=5)
  assert m.exit_shape == MC.EXIT_SHAPE_LINK, (m.exit_shape, m.exit_link, m.exit_hwy)
  assert 0 < m.exit_link[0] < m.exit_hwy[0] and m.exit_link[1] < m.exit_hwy[1]
  # stayed on the motorway in its right lane (5 m right of its line), 60 m past the split: the link is close, but our
  # heading is the highway's (no answer); 150 m past it the link is far: the highway's
  m = car.at(5.0, 60, 0.0, frames=5)
  assert m.exit_shape in (MC.EXIT_SHAPE_NONE, MC.EXIT_SHAPE_HIGHWAY), (m.exit_shape, m.exit_link, m.exit_hwy)
  m = car.at(5.0, 150, 0.0, frames=5)
  assert m.exit_shape == MC.EXIT_SHAPE_HIGHWAY, (m.exit_shape, m.exit_link, m.exit_hwy)
  # published (and an older planner's fields default to "no answer" on the car side)
  plan = _plan_ns()
  MC.MapCurvesPlanner.publish(_FakePlanner(m), plan)
  assert plan.mapExitShape == MC.EXIT_SHAPE_HIGHWAY and plan.mapExitHighwayDist > 0
  f = CTL.MapFields.from_plan(_plan_ns(mapCurveMode=2))
  assert f.exit_shape == 0 and f.exit_curve_radius == 0.0 and f.limit == 0.0


class _FakePlanner:
  def __init__(self, mc):
    self.mc, self.mode, self.road_class, self.gps_acc = mc, 2, 1, 0.0


def _plan_ns(**kw):
  from types import SimpleNamespace
  base = dict(mapCurveMode=0, mapCurveStatus=0, mapCurveDistance=0.0, mapCurveRadius=0.0, mapOnRamp=False, mapRampType=0,
              mapRampToSpeed=0.0, mapRampToHighway=False, mapRampEndDistance=0.0, mapNavExit=0, mapNavExitDistance=0.0,
              mapNavCurveRadius=0.0, mapNavCurveDistance=0.0)
  base.update(kw)
  return SimpleNamespace(**base)


def _fork_drive(r_cont, r_side, take, nav_side=0):
  """Announce the forked exit, drive the trunk to the fork, then the branch `take` ("cont" / "side"). Returns rows of
  (where, MapCurves state) at: announced, on the trunk 150 m in (fork not decided), 60 m into the branch."""
  root, curves, ramps = write_files(fork_scene(r_cont, r_side))
  car = _Car(root)
  rows = {}
  m = _announce(car)
  rows["announced"] = (m.nav_curve["r"], m.nav_curve_dist, len(m.watches[0]["forks"]))
  trunk = _ramp_pts(8, [("s", 200)])
  for s in range(10, 160, 5):
    x, y, h = _along(trunk, s)
    nav = ("fork", "slight left" if nav_side < 0 else "slight right", 200 - s) if nav_side else None
    m = car.at(x, y, h, nav=nav, frames=4)
  rows["trunk"] = (m.exit_fork, m.exit_curve["r"] if m.exit_curve else 0.0, m.status, m.radius)
  fx, fy, _ = _along(trunk, 200)
  branch = _path(fx, fy, 8, [("s", 120), ("a", r_cont, -80), ("s", 120)]) if take == "cont" else \
    _path(fx, fy, 38, [("s", 120), ("a", r_side, 60), ("s", 120)])
  for s in list(range(160, 200, 5)) + [200 + k for k in range(5, 65, 5)]:
    x, y, h = _along(trunk, s) if s < 200 else _along(branch, s - 200)
    m = car.at(x, y, h, frames=4)
  rows["branch"] = (m.exit_fork, m.exit_curve["r"] if m.exit_curve else 0.0, m.status, m.radius, m.exit_curve_dist)
  return rows, m


def test_a_fork_after_the_split_plans_the_lower_branch_until_the_car_is_on_one():
  # a 2-lane exit forking 200 m in: the continuing branch turns left at 60 m (22 mph), the side branch right at 150 m
  r_c, r_s = 60.0, 150.0
  for take in ("cont", "side"):
    rows, m = _fork_drive(r_c, r_s, take)
    nav_r, nav_d, n_forks = rows["announced"]
    assert n_forks == 1 and abs(nav_r - r_c) < 15, rows      # before the split: the lower of the two (the 60 m)
    fork, ex_r, st, r = rows["trunk"]
    assert fork == MC.FORK_OPEN and abs(ex_r - r_c) < 15, rows   # on the trunk: not decided, still the lower
    assert st == MC.STATUS_RAMP_CURVE and abs(r - r_c) < 15, rows
    fork, ex_r, st, r, d = rows["branch"]
    assert fork in (MC.FORK_NAV, MC.FORK_OTHER), (take, rows)
    if take == "cont":
      assert abs(ex_r - r_c) < 15, rows                       # the branch taken has the tight curve
    else:
      assert abs(ex_r - r_s) < 30 and (m.curve is None or abs(m.radius - r_s) < 30), (take, rows, m.radius)
  # the other way round (the side branch is the tight one): the lower until decided, then never the unused one
  r_c, r_s = 150.0, 60.0
  for take in ("cont", "side"):
    rows, m = _fork_drive(r_c, r_s, take)
    assert abs(rows["announced"][0] - r_s) < 15 and abs(rows["trunk"][1] - r_s) < 15, rows
    fork, ex_r, st, r, d = rows["branch"]
    want = r_c if take == "cont" else r_s
    assert abs(ex_r - want) < 30, (take, rows)
    if take == "cont":
      assert m.curve is None or abs(m.radius - r_c) < 30, (rows, m.radius)   # never the side branch's 60 m after


def test_navigation_says_which_branch_and_mapexitfork_tells_when_the_car_took_the_other():
  rows, m = _fork_drive(60.0, 150.0, "side", nav_side=-1)     # navigation: the left (continuing) branch; car went right
  assert rows["branch"][0] == MC.FORK_OTHER, rows
  rows, m = _fork_drive(60.0, 150.0, "cont", nav_side=-1)
  assert rows["branch"][0] == MC.FORK_NAV, rows


# ---------------- car side: the camera's lane view ----------------

def _lanes(lv, seconds, right_edge=True, left_edge=False, lines=(-5.4, -1.8, 1.8, 5.4), probs=None, valid=True, dt=0.05):
  for _ in range(int(round(seconds / dt))):
    p = probs or (0.0 if left_edge else 0.8, 0.9, 0.9, 0.0 if right_edge else 0.8)
    edges = (lines[1] - (1.0 if left_edge else 6.0), lines[2] + (1.0 if right_edge else 6.0))
    lv.update(dt, valid, p, list(lines), list(edges), [0.3, 0.3])


def test_lane_view_says_rightmost_lane_with_hysteresis_and_rides_out_dropouts():
  lv = CTL.LaneView()
  _lanes(lv, 0.8)
  assert not lv.edge[1]                                       # not yet LANE_ON_TIME
  _lanes(lv, 0.3)
  assert lv.edge[1] and not lv.edge[-1]
  _lanes(lv, 1.5, right_edge=False)                           # a lane line to our right for 1.5 s (a gore): still on
  assert lv.edge[1]
  _lanes(lv, 0.6, right_edge=False)
  assert not lv.edge[1]
  _lanes(lv, 1.2)
  _lanes(lv, 2.5, valid=False)                                # the model has no lines for 2.5 s: kept
  assert lv.edge[1]
  _lanes(lv, 1.0, valid=False)
  assert not lv.edge[1]
  # a middle lane: lines both sides, edges far
  lv = CTL.LaneView()
  _lanes(lv, 3.0, right_edge=False)
  assert not lv.edge[1] and not lv.edge[-1]
  # mirrored for a left exit
  lv = CTL.LaneView()
  _lanes(lv, 1.2, right_edge=False, left_edge=True)
  assert lv.edge[-1] and not lv.edge[1]
  # a wide shoulder (edge 4.5 m past our line) is not the edge lane
  lv = CTL.LaneView()
  for _ in range(40):
    lv.update(0.05, True, (0.8, 0.9, 0.9, 0.0), [-5.4, -1.8, 1.8, 5.4], [-8.0, 6.3], [0.3, 0.3])
  assert not lv.edge[1]


def test_lane_view_sees_a_lane_change_and_its_direction():
  lv = CTL.LaneView()
  _lanes(lv, 0.5, right_edge=False)
  lv.update(0.05, True, (0.8, 0.9, 0.9, 0.8), [-5.4, -0.2, 3.4, 7.0], [-9.0, 9.0], [0.3, 0.3])
  assert lv.change == 0
  lv.update(0.05, True, (0.8, 0.9, 0.9, 0.8), [-7.0, -3.4, 0.2, 3.8], [-9.0, 9.0], [0.3, 0.3])   # crossed to the left
  assert lv.change == -1
  lv.update(0.05, True, (0.8, 0.9, 0.9, 0.8), [-7.0, -3.3, 0.3, 3.9], [-9.0, 9.0], [0.3, 0.3])
  assert lv.change == 0
  lv.update(0.05, True, (0.8, 0.9, 0.9, 0.8), [-3.5, 0.1, 3.7, 7.3], [-9.0, 9.0], [0.3, 0.3])    # back to the right
  assert lv.change == 1


# ---------------- car side: closed loop ----------------

def drive_exit(fields, frames, set_mph, v_mph=None, split_mode=0, metric=False, lane=None, signal=None, gas=None, lead=None,
               max_drop=10):
  """drive_53 with the exit plan's inputs: lane(x) -> the exit side the camera sees us next to the edge of (None: no
  edge lane), the setting, km/h. Returns the run with the chimes [(x, why, v mph, m to the exit point)]."""
  step = KPH if metric else MPH
  set0 = set_mph * step / MPH
  car = T.Car(set0, dash_offset_mph=0.0, lag=10, accel=0.6, step=step, **TM.MEASURED_CRUISE)
  car.set_speed, car.v_ego = set_mph * step, (v_mph or set_mph) * step
  logic = cs.CurveSlowdownLogic(metric, max_drop)
  ctl = CTL.MapCurveControl(3.0, CTL.MAP_DECEL, split_mode=split_mode, metric=metric)
  mix = CTL.CurveMix(cs.CURVE_MARGIN)
  md = logic.max_drop_steps * logic.step
  x = 0.0
  r = dict(presses=[], sets=[], vs=[], xs=[], why=[], state=[], chimes=[], ctl=ctl, logic=logic, holds=0)
  for f in range(frames):
    fld = fields(x)
    side = lane(x) if lane else None
    ctl.lane.edge = {-1: side == -1, 1: side == 1}
    driver = car.set_speed + logic.offset * logic.step
    foot = gas(x) if gas else None
    mo = ctl.update(fld, driver, signal(x) if signal else 0, v_ego=car.v_ego, max_drop=md, gas=foot is not None)
    if mo.chime:
      r["chimes"].append((round(x), ctl.chime_why, round(car.v_ego / MPH, 1), round(ctl.exit_rem or 0.0)))
    active, speed, floor, budget, track, raise_ok = mix.update(False, CTL.NO_CURVE, mo,
                                                               drop_floor=max(driver - md, cs.MIN_SET_SPEED))
    inp = cs.SlowdownInputs(enabled=True, cruise_enabled=True, set_speed=car.set_speed, v_ego=car.v_ego,
                            gas_pressed=foot is not None, brake_pressed=False, standstill=False, plan_alive=True,
                            csc_active=active, csc_speed=min(speed, 100.0), other_button_active=False, driver_button=None,
                            curve_floor=floor, curve_budget=budget, curve_track=track, curve_raise=raise_ok,
                            curve_no_accel=mix.no_accel, curve_hold=mix.firm, fast_restore=mo.fast_restore,
                            btn_counter=(f // 10) % 16)    # the wheel's 10 Hz frames (holds are phase locked to them)
    btn, _ = logic.update(inp)
    if btn is not None:
      r["presses"].append((f, round(x), btn, logic.last_action, round(car.set_speed / step), round(car.v_ego / step, 1)))
      if logic.last_action == "hold":
        car.hold_frame(btn)
      else:
        car.press(btn)
    if foot is not None:
      car.gas_frame = car.frame
    car.tick()
    if foot is not None:
      car.v_ego = max(car.v_ego, foot)
    cap = lead(x) if lead else None
    if cap is not None:
      car.v_ego = min(car.v_ego, cap)
    x += car.v_ego * T.DT
    r["sets"].append(car.set_speed / step)
    r["vs"].append(car.v_ego / MPH)
    r["xs"].append(x)
    r["why"].append(mo.why)
    r["state"].append(ctl.state())
  return r


def _at(r, x_at, key="vs"):
  return next(v for v, xx in zip(r[key], r["xs"], strict=True) if xx >= x_at)


def _minus(r, lo, hi):
  return [p for p in r["presses"] if lo <= p[1] < hi and p[2] == cs.SET_MINUS]


def _plus(r, lo, hi):
  return [p for p in r["presses"] if lo <= p[1] < hi and p[2] == cs.SET_PLUS]


def exit_fields(exit_x, curve_x, r=95.0, ann=800.0, clear=46.0, limit=70.0, link_at=95.0, ramp_at=137.0, trust_from=None,
                stay=False, shape=True, curve_held=56.0, side=1, ramp_end=230.0, nav_exit=1):
  """An exit at exit_x (m driven), its tightest curve (radius r) at curve_x, announced from ann m before; navigation
  clears `clear` m before the exit point (drive 53: 46 m at E1, 9 m at E2). Past the exit point the shapes say "link"
  from link_at m (drive 53, GPS replay through the planner: E1 95 m, E2 ~120 m past navigation's exit point), the road
  match is on the link from ramp_at, its curve held (doubtful) until curve_held m before it. Before: "highway". stay:
  the car stays on the highway (shapes "highway" from 40 m past the exit point). shape=False: an older planner."""
  trust_from = exit_x - clear if trust_from is None else trust_from

  def fields(x):
    hwy = dict(road_class=1, limit=limit * MPH, gps_trust=2 if x >= trust_from and x < exit_x + ramp_at else 0)
    sh = (lambda v: v if shape else 0)  # noqa: E731
    if x < exit_x - ann:
      return F(**hwy)
    if x < exit_x - clear:
      return F(nav_exit=nav_exit, nav_exit_distance=exit_x - x, nav_curve_radius=r, nav_curve_distance=curve_x - x,
               nav_exit_side=side, exit_shape=sh(1), exit_curve_radius=r, exit_curve_distance=curve_x - x, **hwy)
    if stay:
      return F(exit_shape=sh(1 if x > exit_x + 40 else 0), **{**hwy, "gps_trust": 2 if x < exit_x + 40 else 0})
    ex = dict(exit_curve_radius=r, exit_curve_distance=max(curve_x - x, 0.0))
    if x < exit_x + link_at:
      return F(exit_shape=sh(0 if x > exit_x - 20 else 1), **ex, **hwy)
    if x < exit_x + ramp_at:
      return F(exit_shape=sh(2), **ex, **hwy)
    ramp = dict(road_class=2, on_ramp=True, ramp_type=1, ramp_to_speed=45 * MPH, ramp_end_distance=max(curve_x + ramp_end - x, 0.0))
    if x < curve_x - curve_held:
      return F(status=5, radius=r, distance=curve_x - x, road_class=2, gps_trust=2, exit_shape=sh(2), **ex)
    if x < curve_x + 127:
      return F(status=2, radius=r, distance=max(curve_x - x, 0.0), exit_shape=sh(2), **ex, **ramp)
    return F(exit_shape=sh(2), **ramp)
  return fields


V_C95 = math.sqrt(2.2 * 95.0) / MPH      # 32.3 mph
V_C85 = math.sqrt(2.2 * 85.0) / MPH      # 30.6 mph


def _e1(**kw):
  """Drive 53, E1 (t 1130-1182): 57 mph behind a car with his 75 set, announced 807 m ahead, the 95 m curve 270 m after
  the exit point, the car ahead gone 250 m before it; he signalled right 300 m before the exit point for 2 s."""
  exit_x, curve_x = 900.0, 1170.0
  fields = exit_fields(exit_x, curve_x, ann=807.0, **kw)
  return fields, exit_x, curve_x, (lambda x: 57 * MPH if x < exit_x - 250 else None), \
    (lambda x: 1 if exit_x - 300 <= x < exit_x - 250 else 0)


def test_drive_53_e1_no_slowing_below_the_limit_an_early_chime_and_the_firm_slowdown_once_confirmed():
  fields, exit_x, curve_x, lead, signal = _e1()
  r = drive_exit(fields, 6000, 75, v_mph=57, lead=lead, signal=signal)
  v_split = _at(r, exit_x)
  # below the highway's 70 already: not slowed before the split, but not let speed up into the exit either (the car
  # ahead left 250 m before it): the set speed went to 58 (his 75 + nothing pressed would have taken it to 70+)
  assert 56.5 <= v_split <= 59.5, v_split
  assert _plus(r, 0, exit_x + 95) == [], _plus(r, 0, exit_x + 95)
  # "Exit curve - brake" before the split (12 s ahead, in the exit lane by his signal): 270 m is too short from 58
  pre = [c for c in r["chimes"] if c[3] > 0]
  assert len(pre) == 1 and 200 <= pre[0][3] <= 320, r["chimes"]
  # confirmed by the shapes 95 m past the exit point, then the strongest slowdown (a SET- hold): the set speed far below
  conf = next(xx for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed")
  assert exit_x + 95 <= conf <= exit_x + 112, conf - exit_x   # (sustained EXIT_CONFIRM_TIME)
  holds = [p for p in r["presses"] if p[2] == cs.SET_MINUS and p[3] == "hold" and p[1] >= conf]
  assert holds, "the firm slowdown takes a SET- hold"
  assert _at(r, conf + 60, "sets") <= 46, _at(r, conf + 60, "sets")
  v_entry = _at(r, curve_x)
  # without his brake the cruise can only take ~0.9-1.0 m/s^2 off in 175 m: in at ~52 mph (the chime asked him to
  # brake); with nothing done (the old drive 53 build after the restore) it was 59.6
  assert 48 <= v_entry <= 55, v_entry
  assert len([c for c in r["chimes"] if c[3] < 0]) == 1      # and once more at the confirmation (too little room)
  assert min(r["sets"]) >= 25 - 1e-6


def _e2(**kw):
  """Drive 53, E2 (t 355-421): 75 mph with his 75 set, the 70 highway, announced 1478 m ahead, the 85 m curve 496 m
  after the exit point; he signalled 270 m before it. In the exit lane (the camera) from 400 m before."""
  exit_x, curve_x = 1600.0, 2096.0
  fields = exit_fields(exit_x, curve_x, r=85.0, ann=1478.0, clear=9.0, link_at=120.0, ramp_at=160.0, curve_held=325.0,
                       ramp_end=190.0, **kw)
  return fields, exit_x, curve_x


def test_drive_53_e2_slows_gently_to_the_limit_at_the_split_then_firm_after_the_confirmation():
  fields, exit_x, curve_x = _e2()
  r = drive_exit(fields, 9000, 75, lane=lambda x: 1 if x >= 1200 else None)
  v_split = _at(r, exit_x)
  assert 69.5 <= v_split <= 72.0, v_split                    # at about the limit, not lower
  pre = _minus(r, 0, exit_x)
  assert 4 <= len(pre) <= 7 and all(p[3] == "tap" for p in pre), pre   # gently: 1 mph taps, 75 -> 70
  assert _plus(r, 0, exit_x + 120) == [] and min(r["sets"][:next(i for i, xx in enumerate(r["xs"]) if xx >= exit_x)]) >= 70 - 1e-6
  first = pre[0][1]
  assert exit_x - 400 <= first <= exit_x - 150, exit_x - first  # starts only as far out as the gentle plan needs
  # the split to the confirmation: held (no SET-, no SET+)
  assert _minus(r, exit_x + 5, exit_x + 119) == [] and _plus(r, exit_x, exit_x + 119) == []
  # the chime before the split: from 70 at the split the 85 m curve 496 m on needs ~485 m after a ~3 s confirmation
  pre_chime = [c for c in r["chimes"] if c[3] > 0]
  assert len(pre_chime) == 1, r["chimes"]
  v_entry = _at(r, curve_x)
  assert 44 <= v_entry <= 52, v_entry                        # (was 38.5 on the drive's build with his signal, 30.4 after a2c9aac70)
  assert min(r["sets"]) >= 25 - 1e-6


def test_the_exit_speed_at_the_split_setting_incl_off_and_metric():
  fields, exit_x, curve_x = _e2(limit=70.0)
  lane = lambda x: 1 if x >= 900 else None  # noqa: E731
  speeds = {}
  for mode in (0, 1, 2, 3):
    r = drive_exit(fields, 6000, 85, lane=lane, split_mode=mode)
    speeds[mode] = _at(r, exit_x)
  assert 69.5 <= speeds[0] <= 72.5 and 74.5 <= speeds[1] <= 77.5 and 79.5 <= speeds[2] <= 82.5, speeds
  assert speeds[3] >= 84.5, speeds                           # Off: nothing before the split
  # metric: the highway's 110 km/h (+5 km/h), set 130 km/h
  fields_m = exit_fields(1600.0, 2096.0, r=85.0, ann=1478.0, clear=9.0, limit=110 / 3.6 / MPH, link_at=120.0, ramp_at=160.0,
                         curve_held=325.0)
  r = drive_exit(fields_m, 6000, 130, metric=True, lane=lane, split_mode=1)
  v_kph = _at(r, 1600.0) * MPH * 3.6
  assert 114.0 <= v_kph <= 119.0, v_kph
  # the highway's limit unknown: nothing before the split
  r = drive_exit(exit_fields(1600.0, 2096.0, r=85.0, ann=1478.0, clear=9.0, limit=0.0, link_at=120.0, ramp_at=160.0,
                             curve_held=325.0), 6000, 80, lane=lane)
  assert _minus(r, 0, 1600.0) == []
  # never raises: his 68 under the 70 highway
  r = drive_exit(fields, 7000, 68, lane=lane)
  assert _plus(r, 0, 1600.0) == [] and _minus(r, 0, 1600.0) == []


def test_a_middle_lane_gets_no_slowing_before_the_split_but_the_ramp_is_still_handled():
  # a 2-lane exit taken from its left lane (a lane to our right: no rightmost-lane signal) without a signal: nothing
  # before the split; a dive into the ramp at the last minute is confirmed and slowed for (with the chime)
  fields, exit_x, curve_x = _e2()
  r = drive_exit(fields, 9000, 80)
  assert _minus(r, 0, exit_x) == [] and abs(_at(r, exit_x) - 80) < 0.6
  assert [c for c in r["chimes"] if c[3] > 0] == []            # (no chime before the split outside an exit lane)
  conf = next(xx for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed")
  assert _minus(r, conf, curve_x) and len([c for c in r["chimes"] if c[3] < 0]) == 1
  assert _at(r, curve_x) < _at(r, conf) - 20
  # the same lane with his signal toward the exit (kept after it goes off): slowed to the limit before the split
  r = drive_exit(fields, 9000, 80, signal=lambda x: 1 if 900 <= x < 960 else 0)
  assert 69.5 <= _at(r, exit_x) <= 72.5 and _minus(r, 0, exit_x)


def test_no_navigation_no_exit_plan_the_ramp_as_before():
  fields, exit_x, curve_x = _e2()

  def no_nav(x):
    f = fields(x)
    f.nav_exit, f.nav_exit_distance, f.nav_curve_radius, f.exit_shape, f.exit_curve_radius = 0, 0.0, 0.0, 0, 0.0
    return f
  r = drive_exit(no_nav, 9000, 75, lane=lambda x: 1)
  assert r["ctl"].plan is None and r["ctl"].last_plan is None and r["chimes"] == []
  assert _minus(r, 0, exit_x + 160) == []                     # nothing until the ramp is matched
  ramp = _minus(r, exit_x + 160, curve_x)
  assert ramp and all(w in ("", "ramp", "street cap") for w in r["why"])


def _stay(set_mph=75, shape=True, gas=None, signal=None):
  fields, exit_x, curve_x = _e2(stay=True, shape=shape)
  return drive_exit(fields, 9000, set_mph, lane=lambda x: 1 if x >= 900 else None, gas=gas, signal=signal), exit_x


def test_a_missed_exit_restores_the_speed_at_once():
  r, exit_x = _stay()
  assert r["ctl"].plan is None and r["ctl"].last_plan.why == "missed"
  past = [(v, xx) for v, xx in zip(r["vs"], r["xs"], strict=True) if xx >= exit_x]
  assert min(v for v, _ in past) >= 69.0, min(v for v, _ in past)     # never more than the split's slowing
  # the shapes say "highway" from 40 m past the exit point: missed 1 s later, SET+ holds back to his 75
  restore = _plus(r, exit_x, exit_x + 400)
  assert len(restore) == 5 and restore[0][1] <= exit_x + 40 + 45, restore
  assert restore[-1][0] - restore[0][0] <= 5 * 25, restore     # at once: a tap every ~0.2 s (5 mph: no hold needed)
  back = next(xx for v, xx in past if v >= 74.5)
  assert back - exit_x <= 600, back - exit_x                  # back at 75 within ~17 s (the cruise's speed-up, 0.33 m/s^2 at 5 mph)
  assert abs(r["sets"][-1] - 75) < 1e-6 and max(r["sets"]) <= 75 + 1e-6
  # from his 85 (slowed to the highway's 70): SET+ holds, then taps
  r, exit_x = _stay(set_mph=85)
  restore = _plus(r, exit_x, exit_x + 400)
  assert restore and any(p[3] == "hold" for p in restore) and restore[0][1] <= exit_x + 40 + 45, restore
  assert abs(r["sets"][-1] - 85) < 1e-6 and max(r["sets"]) <= 85 + 1e-6
  # an older planner (no shape check): on the highway, not doubtful, EXIT_STAY_MARGIN past the exit point
  r, exit_x = _stay(shape=False)
  assert r["ctl"].last_plan.why == "missed"
  restore = _plus(r, exit_x, exit_x + 400)
  assert restore and exit_x + CTL.EXIT_STAY_MARGIN - 5 <= restore[0][1] <= exit_x + CTL.EXIT_STAY_MARGIN + 40, restore


def test_his_gas_or_the_other_signal_ends_the_exit_slowing_at_once():
  fields, exit_x, curve_x = _e2()
  lane = lambda x: 1 if x >= 1200 else None  # noqa: E731
  # his foot on the gas 150 m before the split while we slow: the speed comes back (SET+), nothing more before the ramp
  r = drive_exit(fields, 6000, 75, lane=lane, gas=lambda x: 0.0 if exit_x - 150 <= x < exit_x - 145 else None)
  p = r["ctl"].plan or r["ctl"].last_plan
  assert p.declined and _plus(r, exit_x - 150, exit_x) and _minus(r, exit_x - 140, exit_x + 120) == []
  assert _at(r, exit_x, "sets") >= 74
  # the other signal near the split (a lane change to the left): the same
  r = drive_exit(fields, 6000, 75, lane=lane, signal=lambda x: -1 if exit_x - 150 <= x < exit_x - 130 else 0)
  assert (r["ctl"].plan or r["ctl"].last_plan).declined and _plus(r, exit_x - 150, exit_x)
  # a lane change away seen by the camera near the split (no signal): the same
  fields2 = fields

  def lane_change(x, ctl_ref=[]):  # noqa: B006
    return 1 if x >= 1200 else None
  r2 = drive_exit(fields2, 2600, 75, lane=lane_change)
  ctl = r2["ctl"]
  ctl.lane.change = -1
  ctl.update(fields2(exit_x - 100), 75 * MPH, 0, v_ego=70 * MPH)
  assert ctl.plan.declined
  # his decision taken back by a signal toward the exit: planned again
  ctl.lane.change = 0
  ctl.update(fields2(exit_x - 95), 75 * MPH, 1, v_ego=70 * MPH)
  assert not ctl.plan.declined and ctl.plan.signalled
  # a last-minute dive after declining: still confirmed and slowed for on the ramp
  r = drive_exit(fields, 9000, 75, lane=lane, gas=lambda x: 0.0 if exit_x - 150 <= x < exit_x - 145 else None)
  conf = [xx for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed"]
  assert conf and _minus(r, conf[0], curve_x)


def test_the_plan_is_tied_to_the_exit_navigation_counts_down_to():
  ctl = CTL.MapCurveControl(3.0)
  v = 30.0

  def step(f, n=1):
    out = None
    for _ in range(n):
      out = ctl.update(f, 33.0, 0, v_ego=v)
    return out
  base = dict(road_class=1, limit=70 * MPH)
  step(F(nav_exit=1, nav_exit_distance=1200.0, nav_curve_radius=80.0, nav_curve_distance=1500.0, nav_exit_side=1, **base))
  p = ctl.plan
  assert p is not None and abs(ctl.exit_rem - 1200.0) < 1
  # navigation's distance is counted down between its updates: the same exit
  step(F(nav_exit=1, nav_exit_distance=1200.0 - 3.0 - 20.0, nav_curve_radius=80.0, nav_curve_distance=1477.0, **base))
  assert ctl.plan is p
  # a reroute: another exit 900 m farther, far from this one's split: the plan is that exit's now
  step(F(nav_exit=1, nav_exit_distance=2100.0, nav_curve_radius=60.0, nav_curve_distance=2300.0, **base))
  assert ctl.plan is not p and abs(ctl.exit_rem - 2100.0) < 2 and ctl.last_plan.why == "rerouted"
  p = ctl.plan
  # close to its split (150 m), navigation moves on to the next maneuver, an exit 1300 m on: kept, the split decides
  ctl.odo = p.exit_at - 150.0
  step(F(nav_exit=1, nav_exit_distance=1450.0, nav_curve_radius=60.0, nav_curve_distance=1600.0, **base))
  assert ctl.plan is p
  # navigation's notice clears 46 m before the exit point (drive 53): still the plan
  ctl.odo = p.exit_at - 46.0
  step(F(**base), n=100)                                     # (1 s, 30 m: NAV_GRACE does not apply this close)
  assert ctl.plan is p
  # far from it, navigation dropping it for NAV_GRACE ends it (rerouted away)
  ctl2 = CTL.MapCurveControl(3.0)
  ctl2.update(F(nav_exit=1, nav_exit_distance=1200.0, nav_curve_radius=80.0, nav_curve_distance=1500.0, **base), 33.0, 0, v_ego=v)
  for _ in range(int(CTL.NAV_GRACE / CTL.DT_CTRL) + 2):
    ctl2.update(F(**base), 33.0, 0, v_ego=v)
  assert ctl2.plan is None and ctl2.last_plan.why == "navigation dropped it"
  # a planner gap: counted on by the car's own speed (MapCurveControl.drive)
  ctl.drive(10.0)
  assert abs(ctl.exit_rem - (46.0 - 30.0 - 10.0)) < 0.5


def test_two_exits_close_together_the_first_one_taken_is_its_own_ramp():
  # navigation counts down to exit B; exit A (not ours) is 300 m before it. Passing A changes nothing; taking A (the
  # road match on its link, trusted, 300 m before B's exit point) ends B's plan: A's ramp rules from there
  fields, exit_x, curve_x = _e2()
  r = drive_exit(fields, 4500, 75, lane=lambda x: 1 if x >= 1200 else None)
  assert r["ctl"].plan is not None and abs(r["ctl"].plan.exit_at - exit_x) < 2

  def took_a(x):
    if x < exit_x - 300:
      return fields(x)
    return F(road_class=2, on_ramp=True, ramp_type=1, ramp_to_speed=45 * MPH, ramp_end_distance=300.0, status=2,
             radius=50.0, distance=max(exit_x - 150 - x, 0.0))
  r = drive_exit(took_a, 4500, 75, lane=lambda x: 1 if x >= 1200 else None)
  assert r["ctl"].plan is None and r["ctl"].last_plan.why == "an earlier exit"
  assert "ramp" in r["why"] and "nav exit" not in r["why"]
  # and that exit's (B's) announcement is not taken on again for a new plan
  ctl = r["ctl"]
  ctl.update(F(nav_exit=1, nav_exit_distance=ctl.done_at - ctl.odo, nav_curve_radius=85.0, nav_curve_distance=600.0,
               road_class=1, limit=70 * MPH), 33.0, 0, v_ego=20.0)
  assert ctl.plan is None


def test_the_brake_chime_only_when_the_curve_is_out_of_reach_once_per_exit():
  # a long exit (the 85 m curve 900 m after the split): no chime
  fields = exit_fields(1600.0, 2500.0, r=85.0, ann=1478.0, clear=9.0, link_at=60.0, ramp_at=100.0, curve_held=325.0)
  r = drive_exit(fields, 9000, 75, lane=lambda x: 1 if x >= 1200 else None)
  assert r["chimes"] == [], r["chimes"]
  assert _at(r, 2500.0) <= V_C85 + 2.0, _at(r, 2500.0)         # reached on the cruise alone
  # E1's short exit: once before the split, once at the confirmation
  fields, exit_x, curve_x, lead, signal = _e1()
  r = drive_exit(fields, 6000, 75, v_mph=57, lead=lead, signal=signal)
  assert [c[1] for c in r["chimes"]] == ["before the split", "confirmed, too little room"], r["chimes"]


def test_the_firm_follower_takes_a_set_minus_hold_and_steps_back_up_to_the_curve_speed():
  # confirmed at 60 mph 325 m before a 32 mph curve: needs ~0.9 m/s^2 (the usual follow's 0.6 is not enough) -> a SET-
  # hold far below, back up to the curve's speed before the curve
  fields = exit_fields(500.0, 840.0, ann=450.0, link_at=1.0, ramp_at=40.0, curve_held=330.0)
  r = drive_exit(fields, 4000, 60, lane=lambda x: 1)
  holds = [p for p in r["presses"] if p[2] == cs.SET_MINUS and p[3] == "hold"]
  assert holds and holds[0][1] >= 500.0, holds
  ups = _plus(r, 600.0, 840.0)
  assert ups and all(p[3] == "raise" for p in ups), ups
  v_entry = _at(r, 840.0)
  assert V_C95 - 1.0 <= v_entry <= V_C95 + 1.5, v_entry
  assert min(v for v, xx in zip(r["vs"], r["xs"], strict=True) if 840 <= xx < 940) >= V_C95 - 1.5
  # the usual follow (0.6 m/s^2 at most) would have entered faster: 380 m of room is enough for it, no hold
  fields = exit_fields(500.0, 900.0, ann=450.0, link_at=1.0, ramp_at=40.0, curve_held=390.0)
  r = drive_exit(fields, 4000, 60, lane=lambda x: 1)
  assert [p for p in r["presses"] if p[3] == "hold"] == [] and _at(r, 900.0) <= V_C95 + 1.5
