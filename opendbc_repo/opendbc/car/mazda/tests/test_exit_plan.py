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
               max_drop=10, model=None, change=None):
  """drive_53 with the exit plan's inputs: lane(x) -> the exit side the camera sees us next to the edge of (None: no
  edge lane), the setting, km/h, model(x) -> the model path's heading ahead (ModelPath, deg right positive) or None (no
  model), change(x) -> the camera's lane change this frame (-1 left, 1 right, 0). Returns the run with the chimes [(x,
  why, v mph, m to the exit point)]."""
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
    ctl.lane.change = change(x) if change else 0
    rel = model(x) if model else None
    if rel is not None:
      ctl.model.update(rel)
    driver = car.set_speed + logic.offset * logic.step
    foot = gas(x) if gas else None
    mo = ctl.update(fld, driver, signal(x) if signal else 0, v_ego=car.v_ego, max_drop=md, gas=foot is not None,
                    set_speed=car.set_speed)
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
    if mo.adopt and logic.offset > 0:
      logic._forget()             # (as the wrapper does: the set speed becomes his)
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
                stay=False, shape=True, curve_held=56.0, side=1, ramp_end=230.0, nav_exit=1, stay_from=40.0):
  """An exit at exit_x (m driven), its tightest curve (radius r) at curve_x, announced from ann m before; navigation
  clears `clear` m before the exit point (drive 53: 46 m at E1, 9 m at E2). Past the exit point the shapes say "link"
  from link_at m (drive 53, GPS replay through the planner: E1 95 m, E2 ~120 m past navigation's exit point), the road
  match is on the link from ramp_at, its curve held (doubtful) until curve_held m before it. Before: "highway". stay:
  the car stays on the highway (shapes "highway" from stay_from m past the exit point). shape=False: an older planner."""
  trust_from = exit_x - clear if trust_from is None else trust_from

  def fields(x):
    hwy = dict(road_class=1, limit=limit * MPH, gps_trust=2 if x >= trust_from and x < exit_x + ramp_at else 0)
    if exit_x - 300 <= x < exit_x + ramp_at:
      # the roads ahead from the map: the highway straight on, the link 5 deg off it (drive 53's)
      hwy.update(exit_bend_valid=True, exit_hwy_bend=0.0, exit_link_bend=5.0)
    sh = (lambda v: v if shape else 0)  # noqa: E731
    if x < exit_x - ann:
      return F(**hwy)
    if x < exit_x - clear:
      return F(nav_exit=nav_exit, nav_exit_distance=exit_x - x, nav_curve_radius=r, nav_curve_distance=curve_x - x,
               nav_exit_side=side, exit_shape=sh(1), exit_curve_radius=r, exit_curve_distance=curve_x - x, **hwy)
    if stay:
      return F(exit_shape=sh(1 if x > exit_x + stay_from else 0), **{**hwy, "gps_trust": 2 if x < exit_x + 40 else 0})
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


def test_drive_53_e1_a_short_ramp_lowers_the_split_speed_and_the_slowdown_starts_at_the_split():
  fields, exit_x, curve_x, lead, signal = _e1()
  r = drive_exit(fields, 6000, 75, v_mph=57, lead=lead, signal=signal)
  v_split = _at(r, exit_x)
  # the 95 m curve 270 m on cannot be made from the highway's 70 (or the 57 behind the car ahead, gone 250 m before the
  # split): the short ramp's split speed (B), about 45-50 mph, reached with taps before the split, never a SET+
  assert 44.5 <= v_split <= 50.5, v_split
  assert _plus(r, 0, exit_x) == [], _plus(r, 0, exit_x)
  # (the shapes alone confirm 1.5 s after their "link": past TENT_MAX the set speed goes back up to the speed meanwhile)
  assert all(p[3] == "raise" for p in _plus(r, exit_x, exit_x + 140)) and len(_plus(r, exit_x, exit_x + 140)) <= 5
  # from the split, before the shapes confirm the ramp (106 m on): the tentative slowdown (A), the set speed at most
  # TENT_GAP below the speed
  conf = next(xx for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed")
  assert exit_x + 125 <= conf <= exit_x + 140, conf - exit_x     # (no model path: the shapes alone, 1.5 s)
  tent = _minus(r, exit_x, conf)
  assert tent and all(p[5] - p[4] <= CTL.TENT_GAP / MPH + 1.01 for p in tent), tent
  v_entry = _at(r, curve_x)
  assert V_C95 - 1.0 <= v_entry <= V_C95 + 2.5, v_entry        # at about the curve's speed, nobody braking
  assert min(v for v, xx in zip(r["vs"], r["xs"], strict=True) if curve_x <= xx < curve_x + 127) >= V_C95 - 1.5
  # no chime before the split (A + B make it); with the shapes alone the confirmation comes late enough that the room
  # left reads just short (the entry is 1-2 mph over)
  assert [c[1] for c in r["chimes"]] in ([], ["confirmed, too little room"]), r["chimes"]
  assert min(r["sets"]) >= 25 - 1e-6
  # the model path is logged only (MODEL_DECIDES, outside review of e1be62454): agreeing or not, the same run
  base = (conf, _at(r, curve_x), r["presses"])
  for model in (lambda x: 5.0 if x >= exit_x + 95 else 0.0, lambda x: 0.0):
    r = drive_exit(fields, 6000, 75, v_mph=57, lead=lead, signal=signal, model=model)
    conf_m = next(xx for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed")
    assert (conf_m, _at(r, curve_x), r["presses"]) == base
    assert r["ctl"].model_v is not None or r["ctl"].plan is None     # (still computed, for the logs)


def _e2(**kw):
  """Drive 53, E2 (t 355-421): 75 mph with his 75 set, the 70 highway, announced 1478 m ahead, the 85 m curve 496 m
  after the exit point; he signalled 270 m before it (E2_SIGNAL). In the exit lane (the camera) from 400 m before."""
  exit_x, curve_x = 1600.0, kw.pop("curve_x", 2096.0)
  fields = exit_fields(exit_x, curve_x, r=85.0, ann=1478.0, clear=9.0, link_at=120.0, ramp_at=160.0,
                       curve_held=kw.pop("curve_held", 325.0), ramp_end=190.0, **kw)
  return fields, exit_x, curve_x


def E2_SIGNAL(x):
  return 1 if 1330 <= x < 1350 else 0


def SIGNAL_FAR(x):
  """his signal toward the exit 700 m before the exit point (1600)"""
  return 1 if 900 <= x < 960 else 0


def _long(**kw):
  """E2 with its curve 1500 m after the split (a long ramp: the setting's split speed)"""
  return _e2(curve_x=3100.0, curve_held=1300.0, **kw)


def test_drive_53_e2_slows_gently_to_the_short_ramp_speed_at_the_split_then_on_to_the_curve():
  fields, exit_x, curve_x = _e2()
  r = drive_exit(fields, 9000, 75, lane=lambda x: 1 if x >= 1200 else None, signal=E2_SIGNAL)
  v_split = _at(r, exit_x)
  # 496 m to the 85 m curve: short for the highway's 70 (the planned A + B need ~640 m), so ~59-64 at the split
  assert 58.5 <= v_split <= 64.5, v_split
  pre = _minus(r, 0, exit_x)
  assert 9 <= len(pre) <= 18 and all(p[3] == "tap" for p in pre), pre   # gently: 1 mph taps
  assert _plus(r, 0, exit_x) == [] and all(p[3] == "raise" for p in _plus(r, exit_x, exit_x + 180))
  assert exit_x - 300 <= pre[0][1] <= exit_x - 150, exit_x - pre[0][1]
  assert _minus(r, exit_x + 2, exit_x + 119), "the slowdown goes on from the split (tentative) before the confirmation"
  v_entry = _at(r, curve_x)
  assert V_C85 - 1.0 <= v_entry <= V_C85 + 1.5, v_entry        # (was 44-52 on dff3e3310, 30.4 after a2c9aac70)
  assert min(r["sets"]) >= 25 - 1e-6
  # (the camera's edge lane from 400 m: its chime at DECIDE_NEAR, before his signal could say more)
  assert [c[1] for c in r["chimes"]] in ([], ["before the split"]), r["chimes"]
  # the camera alone (final review: the edge lane is also the ordinary right lane): from DECIDE_NEAR, no deeper than
  # SPLIT_DECEL takes the speed there; then the ramp's own slowdown. No chime before the split on the camera alone (drive
  # 55: it chimed 300 m before an exit he stayed past): only at the confirmation
  r = drive_exit(fields, 9000, 75, lane=lambda x: 1 if x >= 1200 else None)
  assert _minus(r, 0, exit_x - CTL.DECIDE_NEAR - 1) == [] and _at(r, exit_x) >= 70.0, _at(r, exit_x)
  assert [c[1] for c in r["chimes"]] == ["confirmed, too little room"], r["chimes"]
  assert _at(r, curve_x) <= V_C85 + 11.0, _at(r, curve_x)


def test_the_exit_speed_at_the_split_setting_incl_off_and_metric():
  fields, exit_x, curve_x = _long(limit=70.0)
  lane = lambda x: 1 if x >= 900 else None  # noqa: E731
  speeds = {}
  for mode in (0, 1, 2, 3):
    r = drive_exit(fields, 6000, 85, lane=lane, split_mode=mode, signal=SIGNAL_FAR)
    speeds[mode] = _at(r, exit_x)
  assert 69.5 <= speeds[0] <= 72.5 and 74.5 <= speeds[1] <= 77.5 and 79.5 <= speeds[2] <= 82.5, speeds
  assert speeds[3] >= 84.5, speeds                           # Off: nothing before the split
  # metric: the highway's 110 km/h (+5 km/h), set 130 km/h
  fields_m = exit_fields(1600.0, 3100.0, r=85.0, ann=1478.0, clear=9.0, limit=110 / 3.6 / MPH, link_at=120.0, ramp_at=160.0,
                         curve_held=1300.0)
  r = drive_exit(fields_m, 6000, 130, metric=True, lane=lane, split_mode=1, signal=SIGNAL_FAR)
  v_kph = _at(r, 1600.0) * MPH * 3.6
  assert 114.0 <= v_kph <= 119.0, v_kph
  # the highway's limit unknown, a long ramp: nothing before the split
  r = drive_exit(exit_fields(1600.0, 3100.0, r=85.0, ann=1478.0, clear=9.0, limit=0.0, link_at=120.0, ramp_at=160.0,
                             curve_held=1300.0), 6000, 80, lane=lane, signal=SIGNAL_FAR)
  assert _minus(r, 0, 1600.0) == []
  # never raises: his 68 under the 70 highway
  r = drive_exit(fields, 7000, 68, lane=lane, signal=SIGNAL_FAR)
  assert _plus(r, 0, 1600.0) == [] and _minus(r, 0, 1600.0) == []


def test_a_short_ramp_lowers_the_split_speed_only_in_an_exit_lane_and_never_raises():
  lane = lambda x: 1 if x >= 900 else None  # noqa: E731
  # the short ramp (E2's 496 m): lower than the setting's for each setting; the long one keeps the setting's
  for mode in (0, 1, 2):
    r = drive_exit(_e2()[0], 6000, 85, lane=lane, split_mode=mode, signal=SIGNAL_FAR)
    assert 58.5 <= _at(r, 1600.0) <= 63.5, (mode, _at(r, 1600.0))
  # Off: nothing before the split (the setting wins)
  r = drive_exit(_e2()[0], 6000, 85, lane=lane, split_mode=3, signal=SIGNAL_FAR)
  assert _minus(r, 0, 1600.0) == []
  # the highway's limit unknown: the short ramp still lowers it from his speed
  r = drive_exit(_e2(limit=0.0)[0], 6000, 80, lane=lane, signal=SIGNAL_FAR)
  assert 58.5 <= _at(r, 1600.0) <= 63.5, _at(r, 1600.0)
  # not in an exit lane: his speed (no SET- before the split)
  r = drive_exit(_e2()[0], 6000, 80)
  assert _minus(r, 0, 1600.0) == []
  # already slower than the short ramp's speed: never raised
  r = drive_exit(_e2()[0], 6000, 60, lane=lane, signal=SIGNAL_FAR)
  assert _plus(r, 0, 1600.0) == [] and _minus(r, 0, 1600.0) == []
  # the camera alone: B only from DECIDE_NEAR and no deeper than SPLIT_DECEL takes it from there
  r = drive_exit(_e2()[0], 6000, 85, lane=lane)
  assert _minus(r, 0, 1600.0 - CTL.DECIDE_NEAR - 1) == [] and _at(r, 1600.0) >= 79.0, _at(r, 1600.0)
  # the speed it computes: E1's 270 m to a 95 m curve, E2's 496 m to an 85 m curve
  e1 = CTL.short_split_speed(270.0, math.sqrt(2.2 * 95.0), 70 * MPH) / MPH
  e2 = CTL.short_split_speed(496.0, math.sqrt(2.2 * 85.0), 70 * MPH) / MPH
  long_ = CTL.short_split_speed(1500.0, math.sqrt(2.2 * 85.0), 70 * MPH) / MPH
  assert 43 <= e1 <= 49 and 57 <= e2 <= 63 and abs(long_ - 70) < 1e-6, (e1, e2, long_)


def test_a_middle_lane_gets_no_slowing_before_the_split_but_the_ramp_is_still_handled():
  # a 2-lane exit taken from its left lane (a lane to our right: no rightmost-lane signal) without a signal: nothing
  # before the split, no tentative slowdown from it (no evidence); a dive into the ramp is confirmed and slowed for
  # (with the chime: the room is too short from 80)
  fields, exit_x, curve_x = _e2()
  r = drive_exit(fields, 9000, 80)
  assert _minus(r, 0, exit_x) == [] and abs(_at(r, exit_x) - 80) < 0.6
  conf = next(xx for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed")
  assert _minus(r, exit_x, conf - 5) == []                     # no exit lane evidence: held until confirmed
  assert [c for c in r["chimes"] if c[3] > 0] == []            # (no chime before the split outside an exit lane)
  assert _minus(r, conf, curve_x) and len([c for c in r["chimes"] if c[3] < 0]) == 1
  assert _at(r, curve_x) < _at(r, conf) - 15
  # the same lane with his signal toward the exit (kept after it goes off): slowed before the split
  r = drive_exit(fields, 9000, 80, signal=lambda x: 1 if 900 <= x < 960 else 0)
  assert 58.5 <= _at(r, exit_x) <= 63.5 and _minus(r, 0, exit_x)


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


def _stay(set_mph=75, shape=True, gas=None, signal=None, long=True, hwy_from=40.0):
  fields, exit_x, curve_x = (_long if long else _e2)(stay=True, shape=shape, stay_from=hwy_from)
  return drive_exit(fields, 9000, set_mph, lane=lambda x: 1 if x >= 900 else None, gas=gas, signal=signal), exit_x


def test_a_missed_exit_restores_the_speed_at_once_and_the_tentative_slowdown_costs_a_few_mph():
  r, exit_x = _stay()
  assert r["ctl"].plan is None and r["ctl"].last_plan.why == "missed" and r["ctl"].last_plan.tentative
  past = [(v, xx) for v, xx in zip(r["vs"], r["xs"], strict=True) if xx >= exit_x]
  v_split = _at(r, exit_x)
  # the tentative slowdown from the split until the shapes say "highway" (40 m on, 1 s): a few mph
  assert min(v for v, _ in past) >= v_split - 3.5, (v_split, min(v for v, _ in past))
  restore = _plus(r, exit_x, exit_x + 400)
  assert restore and restore[0][1] <= exit_x + 40 + 45, restore
  back = next(xx for v, xx in past if v >= 74.5)
  assert back - exit_x <= 600, back - exit_x
  assert abs(r["sets"][-1] - 75) < 1e-6 and max(r["sets"]) <= 75 + 1e-6
  # the bound (TENT_MAX): however late the shapes say "highway" (drive 53's split geometry: 1.3-4.3 s for a car going
  # on 4-7 m right of the highway's line; here up to 250 m, the last resort), at most ~3.5 mph under the split speed
  for hf in (90.0, 135.0, 180.0):
    r, exit_x = _stay(hwy_from=hf)
    low = min(v for v, xx in zip(r["vs"], r["xs"], strict=True) if xx >= exit_x)
    assert low >= _at(r, exit_x) - 3.5 and r["ctl"].last_plan.why == "missed", (hf, _at(r, exit_x), low)
  # from his 85 (his signal far out: slowed to the highway's 70): SET+ holds, then taps
  r, exit_x = _stay(set_mph=85, signal=SIGNAL_FAR)
  restore = _plus(r, exit_x, exit_x + 400)
  assert restore and any(p[3] == "hold" for p in restore) and restore[0][1] <= exit_x + 40 + 45, restore
  assert abs(r["sets"][-1] - 85) < 1e-6 and max(r["sets"]) <= 85 + 1e-6
  # no shape answer (an older planner): the road match on the highway, trusted, HWY_MATCH_STAY past EXIT_STAY_MARGIN is
  # staying on (final review: held, then adopted 20-50 mph under his speed)
  r, exit_x = _stay(shape=False)
  restore = _plus(r, exit_x, exit_x + 400)
  assert r["ctl"].last_plan.why == "missed" and restore and restore[0][1] <= exit_x + CTL.EXIT_STAY_MARGIN + 80, restore
  assert abs(r["sets"][-1] - 75) < 1e-6
  # the model path saying "highway" too: no sooner (logged only, MODEL_DECIDES)
  fields, exit_x, _ = _long(stay=True)
  r = drive_exit(fields, 9000, 75, lane=lambda x: 1 if x >= 900 else None, model=lambda x: 0.0)
  restore = _plus(r, exit_x, exit_x + 400)
  assert r["ctl"].last_plan.why == "missed" and restore and exit_x + 40 + 30 < restore[0][1] <= exit_x + 40 + 45, restore
  # a short ramp he does not take, the camera's edge lane alone (the ordinary right lane): no B from far out (final
  # review: 47 mph at the split from his 75), the same few mph from the split on
  r, exit_x = _stay(long=False)
  low = min(v for v, xx in zip(r["vs"], r["xs"], strict=True) if xx >= exit_x)
  assert _minus(r, 0, exit_x - CTL.DECIDE_NEAR - 1) == [] and _at(r, exit_x) >= 70.0, _at(r, exit_x)
  assert low >= _at(r, exit_x) - 3.5 and low >= 75 - 9.0, (_at(r, exit_x), low)


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


def test_the_brake_chime_only_when_even_the_short_ramp_speed_cannot_make_it():
  # a long exit (the 85 m curve 900 m after the split): no chime
  fields = exit_fields(1600.0, 2500.0, r=85.0, ann=1478.0, clear=9.0, link_at=60.0, ramp_at=100.0, curve_held=325.0)
  r = drive_exit(fields, 9000, 75, lane=lambda x: 1 if x >= 1200 else None)
  assert r["chimes"] == [], r["chimes"]
  assert _at(r, 2500.0) <= V_C85 + 2.0, _at(r, 2500.0)
  # E1's short exit with the exit lane known in time: A + B make it, no chime (test above)
  # ... but his signal only 120 m before the split at 70 mph: too late to get down to the short ramp's speed: once
  # before the split, and at the confirmation
  fields, exit_x, curve_x, _lead, _sig = _e1()
  r = drive_exit(fields, 6000, 70, signal=lambda x: 1 if exit_x - 120 <= x < exit_x - 100 else 0)
  whys = [c[1] for c in r["chimes"]]
  assert whys[:1] == ["before the split"] and whys.count("before the split") == 1, r["chimes"]
  assert 90 <= r["chimes"][0][3] <= 125, r["chimes"]
  # no exit lane evidence at all: only the confirmation can tell
  r = drive_exit(fields, 6000, 70)
  assert [c[1] for c in r["chimes"]] == ["confirmed, too little room"], r["chimes"]


def test_the_firm_follower_takes_a_set_minus_hold_and_steps_back_up_to_the_curve_speed():
  # no exit lane evidence (nothing before the split, no tentative slowdown), confirmed at 60 mph 325 m before a 32 mph
  # curve: needs ~0.9 m/s^2 (the usual follow's 0.6 is not enough) -> a SET- hold far below, back up to the curve's
  # speed before the curve
  fields = exit_fields(500.0, 840.0, ann=450.0, link_at=-25.0, ramp_at=40.0, curve_held=330.0)
  r = drive_exit(fields, 4000, 60)
  holds = [p for p in r["presses"] if p[2] == cs.SET_MINUS and p[3] == "hold"]
  assert holds and holds[0][1] >= 500.0, holds
  ups = _plus(r, 600.0, 840.0)
  assert ups and all(p[3] == "raise" for p in ups), ups
  v_entry = _at(r, 840.0)
  assert V_C95 - 1.0 <= v_entry <= V_C95 + 1.5, v_entry
  assert min(v for v, xx in zip(r["vs"], r["xs"], strict=True) if 840 <= xx < 940) >= V_C95 - 1.5
  # the usual follow (0.6 m/s^2 at most) would have entered faster: 380 m of room is enough for it, no hold
  fields = exit_fields(500.0, 1000.0, ann=450.0, link_at=-25.0, ramp_at=40.0, curve_held=490.0)
  r = drive_exit(fields, 4500, 60)
  assert [p for p in r["presses"] if p[3] == "hold"] == [] and _at(r, 1000.0) <= V_C95 + 1.5


# ---------------- safety review of dff3e3310 ----------------

def _hwy_stay(shape_fn=None, cls_fn=None, trust_fn=None, set_mph=78, lane=lambda x: 1, gas=None, model=None, signal=None,
              exit_x=1600.0, curve_x=1900.0, r=60.0, limit=70.0, ramp_fn=None):
  """falseconf.py: an exit 300 m long to a 60 m curve; he stays on in the lane next to it. shape_fn(x): the shapes."""
  def fields(x):
    hw = dict(road_class=cls_fn(x) if cls_fn else 1, limit=limit * MPH, gps_trust=trust_fn(x) if trust_fn else 0,
              on_ramp=bool(ramp_fn and ramp_fn(x)))
    if exit_x - 300 <= x < exit_x + 400:
      hw.update(exit_bend_valid=True, exit_hwy_bend=0.0, exit_link_bend=5.0)
    ex = dict(exit_curve_radius=r, exit_curve_distance=max(curve_x - x, 0.0)) if x < curve_x + 40 else {}
    sh = shape_fn(x) if shape_fn else (1 if x > exit_x + 40 else 0)
    if x < exit_x - 1400:
      return F(**hw)
    if x < exit_x - 46:
      return F(nav_exit=1, nav_exit_distance=exit_x - x, nav_curve_radius=r, nav_curve_distance=curve_x - x,
               nav_exit_side=1, exit_shape=sh, **ex, **hw)
    return F(exit_shape=sh, **ex, **hw)
  return drive_exit(fields, 12000, set_mph, lane=lane, gas=gas, model=model, signal=signal), exit_x


def _past(r, exit_x):
  return [v for v, xx in zip(r["vs"], r["xs"], strict=True) if xx >= exit_x]


def test_review_noisy_or_alternating_shapes_never_confirm_on_the_highway():
  # "link" every other answer from 60 m before the exit point to 300 m past it (a coin-flip heading in the lane next to
  # the exit), then "highway": was confirmed and slowed to 37 mph with the traffic at 78
  for every in (2, 3):
    alt = lambda x, k=every: (2 if int(x / 1.5) % k == 0 else 0) if 1540 <= x < 1900 else (1 if x >= 1900 else 0)  # noqa: E731
    r, exit_x = _hwy_stay(shape_fn=alt)
    assert "confirmed" not in r["state"], every
    v_split = _at(r, exit_x)
    assert min(_past(r, exit_x)) >= v_split - 4.0, (every, v_split, min(_past(r, exit_x)))
    assert r["ctl"].last_plan.why == "missed" and abs(r["sets"][-1] - 78) < 1e-6


def test_review_a_false_confirmation_is_overturned_and_restored_at_once():
  # the shapes say "link" 1.6 s without a gap (a drift toward the opening, no model path) 20-90 m past the exit point,
  # then "highway": confirmed (shapes alone), then overturned while not on the link for sure: back to his speed
  sh = lambda x: 2 if 1620 <= x < 1690 else (1 if x >= 1690 else 0)  # noqa: E731
  r, exit_x = _hwy_stay(shape_fn=sh)
  assert "confirmed" in r["state"] and r["ctl"].last_plan.why == "overturned: still on the highway"
  over = next(xx for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed")
  restore = _plus(r, over, over + 300)
  assert restore and restore[0][1] <= 1690 + 55, restore
  assert min(_past(r, exit_x)) >= _at(r, exit_x) - 12.0 and abs(r["sets"][-1] - 78) < 1e-6
  # the model path following the highway: the same (logged only, MODEL_DECIDES)
  r2, _ = _hwy_stay(shape_fn=sh, model=lambda x: 0.0)
  assert r2["presses"] == r["presses"] and r2["ctl"].last_plan.why == "overturned: still on the highway"
  # his gas after a confirmation by a (wrong) road match, the shapes not saying "link": overturned at once (was: gas held
  # 300 m still ended at 38)
  match = lambda x: 2 if 1610 <= x < 1700 else 1  # noqa: E731
  r, exit_x = _hwy_stay(shape_fn=lambda x: 0, cls_fn=match, gas=lambda x: 0.0 if 1705 <= x < 1710 else None)
  assert r["ctl"].last_plan.why == "overturned: gas" and abs(r["sets"][-1] - 78) < 1e-6
  # ... the other signal too
  r, exit_x = _hwy_stay(shape_fn=lambda x: 0, cls_fn=match, signal=lambda x: -1 if 1705 <= x < 1720 else 0)
  assert r["ctl"].last_plan.why == "overturned: the other signal"
  # with the shapes saying "link" (the real ramp: a gas tap, a fork's keep-left signal): not overturned (final review)
  for kw in (dict(gas=lambda x: 0.0 if 1700 <= x < 1705 else None), dict(signal=lambda x: -1 if 1700 <= x < 1720 else 0)):
    r, exit_x = _hwy_stay(shape_fn=lambda x: 2 if 1620 <= x < 2400 else 0, **kw)
    assert "overturned" not in r["ctl"].last_plan.why, (kw, r["ctl"].last_plan.why)
  # once on the ramp for sure (matched, trusted, LINK_SETTLE) not any more (the shapes silent)
  ctl = CTL.MapCurveControl(3.0)
  ctl.update(F(nav_exit=1, nav_exit_distance=50.0, nav_curve_radius=60, nav_curve_distance=300, nav_exit_side=1, road_class=1),
             30.0, 0, v_ego=25.0)
  ctl.odo = ctl.plan.exit_at + 20
  for _ in range(int((CTL.LINK_SETTLE + CTL.LINK_HOLD) / CTL.DT_CTRL) + 10):
    ctl.update(F(road_class=2, exit_curve_radius=60, exit_curve_distance=200), 30.0, 0, v_ego=20.0)
  assert ctl.plan.confirmed and ctl.plan.settled
  ctl.update(F(road_class=2, exit_curve_radius=60, exit_curve_distance=190), 30.0, -1, v_ego=20.0, gas=True)
  assert ctl.plan is not None and ctl.plan.confirmed


def test_review_a_short_wrong_road_match_does_not_confirm():
  # a 34 m wrong match on the link (trust 0) just past the exit point: was confirmed after LINK_HOLD (0.5 s)
  # (final review: with the planner's mapOnRamp too, as the planner sets it on such a match)
  for lo in (1620, 1560):
    for ramp in (False, True):
      wrong = lambda x, lo=lo: lo <= x < lo + 34  # noqa: E731
      r, exit_x = _hwy_stay(cls_fn=lambda x, w=wrong: 2 if w(x) else 1, ramp_fn=wrong if ramp else None)
      assert "confirmed" not in r["state"], (lo, ramp)
      assert min(_past(r, exit_x)) >= _at(r, exit_x) - 4.0


def test_review_a_gps_pause_at_the_split_holds_the_speed_never_a_raise_on_the_ramp():
  # misc.py P: the exit taken, GPS paused from the split for 400 m (no shapes, no road match): was a fast SET+ 70 -> 80
  # on the ramp (the r85 curve at 77 mph)
  fields, exit_x, curve_x = _e2()

  def paused(x):
    if exit_x - 46 <= x < exit_x + 400:
      return F(gps_trust=3, road_class=0, limit=70 * MPH)
    return fields(x)
  r = drive_exit(paused, 9000, 80, lane=lambda x: 1 if x >= 1200 else None, signal=E2_SIGNAL)
  # (the tentative slowdown's TENT_MAX: the set speed back up to the speed, never more)
  ups = _plus(r, exit_x, exit_x + 400)
  assert all(p[3] == "raise" and p[4] <= p[5] + 0.5 for p in ups) and max(
    st - v for st, v, xx in zip(r["sets"], r["vs"], r["xs"], strict=True) if exit_x <= xx < exit_x + 400) <= 1.0, ups
  # (blind for 400 of the 496 m to the curve: the tentative slowdown, then held; the chime once the ramp is matched)
  assert _at(r, curve_x) <= 62.0 and "confirmed, too little room" in [c[1] for c in r["chimes"]], (_at(r, curve_x), r["chimes"])


def test_review_a_signal_the_other_way_clears_his_signal_and_losing_the_lane_holds():
  fields, exit_x, curve_x = _long()
  # right far out (into the right lane), later left (the middle lane, no edge lane): nothing before the split
  r = drive_exit(fields, 9000, 80, signal=lambda x: 1 if exit_x - 1300 <= x < exit_x - 1250 else (
    -1 if exit_x - 600 <= x < exit_x - 560 else 0))
  assert _minus(r, 0, exit_x) == [] and not (r["ctl"].plan or r["ctl"].last_plan).signalled
  # the camera's edge lane on, off for 650 m, on again (churn.py: 80 -> 72 -> 79 -> ...): no SET+ before the split; with
  # his signal far out the slowdown from there, held while the lane is lost
  fields, exit_x, curve_x = _e2()
  lane = lambda x: 1 if (exit_x - 900 <= x < exit_x - 250 or x >= exit_x - 120) else None  # noqa: E731
  r = drive_exit(fields, 9000, 80, lane=lane, signal=lambda x: 1 if exit_x - 900 <= x < exit_x - 880 else 0)
  assert _plus(r, exit_x - 900, exit_x) == [], _plus(r, exit_x - 900, exit_x)
  assert _at(r, exit_x) <= 66.0, _at(r, exit_x)
  # the camera alone: nothing before DECIDE_NEAR, never a SET+
  r = drive_exit(fields, 9000, 80, lane=lane)
  assert _plus(r, exit_x - 900, exit_x) == [] and _minus(r, 0, exit_x - CTL.DECIDE_NEAR - 1) == []


def test_review_the_ramps_limit_before_the_split_is_never_the_highways():
  # limit.py: the highway's limit unknown, the ramp's 45 shows up 250 m before the split (still matched to the highway)
  fields0, exit_x, curve_x = _long(limit=0.0, stay=True)

  def f(x):
    g = fields0(x)
    g.limit = 45 * MPH if x >= exit_x - 250 else 0.0
    return g
  r = drive_exit(f, 9000, 75, lane=lambda x: 1 if x >= 900 else None)
  assert min(r["sets"]) >= 74 - 1e-6, min(r["sets"])

  # the highway's 70 known, then the ramp's 45 250 m before the split: still 70
  def g(x):
    h = fields0(x)
    h.limit = 45 * MPH if x >= exit_x - 250 else 70 * MPH
    return h
  r = drive_exit(g, 9000, 80, lane=lambda x: 1 if x >= 900 else None, signal=SIGNAL_FAR)
  assert 69.5 <= _at(r, exit_x) <= 72.5, _at(r, exit_x)


def test_review_kept_from_speeding_up_never_far_below_the_split_speed():
  # misc.py V2: behind a truck at 45 until 290 m before the split (the 70 highway), he stays on: was kept at 46
  fields, exit_x, curve_x = _long(stay=True)
  r = drive_exit(fields, 9000, 80, lane=lambda x: 1 if x >= 900 else None, lead=lambda x: 45 * MPH if x < exit_x - 290 else None)
  assert _at(r, exit_x, "sets") >= 60 - 1e-6, _at(r, exit_x, "sets")
  # the limit unknown: not kept at all
  fields, exit_x, curve_x = _long(limit=0.0, stay=True)
  r = drive_exit(fields, 9000, 80, lane=lambda x: 1 if x >= 900 else None, lead=lambda x: 60 * MPH if x < exit_x - 280 else None)
  assert _minus(r, 0, exit_x) == []


def test_review_the_maps_link_start_is_the_split_point():
  # navigation's exit point 60 m before the map's link start: the missed exit test counts from the map's
  ctl = CTL.MapCurveControl(3.0)
  base = dict(road_class=1, limit=70 * MPH)
  ctl.update(F(nav_exit=1, nav_exit_distance=400.0, nav_curve_radius=60, nav_curve_distance=700, nav_exit_side=1,
               exit_start=460.0, **base), 33.0, 0, v_ego=30.0)
  assert abs(ctl.exit_rem - 460.0) < 1.0
  ctl.update(F(nav_exit=1, nav_exit_distance=400.0, nav_curve_radius=60, nav_curve_distance=700, nav_exit_side=1,
               exit_start=900.0, **base), 33.0, 0, v_ego=30.0)
  assert abs(ctl.exit_rem - 460.0) < 1.5                         # (one far off navigation's is not taken)
  # re-identified as an exit without curves: not the old curve
  ctl.update(F(nav_exit=2, nav_exit_distance=455.0, nav_exit_side=1, **base), 33.0, 0, v_ego=30.0)
  assert ctl.plan.curve[0] == 0.0


def test_review_road_edge_stds_missing_means_not_sure():
  lv = CTL.LaneView()
  for _ in range(40):
    lv.update(0.05, True, (0.8, 0.9, 0.9, 0.0), [-5.4, -1.8, 1.8, 5.4], [-8.0, 2.8], [])
  assert not lv.edge[1]
  for _ in range(40):
    lv.update(0.05, True, (0.8, 0.9, 0.9, 0.0), [-5.4, -1.8, 1.8, 5.4], [-8.0, 2.8], [0.3, 0.3])
  assert lv.edge[1] and lv.left == -1.0                          # (modelV2: y positive to the right)
  # an older model with y to the left: read from our two lines all the same
  lv = CTL.LaneView()
  for _ in range(40):
    lv.update(0.05, True, (0.8, 0.9, 0.9, 0.0), [5.4, 1.8, -1.8, -5.4], [8.0, -2.8], [0.3, 0.3])
  assert lv.edge[1] and lv.left == 1.0


def test_the_model_path_heading_ahead_against_our_lane():
  import numpy as np
  xs = list(np.linspace(0, 120, 25))
  straight = [0.0 for _ in xs]
  # the path turning off 5 deg to the right from 40 m on (modelV2: y positive to the right, MODEL_LEFT -1; final review)
  assert CTL.MODEL_LEFT == -1.0
  right = [0.0 if x < 40 else (x - 40) * math.tan(math.radians(5.0)) for x in xs]
  lanes = [(xs, [-1.8] * len(xs)), (xs, [1.8] * len(xs))]
  assert abs(CTL.model_path_heading(xs, straight, lanes)) < 0.1
  assert abs(CTL.model_path_heading(xs, right, lanes) - 5.0) < 0.3
  # ... as LaneView reads the side from these lines
  lv = CTL.LaneView()
  _lanes(lv, 0.5)
  assert lv.left == -1.0 and abs(CTL.model_path_heading(xs, right, lanes, lv.left) - 5.0) < 0.3
  # the car itself turned 2 deg in its lane (lines and path rotated alike): still the path against the lane
  rot = lambda ys, a: [y + x * math.tan(math.radians(a)) for x, y in zip(xs, ys, strict=True)]  # noqa: E731
  assert abs(CTL.model_path_heading(xs, rot(right, 2.0), [(xs, rot([-1.8] * 25, 2.0)), (xs, rot([1.8] * 25, 2.0))]) - 5.0) < 0.4
  assert CTL.model_path_heading(xs[:10], straight[:10], lanes) is None          # too short (slow)
  # the verdict against the map's roads ahead
  mp = CTL.ModelPath()
  f = F(exit_bend_valid=True, exit_hwy_bend=0.0, exit_link_bend=5.0)
  mp.update(4.6)
  assert mp.verdict(f) == 2
  mp.update(0.4)
  assert mp.verdict(f) == 1
  mp.update(2.5)
  assert mp.verdict(f) == 0
  mp.tick(CTL.MODEL_STALE + 0.1)
  assert mp.verdict(f) is None                                    # stale: a dropout
  mp.update(4.6)
  assert mp.verdict(F(exit_bend_valid=True, exit_hwy_bend=0.0, exit_link_bend=2.0)) is None   # the roads not apart


def test_planner_shape_check_needs_a_heading_gap_and_speed_and_gives_the_roads_ahead():
  root, _, ramps = write_files(fork_scene(fork=False))
  car = _Car(root)
  _announce(car)
  pts = _ramp_pts(8, [("s", 200)])
  x, y, h = _along(pts, 60)
  m = car.at(x + 2.5, y, h, frames=5)
  assert m.exit_shape == MC.EXIT_SHAPE_LINK
  hb, lb = m.exit_bends
  assert hb is not None and abs(hb) < 1.0 and lb is not None and 5.0 <= lb <= 11.0, (hb, lb)
  m = car.at(x + 2.5, y, h, frames=5, v=5.0)                     # a jam: no answer
  assert m.exit_shape == MC.EXIT_SHAPE_NONE
  # our heading only 0.5 deg closer to the link's than the highway's (in between): no "link"
  m = car.at(x + 2.5, y, 3.5, frames=5)
  assert m.exit_shape != MC.EXIT_SHAPE_LINK, (m.exit_link, m.exit_hwy)


# ---------------- final review of 7d155ce18 ----------------

def _right_lane(set_mph, r=95.0, curve_after=270.0, hwy_from=40.0, lane_from=2000.0, signal=None, change=None, ex=3000.0,
                frames=16000):
  """s1_bstay.py: navigation says exit at ex (a short ramp), he is in the camera's edge lane (the ordinary right lane)
  from lane_from m before it and stays on (the shapes "highway" from hwy_from m past, None: no answer)."""
  def fields(x):
    hw = dict(road_class=1, limit=70 * MPH)
    sh = 1 if (hwy_from is not None and x > ex + hwy_from) else 0
    if x < ex - 2500:
      return F(**hw)
    if x < ex - 46:
      return F(nav_exit=1, nav_exit_distance=ex - x, nav_curve_radius=r, nav_curve_distance=ex + curve_after - x,
               nav_exit_side=1, exit_shape=sh, **hw)
    return F(exit_shape=sh, **hw)
  return drive_exit(fields, frames, set_mph, lane=lambda x: 1 if ex - lane_from < x < ex + 300 else None, signal=signal,
                    change=change), ex


def _under(r, set_mph, mph):
  """s more than `mph` under HIS speed"""
  return sum(1 for v in r["vs"] if v < set_mph - mph) * CTL.DT_CTRL


def test_final_review_the_camera_alone_slows_only_near_the_split_and_gently():
  # was: from 1.2-1.7 km before the split, 47 mph at it from his 75 / 80, 27-66 s more than 20 mph under his speed
  for set_mph in (75, 80):
    for r_, after in ((95.0, 270.0), (60.0, 200.0)):
      for hf in (40.0, 135.0, None):
        r, ex = _right_lane(set_mph, r_, after, hf)
        assert _minus(r, 0, ex - CTL.DECIDE_NEAR - 1) == [], (set_mph, r_, hf)
        assert _at(r, ex) >= set_mph - 5.0, (set_mph, r_, hf, _at(r, ex))
        assert min(r["vs"]) >= set_mph - 7.0 and _under(r, set_mph, 10) == 0.0, (set_mph, r_, hf, min(r["vs"]))
        back = next(xx for v, xx in zip(r["vs"], r["xs"], strict=True) if xx > ex and v >= set_mph - 1)
        assert back - ex <= 320, (set_mph, r_, hf, back - ex)
        assert abs(r["sets"][-1] - set_mph) < 1e-6, (set_mph, r_, hf, r["sets"][-1])
  # his signal toward the exit (1.8 km before it): the short ramp's split speed from far out, as before
  r, ex = _right_lane(75, signal=lambda x: 1 if 1200 <= x < 1300 else 0)
  assert _minus(r, 0, ex - 900) and _at(r, ex) <= 50.5, _at(r, ex)


def test_final_review_a_lane_change_away_while_slowing_ends_it_at_any_distance():
  # his signal 1.8 km before the exit, the slowdown on; then he changes lanes to the left 700 m before it (no signal):
  # the slowing ends (restored) and his signal no longer counts; farther than DECIDE_NEAR not declined (re-review: the
  # camera's edge lane, here still on, may take it again from DECIDE_NEAR, gently)
  r, ex = _right_lane(75, signal=lambda x: 1 if 1200 <= x < 1300 else 0, change=lambda x: -1 if 2300 <= x < 2301.5 else 0)
  p = r["ctl"].plan or r["ctl"].last_plan
  assert not p.declined and not p.signalled
  assert _plus(r, 2300, 2400) and _minus(r, 2310, ex - CTL.DECIDE_NEAR - 1) == [], _minus(r, 2310, ex - CTL.DECIDE_NEAR)
  assert _at(r, ex) >= 70.0, _at(r, ex)
  # 200 m before it: declined
  r, ex = _right_lane(75, signal=lambda x: 1 if 1200 <= x < 1300 else 0, change=lambda x: -1 if 2800 <= x < 2801.5 else 0)
  assert (r["ctl"].plan or r["ctl"].last_plan).declined and _minus(r, 2810, ex + 400) == []


def test_final_review_on_the_real_ramp_gas_a_keep_left_signal_or_a_lane_change_do_not_undo_it():
  # E1 / E2 timelines (the shapes "link"): was overturned, E1 35-49 mph into its 32 mph curve, E2 up to 40 into 31
  fields1, ex1, cx1, lead, sig = _e1()
  fields2, ex2, cx2 = _e2()
  for at in (150.0, 180.0, 220.0):
    for kind in ("gas", "signal", "change"):
      gas = (lambda x, a=at: 0.0 if ex1 + a <= x < ex1 + a + 3 else None) if kind == "gas" else None
      chg = (lambda x, a=at: -1 if ex1 + a <= x < ex1 + a + 0.2 else 0) if kind == "change" else None
      s1 = (lambda x, a=at: -1 if ex1 + a <= x < ex1 + a + 30 else sig(x)) if kind == "signal" else sig
      r = drive_exit(fields1, 6000, 75, v_mph=57, lead=lead, signal=s1, gas=gas, change=chg)
      assert "overturned" not in r["ctl"].last_plan.why and _at(r, cx1) <= V_C95 + 2.0, ("E1", at, kind, _at(r, cx1))
      gas = (lambda x, a=at: 0.0 if ex2 + a <= x < ex2 + a + 3 else None) if kind == "gas" else None
      chg = (lambda x, a=at: -1 if ex2 + a <= x < ex2 + a + 0.2 else 0) if kind == "change" else None
      s2 = (lambda x, a=at: -1 if ex2 + a <= x < ex2 + a + 30 else E2_SIGNAL(x)) if kind == "signal" else E2_SIGNAL
      r = drive_exit(fields2, 9000, 75, lane=lambda x: 1 if x >= 1200 else None, signal=s2, gas=gas, change=chg)
      assert "overturned" not in r["ctl"].last_plan.why and not r["ctl"].last_plan.declined, ("E2", at, kind)
      assert _at(r, cx2) <= V_C85 + (6.0 if kind == "gas" else 1.5), ("E2", at, kind, _at(r, cx2))


def test_final_review_the_planners_ramp_match_needs_link_confirm_too():
  # s4_rampmatch.py: a 34-60 m wrong match with mapOnRamp, trust 0, just past the split: was confirmed and settled at
  # once (RAMP_HOLD), 78 -> 32 on the highway for 64 s
  for lo, hi in ((20, 54), (20, 60), (20, 80), (-40, 10)):
    def fields(x, lo=lo, hi=hi):
      ex = 1600.0
      wrong = ex + lo <= x < ex + hi
      hw = dict(road_class=2 if wrong else 1, limit=70 * MPH, gps_trust=0, on_ramp=wrong, ramp_type=1, ramp_end_distance=400.0)
      e = dict(exit_curve_radius=95.0, exit_curve_distance=max(ex + 300 - x, 0.0)) if x < ex + 340 else {}
      sh = 1 if x > ex + 40 else 0
      if x < ex - 1400:
        return F(**hw)
      if x < ex - 46:
        return F(nav_exit=1, nav_exit_distance=ex - x, nav_curve_radius=95.0, nav_curve_distance=ex + 300 - x,
                 nav_exit_side=1, exit_shape=sh, **e, **hw)
      return F(exit_shape=sh, **e, **hw)
    r = drive_exit(fields, 12000, 78, lane=lambda x: 1)
    assert "confirmed" not in r["state"], (lo, hi)
    assert min(r["vs"]) >= 78 - 9.0 and abs(r["sets"][-1] - 78) < 1e-6, (lo, hi, min(r["vs"]))


def test_final_review_no_answer_past_the_split_is_never_adopted_low_on_the_highway():
  # s3_tent.py / s1_bstay.py: no shape answer past the split, the road match on the highway: was held, then adopted 20-50
  # mph under his speed. The trusted match is staying on (missed: his speed back)
  r, ex = _right_lane(80, hwy_from=None)
  assert r["ctl"].last_plan.why == "missed" and abs(r["sets"][-1] - 80) < 1e-6, r["ctl"].last_plan.why
  # the match on the highway but doubtful (trust low) to just before EXIT_LOST_MAX, then trusted: held, then his speed
  # back with taps (not adopted); doubtful or paused then: adopted (it may be the ramp; re-review of d4f848215)
  for trust, ok_at, why, end in ((2, 795.0, "not confirmed, on the highway", 80.0), (2, 1e9, "not confirmed", None),
                                 (3, 1e9, "not confirmed", None)):
    def fields(x, trust=trust, ok_at=ok_at):
      ex = 1600.0
      hw = dict(road_class=1, limit=70 * MPH, gps_trust=trust if ex - 46 <= x < ex + ok_at else 0)
      if x < ex - 1400:
        return F(**hw)
      if x < ex - 46:
        return F(nav_exit=1, nav_exit_distance=ex - x, nav_curve_radius=95.0, nav_curve_distance=ex + 270 - x,
                 nav_exit_side=1, exit_shape=0, **hw)
      return F(exit_shape=0, **hw)
    r = drive_exit(fields, 14000, 80, lane=lambda x: 1, signal=lambda x: 1 if 900 <= x < 960 else 0)
    p = r["ctl"].last_plan
    assert p.why == why, (trust, ok_at, p.why)
    ended = next(xx for s, xx in zip(r["state"], r["xs"], strict=True) if xx > 1600 + CTL.EXIT_LOST_MAX and s == "")
    assert ended - 1600 <= CTL.EXIT_LOST_MAX + 30, ended
    if end is not None:
      ups = _plus(r, 1600, 1600 + CTL.EXIT_LOST_MAX)              # (held: up to the speed at most, never a restore)
      assert abs(r["sets"][-1] - end) < 1e-6 and all(u[3] == "raise" and u[4] <= u[5] + 0.5 for u in ups), (r["sets"][-1], ups)
    else:
      assert r["sets"][-1] < 70, r["sets"][-1]


def test_final_review_the_tentative_slowdown_is_bounded_whatever_carries_it_on():
  # s3_tent.py: the shapes "link" past the split for 150-600 m while the model path says "highway" (was carried on, 27 mph
  # under), and "link" 34 m / nothing 6 m over and over with no model path (ratcheted down)
  def run(shape_fn, model=None):
    ex = 3000.0

    def fields(x):
      hw = dict(road_class=1, limit=70 * MPH, gps_trust=2, exit_bend_valid=True, exit_hwy_bend=0.0, exit_link_bend=5.0)
      if x < ex - 2500:
        return F(**hw)
      if x < ex - 46:
        return F(nav_exit=1, nav_exit_distance=ex - x, nav_curve_radius=85.0, nav_curve_distance=ex + 500 - x,
                 nav_exit_side=1, exit_shape=shape_fn(x), **hw)
      return F(exit_shape=shape_fn(x), **hw)
    return drive_exit(fields, 17000, 80, lane=lambda x: 1 if ex - 300 < x < ex + 300 else None, model=model,
                      signal=lambda x: 1 if ex - 280 <= x < ex - 260 else 0), ex
  # (the model path saying "highway" no longer cuts it short: logged only, MODEL_DECIDES; the shapes' "link" 150-600 m
  # without a gap confirms, the "highway" after it overturns, his speed back)
  for hi in (150.0, 300.0, 600.0):
    r, ex = run(lambda x, hi=hi: 2 if 3000 < x < 3000 + hi else (1 if x >= 3000 + hi else 0), model=lambda x: 0.0)
    assert r["ctl"].last_plan.why == "overturned: still on the highway" and abs(r["sets"][-1] - 80) < 1e-6, hi
  for model in (None, lambda x: 0.0):
    r, ex = run(lambda x: (2 if ((x - 3000) % 40.0) < 34.0 else 0) if 3000 < x < 3600 else (1 if x >= 3600 else 0), model)
    v0 = _at(r, ex)
    tent = [xx for st, xx in zip(r["state"], r["xs"], strict=True) if st == "tentative"]
    low = min(_past(r, ex))
    assert low >= v0 - CTL.TENT_DIP_MAX / MPH - 2.5, (model, v0, low)      # (the speed's cap, the cruise's lag after it)
    p = r["ctl"].last_plan
    assert "confirmed" not in r["state"] and p.why == "missed", p.why
    # (tentative: the plan's state past the split until the missed exit, its slowing TENT_HARD_MAX at most)
    assert tent and p.tent_s <= (tent[-1] - ex) / 20.0 + 1.0


# ---------------- re-review of d4f848215 ----------------

def _once(events):
  """change(x): the camera's lane change at each (x, dir), one frame each"""
  done = set()

  def change(x):
    for k, (at, d) in enumerate(events):
      if x >= at and k not in done:
        done.add(k)
        return d
    return 0
  return change


def test_rereview_a_lane_change_away_far_out_only_ends_the_slowing():
  # n1.py N1: his signal far out, a lane change away 500 m before the exit while slowing, back into the exit lane at
  # 350 m (no new signal): was declined for good, E1 70.9 mph into its 32 mph curve, E2 55.3 into 31
  fields, ex, cx = _e2()
  lane = lambda x: 1 if (700 <= x < ex - 500 or x >= ex - 310) else None  # noqa: E731
  for back_sig, top in ((False, 11.0), (True, 1.5)):
    sig = (lambda x, b=back_sig: 1 if (900 <= x < 960) or (b and 1240 <= x < 1260) else 0)  # noqa: E731
    r = drive_exit(fields, 9000, 75, lane=lane, signal=sig, change=_once([(ex - 500, -1), (ex - 350, 1)]))
    p = r["ctl"].plan or r["ctl"].last_plan
    assert not p.declined and _at(r, cx) <= V_C85 + top, (back_sig, _at(r, cx))
    assert _plus(r, ex - 500, ex - 450), "the slowing ended at the lane change (restored)"
  f1, ex1, cx1, lead1, _sig1 = _e1()
  lane1 = lambda x: 1 if (x < ex1 - 450 or x >= ex1 - 280) else None  # noqa: E731
  r = drive_exit(f1, 6000, 75, v_mph=57, lead=lead1, lane=lane1, signal=lambda x: 1 if ex1 - 700 <= x < ex1 - 650 else 0,
                 change=_once([(ex1 - 450, -1), (ex1 - 320, 1)]))
  assert _at(r, cx1) <= V_C95 + 5.0, _at(r, cx1)
  # within DECIDE_NEAR it is still his decision (declined, test_his_gas_or_the_other_signal_ends_the_exit_slowing_at_once)


def _false_link(shape_fn, gas=None, signal=None, model=None):
  """n3.py: he stays on (set 80, the camera's edge lane near the split), the shapes say what shape_fn gives"""
  ex = 3000.0

  def fields(x):
    hw = dict(road_class=1, limit=70 * MPH)
    if x < ex - 2500:
      return F(**hw)
    if x < ex - 46:
      return F(nav_exit=1, nav_exit_distance=ex - x, nav_curve_radius=85.0, nav_curve_distance=ex + 500 - x,
               nav_exit_side=1, exit_shape=shape_fn(x), **hw)
    return F(exit_shape=shape_fn(x), **hw)
  return drive_exit(fields, 17000, 80, lane=lambda x: 1 if ex - 300 < x < ex + 300 else None, gas=gas, signal=signal,
                    model=model), ex


def test_rereview_his_gas_held_past_the_split_is_his_decision_even_after_a_link():
  # a false "link" for 200 m right after the split (confirmed by the shapes alone), then silence: was the full ramp
  # slowdown on the highway to ~30 mph, his gas ignored. Held 1.5 s (or speeding the car up 3 mph): overturned
  link_then_none = lambda x: 2 if 3000 < x < 3200 else 0  # noqa: E731
  r, ex = _false_link(link_then_none, gas=lambda x: 36.0 if 3300 <= x < 3360 else None)
  assert r["ctl"].last_plan.why == "overturned: gas", r["ctl"].last_plan.why
  assert _at(r, ex + 500, "sets") >= 79.0 and abs(r["sets"][-1] - 80) < 1e-6, _at(r, ex + 500, "sets")
  # before a confirmation (alternating "link" / nothing): declined, his speed kept, never adopted low
  alt = lambda x: (2 if ((x - 3000) % 40.0) < 34.0 else 0) if 3000 < x < 3600 else 0  # noqa: E731
  r, ex = _false_link(alt, gas=lambda x: 36.0 if 3300 <= x < 3700 else None)
  assert r["ctl"].last_plan.declined and abs(r["sets"][-1] - 80) < 1e-6, (r["ctl"].last_plan.why, r["sets"][-1])
  # a tap (3 m) on the real ramp stays ignored (test_final_review_on_the_real_ramp_gas_..., E1 / E2)


def test_rereview_the_800_m_highway_match_needs_gps_trusted_and_no_link():
  # n2.py N6: he took the ramp, GPS paused to +790, then a doubtful (trust 2) match to the highway beside it, or the
  # shapes' last answer "link": was taken as the highway, set 54 -> 75 on the ramp. Adopted instead (no taps up)
  ex = 3000.0
  for trust, last in ((2, 1), (0, 2)):
    def fields(x, trust=trust, last=last):
      paused = ex - 50 <= x < ex + 790
      hw = dict(road_class=1, limit=70 * MPH, gps_trust=3 if paused else (trust if x >= ex + 790 else 0))
      if x < ex - 2500:
        return F(**hw)
      if x < ex - 46:
        return F(nav_exit=1, nav_exit_distance=ex - x, nav_curve_radius=85.0, nav_curve_distance=ex + 500 - x,
                 nav_exit_side=1, exit_shape=1, **hw)
      if paused:
        return F(exit_shape=0, **hw)
      return F(exit_shape=last if x < ex + 795 else 0, **hw)
    r = drive_exit(fields, 17000, 75, lane=lambda x: 1 if ex - 400 < x < ex + 300 else None,
                   signal=lambda x: 1 if ex - 700 < x < ex - 650 else 0)
    p = r["ctl"].last_plan
    assert p.why == "not confirmed", (trust, last, p.why)
    assert _at(r, ex + 1000, "sets") <= 62 and r["sets"][-1] <= 62, (trust, last, r["sets"][-1])


# ---------------- outside review of e1be62454 ----------------

def _ramp_heading(div=5.0, taper=40.0, bend_at=None, bend_deg=0.0, bend_len=60.0, curve_at=270.0, curve_r=95.0):
  """sim_1a.py: the exit lane's heading (deg, right positive, from the highway) s m past the split"""
  def th(s):
    h = 0.0 if s <= 0 else div * min(s, taper) / taper
    if bend_at is not None and s > bend_at:
      h += bend_deg * min(s - bend_at, bend_len) / bend_len
    if s > curve_at:
      h += math.degrees(min(s - curve_at, 150.0) / curve_r)
    return h
  return th


def _live_model(th, s_car):
  """sim_1a.py: model_path_heading as the car would compute it, the path and our two lines along the lane in the car's
  frame (modelV2: x forward, y right)"""
  pts, x, y, s = [], 0.0, 0.0, s_car - 5.0
  while s <= s_car + 260.0:
    pts.append((s, x, y, th(s)))
    a = math.radians(th(s))
    x, y, s = x + math.sin(a), y + math.cos(a), s + 1.0
  _, cx, cy, ch = min(pts, key=lambda p: abs(p[0] - s_car))
  a = math.radians(ch)

  def car(px, py):
    dx, dy = px - cx, py - cy
    return dx * math.sin(a) + dy * math.cos(a), dx * math.cos(a) - dy * math.sin(a)
  ahead = [p for p in pts if p[0] >= s_car]
  path = [car(p[1], p[2]) for p in ahead]
  lines = []
  for off in (-1.8, 1.8):
    ln = [car(px + off * math.cos(math.radians(h)), py - off * math.sin(math.radians(h))) for _, px, py, h in ahead]
    lines.append(([q[0] for q in ln], [q[1] for q in ln]))
  return CTL.model_path_heading([q[0] for q in path], [q[1] for q in path], lines, CTL.MODEL_LEFT)


def test_outside_review_the_model_path_on_a_straight_ramp_is_logged_only():
  # sim_1a.py: on a ramp straight for its first 270 m (E1) the live model path reads 0 against our own lane, the map's
  # link 5 deg off the highway: "highway" on the ramp. It decided: E1 confirmed only 213 m past the split, 44 mph into
  # its 32.3 mph curve; E2 never confirmed. Logged only now (MODEL_DECIDES): E1 ~32, E2 ~30
  th1 = _ramp_heading()
  assert abs(_live_model(th1, 150.0)) < 0.5
  mp = CTL.ModelPath()
  mp.update(_live_model(th1, 150.0))
  assert mp.verdict(F(exit_bend_valid=True, exit_hwy_bend=0.0, exit_link_bend=th1(230.0))) == 1   # (the wrong frame)
  assert CTL.MODEL_DECIDES is False
  for name, th, mk, kw, vc, lo in (
      ("E1", th1, _e1, None, V_C95, 300),
      ("E2", _ramp_heading(bend_at=80, bend_deg=10, curve_at=496, curve_r=85), _e2, None, V_C85, 560)):
    out = mk()
    fields0, ex, cx = out[0], out[1], out[2]

    def fields(x, f0=fields0, ex=ex, th=th, lo=lo):
      f = f0(x)
      if -300 <= x - ex <= lo:
        f.exit_bend_valid, f.exit_hwy_bend, f.exit_link_bend = True, 0.0, th(max(x - ex, 0.0) + 80.0)
      return f
    cache = {}

    def model(x, ex=ex, th=th, cache=cache):
      k = round(x)
      if k not in cache:
        cache[k] = _live_model(th, x - ex) if x - ex >= -300 else 0.0
      return cache[k]
    kw = dict(v_mph=57, lead=out[3], signal=out[4]) if name == "E1" else dict(lane=lambda x: 1 if x >= 1200 else None,
                                                                             signal=E2_SIGNAL)
    r = drive_exit(fields, 6000 if name == "E1" else 9000, 75, model=model, **kw)
    conf = next((xx - ex for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed"), None)
    assert conf is not None and conf <= 170, (name, conf)
    assert _at(r, cx) <= vc + 1.5, (name, _at(r, cx))


def _street_exit(ramp_to_mph, curve=True, start=300.0, curve_in=330.0, r=95.0, clen=80.0, ramp_len=630.0):
  """sim_1c.py: an exit ramp onto a street (no navigation), a ramp curve on it, the street's limit ramp_to_mph (0: the
  map does not know it)"""
  c0, c1, end = start + curve_in, start + curve_in + clen, start + ramp_len

  def fields(x):
    if x < start:
      return F(road_class=1, limit=70 * MPH)
    if x < end:
      ramp = dict(on_ramp=True, ramp_type=1, ramp_to_highway=False, ramp_to_speed=ramp_to_mph * MPH,
                  ramp_end_distance=end - x, road_class=2)
      if curve and x < c1:
        return F(status=2, radius=r, distance=max(c0 - x, 0.0), **ramp)
      return F(**ramp)
    return F(road_class=5)
  return fields, c1, end


def test_outside_review_an_exit_onto_a_street_without_its_limit_never_restores_on_the_ramp():
  # sim_1c.py: after the ramp curve 53 SET+ back toward his 75, 51.8 mph near the ramp's end
  fields, c1, end = _street_exit(0.0)
  r = drive_exit(fields, 6000, 75, v_mph=70)
  assert _plus(r, c1, end) == [], _plus(r, c1, end)
  assert _at(r, end - 20) <= 36.0 and _at(r, end + 300, "sets") <= 34.0, (_at(r, end - 20), _at(r, end + 300, "sets"))
  # no curve on it: at most STREET_DEFAULT (45 mph) near its end, his lower speed kept
  fields, c1, end = _street_exit(0.0, curve=False)
  r = drive_exit(fields, 6000, 75, v_mph=70)
  # (1 mph taps from 75 over the last 200 m, as the known limit's street cap; then kept as his speed off the ramp)
  assert _minus(r, end - CTL.STREET_CAP_DISTANCE - 30, end) and _at(r, end, "sets") <= 47.0, _at(r, end, "sets")
  assert _at(r, end + 300, "sets") <= 47.0 and _plus(r, end - CTL.STREET_CAP_DISTANCE, end + 300) == []
  assert _minus(r, 0, end - CTL.STREET_CAP_DISTANCE - 30) == []
  r = drive_exit(fields, 6000, 40)
  assert _minus(r, 0, end + 100) == [] and _plus(r, 0, end + 100) == []
  # metric: 70 km/h
  r = drive_exit(fields, 6000, 76, v_mph=74, metric=True)
  assert _minus(r, 0, end) and _at(r, end, "sets") <= 70 + 2.0, _at(r, end, "sets")          # (the end gap)


# ---------------- drives 54 / 55 (owner's tags; build 0c1b66da3) ----------------

def test_drives_54_55_planner_past_the_link_start_heading_along_the_highway_is_the_highway():
  # drive 55, exit 1 (missed, right lane): 1-6 m from the link's first shape, 7-8 m from the highway's line, heading the
  # highway's (0.1-0.2 deg off) and 5 deg off the link's: no answer for 5.5 s (the tentative slowdown ran on, 55 mph in
  # 67 mph traffic). Now "highway" (SHAPE_HWY_HEADING_GAP); before the link's start, or turned toward the link, not.
  assert MC.shape_verdict((3.0, 5.0), (7.5, 0.2), MC.SHAPE_LINK_GAP) == 0
  hw = (MC.SHAPE_HWY_HEADING_GAP, MC.SHAPE_HWY_NEAR, MC.SHAPE_HWY_HEADING_MAX)
  assert MC.shape_verdict((3.0, 5.0), (7.5, 0.2), MC.SHAPE_LINK_GAP, hw) == -1
  # drive 53, E2 (taken), 26-54 m on: 2.3-2.9 deg closer to the highway's, the highway's 1.3-1.6 deg off ours: not
  assert MC.shape_verdict((9.3, 4.2), (11.6, 1.3), MC.SHAPE_LINK_GAP, hw) == 0
  assert MC.shape_verdict((6.8, 3.9), (12.0, 1.6), MC.SHAPE_LINK_GAP, hw) == 0
  # the highway's shape farther than SHAPE_HWY_NEAR: not
  assert MC.shape_verdict((3.0, 5.0), (13.0, 0.2), MC.SHAPE_LINK_GAP, hw) == 0
  # in the planner (the motorway with its 8 deg exit): 5 m right of its line, 30 m past the link's start, heading north
  root, _, _ = write_files(fork_scene(fork=False))
  car = _Car(root)
  _announce(car)
  m = car.at(5.0, 30, 0.0, frames=5)
  assert m.exit_shape == MC.EXIT_SHAPE_HIGHWAY and m.exit_start < 0, (m.exit_shape, m.exit_link, m.exit_hwy)
  # the same place 30 m before the link's start: no answer from the heading alone there
  m = car.at(5.0, -30, 0.0, frames=5)
  assert m.exit_start > 0 and m.exit_shape != MC.EXIT_SHAPE_LINK
  # on the link, heading along it: still the link's
  x, y, h = _along(_ramp_pts(8, [("s", 200)]), 60)
  assert car.at(x + 2.5, y, h, frames=5).exit_shape == MC.EXIT_SHAPE_LINK


def _no_curve_exit(street_mph=45.0, exit_x=1500.0, link_len=450.0, shapes_link_from=100.0, street_v=40.0):
  """Drive 55, exit 2 (tag 552.3): navigation's exit with no curves (mapNavExit 2), his signal 300 m before it, GPS trust
  2 from the split along the whole link (so the planner never says "on a ramp": no street rules), the shapes "link" from
  100 m past it, then a street (class 7, trust 0, its limit street_mph, 0: unknown); traffic takes the car to street_v
  on the street (the v_ego cap)."""
  def fields(x):
    hwy = dict(road_class=1, limit=70 * MPH)
    if x < exit_x - 1300:
      return F(**hwy)
    if x < exit_x - 30:
      return F(nav_exit=2, nav_exit_distance=exit_x - x, nav_exit_side=1, exit_shape=1, **hwy)
    if x < exit_x + 15:
      return F(exit_shape=0, **{**hwy, "gps_trust": 2})
    if x < exit_x + link_len:
      return F(road_class=2, gps_trust=2, exit_shape=2 if x > exit_x + shapes_link_from else 0)
    return F(road_class=7, gps_trust=0, limit=street_mph * MPH)
  signal = lambda x: 1 if exit_x - 300 <= x < exit_x - 280 else 0  # noqa: E731
  lead = lambda x: street_v * MPH if x > exit_x + link_len - 150 else None  # noqa: E731
  return fields, exit_x, exit_x + link_len, signal, lead


def test_drive_55_exit_2_no_curve_exit_holds_on_the_ramp_then_the_streets_limit_plus_the_setting():
  fields, exit_x, street_x, signal, lead = _no_curve_exit()
  r = drive_exit(fields, 9000, 75, signal=signal, lead=lead)
  # before the split: the highway's 70 (his signal); on the link (no curve): held, never back up toward his 75 (was:
  # the plan ended on the link match, 64 -> 75 on the ramp)
  assert _plus(r, exit_x - 300, street_x) == [], _plus(r, exit_x - 300, street_x)
  assert max(s for s, xx in zip(r["sets"], r["xs"], strict=True) if exit_x <= xx < street_x) <= 70 + 1e-6
  assert r["ctl"].last_plan is not None and r["ctl"].last_plan.why in ("on the ramp, no curve", "on a street")
  # the street (45): the exit's last target 45 + 5 (the setting's default) by our SET- taps, then his speed
  assert _minus(r, street_x, street_x + 400) and all(p[3] == "tap" for p in _minus(r, street_x, street_x + 400))
  assert abs(r["sets"][-1] - 50) < 1e-6 and r["logic"].anchor is None, (r["sets"][-1], r["logic"].anchor)
  # the map without the street's limit: STREET_FALLBACK (43 mph, no offset)
  fields, exit_x, street_x, signal, lead = _no_curve_exit(street_mph=0.0)
  r = drive_exit(fields, 9000, 75, signal=signal, lead=lead)
  assert abs(r["sets"][-1] - 43) < 1e-6 and r["logic"].anchor is None, r["sets"][-1]
  # still at highway speed on the "street" (a wrong match beside the highway): nothing (EXIT_STREET_MAX_V)
  fields, exit_x, street_x, signal, lead = _no_curve_exit(street_v=65.0)
  r = drive_exit(fields, 9000, 75, signal=signal, lead=lead)
  assert _minus(r, street_x, street_x + 600) == [] and r["sets"][-1] >= 65 - 1e-6, r["sets"][-1]


def test_exit_street_target_res_gas_and_metric():
  """The exit's street target directly (MapCurveControl): his RES within EXIT_STREET_TIME brings back the highway's 75
  (drive 55, t 581: on a 45 street) -> 50; his gas ends it (what it reached is his); metric."""
  def street_run(c, frames, set_v, driver, v=10.0, gas=lambda k: False, limit=45 * MPH, metric_unit=MPH, res=False):
    outs = []
    for k in range(frames):
      out = c.update(F(road_class=7, limit=limit), driver if driver else 0.0, 0, v_ego=v, gas=gas(k),
                     set_speed=set_v if driver else None)
      outs.append(out)
      if out.active and out.speed < set_v - 0.1 and k % 30 == 0:
        set_v -= metric_unit                                   # (a tap every 0.3 s)
    return outs, set_v

  def exited(c, metric_unit=MPH):
    for _ in range(100):
      c.update(F(road_class=1, limit=70 * MPH), 75 * metric_unit / MPH, 0, v_ego=30.0)
    for _ in range(300):
      c.update(F(road_class=2, gps_trust=2), 75 * metric_unit / MPH, 0, v_ego=25.0)
    for _ in range(300):                                       # his brake: cruise off from the ramp on
      c.update(F(road_class=7, limit=45 * MPH), 0.0, 0, v_ego=10.0)
  c = CTL.MapCurveControl(3.0)
  exited(c)
  assert c.xs_since is not None
  outs, set_v = street_run(c, 1500, 75 * MPH, 75 * MPH)        # RES at 75
  assert outs[0].active and abs(outs[0].speed - 50 * MPH) < 1e-6 and outs[0].why == "exit street"
  assert abs(set_v - 50 * MPH) < 1e-6 and any(o.adopt for o in outs)
  # his gas while it lowers: it ends there, adopted
  c = CTL.MapCurveControl(3.0)
  exited(c)
  outs, set_v = street_run(c, 1500, 75 * MPH, 75 * MPH, gas=lambda k: k >= 150)
  assert set_v > 60 * MPH and any(o.adopt for o in outs) and not outs[-1].active
  # RES later than EXIT_STREET_TIME: still (review of merge-f: the window never ends before the exit's street speed
  # was applied, so his highway speed is never left the target on the street)
  c = CTL.MapCurveControl(3.0)
  exited(c)
  for _ in range(int(CTL.EXIT_STREET_TIME / CTL.DT_CTRL) + 10):
    c.update(F(road_class=7, limit=45 * MPH), 0.0, 0, v_ego=10.0)
  outs, set_v = street_run(c, 1500, 75 * MPH, 75 * MPH)
  assert outs[0].active and abs(set_v - 50 * MPH) < 1e-6
  # his own lower speed at RES (40 on the 45 street): his
  c = CTL.MapCurveControl(3.0)
  exited(c)
  outs, set_v = street_run(c, 200, 40 * MPH, 40 * MPH)
  assert not any(o.active for o in outs) and set_v == 40 * MPH
  # metric (+8 km/h the default): a 50 km/h street -> 58
  c = CTL.MapCurveControl(3.0, metric=True)
  exited(c, KPH)
  outs, set_v = street_run(c, 3000, 120 * KPH, 120 * KPH, limit=50 * KPH, metric_unit=KPH)
  first = next(o for o in outs if o.active)                    # (once the 50 is steady: his RES is kept until then)
  assert abs(first.speed - 58 * KPH) < 1e-6 and abs(set_v - 58 * KPH) < 1e-6
  # the setting's options
  for m, add in ((0, 0), (1, 3), (2, 5), (3, 8), (7, 5)):
    assert abs(CTL.MapCurveControl(3.0, street_mode=m).street_add - add * MPH) < 1e-9, m
  for m, add in ((0, 0), (1, 5), (2, 8), (3, 13)):
    assert abs(CTL.MapCurveControl(3.0, street_mode=m, metric=True).street_add - add * KPH) < 1e-9, m


def test_drive_55_exit_1_the_camera_alone_losing_its_edge_lane_ends_the_slowing():
  # in the right lane (the camera's edge lane) toward a short exit he does not take; 230 m before it the exit lane opens
  # beside him (no edge lane any more); the shapes say "highway" only 186 m past it (drive 55). Was: held at the split
  # speed for 8 s, then the tentative slowdown past the split (67 -> 55.6 mph).
  ex = 3000.0

  def fields(x):
    hw = dict(road_class=1, limit=70 * MPH)
    sh = 1 if x > ex + 186 else 0
    if x < ex - 2500:
      return F(**hw)
    if x < ex - 46:
      return F(nav_exit=1, nav_exit_distance=ex - x, nav_curve_radius=95.0, nav_curve_distance=ex + 305 - x,
               nav_exit_side=1, exit_shape=1, **hw)
    return F(exit_shape=sh, **hw)
  r = drive_exit(fields, 16000, 75, lane=lambda x: 1 if ex - 2000 < x < ex - 230 else None)
  assert _minus(r, ex - CTL.DECIDE_NEAR, ex - 225), "the camera's slowdown from DECIDE_NEAR, as before"
  assert _minus(r, ex - 220, ex + 400) == [], _minus(r, ex - 220, ex + 400)
  low = min(v for v, xx in zip(r["vs"], r["xs"], strict=True) if ex - 220 <= xx <= ex + 400)
  assert low >= 67.0 and r["ctl"].last_plan.why == "missed" and not r["ctl"].last_plan.tentative, low
  assert r["chimes"] == [], r["chimes"]                          # (no chime on the camera alone)
  # his signal toward it (the same lanes): the slowdown holds, then the tentative one from the split (unchanged)
  r = drive_exit(fields, 16000, 75, lane=lambda x: 1 if ex - 2000 < x < ex - 230 else None,
                 signal=lambda x: 1 if ex - 260 <= x < ex - 240 else 0)
  assert r["ctl"].last_plan.tentative and _minus(r, ex - 260, ex)


def test_drive_54_exit_2_no_set_plus_from_the_exit_wait_on_the_real_ramp():
  # his signal, the split; on the link (class 2) the shapes silent for 250 m (drive 54: 5.6 s), the 95 m curve 600 m on:
  # the tentative slowdown's end (TENT_MAX_QUIET) gave a SET+ 55 -> 57 on the ramp; now held until the confirmation
  ex = 1500.0

  def fields(x):
    hw = dict(road_class=1, limit=70 * MPH)
    ec = dict(exit_curve_radius=95.0, exit_curve_distance=max(ex + 600 - x, 0.0))
    if x < ex - 1300:
      return F(**hw)
    if x < ex - 30:
      return F(nav_exit=1, nav_exit_distance=ex - x, nav_exit_side=1, nav_curve_radius=95.0, nav_curve_distance=ex + 600 - x,
               exit_shape=1, **ec, **hw)
    if x < ex + 20:
      return F(exit_shape=0, **ec, **{**hw, "gps_trust": 2})
    return F(road_class=2, gps_trust=2, exit_shape=2 if x > ex + 250 else 0, **ec)
  r = drive_exit(fields, 9000, 70, signal=lambda x: 1 if ex - 350 <= x < ex - 330 else 0)
  conf = next(xx for s, xx in zip(r["state"], r["xs"], strict=True) if s == "confirmed")
  assert conf > ex + 250 and _plus(r, ex, conf) == [], (conf, _plus(r, ex, conf))


def test_review_an_exit_without_curves_confirmed_but_never_matched_to_its_link_ends():
  # review of merge-f: confirmed by the shapes, the road match never on the link (still class 1, trust 2): the plan held
  # "exit hold" for good (no caps, no raises). EXIT_PASS_MARGIN past the split: what it reached is his
  ex = 1500.0

  def fields(x):
    hwy = dict(road_class=1, limit=70 * MPH)
    if x < ex - 1300:
      return F(**hwy)
    if x < ex - 30:
      return F(nav_exit=2, nav_exit_distance=ex - x, nav_exit_side=1, exit_shape=1, **hwy)
    return F(exit_shape=2 if x > ex + 60 else 0, **{**hwy, "gps_trust": 2})
  r = drive_exit(fields, 9000, 75, signal=lambda x: 1 if ex - 300 <= x < ex - 280 else 0,
                 lead=lambda x: 50 * MPH if x > ex + 100 else None)
  assert r["ctl"].plan is None and r["ctl"].last_plan.why == "confirmed, no link match", r["ctl"].last_plan.why
  assert r["logic"].anchor is None and r["sets"][-1] <= 70 + 1e-6 and _plus(r, ex, 10 ** 9) == [], r["sets"][-1]


def test_planner_navigations_limit_for_the_street_speed():
  # the car side's second opinion (mapNavSpeedLimit): Mapbox's limit fresh (NAV_FRESH), junk under 15 mph none
  from types import SimpleNamespace
  p = _FakePlanner(object())
  ni = lambda lim, d=200.0, t="turn", m="right": SimpleNamespace(speedLimit=lim * MPH, maneuverDistance=d,  # noqa: E731
                                                                 maneuverType=t, maneuverModifier=m)
  for lim, want in ((25, 25), (5, 0), (0, 0), (50, 50)):
    MC.MapCurvesPlanner._nav_street(p, ni(lim), 100.0)
    p.now = 101.0
    plan = _plan_ns(mapNavSpeedLimit=0.0, mapNavManeuverDistance=0.0, mapNavManeuverTurn=False)
    p.mc = SimpleNamespace(data_here=False, paused=False, curve=None, held=None)
    try:
      MC.MapCurvesPlanner.publish(p, plan)
    except AttributeError:
      pass                                                       # (the rest of the fake planner: not needed here)
    assert abs(plan.mapNavSpeedLimit - want * MPH) < 1e-6 and plan.mapNavManeuverTurn and plan.mapNavManeuverDistance == 200.0
  # stale (more than NAV_FRESH): none
  p.now = 100.0 + MC.NAV_FRESH + 0.5
  plan = _plan_ns(mapNavSpeedLimit=0.0, mapNavManeuverDistance=0.0, mapNavManeuverTurn=False)
  try:
    MC.MapCurvesPlanner.publish(p, plan)
  except AttributeError:
    pass
  assert plan.mapNavSpeedLimit == 0.0
  # straight on is no turn
  MC.MapCurvesPlanner._nav_street(p, ni(45, t="continue", m="straight"), 100.0)
  assert not p.nav_street[2]
