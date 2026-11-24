"""Round W: camera CSC v_ego guard, motorway short-kink filter, and Half Day Rd carriageway-split jogs."""
import importlib.util
import math
import os

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


CT = _load("ct_round_w", os.path.join(HERE, "..", "curve_target.py"))
CTL = _load("ctl_round_w", os.path.join(HERE, "..", "map_curve_control.py"))
TM = _load("tmc_round_w", os.path.join(HERE, "test_map_curves.py"))
MC, G, B = TM.MC, TM.G, TM.B
MPH = CT.MS_PER_MPH


def _set_node(w, i, other, j):
  w.lat[i], w.lon[i] = other.lat[j], other.lon[j]
  w.start, w.end = B.key(w.lat[0], w.lon[0]), B.key(w.lat[-1], w.lon[-1])


# ---- W1: camera CSC ignores targets far above v_ego (crawling in traffic with TI_ONLY_LAT) ----

def test_w1_camera_csc_ignores_targets_far_above_v_ego():
  t = CT.CurveTarget(3.0)
  t.ti_lat = CTL.TI_ONLY_LAT
  t.ti_only = True
  k_65 = CTL.TI_ONLY_LAT / (65.0 * MPH) ** 2
  k_40 = CTL.TI_ONLY_LAT / (40.0 * MPH) ** 2
  k_22 = CTL.TI_ONLY_LAT / (22.0 * MPH) ** 2

  for _ in range(150):
    act, spd = t.update(k_65, 85.0 * MPH, v_ego=25.0 * MPH)
  assert not act and abs(spd / MPH - 65.0) < 0.05

  t.reset()
  for _ in range(150):
    act, spd = t.update(k_40, 85.0 * MPH, v_ego=25.0 * MPH)
  assert not act and abs(spd / MPH - 40.0) < 0.05

  # a real tight curve below or near v_ego still acts normally
  t.reset()
  for _ in range(150):
    act, spd = t.update(k_22, 85.0 * MPH, v_ego=25.0 * MPH)
  assert act and abs(spd / MPH - 22.0) < 0.05


def test_w1_camera_csc_normal_cruising_and_in_curve_unchanged():
  k_72 = 3.0 / (72.0 * MPH) ** 2
  t_none = CT.CurveTarget(3.0)
  t_v = CT.CurveTarget(3.0)
  for _ in range(150):
    a1, s1 = t_none.update(k_72, 80.0 * MPH)
    a2, s2 = t_v.update(k_72, 80.0 * MPH, v_ego=80.0 * MPH)
  assert a1 and a2 and abs(s1 - s2) < 1e-9

  # enter a 45 mph curve at 70 mph and decelerate through it: stays active down through 40 mph,
  # releasing only if traffic slows the car far below it (>= 10 mph below curve speed)
  k_45 = 3.0 / (45.0 * MPH) ** 2
  t = CT.CurveTarget(3.0)
  for v_mph in (70.0, 50.0, 45.0, 40.0):
    for _ in range(120):
      act, _ = t.update(k_45, 70.0 * MPH, v_ego=v_mph * MPH)
    assert act, v_mph
  for _ in range(120):
    act, _ = t.update(k_45, 70.0 * MPH, v_ego=35.0 * MPH)
  assert not act


def test_w1_camera_csc_v_ego_hysteresis():
  k_50 = 3.0 / (50.0 * MPH) ** 2
  t = CT.CurveTarget(3.0)
  # v_ego = 41.5 -> ref = 51.5; needs <= 49.5 (START_GAP=2) to activate -> False
  for _ in range(150):
    assert not t.update(k_50, 80.0 * MPH, v_ego=41.5 * MPH)[0]
  # v_ego = 42.5 -> ref = 52.5; 50 <= 50.5 -> activates
  for _ in range(10):
    act, _ = t.update(k_50, 80.0 * MPH, v_ego=42.5 * MPH)
  assert act
  # drops to 41.5 -> ref = 51.5; stays active while 50 < 50.5 (END_GAP=1)
  assert t.update(k_50, 80.0 * MPH, v_ego=41.5 * MPH)[0]
  # drops to 40.5 -> ref = 50.5; 50 >= 49.5 -> releases
  assert not t.update(k_50, 80.0 * MPH, v_ego=40.5 * MPH)[0]


# ---- W2: read-time motorway short-kink filter (Tri-State / I-96 / Kingery / I-80 / M-14) ----

def _kink_motorway(hdg=13.0, cls=1, adv=0.0, speed_mph=70):
  pts = G.straight(0, -400, 0, 400)[:-1] + G.straight(0, 0, hdg, 400)
  return {10: G.way(10, pts, name="Tri-State Tollway", cls=cls, speed_mph=speed_mph, adv=adv, oneway=True)}


def test_w2_motorway_short_kink_skipped_at_read_time():
  assert MC.motorway_kink({"cls": 1, "len": 44.0, "turn": 12.1, "adv": 0.0})
  assert MC.motorway_kink({"cls": 1, "len": 30.0, "turn": 15.0, "adv": 0.0})

  # Find a short shallow motorway kink produced by build_curve_map on a cls=1 way and verify MapCurves skips it
  ways = _kink_motorway()
  root, curves, _ = TM.write_files(ways)
  assert curves, "expected synthetic motorway kink to be emitted by builder"
  assert all(MC.motorway_kink(c) for c in curves), curves
  m = TM.planner(root)
  la, lo = TM.pos(0, -150)
  m.update(la, lo, 0.0, 10, True)
  assert m.curve is None and m.status == MC.STATUS_NONE


def test_w2_real_motorway_ramps_streets_and_advisory_curves_kept():
  assert not MC.motorway_kink({"cls": 1, "len": 190.0, "turn": 30.0, "adv": 0.0})
  assert not MC.motorway_kink({"cls": 1, "len": 30.0, "turn": 21.7, "adv": 0.0})
  assert not MC.motorway_kink({"cls": 1, "len": 44.0, "turn": 12.1, "adv": 22.35})
  assert not MC.motorway_kink({"cls": 2, "ramp": {"type": "exit"}, "len": 30.0, "turn": 15.0, "adv": 0.0})
  assert not MC.motorway_kink({"cls": 3, "len": 30.0, "turn": 15.0, "adv": 0.0})
  assert not MC.motorway_kink({"cls": 7, "len": 30.0, "turn": 15.0, "adv": 0.0})

  # Real 265 m / 30 deg motorway curve (Tri-State 19:13:50) stays
  root, curves, _ = TM.write_files(G.curve_road(r=265.0, turn=30.0, name="Tri-State Tollway", cls=1, speed=70))
  assert len(curves) == 1 and not MC.motorway_kink(curves[0])
  m = TM.planner(root)
  la, lo = TM.pos(0, -150)
  m.update(la, lo, 0.0, 1, True)
  assert m.curve is not None and m.status == MC.STATUS_CURVE

  # Advisory-signed motorway curve stays even if short
  ways = _kink_motorway(adv=45.0 * B.MPH)
  root, curves, _ = TM.write_files(ways)
  assert curves and all(not MC.motorway_kink(c) for c in curves)
  m = TM.planner(root)
  la, lo = TM.pos(0, -150)
  m.update(la, lo, 0.0, 10, True)
  assert m.curve is not None


# ---- W3: carriageway-split jog with directional prefix & 35-45 deg bends (Half Day Rd) ----

def _half_day_scene(along=8.0, side=7.1, cls=5, oneway=True, name1="Half Day Road", name2="West Half Day Road", ref="IL 22"):
  w1 = G.way(70, G.straight(0, -400, 0, 400), name=name1, cls=cls, speed_mph=45, oneway=False)
  w1.ref = ref
  w2 = G.way(71, [(0, 0), (side, along)] + [(side, along + 20 * i) for i in range(1, 21)],
             name=name2, cls=cls, speed_mph=45, oneway=oneway)
  w2.ref = ref
  _set_node(w2, 0, w1, len(w1.lat) - 1)
  return {70: w1, 71: w2}


def test_w3_half_day_rd_directional_prefix_and_steep_jog_skipped():
  ways = _half_day_scene()
  root, curves, _ = TM.write_files(ways)
  assert len(curves) == 2 and all(35.0 < c["turn"] <= 45.0 for c in curves), curves
  m = TM.planner(root)
  la, lo = TM.pos(0, -150)
  m.update(la, lo, 0.0, 70, True)
  assert m.curve is None and m.status == MC.STATUS_NONE

  # Direct check on North/South prefix pair seen in corridor tiles ('South Hope Street' / 'North Hope Street')
  c1 = {"chain": 99, "s_a": 100.0, "s_b": 134.0, "dir": "R", "cls": 5, "turn": 22.2, "name": "South Hope Street",
        "ways": [1], "a": (42.0, -87.0), "b": (42.0003, -86.9999), "a_hdg": 0.0}
  c2 = {"chain": 99, "s_a": 140.0, "s_b": 188.0, "dir": "L", "cls": 5, "turn": 22.0, "name": "North Hope Street",
        "ways": [2], "a": (42.00035, -86.9999), "b": (42.0008, -86.9999), "a_hdg": 20.0}
  assert MC.jogs([c1, c2], {2}) == {(99, 100.0), (99, 140.0)}


def test_w3_different_names_two_way_wide_sbend_and_links_stay():
  # Truly different street names even sharing a route ref stay
  m = TM.planner(TM.write_files(_half_day_scene(name1="Half Day Road", name2="Milwaukee Avenue"))[0])
  la, lo = TM.pos(0, -150)
  m.update(la, lo, 0.0, 70, True)
  assert m.curve is not None

  # Both two-way stays
  m = TM.planner(TM.write_files(_half_day_scene(oneway=False))[0])
  m.update(la, lo, 0.0, 70, True)
  assert m.curve is not None

  # Sideways > 12 m real S-bend stays
  m = TM.planner(TM.write_files(_half_day_scene(along=16.0, side=14.0))[0])
  m.update(la, lo, 0.0, 70, True)
  assert m.curve is not None

  # Link class never dropped by jogs()
  c1 = {"chain": 88, "s_a": 100.0, "s_b": 130.0, "dir": "R", "cls": 6, "turn": 40.0, "name": "West Half Day Road",
        "ways": [1], "a": (42.0, -87.0), "b": (42.0003, -86.99995), "a_hdg": 0.0}
  c2 = {"chain": 88, "s_a": 134.0, "s_b": 162.0, "dir": "L", "cls": 6, "turn": 40.0, "name": "Half Day Road",
        "ways": [2], "a": (42.00034, -86.99995), "b": (42.0006, -86.9999), "a_hdg": 35.0}
  assert MC.jogs([c1, c2], {1}) == set()
