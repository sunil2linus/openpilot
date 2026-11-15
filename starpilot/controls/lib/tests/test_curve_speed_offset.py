"""Round S, Curve Speed Offset on the planner's side (map_curves.CURVE_OFFSET_*): the grant with its lateral cap, the "no
offset" marks of curves that proved too tight. Synthetic geometry only (no openpilot runtime needed)."""
import importlib.util
import json
import math
import os
import tempfile

_spec = importlib.util.spec_from_file_location("map_curves_round_s", os.path.join(os.path.dirname(__file__), "..", "map_curves.py"))
MC = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(MC)
MPH = MC.MPH


class P:
  def __init__(self, v=None):
    self.v = dict(v or {})

  def get(self, key, encoding=None):
    return self.v.get(key)

  def get_bool(self, key):
    return bool(self.v.get(key))


def _curve(r, ramp=False, adv=0.0, k=0):
  # a curve heading north-east (entry a, exit b), its apex at a synthetic origin
  c = {"apex": (10.0 + 0.01 * k, 10.0), "a": (9.999 + 0.01 * k, 10.0), "b": (10.001 + 0.01 * k, 10.001), "r": float(r),
       "s_a": float(k), "len": 100.0, "turn": 60.0, "name": f"c{k}", "ways": [100 + k]}
  if ramp:
    c["ramp"] = True
  if adv:
    c["adv"] = adv
  return c


def _mc(tmp, setting=4, metric=False):
  m = MC.MapCurves(MC.MapCurveStore(tempfile.mkdtemp(), background=False), memory=MC.CurveMemory(os.path.join(tmp, "m.json")))
  m.increasing = True
  m.offset_set = MC.curve_offset_setting(setting, metric)
  return m


def _passes(m, c, r, n=3):
  d = m.direction(c)
  for t in range(n):
    if c.get("ramp"):
      m.memory.add_ramp_pass(c, r, d, t=1000.0 * (t + 1), v=15.0, flags=["full"])
    else:
      m.memory.add_pass(c, r, t=1000.0 * (t + 1), v=15.0, flags=["full"], direction=d)


def test_the_setting_in_mph_and_km_h_steps_and_bad_values_are_off():
  assert [round(MC.curve_offset_setting(n) / MPH, 3) for n in range(6)] == [0, 1, 2, 3, 4, 5]
  assert [round(MC.curve_offset_setting(str(n), True) * 3.6, 3) for n in range(6)] == [0, 2, 3, 5, 6, 8]
  for bad in (None, "", "x", -1, 6, "9"):
    assert MC.curve_offset_setting(bad) == 0.0
  mp = MC.MapCurvesPlanner(P({"MazdaMapCurves": "2", "MazdaCurveSpeedOffset": "4"}), P())
  mp._read_mode()
  assert abs(mp.offset_set - 4 * MPH) < 1e-9
  mp = MC.MapCurvesPlanner(P({"MazdaMapCurves": "2"}), P())         # (an older params build without the key: off)
  mp._read_mode()
  assert mp.offset_set == 0.0


def test_a_road_curve_gets_the_offset_only_up_to_2p3_on_its_planning_radius(tmp_path):
  m = _mc(str(tmp_path))
  for r in (60, 90, 131, 142, 200, 400, 800):
    c = _curve(r, k=r)
    g, st, rc = m._offset(c)
    v = math.sqrt(MC.PLAN_LAT * r)
    assert st == MC.OFS_MAP and rc == r
    assert abs(g - min(4 * MPH, math.sqrt(MC.CURVE_OFFSET_LAT * r) - v)) < 1e-9, (r, g)
    assert (v + g) ** 2 / r <= MC.CURVE_OFFSET_LAT + 1e-9
  # (the cap leaves ~2.2 % of the curve's speed on its own planning radius: +1 mph at ~46 mph, the full +4 only past 180 mph)
  assert m._offset(_curve(200, k=1))[0] < 1.1 * MPH
  m.offset_set = MC.curve_offset_setting(1)
  assert m._offset(_curve(200, k=3))[0] == min(1 * MPH, math.sqrt(2.3 * 200) - math.sqrt(2.2 * 200))
  m.offset_set = MC.curve_offset_setting(4)
  # the memory's learned radius: the plan and its cap on it (two agreeing clean passes)
  c = _curve(131, k=2)
  _passes(m, c, 157.0)
  m._ofs = {}
  g, st, rc = m._offset(c)
  assert st == MC.OFS_MEMORY and abs(rc - 157.0) < 0.5 and abs((math.sqrt(2.2 * rc) + g) ** 2 / rc - 2.3) < 1e-6


def test_a_ramp_curve_gets_nothing_on_its_map_radius_and_its_measured_radius_after_clean_passes(tmp_path):
  m = _mc(str(tmp_path))
  for r in (60, 71, 100, 200):
    g, st, rc = m._offset(_curve(r, ramp=True, k=r))
    assert g == 0.0 and st == MC.OFS_CAP and rc == r             # (planned at 2.6 on it already: the cap leaves nothing)
  # exit 46's second curve (map 100, really ~116): the ramp memory plans it faster itself, the cap still leaves nothing
  c = _curve(100, ramp=True, k=1)
  _passes(m, c, 116.0)
  m._ofs = {}
  g, st, rc = m._offset(c)
  assert m.memory.ramp_radius(c, m.direction(c)) is not None and g == 0.0 and st == MC.OFS_CAP and abs(rc - 116) < 0.5
  # measured much gentler than the map (map 90, two clean passes at 135 m): the offset up to 2.3 on the measured radius
  c = _curve(90, ramp=True, k=2)
  _passes(m, c, 135.0)
  m._ofs = {}
  g, st, rc = m._offset(c)
  v = math.sqrt(MC.PLAN_LAT * m._radius(c))
  assert st == MC.OFS_MEMORY and rc == 135.0 and 2.0 * MPH < g < 4 * MPH and abs((v + g) ** 2 / 135.0 - 2.3) < 1e-6
  # one pass only: the map's radius
  c = _curve(90, ramp=True, k=3)
  _passes(m, c, 135.0, n=1)
  m._ofs = {}
  assert m._offset(c)[:2] == (0.0, MC.OFS_CAP)


def test_never_with_the_setting_0_untrusted_gps_unknown_direction_a_veto_a_ghost_mark_a_speed_cap_or_no_memory(tmp_path):
  c = _curve(400)
  assert _mc(str(tmp_path), setting=0)._offset(c) == (0.0, MC.OFS_OFF, 0.0)
  m = _mc(str(tmp_path))
  assert m._offset(c)[0] > 0
  for prep in ("trust", "dir", "veto", "ghost", "cap", "memory"):
    m = _mc(tempfile.mkdtemp())
    if prep == "trust":
      m.trust_status = MC.TRUST_SHAKY
    elif prep == "dir":
      m.increasing = None
    elif prep == "veto":
      m.veto.flips = lambda key: 1                # (taken back once this drive)
    elif prep == "ghost":
      m.memory.add_ghost(c, m.direction(c), way=100)
    elif prep == "cap":
      m.memory.add_saturated_pass(c, 18.0, direction=m.direction(c))
    elif prep == "memory":
      m.memory = None
    assert m._offset(c) == (0.0, MC.OFS_NOT, 0.0), prep


def test_never_above_the_yellow_sign_plus_10(tmp_path):
  m = _mc(str(tmp_path))
  adv = 25 * MPH
  c = _curve(400, adv=adv)                       # shape 66 mph, the sign 25: planned at 35, no room over it
  assert m._offset(c)[:2] == (0.0, MC.OFS_CAP)
  c = _curve(100, adv=adv, k=1)                  # shape 33.2 mph under the sign's 35: up to 35 at most
  g = m._offset(c)[0]
  assert 0 < g and math.sqrt(2.2 * 100) + g <= adv + MC.ADVISORY_OVER + 1e-9


def test_the_offset_changes_which_curve_governs_and_is_published_with_its_state(tmp_path):
  # two curves ahead: the one with the offset can need less than the one without it
  m = _mc(str(tmp_path))
  a, b = _curve(400, k=1), _curve(390, k=2)
  m.memory.add_no_offset(b, m.direction(b), way=102, reason="lateral")
  ra, rb = m._plan_r(a), m._plan_r(b)
  assert ra > 400 and rb == 390
  assert m._offset(b) == (0.0, MC.OFS_MARK, 390.0)


def _finish(m, c, samples, ofs=1.0 * MPH, alert=False, brake=False, blinker=False):
  mp = MC.MapCurvesPlanner(P(), P())
  mp.mc = m
  mp.pass_curve, mp.pass_samples = c, samples
  mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed, mp.pass_shaky = blinker, 5.0, 16.0, False, False
  mp.pass_trust0, mp.pass_dir, mp.pass_ofs, mp.pass_alert, mp.pass_brake = True, m.direction(c), ofs, alert, brake
  mp.pass_ofs_way = MC.curve_way(c)
  mp._finish_pass()
  return mp


def _samples(r, v=17.0, seconds=6.0, cmd=0.5, driver=False, sat_s=0.0):
  n = int(seconds / 0.05)
  out = [(v, 1.0 / r, True, driver, cmd)] * n
  k = int(sat_s / 0.05)
  return out[:n // 2] + [(v, 1.0 / r, True, driver, 1.0)] * k + out[n // 2 + k:]


def test_a_pass_that_proved_the_curve_too_tight_marks_it_for_that_direction_only(tmp_path):
  cases = {"steering limit": dict(samples=_samples(200, sat_s=0.2)), "his steering": dict(samples=_samples(200, driver=True)),
           "his brake": dict(brake=True), "steer alert": dict(alert=True),
           "lateral": dict(samples=_samples(115, v=17.0))}          # 17^2 / 115 = 2.51, + 0.15 = 2.66
  for k, (why, kw) in enumerate(cases.items()):
    m = _mc(tempfile.mkdtemp())
    c = _curve(400, k=k)
    samples = kw.pop("samples", _samples(200))
    mp = _finish(m, c, samples, **kw)
    d = m.direction(c)
    assert m.memory.no_offset(c, d, way=MC.curve_way(c)) == why, (why, m.memory.entries)
    m._ofs = {}
    assert m._offset(c)[:2] == (0.0, MC.OFS_MARK)
    assert mp.offset_events and mp.offset_events[0]["reason"] == why
    # the other direction of the same curve keeps its offset
    m.increasing = False
    m._ofs = {}
    assert m.direction(c) is not None and m._offset(c)[0] > 0, why
  # the mark is kept in the file (with the curve's way id, no other position than the record's own) and read back
  with open(m.memory.path, "w") as f:
    json.dump({"version": 2, "curves": m.memory.entries}, f)
  m2 = MC.CurveMemory(m.memory.path)
  m.increasing = True
  assert m2.no_offset(c, m.direction(c), way=MC.curve_way(c)) == "lateral"
  assert m2.no_offset(c, m.direction(c), way=999) == ""           # (another way's: not this curve's)


def test_a_clean_pass_a_pass_without_the_offset_or_with_his_signal_marks_nothing(tmp_path):
  for kw in (dict(), dict(ofs=0.0, samples=_samples(200, sat_s=0.5)), dict(blinker=True, alert=True),
             dict(samples=_samples(200, sat_s=0.05))):              # (a 0.05 s blip at the limit: PASS_SAT_BLIP)
    m = _mc(tempfile.mkdtemp())
    c = _curve(400)
    samples = kw.pop("samples", _samples(200))
    mp = _finish(m, c, samples, **kw)
    assert m.memory.no_offset(c, m.direction(c)) == "" and not getattr(mp, "offset_events", []), kw
    assert m._offset(c)[0] > 0


def test_the_planner_carries_the_offset_and_the_steering_alert_into_the_pass():
  from types import SimpleNamespace as N
  assert MC.MapCurvesPlanner._steer_alert({"selfdriveState": N(alertType="steerSaturated/warning")})
  assert MC.MapCurvesPlanner._steer_alert({"selfdriveState": N(alertType=""), "onroadEvents": [N(name="steerSaturated")]})
  assert not MC.MapCurvesPlanner._steer_alert({"selfdriveState": N(alertType="laneChange/x"), "onroadEvents": []})
  assert not MC.MapCurvesPlanner._steer_alert({})
