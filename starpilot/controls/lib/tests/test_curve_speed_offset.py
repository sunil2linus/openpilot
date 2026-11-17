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


def test_the_cap_setting_each_option_the_default_and_bad_values():
  assert [MC.curve_offset_lat(n) for n in range(5)] == [2.0, 2.5, 3.0, 3.5, 4.0]
  assert [MC.curve_offset_lat(str(n)) for n in range(5)] == [2.0, 2.5, 3.0, 3.5, 4.0]
  for bad in (None, "", "x", -1, 5, "25", "2.5", 2.5):
    assert MC.curve_offset_lat(bad) == 2.5, bad
  for raw, want in (("2", 3.0), (None, 2.5), ("9", 2.5)):
    mp = MC.MapCurvesPlanner(P({"MazdaMapCurves": "2", "MazdaCurveSpeedOffset": "4", "MazdaCurveOffsetLat": raw}), P())
    mp.mc = MC.MapCurves(MC.MapCurveStore(tempfile.mkdtemp(), background=False))
    mp._read_mode()                                    # (at the start of a drive: the planner and its map curves)
    assert mp.offset_lat == want and mp.mc.offset_lat == want


def test_a_road_curve_gets_the_offset_up_to_the_cap_on_its_planning_radius_never_more_than_the_setting(tmp_path):
  for cap in (2.0, 2.5, 3.0, 4.0):
    m = _mc(tempfile.mkdtemp())
    m.offset_lat = cap
    for r in (60, 90, 131, 142, 200, 400):
      c = _curve(r, k=r)
      g, st, rc = m._offset(c)
      v = math.sqrt(MC.PLAN_LAT * r)
      want = max(min(4 * MPH, math.sqrt(cap * r) - v), 0.0)
      assert abs(g - want) < 1e-9, (cap, r, g)
      assert st == (MC.OFS_MAP if want > 0 else MC.OFS_CAP) and rc == r
      assert g <= 4 * MPH + 1e-12 and ((v + g) ** 2 / r <= cap + 1e-9 or g == 0.0)
  # cap 4.0: the setting in full on every road curve of these radii (the worst case: plan + 4 mph)
  m = _mc(tempfile.mkdtemp())
  m.offset_lat = 4.0
  assert all(m._offset(_curve(r, k=r))[0] == 4 * MPH for r in (60, 90, 131, 200))
  # the memory's learned radius: the plan and its cap on it (two agreeing clean passes)
  m = _mc(str(tmp_path))
  c = _curve(131, k=2)
  _passes(m, c, 157.0)
  g, st, rc = m._offset(c)
  assert st == MC.OFS_MEMORY and abs(rc - 157.0) < 0.5 and abs((math.sqrt(2.2 * rc) + g) ** 2 / rc - 2.5) < 1e-6


def test_a_ramp_curve_needs_two_clean_passes_and_its_cap_is_on_the_measured_radius(tmp_path):
  for cap in (2.5, 4.0):
    m = _mc(tempfile.mkdtemp())
    m.offset_lat = cap
    for r in (60, 71, 100, 200):
      assert m._offset(_curve(r, ramp=True, k=r)) == (0.0, MC.OFS_PASSES, 0.0)      # (no clean passes: nothing at any cap)
    c = _curve(90, ramp=True, k=3)
    _passes(m, c, 92.0, n=1)
    assert m._offset(c)[:2] == (0.0, MC.OFS_PASSES)                                # (one pass)
  # exit 46's second curve (map 100, really ~116), two clean passes: up to the cap on 116 m
  for cap, lo, hi in ((2.5, 1.0, 1.5), (3.0, 3.5, 4.0), (4.0, 4.0, 4.0)):
    m = _mc(tempfile.mkdtemp())
    m.offset_lat = cap
    c = _curve(100, ramp=True, k=1)
    _passes(m, c, 116.0)
    g, st, rc = m._offset(c)
    assert st == MC.OFS_MEMORY and abs(rc - 116) < 0.5 and lo * MPH - 1e-9 <= g <= hi * MPH + 1e-9, (cap, g / MPH)
    assert (math.sqrt(2.2 * m._radius(c)) + g) ** 2 / 116.0 <= cap + 1e-9
  # a pass with the steering alert or his brake recorded: not clean (the ramp memory itself is unchanged by them)
  m = _mc(tempfile.mkdtemp())
  m.offset_lat = 3.0
  c = _curve(100, ramp=True, k=4)
  d = m.direction(c)
  m.memory.add_ramp_pass(c, 116.0, d, t=1000.0, v=15.0, flags=["full"])
  m.memory.add_ramp_pass(c, 116.0, d, t=2000.0, v=15.0, flags=["full", "alert"])
  assert m.memory.ramp_radius(c, d) is not None and m.memory.clean_passes(c, d) == 1
  assert m._offset(c)[:2] == (0.0, MC.OFS_PASSES)
  m.memory.add_ramp_pass(c, 116.0, d, t=3000.0, v=15.0, flags=["full"])
  m._ofs = {}                                       # (the next planner update)
  assert m._offset(c)[0] > 0
  # a saturated pass (the ramp memory clears its passes): back to none, e.g. the M-59 loop
  m.memory.clear_ramp(c, d)
  m._ofs = {}
  assert m._offset(c)[:2] == (0.0, MC.OFS_PASSES)
  # a road curve on a link road (no ramp flag) needs them too; a road curve does not
  m = _mc(tempfile.mkdtemp())
  assert m._offset(dict(_curve(400, k=5), cls=2))[:2] == (0.0, MC.OFS_PASSES)
  assert m._offset(_curve(400, k=6))[0] > 0


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
      # (round U, U8 c: a cap with its saturated pass's speed may leave room for the offset; one without it, an older
      # cap, still blocks it)
      m.memory._find(*c["apex"], m.direction(c)).pop("cap_v")
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


def test_the_lateral_mark_is_2p6_up_to_a_cap_of_2p5_then_the_cap_plus_0p3_the_other_triggers_stay():
  assert [MC.offset_mark_lat(c) for c in (2.0, 2.5)] == [2.6, 2.6]
  assert [round(MC.offset_mark_lat(c), 3) for c in (3.0, 3.5, 4.0)] == [3.3, 3.8, 4.3]
  for cap, lat, marked in ((2.5, 2.55, False), (2.5, 2.7, True), (3.0, 2.7, False), (3.0, 3.2, False), (3.0, 3.4, True), (4.0, 4.2, False), (4.0, 4.4, True)):
    m = _mc(tempfile.mkdtemp())
    m.offset_lat = cap
    c = _curve(400)
    v = 17.0
    _finish(m, c, _samples(v * v / (lat - MC.OFFSET_MEAS_MARGIN), v=v))
    assert bool(m.memory.no_offset(c, m.direction(c))) == marked, (cap, lat)
  for kw in (dict(samples=_samples(400, sat_s=0.2)), dict(samples=_samples(400, driver=True)), dict(alert=True), dict(brake=True)):
    m = _mc(tempfile.mkdtemp())
    m.offset_lat = 4.0
    c = _curve(400)
    samples = kw.pop("samples", _samples(400))
    _finish(m, c, samples, **kw)
    assert m.memory.no_offset(c, m.direction(c)), kw


def test_passes_record_the_alert_and_his_brake_as_not_clean(tmp_path):
  m = _mc(str(tmp_path))
  c = _curve(150)
  for k, kw in enumerate((dict(alert=True), dict(brake=True), dict())):
    mp = _finish(m, c, _samples(150), ofs=0.0, **kw)
    mp.mc.memory.entries[-1]["passes"][-1][1] = 1000.0 * k          # (separate passes in time)
  flags = [p[3] for p in m.memory._find(*c["apex"], m.direction(c))["passes"]]
  # (round U, U8 a: every clean pass now also carries "op")
  assert flags[0] == ["alert", "full", "op"] and flags[1] == ["brake", "full", "op"] and flags[2] == ["full", "op"], flags
  assert m.memory.clean_passes(c, m.direction(c)) == 1


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
