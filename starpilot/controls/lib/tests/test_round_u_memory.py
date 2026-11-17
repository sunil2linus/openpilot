"""Round U, U8: curve memory speed caps (the fastest clean pass, the 30-day expiry, the Curve Speed Offset under a cap) and
the maintenance tool. Synthetic records only (no drive data, no real positions)."""
import importlib.util
import io
import json
import os
import tempfile
import time

_here = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location("map_curves_round_u", os.path.join(_here, "..", "map_curves.py"))
MC = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(MC)
_spec = importlib.util.spec_from_file_location("curve_memory_tool_round_u", os.path.join(_here, "..", "curve_memory_tool.py"))
TOOL = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(TOOL)
MPH = MC.MPH
DAY = 86400.0


def _curve(r=400.0, name="Test Rd", k=0):
  return {"apex": (10.0 + 0.01 * k, 10.0), "a": (9.999 + 0.01 * k, 10.0), "b": (10.001 + 0.01 * k, 10.001), "r": float(r),
          "s_a": float(k), "len": 100.0, "turn": 60.0, "name": name, "ways": [100 + k]}


def _mem():
  return MC.CurveMemory(os.path.join(tempfile.mkdtemp(), "m.json"))


def test_u8a_a_cap_is_never_below_the_fastest_clean_openpilot_pass():
  now = time.time()
  m, c = _mem(), _curve()
  m.add_pass(c, 300.0, t=now - 5 * DAY, v=38.5 * MPH, flags=["full", "op"], direction=45.0)
  m.add_pass(c, 300.0, t=now - 4 * DAY, v=39.5 * MPH, flags=["full", "op", "alert"], direction=45.0)   # the alert: no
  m.add_pass(c, 300.0, t=now - 3 * DAY, v=40.0 * MPH, flags=["full"], direction=45.0)                 # no "op" (older)
  m.add_saturated_pass(c, 40.0 * MPH, t=now - 1 * DAY, direction=45.0)                                # cap 36
  assert abs(m.speed_cap(c, 45.0) - 38.5 * MPH) < 0.01
  m2 = _mem()
  m2.add_saturated_pass(c, 40.0 * MPH, t=now - 1 * DAY, direction=45.0)
  assert abs(m2.speed_cap(c, 45.0) - 36.0 * MPH) < 0.01                                               # as before


def test_u8a_clean_passes_now_carry_op():
  class P:
    def get(self, key, encoding=None):
      return None
  mp = MC.MapCurvesPlanner(P(), P())
  mp.mc = MC.MapCurves(MC.MapCurveStore(tempfile.mkdtemp(), background=False), memory=_mem())
  c = {"apex": (10.0, 10.0), "r": 129.0, "s_a": 0.0}
  mp.pass_curve = c
  mp.pass_samples = [(18.0, 1 / 129.0, True, False, 0.4)] * 60
  mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed = False, 5.0, 16.0, False
  mp._finish_pass()
  assert "op" in mp.mc.memory._find(*c["apex"])["passes"][-1][3]


def test_u8b_a_cap_expires_30_days_after_its_last_saturated_pass():
  now = time.time()
  for age, live in ((29.0, True), (31.0, False)):
    m, c = _mem(), _curve()
    m.add_saturated_pass(c, 40.0 * MPH, t=now - age * DAY, direction=45.0)
    assert (m.speed_cap(c, 45.0) is not None) == live, age
  # a new saturated pass after the expiry starts over (not the old lower cap)
  m, c = _mem(), _curve()
  m.add_saturated_pass(c, 32.0 * MPH, t=now - 40 * DAY, direction=45.0)
  m.add_saturated_pass(c, 44.0 * MPH, t=now - 1 * DAY, direction=45.0)
  assert abs(m.speed_cap(c, 45.0) - 0.9 * 44.0 * MPH) < 0.01 and abs(m.cap_speed(c, 45.0) - 44.0 * MPH) < 0.01
  # a device clock that is not set (or synthetic times): never expires
  m, c = _mem(), _curve()
  m.add_saturated_pass(c, 40.0 * MPH, t=1000.0, direction=45.0)
  assert m.speed_cap(c, 45.0) is not None


def _mc(setting=4):
  m = MC.MapCurves(MC.MapCurveStore(tempfile.mkdtemp(), background=False), memory=_mem())
  m.increasing = True
  m.offset_set = MC.curve_offset_setting(setting)
  return m


def test_u8c_the_offset_under_a_cap_below_the_speed_its_pass_saturated_at():
  now = time.time()
  c = _curve(400.0)
  m = _mc()
  m.memory.add_saturated_pass(c, 30.0 * MPH, t=now - DAY, direction=m.direction(c))     # cap 27 mph, saturated at 30
  g, state, _ = m._offset(c)
  assert state != MC.OFS_NOT and 0 < g <= 30.0 * MPH - MC.OFFSET_CLEAR_TOL - 27.0 * MPH + 1e-6, (g / MPH, state)
  assert abs(g - 2.0 * MPH) < 0.01                                                     # 27 + 2 = 29: 1 mph under 30
  # the cap at its floor (25 mph) from a pass that saturated at 25.5: no room: state 4 as before
  m = _mc()
  m.memory.add_saturated_pass(c, 25.5 * MPH, t=now - DAY, direction=m.direction(c))
  assert m._offset(c) == (0.0, MC.OFS_NOT, 0.0)
  # an older cap (its saturated pass's speed not recorded): state 4 as before
  m = _mc()
  m.memory.add_saturated_pass(c, 30.0 * MPH, t=now - DAY, direction=m.direction(c))
  m.memory._find(*c["apex"], m.direction(c)).pop("cap_v")
  assert m._offset(c) == (0.0, MC.OFS_NOT, 0.0)
  # a cap that expired: the offset as without a cap
  m = _mc()
  m.memory.add_saturated_pass(c, 30.0 * MPH, t=now - 40 * DAY, direction=m.direction(c))
  assert m._offset(c)[0] > 2.0 * MPH


def _file(records):
  d = tempfile.mkdtemp()
  p = os.path.join(d, "curve_memory.json")
  with open(p, "w") as f:
    json.dump({"version": 2, "curves": records}, f)
  return p


def _records():
  now = time.time()
  return [
    {"lat": 12.345678, "lon": -98.765432, "dir": 90.0, "r_map": 90.0, "name": "W Utica Rd", "cap": 17.0, "cap_t": now - DAY,
     "cap_sat_t": now - DAY, "cap_v": 18.9, "cap_hits": 2, "no_offset": [now - DAY, 7, "saturated"],
     "passes": [[95.0, now - 2 * DAY, 16.5, ["full", "op"]]]},
    {"lat": 12.355678, "lon": -98.775432, "dir": 270.0, "r_map": 250.0, "name": "W Utica Rd", "cap": 16.0, "cap_t": now - DAY,
     "passes": []},
    {"lat": 12.445678, "lon": -98.865432, "dir": 0.0, "r_map": 120.0, "name": "Other St", "cap": 12.0, "cap_t": now - DAY,
     "no_offset": [now - DAY, 3, "lateral"], "passes": []},
  ]


def _run(args):
  out = io.StringIO()
  rc = TOOL.main(args, out=out)
  return rc, out.getvalue()


def test_u8d_the_tool_lists_by_road_without_positions():
  p = _file(_records())
  rc, text = _run(["--file", p])
  assert rc == 0 and "W Utica Rd: 2 record(s)" in text and "Other St: 1 record(s)" in text
  assert "cap 38.0 mph (saturated at 42.3)" in text and "no offset: saturated" in text and "fastest 36.9 mph" in text
  for pos in ("12.3", "98.7", "12.4", "98.8", "lat:", "lon"):
    assert pos not in text, pos
  rc, text = _run(["--file", p, "--road", "utica"])
  assert "Other St" not in text and "W Utica Rd" in text


def test_u8d_the_tool_clears_one_road_dry_run_first_and_keeps_a_backup():
  p = _file(_records())
  before = open(p).read()
  rc, text = _run(["--file", p, "--road", "W Utica", "--clear-caps", "--clear-marks"])
  assert rc == 0 and "dry run: would clear 2 cap(s) and 1" in text and open(p).read() == before
  assert not os.path.exists(p + ".bak")
  rc, text = _run(["--file", p, "--road", "W Utica", "--clear-caps", "--clear-marks", "--apply"])
  assert rc == 0 and open(p + ".bak").read() == before
  data = json.load(open(p))["curves"]
  utica = [e for e in data if e["name"] == "W Utica Rd"]
  assert all("cap" not in e and "cap_v" not in e and "no_offset" not in e for e in utica)
  assert utica[0]["passes"] and utica[0]["lat"] == 12.345678                     # everything else kept
  other = [e for e in data if e["name"] == "Other St"][0]
  assert other["cap"] == 12.0 and other["no_offset"][2] == "lateral"
  # only the marks
  p = _file(_records())
  _run(["--file", p, "--road", "Other", "--clear-marks", "--apply"])
  other = [e for e in json.load(open(p))["curves"] if e["name"] == "Other St"][0]
  assert other["cap"] == 12.0 and "no_offset" not in other
  # a clear needs a road
  assert _run(["--file", p, "--clear-caps"])[0] == 2
