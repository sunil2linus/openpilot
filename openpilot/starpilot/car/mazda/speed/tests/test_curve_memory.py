"""Curve memory: what a pass is, what teaches a cap, the cap/offset rules on the tight points, expiry, the file, the
tool, and passes recorded through the curve contributor."""
import json
import math

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import curve_memory as CM
from openpilot.starpilot.car.mazda.speed import curve_memory_tool as T
from openpilot.starpilot.car.mazda.speed.curve_plan import CurvePlan

MPH = CM.MPH
DAY = CM.DAY


def sample(v=15.0, angle=20.0, lat_active=True, torque=0.3, **kw):
  s = {"v_ego": v, "angle": angle, "lat_active": lat_active, "torque": torque, "cruise_enabled": True}
  s.update(kw)
  return s


def run_pass(n=300, ramp=False, ours=True, before_apex=150, **kw):
  rec = CM.PassRecorder("7:ab", "Test Rd", ramp, ours)
  for k in range(n):
    rec.sample(sample(**{key: (val(k) if callable(val) else val) for key, val in kw.items()}), before_apex=k < before_apex)
  return rec.finish(1_700_000_000.0)


def mem(tmp_path, now=1_700_000_000.0):
  clock = [now]
  m = CM.CurveMemory(path=str(tmp_path / "m.json"), clock=lambda: clock[0], background=False)
  return m, clock


def test_what_a_pass_is():
  p = run_pass()
  assert p["clean"] and not p["teach"] and abs(p["v"] - 15.0) < 1e-9
  assert abs(p["lat"] - CM.wheel_lat(15.0, 20.0)) < 0.01
  assert run_pass(n=150) is None                                      # too short
  assert run_pass(lat_active=lambda k: k < 200) is None               # not openpilot-steered
  assert run_pass(v=5.0) is None
  assert run_pass(blinker=lambda k: k == 10) is None                  # a road pass with a blinker before the apex
  assert run_pass(blinker=lambda k: k == 10, ramp=True) is not None   # a ramp pass is kept
  assert run_pass(blinker=lambda k: k == 200) is not None             # after the apex: kept


def test_what_teaches_a_cap():
  sat = run_pass(torque=lambda k: 1.0 if k < 120 else 0.3)
  assert not sat["clean"] and sat["teach"]
  assert run_pass(steer_alert=lambda k: k == 5)["teach"]
  assert run_pass(steering_pressed=lambda k: k == 5)["teach"]
  assert not run_pass(torque=lambda k: 1.0 if k < 50 else 0.3)["teach"]   # under SAT_TIME: clean
  assert not run_pass(torque=1.0, gas=lambda k: k == 3)["teach"]          # his gas: never a cap
  assert not run_pass(torque=1.0, cruise_enabled=lambda k: k > 10)["teach"]   # cruise off
  assert not run_pass(torque=1.0, ours=False)["teach"]                    # not our plan's speed


def test_cap_never_below_the_fastest_clean_pass_and_expires(tmp_path):
  m, clock = mem(tmp_path)
  key = "7:ab"
  m.add_pass(key, "Test Rd", False, {"t": clock[0], "v": 17.0, "lat": 2.0, "clean": True, "teach": False})
  m.add_pass(key, "Test Rd", False, {"t": clock[0], "v": 16.0, "lat": 2.0, "clean": False, "teach": True})
  assert m.cap(key) == 17.0                                           # 0.9 x 16 would be under the clean 17
  assert not m.offset_allowed(key, False)
  m.add_pass(key, "Test Rd", False, {"t": clock[0], "v": 18.5, "lat": 2.0, "clean": True, "teach": False})
  assert m.cap(key) == 18.5
  clock[0] += 31 * DAY
  assert m.cap(key) is None and m.offset_allowed(key, False)
  m.add_pass("9:ba", "Hard Rd", False, {"t": clock[0], "v": 20.0, "lat": 2.8, "clean": True, "teach": False})
  assert not m.offset_allowed("9:ba", False) and m.cap("9:ba") is None    # a hard clean pass: no offset there


def test_ramp_offset_needs_two_clean_passes(tmp_path):
  m, clock = mem(tmp_path)
  assert not m.offset_allowed("5:ab", True) and m.offset_allowed("5:ab", False)
  for _ in range(2):
    m.add_pass("5:ab", "Ramp", True, {"t": clock[0], "v": 12.0, "lat": 2.0, "clean": True, "teach": False})
  assert m.offset_allowed("5:ab", True)


def test_cap_and_offset_act_on_the_tight_points_only(tmp_path):
  m, clock = mem(tmp_path)
  pts = [(50.0, 72.0), (100.0, 85.0), (160.0, 200.0)]
  base = [math.sqrt(2.2 * r) for _, r in pts]
  got = CM.apply_memory(pts, base, "7:ab", False, m, 3 * MPH, 3.0, 2.2)
  assert got[0] == min(base[0] + 3 * MPH, math.sqrt(3.0 * 72)) and got[1] > base[1] and got[2] == base[2]
  m.add_pass("7:ab", "Test Rd", False, {"t": clock[0], "v": 14.0, "lat": 2.0, "clean": False, "teach": True})
  got = CM.apply_memory(pts, base, "7:ab", False, m, 3 * MPH, 3.0, 2.2)
  cap = m.cap("7:ab")
  assert got[0] == min(base[0], cap) and got[2] == base[2]           # capped, offset removed, wide point untouched


def test_file_roundtrip_and_tool(tmp_path):
  m, clock = mem(tmp_path)
  m.add_pass("7:ab", "Rochester Rd", False, {"t": clock[0], "v": 14.0, "lat": 2.0, "clean": False, "teach": True})
  m.add_pass("8:ba", "Main St", False, {"t": clock[0], "v": 15.0, "lat": 2.0, "clean": True, "teach": False})
  assert m.save_if_due(0.0)
  raw = (tmp_path / "m.json").read_text()
  assert "lat\": 2.0" in raw and "latitude" not in raw and "lon" not in raw
  assert CM.CurveMemory(path=str(tmp_path / "m.json"), clock=lambda: clock[0], background=False).cap("7:ab") is not None
  lines = []
  assert T.main(["list", "--file", str(tmp_path / "m.json")], out=lines.append, now=clock[0]) == 0
  assert any("Rochester Rd" in line and "cap" in line for line in lines) and lines[-1] == "2 of 2 curves"
  lines = []
  T.main(["clear", "--name", "rochester", "--file", str(tmp_path / "m.json")], out=lines.append, now=clock[0])
  assert "dry run" in lines[-1] and len(json.loads(raw)["curves"]) == 2
  assert json.load(open(tmp_path / "m.json"))["curves"].keys() == {"7:ab", "8:ba"}
  T.main(["clear", "--name", "rochester", "--apply", "--file", str(tmp_path / "m.json")], out=lines.append, now=clock[0])
  assert set(json.load(open(tmp_path / "m.json"))["curves"]) == {"8:ba"}
  assert set(json.load(open(str(tmp_path / "m.json") + ".bak"))["curves"]) == {"7:ab", "8:ba"}


def test_passes_are_recorded_through_the_curve_contributor(tmp_path):
  m, clock = mem(tmp_path)
  cfg = C.Config(enabled=True, curves="on", curve_offset_mph=3)
  plan = CurvePlan(cfg, memory=m)
  v = 15.0
  pts0 = [[20.0, 80.0], [60.0, 80.0], [100.0, 150.0]]
  for k in range(int(200 / v / 0.01)):
    driven = v * 0.01 * k
    view = {"v": 1, "t": 0.0, "gps": "ok", "trust": 0,
            "curves": [{"id": "1:0:ab", "way": 7, "name": "Test Rd", "ramp": False, "pts": [[d - driven, r] for d, r in pts0]}]}
    plan({"view": view, "now": 0.0, "v_ego": v, "lat_active": True, "angle": 15.0, "torque": 0.4, "cruise_enabled": True,
          "plan_reason": "map_curve"})
  plan({"view": {"v": 1, "t": 0.0, "gps": "ok", "trust": 0, "curves": []}, "now": 0.0, "v_ego": v})
  e = m.curves["7:ab"]
  assert e["name"] == "Test Rd" and len(e["passes"]) == 1 and e["passes"][0]["clean"]
