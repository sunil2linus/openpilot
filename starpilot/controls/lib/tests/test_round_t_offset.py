"""Round T, T5: the Curve Speed Offset's "no offset" marks (only from passes the offset plan made, expiry, removal by a clean
pass at the offset plan) and ramp passes with his signal before the apex. Synthetic data only."""
import importlib.util
import os
import tempfile
import time

_spec = importlib.util.spec_from_file_location("tcso_round_t", os.path.join(os.path.dirname(__file__), "test_curve_speed_offset.py"))
TS = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(TS)
MC, MPH, P = TS.MC, TS.MPH, TS.P


def _mark_run(cruise_all=True, gas=False, v_max=17.0, plan=17.0, samples=None):
  m = TS._mc(tempfile.mkdtemp())
  c = TS._curve(400)
  mp = MC.MapCurvesPlanner(P(), P())
  mp.mc = m
  mp.pass_curve, mp.pass_samples = c, samples or TS._samples(200, sat_s=0.2)     # proves the curve "too tight"
  mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed, mp.pass_shaky = False, 5.0, 16.0, False, False
  mp.pass_trust0, mp.pass_dir, mp.pass_ofs, mp.pass_alert, mp.pass_brake = True, m.direction(c), 1.0 * MPH, False, False
  mp.pass_ofs_way = MC.curve_way(c)
  mp.pass_cruise_all, mp.pass_gas, mp.pass_ofs_plan, mp.pass_curve_v = cruise_all, gas, plan, [v_max - 1.0, v_max]
  mp._finish_pass()
  return m, c, mp


def test_t5a_a_mark_only_from_a_pass_the_offset_plan_made():
  m, c, mp = _mark_run()
  assert m.memory.no_offset(c, m.direction(c)) == "steering limit"
  for kw, skip in ((dict(cruise_all=False), "cruise off in the pass"), (dict(gas=True), "his gas in the pass"),
                   (dict(v_max=22.3, plan=20.0 + 0.0), "faster than the offset plan")):
    m, c, mp = _mark_run(**kw)
    assert m.memory.no_offset(c, m.direction(c)) == "", kw
    assert mp.offset_skips and mp.offset_skips[-1]["skip"] == skip and not mp.offset_events, kw
  # at most the plan + 2 mph: the offset plan made that speed: marked
  m, c, mp = _mark_run(v_max=20.0 + 2 * MPH, plan=20.0)
  assert m.memory.no_offset(c, m.direction(c)) == "steering limit"


def test_t5a_a_mark_older_than_30_days_expires():
  m = TS._mc(tempfile.mkdtemp())
  c = TS._curve(400)
  d = m.direction(c)
  m.memory.add_no_offset(c, d, way=MC.curve_way(c), reason="lateral", t=time.time() - 31 * 86400.0)
  assert m.memory.no_offset(c, d) == ""
  m._ofs = {}
  assert m._offset(c)[0] > 0                                      # the offset again
  m.memory.add_no_offset(c, d, way=MC.curve_way(c), reason="lateral", t=time.time() - 29 * 86400.0)
  assert m.memory.no_offset(c, d) == "lateral"


def _clear_run(v=17.0, sat_s=0.0, driver=False, brake=False, alert=False, shaky=False, plan=None, r=400.0, lat_on=True):
  m = TS._mc(tempfile.mkdtemp())
  c = TS._curve(400)
  d = m.direction(c)
  m.memory.add_no_offset(c, d, way=MC.curve_way(c), reason="steering limit")
  mp = MC.MapCurvesPlanner(P(), P())
  mp.mc = m
  samples = TS._samples(r, v=v, sat_s=sat_s, driver=driver)
  if not lat_on:
    samples = [(s[0], s[1], False, s[3], s[4]) for s in samples]
  mp.pass_curve, mp.pass_samples = c, samples
  mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed, mp.pass_shaky = False, 5.0, 16.0, False, shaky
  mp.pass_trust0, mp.pass_dir, mp.pass_ofs, mp.pass_alert, mp.pass_brake = True, d, 0.0, alert, brake
  mp.pass_ofs_way = MC.curve_way(c)
  mp.pass_mark_plan = 17.0 if plan is None else plan
  mp.pass_curve_v = [v] * 20
  mp._finish_pass()
  return m.memory.no_offset(c, d), mp


def test_t5a_a_clean_pass_at_the_offset_plan_removes_the_mark():
  mark, mp = _clear_run()
  assert mark == "" and mp.offset_skips[-1]["kind"] == "curveOffsetMarkCleared"
  assert _clear_run(v=17.0 - 0.5 * MPH)[0] == ""                    # within 1 mph of it
  for kw in (dict(v=17.0 - 1.5 * MPH), dict(sat_s=1.0), dict(driver=True), dict(brake=True), dict(alert=True),
             dict(shaky=True), dict(plan=0.0), dict(r=105.0), dict(lat_on=False)):
    assert _clear_run(**kw)[0] == "steering limit", kw
  assert _clear_run(sat_s=0.5)[0] == ""                             # a short saturation under 1 s: still clean


def test_t5a_the_planner_finds_the_plan_a_marked_curve_would_get():
  m = TS._mc(tempfile.mkdtemp())
  c = TS._curve(400)
  g0 = m._offset(c)[0]
  m.memory.add_no_offset(c, m.direction(c), way=MC.curve_way(c), reason="lateral")
  m._ofs = {}
  assert m._offset(c)[:2] == (0.0, MC.OFS_MARK)
  assert abs(m.offset_would(c) - g0) < 1e-9 and g0 > 0


# ---------------- T5b: his signal onto a ramp before the apex no longer drops the pass ----------------

def _end_pass_blinker(ramp, blink_until=None, blink_from=None, apex_x=150.0):
  mp = MC.MapCurvesPlanner(P(), P())
  mp.mc = TS._mc(tempfile.mkdtemp())
  c = TS._curve(80, ramp=ramp)
  c["len"] = 200.0
  mp.pass_curve, mp.pass_x_exit, mp.pass_prev_end, mp.pass_apex_x = c, 250.0, -1e9, apex_x
  buf, ctl = [], []
  for k in range(300):
    x = float(k)
    b = (blink_until is not None and x < blink_until) or (blink_from is not None and x >= blink_from)
    buf.append((x, (15.0, 1 / 80.0, True, False, 0.4), b, False))
    ctl.append((x, True, False))
  mp.pass_buf, mp.pass_ctl = buf, ctl
  seen = {}
  mp._finish_pass = lambda: seen.update(blinker=mp.pass_blinker, cruise=mp.pass_cruise_all, gas=mp.pass_gas,
                                        vs=len(mp.pass_curve_v))
  mp._end_pass(275.0)
  return seen


def test_t5b_ramp_passes_keep_a_signal_that_ended_before_the_apex():
  assert _end_pass_blinker(True, blink_until=120.0)["blinker"] is False          # signalled onto the ramp
  assert _end_pass_blinker(True, blink_from=160.0)["blinker"] is True            # on after the apex: a lane change / exit
  assert _end_pass_blinker(True, blink_until=155.0)["blinker"] is True           # still on at the apex
  # the apex unknown: the first 40 % of the pass
  assert _end_pass_blinker(True, blink_until=60.0, apex_x=None)["blinker"] is False
  assert _end_pass_blinker(True, blink_until=200.0, apex_x=None)["blinker"] is True
  # road curves as before: any signal in the window
  assert _end_pass_blinker(False, blink_until=120.0)["blinker"] is True
  s = _end_pass_blinker(True)
  assert s["blinker"] is False and s["cruise"] is True and s["gas"] is False and s["vs"] == 201


def test_t5b_the_ramp_memory_gets_the_pass_through_the_same_check():
  # a ramp pass with an early signal reaches the ramp memory (add_ramp_pass) now
  m = TS._mc(tempfile.mkdtemp())
  c = TS._curve(80, ramp=True)
  mp = MC.MapCurvesPlanner(P(), P())
  mp.mc = m
  mp.pass_curve, mp.pass_samples = c, [(15.0, 1 / 80.0, True, False, 0.4)] * 120
  mp.pass_blinker, mp.pass_apex, mp.pass_exit, mp.pass_vetoed, mp.pass_shaky = False, 5.0, 16.0, False, False
  mp.pass_trust0, mp.pass_dir, mp.pass_ofs, mp.pass_alert, mp.pass_brake = True, m.direction(c), 0.0, False, False
  mp.pass_ofs_way = MC.curve_way(c)
  mp._finish_pass()
  e = m.memory._find(*c["apex"], m.direction(c))
  assert e is not None and any("ramp" in (p[3] or []) for p in e["passes"])
