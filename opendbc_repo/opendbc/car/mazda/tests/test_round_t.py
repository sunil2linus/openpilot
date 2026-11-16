"""Round T (drive 77, round S's build): the fixes T1-T6. Unit tests on synthetic inputs only (no drive data)."""
import importlib.util
import os
from types import SimpleNamespace as NS

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, path))
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


TMC = _load("tmc_round_t", "test_map_curves.py")
MC, CTL, G = TMC.MC, TMC.CTL, TMC.G
slf = _load("slf_round_t", os.path.join("..", "speed_limit_follow.py"))
W = _load("wcp_round_t", "test_car_profile_wrapper.py")
MPH = 0.44704
DT = 0.01


class P:
  def __init__(self, vals=None):
    self.vals = vals or {}

  def get(self, key, encoding=None):
    return self.vals.get(key)

  def get_bool(self, key):
    return bool(self.vals.get(key))


class SM(dict):
  alive = {"navInstruction": False}
  valid = {"navInstruction": False}


def _sm(cruise=False):
  return SM(deviceState=NS(started=True), carState=NS(cruiseState=NS(enabled=cruise), leftBlinker=False, rightBlinker=False))


def _planner():
  root, _, _ = TMC.write_files(G.curve_road(r=115, turn=40, split=True))
  mp = MC.MapCurvesPlanner(P({"MazdaMapCurves": "2"}), P())
  mp.mc = MC.MapCurves(MC.MapCurveStore(root, background=False))
  mp.matcher = MC.RoadMatcher(mp.mc.store)
  return mp


# ---------------- T1b: "no GPS position yet" is its own state, with a one-time notice ----------------

def test_t1b_no_position_publishes_status_6_and_trust_5_never_no_data_and_ok():
  mp = _planner()
  mp.update(_sm(), None, 5.0, 0.0)
  plan = NS()
  mp.publish(plan)
  assert plan.mapCurveStatus == MC.STATUS_NO_GPS == 6 and plan.mapGpsTrust == MC.TRUST_NO_GPS == 5
  assert plan.mapGpsShakyWhy == MC.SHAKY_NONE
  # the position arrives: the normal states again
  la, lo = TMC.pos(0, -250)
  for _ in range(3):
    mp.update(_sm(), {"latitude": la, "longitude": lo, "bearing": 0.0}, 18.0, 18.0)
  plan = NS()
  mp.publish(plan)
  assert plan.mapCurveStatus not in (MC.STATUS_NO_GPS, MC.STATUS_NO_DATA) and plan.mapGpsTrust != MC.TRUST_NO_GPS


def test_t1b_the_notice_once_per_drive_when_engaged_or_above_15_mph_and_never_after_a_position():
  mp = _planner()
  notices = []
  for _ in range(200):                         # parked, cruise off: nothing
    mp.update(_sm(), None, 0.0, 0.0)
    notices.append(mp.notice)
  assert not any(notices)
  mp.update(_sm(), None, 16 * MPH, 0.0)        # above 15 mph: once
  assert mp.notice == "no_gps"
  for _ in range(50):
    mp.update(_sm(cruise=True), None, 20 * MPH, 20 * MPH)
    assert mp.notice is None
  # a new drive, cruise engaged at low speed: once
  mp2 = _planner()
  mp2.update(_sm(), None, 0.0, 0.0)
  mp2.update(_sm(cruise=True), None, 5 * MPH, 25 * MPH)
  assert mp2.notice == "no_gps"
  # a position came first, then lost: no "no GPS yet" notice (the state is still published)
  mp3 = _planner()
  la, lo = TMC.pos(0, -250)
  mp3.update(_sm(), {"latitude": la, "longitude": lo, "bearing": 0.0}, 18.0, 18.0)
  for _ in range(20):
    mp3.update(_sm(cruise=True), None, 20 * MPH, 20 * MPH)
    assert mp3.notice is None
  plan = NS()
  mp3.publish(plan)
  assert plan.mapCurveStatus == MC.STATUS_NO_GPS


def test_t1b_the_car_side_never_reads_the_no_gps_state_as_a_good_map(tmp_path, monkeypatch):
  # the no-GPS fallback (round R, R8): status 6 with no accuracy is "no fix", as status 3 was
  c = W.make_cs(tmp_path, W.FakeParams({"IsMetric": False}), monkeypatch)
  Pl = lambda **k: NS(**{**dict(mapCurveMode=2, mapRoadClass=0, mapCurveStatus=0, mapGpsAccuracy=0.0), **k})  # noqa: E731
  assert c._no_gps(Pl(mapCurveStatus=6), True)
  assert not c._no_gps(Pl(mapCurveStatus=6, mapGpsAccuracy=6.0), True)   # a fix lost later keeps its accuracy: as before
  # trust 5 is above every car-side trust gate
  assert MC.TRUST_NO_GPS > slf.LIMIT_GPS_TRUST_MAX and MC.TRUST_NO_GPS >= CTL.TRUST_PAUSED
  ar = _load("ar_round_t", os.path.join("..", "auto_resume.py"))
  assert MC.TRUST_NO_GPS > ar.MAX_GPS_TRUST
  # the car-side map control: status 6 is no curve (only 1 / 2 are), and a ramp curve's hold keeps it as for 4 / 5
  assert CTL.STATUS_NO_GPS == MC.STATUS_NO_GPS
  ctl = CTL.MapCurveControl(3.0)
  f = CTL.MapFields(mode=CTL.ON)
  f.status, f.radius, f.distance, f.gps_trust = 6, 0.0, 0.0, 5
  assert not ctl.update(f, 45 * MPH, 0).active


# ---------------- T1c: why the trust is 1, and the merge window with the start-of-drive trust 1 ----------------

def test_t1c_the_planner_says_why_the_trust_is_1():
  t = MC.GpsTrust()
  dt = 0.05
  for _ in range(int(10 / dt)):
    t.update(dt, 20.0, 5.0, 4, True, True, False)
  assert t.status == MC.TRUST_SHAKY and t.shaky_why == MC.SHAKY_START
  for _ in range(int(MC.STEADY_TIME / dt) + 2):
    t.update(dt, 20.0, 5.0, 4, True, True, False)
  assert t.status == MC.TRUST_OK and t.shaky_why == MC.SHAKY_NONE
  # a suspicious gap restarts the steady time: shaky for another reason now
  for _ in range(int((MC.STEADY_GAP + 0.5) / dt)):
    t.update(dt, 20.0, 5.0, 4, True, False, False)
  t.update(dt, 20.0, 5.0, 4, True, True, False)
  assert t.status == MC.TRUST_SHAKY and t.shaky_why == MC.SHAKY_RESTART
  # low (another road close): not 1 at all
  t2 = MC.GpsTrust()
  t2.update(dt, 20.0, 5.0, 4, True, True, True)
  assert t2.status == MC.TRUST_LOW and t2.shaky_why == MC.SHAKY_NONE
  # a pause: restart afterwards
  t3 = MC.GpsTrust()
  for _ in range(int((MC.BAD_CONFIRM + 0.2) / dt)):
    t3.update(dt, 20.0, 80.0, 4, True, True, False)
  for _ in range(int((MC.RESUME_TIME + 0.2) / dt)):
    t3.update(dt, 20.0, 5.0, 4, True, True, False)
  assert t3.status == MC.TRUST_SHAKY and t3.shaky_why == MC.SHAKY_RESTART


def test_t1c_the_planner_publishes_the_reason():
  m, rm = TMC.planner(TMC.write_files(G.curve_road(r=115, turn=40))[0]), None
  rm = MC.RoadMatcher(m.store)
  trust = MC.GpsTrust()
  la, lo = TMC.pos(0, -250)
  for _ in range(3):
    w, f = rm.update(la, lo, 0.0, 0.0)
    m.update(la, lo, 0.0, w, f, trust=trust, gps={"dt": 0.05, "v_ego": 20.0, "acc": 0.0, "rel": 0, "ambiguous": False})
  mp = MC.MapCurvesPlanner(None, None)
  mp.mode, mp.mc = 2, m
  plan = NS()
  mp.publish(plan)
  assert plan.mapGpsTrust == MC.TRUST_SHAKY and plan.mapGpsShakyWhy == MC.SHAKY_START


STREET, LINK = 5, 2


def _merge_run(next_fn, trust=0, why=0, acc=0.0, driver=45, frames=3000, link_at=600):
  """A street (45) then a link to a highway from frame link_at; next_fn(k) the next limit (mph) on the link."""
  f = slf.SpeedLimitFollower(False)
  offers = []
  for k in range(frames):
    on_link = k >= link_at
    li = slf.LimitInputs(limit=45 * MPH, source="Map Data", gps_trust=0 if not on_link else trust,
                         driver_speed=driver * MPH, cruise_enabled=True, road_class=LINK if on_link else STREET,
                         ramp_to_highway=on_link, next_limit=(next_fn(k - link_at) * MPH if on_link else 0.0),
                         next_distance=400.0 if on_link else -1.0, v_ego=driver * MPH,
                         gps_shaky_why=why if on_link else 0, gps_acc=acc)
    f.update(li)
    if f.offer is not None and f.offer.kind == "merge" and not offers:
      offers.append((k - link_at, f.offer.target))
  reasons = [e[1]["reason"] for e in f.events if e[0] == "speedLimitNotFollowed"]
  return offers, reasons


def test_t1c_the_merge_opens_with_the_start_of_drive_trust_1_on_a_link_to_a_highway_with_good_accuracy():
  ofs, _ = _merge_run(lambda k: 70, trust=1, why=MC.SHAKY_START, acc=8.0)
  assert ofs and ofs[0][1] == 70 + slf.MERGE_OVER_MPH
  for kw in (dict(why=MC.SHAKY_RESTART, acc=8.0), dict(why=MC.SHAKY_START, acc=20.0), dict(why=MC.SHAKY_START, acc=0.0)):
    ofs, reasons = _merge_run(lambda k: 70, trust=1, **kw)
    assert not ofs, kw
    assert reasons.count("merge: GPS trust 1 (waiting)") == 1, (kw, reasons)   # logged once, never silent
  ofs, reasons = _merge_run(lambda k: 70, trust=2, why=0, acc=8.0)
  assert not ofs and "merge: GPS trust 2 (waiting)" in reasons
  ofs, _ = _merge_run(lambda k: 70, trust=0)
  assert ofs


# ---------------- T2: the street's limit at the link's start does not end the merge for the link ----------------

def test_t2_the_streets_45_at_the_start_of_the_link_waits_for_the_highways_70():
  # drive 77 t 310.1: the next limit 45 for 5 s on the link, then 70; his speed 50
  ofs, reasons = _merge_run(lambda k: 45 if k < 500 else 70, driver=50)
  assert ofs and ofs[0][0] >= 500 and ofs[0][1] == 75, (ofs, reasons)
  assert "merge: next limit 45 is not a highway limit (waiting)" in reasons
  assert not any("is not below" in r for r in reasons), reasons


def test_t2_his_speed_at_the_merges_target_is_his_but_is_decided_again_if_the_next_limit_rises():
  # the next limit 50 (a highway limit: 50 + 5 = 55), his speed 55: refused; then 70: offered
  ofs, reasons = _merge_run(lambda k: 50 if k < 800 else 70, driver=55)
  assert any("driver's speed 55 is not below the merge's 55" in r for r in reasons), reasons
  assert ofs and ofs[0][0] >= 800 and ofs[0][1] == 75
  # it stays 50: refused for the link, once
  ofs, reasons = _merge_run(lambda k: 50, driver=55)
  assert not ofs and sum("is not below" in r for r in reasons) == 1
  # his speed above the highway's target: still his
  ofs, _ = _merge_run(lambda k: 70, driver=76)
  assert not ofs


# ---------------- T3: the loop's speed is kept at the LKAS cut-off ----------------

cs = W.cs


def _inp(**k):
  d = dict(enabled=True, cruise_enabled=True, set_speed=20 * MPH, v_ego=25 * MPH, gas_pressed=False, brake_pressed=False,
           standstill=False, plan_alive=True, csc_active=False, csc_speed=0.0, other_button_active=False)
  d.update(k)
  return cs.SlowdownInputs(**d)


def _run_logic(frames=1500, start=45.0, **kw):
  lg = cs.CurveSlowdownLogic(False, 10)
  lg.update(_inp(set_speed=start * MPH, v_ego=start * MPH))
  set_mph, low = start, start
  for f in range(frames):
    b, _ = lg.update(_inp(set_speed=set_mph * MPH, v_ego=max(set_mph + 4, 31) * MPH, btn_counter=(f // 10) % 16, **kw))
    if b == cs.SET_MINUS and (lg.last_action != "hold" or f % 10 == 0):
      set_mph -= 1
    low = min(low, set_mph)
  return low


def test_t3a_the_followers_undershoot_never_takes_the_set_under_29_and_stops_still_go_lower():
  # drive 77: the ramp follower asked 28 (and lower when firm) with the car well above the cut-off
  firm = dict(csc_active=True, csc_speed=24.3 * MPH, curve_track=True, curve_hold=True, curve_floor=24.3 * MPH, curve_budget=60)
  assert _run_logic(**firm) <= 26                                         # before: under the cut-off
  assert _run_logic(lkas_hard=cs.LKAS_HARD_SPEED[0], **firm) == 29        # on the approach too (the car at 33+)
  stop = dict(stop_active=True, stop_target=20 * MPH, stop_urgent=True)
  assert _run_logic(lkas_hard=cs.LKAS_HARD_SPEED[0], **stop, **firm) <= 25
  # never above his own speed
  assert _run_logic(start=27.0, lkas_hard=cs.LKAS_HARD_SPEED[0], **firm) <= 27


def _w(own_mph, why="ramp", cam=None, ti_only=False, metric=False):
  return NS(lkas_hard=0.0, has_ti=True, ti_only=ti_only, map_on_used=True, curve_enabled=True,
            logic=NS(step=0.277778 if metric else MPH),
            map_out=NS(active=True, track=True, why=why, curve_speed=own_mph * MPH),
            curve_target=NS(active=cam is not None, speed=(cam or 0.0) * MPH),
            _followed_map_curve=lambda: True)


def test_t3a_the_hard_floor_only_for_a_followed_map_curve_at_or_above_30():
  def hard(w):
    w._followed_map_curve = lambda: cs.CurveSlowdown._followed_map_curve(w)
    return cs.CurveSlowdown._lkas_hard(w, True, False)
  assert abs(hard(_w(31.0)) - 29 * MPH) < 1e-9 and abs(hard(_w(30.0, why="curve")) - 29 * MPH) < 1e-9
  assert hard(_w(29.5)) == 0.0                                           # a curve that needs less: as before
  assert abs(hard(_w(31.0, cam=27.5)) - 27.5 * MPH) < 1e-9               # the camera's own lower target counts
  assert hard(_w(31.0, ti_only=True)) == 0.0                             # TI-only planning (a block from before)
  assert hard(_w(31.0, why="nav exit")) == 0.0
  w = _w(31.0)
  w.map_out.track = False
  assert hard(w) == 0.0
  assert abs(hard(_w(31.0, metric=True)) - cs.LKAS_HARD_SPEED[1]) < 1e-9 and abs(cs.LKAS_HARD_SPEED[1] / 0.277778 - 47) < 0.01
  assert cs.CurveSlowdown._lkas_hard(_w(31.0), True, True) == 0.0        # the street speed governs: not a curve


def _cs_block(tmp_path, monkeypatch):
  c = W.make_cs(tmp_path, W.FakeParams({"IsMetric": False}), monkeypatch)
  c.has_ti = True
  return c


def _blk(c, block, v_mph=27.0, fault=False):
  c._lkas_state(NS(lkas_block=block, cam_fault=2 if fault else 0, ti_state_log=3, cam_err=fault, stock_lkas_off=False),
                v_mph * MPH)


def test_t3b_a_block_that_starts_in_a_followed_curve_keeps_its_plan(tmp_path, monkeypatch):
  c = _cs_block(tmp_path, monkeypatch)
  c.map_on_used, c.map_out = True, NS(active=True, track=True, why="ramp", curve_speed=30.5 * MPH)
  _blk(c, False)
  _blk(c, True)                                   # the car fell under the cut-off in the loop
  assert c.block_hold and not c.ti_only and not c.curve_target.ti_only
  for _ in range(300):
    _blk(c, True)
  assert not c.ti_only
  # the curve is not followed any more: TI-only after TI_ONLY_HYST (the block still on)
  c.map_out = NS(active=False, track=False, why="", curve_speed=100.0)
  for _ in range(int(cs.TI_ONLY_HYST / DT) + 2):
    _blk(c, True)
  assert c.ti_only and not c.block_hold
  # the next curve while the block is still on: TI-only (the block was there before it)
  c.map_out = NS(active=True, track=True, why="ramp", curve_speed=30.5 * MPH)
  _blk(c, True)
  assert c.ti_only


def test_t3b_a_camera_fault_or_a_block_from_before_the_curve_plans_ti_only(tmp_path, monkeypatch):
  c = _cs_block(tmp_path, monkeypatch)
  c.map_on_used, c.map_out = True, NS(active=True, track=True, why="ramp", curve_speed=30.5 * MPH)
  _blk(c, False)
  _blk(c, True, fault=True)
  assert c.ti_only and not c.block_hold
  c2 = _cs_block(tmp_path, monkeypatch)
  c2.map_on_used, c2.map_out = True, NS(active=False, track=False, why="", curve_speed=100.0)
  _blk(c2, True)                                  # blocked first
  c2.map_out = NS(active=True, track=True, why="ramp", curve_speed=30.5 * MPH)
  _blk(c2, True)
  assert c2.ti_only and not c2.block_hold
  # a hold ends with a camera fault
  c3 = _cs_block(tmp_path, monkeypatch)
  c3.map_on_used, c3.map_out = True, NS(active=True, track=True, why="curve", curve_speed=31 * MPH)
  _blk(c3, False)
  _blk(c3, True)
  assert c3.block_hold
  _blk(c3, True, fault=True)
  assert c3.ti_only and not c3.block_hold


def test_t3c_the_notice_shows_the_car_sides_own_target():
  c = CTL.MapCurveControl(3.0)
  c.ti_plan = True
  f = CTL.MapFields(mode=CTL.ON)
  f.status, f.radius, f.distance, f.on_ramp, f.road_class = 2, 84.0, 0.0, True, 2
  for _ in range(200):
    c.update(f, 45 * MPH, 0, v_ego=30 * MPH)
  normal = c.target_now
  assert normal > 0
  c.ti_only = True
  c.update(f, 45 * MPH, 0, v_ego=30 * MPH)
  assert 0 < c.target_now < normal - 1.0                                  # TI-only: the lower speed is what is shown
  w = NS(map_control=c)
  assert cs.CurveSlowdown.curve_target_speed.fget(w) == c.target_now
  f.status, f.radius = 0, 0.0
  c.update(f, 45 * MPH, 0, v_ego=30 * MPH)
  assert c.target_now == 0.0
