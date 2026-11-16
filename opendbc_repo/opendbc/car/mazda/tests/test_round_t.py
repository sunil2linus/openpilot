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
