"""Every cereal message our code reads, built as a REAL 7.0 message (pycapnp, this tree's schema) and given to the
reader we use on the car. A renamed field then fails here instead of failing silently on the car (radarState's
LeadData has `present`, not `status`: the lead was never read on the recorded drives).

Services: radarState, modelV2, slcState, mapdOut, selfdriveState, userBookmark (Sources); gpsLocation /
gpsLocationExternal (map_view, LastGPSPosition); carState + carControl (caller.inputs: card's CS / CC are capnp);
carParams (caller.eligible, processes, the TAG button); pandaStates (ti_startup). And the missing-field report: once per
field per drive, never silent."""
from types import SimpleNamespace as NS

from opendbc.car import structs
from openpilot.cereal import log as capnp_log
from openpilot.cereal import messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.starpilot.car.mazda import ti_startup
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import contributors as K
from openpilot.starpilot.car.mazda.speed import curve_mapd, limits, map_view, processes
from openpilot.starpilot.car.mazda.speed.caller import SpeedControl, eligible
from openpilot.starpilot.car.mazda.speed.camera import model_path
from openpilot.starpilot.car.mazda.speed.exit_plan import model_lanes
from openpilot.starpilot.car.mazda.speed.map_match import gps_quality


def real(service, size=None):
  msg = messaging.new_message(service, size) if size is not None else messaging.new_message(service)
  return msg


def reader(msg, service):
  return getattr(msg.as_reader(), service)


class Log:
  def __init__(self):
    self.rows = []

  def log(self, kind, **kw):
    self.rows.append((kind, kw))


class SM:
  """A SubMaster stand-in holding REAL message readers (the SubMaster itself is 7.0's; only its contents matter)."""

  def __init__(self, msgs):
    self.data = {s: reader(m, s) for s, m in msgs.items()}
    self.updated = dict.fromkeys(self.data, True)
    self.valid = dict.fromkeys(self.data, True)
    self.seen = dict.fromkeys(self.data, True)
    self.alive = dict.fromkeys(self.data, True)

  def update(self, _t):
    pass

  def __getitem__(self, s):
    return self.data[s]


def test_every_service_we_subscribe_exists():
  src = K.Sources(want_model=True, want_view=False, sm=SM({}), read_view=lambda: None, want_slc=True, want_radar=True,
                  want_bookmark=True, want_selfdrive=True)
  assert src is not None
  for s in ("modelV2", "slcState", "mapdOut", "radarState", "userBookmark", "selfdriveState", "gpsLocation",
            "gpsLocationExternal", "carState", "carControl", "carParams", "pandaStates"):
    assert s in SERVICE_LIST, s
  assert set(map_view.GPS_SERVICES) <= set(SERVICE_LIST)


# ---- radarState ----
def radar(present=True, d=42.0, v=3.0, vrel=-9.0, prob=0.8):
  m = real("radarState")
  lead = m.radarState.leadOne
  lead.present, lead.dRel, lead.vLead, lead.vRel, lead.modelProb, lead.radar = present, d, v, vrel, prob, False
  return reader(m, "radarState")


def test_radar_lead_reads_7_0s_present():
  out = K.radar_lead(radar())
  assert out == {"status": True, "v": 3.0, "vrel": -9.0, "d": 42.0, "prob": out["prob"], "a": 0.0}
  assert abs(out["prob"] - 0.8) < 1e-6
  assert K.radar_lead(radar(present=False))["status"] is False
  assert "status" not in capnp_log.RadarState.LeadData.schema.fieldnames      # why the old read always failed


def test_radar_lead_falls_back_to_status_and_reports_a_missing_field_once_per_drive():
  log = Log()
  src = K.Sources(want_model=False, want_view=False, sm=SM({}), read_view=lambda: None, log=log)
  old = NS(leadOne=NS(status=True, dRel=10.0, vLead=1.0, vRel=-2.0, modelProb=0.9, aLeadK=0.0))
  assert K.radar_lead(old, warn=src.warn)["status"] is True                    # an older schema's name
  assert log.rows == []
  neither = NS(leadOne=NS(dRel=10.0, vLead=1.0, vRel=-2.0, modelProb=0.9))
  for _ in range(5):
    assert K.radar_lead(neither, warn=src.warn) is None
  assert log.rows == [("schema", {"field": "radarState.leadOne.present", "error": "neither present nor status"})]


def test_sources_hand_the_real_lead_to_the_contributors():
  sm = SM({"radarState": real_radar_msg()})
  src = K.Sources(want_model=False, want_view=False, sm=sm, read_view=lambda: None, want_radar=True, clock=lambda: 5.0)
  ctx = src.context({"v_ego": 20.0})
  assert ctx["lead"]["status"] is True and ctx["lead"]["d"] == 42.0 and ctx["lead"]["t"] == 5.0


def real_radar_msg():
  m = real("radarState")
  lead = m.radarState.leadOne
  lead.present, lead.dRel, lead.vLead, lead.vRel, lead.modelProb = True, 42.0, 3.0, -9.0, 0.8
  return m


# ---- modelV2 ----
def model_msg():
  m = real("modelV2")
  md = m.modelV2
  xs = [float(i * 10) for i in range(33)]
  for name in ("position", "velocity", "orientationRate"):
    xyz = getattr(md, name)
    xyz.x = xs if name == "position" else [20.0] * 33
    xyz.y = [0.1 * i for i in range(33)] if name == "position" else [0.0] * 33
    xyz.z = [0.01] * 33 if name == "orientationRate" else [0.0] * 33
    xyz.t = [0.0] * 33
  lines = md.init("laneLines", 4)
  for k, line in enumerate(lines):
    line.x, line.y, line.z, line.t = [0.0, 10.0, 30.0], [-5.4 + 3.6 * k] * 3, [0.0] * 3, [0.0] * 3
  md.laneLineProbs = [0.9, 0.9, 0.9, 0.9]
  edges = md.init("roadEdges", 2)
  for k, e in enumerate(edges):
    e.x, e.y, e.z, e.t = [0.0, 10.0, 30.0], [-7.0 if k == 0 else 7.0] * 3, [0.0] * 3, [0.0] * 3
  md.roadEdgeStds = [0.3, 0.4]
  return m


def test_model_readers_on_a_real_modelv2():
  md = reader(model_msg(), "modelV2")
  xs, ks = model_path(md)
  assert xs[-1] == 320.0 and abs(ks[0] - 0.01 / 20.0) < 1e-9
  xs2, ys = K.model_path_y(md)
  assert xs2 == xs and abs(ys[-1] - 3.2) < 1e-6
  probs, lines, edges, stds = model_lanes(md)
  assert all(abs(p - 0.9) < 1e-6 for p in probs) and len(lines) == 4
  assert abs(lines[0] + 5.4) < 1e-5 and abs(edges[1] - 7.0) < 1e-6 and abs(stds[1] - 0.4) < 1e-6


def test_model_reader_misses_are_reported():
  got = []
  assert model_path(NS(), warn=lambda what, e: got.append(what)) is None
  assert model_lanes(NS(), warn=lambda what, e: got.append(what)) is None
  assert K.model_path_y(NS(), warn=lambda what, e: got.append(what)) is None
  assert len(got) == 3


# ---- slcState / mapdOut ----
def test_limit_readers_on_real_slcstate_and_mapdout():
  m = real("slcState")
  s = m.slcState
  s.observationKind, s.source, s.speedLimit, s.upcomingSpeedLimit = "valid", "map", 20.1168, 0.0
  assert abs(limits.reading(reader(m, "slcState"))[0] - 20.1168) < 1e-4
  s.observationKind, s.source = "absent", "vision"
  rows = s.init("sourceReadings", 1)
  rows[0].source, rows[0].observationKind, rows[0].speedLimit = "map", "valid", 15.6464
  assert abs(limits.reading(reader(m, "slcState"))[0] - 15.6464) < 1e-4

  m = real("mapdOut")
  o = m.mapdOut
  o.sampleVersion, o.roadStatus, o.waySelectionType, o.speedLimit = 2, "matchedLimit", "possible", 20.1168
  o.previewSpeedLimit, o.previewDistance, o.previewWayId = 15.6464, 300.0, 7
  r = reader(m, "mapdOut")
  assert abs(limits.map_possible(r) - 20.1168) < 1e-3
  pv = limits.map_preview(r)
  assert pv is not None and abs(pv["speed"] - 15.6464) < 1e-3 and pv["dist"] == 300.0


def test_limit_reader_misses_are_reported():
  got = []
  w = lambda what, e: got.append(what)  # noqa: E731
  assert limits.reading(NS(), warn=w) == (None, 0.0)
  assert limits.map_preview(NS(sampleVersion=2), warn=w) is None
  assert limits.map_possible(NS(sampleVersion=2), warn=w) is None
  assert got == ["slcState", "mapdOut.preview", "mapdOut.roadStatus/waySelectionType"]


# ---- selfdriveState / userBookmark ----
def test_selfdrive_state_and_bookmark():
  sd = real("selfdriveState")
  sd.selfdriveState.engageable = True
  sm = SM({"selfdriveState": sd, "userBookmark": real("userBookmark")})
  src = K.Sources(want_model=False, want_view=False, sm=sm, read_view=lambda: None, want_selfdrive=True,
                  want_bookmark=True)
  ctx = src.context({"v_ego": 0.0})
  assert ctx["engageable"] is True and ctx["bookmark"] is True
  log = Log()
  src = K.Sources(want_model=False, want_view=False, sm=SM({}), read_view=lambda: None, log=log)
  assert src._engageable(NS()) is False and log.rows[0][1]["field"] == "selfdriveState.engageable"


# ---- GPS ----
def test_gps_readers_on_both_services():
  for service, source in (("gpsLocation", "qcomdiag"), ("gpsLocationExternal", "ublox")):
    m = real(service)
    g = getattr(m, service)
    g.latitude, g.longitude, g.hasFix, g.speed, g.bearingDeg, g.horizontalAccuracy = 1.5, 2.5, True, 11.0, 90.0, 4.0
    g.source, g.flags, g.satelliteCount = source, 0, 9
    s = map_view.gps_sample(reader(m, service), 10.0, 10.1)
    assert s["fix"] and s["src"] == source and s["speed"] == 11.0 and s["bearing"] == 90.0 and s["acc"] == 4.0
    assert gps_quality(reader(m, service))["satellites"] == 9
    raw = m.to_bytes()
    assert curve_mapd.parse_position(raw) == (1.5, 2.5)


# ---- carState / carControl / carParams ----
def test_caller_inputs_on_real_carstate_and_carcontrol():
  cs = structs.CarState.new_message()
  cs.vEgo, cs.gasPressed, cs.brakePressed, cs.leftBlinker, cs.steeringAngleDeg = 19.0, True, False, True, 3.0
  cs.steeringRateDeg, cs.doorOpen, cs.seatbeltUnlatched, cs.canValid = 12.0, True, True, True
  cs.gearShifter = structs.CarState.GearShifter.drive
  cs.cruiseState.enabled, cs.cruiseState.available, cs.cruiseState.speed = True, True, 20.0
  cc = messaging.new_message("carControl").carControl
  cc.enabled, cc.latActive, cc.actuators.torque = True, True, 0.25
  cc.cruiseControl.cancel, cc.cruiseControl.resume = False, True
  cc.hudControl.visualAlert = "steerRequired"
  ocs = NS(crz_set_plus=False, crz_set_minus=False, crz_resume=False, crz_cancel=False, crz_btns_counter=3,
           lkas_blocked=False, radar_has_lead=True, dashboard_stop_sign=None, cam_lkas=None, cam_laneinfo=None,
           crz_set_allowed=True, crz_not_allowed=False)
  log = Log()
  ctl = SpeedControl(None, C.Config(enabled=True, stops="test"), None, False, log=log)
  d = ctl.inputs(NS(CI=NS(CS=ocs)), cs.as_reader(), cc.as_reader(), [])
  assert (d["v_ego"], d["gas"], d["left_blinker"], d["blinker"], d["angle"], d["steer_rate"]) == (19.0, True, True, True, 3.0, 12.0)
  assert d["gear_drive"] and d["door_open"] and d["seatbelt_unlatched"] and d["can_valid"]
  assert d["engaged"] and d["lat_active"] and d["op_resume"] and d["steer_alert"] and abs(d["torque"] - 0.25) < 1e-6
  assert d["set_speed"] == 20.0 and d["cruise_enabled"] and d["main_on"]
  assert not [r for r in log.rows if r[0] == "schema"]


def test_caller_reports_a_missing_carstate_field_once():
  log = Log()
  ctl = SpeedControl(None, C.Config(enabled=True, stops="test"), None, False, log=log)
  cs = NS(vEgo=1.0)
  for _ in range(3):
    assert ctl._cs(cs, "doorOpen", False) is False
  assert [r for r in log.rows if r[0] == "schema"] == [("schema", {"field": "carState.doorOpen",
                                                                   "error": log.rows[-1][1]["error"]})]


def test_car_params_readers():
  from opendbc.car.mazda.values import MazdaFlags
  cp = structs.CarParams.new_message()
  cp.brand, cp.flags, cp.pcmCruise = "mazda", int(MazdaFlags.GEN1), True
  assert eligible(cp.as_reader()) and processes.car_brand(None, cp.as_reader()) == ("mazda", False)
  assert not eligible(None)


def test_panda_states_reader():
  from opendbc.car.mazda.values import MazdaSafetyFlags
  m = real("pandaStates", 1)
  m.pandaStates[0].safetyModel = "mazda"
  m.pandaStates[0].safetyParam = int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
  assert ti_startup.panda_ti_configured(m.as_reader().pandaStates)
  m.pandaStates[0].safetyParam = 0
  assert not ti_startup.panda_ti_configured(m.as_reader().pandaStates)
