"""The map features' GPS on both of 7.0's GPS daemons: gpsLocation (qcomgpsd, comma 3X / tizi such as the Konik A1M)
and gpsLocationExternal (ubloxd); the parked builder's last position from either; the helpers start on a Mazda before
the first drive (CarParamsPersistent); and the status the settings rows and the event log show."""
import json
import math

from openpilot.cereal import log as capnp_log
from openpilot.cereal import messaging
from openpilot.starpilot.car.mazda.speed import curve_mapd as D
from openpilot.starpilot.car.mazda.speed import map_view as V
from openpilot.starpilot.car.mazda.speed import status as ST
from openpilot.starpilot.car.mazda.speed.tests.test_map_view import loop_road, make_view, way
from openpilot.starpilot.car.mazda.speed.tests.test_curve_builder import LAT0, LON0, M_LAT, M_LON

SOURCE = capnp_log.GpsLocationData.SensorSource


def qcom_msg(x=0.0, y=-300.0, fix=True, flags=0):
  """A gpsLocation as 7.0's qcomgpsd fills it (system/qcomgpsd/qcomgpsd.py): no flags, accuracy = ellipse axis."""
  msg = messaging.new_message("gpsLocation", valid=True)
  g = msg.gpsLocation
  g.latitude, g.longitude, g.altitude = LAT0 + y * M_LAT, LON0 + x * M_LON, 200.0
  g.speed, g.bearingDeg = 20.0, -0.0
  g.horizontalAccuracy, g.verticalAccuracy = 4.0, 6.0 if fix else 500.0
  g.source, g.flags, g.hasFix = SOURCE.qcomdiag, flags, fix
  g.unixTimestampMillis = 1_700_000_000_000
  return msg


def ublox_msg(x=0.0, y=-300.0, fix=True):
  """A gpsLocationExternal as ubloxd fills it: the ublox flags (bit 0 = fix ok), accuracy h_acc in m."""
  msg = messaging.new_message("gpsLocationExternal", valid=True)
  g = msg.gpsLocationExternal
  g.latitude, g.longitude = LAT0 + y * M_LAT, LON0 + x * M_LON
  g.speed, g.bearingDeg, g.horizontalAccuracy = 20.0, 0.0, 3.0
  g.source, g.flags, g.hasFix = SOURCE.ublox, 1 if fix else 0, fix
  return msg


class FakeSM:
  def __init__(self):
    self.seen = dict.fromkeys(V.GPS_SERVICES, False)
    self.updated = dict.fromkeys(V.GPS_SERVICES, False)
    self.valid = dict.fromkeys(V.GPS_SERVICES, True)
    self.logMonoTime = dict.fromkeys(V.GPS_SERVICES, 0)
    self.data = {}

  def feed(self, service, msg, t):
    self.seen[service] = self.updated[service] = True
    self.logMonoTime[service] = int(t * 1e9)
    self.data[service] = getattr(msg, service)

  def tick(self):
    self.updated = dict.fromkeys(V.GPS_SERVICES, False)

  def __getitem__(self, s):
    return self.data[s]


def test_both_message_types_give_the_same_sample():
  q = V.gps_sample(qcom_msg().gpsLocation, 10.0, 10.1)
  u = V.gps_sample(ublox_msg().gpsLocationExternal, 10.0, 10.1)
  for g in (q, u):
    assert g["fix"] and abs(g["lat"] - (LAT0 - 300 * M_LAT)) < 1e-9 and g["speed"] == 20.0 and g["bearing"] == 0.0
  assert (q["src"], q["acc"], q["rel"]) == ("qcomdiag", 4.0, 0)
  assert (u["src"], u["acc"], u["rel"]) == ("ublox", 3.0, 0)      # ublox flags = 1 is not "reliability very low"
  assert V.gps_sample(qcom_msg(flags=1).gpsLocation, 10.0, 10.1)["rel"] == 1
  assert not V.gps_sample(qcom_msg(fix=False).gpsLocation, 10.0, 10.1)["fix"]
  assert not V.gps_sample(qcom_msg().gpsLocation, 10.0, 10.0 + V.GPS_STALE)["fix"]          # too old
  assert not V.gps_sample(qcom_msg().gpsLocation, 10.0, 10.1, valid=False)["fix"]
  m = qcom_msg()
  m.gpsLocation.bearingDeg = -90.0                                  # a heading in -180..180 is a compass bearing
  assert V.gps_sample(m.gpsLocation, 10.0, 10.1)["bearing"] == 270.0
  m.gpsLocation.latitude = math.nan
  assert not V.gps_sample(m.gpsLocation, 10.0, 10.1)["fix"]
  from openpilot.starpilot.car.mazda.speed.map_match import gps_quality
  assert gps_quality(ublox_msg().gpsLocationExternal)["reliability"] == 0
  assert gps_quality(qcom_msg(flags=2).gpsLocation)["reliability"] == 2


def test_tizi_only_gpslocation_reaches_the_map(tmp_path):
  """Drive logs: on the Konik A1M only qcomgpsd runs; the view must say "ok" from gpsLocation."""
  mv, _ = make_view(tmp_path, {1: way(1, loop_road(), speed_mph=45)})
  sm, out, t = FakeSM(), None, 100.0
  assert V.no_gps_why(sm, t) == ST.GPS_NO_MESSAGES
  for k, y in enumerate(range(-380, -150, 2)):
    t = 100.0 + k * 0.1
    sm.tick()
    sm.feed("gpsLocation", qcom_msg(y=float(y)), t)
    got = V.pick_gps(sm, t + 0.02)
    assert got is not None and got[2] == "gpsLocation"
    out = mv.update(got[0], got[1])
  assert out["gps"] == "ok" and out["matched"] and len(out["curves"]) == 1
  sm.tick()
  assert V.pick_gps(sm, t + 0.05) is None                           # nothing new this cycle
  assert V.no_gps_why(sm, t + 0.5) == ST.GPS_NO_FIX and V.no_gps_why(sm, t + V.GPS_STALE) == ST.GPS_STALE


def test_ublox_and_newest_fix_wins():
  sm, t = FakeSM(), 50.0
  sm.feed("gpsLocationExternal", ublox_msg(), t)
  assert V.pick_gps(sm, t)[2] == "gpsLocationExternal"
  sm.tick()
  sm.feed("gpsLocation", qcom_msg(fix=False), t + 0.1)              # a no-fix message never hides a live fix
  assert V.pick_gps(sm, t + 0.1) is None
  sm.tick()
  sm.feed("gpsLocation", qcom_msg(), t + 0.2)
  assert V.pick_gps(sm, t + 0.2)[2] == "gpsLocation"                # the newest fix
  sm2 = FakeSM()
  sm2.feed("gpsLocation", qcom_msg(fix=False), t)
  got = V.pick_gps(sm2, t)
  assert got[2] == "gpsLocation" and not got[1]["fix"]               # no fix anywhere: processed as no fix


def test_drive_gps_status_says_why_there_was_no_position(tmp_path, monkeypatch):
  monkeypatch.setattr(ST, "STATUS_DIR", str(tmp_path))
  d = V.DriveGps()
  writes = [w for w in (d.update(0.0, False, ST.GPS_NO_MESSAGES), d.update(0.2, False, ST.GPS_NO_MESSAGES),
                        d.update(1.0, False, ST.GPS_NO_FIX)) if w]
  assert [w["state"] for w in writes] == [ST.GPS_NO_MESSAGES, ST.GPS_NO_FIX]
  ST.write(ST.GPS_NAME, writes[0])
  assert "no GPS position: no message from the GPS" in ST.gps_text(ST.read(ST.GPS_NAME))
  w = d.update(2.0, True, ST.GPS_NO_FIX, "gpsLocation")
  assert w == {"state": ST.GPS_OK, "src": "gpsLocation"}
  assert d.update(9.0, False, ST.GPS_STALE) is None                 # had a position this drive: stays "ok"
  ST.write(ST.GPS_NAME, w)
  assert ST.gps_text(ST.read(ST.GPS_NAME)) == ""
  assert all(k not in json.dumps(ST.read(ST.GPS_NAME)) for k in ("lat", "lon"))


def test_parse_position_reads_both_services_and_navigation_store(tmp_path):
  for msg in (qcom_msg(), ublox_msg()):
    got = D.parse_position(msg.to_bytes())
    assert got is not None and abs(got[0] - (LAT0 - 300 * M_LAT)) < 1e-9
  assert D.parse_position(qcom_msg(fix=False).to_bytes()) is None
  assert D.parse_position(messaging.new_message("carState").to_bytes()) is None
  from openpilot.starpilot.navigation.position import LastPositionStore
  assert D.navigation_position(tmp_path) is None
  LastPositionStore(tmp_path).record({"latitude": 42.5, "longitude": -83.25, "bearing": 0.0})
  LastPositionStore(tmp_path).flush(force=True)
  assert D.navigation_position(tmp_path) == (42.5, -83.25)

  class P:
    def __init__(self, raw):
      self.raw = raw

    def get(self, key):
      assert key == "LastGPSPosition"
      return self.raw
  assert D.last_position(P(None), tmp_path) == (42.5, -83.25)       # tizi: nothing writes LastGPSPosition
  assert D.last_position(P(ublox_msg().to_bytes()), tmp_path)[0] != 42.5


def test_helpers_start_before_the_first_drive_from_carparams_persistent(tmp_path, monkeypatch):
  from opendbc.car.structs import car
  from openpilot.common.params import Params
  from openpilot.starpilot.car.mazda.speed import config as C
  from openpilot.starpilot.car.mazda.speed import processes as PR
  from openpilot.starpilot.schema_cache import put_cache
  from pathlib import Path
  monkeypatch.setattr(PR, "_cache", {"t": None, "raw": None, "car": ("", False)})
  params = Params(str(tmp_path))
  Path(params.get_param_path(C.KEY)).write_bytes(json.dumps({"version": 3, "enabled": True, "curves": "on"}).encode())
  empty = car.CarParams.new_message()                               # the manager's carParams before card ran
  assert not PR.curve_build(False, params, empty)                   # nothing saved yet
  put_cache(params, "CarParamsPersistent", car.CarParams.new_message(brand="mazda"), block=True)
  assert not PR.curve_build(False, params, empty)                   # re-read at most every PERSISTENT_RECHECK s
  monkeypatch.setitem(PR._cache, "t", None)
  import openpilot.starpilot.schema_cache as SC
  decoded = []
  real = SC.inspect_cache
  monkeypatch.setattr(SC, "inspect_cache", lambda *a: decoded.append(1) or real(*a))
  assert PR.curve_build(False, params, empty) and PR.mapview(True, params, empty)
  monkeypatch.setitem(PR._cache, "t", None)
  assert PR.curve_build(False, params, empty)
  assert len(decoded) == 1                                          # unchanged bytes are not decoded again
  assert D.wanted(params, empty)
  assert not PR.curve_build(False, params, car.CarParams.new_message(brand="toyota"))   # the live car wins
  assert not PR.curve_build(True, params, empty) and not PR.mapview(False, params, empty)
  put_cache(params, "CarParamsPersistent", car.CarParams.new_message(brand="mazda", notCar=True), block=True)
  monkeypatch.setitem(PR._cache, "t", None)
  assert not PR.curve_build(False, params, empty)
  params.put("CarParamsPersistent", b"junk", block=True)            # an unchecked value is no car
  monkeypatch.setitem(PR._cache, "t", None)
  assert not PR.curve_build(False, params, empty)


def test_build_status_counts_and_words(tmp_path):
  from openpilot.starpilot.car.mazda.speed.tests.test_curve_mapd import FOLDER, TILES
  reports = []
  folders = {FOLDER: {"dir": "/x", "tiles": TILES}}

  def fake(src, out, **kw):
    return {"files": [], "curves": 0, "ramps": 0}
  assert D.run_once(folders, str(tmp_path), log=lambda s: None, builder=fake, report=reports.append) == 1
  assert [r["state"] for r in reports] == [ST.BUILDING, ST.UP_TO_DATE]
  assert reports[0]["built"] == 0 and reports[0]["total"] == 1 and reports[-1]["built"] == 1
  assert ST.build_text(reports[0]) == "curve maps: building… 0 of 1 folders built (parked, on power)."
  assert ST.build_text(reports[-1]) == "curve maps: up to date (1 of 1 folders built)."
  assert "no offline map downloaded" in ST.build_text(D.build_status(0, None))
  assert ST.build_text(D.build_status(3, 5)) == "curve maps: 3 of 5 folders built; the rest are built while parked."
  assert ST.build_text(None) == "" and ST.build_text({"state": "odd"}) == ""

  def boom(*a, **k):
    raise MemoryError
  reports.clear()
  assert D.run_once({"44/-86": {"dir": "/x", "tiles": TILES}}, str(tmp_path), log=lambda s: None, builder=boom,
                    report=reports.append) == 0
  assert reports[-1]["state"] == ST.PARTIAL and reports[-1]["failed"] == 1


def test_event_log_says_when_the_map_has_no_gps():
  from openpilot.starpilot.car.mazda.speed.caller import SpeedControl

  class Stub:
    gps_key = None
    lines = []

    def _log(self, kind, **kw):
      self.lines.append((kind, kw))
  s = Stub()
  for view in (None, None, {"t": 10.0, "gps": "none", "why": ST.GPS_NO_MESSAGES}, {"t": 10.5, "gps": "none", "why": ST.GPS_NO_MESSAGES},
               {"t": 11.0, "gps": "ok", "src": "gpsLocation"}):
    SpeedControl._note_gps(s, {"view": view, "now": 11.0}, 11.0)
  SpeedControl._note_gps(s, {"view": {"t": 11.0, "gps": "ok", "src": "gpsLocation"}, "now": 30.0}, 30.0)
  got = [(k["state"], k["why"], k["src"], k["view_stale"]) for _, k in s.lines]
  assert got == [("none", "no_view", "", False), ("none", ST.GPS_NO_MESSAGES, "", False), ("ok", "", "gpsLocation", False),
                 ("ok", "", "gpsLocation", True)]
  assert all(kind == "gps" for kind, _ in s.lines)
