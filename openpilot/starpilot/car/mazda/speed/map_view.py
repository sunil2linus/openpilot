"""
mazda_mapviewd: the onroad map side. From the GPS and the curve files it writes a small "map view" for card (no capnp
change): the curve profiles ahead on the matched road as [distance ahead (m), radius (m)] points at the GPS sample's
time, the GPS trust state and the matched road's class. card (curve_plan.py) moves the points by the distance it
drove since that time.

Protocol: VIEW_PATH on tmpfs, one JSON object, replaced atomically (tmp + rename) at the GPS rate (10 Hz):
  {"v": 1, "t": monotonic s of the position (the GPS sample minus GPS_LAG), "gps": "ok" | "none",
   "trust": GpsTrust status (0 ok ...), "start_ok": bool, "held": bool, "matched": bool, "cls": road class,
   "link": bool, "data": bool (curve files cover here), "curves": [{"id", "pts": [[d, r], ...], "ramp", "adv",
   "len"}]}
The reader (read_view) never raises and returns None for a missing, unreadable, other-version or stale view.

Curves ahead (ported from map_curves.MapCurves._curves_ahead, simplified): the curves of the matched chain in our
direction within LOOKAHEAD, never one on a later way of the chain driven against its one-way direction; a curve
ahead is placed by the straight distance to its entry (never more than the road distance: slows early, not late);
a curve we are in by the distance driven since its entry (or, entered unseen, its length minus the distance to its
exit). The profile is mirrored for travel b -> a.
"""
import json
import math
import os
import time

from openpilot.starpilot.car.mazda.speed import map_match as MM

VIEW_PATH = "/dev/shm/mazda_speed_view.json"
VIEW_VERSION = 1
GPS_LAG = 0.85                # s: the position is this old when it arrives (0.7-1.0 s measured)
GPS_STALE = 1.5               # s without a new fix: "none"
LOOKAHEAD = 1000.0            # m
START_ACC = 15.0              # m: start-of-drive trust on a matched link needs the GPS error at or below this
IN_CURVE_SLACK = 60.0         # m past an exit a curve still counts as the one we are in (the old value)


def directed_profile(c, increasing):
  prof = c.get("prof") or [[0.0, c["r"]], [c.get("len", 0.0), c["r"]]]
  if increasing:
    return [(float(s), float(r)) for s, r in prof]
  length = float(c.get("len", prof[-1][0]))
  return [(length - float(s), float(r)) for s, r in reversed(prof)]


class CurvesAhead:
  def __init__(self, store):
    self.store = store
    self.entered = {}           # curve id -> odometer at its entry
    self.chain = None
    self.odo = 0.0
    self._ow = None

  def oneway_ways(self):
    key = tuple(id(ri) for ri in self.store.grid)
    if self._ow is None or self._ow[0] != key:
      self._ow = (key, {int(ri.way[k]) for ri in self.store.grid for k in range(len(ri.way)) if ri.oneway[k]})
    return self._ow[1]

  def against_oneway(self, c, increasing):
    if not c.get("oneway") or not self.store.grid:
      return False
    ow = self.oneway_ways()
    return any(int(w) in ow and bool(fwd) != bool(increasing) for w, fwd in c.get("wdir", []))

  def road(self, way_id, is_forward):
    hit = self.store.way_chain.get(way_id)
    if hit is None:
      return None
    cid, k = hit
    ch = self.store.chains.get(cid)
    if ch is None:
      return None
    increasing = bool(ch["w"][k][1]) == bool(is_forward)
    s0 = ch["w"][k][2]
    s1 = ch["w"][k + 1][2] if k + 1 < len(ch["w"]) else ch["len"]
    return cid, increasing, s0, s1

  def update(self, lat, lon, bearing, way_id, is_forward, dist_driven):
    self.odo += max(dist_driven, 0.0)
    road = self.road(way_id, is_forward) if way_id else None
    if road is None:
      return []
    cid, increasing, s0, s1 = road
    if cid != self.chain:
      self.chain, self.entered = cid, {}
    out = []
    for c in self.store.curves.get(cid, []):
      key = f"{cid}:{c['s_a']}:{'ab' if increasing else 'ba'}"
      entry, exit_ = (c["a"], c["b"]) if increasing else (c["b"], c["a"])
      s_entry, s_exit = (c["s_a"], c["s_b"]) if increasing else (c["s_b"], c["s_a"])
      if increasing and (s_exit < s0 or s_entry > s1 + LOOKAHEAD):
        continue
      if not increasing and (s_exit > s1 or s_entry < s0 - LOOKAHEAD):
        continue
      length = float(c.get("len", 0.0))
      later_way = (s_entry > s1) if increasing else (s_entry < s0)
      if later_way and self.against_oneway(c, increasing):
        continue
      on_this_way = (s0 <= s_entry <= s1) if increasing else (s0 <= s_entry <= s1)
      ent_ahead, d_entry = MM.ahead(lat, lon, bearing, *entry)
      ex_ahead, d_exit = MM.ahead(lat, lon, bearing, *exit_)
      if key in self.entered:
        s_in = self.odo - self.entered[key]
        if s_in > length + IN_CURVE_SLACK:
          continue
        d0 = -s_in
      elif later_way or (on_this_way and ent_ahead):
        if d_entry > LOOKAHEAD:
          continue
        d0 = d_entry
        if d_entry <= MM.NEAR:
          self.entered[key] = self.odo + d_entry
      elif ex_ahead and d_exit <= length + IN_CURVE_SLACK:
        s_in = max(length - d_exit, 0.0)
        self.entered[key] = self.odo - s_in
        d0 = -s_in
      else:
        continue
      pts = [[round(d0 + s, 1), r] for s, r in directed_profile(c, increasing)]
      out.append({"id": key, "pts": pts, "ramp": bool(c.get("ramp")) or int(c.get("cls", 0)) in MM.LINK_CLASSES,
                  "adv": float(c.get("adv", 0.0) or 0.0), "len": length, "r": float(c["r"])})
    return out


def write_view(view, path=VIEW_PATH):
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(view, f, separators=(",", ":"))
  os.replace(tmp, path)


def read_view(path=VIEW_PATH):
  try:
    with open(path) as f:
      v = json.load(f)
  except (OSError, ValueError):
    return None
  if not isinstance(v, dict) or v.get("v") != VIEW_VERSION or not isinstance(v.get("t"), (int, float)):
    return None
  return v


class MapView:
  """The daemon's per-GPS-sample work, without the sockets (tests drive it directly)."""

  def __init__(self, store):
    self.store = store
    self.matcher = MM.RoadMatcher(store)
    self.trust = MM.GpsTrust()
    self.ahead = CurvesAhead(store)
    self.last_t = None

  def update(self, t, gps):
    """t: monotonic s of the GPS sample; gps: dict(lat, lon, bearing, speed, acc, fix). Returns the view dict."""
    view = {"v": VIEW_VERSION, "t": t - GPS_LAG, "gps": "none", "trust": MM.TRUST_NO_GPS, "curves": [],
            "matched": False, "cls": 0, "link": False, "held": False, "start_ok": False, "data": False}
    if gps is None or not gps.get("fix"):
      self.last_t = None
      return view
    dt = 0.1 if self.last_t is None else min(max(t - self.last_t, 0.0), 1.0)
    self.last_t = t
    lat, lon, bearing, speed = gps["lat"], gps["lon"], gps["bearing"], max(gps.get("speed", 0.0), 0.0)
    acc = float(gps.get("acc", 0.0) or 0.0)
    self.store.update(lat, lon)
    self.store.rescan_if_changed()
    data = self.store.covers(lat, lon)
    way_id, fwd = self.matcher.update(lat, lon, bearing, acc if MM.USE_GPS_ACCURACY else 0.0)
    matched = bool(way_id)
    self.trust.update(dt, speed, acc if MM.USE_GPS_ACCURACY else 0.0, 0, data, matched, self.matcher.ambiguous)
    view.update(gps="ok", trust=self.trust.status, matched=matched, cls=self.matcher.road_class,
                link=bool(self.matcher.on_link), data=data)
    # start of the drive: a matched highway link with a good fix is trusted for stepping up in its curves
    view["start_ok"] = bool(self.trust.status == MM.TRUST_SHAKY and self.trust.shaky_why == MM.SHAKY_START and
                            matched and self.matcher.on_link and 0.0 < acc <= START_ACC)
    if self.trust.paused:
      view["paused"] = True
      return view
    curves = self.ahead.update(lat, lon, bearing, way_id, fwd, speed * dt) if matched else []
    if self.trust.low and curves:
      view["held"] = True        # another road this close may be ours: logged, not slowed for
    view["curves"] = curves
    return view


def main():
  from openpilot.cereal import messaging
  from openpilot.common.params import Params
  from openpilot.common.swaglog import cloudlog
  from openpilot.starpilot.car.mazda.speed import config as C
  try:
    os.nice(5)
  except OSError:
    pass
  if not C.read(Params()).active("curves"):
    while True:                  # (the manager only starts it with the setting on; settings apply per drive)
      time.sleep(60)
  store = MM.MapCurveStore()
  mv = MapView(store)
  sm = messaging.SubMaster(["gpsLocationExternal"])
  last_write = 0.0
  while True:
    sm.update(200)
    now = time.monotonic()
    try:
      if sm.updated["gpsLocationExternal"]:
        g = sm["gpsLocationExternal"]
        t = sm.logMonoTime["gpsLocationExternal"] * 1e-9
        fix = bool(g.hasFix) and now - t < GPS_STALE and math.isfinite(g.latitude) and math.isfinite(g.longitude)
        gps = {"lat": g.latitude, "lon": g.longitude, "bearing": g.bearingDeg, "speed": g.speed,
               "acc": g.horizontalAccuracy, "fix": fix}
        write_view(mv.update(t, gps))
        last_write = now
      elif now - last_write > GPS_STALE:
        write_view(mv.update(now, None))
        last_write = now
    except Exception:
      cloudlog.exception("mazda_mapviewd failed")
      time.sleep(1.0)


if __name__ == "__main__":
  main()
