"""Our map's tagged speed limit as a FALLBACK source (format 6 curve files): the road shapes carry it, the map view
gives the matched way's tag for the direction of travel (trust 0, matched; a link only with its own tag), and the limit
follow / street cap use it only where 7.0's map service has no valid reading (7.0's valid reading always wins; a
different 7.0 value keeps our map off that road for a while; a different "possible" value is a gap). Never a class
default. Synthetic roads only; no positions in the event lines."""
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import curve_builder as B
from openpilot.starpilot.car.mazda.speed import limits as L
from openpilot.starpilot.car.mazda.speed import map_match as MM
from openpilot.starpilot.car.mazda.speed import map_view as V
from openpilot.starpilot.car.mazda.speed.contributors import Sources
from openpilot.starpilot.car.mazda.speed.planner import OURS
from openpilot.starpilot.car.mazda.speed.street_cap import StreetCap
from openpilot.starpilot.car.mazda.speed.tests.rig import MPH, Rig, engaged, view
from openpilot.starpilot.car.mazda.speed.tests.test_curve_builder import straight, way
from openpilot.starpilot.car.mazda.speed.tests.test_map_view import gps_at, write_folder

CFG = C.Config(enabled=True, speed_limit="on", exits="on", limit_offset_mph=3)


# ---------------- the view ----------------

def tagged(wid, pts, tag, cls=7, oneway=False, name="Test Road"):
  w = way(wid, pts, name=name, cls=cls, oneway=oneway)
  w.tag = tag
  return w


def trusted_view(tmp_path, ways, **kw):
  write_folder(str(tmp_path), ways, **kw)
  mv = V.MapView(MM.MapCurveStore(root=str(tmp_path), background=False))
  mv.trust.steady_t = MM.STEADY_TIME            # past the start-of-drive time (trust 0 once matched)
  return mv


def drive_north(mv, x=0.0, ys=range(-380, -150, 3), bearing=0.0):
  out = None
  for k, y in enumerate(ys):
    out = mv.update(k * 0.1, gps_at(x, float(y), bearing))
  return out


def test_the_view_gives_the_matched_ways_tag_for_the_direction_of_travel(tmp_path):
  # a two-way road going north (node order): 45 northbound (forward), 40 southbound
  mv = trusted_view(tmp_path, {1: tagged(1, straight(0, -500, 0, 1000), (45, 40))})
  v = drive_north(mv)
  assert v["matched"] and v["trust"] == 0 and abs(v["lim"] - 45 * MPH) < 1e-3 and v["limu"] == "mph"
  v = drive_north(mv, ys=range(150, -100, -3), bearing=180.0)
  assert abs(v["lim"] - 40 * MPH) < 1e-3
  # km/h stays km/h (the car's whole-limit rule converts it as 7.0's reading)
  mv = trusted_view(tmp_path / "kmh", {1: tagged(1, straight(0, -500, 0, 1000), (-50, -50))})
  v = drive_north(mv)
  assert abs(v["lim"] - 50 / 3.6) < 1e-3 and v["limu"] == "kmh"


def test_no_limit_without_a_tag_a_clean_trust_or_a_new_format_file(tmp_path):
  # untagged: no limit (the class default is never one)
  v = drive_north(trusted_view(tmp_path / "a", {1: tagged(1, straight(0, -500, 0, 1000), (0, 0))}))
  assert v["matched"] and "lim" not in v
  # trust not clean (the start of the drive): none
  write_folder(str(tmp_path / "b"), {1: tagged(1, straight(0, -500, 0, 1000), (45, 45))})
  mv = V.MapView(MM.MapCurveStore(root=str(tmp_path / "b"), background=False))
  v = drive_north(mv)
  assert v["matched"] and v["trust"] == MM.TRUST_SHAKY and "lim" not in v
  # a format 5 file (an area not rebuilt yet) still loads (curves, the match) but has no limits
  write_folder(str(tmp_path / "c"), {1: tagged(1, straight(0, -500, 0, 1000), (45, 45))}, version=5, builder=3)
  import json
  for name in (tmp_path / "c" / "42" / "-84").iterdir():
    d = json.loads(name.read_text())
    d["roads"] = [{k: val for k, val in r.items() if k != "l"} for r in d["roads"]]
    name.write_text(json.dumps(d))
  mv = V.MapView(MM.MapCurveStore(root=str(tmp_path / "c"), background=False))
  mv.trust.steady_t = MM.STEADY_TIME
  v = drive_north(mv)
  assert v["matched"] and v["trust"] == 0 and "lim" not in v and mv.store.stale_files == 0
  # an older builder than the car reads (the old fork's format 4): ignored as before
  write_folder(str(tmp_path / "d"), {1: tagged(1, straight(0, -500, 0, 1000), (45, 45))}, version=4, builder=1)
  mv = V.MapView(MM.MapCurveStore(root=str(tmp_path / "d"), background=False))
  v = drive_north(mv)
  assert not v["matched"] and mv.store.stale_files > 0


def test_a_link_gives_a_limit_only_with_its_own_tag(tmp_path):
  ways = {1: tagged(1, straight(0, -500, 0, 1000), (0, 0), cls=4, oneway=True, name="")}
  v = drive_north(trusted_view(tmp_path / "a", ways))
  assert v["link"] and "lim" not in v
  ways = {1: tagged(1, straight(0, -500, 0, 1000), (35, 35), cls=4, oneway=True, name="")}
  v = drive_north(trusted_view(tmp_path / "b", ways))
  assert v["link"] and abs(v["lim"] - 35 * MPH) < 1e-3


def test_road_shapes_carry_the_tag_compactly():
  w = tagged(1, straight(0, -500, 0, 1000), (45, 45))
  assert B.limit_field(w) == 45
  w.tag = (45, 40)
  assert B.limit_field(w) == [45, 40]
  w.oneway, w.tag = True, (45, 0)
  assert B.limit_field(w) == 45                 # one-way: the forward value only
  w.oneway, w.tag = False, (0, 0)
  assert B.limit_field(w) is None
  shapes = [r for lst in B.road_shapes({1: tagged(1, straight(0, -500, 0, 1000), (-60, -60))}).values() for r in lst]
  assert shapes and all(r["l"] == -60 for r in shapes)
  ri = MM.RoadIndex(shapes + [{"w": 2, "k": 7, "p": [1, 1, 1, 1], "l": [45, 0]}, {"w": 3, "k": 7, "p": [1, 1, 1, 1]}])
  assert ri.limit(1, True) == -60 and ri.limit(2, True) == 45 and ri.limit(2, False) == 0 and ri.limit(3, True) == 0


# ---------------- the precedence (limits.MapLimit) ----------------

def mapview(lim_mph=None, cls=3, trust=0, chain=1, matched=True, link=False):
  def f(t):
    v = view(t, trust=trust, chain=chain, cls=cls, matched=matched, link=link)
    if lim_mph is not None:
      v["lim"], v["limu"] = lim_mph * MPH, "mph"
    return v
  return f


def slc(mph):
  return {"limit": None if mph is None else mph * MPH, "upcoming": 0.0, "t": 0.0}


class Logged:
  def __init__(self):
    self.lines = []

  def __call__(self, kind, **kw):
    self.lines.append(dict(kw, kind=kind))


def frame_loop(contribs, steps, dt=0.05):
  """steps: [(seconds, 7.0 mph or None or a 1 s pattern list, our map mph, extra ctx)] -> (rig, the map_limit lines)."""
  log = Logged()
  ml = L.MapLimit(log=log)
  r = Rig(contribs, dt=dt)

  def every(rig, ctx):
    ctx["limit_in"] = ml.decide(ctx)
  r.ml = ml
  for seconds, theirs, ours, extra in steps:
    n = int(round(seconds / dt))
    for k in range(n):
      lim = theirs
      if isinstance(theirs, list):
        lim = theirs[int(k * dt) % len(theirs)]
      r.step(v_mph=extra.get("v", 45.0), set_mph=extra.get("set") if k == 0 else None,
             events=extra.get("events", []) if k == 0 else [], cruise=extra.get("cruise", True),
             every=every, slc=slc(lim), view=mapview(ours, **extra.get("view", {})),
             slc_possible=extra.get("possible"))
  return r, log.lines


def test_our_map_supplies_the_limit_when_7_0_has_none_and_it_becomes_steady_as_a_reading():
  r, lines = frame_loop([L.SpeedLimit(CFG, presses=True)], [(4.9, None, 45, {}), (0.3, None, 45, {})])
  sl = r.cs[0]
  assert sl.steady == 45 and sl.src == "map"                       # the first limit's 5 s steady time
  assert round(r.ctx["contribs"]["speed_limit"].anchor / MPH) == 48  # a trunk road is a street: + 3
  assert [ln["event"] for ln in lines] == ["start"] and lines[0]["mph"] == 45.0 and lines[0]["cls"] == 3
  assert all("lat" not in str(ln) and "lon" not in str(ln) for ln in lines)
  r, _ = frame_loop([L.SpeedLimit(CFG, presses=True)], [(4.5, None, 45, {})])
  assert r.cs[0].steady is None                                   # not before its steady time


def test_7_0s_valid_reading_always_wins_and_a_disagreement_is_logged_once_per_road():
  # 7.0 valid on 1 fix in 3 with 50, our map says 45: 50 becomes steady and stays (no flipping in 7.0's gaps)
  r, lines = frame_loop([L.SpeedLimit(CFG, presses=True)], [(60.0, [50, None, None], 45, {})])
  sl = r.cs[0]
  assert sl.steady == 50 and sl.seq == 1                          # 7.0's 50, never our 45
  dis = [ln for ln in lines if ln["event"] == "disagree"]
  assert len(dis) == 1 and dis[0]["theirs_mph"] == 50.0 and dis[0]["ours_mph"] == 45.0
  assert not [ln for ln in lines if ln["event"] == "start"]       # our map never supplied it
  # the same value from both: one reading, our map fills 7.0's gaps (steady from the first second, never restarted)
  r, lines = frame_loop([L.SpeedLimit(CFG, presses=True)], [(5.3, [45, None, None, None, None], 45, {})])
  assert r.cs[0].steady == 45 and not [ln for ln in lines if ln["event"] == "disagree"]


def test_after_a_disagreement_our_map_comes_back_once_7_0_agrees_or_the_hold_is_over():
  r, lines = frame_loop([L.SpeedLimit(CFG, presses=True)],
                        [(1.0, 50, 45, {}), (L.DISAGREE_HOLD - 2.0, None, 45, {})])
  assert r.ctx["limit_in"]["src"] is None                          # held off on this road after 7.0's 50
  r2, _ = frame_loop([L.SpeedLimit(CFG, presses=True)],
                     [(1.0, 50, 45, {}), (L.DISAGREE_HOLD + 1.0, None, 45, {})])
  assert r2.ctx["limit_in"]["src"] == "map"                        # the hold is over
  r3, _ = frame_loop([L.SpeedLimit(CFG, presses=True)],
                     [(1.0, 50, 45, {}), (1.0, None, 45, {"view": {"chain": 2}})])
  assert r3.ctx["limit_in"]["src"] == "map"                        # another road chain
  r4, _ = frame_loop([L.SpeedLimit(CFG, presses=True)], [(1.0, 50, 45, {}), (1.0, 45, 45, {}), (1.0, None, 45, {})])
  assert r4.ctx["limit_in"]["src"] == "map"                        # 7.0 agreed since


def test_a_different_possible_fix_is_a_gap_the_same_one_is_a_reading():
  poss50 = {"limit": 50 * MPH, "t": 0.0}
  r, _ = frame_loop([L.SpeedLimit(CFG, presses=True)], [(6.0, None, 45, {"possible": poss50})])
  assert r.cs[0].steady is None and r.ctx["limit_in"]["limit"] is None
  # a short "possible" with another value inside our map's reading: the steady time holds (GAP_HOLD), no restart
  r, _ = frame_loop([L.SpeedLimit(CFG, presses=True)],
                    [(3.0, None, 45, {}), (1.0, None, 45, {"possible": poss50}), (2.1, None, 45, {})])
  assert r.cs[0].steady == 45
  poss45 = {"limit": 45 * MPH, "t": 0.0}
  r, _ = frame_loop([L.SpeedLimit(CFG, presses=True)], [(5.3, None, 45, {"possible": poss45})])
  assert r.cs[0].steady == 45


def test_our_map_needs_the_same_trust_and_view_rules():
  # trust 1 frames restart the steady time exactly as for 7.0's reading (the view has no "lim" then either)
  r, _ = frame_loop([L.SpeedLimit(CFG, presses=True)],
                    [(4.0, None, 45, {}), (0.2, None, None, {"view": {"trust": 1}}), (4.0, None, 45, {})])
  assert r.cs[0].steady is None
  # a view without a match: none
  r, _ = frame_loop([L.SpeedLimit(CFG, presses=True)], [(6.0, None, 45, {"view": {"matched": False}})])
  assert r.cs[0].steady is None and r.ctx["limit_in"]["src"] is None
  # stateless (a ctx without limit_in): the same rules minus the disagreement memory
  ctx = {"now": 1.0, "view": mapview(45)(1.0), "slc": None}
  assert L.limit_in(ctx)["src"] == "map"
  ctx = {"now": 1.0, "view": mapview(45, trust=2)(1.0), "slc": None}
  assert L.limit_in(ctx)["src"] is None
  ctx = {"now": 1.0, "view": mapview(45)(1.0), "slc": slc(50)}
  assert L.limit_in(ctx) == {"limit": 50 * MPH, "src": "7.0", "ours": 45 * MPH}
  ctx = {"now": 10.0, "view": mapview(45)(1.0), "slc": None}         # a gone view (9 s old)
  assert L.limit_in(ctx)["src"] is None


def test_the_start_and_stop_lines():
  r, lines = frame_loop([L.SpeedLimit(CFG, presses=True)],
                        [(3.0, None, 45, {}), (3.0, None, 50, {}), (L.STOP_AFTER + 1.0, 50, 50, {})])
  ev = [(ln["event"], ln.get("mph")) for ln in lines]
  assert ev == [("start", 45.0), ("start", 50.0), ("stop", 50.0)] and lines[-1]["why"] == "7.0"


def test_hall_rd_a_red_light_resumes_to_the_limit_plus_3_not_the_unknown_45():
  # 7.0's map service "unknown" on the whole stretch (ErrAmbiguousWay) and during the stop: with our map's tagged 45
  # the RES cap is 48, without it the "limit unknown" rule's 45. (With 7.0's 45 read within 400 m before the stop
  # the round 3 memory already gave 48: test_street_cap.)
  def run(ours):
    sc, sl = StreetCap(CFG, presses=True), L.SpeedLimit(CFG, presses=True)
    steps = [(0.05, None, ours, {"set": 53, "events": [engaged(53)], "v": 50}),
             (40.0, None, ours, {"v": 50}),
             (30.0, None, ours, {"v": 0, "cruise": False}),
             (0.05, None, ours, {"v": 12, "events": [{"kind": "resumed", "after": 53 * MPH}]}),
             (6.0, None, ours, {"v": 20})]
    r, _ = frame_loop([sc, sl], steps)
    return r
  r = run(45)
  assert r.anchor_mph == 48 and r.p.owner == OURS and r.p.ours_why in ("street", "limit")
  r = run(None)
  assert r.anchor_mph == 45                                         # (the old answer: 7.0 alone)


def test_sources_fill_limit_in_and_log_through_the_event_log():
  class Log:
    def __init__(self):
      self.rows = []

    def log(self, kind, **kw):
      self.rows.append((kind, kw))
  lg = Log()
  clock = {"t": 0.0}
  vw = mapview(45)
  src = Sources(want_model=False, want_view=True, sm=None, read_view=lambda: vw(clock["t"]), clock=lambda: clock["t"],
                want_slc=True, log=lg)
  ctx = None
  for k in range(30):
    clock["t"] = k * 0.1
    ctx = src.context({"v_ego": 20.0})
  assert ctx["limit_in"]["src"] == "map" and abs(ctx["limit_in"]["limit"] - 45 * MPH) < 1e-6
  assert lg.rows and lg.rows[0][0] == "map_limit" and lg.rows[0][1]["event"] == "start"
  no = Sources(want_model=False, want_view=True, sm=None, read_view=lambda: vw(0.0), clock=lambda: 0.0)
  assert "limit_in" not in no.context({"v_ego": 20.0})              # no limit option: not computed

