"""The on-ramp merge (part f, F3): wait for a real highway limit (the ramp's destination speed when the next limit is
missing or still the street's), boost capped at the car's max, taper, end at the target; his SET- in the loop is a
curve speed; the guards; start-of-drive trust; test mode."""
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed.merge import Merge
from openpilot.starpilot.car.mazda.speed.planner import Contribution, OURS
from openpilot.starpilot.car.mazda.speed.tests.rig import MPH, Rig, driver, engaged, view

CFG = C.Config(enabled=True, merge="on", limit_offset_mph=3)
PATH = (list(range(0, 200, 5)), [0.0] * 40)


def ramp(spd_mph=70.0, rid=7):
  return {"id": rid, "hwy": True, "spd": spd_mph * MPH, "from_hwy": False}


def link_view(spd_mph=70.0, trust=0, start_ok=False, rid=7):
  return lambda t: view(t, link=True, cls=2, chain=rid, ramp=ramp(spd_mph, rid), trust=trust, start_ok=start_ok)


def on_link(r, seconds, v, limit=45, upcoming=0.0, spd=70.0, trust=0, start_ok=False, events=(), **kw):
  kw.setdefault("path", PATH)
  kw.setdefault("model_age", 0.0)
  slc = {"limit": None if limit is None else limit * MPH, "upcoming": 0.0, "t": 0.0}
  kw.setdefault("preview", {"speed": upcoming * MPH, "dist": 300.0, "t": 0.0} if upcoming else None)
  return r.step(v_mph=v, n=int(round(seconds / r.dt)), events=events, slc=slc,
                view=link_view(spd, trust, start_ok), **kw)


def merge_rig(cfg=CFG, extra=()):
  return Rig([*extra, Merge(cfg, presses=cfg.presses("merge"))])


def test_the_next_limit_still_the_streets_at_the_link_start_uses_the_ramps_destination():
  r = merge_rig()
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 1.5, 40, limit=45)
  assert r.anchor_mph == 45                 # its steady time first
  on_link(r, 1.0, 40, limit=45)
  assert r.anchor_mph == 90 and r.p.owner == OURS and r.p.ours_why == "merge"   # 70 + 3 + 20, capped at 90


def test_without_any_highway_limit_it_keeps_waiting_on_the_link():
  r = merge_rig()
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 10, 40, limit=45, spd=45)      # only the street's 45 known: wait, never give up
  assert r.anchor_mph == 45 and r.p.owner != OURS
  on_link(r, 2.5, 42, limit=45, upcoming=65, spd=45)   # the map's preview (mapdOut) shows the highway's 65
  assert r.anchor_mph == 88                 # 65 + 3 + 20


def test_boost_taper_and_end_at_the_target():
  r = merge_rig()
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 2.5, 40)
  assert r.anchor_mph == 90
  on_link(r, 0.5, 60)
  assert r.anchor_mph == 90                 # car 60 (62 on the dash): not yet within 10 of 73
  on_link(r, 0.5, 62)                       # 64 on the dash: within 10, down to 78
  assert r.anchor_mph == 78
  on_link(r, 0.5, 70)                       # within 2: the target, done
  assert r.anchor_mph == 73
  on_link(r, 2.0, 66)
  assert r.anchor_mph == 73                 # done stays done on this link


def test_his_set_minus_before_the_loop_is_a_curve_speed_then_the_merge_after_it():
  curve_target = {"v": 30 * MPH}

  def curve(ctx):
    return Contribution("map_curve", curve_target["v"])
  r = merge_rig(extra=[curve])
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 0.5, 40)
  on_link(r, 0.1, 38, events=[driver("SET_MINUS", 35)])
  on_link(r, 3.0, 33)
  assert r.anchor_mph == 35                 # the loop ahead: the merge waits
  curve_target["v"] = None                  # the curve profile is behind the car
  on_link(r, 0.1, 33)
  assert r.anchor_mph == 90                 # his 35 is a street's speed: the merge raises it
  # without a curve his SET- on the link declines the merge for this link
  r = merge_rig()
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 0.5, 40)
  on_link(r, 0.1, 40, events=[driver("SET_MINUS", 40)])
  on_link(r, 5.0, 40)
  assert r.anchor_mph == 40


def test_guards_hold_the_boost():
  for kw in ({"path": ([0.0, 10.0, 20.0], [0.0] * 3)}, {"radar_lead": True, "lead": None},
             {"radar_lead": True, "lead": {"status": True, "v": 8.0, "vrel": -1.0, "d": 30.0, "t": 1e9}},
             {"lkas_blocked": True}):
    r = merge_rig()
    r.step(v_mph=40, set_mph=45, events=[engaged(45)])
    on_link(r, 3.0, 40, **kw)
    assert r.anchor_mph == 45, kw
  # a moving lead pulling away does not hold it
  r = merge_rig()
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 3.0, 40, radar_lead=True, lead={"status": True, "v": 20.0, "vrel": 2.0, "d": 30.0, "t": 1e9})
  assert r.anchor_mph == 90
  # a guard during the boost drops it to the target
  on_link(r, 0.2, 45, lkas_blocked=True)
  assert r.anchor_mph == 73


def test_start_of_drive_trust_on_the_link():
  r = merge_rig()
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 3.0, 40, trust=1)
  assert r.anchor_mph == 45
  on_link(r, 3.0, 40, trust=1, start_ok=True)
  assert r.anchor_mph == 90


def test_his_own_highway_speed_is_not_raised():
  r = merge_rig()
  r.step(v_mph=40, set_mph=75, events=[engaged(75)])
  on_link(r, 3.0, 40)
  assert r.anchor_mph == 75


def test_test_mode_never_moves_the_anchor():
  cfg = C.Config(enabled=True, merge="test")
  r = merge_rig(cfg)
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  out = on_link(r, 3.0, 40)
  assert r.anchor_mph == 45 and abs(out.dry_anchor / MPH - 90) < 1e-6


def test_with_map_curves_off_the_merge_still_waits_for_the_ramp_curve():
  from openpilot.starpilot.car.mazda.speed.contributors import build
  contributors, _ = build(CFG, None)
  r = Rig(contributors)
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  loop = [{"id": "c", "pts": [[150.0, 40.0], [200.0, 40.0]], "ramp": True, "adv": 0.0}]

  def v_loop(t):
    v = link_view()(t)
    v["curves"] = loop
    return v
  r.step(v_mph=40, n=300, slc={"limit": 45 * MPH, "upcoming": 0.0, "t": 0.0}, view=v_loop, path=PATH, model_age=0.0)
  assert r.anchor_mph == 45                 # the loop ahead (a log-only curve plan): no boost
  on_link(r, 3.0, 40)                       # the loop behind the car
  assert r.anchor_mph == 90


def test_a_stale_view_keeps_the_episode_he_declined():
  r = merge_rig()
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 0.5, 40)
  on_link(r, 0.1, 40, events=[driver("SET_MINUS", 40)])
  r.step(v_mph=40, n=300, slc=None, view=None, path=PATH, model_age=0.0)     # 3 s without a view
  on_link(r, 5.0, 40)
  assert r.anchor_mph == 40                 # still declined: not a new merge
  # another link: a new episode
  r.step(v_mph=40, n=500, slc={"limit": 45 * MPH, "upcoming": 0.0, "t": 0.0}, view=link_view(rid=8), path=PATH,
         model_age=0.0)
  assert r.anchor_mph == 90


def test_the_end_anchor_is_a_whole_step_and_survives_his_change_in_that_frame():
  m = Merge(CFG)
  assert abs(m.whole(47.3 * MPH) / MPH - 47) < 1e-6
  r = merge_rig()
  r.step(v_mph=40, set_mph=45, events=[engaged(45)])
  on_link(r, 2.5, 40)
  assert r.anchor_mph == 90
  # the car reaches the target in a frame with his own change that keeps our anchor (his SET+ under our lowering:
  # the planner takes no proposal in that frame): the one-shot end is proposed again and taken next frame
  r.p.lowered = True
  on_link(r, 0.01, 72, events=[driver("SET_PLUS", 80)])
  assert r.anchor_mph == 90 and r.p.owner == "ours"
  on_link(r, 0.1, 72)
  assert r.anchor_mph == 73


def test_his_set_minus_in_an_interchange_loop_above_street_speed_is_a_curve_speed():
  curve_target = {"v": 30 * MPH}

  def curve(ctx):
    return Contribution("map_curve", curve_target["v"])
  r = merge_rig(extra=[curve])
  r.step(v_mph=60, set_mph=75, events=[engaged(75)])
  on_link(r, 0.5, 60, limit=70, spd=70)
  on_link(r, 0.1, 58, events=[driver("SET_MINUS", 60)], limit=70, spd=70)
  on_link(r, 3.0, 55, limit=70, spd=70)
  assert r.anchor_mph == 60 and r.p.owner == "his"
  curve_target["v"] = None                  # the loop is behind
  on_link(r, 0.2, 55, limit=70, spd=70)
  assert r.anchor_mph == 90                 # his 60 was a curve speed: the merge raises it
