"""The planner's contributors for this drive, chosen from the saved config, and the inputs they share (Sources).

Each contributor is a callable(ctx) -> planner.Contribution or None, run in this order (a later one may read an
earlier one's contribution in ctx["contribs"] and its "busy" flag): the map curve profile (curves), the camera-only
curve (camera_curves), the exit plan (exits), the merge (merge), the speed limit (speed_limit), the stop slowdown
(stops). Test mode builds them with presses=False: computed and logged, never commanded / never taken as an anchor.
With the limit follow it also keeps mapdOut's own limit on a "possible" fix (ctx["slc_possible"], limits.map_possible):
the first fix of a new match, when slcState is not "valid" yet.
The private SubMaster also reads userBookmark (the onroad Bookmark button, for the speed control's tag line; only
while the speed control runs this drive): ctx["bookmark"] is True on a frame with a new one.
Stage 3: with the gas auto-resume it also reads selfdriveState (ctx["engageable"]: openpilot has no no-entry now; False
when not received) and radarState (its creep guard), and the map view (its map guard).
"""
import math
import time

from openpilot.starpilot.car.mazda.speed.camera import CameraCurve, CameraVeto, model_path
from openpilot.starpilot.car.mazda.speed.curve_plan import CurvePlan
from openpilot.starpilot.car.mazda.speed.view_age import GONE, ViewClock

VIEW_EVERY = 10               # frames (10 Hz) between two reads of the map view file
MODEL_DT = 0.05
SLC_STALE = 0.5               # s: an older slcState is no limit reading


def model_path_y(model):
  """(xs, ys) of the model's path (y right positive), or None."""
  try:
    xs, ys = list(model.position.x), list(model.position.y)
  except AttributeError:
    return None
  n = min(len(xs), len(ys))
  if n < 2 or not all(math.isfinite(v) for v in xs[:n] + ys[:n]):
    return None
  return xs[:n], ys[:n]


def radar_lead(radar):
  """The lead radard publishes (on this car from the camera's leads), or None."""
  try:
    lead = radar.leadOne
    out = {"status": bool(lead.status), "v": float(lead.vLead), "vrel": float(lead.vRel), "d": float(lead.dRel)}
  except AttributeError:
    return None
  try:
    prob = float(lead.modelProb)
    out["prob"] = prob if math.isfinite(prob) else 0.0
  except (AttributeError, TypeError, ValueError):
    pass
  return out if all(math.isfinite(out[k]) for k in ("v", "vrel", "d")) else None


class Sources:
  """modelV2 through a private SubMaster (created only when a contributor needs it; card's own subscriptions are
  unchanged) and the map view file. Fills the ctx every contributor reads."""

  def __init__(self, want_model=True, want_view=True, sm=None, read_view=None, clock=time.monotonic, map_presses=True,
               want_slc=False, want_radar=False, want_bookmark=False, want_selfdrive=False):
    self.want_model, self.want_view = want_model, want_view
    self.want_slc, self.want_radar = want_slc, want_radar
    self.want_bookmark = want_bookmark
    self.want_selfdrive = want_selfdrive
    self.map_presses = map_presses   # the camera only steps aside for map curves that are really pressed for
    self.sm = sm
    services = (["modelV2"] if want_model else []) + (["slcState", "mapdOut"] if want_slc else []) + \
               (["radarState"] if want_radar else []) + (["userBookmark"] if want_bookmark else []) + \
               (["selfdriveState"] if want_selfdrive else [])
    if self.sm is None and services:
      from openpilot.cereal import messaging
      self.sm = messaging.SubMaster(services)
    if read_view is None:
      from openpilot.starpilot.car.mazda.speed.map_view import read_view
    self.read_view = read_view
    self.clock = clock
    self.frame = 0
    self.view = None
    self.path = None
    self.model_t = -1e9
    self.path_y = None
    self.lanes = None
    self.slc = None
    self.preview = None
    self.possible = None
    self.lead = None
    self.engageable = False
    self.veto = CameraVeto(MODEL_DT)
    self.view_clock = ViewClock()

  def context(self, d):
    """d: the caller's per-frame dict (v_ego, blinker, lkas_blocked, ...). Returns the ctx for contributors."""
    now = self.clock()
    self.frame += 1
    ctx = dict(d)
    ctx["now"] = now
    model_new = False
    bookmark = False
    if self.sm is not None:
      self.sm.update(0)
      if self.want_model and self.sm.updated["modelV2"]:
        model = self.sm["modelV2"]
        self.path = model_path(model)
        self.path_y = model_path_y(model)
        from openpilot.starpilot.car.mazda.speed.exit_plan import model_lanes
        self.lanes = model_lanes(model)
        self.model_t = now
        model_new = True
      if self.want_slc and self.sm.updated["slcState"]:
        from openpilot.starpilot.car.mazda.speed.limits import reading
        limit, upcoming = reading(self.sm["slcState"]) if self.sm.valid["slcState"] else (None, 0.0)
        self.slc = {"limit": limit, "upcoming": upcoming, "t": now}
      if self.want_slc and self.sm.updated["mapdOut"]:
        from openpilot.starpilot.car.mazda.speed.limits import map_possible, map_preview
        pv = map_preview(self.sm["mapdOut"], bool(self.sm.valid["mapdOut"]))
        self.preview = None if pv is None else dict(pv, t=now)
        poss = map_possible(self.sm["mapdOut"], bool(self.sm.valid["mapdOut"]))
        self.possible = None if poss is None else {"limit": poss, "t": now}
      if self.want_radar and self.sm.updated["radarState"]:
        lead = radar_lead(self.sm["radarState"])
        self.lead = None if lead is None else dict(lead, t=now)
      if self.want_bookmark and self.sm.updated["userBookmark"]:
        bookmark = True
      if self.want_selfdrive:
        self.engageable = bool(self.sm.seen["selfdriveState"] and self.sm.alive["selfdriveState"] and
                               getattr(self.sm["selfdriveState"], "engageable", False))
    if self.want_view and self.frame % VIEW_EVERY == 1:
      self.view = self.read_view()
    ctx.update(view=self.view, path=self.path, model_new=model_new, model_age=now - self.model_t, bookmark=bookmark,
               path_y=self.path_y, lanes=self.lanes, lead=self.lead, engageable=self.engageable,
               slc=self.slc if self.slc is not None and now - self.slc["t"] <= SLC_STALE else None,
               preview=self.preview if self.preview is not None and now - self.preview["t"] <= SLC_STALE else None,
               slc_possible=self.possible if self.possible is not None and now - self.possible["t"] <= SLC_STALE else None)
    # the one rule (view_age.py): the view is alive up to VIEW_ALIVE after its GPS message (a 1 Hz GPS included)
    live = self.view_clock.update(self.view, now) if self.want_view else GONE
    view_ok = live != GONE
    ctx["view_live"], ctx["view_new"] = live, bool(self.want_view and self.view_clock.new)
    ctx["gps_ok"] = view_ok if self.want_view else True
    ctx["no_gps"] = bool(self.want_view and not view_ok)   # F5: no GPS position (the camera-only curve is more careful)
    ctx["veto"] = self.veto
    self.veto.update(ctx)
    ctx["map_covers_camera"] = bool(self.map_presses and view_ok and not self.view.get("held") and self._covers(ctx))
    return ctx

  def _covers(self, ctx):
    v = ctx.get("v_ego", 0.0)
    driven = max(v, 0.0) * max(ctx["now"] - self.view["t"], 0.0)
    far = v * 2.5
    for c in self.view.get("curves", []):
      if self.veto.vetoed(c):
        continue
      ds = [p[0] - driven for p in c["pts"]]
      if ds and min(ds) <= far and max(ds) >= -20.0:
        return True
    return False


def build(cfg, CP, log=None, memory=None, metric=False, auto_resume=False):
  """(contributors, sources) for this drive. auto_resume (Stage 3): the gas auto-resume runs this drive (it has no
  contributor; it needs the view, the radar lead and selfdriveState)."""
  out = []
  if cfg.active("curves"):
    out.append(CurvePlan(cfg, presses=cfg.presses("curves"), memory=memory))
  elif cfg.active("merge"):
    out.append(CurvePlan(cfg, presses=False))   # map curves off: a log-only curve plan, the merge's curve guard
  if cfg.active("camera_curves"):
    out.append(CameraCurve(cfg, presses=cfg.presses("camera_curves")))
  if cfg.active("exits"):
    from openpilot.starpilot.car.mazda.speed.exit_plan import ExitPlan
    out.append(ExitPlan(cfg, presses=cfg.presses("exits"), metric=metric))
  if cfg.active("merge"):
    from openpilot.starpilot.car.mazda.speed.merge import Merge
    out.append(Merge(cfg, presses=cfg.presses("merge"), metric=metric))
  if cfg.active("speed_limit"):
    from openpilot.starpilot.car.mazda.speed.limits import SpeedLimit
    out.append(SpeedLimit(cfg, presses=cfg.presses("speed_limit"), metric=metric))
  if cfg.active("stops"):
    from openpilot.starpilot.car.mazda.speed.stops import StopSlowdown
    out.append(StopSlowdown(cfg, presses=cfg.presses("stops"), metric=metric))
  if not out and not auto_resume and not cfg.tag_on:
    return [], None
  return out, Sources(want_model=bool(out), want_view=cfg.needs_view, map_presses=cfg.presses("curves"),
                      want_slc=any(cfg.active(n) for n in ("speed_limit", "merge", "exits")),
                      want_radar=cfg.active("merge") or cfg.active("stops") or auto_resume, want_bookmark=True,
                      want_selfdrive=auto_resume)
