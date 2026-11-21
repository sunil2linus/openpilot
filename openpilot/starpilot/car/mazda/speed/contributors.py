"""The planner's contributors for this drive, chosen from the saved config, and the inputs they share (Sources).

Each contributor is a callable(ctx) -> planner.Contribution or None. Stage 2: the map curve profile (curves) and the
camera-only curve (camera_curves). Test mode builds them with presses=False: computed and logged, never commanded.
"""
import time

from openpilot.starpilot.car.mazda.speed.camera import CameraCurve, CameraVeto, model_path
from openpilot.starpilot.car.mazda.speed.curve_plan import CurvePlan, VIEW_STALE

VIEW_EVERY = 10               # frames (10 Hz) between two reads of the map view file
MODEL_DT = 0.05


class Sources:
  """modelV2 through a private SubMaster (created only when a contributor needs it; card's own subscriptions are
  unchanged) and the map view file. Fills the ctx every contributor reads."""

  def __init__(self, want_model=True, want_view=True, sm=None, read_view=None, clock=time.monotonic):
    self.want_model, self.want_view = want_model, want_view
    self.sm = sm
    if self.sm is None and want_model:
      from openpilot.cereal import messaging
      self.sm = messaging.SubMaster(["modelV2"])
    if read_view is None:
      from openpilot.starpilot.car.mazda.speed.map_view import read_view
    self.read_view = read_view
    self.clock = clock
    self.frame = 0
    self.view = None
    self.path = None
    self.model_t = -1e9
    self.veto = CameraVeto(MODEL_DT)

  def context(self, d):
    """d: the caller's per-frame dict (v_ego, blinker, lkas_blocked, ...). Returns the ctx for contributors."""
    now = self.clock()
    self.frame += 1
    ctx = dict(d)
    ctx["now"] = now
    model_new = False
    if self.sm is not None:
      self.sm.update(0)
      if self.sm.updated["modelV2"]:
        self.path = model_path(self.sm["modelV2"])
        self.model_t = now
        model_new = True
    if self.want_view and self.frame % VIEW_EVERY == 1:
      self.view = self.read_view()
    ctx.update(view=self.view, path=self.path, model_new=model_new, model_age=now - self.model_t)
    view_ok = self.view is not None and self.view.get("gps") == "ok" and now - self.view.get("t", -1e9) <= VIEW_STALE
    ctx["gps_ok"] = view_ok if self.want_view else True
    ctx["veto"] = self.veto
    self.veto.update(ctx)
    ctx["map_covers_camera"] = bool(view_ok and not self.view.get("held") and self._covers(ctx))
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


def build(cfg, CP, log=None):
  """(contributors, sources) for this drive."""
  out = []
  if cfg.active("curves"):
    out.append(CurvePlan(cfg, presses=cfg.presses("curves")))
  if cfg.active("camera_curves"):
    out.append(CameraCurve(cfg, presses=cfg.presses("camera_curves")))
  if not out:
    return [], None
  return out, Sources(want_model=True, want_view=cfg.active("curves"))
