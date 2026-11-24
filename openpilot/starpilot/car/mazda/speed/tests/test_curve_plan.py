"""The curve contributor (profile points -> one target) and the camera's two jobs (veto, camera-only curve)."""
import math

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed import view_age as VA
from openpilot.starpilot.car.mazda.speed.camera import CameraCurve, CameraVeto, VETO_CONFIRM, REARM_CONFIRM, VETO_MAX_FLIPS
from openpilot.starpilot.car.mazda.speed.curve_plan import CurvePlan, KEEP_PAST, point_speed

MPH = M.MPH
CFG = C.Config(enabled=True, curves="on", camera_curves="on")


def view(pts, t=0.0, ramp=False, adv=0.0, trust=0, cid="c1", **kw):
  v = {"v": 1, "t": t, "gps": "ok", "trust": trust, "held": False, "curves": [{"id": cid, "pts": pts, "ramp": ramp, "adv": adv}]}
  v.update(kw)
  return v


def target(plan, vw, now=0.0, v=25.0, **ctx):
  return plan({"view": vw, "now": now, "v_ego": v, **ctx}).target


def test_tight_entry_governs_then_the_target_rises_by_itself():
  plan = CurvePlan(CFG)
  pts = [[300.0, 72.0], [360.0, 92.0], [430.0, 92.0]]
  v = 25.0
  t0 = target(plan, view(pts), v=v)
  assert abs(t0 - M.envelope(math.sqrt(2.2 * 72), 300.0, v)) < 1e-6
  def moved(d):
    return [[p[0] - d, p[1]] for p in pts]
  # 250 m on: the target falls toward the pinch's own speed
  t_near = target(plan, view(moved(250.0)), v=v)
  assert t_near < t0 and t_near >= math.sqrt(2.2 * 72) - 1e-6
  # 15 m past the pinch: still governs (GPS lag); KEEP_PAST + 7 m past: only the wider points remain
  v2 = math.sqrt(2.2 * 72)
  assert abs(target(plan, view(moved(315.0)), v=v2) - v2) < 1e-6
  wider = M.envelope(math.sqrt(2.2 * 92), 60.0 - KEEP_PAST - 7.0, v2)
  assert abs(target(plan, view(moved(300.0 + KEEP_PAST + 7.0)), v=v2) - wider) < 1e-6 and wider > v2
  # the view's age moves the points too (0.5 s at v2)
  assert abs(target(plan, view(moved(300.0 + KEEP_PAST + 7.0 - 0.5 * v2)), now=0.5, v=v2) - wider) < 1e-6


def test_mid_curve_step_up_only_with_gps_trust():
  plan = CurvePlan(CFG)
  pts = [[-30.0, 72.0], [40.0, 92.0]]
  v = math.sqrt(2.2 * 72)
  held = target(plan, view([[-5.0, 72.0], [40.0, 92.0]], trust=1), v=v)
  assert abs(held - v) < 1e-6
  assert abs(target(plan, view(pts, trust=1), v=v) - v) < 1e-6           # shaky: holds the lowest
  plan2 = CurvePlan(CFG)
  target(plan2, view([[-5.0, 72.0], [40.0, 92.0]], trust=0), v=v)
  assert target(plan2, view(pts, trust=0), v=v) > v + 1.0                # trusted: steps up
  plan3 = CurvePlan(CFG)
  target(plan3, view([[-5.0, 72.0], [40.0, 92.0]], trust=1), v=v)
  assert target(plan3, view(pts, trust=1, start_ok=True), v=v) > v + 1.0  # start-of-drive trust on a link


def test_ti_only_lateral_only_when_lkas_was_blocked_before_the_curve():
  plan = CurvePlan(CFG)
  r = 150.0
  t_block = target(plan, view([[0.5, r]]), v=10.0, lkas_blocked=True)
  assert abs(t_block - math.sqrt(1.8 * r)) < 1e-6
  plan = CurvePlan(CFG)
  assert abs(target(plan, view([[0.5, r]]), v=10.0) - math.sqrt(2.2 * r)) < 1e-6
  # blocked only once inside (the car slowed below the cut-off in the curve): no re-plan lower
  assert abs(target(plan, view([[-3.0, r]]), v=10.0, lkas_blocked=True) - math.sqrt(2.2 * r)) < 1e-6


def test_offset_cap_advisory_and_ramp_lateral():
  r = 100.0
  base = math.sqrt(2.2 * r)
  assert point_speed(r, 2.2) == base
  assert point_speed(r, 2.2, offset=5 * MPH, cap_lat=3.0) == min(base + 5 * MPH, math.sqrt(3.0 * r))
  assert point_speed(r, 2.2, offset=5 * MPH, cap_lat=2.3) == math.sqrt(2.3 * r)
  assert point_speed(r, 2.2, advisory=8.0) == 8.0 + 10 * MPH
  plan = CurvePlan(CFG)
  assert abs(target(plan, view([[0.1, r]], ramp=True), v=5.0) - math.sqrt(2.6 * r)) < 1e-6
  assert CurvePlan(CFG).offset == 0.0                  # (no curve speed offset until the per-curve memory exists)


def test_no_map_stale_held_or_no_gps_gives_no_target():
  plan = CurvePlan(CFG)
  assert target(plan, None) is None
  # gone: older than VIEW_ALIVE after its GPS message (view_age.py; the old rule was 1.0 s after the stamp)
  assert target(plan, view([[100.0, 80.0]], t=0.0), now=VA.GPS_LAG + VA.VIEW_ALIVE + 0.01) is None
  assert target(CurvePlan(CFG), view([[100.0, 80.0]], t=0.0), now=1.85) is not None   # a 1 Hz GPS's view: alive
  assert target(plan, view([[100.0, 80.0]], held=True)) is None
  assert target(plan, view([[100.0, 80.0]], gps="none")) is None


def test_a_view_dropping_out_mid_curve_follows_the_last_good_one():
  plan = CurvePlan(CFG)
  v = 12.0
  good = view([[-10.0, 72.0], [60.0, 92.0]], t=0.0)
  t_good = target(plan, good, now=0.0, v=v)
  assert abs(t_good - math.sqrt(2.2 * 72)) < 1e-6
  # the GPS goes away 0.5 s later: the same curve, moved by the distance driven (held: no step up, trust not ok)
  for bad in (None, view([], gps="none"), view([[5.0, 30.0]], held=True), dict(view([]), paused=True)):
    assert abs(target(plan, bad, now=0.5, v=v) - t_good) < 1e-6
  # ... and after DEAD_RECKON it is gone (no new curves from an old view)
  assert target(plan, None, now=11.0, v=v) is None


def test_lkas_floor_flag_follows_the_curves_own_speed():
  plan = CurvePlan(CFG)
  assert plan({"view": view([[300.0, 300.0]]), "now": 0.0, "v_ego": 30.0}).lkas_floor
  assert not plan({"view": view([[300.0, 50.0]]), "now": 0.0, "v_ego": 30.0}).lkas_floor


def test_test_mode_marks_the_contribution():
  cfg = C.Config(enabled=True, curves="test")
  c = CurvePlan(cfg, presses=cfg.presses("curves"))({"view": view([[100.0, 80.0]]), "now": 0.0, "v_ego": 25.0})
  assert c.target is not None and not c.presses


# ---------------- camera ----------------

def path(kappa, n=33, v=25.0):
  xs = [v * 10.0 * (i / (n - 1)) ** 2 for i in range(n)]
  return xs, [kappa] * n


def veto_ctx(vw, p, now, v=25.0, blinker=False):
  return {"view": vw, "path": p, "model_new": True, "now": now, "v_ego": v, "blinker": blinker}


def test_veto_a_curve_the_camera_sees_straight_then_rearm_and_flip_limit():
  vw = view([[40.0, 80.0], [60.0, 80.0]])
  veto = CameraVeto(0.05)
  frames = int(VETO_CONFIRM / 0.05)
  for _ in range(frames):
    assert not veto.vetoed(vw["curves"][0])
    veto.update(veto_ctx(vw, path(0.0), 0.0))
  assert veto.vetoed(vw["curves"][0])
  # its curve with a curved path: back after REARM_CONFIRM
  for _ in range(int(REARM_CONFIRM / 0.05)):
    veto.update(veto_ctx(vw, path(1 / 80.0), 0.0))
  assert not veto.vetoed(vw["curves"][0])
  for _ in range(VETO_MAX_FLIPS):
    for _ in range(frames):
      veto.update(veto_ctx(vw, path(0.0), 0.0))
    for _ in range(int(REARM_CONFIRM / 0.05)):
      veto.update(veto_ctx(vw, path(1 / 80.0), 0.0))
  for _ in range(3 * frames):
    veto.update(veto_ctx(vw, path(0.0), 0.0))
  assert not veto.vetoed(vw["curves"][0])               # flipped too often: never vetoed again
  # a blinker or tight points beyond the reliable view: no verdict
  veto = CameraVeto(0.05)
  for _ in range(2 * frames):
    veto.update(veto_ctx(vw, path(0.0), 0.0, blinker=True))
    veto.update(veto_ctx(view([[200.0, 80.0]], cid="far"), path(0.0), 0.0))
  assert not veto.vetoed(vw["curves"][0]) and not veto.vetoed({"id": "far"})


def test_vetoed_curve_is_not_planned():
  vw = view([[40.0, 80.0]])
  veto = CameraVeto(0.05)
  for _ in range(40):
    veto.update(veto_ctx(vw, path(0.0), 0.0))
  assert target(CurvePlan(CFG), vw, veto=veto) is None


def test_camera_curve_needs_a_steady_view_and_steps_aside_for_the_map():
  cam = CameraCurve(CFG, dt=0.01)
  ctx = {"path": path(1 / 100.0), "v_ego": 25.0, "model_age": 0.0, "gps_ok": True}
  outs = [cam(dict(ctx)).target for _ in range(60)]
  assert outs[0] is None and outs[-1] is not None
  expect = min(M.envelope(math.sqrt(2.0 / 0.01), x, 25.0) for x in path(0.01)[0] if 12.5 <= x <= 50.0)
  assert abs(outs[-1] - expect) < 1e-6
  # one straight frame among curved ones: the least demanding of the window wins (no slowdown from a flicker)
  cam = CameraCurve(CFG, dt=0.01)
  for k in range(60):
    t = cam(dict(ctx, path=path(1 / 100.0) if k == 30 else path(0.0))).target
  assert t is None
  assert CameraCurve(CFG)(dict(ctx, map_covers_camera=True)).target is None
  assert CameraCurve(CFG).raw_target(dict(ctx, v_ego=8.0)) is None
  assert CameraCurve(CFG).raw_target(dict(ctx, blinker=True)) is None
  careful = CameraCurve(CFG).raw_target(dict(ctx, gps_ok=False))
  assert careful < CameraCurve(CFG).raw_target(ctx)


def test_sources_camera_steps_aside_only_for_pressed_map_curves():
  from openpilot.starpilot.car.mazda.speed.contributors import Sources
  vw = view([[30.0, 80.0], [80.0, 80.0]])
  for presses, covers in ((True, True), (False, False)):
    src = Sources(want_model=False, sm=None, read_view=lambda: vw, clock=lambda: 0.0, map_presses=presses)
    ctx = src.context({"v_ego": 25.0})
    assert ctx["map_covers_camera"] is covers and ctx["gps_ok"]
  far = view([[400.0, 80.0]])
  src = Sources(want_model=False, sm=None, read_view=lambda: far, clock=lambda: 0.0)
  assert not src.context({"v_ego": 25.0})["map_covers_camera"]        # beyond the camera's range: the camera acts


def test_camera_curve_ignores_targets_far_above_v_ego():
  cam = CameraCurve(CFG, dt=0.01)
  ctx = {"path": path(1 / 500.0), "v_ego": 28 * MPH, "model_age": 0.0, "gps_ok": True}
  for _ in range(60):
    assert cam(dict(ctx)).target is None
  assert cam.last_vmin is None
  # tight curve near or below v_ego still activates after CAM_CONFIRM
  tight = dict(ctx, path=path(1 / 50.0))
  for _ in range(60):
    t = cam(dict(tight)).target
  assert t is not None
  # start-only gate: ~45 mph curve does not start at 30 mph, starts at 40 mph, then holds without
  # flip-flopping if traffic drops v_ego to 30 mph until the curve ends
  cam = CameraCurve(CFG, dt=0.01)
  mod_k = 2.0 / (45 * MPH) ** 2
  for _ in range(60):
    assert cam({"path": path(mod_k), "v_ego": 30 * MPH, "model_age": 0.0, "gps_ok": True}).target is None
  for _ in range(60):
    t = cam({"path": path(mod_k), "v_ego": 40 * MPH, "model_age": 0.0, "gps_ok": True}).target
  assert t is not None
  for _ in range(60):
    assert cam({"path": path(mod_k), "v_ego": 30 * MPH, "model_age": 0.0, "gps_ok": True}).target is not None
  assert cam({"path": path(0.0), "v_ego": 30 * MPH, "model_age": 0.0, "gps_ok": True}).target is None
  for _ in range(60):
    assert cam({"path": path(mod_k), "v_ego": 30 * MPH, "model_age": 0.0, "gps_ok": True}).target is None
