"""
On-screen notices of the Mazda speed control (part H), the card side. No capnp change: when a notice fires, card
writes one tiny JSON file on tmpfs (NOTICE_PATH, atomic tmp + rename)

    {"v": 1, "sid": writer session, "seq": n, "kind": ..., "text1": ..., "text2": ..., "sound": bool,
     "until": monotonic s by which it must be shown}

and a small hook in selfdrived (notice_alert.py) turns each new seq into an ordinary Alert (low priority: every
stock alert wins), so the text and the sound travel in selfdriveState as always. Never a position.

Rules (Notices): each notice at most once per its event (a curve, an exit, a stop episode, a limit change, a merge
link, a drive), at least GAP between two notices (WARN_GAP for the warnings); a notice that falls inside the gap
waits in one pending slot (a warning takes the slot over an info) for at most PENDING_MAX. Only the stop warnings
fire in Test mode too (they warn without pressing); the rest only when our set really changes.

The texts (plain, short; old fork wording where it had one):
  "Curve ahead — slowing to NN"                  a map / camera curve starts to lower the set (once per curve)
  "Exit ramp — slowing"                          the exit plan starts to lower the set (once per exit)
  "Exit ramp — brake for the curve"              the exit plan cannot reach the ramp's speed inside its bound
  "Stop ahead" / "Stopped car ahead"             the stop slowdown (stops.py), "Cruise may not stop · brake"
  "Speed limit NN — set to NN"                   the limit rule changes OUR set (no sound)
  "Merging — set speed high"                     the merge boost starts (once per ramp)
  "Mazda speed control: no GPS position"         once per drive
  "Curve maps not built here"                    once per drive (the view has no curve files for the area)
  "Car's camera reports a fault" / "Car's lane keep is off" / "Car's camera fault cleared"   (TI cars, the old
                                                 fork's MazdaCamNotice: CAM_LKAS ERR bits / LANE_LINES 0 at speed)
  "Tagged"                                       the onroad Bookmark button wrote a tag line (caller.py)
Sound (notice_sound on): the stop warnings, the exit brake warning and the camera fault; never the others.
"""
import json
import os
import time
import uuid

from openpilot.starpilot.car.mazda.speed import cruise_model as M

NOTICE_PATH = "/dev/shm/mazda_speed_notice.json"
VERSION = 1
GAP = 3.0                     # s between two notices
WARN_GAP = 1.0                # s before a warning (stop, brake) may follow another notice
PENDING_MAX = 2.0             # s a notice waits inside the gap
SHOW_WITHIN = 3.0             # s: selfdrived drops a notice not shown by then (a stock alert was up)
NO_GPS_AFTER = 15.0           # s without a GPS position while moving
NO_DATA_AFTER = 10.0          # s with a GPS position but no curve files here
MOVING = 5.0                  # m/s
CAMERA_CURVE_REARM = 20.0     # s: the camera curve has no id: one notice per lowering, at most this often
CAM_FAULT_REPEAT = 60.0       # s (old fork MazdaCamNotice)
CAM_FAULT_CLEAR = 2.0         # s
LINES_OFF_SPEED = 35 * M.MPH  # old carstate STOCK_LKAS_OFF_SPEED ...
LINES_OFF_TIME = 5.0          # ... and STOCK_LKAS_OFF_TIME

WARNINGS = frozenset(("stop_ahead", "stopped_car", "exit_brake", "cam_fault"))
TEXTS = {
  "curve": ("Curve ahead — slowing to {n}", ""),
  "exit": ("Exit ramp — slowing", ""),
  "exit_brake": ("Exit ramp — brake for the curve", "Cruise cannot slow enough"),
  "stop_ahead": ("Stop ahead", "Cruise may not stop · brake"),
  "stopped_car": ("Stopped car ahead", "Cruise may not stop · brake"),
  "limit": ("Speed limit {n} — set to {m}", ""),
  "limit_unknown": ("Speed limit unknown — set to {m}", ""),
  "merge": ("Merging — set speed high", ""),
  "no_gps": ("Mazda speed control: no GPS position", ""),
  "no_data": ("Curve maps not built here", ""),
  "cam_fault": ("Car's camera reports a fault", "Lane keep may be off · steering weaker"),
  "lines_off": ("Car's lane keep is off", "Steering is weaker in curves until it clears"),
  "cam_cleared": ("Car's camera fault cleared", ""),
  "tagged": ("Tagged", ""),
}
KINDS = frozenset(TEXTS)


class NoticeWriter:
  """Writes the notice file atomically; never raises. sid: one random session per drive (seq restarts with it)."""

  def __init__(self, path=NOTICE_PATH, clock=time.monotonic):
    self.path = path
    self.clock = clock
    self.sid = uuid.uuid4().hex[:12]
    self.seq = 0
    self.written = 0
    self.failed = 0

  def write(self, kind, text1, text2="", sound=False):
    self.seq += 1
    doc = {"v": VERSION, "sid": self.sid, "seq": self.seq, "kind": str(kind), "text1": str(text1)[:48],
           "text2": str(text2)[:64], "sound": bool(sound), "until": round(self.clock() + SHOW_WITHIN, 3)}
    tmp = self.path + ".tmp"
    try:
      with open(tmp, "w") as f:
        f.write(json.dumps(doc, separators=(",", ":")))
      os.replace(tmp, self.path)
      self.written += 1
      return True
    except (OSError, TypeError, ValueError):
      self.failed += 1
      return False


class CamNotice:
  """The old fork's MazdaCamNotice (starpilot_events.py, rounds P / U9): update(fault) -> "fault" / "lines" /
  "cleared" / None, fault 0 none, 1 the lane lines off (latched: LANE_LINES 0 for LINES_OFF_TIME above
  LINES_OFF_SPEED, cleared by the lines back as long), 2 the camera's ERR bits. Each kind again at most every
  CAM_FAULT_REPEAT; "cleared" once no fault of either kind is left for CAM_FAULT_CLEAR."""

  def __init__(self):
    self.t = None
    self.active = False
    self.shown = False
    self.last = None
    self.clear_s = 0.0
    self.l_active = False
    self.l_shown = False
    self.l_last = None
    self.l_clear = 0.0
    self.lines_off = False         # the latch (old carstate stock_lkas_off)
    self.lines_t = 0.0

  def lines(self, lane_lines, v, dt):
    """The lane-lines-off latch from CAM_LANEINFO LANE_LINES (0: the camera's LKAS is off)."""
    if lane_lines is None:
      return self.lines_off
    if v > LINES_OFF_SPEED:
      off = lane_lines == 0
      self.lines_t = self.lines_t + dt if off != self.lines_off else 0.0
      if self.lines_t >= LINES_OFF_TIME - 1e-6:
        self.lines_off, self.lines_t = off, 0.0
    else:
      self.lines_t = 0.0
    return self.lines_off

  def _lines_start(self, now):
    self.l_active, self.l_clear = True, 0.0
    self.l_shown = self.l_last is None or now - self.l_last >= CAM_FAULT_REPEAT - 1e-6
    if self.l_shown:
      self.l_last = now
      return "lines"
    return None

  def update(self, fault, now):
    dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
    self.t = now
    if fault == 2:
      self.clear_s = self.l_clear = 0.0
      if not self.active:
        self.active = True
        self.shown = self.last is None or now - self.last >= CAM_FAULT_REPEAT - 1e-6
        if self.shown:
          self.last = now
          return "fault"
      return None
    if self.active:
      self.clear_s += dt
      if self.clear_s >= CAM_FAULT_CLEAR - 1e-6:
        self.active = False
        shown, self.shown = self.shown, False
        if fault == 1:
          out = None if self.l_active else self._lines_start(now)
          self.l_shown = self.l_shown or shown
          return out
        l_shown = self.l_active and self.l_shown
        self.l_active = self.l_shown = False
        if shown or l_shown:
          return "cleared"
      return None
    if fault == 1:
      self.l_clear = 0.0
      return None if self.l_active else self._lines_start(now)
    if self.l_active:
      self.l_clear += dt
      if self.l_clear >= CAM_FAULT_CLEAR - 1e-6:
        self.l_active = False
        shown, self.l_shown = self.l_shown, False
        if shown:
          return "cleared"
    return None


class Notices:
  """Decides when a notice fires (from the plan, the planner's anchor changes and the contributors' states) and sends
  it through the writer with the rate limits. update() runs every card frame: cheap."""

  def __init__(self, cfg, writer=None, metric=False, ti=False, log=None):
    self.cfg = cfg
    self.sound = bool(cfg.notice_sound)
    self.writer = writer if writer is not None else NoticeWriter()
    self.metric = bool(metric)
    self.unit = M.KPH if self.metric else M.MPH
    self.ti = bool(ti)
    self.log = log
    self.last_t = -1e9
    self.pending = None            # (kind, text1, text2, sound, since)
    self.done = set()              # event keys already noticed
    self.prev_reason = ""
    self.camera_t = -1e9
    self.no_gps_t = 0.0
    self.no_data_t = 0.0
    self.t = None
    self.cam = CamNotice()
    self.sent = []                 # (t, kind) for the tests / the log

  def _n(self, mps):
    return int(round(mps / self.unit))

  def fire(self, kind, now, key=None, **fmt):
    """Queue a notice; key: its event (at most once per key). Returns True when it is sent or queued."""
    if key is not None:
      if key in self.done:
        return False
      self.done.add(key)
    t1, t2 = TEXTS[kind]
    t1, t2 = t1.format(**fmt), t2.format(**fmt)
    sound = self.sound and kind in WARNINGS
    gap = WARN_GAP if kind in WARNINGS else GAP
    if now - self.last_t >= gap - 1e-6:
      self._send(kind, t1, t2, sound, now)
      return True
    if self.pending is None or kind in WARNINGS or self.pending[0] not in WARNINGS:
      self.pending = (kind, t1, t2, sound, now)
    return True

  def _send(self, kind, t1, t2, sound, now):
    self.last_t = now
    self.pending = None
    ok = self.writer.write(kind, t1, t2, sound)
    self.sent.append((now, kind))
    if self.log is not None:
      self.log.log("notice", notice=kind, text=t1, sound=sound, written=ok)

  def _flush(self, now):
    p = self.pending
    if p is None:
      return
    if now - p[4] > PENDING_MAX:
      self.pending = None
      return
    gap = WARN_GAP if p[0] in WARNINGS else GAP
    if now - self.last_t >= gap - 1e-6:
      self._send(p[0], p[1], p[2], p[3], now)

  def update(self, now, d, ctx, plan, contributors, changes, want_view=False):
    dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
    self.t = now
    self._flush(now)
    by_name = {type(c).__name__: c for c in contributors}
    contribs = ctx.get("contribs") or {}
    reason = plan.reason
    # ---- curves (only when our set really goes down for one) ----
    if reason in ("map_curve", "camera_curve") and reason != self.prev_reason:
      if reason == "map_curve":
        c = contribs.get("map_curve")
        plan_c = by_name.get("CurvePlan")
        vmin = getattr(plan_c, "last_vmin", None)
        key = "curve:" + str(getattr(c, "episode", None))
      else:
        cam = by_name.get("CameraCurve")
        vmin = getattr(cam, "last_vmin", None)
        key = None if now - self.camera_t < CAMERA_CURVE_REARM else f"cam:{int(now)}"
        if key is not None:
          self.camera_t = now
      if key is not None and vmin is not None:
        self.fire("curve", now, key=key, n=self._n(vmin + M.DASH_OFFSET))
    # ---- exits ----
    ex = by_name.get("ExitPlan")
    exit_c = contribs.get("exit")
    if reason == "exit" and self.prev_reason != "exit" and exit_c is not None:
      self.fire("exit", now, key="exit:" + str(exit_c.episode))
    if ex is not None and ex.presses and getattr(ex, "brake_needed", False) and exit_c is not None:
      self.fire("exit_brake", now, key="exit_brake:" + str(exit_c.episode))
    # ---- stops (also in Test mode: a warning without pressing) ----
    st = by_name.get("StopSlowdown")
    if st is not None and st.notice in ("stop_ahead", "stopped_car"):
      ep = st.active["id"] if st.active is not None else ""
      self.fire(st.notice, now, key=st.notice + ":" + ep)
    # ---- our anchor changes: the limit, the merge ----
    for why, before, after in changes:
      if after is None:
        continue
      if why == "limit":
        lim = getattr(by_name.get("SpeedLimit"), "steady", None)
        if lim is not None:
          self.fire("limit", now, key=f"limit:{lim}:{self._n(after)}", n=int(lim), m=self._n(after))
      elif why == "fallback":
        self.fire("limit_unknown", now, key=f"fallback:{self._n(after)}", m=self._n(after))
      elif why == "merge" and before is not None and after > before:
        mg = by_name.get("Merge")
        self.fire("merge", now, key="merge:" + str(getattr(mg, "link", None)))
    # ---- once per drive: no GPS position / no curve files here ----
    if want_view:
      moving = d.get("v_ego", 0.0) >= MOVING
      view = ctx.get("view")
      self.no_gps_t = self.no_gps_t + dt if (moving and ctx.get("no_gps")) else 0.0
      if self.no_gps_t >= NO_GPS_AFTER:
        self.fire("no_gps", now, key="no_gps")
      no_data = bool(moving and ctx.get("gps_ok") and view is not None and view.get("data") is False)
      self.no_data_t = self.no_data_t + dt if no_data else 0.0
      if self.no_data_t >= NO_DATA_AFTER:
        self.fire("no_data", now, key="no_data")
    # ---- the car's camera (TI cars) ----
    if self.ti:
      lines = self.cam.lines(d.get("lane_lines"), d.get("v_ego", 0.0), dt)
      fault = 2 if d.get("cam_err") else (1 if lines else 0)
      got = self.cam.update(fault, now)
      if got == "fault":
        self.fire("cam_fault", now)
      elif got == "lines":
        self.fire("lines_off", now)
      elif got == "cleared":
        self.fire("cam_cleared", now)
    self.prev_reason = reason

  def tagged(self, now):
    self.fire("tagged", now)
