"""The hook card.py calls on a Mazda after CI.apply(): reads the car, runs the contributors, the planner and the
actuator, and returns the CAN frames to send (the given list unchanged when there is nothing to add).

With the master switch off (or nothing chosen, or not an eligible car) the first call stores `False` on card and
every later call returns the frames list untouched: the drive is the Stage 1 drive.
Any exception turns the feature off for the rest of the drive (steering is never affected).

Also here (part H / T): the on-screen notices (notices.py, only with `notices` on) and the tag line: on each new
userBookmark (the onroad Bookmark button, read through the private SubMaster) one "tag" line with a snapshot of the
speed control goes into the event log (never a position) and, with notices on, a short "Tagged" notice.

Tag only (v7 `tag_button` with the master switch, every other feature off): the speed control runs only for the tag
lines (the Tag button on the driving screen, ui/mazda_tag_button.py, sends the same userBookmark): no contributor,
no car profile, no notices, and it never presses (the command is always None).

Stage 3: the gas auto-resume (auto_resume.py, only with `auto_resume` on AND its panda bit in this drive's safety param,
opendbc/car/mazda/speed_flags.py): its logic runs every frame next to the planner; its RES goes out through the same
actuator (its RES path for the cruise off) and the same frame builder, never through a second sender.
"""
import time
from collections import deque

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed.actuator import Actuator, ActInputs, RESUME, SET_PLUS
from openpilot.starpilot.car.mazda.speed.planner import Planner, PlanInputs

CRZ_BTNS = 0x09D
PLAN_LOG_GAP = 0.2            # s: a changed plan is logged at most this often
GPS_LOG_STALE = 5.0           # s: an older map view is logged as "stale" (mazda_mapviewd stopped writing)
TAG_PRESSES = 10.0            # s of our own button presses in a tag's snapshot


def _cancel_frame(f):
  try:
    return f[0] == CRZ_BTNS and bool(bytes(f[1])[0] & 0x01)
  except (IndexError, TypeError, ValueError):
    return False


def _gear_drive(gear):
  from opendbc.car import structs
  return gear == structs.CarState.GearShifter.drive    # (as card.py compares it)


def _plain(v):
  if isinstance(v, tuple):
    return [x if isinstance(x, (str, int, float, bool, type(None))) else str(x) for x in v]
  return v if isinstance(v, (str, int, float, bool, type(None))) else str(v)


def eligible(CP) -> bool:
  if CP is None:
    return False
  try:
    from opendbc.car.mazda.values import MazdaFlags
    return bool(CP.brand == "mazda" and not CP.notCar and int(CP.flags) & int(MazdaFlags.GEN1) and CP.pcmCruise and
                not CP.openpilotLongitudinalControl and not CP.passive)
  except AttributeError as e:
    from openpilot.starpilot.car.mazda.speed.events import schema_miss
    schema_miss(None, "carParams (brand/notCar/flags/pcmCruise/openpilotLongitudinalControl/passive)", e)
    return False
  except (TypeError, ValueError):
    return False


class SpeedControl:
  def __init__(self, CP, cfg, params, is_metric, log=None, profile=None, contributors=None, sources=None, memory=None,
               notices=None, auto_resume=None):
    self.CP = CP
    self.cfg = cfg
    self.auto_resume = auto_resume   # (Stage 3) auto_resume.AutoResumeLogic, or None
    self.tag_only = bool(cfg.tag_on and not cfg.any_active)   # (v7) only the tag lines: never a press
    self.ar_road = None
    self.ar_key = None
    self.ar_key_t = 0.0
    if auto_resume is not None:
      from openpilot.starpilot.car.mazda.speed.auto_resume import ViewRoad
      self.ar_road = ViewRoad()
    self.params = params
    self.notices = notices           # notices.Notices (only with notices on), else None
    self.t0 = None                   # monotonic s of the first frame
    self.presses = deque(maxlen=64)  # (monotonic s, button) of our own presses, for the tag snapshot
    self.tags = 0
    self.metric = bool(is_metric)
    self.log = log
    self.profile = profile
    lat = profile.latencies if profile is not None else ()
    up, down = (profile.hold_start(1)[0], profile.hold_start(0)[0]) if profile is not None else (1, 0)
    self.actuator = Actuator(self.metric, latencies=lat, overrun_up=up, overrun_down=down)
    self.planner = Planner(self.metric)
    self.contributors = list(contributors or [])
    self.sources = sources
    self.last_reason = ""
    self.memory = memory
    self.events = []                 # last frame's actuator events (the planner reads them one frame later)
    self.last_plan = None
    self.last_plan_t = 0.0
    self.holds_seen = 0
    self.cruise_was = False
    self.packer = None
    self.gps_key = None
    self.missing = set()             # cereal fields found missing this drive (one "schema" line each)
    if profile is not None:
      profile.note_drive()
    self._log("start", config=C.encode(cfg).decode(), metric=self.metric,
              profile=profile.summary() if profile is not None else None)

  def _log(self, kind, **kw):
    if self.log is not None:
      self.log.log(kind, **kw)

  def warn(self, what, err=None):
    """A carState / carControl field we read is not in this schema: one "schema" event-log line per field per drive."""
    if what in self.missing:
      return
    self.missing.add(what)
    self._log("schema", field=what, error=str(err)[:160])
    from openpilot.starpilot.car.mazda.speed.events import schema_miss
    schema_miss(None, what, err)

  def _cs(self, CS, name, default):
    """A carState field (7.0's schema has all of these); missing -> the default, reported once (never silent)."""
    try:
      return getattr(CS, name)
    except AttributeError as e:
      self.warn("carState." + name, e)
      return default

  def inputs(self, card, CS, CC, can_sends):
    ocs = card.CI.CS      # (carstate's own Python attributes: CAN reads, not cereal fields)
    f = self._cs
    other = bool(CC.cruiseControl.cancel or CC.cruiseControl.resume or
                 any(f[0] == CRZ_BTNS for f in can_sends))
    return {"cruise_enabled": bool(CS.cruiseState.enabled), "main_on": bool(CS.cruiseState.available),
                "set_speed": float(CS.cruiseState.speed), "v_ego": float(CS.vEgo), "gas": bool(CS.gasPressed),
                "brake": bool(CS.brakePressed), "standstill": bool(CS.standstill or CS.cruiseState.standstill),
                "set_plus": bool(ocs.crz_set_plus), "set_minus": bool(ocs.crz_set_minus), "resume": bool(ocs.crz_resume),
                "cancel": bool(ocs.crz_cancel), "wheel_counter": int(ocs.crz_btns_counter), "other_button": other,
                "engaged": bool(CC.enabled), "lkas_blocked": bool(getattr(ocs, "lkas_blocked", False)),
                "blinker": bool(f(CS, "leftBlinker", False) or f(CS, "rightBlinker", False)),
                "left_blinker": bool(f(CS, "leftBlinker", False)), "right_blinker": bool(f(CS, "rightBlinker", False)),
                "radar_lead": bool(getattr(ocs, "radar_has_lead", False)),
                "lat_active": bool(CC.latActive), "torque": float(CC.actuators.torque),
                "steer_alert": str(CC.hudControl.visualAlert).endswith("steerRequired"),
                "angle": float(CS.steeringAngleDeg), "steering_pressed": bool(CS.steeringPressed),
                "stop_sign_raw": getattr(ocs, "dashboard_stop_sign", None), **self._camera(ocs),
                # (Stage 3) the gas auto-resume's inputs
                "steer_rate": float(f(CS, "steeringRateDeg", 0.0) or 0.0), "gear_drive": _gear_drive(f(CS, "gearShifter", None)),
                "door_open": bool(f(CS, "doorOpen", False)),
                "seatbelt_unlatched": bool(f(CS, "seatbeltUnlatched", False)),
                "can_valid": bool(f(CS, "canValid", False)),
                "set_allowed": bool(getattr(ocs, "crz_set_allowed", False)),
                "not_allowed": bool(getattr(ocs, "crz_not_allowed", False)),
                "op_cancel": bool(CC.cruiseControl.cancel), "op_resume": bool(CC.cruiseControl.resume),
                "cancel_sent": any(_cancel_frame(f) for f in can_sends)}

  @staticmethod
  def _camera(ocs):
    """The car camera's fault bits (CAM_LKAS ERR_BIT_1/2) and LANE_LINES (CAM_LANEINFO), as carstate keeps them."""
    try:
      cam, info = getattr(ocs, "cam_lkas", None), getattr(ocs, "cam_laneinfo", None)
      err = bool(cam is not None and (cam.get("ERR_BIT_1") == 1 or cam.get("ERR_BIT_2") == 1))
      lines = None if info is None or info.get("LANE_LINES") is None else int(info.get("LANE_LINES"))
      return {"cam_err": err, "lane_lines": lines}
    except (AttributeError, TypeError, ValueError):
      return {"cam_err": False, "lane_lines": None}

  def step(self, d, ctx=None, now=None):
    """d: the dict from inputs(); ctx: what contributors get (the car's other signals). Returns (button, counter)."""
    now = time.monotonic() if now is None else now
    if self.t0 is None:
      self.t0 = now
    d.setdefault("plan_reason", self.last_reason)
    if ctx is None:
      ctx = self.sources.context(d) if self.sources is not None else dict(d)
    else:
      ctx.update(d)
    ctx.update(anchor=self.planner.anchor, owner=self.planner.owner, ours_why=self.planner.ours_why,
               act_events=self.events, contribs={})
    if self.sources is not None and getattr(self.sources, "want_view", False):
      self._note_gps(ctx, now)
    contributions = []
    for c in self.contributors:
      got = c(ctx)
      if got is not None:
        contributions.append(got)
        ctx["contribs"][got.name] = got
    plan = self.planner.update(PlanInputs(v_ego=d["v_ego"], set_speed=d["set_speed"], cruise_enabled=d["cruise_enabled"],
                                          main_on=d["main_on"], events=self.events, contributions=contributions,
                                          metric=self.metric))
    for why, before, after in self.planner.changes:
      self._log("anchor", why=why, before=before, after=after)
    self.last_reason = plan.reason
    command = plan.command if d["engaged"] and not self.tag_only else None
    want_res = self._auto_resume(d, ctx, now)
    busy = bool(self.auto_resume is not None and self.auto_resume.busy and not want_res)
    act = self.actuator.update(ActInputs(cruise_enabled=d["cruise_enabled"], set_speed=d["set_speed"], v_ego=d["v_ego"],
                                         gas=d["gas"], brake=d["brake"], standstill=d["standstill"],
                                         set_plus=d["set_plus"], set_minus=d["set_minus"], resume=d["resume"],
                                         cancel=d["cancel"], wheel_counter=d["wheel_counter"], command=command,
                                         allow_lower=plan.allow_lower, other_button=d["other_button"] or busy,
                                         auto_resume=want_res))
    if want_res and act.button != RESUME:
      self.auto_resume.unsend()           # the actuator did not send it: not counted against the panda's budget
    self.events = act.events
    for e in act.events:
      self._log("act", event=e)
    if act.button is not None:
      self.presses.append((round(now, 2), act.button))
    states = {getattr(c, "__class__", type(c)).__name__: getattr(c, "last", None) for c in self.contributors
              if not isinstance(getattr(c, "last", None), tuple) or getattr(c, "last", (None,))[0] not in (None, "idle", "watch")}
    key = (plan.reason, None if plan.command is None else round(plan.command, 1), plan.dry_reason,
           None if plan.dry_command is None else round(plan.dry_command, 1),
           None if plan.dry_anchor is None else round(plan.dry_anchor, 1), self.planner.owner,
           tuple(sorted((k, str(v[0]) if isinstance(v, tuple) else "") for k, v in states.items() if k != "CurvePlan")))
    if key != self.last_plan and now - self.last_plan_t >= PLAN_LOG_GAP:
      self.last_plan, self.last_plan_t = key, now
      self._log("plan", reason=plan.reason, command=plan.command, target=plan.target, anchor=self.planner.anchor,
                owner=self.planner.owner, set=d["set_speed"], v=round(d["v_ego"], 2), test_reason=plan.dry_reason,
                test_command=plan.dry_command, test_anchor=plan.dry_anchor, test_anchor_kind=plan.dry_anchor_kind,
                states={k: _plain(v) for k, v in states.items() if k != "CurvePlan"})
    if self.notices is not None:
      self.notices.update(now, d, ctx, plan, self.contributors, self.planner.changes,
                          want_view=bool(self.sources is not None and getattr(self.sources, "want_view", False)))
    if ctx.get("bookmark"):
      self._tag(now, d, ctx, plan, states)
    self._learn(d["cruise_enabled"])
    return (act.button, act.counter) if act.button is not None else None

  def _auto_resume(self, d, ctx, now):
    """(Stage 3) one frame of the gas auto-resume logic. True: it wants its RES frame now."""
    ar = self.auto_resume
    if ar is None:
      return False
    from openpilot.starpilot.car.mazda.speed.actuator import frames
    from openpilot.starpilot.car.mazda.speed.auto_resume import ARMED, BUSY_AFTER, SENDING, STATE_NAMES, inputs_from
    act = self.actuator
    # the actuator has a press going: a hold, or a SET+/SET- frame within BUSY_AFTER
    busy = bool(act.hold is not None or act.frame - act.last_send < frames(BUSY_AFTER))
    i = inputs_from(d, ctx, self.ar_road, wheelbase=float(getattr(self.CP, "wheelbase", 0.0) or 2.7),
                    steer_ratio=float(getattr(self.CP, "steerRatio", 0.0) or 15.5), slowdown_busy=busy,
                    is_metric=self.metric)
    button = ar.update(i)
    for kind, data in ar.events:
      self._log("auto_resume", event=kind, **data)
    ar.events.clear()
    key = (STATE_NAMES.get(ar.state, str(ar.state)), ar.wait_why if ar.state in (ARMED, SENDING) else "")
    if key != self.ar_key and now - self.ar_key_t >= PLAN_LOG_GAP:
      self.ar_key, self.ar_key_t = key, now
      self._log("auto_resume_state", state=key[0], wait=key[1], map=bool(i.plan_alive), cls=i.road_class,
                trust=i.gps_trust, set_allowed=i.set_allowed, engageable=i.engageable)
    return button is not None

  def _route(self):
    """The route name 7.0's loggerd keeps in CurrentRoute (no position), or ""."""
    try:
      raw = self.params.get("CurrentRoute") if self.params is not None else None
      return raw.decode() if isinstance(raw, bytes) else str(raw or "")
    except Exception:
      return ""

  def _tag(self, now, d, ctx, plan, states):
    """One "tag" line: what the speed control saw and did when he pressed the Bookmark button. Never a position."""
    self.tags += 1
    view = ctx.get("view") or {}
    contribs = {name: {"target": None if c.target is None else round(c.target, 2),
                       "ceiling": None if c.ceiling is None else round(c.ceiling, 2),
                       "anchor": None if c.anchor is None else round(c.anchor, 2), "presses": c.presses,
                       "episode": None if c.episode is None else str(c.episode)}
                for name, c in (ctx.get("contribs") or {}).items()}
    snapshot = {
      "n": self.tags, "wall": round(time.time(), 1), "route": self._route(),  # noqa: TID251 (wall time for the owner)
      "drive_s": round(now - self.t0, 1), "segment_approx": int(max(now - self.t0, 0.0) // 60),
      "v": round(d["v_ego"], 2), "set": round(d["set_speed"], 2), "anchor": self.planner.anchor,
      "owner": self.planner.owner, "reason": plan.reason, "command": plan.command, "test_reason": plan.dry_reason,
      "test_command": plan.dry_command, "cruise": d["cruise_enabled"], "metric": self.metric,
      "states": {k: _plain(v) for k, v in states.items()}, "contribs": contribs,
      "view": {"present": bool(view), "gps": view.get("gps"), "why": view.get("why", ""), "trust": view.get("trust"),
               "cls": view.get("cls"), "link": view.get("link"), "matched": view.get("matched"),
               "data": view.get("data"), "curves": len(view.get("curves") or ()), "exits": len(view.get("exits") or ()),
               "ramp": bool(view.get("ramp")), "name": any(bool(c.get("name")) for c in view.get("curves") or ())},
      "gps_ok": ctx.get("gps_ok"), "model_age": round(min(ctx.get("model_age", 1e9), 1e3), 2),
      "lead": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in (ctx.get("lead") or {}).items() if k != "t"},
      "radar_lead": d.get("radar_lead"),
      "presses": [list(p) for p in self.presses if now - p[0] <= TAG_PRESSES],
      "auto_resume": None if self.auto_resume is None else {"state": self.auto_resume.state,
                                                            "wait": self.auto_resume.wait_why},
    }
    self._log("tag", **snapshot)
    if self.notices is not None:
      self.notices.tagged(now)

  def _note_gps(self, ctx, now):
    """One "gps" line whenever the map view's GPS state changes: "ok" with the service it came from, or why there is
    no position ("no_messages" / "no_fix" / "stale" from mazda_mapviewd, "no_view": its file is missing). Never a
    position."""
    view = ctx.get("view")
    if view is None:
      key = ("none", "no_view", "", False)
    else:
      stale = ctx.get("now", now) - float(view.get("t", -1e9)) > GPS_LOG_STALE
      key = (str(view.get("gps", "none")), str(view.get("why", "")), str(view.get("src", "")), bool(stale))
    if key != self.gps_key:
      self.gps_key = key
      self._log("gps", state=key[0], why=key[1], src=key[2], view_stale=key[3])

  def _learn(self, cruise):
    p = self.profile
    if p is None:
      if self.memory is not None:
        self.memory.save_if_due(time.monotonic(), force=self.cruise_was and not cruise)
      self.cruise_was = cruise
      return
    p.set_latencies(self.actuator.latencies)
    while self.holds_seen < len(self.actuator.holds_learned):
      after, driver = self.actuator.holds_learned[self.holds_seen]
      p.add_hold(after, driver=driver)
      self.holds_seen += 1
    p.save_if_due(force=self.cruise_was and not cruise)
    if self.memory is not None:
      self.memory.save_if_due(time.monotonic(), force=self.cruise_was and not cruise)
    self.cruise_was = cruise

  def frame(self, card, button, counter):
    from opendbc.car.mazda import mazdacan
    from opendbc.car.mazda.values import Buttons
    packer = card.CI.CC.packer
    if button == RESUME:
      return mazdacan.create_button_cmd(packer, self.CP, counter, Buttons.RESUME)
    return mazdacan.create_button_cmd(packer, self.CP, counter, Buttons.SET_PLUS if button == SET_PLUS else Buttons.SET_MINUS)


def make(card):
  """SpeedControl for this drive, or False (off: the Stage 1 drive)."""
  try:
    cfg = C.read(card.params)
    if not (cfg.any_active or cfg.tag_on) or not eligible(card.CP):
      return False
    from openpilot.starpilot.car.mazda.speed.car_profile import CarProfile
    from openpilot.starpilot.car.mazda.speed.events import EventLog
    from opendbc.car.mazda.values import MazdaFlags
    log = EventLog()
    profile = None if not cfg.any_active else CarProfile(fingerprint=str(card.CP.carFingerprint),   # (tag only: none)
                                                          ti=bool(int(card.CP.flags) & int(MazdaFlags.TORQUE_INTERCEPTOR)))
    from openpilot.starpilot.car.mazda.speed.contributors import build
    memory = None
    if cfg.active("curves"):
      from openpilot.starpilot.car.mazda.speed.curve_memory import CurveMemory
      memory = CurveMemory()
    auto_resume = None
    if cfg.auto_resume_on:
      from opendbc.car.mazda.speed_flags import auto_resume as panda_auto_resume
      if panda_auto_resume(card.CP):      # the panda's window bit is in this drive's safety param
        from openpilot.starpilot.car.mazda.speed.auto_resume import AutoResumeLogic
        auto_resume = AutoResumeLogic(bool(getattr(card, "is_metric", False)))
    contributors, sources = build(cfg, card.CP, log, memory=memory, metric=bool(getattr(card, "is_metric", False)),
                                  auto_resume=auto_resume is not None)
    notices = None
    if cfg.shows_notices:
      from openpilot.starpilot.car.mazda.speed.notices import Notices
      notices = Notices(cfg, metric=bool(getattr(card, "is_metric", False)), log=log,
                        ti=bool(int(card.CP.flags) & int(MazdaFlags.TORQUE_INTERCEPTOR)))
    return SpeedControl(card.CP, cfg, card.params, getattr(card, "is_metric", False), log=log, profile=profile,
                        contributors=contributors, sources=sources, memory=memory, notices=notices,
                        auto_resume=auto_resume)
  except Exception:
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception("mazda speed control init failed, off for this drive")
    except Exception:
      pass
    return False


def update(card, CS, CC, can_sends, now_ns=None):
  ctl = getattr(card, "mazda_speed", None)
  if ctl is None:
    ctl = card.mazda_speed = make(card)
  if ctl is False:
    return can_sends
  try:
    d = ctl.inputs(card, CS, CC, can_sends)
    r = ctl.step(d)
    if r is None:
      return can_sends
    return list(can_sends) + [ctl.frame(card, *r)]
  except Exception:
    card.mazda_speed = False
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception("mazda speed control failed, off for this drive")
      ctl._log("failed")
    except Exception:
      pass
    return can_sends
