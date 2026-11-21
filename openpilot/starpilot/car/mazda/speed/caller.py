"""The hook card.py calls on a Mazda after CI.apply(): reads the car, runs the contributors, the planner and the
actuator, and returns the CAN frames to send (the given list unchanged when there is nothing to add).

With the master switch off (or nothing chosen, or not an eligible car) the first call stores `False` on card and
every later call returns the frames list untouched: the drive is the Stage 1 drive.
Any exception turns the feature off for the rest of the drive (steering is never affected).
"""
import time

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed.actuator import Actuator, ActInputs, SET_PLUS
from openpilot.starpilot.car.mazda.speed.planner import Planner, PlanInputs

CRZ_BTNS = 0x09D
PLAN_LOG_GAP = 0.2            # s: a changed plan is logged at most this often


def eligible(CP) -> bool:
  try:
    from opendbc.car.mazda.values import MazdaFlags
    return bool(CP.brand == "mazda" and not CP.notCar and int(CP.flags) & int(MazdaFlags.GEN1) and CP.pcmCruise and
                not CP.openpilotLongitudinalControl and not CP.passive)
  except (AttributeError, TypeError, ValueError):
    return False


class SpeedControl:
  def __init__(self, CP, cfg, params, is_metric, log=None, profile=None, contributors=None):
    self.CP = CP
    self.cfg = cfg
    self.metric = bool(is_metric)
    self.log = log
    self.profile = profile
    lat = profile.latencies if profile is not None else ()
    up, down = (profile.hold_start(1)[0], profile.hold_start(0)[0]) if profile is not None else (1, 0)
    self.actuator = Actuator(self.metric, latencies=lat, overrun_up=up, overrun_down=down)
    self.planner = Planner(self.metric)
    self.contributors = list(contributors or [])
    self.events = []                 # last frame's actuator events (the planner reads them one frame later)
    self.last_plan = None
    self.last_plan_t = 0.0
    self.holds_seen = 0
    self.cruise_was = False
    self.packer = None
    if profile is not None:
      profile.note_drive()
    self._log("start", config=C.encode(cfg).decode(), metric=self.metric,
              profile=profile.summary() if profile is not None else None)

  def _log(self, kind, **kw):
    if self.log is not None:
      self.log.log(kind, **kw)

  def inputs(self, card, CS, CC, can_sends):
    ocs = card.CI.CS
    other = bool(CC.cruiseControl.cancel or CC.cruiseControl.resume or
                 any(f[0] == CRZ_BTNS for f in can_sends))
    return {"cruise_enabled": bool(CS.cruiseState.enabled), "main_on": bool(CS.cruiseState.available),
                "set_speed": float(CS.cruiseState.speed), "v_ego": float(CS.vEgo), "gas": bool(CS.gasPressed),
                "brake": bool(CS.brakePressed), "standstill": bool(CS.standstill or CS.cruiseState.standstill),
                "set_plus": bool(ocs.crz_set_plus), "set_minus": bool(ocs.crz_set_minus), "resume": bool(ocs.crz_resume),
                "cancel": bool(ocs.crz_cancel), "wheel_counter": int(ocs.crz_btns_counter), "other_button": other,
                "engaged": bool(CC.enabled), "lkas_blocked": bool(getattr(ocs, "lkas_blocked", False))}

  def step(self, d, ctx=None, now=None):
    """d: the dict from inputs(); ctx: what contributors get (the car's other signals). Returns (button, counter)."""
    now = time.monotonic() if now is None else now
    ctx = ctx if ctx is not None else {}
    ctx.update(d)
    contributions = []
    for c in self.contributors:
      got = c(ctx)
      if got is not None:
        contributions.append(got)
    plan = self.planner.update(PlanInputs(v_ego=d["v_ego"], set_speed=d["set_speed"], cruise_enabled=d["cruise_enabled"],
                                          main_on=d["main_on"], events=self.events, contributions=contributions,
                                          metric=self.metric))
    for why, before, after in self.planner.changes:
      self._log("anchor", why=why, before=before, after=after)
    command = plan.command if d["engaged"] else None
    act = self.actuator.update(ActInputs(cruise_enabled=d["cruise_enabled"], set_speed=d["set_speed"], v_ego=d["v_ego"],
                                         gas=d["gas"], brake=d["brake"], standstill=d["standstill"],
                                         set_plus=d["set_plus"], set_minus=d["set_minus"], resume=d["resume"],
                                         cancel=d["cancel"], wheel_counter=d["wheel_counter"], command=command,
                                         allow_lower=plan.allow_lower, other_button=d["other_button"]))
    self.events = act.events
    for e in act.events:
      self._log("act", event=e)
    key = (plan.reason, None if plan.command is None else round(plan.command, 1), plan.dry_reason,
           None if plan.dry_command is None else round(plan.dry_command, 1))
    if key != self.last_plan and now - self.last_plan_t >= PLAN_LOG_GAP:
      self.last_plan, self.last_plan_t = key, now
      self._log("plan", reason=plan.reason, command=plan.command, target=plan.target, anchor=self.planner.anchor,
                set=d["set_speed"], v=round(d["v_ego"], 2), test_reason=plan.dry_reason, test_command=plan.dry_command)
    self._learn(d["cruise_enabled"])
    return (act.button, act.counter) if act.button is not None else None

  def _learn(self, cruise):
    p = self.profile
    if p is None:
      return
    p.set_latencies(self.actuator.latencies)
    while self.holds_seen < len(self.actuator.holds_learned):
      after, driver = self.actuator.holds_learned[self.holds_seen]
      p.add_hold(after, driver=driver)
      self.holds_seen += 1
    p.save_if_due(force=self.cruise_was and not cruise)
    self.cruise_was = cruise

  def frame(self, card, button, counter):
    from opendbc.car.mazda import mazdacan
    from opendbc.car.mazda.values import Buttons
    packer = card.CI.CC.packer
    return mazdacan.create_button_cmd(packer, self.CP, counter, Buttons.SET_PLUS if button == SET_PLUS else Buttons.SET_MINUS)


def make(card):
  """SpeedControl for this drive, or False (off: the Stage 1 drive)."""
  try:
    cfg = C.read(card.params)
    if not cfg.any_active or not eligible(card.CP):
      return False
    from openpilot.starpilot.car.mazda.speed.car_profile import CarProfile
    from openpilot.starpilot.car.mazda.speed.events import EventLog
    from opendbc.car.mazda.values import MazdaFlags
    log = EventLog()
    profile = CarProfile(fingerprint=str(card.CP.carFingerprint), ti=bool(int(card.CP.flags) & int(MazdaFlags.TORQUE_INTERCEPTOR)))
    from openpilot.starpilot.car.mazda.speed.contributors import build
    contributors = build(cfg, card.CP, log)
    return SpeedControl(card.CP, cfg, card.params, getattr(card, "is_metric", False), log=log, profile=profile,
                        contributors=contributors)
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
