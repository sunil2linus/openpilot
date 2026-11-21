"""
The ONE target arbitrator. Every control cycle:

    target = min(the driver's own speed (his anchor), each contributor's target)

and the actuator (actuator.py) moves the set speed there. Contributors are pure and never press buttons; each gives
a true-speed target (m/s) or None (no wish), and may ask for the LKAS floor (cruise_model.LKAS_FLOOR).

The driver's own speed ("anchor", dash units):
  * the set speed when the cruise engages, or his own new set speed (SET at another speed, his SET-, his SET+ to or
    past the anchor, any change the actuator could not explain);
  * his SET+ while we hold the set below his anchor takes back part of OUR lowering: the anchor stays (we restore to
    it later), but nothing of ours lowers again until the reason is over;
  * his SET- is his new speed (we never raise above it); while a reason is on it also blocks our lowering for that
    reason ("his buttons win": the curve he slowed for himself is his);
  * RES with the set where we left it keeps the anchor (we restore from there); main off forgets it.
We never press above the anchor; his own speed is only pressed down by a contributor he turned on.
Pure logic (tests/test_planner.py).
"""
from dataclasses import dataclass, field

from openpilot.starpilot.car.mazda.speed import cruise_model as M

SET_PLUS = "SET_PLUS"
SET_MINUS = "SET_MINUS"


@dataclass
class Contribution:
  name: str
  target: float | None            # m/s true speed, None: no wish
  lkas_floor: bool = False        # a curve whose own target allows the LKAS floor
  presses: bool = True            # False: test mode (logged, never pressed for)


@dataclass
class PlanInputs:
  v_ego: float
  set_speed: float                # m/s dash units
  cruise_enabled: bool
  main_on: bool
  events: list = field(default_factory=list)   # actuator events of this frame
  contributions: list = field(default_factory=list)
  metric: bool = False


@dataclass
class PlanOutput:
  command: float | None = None    # m/s dash units for the actuator, None: leave the set speed alone
  allow_lower: bool = True
  reason: str = ""                # the governing contributor ("" = the driver's own speed)
  target: float | None = None     # m/s true speed of the governing contributor
  dry_command: float | None = None   # what the test-mode contributors would command
  dry_reason: str = ""


class Planner:
  def __init__(self, metric=False):
    self.metric = metric
    self.step = M.KPH if metric else M.MPH
    self.anchor = None
    self.blocked = set()            # reasons he overruled this episode
    self.lowered = False            # our command was below his anchor at some point this episode
    self.changes = []               # anchor changes for the log: (why, before, after)

  def _set_anchor(self, value, why):
    if self.anchor is None or abs(value - self.anchor) >= 0.5 * self.step:
      self.changes.append((why, self.anchor, value))
    self.anchor = value

  def _driver(self, e, active_reasons):
    d, after = e.get("direction"), e.get("after")
    if after is None:
      return
    if d == SET_PLUS and self.anchor is not None and self.lowered and after < self.anchor - 0.5 * self.step:
      self.blocked |= active_reasons          # he takes back part of our lowering: no more lowering this episode
      self.changes.append(("his SET+ under the anchor", self.anchor, self.anchor))
      return
    self._set_anchor(after, "driver " + str(d))
    self.blocked |= active_reasons

  def _command(self, contributions, i, dry):
    best = None
    for c in contributions:
      if c.target is None or c.presses == dry:
        continue
      if c.name in self.blocked and not dry:
        continue
      if best is None or c.target < best.target:
        best = c
    if best is None:
      return self.anchor, "", None
    cmd = M.set_command(best.target, i.v_ego, self.metric, best.lkas_floor)
    if cmd >= self.anchor - 0.5 * self.step:
      return self.anchor, "", best.target
    return cmd, best.name, best.target

  def update(self, i: PlanInputs) -> PlanOutput:
    self.changes = []
    out = PlanOutput()
    if not i.main_on:
      if self.anchor is not None:
        self.changes.append(("main off", self.anchor, None))
      self.anchor, self.blocked, self.lowered = None, set(), False
      return out
    anchor_wish = {c.name for c in i.contributions if c.target is not None and c.presses and self.anchor is not None and
                   M.set_command(c.target, i.v_ego, self.metric, c.lkas_floor) < self.anchor - 0.5 * self.step}
    for e in i.events:
      k = e["kind"]
      if k == "engaged" and (e.get("before") is None or e.get("changed") or self.anchor is None):
        self._set_anchor(e["after"], "engaged")
        self.blocked, self.lowered = set(), False
      elif k == "resumed" and self.anchor is None:
        self._set_anchor(e["after"], "resumed")
      elif k == "driver":
        self._driver(e, anchor_wish)
    if self.anchor is None or not i.cruise_enabled:
      return out
    # a reason he overruled stays blocked until it no longer wants anything below the anchor
    self.blocked &= anchor_wish
    cmd, reason, target = self._command(i.contributions, i, dry=False)
    if self.blocked:
      # a reason he overruled is still on: the set speed stays where he put it (no raise either), unless another
      # reason wants it lower
      hold = min(self.anchor, i.set_speed)
      if cmd >= hold:
        cmd, reason = hold, "his choice (" + ",".join(sorted(self.blocked)) + ")"
    out.command, out.reason, out.target = cmd, reason, target
    if reason and not self.blocked:
      self.lowered = True
    elif i.set_speed >= self.anchor - 0.5 * self.step:
      self.lowered = False
    out.allow_lower = True
    dry, dry_reason, _ = self._command(i.contributions, i, dry=True)
    out.dry_command, out.dry_reason = (dry, dry_reason) if dry_reason else (None, "")
    return out
