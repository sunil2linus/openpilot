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
  * The press that engages the cruise (SET+, SET- or RES from standby) is the engagement, not a change of his: the
    actuator reports "engaged" / "resumed" for it (also when the car shows the new set speed a few frames after the
    cruise came on). "resumed" keeps the anchor and its value; an anchor his SET+/SET- had made his becomes
    "engaged" again (a new engagement), one of ours stays ours.
We never press above the anchor; his own speed is only pressed down by a contributor he turned on.

Who owns the anchor (`owner`): "engaged" (he engaged / SET / RES at that speed), "his" (his SET+/SET- made it his) or
"ours" (a speed we set: the limit follow, the merge, the exit handover). A contributor may PROPOSE a new anchor (dash
units, whole steps, no margin: Contribution.anchor); the planner takes it only by these rules (test mode: logged only):
  * "limit": when the anchor is ours (raise or lower, any band), or he engaged at a speed within the proposal's band
    (the limit's band). His own speed (engaged or his SET+/SET-) is re-taken only by a NEW limit (the limit
    change rule, the old fork's rules 1 and 9): a proposal whose limit became steady after the anchor became his
    (Contribution.seq above the limit seq of his last SET+/SET- or engagement) and that its contributor marks
    `retake` (fresh, and for a lower one: a street, the longer steady time). A raise then needs his speed in the
    band of the limit it replaces (his 50 on a 45 road -> the 55 road's speed; his 65 there stays); a lower one
    takes any band. Never while he overrules a slowdown (a blocked reason), and a raise never while a slowdown is
    under way (a contributor wants less than the anchor): it waits (the proposal stays fresh for a while). Once
    taken the anchor is ours; his next SET+/SET- makes it his again until the next limit change. His own higher
    speed on a highway is never pressed down (80 on a 70 road);
  * "fallback" (43 mph with the limit unknown): only up, only an engaged or our anchor;
  * "merge": up from an anchor that is ours, a street's (MERGE_SURFACE_MAX), or his curve speed on that link (the
    proposal's band: his SET- in a ramp curve); down (the taper, the end) only from our merge anchor;
  * "handover" (exit onto a street): only down, any owner (the contributor ends it for good on his gas / buttons);
  * "restore" (a handover overturned: the exit was not taken): only up, only from our handover anchor;
  * never in a frame with his own change (his buttons win), never with the cruise off.
The first proposal in the order handover, merge, limit, fallback that passes is taken.

A contributor may also give a CEILING (dash units, Contribution.ceiling) instead of, or with, a target: the command
never goes above it while it governs. The stop slowdown uses it for its 1 mph guard taps (a ceiling one step under
the set at a time, never more than its bound) and for a step-by-step raise after his RES (a ceiling one step above
the set: taps, no 5 mph hold). A ceiling is a wish like a target: the lowest command wins, his overrule blocks it.
Pure logic (tests/test_planner.py).
"""
from dataclasses import dataclass, field

from openpilot.starpilot.car.mazda.speed import cruise_model as M

SET_PLUS = "SET_PLUS"
SET_MINUS = "SET_MINUS"


HIS, ENGAGED, OURS = "his", "engaged", "ours"
KINDS = ("restore", "handover", "merge", "limit", "fallback")
MERGE_SURFACE_MAX = (55 * M.MPH, 90 * M.KPH)   # an anchor at or under this is a street's: a merge may raise it


@dataclass
class Contribution:
  name: str
  target: float | None            # m/s true speed, None: no wish
  lkas_floor: bool = False        # a curve whose own target allows the LKAS floor
  presses: bool = True            # False: test mode (logged, never pressed for)
  episode: str | None = None      # what it is slowing for (a curve id): his overrule lasts for that episode
  anchor: float | None = None     # a proposed new anchor (dash units m/s, a whole step), see the rules above
  kind: str = ""                  # its rule: one of KINDS
  band: tuple | None = None       # "limit": (low, high) dash m/s, an engaged anchor in it follows the proposal
  no_raise: bool = False          # while it governs, the command never goes above the current set (an unresolved exit)
  ceiling: float | None = None    # dash units m/s: the command never above this (stop guard taps, a step-by-step raise)
  seq: int = 0                    # "limit" / "fallback": the id of the steady limit (it grows with each limit change)
  retake: bool = False            # "limit": this proposal may re-take his own speed (the limit change rule above)


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
  dry_anchor: float | None = None    # the anchor a test-mode proposal would have set
  dry_anchor_kind: str = ""


class Planner:
  def __init__(self, metric=False):
    self.metric = metric
    self.step = M.KPH if metric else M.MPH
    self.anchor = None
    self.blocked = {}               # reason -> the episode he overruled (None: until it wants nothing below him)
    self.lowered = False            # our command was below his anchor at some point this episode
    self.changes = []               # anchor changes for the log: (why, before, after)
    self.owner = None               # HIS / ENGAGED / OURS (None: no anchor)
    self.ours_why = ""              # the rule that made it ours
    self.own_seq = 0                # the limit seq when the anchor last became the driver's (engaged / his)

  def _set_anchor(self, value, why):
    if self.anchor is None or abs(value - self.anchor) >= 0.5 * self.step:
      self.changes.append((why, self.anchor, value))
    self.anchor = value

  def _driver(self, e, active_reasons, seq=0):
    d, after = e.get("direction"), e.get("after")
    if after is None:
      return
    self.own_seq = seq                        # a limit steady before his press is answered by it
    if d == SET_PLUS and self.anchor is not None and self.lowered and after < self.anchor - 0.5 * self.step:
      self.blocked.update(active_reasons)     # he takes back part of our lowering: no more lowering this episode
      self.changes.append(("his SET+ under the anchor", self.anchor, self.anchor))
      return
    self._set_anchor(after, "driver " + str(d))
    self.owner, self.ours_why = HIS, ""
    self.blocked.update(active_reasons)

  def _takes(self, c, slowing=False):
    """True when proposal c may become the anchor now (the rules in the module doc). slowing: a contributor wants the
    set speed under the anchor this frame (a slowdown under way)."""
    a, half = self.anchor, 0.5 * self.step
    if c.anchor is None or abs(c.anchor - a) < half:
      return False
    if c.kind == "handover":
      return c.anchor < a
    if c.kind == "restore":
      return c.anchor > a and self.owner == OURS and self.ours_why == "handover"
    if c.kind == "merge":
      if c.anchor > a:
        his_curve = c.band is not None and c.band[0] - half <= a <= c.band[1] + half   # his curve speed on the link
        return self.owner == OURS or a <= MERGE_SURFACE_MAX[1 if self.metric else 0] + half or his_curve
      return self.owner == OURS and self.ours_why == "merge"
    if c.kind == "limit":
      if self.owner == OURS:
        return True
      in_band = c.band is not None and c.band[0] - half <= a <= c.band[1] + half
      if self.owner == ENGAGED and in_band:
        return True
      # the limit change rule: a NEW limit re-takes his own speed (engaged or his)
      if not (c.retake and c.seq > self.own_seq) or self.blocked:
        return False
      if c.anchor > a:
        return in_band and not slowing
      return True
    if c.kind == "fallback":
      return c.anchor > a and self.owner in (OURS, ENGAGED)
    return False

  def _proposals(self, contributions, dry, slowing=False):
    for kind in KINDS:
      for c in contributions:
        if c.kind == kind and c.anchor is not None and c.presses != dry and self._takes(c, slowing):
          return c
    return None

  def _wish(self, c, v_ego):
    """The command (dash units) contribution c asks for, or None (no wish): its target's set command, its ceiling."""
    cmd = None if c.target is None else M.set_command(c.target, v_ego, self.metric, c.lkas_floor)
    if c.ceiling is not None:
      cmd = c.ceiling if cmd is None else min(cmd, c.ceiling)
    return cmd

  def _command(self, contributions, i, dry):
    best, best_cmd = None, None
    for c in contributions:
      if (c.target is None and c.ceiling is None) or c.presses == dry:
        continue
      if c.name in self.blocked and not dry:
        continue
      cmd = self._wish(c, i.v_ego)
      if c.no_raise and not dry:
        cmd = min(cmd, i.set_speed)
      if best is None or cmd < best_cmd:
        best, best_cmd = c, cmd
    if best is None:
      return self.anchor, "", None
    if best_cmd >= self.anchor - 0.5 * self.step:
      return self.anchor, "", best.target
    return best_cmd, best.name, best.target

  def update(self, i: PlanInputs) -> PlanOutput:
    self.changes = []
    out = PlanOutput()
    if not i.main_on:
      if self.anchor is not None:
        self.changes.append(("main off", self.anchor, None))
      self.anchor, self.blocked, self.lowered, self.owner, self.ours_why = None, {}, False, None, ""
      return out
    anchor_wish = {c.name: c.episode for c in i.contributions if (c.target is not None or c.ceiling is not None) and
                   c.presses and self.anchor is not None and self._wish(c, i.v_ego) < self.anchor - 0.5 * self.step}
    seq = max((c.seq for c in i.contributions if c.kind in ("limit", "fallback")), default=0)
    his_change = False
    for e in i.events:
      k = e["kind"]
      if k == "engaged" and (e.get("before") is None or e.get("changed") or self.anchor is None):
        self._set_anchor(e["after"], "engaged")
        self.blocked, self.lowered, self.owner, self.ours_why = {}, False, ENGAGED, ""
        self.own_seq = seq
        his_change = True
      elif k == "resumed" and self.anchor is None:
        self._set_anchor(e["after"], "resumed")
        self.owner, self.ours_why, self.own_seq = ENGAGED, "", seq
        his_change = True
      elif k == "resumed":
        # his RES (or our auto-resume's) is an engagement: the anchor stays (we restore from there); his own speed
        # from before is an engaged one again, ours stays ours
        if self.owner == HIS:
          self.owner, self.own_seq = ENGAGED, seq
          self.changes.append(("resumed (was his)", self.anchor, self.anchor))
          his_change = True
      elif k == "driver":
        self._driver(e, anchor_wish, seq)
        his_change = True
    if self.anchor is None or not i.cruise_enabled:
      return out
    if not his_change:
      prop = self._proposals(i.contributions, dry=False, slowing=bool(anchor_wish))
      if prop is not None:
        self._set_anchor(prop.anchor, prop.kind)
        self.owner, self.ours_why = OURS, prop.kind
      dry_prop = self._proposals(i.contributions, dry=True, slowing=bool(anchor_wish))
      if dry_prop is not None:
        out.dry_anchor, out.dry_anchor_kind = dry_prop.anchor, dry_prop.kind
    # a reason he overruled stays blocked for the episode (curve) he overruled; without an episode, until it no longer
    # wants anything below his anchor
    current = {c.name: c for c in i.contributions if c.presses}
    for name, episode in list(self.blocked.items()):
      c = current.get(name)
      if c is None or (c.target is None and c.ceiling is None):
        del self.blocked[name]
      elif episode is not None:
        if c.episode != episode:
          del self.blocked[name]
      elif name not in anchor_wish:
        del self.blocked[name]
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
