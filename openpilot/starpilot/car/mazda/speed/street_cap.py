"""
The street cap (round 2): a highway's set speed must not stay the anchor on a street. A safety net that does not
depend on the map's ramp marking (the exit plan's handover needs the ramp marked "to a street"; r2: M-53's exit to
23 Mile Rd was marked "to the motorway", so his 70 stayed the anchor there and we pressed SET+ toward it twice).
It only PROPOSES anchors (kind "street": planner.py takes it only DOWN, from any owner; the anchor is then ours and
his next SET+ / SET- is his again); it never presses and gives no slowing target.

The exit to a street (ported from the old fork's map_curve_control "exit to a street, completed", _exit_street_track /
_exit_street / street_target / _street_cmd):
  * The car was on a motorway, or on a link entered from a highway (motorway or trunk) for at least EXIT_LINK_MIN,
    and the street comes within EXIT_FIND_M after it (new: the old fork waited for a street without a bound).
  * Then a street is matched with GPS trust 0 for EXIT_STREET_STEADY (grows on CURRENT views, holds while LATE): road
    class 5-12 (primary .. residential), or a trunk road whose posted limit is known and below STREET_LIMIT_MAX (Hall Rd
    with its 45). The window stays open EXIT_STREET_TIME after that (never closed while it is taking the set down).
  * Its target: the street's posted limit (7.0's reading, unchanged EXIT_STREET_LIMIT_STEADY) + the street offset;
    a limit above STREET_LIMIT_MAX is no street's (no cap); no limit reported for EXIT_STREET_LIMIT_STEADY: the old
    fork's STREET_FALLBACK (43 mph, no offset).
  * Once per street, only with the car under EXIT_STREET_MAX_V, only when the anchor is above the target: the anchor is
    proposed down, paced like the old _street_cmd (never more than LOWER_GAP under the car's dash speed: the cruise
    slows ~0.5 m/s^2), until the planner has it at the target. His SET+ / SET- ends it (his speed); his brake / the
    cruise off lets it go on after his RES (old: "it goes on after his RES"); his gas does not end it.
  * While it is open the merge drops a pending raise (ctx["street_cap"]) and the planner lets a limit raise take our
    street anchor only for a limit that became steady after it (no raise back toward the highway's old limit).
The cap after a RES (round 2, item 10; r3 rt 402: the auto-resume brought back 55 at 21 mph on a 25-35 mph street;
r2 rt 1044: 53 carried onto Hall Rd's 45): after ANY RES (his on the wheel or our auto-resume's, the planner's
"resumed") on a trusted street (GPS trust 0, matched, not a link; class 5-12 or a trunk road) with the anchor above
what the street allows, the anchor is proposed down (paced, kind "street", ours): to the limit + offset when the
limit is known (read now, or last read on this road chain with at most RES_LIMIT_M driven since; above
STREET_LIMIT_MAX: no cap), else to min(his anchor, max(v, RES_V_MIN) + RES_OVER) (a RES at 10 mph: 45). (Round 3: the
remembered limit ages by distance, not by the clock: a red light of any length on Hall Rd keeps its 45, where 7.0's
mapd says "not sure which road" for most of the stop; r2 rt 1030. SpeedLimit's steady limit is no source here: it
carries no road and is held 30 s across a gap, so after an exit it can be the highway's 70.) His SET+ / SET- afterwards is his speed as
always. On a motorway, a link or an unknown road his RES keeps his speed (as before). (The auto-resume itself sends
no RES on a link under 25 mph: auto_resume.py.)
Pure (tests/test_street_cap.py).
"""
import math

from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.limits import whole_limit
from openpilot.starpilot.car.mazda.speed.planner import Contribution
from openpilot.starpilot.car.mazda.speed.view_age import GONE, LATE, ViewClock

MPH, KPH = M.MPH, M.KPH
TRUST_OK = 0
MOTORWAY = 1
TRUNK = 3
HIGHWAY_CLASSES = (1, 3)
STREET_CLASSES = (5, 7, 9, 11, 12)
EXIT_LINK_MIN = 50.0            # m on a link entered from a highway (old EXIT_LINK_MIN)
EXIT_STREET_STEADY = 2.0        # s matched to a street with trust 0 (old EXIT_STREET_STEADY)
EXIT_STREET_LIMIT_STEADY = 2.0  # s the street's limit (or "no limit") reported unchanged (old)
EXIT_STREET_MAX_V = 55 * MPH    # m/s: only below this (a wrong street match beside a highway is at highway speed)
EXIT_STREET_TIME = 60.0         # s the window stays open (old)
STREET_LIMIT_MAX = (55, 90)     # dash units: a posted limit above this is no street's (old STREET_LIMIT_MAX)
STREET_FALLBACK = (43, 70)      # dash units: the limit unknown (old STREET_FALLBACK, the owner's "sweet spot")
LOWER_GAP = 2.4                 # m/s: never more than this under the car's dash speed while it slows (old LOWER_GAP)
HWY_RESET = 1.0                 # s back on a highway: the sequence starts over (old HWY_RESET)
EXIT_FIND_M = 1000.0            # m off the highway / link without a trusted street: no exit street any more (new)
RES_CAP_TIME = 30.0             # s after a RES its cap is proposed (until the planner has it at the cap)
RES_LIMIT_M = 400.0             # m driven since the limit was last read on this road chain: still known at the RES
RES_V_MIN = 25 * MPH            # m/s: the limit unknown, the cap is at least this ...
RES_OVER = 20 * MPH             # ... + this over the car's speed (and never above his anchor)


class StreetCap:
  def __init__(self, cfg, presses=True, metric=False):
    self.cfg = cfg
    self.presses = presses
    self.metric = bool(metric)
    self.k = 1 if self.metric else 0
    self.unit = KPH if self.metric else MPH
    self.offset = int(cfg.limit_offset_mph)
    self.t = None
    self.view_clock = ViewClock()
    self.last = None
    self.reset()
    self.lim, self.lim_t = None, 0.0     # the street's reading (dash units, 0: none) and s unchanged
    self.n = 0                      # caps this drive (the notice's key)
    self.res = None                 # [cap dash m/s, until s]: the cap after a RES (RES_CAP_TIME)
    self.seen = None                # [dash units, chain, m driven since]: the last limit reading (RES_LIMIT_M)
    self.cap_to = None              # dash units of the cap now being taken (the notice's number)
    self.exit_tgt = None            # dash m/s: the exit cap's target while it is taken

  def reset(self):
    self.before = 0                 # the last road class before (0 unknown): a highway or a link from one
    self.link_m = 0.0               # m on a link entered from a highway
    self.exited = False             # EXIT_LINK_MIN of it (or a motorway) driven: a street after it is an exit's
    self.after_m = 0.0              # m driven off the highway / link since (EXIT_FIND_M)
    self.street_t = 0.0             # s matched to a street with trust 0
    self.hwy_t = 0.0
    self.since = None               # s the window has been open (None: closed)
    self.on = False                 # the cap is being taken down
    self.done = False               # applied (or declined by his button) on this street

  def _street(self, view, limit):
    cls = int(view.get("cls", 0) or 0)
    if view.get("link") or not view.get("matched"):
      return False
    if cls in STREET_CLASSES:
      return True
    return cls == TRUNK and limit is not None and limit < STREET_LIMIT_MAX[self.k]

  def target(self):
    """dash m/s of the street's cap, or None (not known yet / no street's limit)."""
    if self.lim_t < EXIT_STREET_LIMIT_STEADY - 1e-6:
      return None
    if not self.lim:
      return STREET_FALLBACK[self.k] * self.unit
    if self.lim > STREET_LIMIT_MAX[self.k]:
      return None
    return (self.lim + self.offset) * self.unit

  def _his(self, ctx):
    plus_minus = brake_off = False
    for e in ctx.get("act_events", ()):
      if e.get("kind") == "driver" and e.get("direction") in ("SET_PLUS", "SET_MINUS"):
        plus_minus = True
    if not ctx.get("cruise_enabled", False):
      brake_off = True
    return plus_minus, brake_off

  def __call__(self, ctx):
    now, v = ctx.get("now", 0.0), max(ctx.get("v_ego", 0.0), 0.0)
    dt = 0.0 if self.t is None else min(max(now - self.t, 0.0), 0.5)
    self.t = now
    out = Contribution("street", None, presses=self.presses, kind="street")
    view = ctx.get("view")
    live = self.view_clock.update(view, now)
    alive = live != GONE
    slc = ctx.get("slc")
    limit = whole_limit(slc["limit"], self.metric) if slc else None
    if live != LATE:
      self.lim_t = self.lim_t + dt if (limit or 0) == (self.lim or 0) else 0.0
      self.lim = limit or 0
    if alive and view.get("matched"):
      self._track(view, limit, v, dt, live)
    if self.since is not None:
      self.since += dt
      if self.since > EXIT_STREET_TIME and not self.on:
        self.since, self.exited, self.on = None, False, False
    plus_minus, cruise_off = self._his(ctx)
    if self.seen is not None:
      self.seen[2] += v * dt                         # ages by distance only: a stop of any length keeps it
    if limit is not None and alive:
      self.seen = [limit, view.get("chain"), 0.0]
    res = self._res(ctx, view, alive, limit, v, now, plus_minus, cruise_off)
    exit_cap = self._exit(ctx, view, alive, v, plus_minus, cruise_off)
    caps = [x for x in (res, exit_cap) if x is not None]
    if caps:
      out.anchor = min(caps)
      # the notice's number: the cap being taken (the lower target of the two when both are on, round 3: a RES and the
      # exit cap starting on one frame said "Street - set to 43" with the RES cap at 28)
      tgts = ([self.res[0]] if res is not None else []) + ([self.exit_tgt] if exit_cap is not None else [])
      self.cap_to = round(min(tgts) / self.unit)
      self.last = ("res cap" if out.anchor == res else "cap", round(out.anchor / self.unit))
    if caps or self.on or (self.since is not None and alive and not view.get("link")):
      ctx["street_cap"] = True                       # (the merge drops a pending raise toward a highway's speed)
    return out

  def _exit(self, ctx, view, alive, v, plus_minus, cruise_off):
    """The exit to a street's cap now (dash m/s, paced) or None; self.last says why not."""
    if self.since is None:
      self.last = ("idle", None)
      return None
    if plus_minus:
      self.on, self.done = False, True               # his own speed now
    if cruise_off:
      self.on = self.done = False                    # (old) it goes on after his RES
      self.last = ("cruise off", None)
      return None
    tgt = self.target()
    anchor = ctx.get("anchor")
    if not self.on:
      if tgt is None or self.done or anchor is None:
        self.last = ("wait limit" if tgt is None else ("done" if self.done else "no anchor"), None)
        return None
      self.done = True
      if v >= EXIT_STREET_MAX_V or anchor <= tgt + 0.5 * self.unit:
        self.last = ("not above", tgt)
        return None
      self.on = True
      self.n += 1
    if anchor is not None and anchor <= tgt + 0.5 * self.unit:
      self.on = False
      self.last = ("capped", tgt)
      return None
    self.exit_tgt = tgt
    return self._paced(tgt, v)

  def _paced(self, tgt, v):
    """(old _street_cmd) never more than LOWER_GAP under the car's dash speed while it slows."""
    floor = math.ceil((v + M.DASH_OFFSET - LOWER_GAP) / self.unit - 1e-6) * self.unit
    return max(tgt, floor)

  def res_cap(self, view, alive, limit, v, anchor, now):
    """dash m/s the set speed may be after a RES here, or None (no cap: not a trusted street; a highway keeps his
    speed). A street (class 5-12, or a trunk road): its limit + offset when known (now, or last read on this road
    chain at most RES_LIMIT_M back; a limit above STREET_LIMIT_MAX: no cap); unknown: min(his anchor, max(v,
    RES_V_MIN) + RES_OVER)."""
    if not (alive and view.get("matched") and not view.get("link") and view.get("trust") == TRUST_OK):
      return None
    cls = int(view.get("cls", 0) or 0)
    if cls not in STREET_CLASSES and cls != TRUNK:
      return None
    if limit is None and self.seen is not None and self.seen[2] <= RES_LIMIT_M and self.seen[1] == view.get("chain"):
      limit = self.seen[0]
    if limit is not None:
      return None if limit > STREET_LIMIT_MAX[self.k] else (limit + self.offset) * self.unit
    cap = math.ceil((max(v, RES_V_MIN) + RES_OVER) / self.unit - 1e-6) * self.unit
    return min(anchor, cap) if anchor is not None else cap

  def _res(self, ctx, view, alive, limit, v, now, plus_minus, cruise_off):
    """The cap after any RES (his on the wheel, or our auto-resume's): the anchor proposed down to res_cap until the
    planner has it (paced), his SET+ / SET- (his speed), the cruise off again, or RES_CAP_TIME. dash m/s or None."""
    anchor = ctx.get("anchor")
    for e in ctx.get("act_events", ()):
      if e.get("kind") == "resumed":
        cap = self.res_cap(view, alive, limit, v, anchor, now)
        if cap is not None and anchor is not None and anchor > cap + 0.5 * self.unit:
          self.res = [cap, now + RES_CAP_TIME]
          self.n += 1
    if self.res is None:
      return None
    if plus_minus or now > self.res[1] or cruise_off:
      self.res = None
      return None
    if anchor is not None and anchor <= self.res[0] + 0.5 * self.unit:
      self.res = None
      return None
    return self._paced(self.res[0], v)

  def _track(self, view, limit, v, dt, live):
    """(old _exit_street_track) the road classes: a highway / a link from one, then a street with trust 0."""
    cls = int(view.get("cls", 0) or 0)
    link = bool(view.get("link"))
    if not link and (cls == MOTORWAY or (cls == TRUNK and not self._street(view, limit))):
      self.hwy_t += dt
      if self.hwy_t >= HWY_RESET - 1e-6 and not self.on:
        self.since, self.street_t = None, 0.0
        self.done = False
      self.before, self.link_m, self.after_m = cls, 0.0, 0.0
      self.exited = self.exited or cls == MOTORWAY
      return
    self.hwy_t = 0.0
    if link or cls == MOTORWAY:
      self.after_m = 0.0
    if link:
      if self.before in HIGHWAY_CLASSES and self.since is None:
        self.link_m += v * dt
        self.exited = self.exited or self.link_m >= EXIT_LINK_MIN
      self.street_t = 0.0
      return
    if not cls:
      return
    if not self.exited or self.since is not None:
      if self.since is None:
        self.before = cls
      return
    self.after_m += v * dt
    if self.after_m > EXIT_FIND_M:
      self.exited, self.street_t, self.before = False, 0.0, cls   # no trusted street soon after the exit: not its street
      return
    street = self._street(view, limit) and view.get("trust") == TRUST_OK
    if live != LATE or not street:
      self.street_t = self.street_t + dt if street else 0.0
    if self.street_t >= EXIT_STREET_STEADY - 1e-6:
      self.since, self.done, self.on = 0.0, False, False
      self.exited = False
