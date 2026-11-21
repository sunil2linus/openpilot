"""A small frame loop for the part (f) contributors with the planner (no actuator, no car): the contributors get the
ctx the caller builds (the planner's anchor / owner, the actuator events of the frame, the earlier contributions),
the planner takes their proposals by its rules. Not a drive simulation."""
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.planner import Planner, PlanInputs

MPH = M.MPH


def view(t, trust=0, chain=1, link=False, cls=1, matched=True, ramp=None, exits=None, gps="ok", start_ok=False, acc=5.0):
  v = {"v": 2, "t": t, "gps": gps, "trust": trust, "chain": chain, "link": link, "cls": cls, "matched": matched,
       "curves": [], "held": False, "start_ok": start_ok, "acc": acc}
  if ramp is not None:
    v["ramp"] = ramp
  if exits is not None:
    v["exits"] = exits
  return v


def engaged(set_mph):
  return {"kind": "engaged", "before": None, "after": set_mph * MPH, "changed": False}


def driver(direction, after_mph):
  return {"kind": "driver", "direction": direction, "after": after_mph * MPH}


class Rig:
  def __init__(self, contributors, metric=False, dt=0.01):
    self.p = Planner(metric)
    self.cs = list(contributors)
    self.t = 0.0
    self.dt = dt
    self.set = None
    self.out = None
    self.ctx = None

  def step(self, v_mph=60.0, set_mph=None, events=(), n=1, cruise=True, every=None, **ctx_extra):
    for k in range(n):
      self.t += self.dt
      ev = list(events) if k == 0 else []
      if set_mph is not None and k == 0:
        self.set = set_mph * MPH
      if self.set is None:
        self.set = v_mph * MPH
      ctx = {"now": self.t, "v_ego": v_mph * MPH, "cruise_enabled": cruise, "set_speed": self.set,
             "anchor": self.p.anchor, "owner": self.p.owner, "act_events": ev, "contribs": {}}
      ctx.update(ctx_extra)
      for key, val in list(ctx_extra.items()):
        if callable(val) and key != "every":
          ctx[key] = val(self.t)
      if every is not None:
        every(self, ctx)
      contribs = []
      for c in self.cs:
        got = c(ctx)
        if got is not None:
          contribs.append(got)
          ctx["contribs"][got.name] = got
      self.out = self.p.update(PlanInputs(v_ego=v_mph * MPH, set_speed=self.set, cruise_enabled=cruise, main_on=True,
                                          events=ev, contributions=contribs))
      self.ctx = ctx
    return self.out

  @property
  def anchor_mph(self):
    return None if self.p.anchor is None else round(self.p.anchor / MPH, 3)
