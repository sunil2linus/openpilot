"""The whole chain without card: map view -> curve contributor -> planner -> actuator -> the MRCC model, on a road
with one curve (a 72 m entry pinch, then a 92 m body). Not a drive simulation: it checks the order of things (slow
before the curve, never past his speed, the LKAS floor, restore after, his button wins, test mode never presses)."""
import math

from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed import cruise_model as M
from openpilot.starpilot.car.mazda.speed.caller import SpeedControl
from openpilot.starpilot.car.mazda.speed.contributors import Sources
from openpilot.starpilot.car.mazda.speed.curve_plan import CurvePlan
from openpilot.starpilot.car.mazda.speed.tests.mrcc_sim import Mrcc

MPH = M.MPH
CURVE = [(1200.0, 72.0), (1260.0, 92.0), (1330.0, 92.0)]   # m along the road, radius


class Road:
  def __init__(self, set_mph=58, curve=CURVE):
    self.car = Mrcc(set_mph, latency=12)
    self.car.track = False
    self.v = (set_mph - 2) * MPH
    self.s = 0.0
    self.t = 0.0
    self.curve = curve
    self.lag = []

  def view(self):
    pts = [[s - self.s, r] for s, r in self.curve if s - self.s > -200]
    return {"v": 1, "t": self.t, "gps": "ok", "trust": 0, "held": False,
            "curves": [{"id": "c", "pts": pts, "ramp": False, "adv": 0.0}] if pts else []}

  def tick(self):
    self.t += 0.01
    self.car.tick()
    # the cruise: after M.LAG it answers the set speed (true = dash - offset) with the measured accel / decel
    self.lag.append(self.car.set - M.DASH_OFFSET)
    want = self.lag[-int(M.LAG / 0.01)] if len(self.lag) > M.LAG / 0.01 else self.lag[0]
    if want < self.v:
      a = -max(M.cruise_decel(self.v - want), 0.2 if self.v - want > 0.3 else 0.0)
    else:
      a = M.cruise_accel(want - self.v) or (0.2 if want - self.v > 0.3 else 0.0)
    self.v = max(self.v + a * 0.01, 0.0)
    self.s += self.v * 0.01
    self.car.v = self.v


def run(cfg, road, frames, every=None):
  plan = CurvePlan(cfg, presses=cfg.presses("curves"))
  src = Sources(want_model=False, sm=None, read_view=road.view, clock=lambda: road.t)
  ctl = SpeedControl(None, cfg, None, False, contributors=[plan], sources=src)
  trace = []
  for k in range(frames):
    road.tick()
    d = {"cruise_enabled": True, "main_on": True, "set_speed": road.car.set, "v_ego": road.v, "gas": False,
         "brake": False, "standstill": False, "set_plus": False, "set_minus": False, "resume": False, "cancel": False,
         "wheel_counter": road.car.counter, "other_button": False, "engaged": True, "lkas_blocked": False,
         "blinker": False}
    if every is not None:
      every(k, d, road)
    r = ctl.step(d)
    if r is not None:
      road.car.receive(*r)
    trace.append((road.s, road.v, road.car.set_units))
  return trace, ctl


def at(trace, s):
  return min(trace, key=lambda x: abs(x[0] - s))


def test_slows_before_the_pinch_and_restores_after():
  road = Road()
  trace, ctl = run(C.Config(enabled=True, curves="on"), road, 9000)
  pinch_v = math.sqrt(2.2 * 72)
  s_pinch, v_pinch, _ = at(trace, CURVE[0][0])
  assert v_pinch < pinch_v + 1.5, (v_pinch, pinch_v)
  assert min(x[2] for x in trace) >= 29 - 1e-6 or min(x[2] for x in trace) >= round(pinch_v / MPH) - 6
  assert max(x[2] for x in trace) <= 58 + 1e-6                     # never above his speed
  assert road.car.sent and any(b == "SET_MINUS" for _, b, *_ in road.car.sent)
  assert abs(trace[-1][2] - 58) < 1e-6                              # back to his speed after the curve
  first_press = road.car.sent[0][0]
  assert trace[first_press][0] < CURVE[0][0] - 300                  # started well before the curve


def test_lkas_floor_for_a_curve_that_allows_it():
  road = Road(set_mph=70, curve=[(1500.0, 260.0), (1600.0, 260.0)])   # sqrt(2.2 * 260) = 23.9 m/s = 53 mph
  trace, _ = run(C.Config(enabled=True, curves="on"), road, 6000)
  assert min(x[2] for x in trace) >= 29
  assert min(x[1] for x in trace) > 30 * MPH


def test_test_mode_never_presses():
  road = Road()
  trace, ctl = run(C.Config(enabled=True, curves="test"), road, 6000)
  assert road.car.sent == [] and all(abs(x[2] - 58) < 1e-6 for x in trace)


def test_his_set_minus_while_we_lower_is_his_speed_and_blocks_the_curve():
  road = Road()
  K = 4300                                       # we are already lowering for the curve here

  def driver(k, d, road):
    if k == K:
      road.car._move(-4)                         # his own SET- (a short hold)
    if K - 10 <= k < K + 10:
      d["set_minus"] = True
  trace, ctl = run(C.Config(enabled=True, curves="on"), road, 9000, every=driver)
  assert trace[K - 20][2] < 58                   # (we had started lowering)
  his = trace[K + 50][2]
  during = [x for k, x in enumerate(trace) if k > K + 50 and x[0] < CURVE[-1][0] + 18]
  assert during and all(abs(x[2] - his) < 1e-6 for x in during)    # held where he put it: no lower, no raise
  assert abs(ctl.planner.anchor - his * MPH) < 0.5 * MPH
  assert max(x[2] for x in trace[K + 50:]) <= his + 1e-6           # never back up past his new speed


def test_a_set_minus_before_any_curve_is_his_new_speed_and_curves_still_apply():
  road = Road()

  def driver(k, d, road):
    if k == 1000:
      road.car._move(-10)
    if 990 <= k < 1010:
      d["set_minus"] = True
  trace, ctl = run(C.Config(enabled=True, curves="on"), road, 9000, every=driver)
  assert abs(ctl.planner.anchor - 48 * MPH) < 0.5 * MPH
  assert min(x[2] for x in trace[1100:]) < 48                      # the curve (a rule he turned on) still lowers
  assert max(x[2] for x in trace[1100:]) <= 48 + 1e-6 and abs(trace[-1][2] - 48) < 1e-6
