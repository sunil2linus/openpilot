"""The speed limit follow and the merge through the whole chain (Sources -> contributors -> planner -> actuator -> the
MRCC model): On presses whole dash steps to the limit + offset / boosts and comes back to the target; Test computes
the same and never presses."""
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.mazda.speed.caller import SpeedControl
from openpilot.starpilot.car.mazda.speed.contributors import Sources
from openpilot.starpilot.car.mazda.speed.limits import SpeedLimit
from openpilot.starpilot.car.mazda.speed.merge import Merge
from openpilot.starpilot.car.mazda.speed.tests.rig import MPH, view
from openpilot.starpilot.car.mazda.speed.tests.test_speed_control import Road

PATH = (list(range(0, 200, 5)), [0.0] * 40)


def run(road, contributors, frames, limit_mph, view_fn, upcoming=0.0):
  src = Sources(want_model=False, sm=None, read_view=lambda: view_fn(road.t), clock=lambda: road.t)
  ctl = SpeedControl(None, C.Config(enabled=True), None, False, contributors=contributors, sources=src)
  trace = []
  for _ in range(frames):
    road.tick()
    src.slc = {"limit": None if limit_mph is None else limit_mph(road.t) * MPH, "upcoming": upcoming * MPH, "t": road.t}
    src.path, src.model_t = PATH, road.t
    d = {"cruise_enabled": True, "main_on": True, "set_speed": road.car.set, "v_ego": road.v, "gas": False,
         "brake": False, "standstill": False, "set_plus": False, "set_minus": False, "resume": False, "cancel": False,
         "wheel_counter": road.car.counter, "other_button": False, "engaged": True, "lkas_blocked": False,
         "blinker": False, "left_blinker": False, "right_blinker": False, "radar_lead": False}
    r = ctl.step(d)
    if r is not None:
      road.car.receive(*r)
    trace.append((road.t, road.v, road.car.set_units))
  return trace, ctl


def test_a_lower_limit_takes_an_engaged_in_band_speed_down_in_whole_steps():
  for mode in ("on", "test"):
    cfg = C.Config(enabled=True, speed_limit=mode)
    road = Road(set_mph=73, curve=[])
    road.v = 71 * MPH
    trace, ctl = run(road, [SpeedLimit(cfg, presses=cfg.presses("speed_limit"))], 4000,
                     lambda t: 70 if t < 8 else 55, lambda t: view(t))
    if mode == "on":
      assert abs(trace[-1][2] - 58) < 1e-6 and road.car.sent
      assert all(abs(x[2] - round(x[2])) < 1e-6 for x in trace)
    else:
      assert road.car.sent == [] and all(abs(x[2] - 73) < 1e-6 for x in trace)


def test_the_merge_boosts_and_ends_at_the_target():
  for mode in ("on", "test"):
    cfg = C.Config(enabled=True, merge=mode)
    road = Road(set_mph=45, curve=[])
    road.v = 43 * MPH
    rv = lambda t: view(t, link=True, cls=2, chain=7, ramp={"id": 7, "hwy": True, "spd": 70 * MPH})  # noqa: E731
    trace, ctl = run(road, [Merge(cfg, presses=cfg.presses("merge"))], 6000, lambda t: 45, rv)
    if mode == "on":
      top = max(x[2] for x in trace)
      assert 80 <= top <= 90 + 1e-6, top            # the boost, capped at the car's max
      assert abs(trace[-1][2] - 73) < 1e-6, trace[-1]
    else:
      assert road.car.sent == [] and all(abs(x[2] - 45) < 1e-6 for x in trace)
