"""Mazda steering smoothing: the previous build's 0.10 s filter, its delay in all four places, the frozen setting,
and no change at all for any other car."""
import ast
import importlib.util
import math
import struct
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import prepare as prepare_ti
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.starpilot.car.mazda import model_smoothing as M
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.car.subaru.smoothing import filter_action
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences

DT = 0.05   # 20 Hz model
STEERING = (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021)    # steer without the Torque Interceptor (not dashcam-only)


def mazda(model, ti=False):
  cp = CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)
  return prepare_ti(cp, True) if ti else cp


def old_smooth_value(val, prev_val, tau, dt=DT):
  # the previous build's selfdrive/controls/lib/drive_helpers.py smooth_value, used there with LAT_SMOOTH_SECONDS = 0.1
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val


def other_cars():
  # one platform of every other brand, as the car interfaces build them
  seen, cars = set(), []
  for candidate, interface in sorted(interfaces.items(), key=lambda kv: str(kv[0])):
    cp = interface.get_non_essential_params(candidate)
    if cp.brand != 'mazda' and cp.brand not in seen:
      seen.add(cp.brand)
      cars.append(cp)
  assert len(cars) >= 10
  return cars


def bits(x):
  return struct.pack('<d', float(x))


# ---- which cars ----

@pytest.mark.parametrize('model', list(CAR))
@pytest.mark.parametrize('ti', (False, True))
def test_every_steering_mazda_with_or_without_the_ti(model, ti):
  cp = mazda(model, ti)
  steers = ti or model in STEERING
  assert M.supported(cp) is steers
  assert M.seconds(cp, True) == (0.1 if steers else 0.)
  assert M.seconds(cp, False) == 0. and M.seconds(cp, None) == 0. and M.seconds(cp, 1) == 0.
  if steers:   # the Stage 3 panda bits do not change it
    cp.safetyConfigs[0].safetyParam |= int(MazdaSafetyFlags.AUTO_RESUME | MazdaSafetyFlags.TI_STEER_700)
    assert M.supported(cp)


@pytest.mark.parametrize('veto', ('brand', 'fingerprint', 'gen1', 'passive', 'dashcamOnly', 'notCar', 'angle', 'pid',
                                  'noOutput', 'two'))
def test_other_mazda_configurations_get_nothing(veto):
  cp = mazda(CAR.MAZDA_CX5_2022)
  if veto == 'brand':
    cp.brand = 'subaru'
  elif veto == 'fingerprint':
    cp.carFingerprint = 'MAZDA_CX30'
  elif veto == 'gen1':
    cp.flags = 0
  elif veto in ('passive', 'dashcamOnly', 'notCar'):
    setattr(cp, veto, True)
  elif veto == 'angle':
    cp.steerControlType = structs.CarParams.SteerControlType.angle
  elif veto == 'pid':
    cp.lateralTuning.init('pid')
  elif veto == 'noOutput':
    cp.safetyConfigs[0].safetyModel = structs.CarParams.SafetyModel.noOutput
  else:
    cp.safetyConfigs = [cp.safetyConfigs[0], cp.safetyConfigs[0]]
  assert not M.supported(cp) and M.seconds(cp, True) == 0.


# ---- the filter ----

@pytest.mark.parametrize('speed', (0., .3, .31, 5., 18., 35.))
def test_step_response_is_the_previous_builds_filter(speed):
  tau = M.seconds(mazda(CAR.MAZDA_CX5_2022), True)
  assert tau == 0.1
  previous = log.ModelDataV2.Action(desiredCurvature=0.)
  old = 0.
  for tick in range(40):
    raw = .002 if tick >= 1 else 0.     # a step of the raw model curvature
    action = log.ModelDataV2.Action(desiredCurvature=raw if speed > .3 else previous.desiredCurvature,
                                    desiredAcceleration=-.25, shouldStop=True)
    filter_action(action, previous, speed, tau)
    old = float(old_smooth_value(raw, old, 0.1)) if speed > .3 else old    # the previous build held below 0.3 m/s
    assert action.desiredCurvature == pytest.approx(old, rel=1e-6, abs=1e-12)
    assert action.desiredAcceleration == -.25 and action.shouldStop     # longitudinal untouched
    if speed > .3:   # the closed form: 1 - exp(-t / 0.1)
      assert action.desiredCurvature == pytest.approx(.002 * (1 - math.exp(-DT * tick / .1)), rel=1e-6, abs=1e-12)
    previous = action
  if speed > .3:
    alpha = 1 - math.exp(-.5)
    assert alpha == pytest.approx(.3935, abs=1e-4)      # 39 % of a step in the first frame, 63 % after 0.10 s


# ---- the frozen setting ----

def cp_and_params(directory, cp):
  saved = Params(directory)
  cp_bytes = cp.to_bytes()
  Path(saved.get_param_path('CarParams')).write_bytes(cp_bytes)
  return saved, cp_bytes


def put_config(saved, raw):
  path = Path(saved.get_param_path(C.KEY))
  if raw is None:
    path.unlink(missing_ok=True)
  else:
    path.write_bytes(raw)


@pytest.mark.parametrize('raw, expected', ((None, True), (b'{"version":8}', True), (b'{"version":8,"steer_smoothing":true}', True),
                                           (b'{"version":8,"steer_smoothing":false}', False),
                                           (b'{"version":8,"steer_smoothing":false,"enabled":true}', False),
                                           (b'{"version":7,"steer_smoothing":false}', True),   # invalid: default (on)
                                           (b'not json', True), (b'{"version":8,"steer_smoothing":0}', True)))
def test_setting_on_by_default_off_only_by_a_valid_document(raw, expected):
  with tempfile.TemporaryDirectory() as d:
    saved = Params(d)
    put_config(saved, raw)
    for enabled in (True, False):
      assert VehicleStartupPreferences.read(saved, enabled=enabled).mazda_steer_smoothing is expected
    # it does not depend on the speed control's master switch, and it changes no CarParams
    cp = mazda(CAR.MAZDA_CX5, ti=True)
    before = cp.to_dict()
    VehicleStartupPreferences.read(saved, enabled=True).prepare(cp)
    prepared = mazda(CAR.MAZDA_CX5, ti=True)
    assert VehicleStartupPreferences.read(saved, enabled=True).prepare(prepared).to_dict() == before


def test_context_is_written_before_carparams_and_read_once_by_both():
  with tempfile.TemporaryDirectory() as d, tempfile.TemporaryDirectory() as shm:
    path = Path(shm) / 'starpilot_mazda_d' / 'smoothing-v1.json'
    cp = mazda(CAR.MAZDA_CX5_2022)
    saved, cp_bytes = cp_and_params(d, cp)
    for requested, expected in ((True, .1), (False, 0.)):
      assert M.install(cp, cp_bytes, requested, None, path=path) == expected       # a new drive
      assert M.read_frozen(cp, saved, path=path) == expected                       # modeld
      assert M.read_frozen(cp, saved, path=path) == expected                       # controlsd
    # frozen for the drive: a card restart (CarParams already published with these bytes) keeps the drive's value
    assert M.install(cp, cp_bytes, True, None, path=path) == .1
    assert M.install(cp, cp_bytes, False, cp_bytes, path=path) == .1
    assert M.read_frozen(cp, saved, path=path) == .1          # a modeld or controlsd restart reads the same
    # the next drive (CarParams cleared at the onroad transition) takes the setting again
    assert M.install(cp, cp_bytes, False, None, path=path) == 0. and M.read_frozen(cp, saved, path=path) == 0.
    # a context for other CarParams, a damaged or a missing one: 0.0 (7.0's behavior) in both processes
    assert M.install(cp, cp_bytes, True, None, path=path) == .1
    other = mazda(CAR.MAZDA_CX5_2022, ti=True)
    Path(saved.get_param_path('CarParams')).write_bytes(other.to_bytes())
    assert M.read_frozen(other, saved, path=path) == 0.
    Path(saved.get_param_path('CarParams')).write_bytes(cp_bytes)
    assert M.read_frozen(other, saved, path=path) == 0.        # the process's CP is not the published one
    for damaged in (path.read_bytes().replace(b'0.1', b'0.2'), path.read_bytes() + b' ', b'', b'x' * 300):
      path.write_bytes(damaged)
      assert M.read_frozen(cp, saved, path=path) == 0.
    path.unlink()
    assert M.read_frozen(cp, saved, path=path) == 0.
    assert M.install(cp, cp_bytes, True, cp_bytes, path=path) == .1     # a restart with no context: written again
    Path(saved.get_param_path('CarParams')).unlink()
    assert M.read_frozen(cp, saved, path=path) == 0.


def test_unwritable_context_is_off_for_both_and_leaves_nothing_stale():
  with tempfile.TemporaryDirectory() as d, tempfile.TemporaryDirectory() as shm:
    cp = mazda(CAR.MAZDA_CX5_2022)
    saved, cp_bytes = cp_and_params(d, cp)
    blocker = Path(shm) / 'file'
    blocker.write_bytes(b'')
    assert M.install(cp, cp_bytes, True, None, path=blocker / 'smoothing-v1.json') == 0.
    path = Path(shm) / 'ctx' / 'smoothing-v1.json'
    assert M.install(cp, cp_bytes, True, None, path=path) == .1
    path.parent.chmod(0o500)
    try:
      if M.install(cp, cp_bytes, False, None, path=path) == 0.:   # root may still write: then it is simply 0.0
        assert M.read_frozen(cp, saved, path=path) == 0.
    finally:
      path.parent.chmod(0o700)


# ---- every other car: nothing changes, bit for bit ----

class NoParams:
  def __getattr__(self, name):
    raise AssertionError(f'Params.{name} used for a non-Mazda car')


def test_other_brands_read_and_write_nothing_and_get_zero():
  with tempfile.TemporaryDirectory() as shm:
    path = Path(shm) / 'starpilot_mazda_d' / 'smoothing-v1.json'
    for cp in other_cars():
      assert not M.supported(cp), cp.brand
      assert M.seconds(cp, True) == 0.
      assert M.install(cp, cp.to_bytes(), True, None, path=path) == 0.
      assert M.read_frozen(cp, NoParams(), path=path) == 0.
    assert not path.parent.exists()


def source(module):
  spec = importlib.util.find_spec(module)
  assert spec is not None and spec.origin is not None
  return ast.parse(Path(spec.origin).read_text()), spec.origin


def assigns(tree, names):
  return [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in n.targets)]


def test_modeld_has_the_tau_in_the_filter_and_the_delay():
  tree, _ = source('openpilot.selfdrive.modeld.modeld')
  text = {ast.unparse(n) for n in assigns(tree, {'mazda_tau', 'lat_delay'})}
  assert "mazda_tau = mazda_smoothing(CP, params)" in text
  assert ("lat_delay = sm['lateralDelay'].lateralDelay + LAT_SMOOTH_SECONDS + subaru_tau + volkswagen_tau + kona_tau + mazda_tau"
          in text)
  filters = [ast.unparse(n) for n in ast.walk(tree) if isinstance(n, ast.Call) and ast.unparse(n.func) == 'filter_action']
  assert filters == ['filter_action(action, prev_action, v_ego, subaru_tau + volkswagen_tau + kona_tau + mazda_tau)']
  imports = [ast.unparse(n) for n in tree.body if isinstance(n, ast.ImportFrom)]
  assert 'from openpilot.starpilot.car.mazda.model_smoothing import read_frozen as mazda_smoothing' in imports
  # read once, before the frame loop (frozen for the process)
  main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
  loop = [n for n in main.body if isinstance(n, ast.While)][-1]
  tau = assigns(main, {'mazda_tau'})
  assert len(tau) == 1 and tau[0].lineno < loop.lineno


def test_controlsd_has_the_tau_in_the_torque_and_lane_centering_delays():
  tree, _ = source('openpilot.selfdrive.controls.controlsd')
  text = [ast.unparse(n) for n in ast.walk(tree) if isinstance(n, (ast.Assign, ast.AugAssign))]
  assert "self.mazda_smoothing = mazda_smoothing(self.CP, self.params)" in text
  assert "mazda_tau = getattr(self, 'mazda_smoothing', 0.0)" in text
  assert "lane_delay += kona_tau + mazda_tau" in text
  torque = "lat_delay = self.sm['lateralDelay'].lateralDelay + LAT_SMOOTH_SECONDS + "
  torque += "subaru_smooth_seconds(CS.vEgo, control_maximum(self)) + kona_tau + mazda_tau"
  assert torque in text
  init = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '__init__' and
              any(ast.unparse(x) == "self.mazda_smoothing = mazda_smoothing(self.CP, self.params)" for x in ast.walk(n)))
  assert init is not None   # read once, at controlsd start


def test_card_installs_the_context_before_it_publishes_carparams():
  tree, _ = source('openpilot.selfdrive.car.card')
  calls = [(n.lineno, ast.unparse(n)) for n in ast.walk(tree) if isinstance(n, ast.Call)]
  install = [line for line, text in calls if text.startswith('install_mazda_smoothing(')]
  publish = [line for line, text in calls if text == "self.params.put('CarParams', cp_bytes, block=True)"]
  assert len(install) == 1 and len(publish) == 1 and install[0] < publish[0]
  assert ("install_mazda_smoothing(self.CP, cp_bytes, startup_preferences.mazda_steer_smoothing, self.params.get('CarParams'))"
          in [text for _, text in calls])


def run_production(tree, filename, names, env, extra=()):
  nodes = assigns(tree, names) + list(extra)
  for node in sorted(nodes, key=lambda n: n.lineno):
    exec(compile(ast.Module(body=[node], type_ignores=[]), filename, 'exec'), env)
  return env


def modeld_env(cp, tau_reader, delay, speed):
  action = log.ModelDataV2.Action(desiredCurvature=.001, desiredAcceleration=-.25, shouldStop=True)
  return {'CP': cp, 'params': NoParams() if cp.brand != 'mazda' else None, 'receipt_owner': SimpleNamespace(load=object()),
          'v_ego': speed, 'subaru_smoothing': 0., 'subaru_smooth_seconds': lambda *args: 0., 'volkswagen_smoothing': lambda *args: 0.,
          'kona_smoothing': lambda *args: 0., 'mazda_smoothing': tau_reader, 'LAT_SMOOTH_SECONDS': 0.,
          'sm': {'lateralDelay': SimpleNamespace(lateralDelay=delay)}, 'action': action,
          'prev_action': log.ModelDataV2.Action(desiredCurvature=0.), 'filter_action': filter_action,
          'frame_delay': .05, 'action_delay': .025, 'inputs': {}, 'np': np}


def modeld_join(tree):
  return [n for n in ast.walk(tree) if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call) and
          isinstance(n.value.func, ast.Name) and n.value.func.id == 'filter_action']


NAMES = {'kona_tau', 'mazda_tau', 'subaru_tau', 'volkswagen_tau', 'lat_delay', 'lat_action_t'}


@pytest.mark.parametrize('delay', (.2893982231616974, .3, .45))
@pytest.mark.parametrize('speed', (.2, 5., 18., 31.))
def test_non_mazda_actions_and_delays_are_bit_identical_to_before(delay, speed):
  tree, filename = source('openpilot.selfdrive.modeld.modeld')
  ctree, cfilename = source('openpilot.selfdrive.controls.controlsd')
  lane = next(n for n in ast.walk(ctree) if isinstance(n, ast.Assign) and
              any(isinstance(t, ast.Name) and t.id == 'lane_delay' for t in n.targets) and isinstance(n.value, ast.Call))
  guard = next(n for n in ast.walk(ctree) if isinstance(n, ast.If) and ast.unparse(n.test) == 'lane_delay is not None')
  for cp in other_cars():
    env = run_production(tree, filename, NAMES, modeld_env(cp, M.read_frozen, delay, speed), modeld_join(tree))
    # before this change: lat_delay = delay + LAT_SMOOTH_SECONDS + subaru + volkswagen + kona; filter tau = their sum (0 here)
    before_delay = delay + 0. + 0. + 0. + 0.
    assert env['mazda_tau'] == 0. and bits(env['lat_delay']) == bits(before_delay), cp.brand
    assert bits(env['lat_action_t']) == bits(before_delay + .05 + .025)
    before = log.ModelDataV2.Action(desiredCurvature=.001, desiredAcceleration=-.25, shouldStop=True)
    filter_action(before, log.ModelDataV2.Action(desiredCurvature=0.), speed, 0. + 0. + 0.)
    assert bits(env['action'].desiredCurvature) == bits(before.desiredCurvature) == bits(np.float32(.001))   # untouched (float32)
    assert env['action'].desiredAcceleration == before.desiredAcceleration and env['action'].shouldStop
    # controlsd: the torque controller's and Lane Centering's delays
    cenv = {'self': SimpleNamespace(kona_smoothing=0., mazda_smoothing=M.read_frozen(cp, NoParams()),
                                    sm={'lateralDelay': SimpleNamespace(lateralDelay=delay)}),
            'CS': SimpleNamespace(vEgo=speed), 'LAT_SMOOTH_SECONDS': 0., 'subaru_smooth_seconds': lambda *args: 0.,
            'control_maximum': lambda *args: 0., 'current_delay': lambda *args: delay, 'lane_now_ns': 1}
    run_production(ctree, cfilename, {'kona_tau', 'mazda_tau', 'lat_delay'}, cenv, [lane, guard])
    assert bits(cenv['lat_delay']) == bits(delay + 0. + 0. + 0.) and bits(cenv['lane_delay']) == bits(delay + 0.)


@pytest.mark.parametrize('ti', (False, True))
def test_mazda_gets_the_filter_and_the_matching_delays_everywhere(ti):
  tree, filename = source('openpilot.selfdrive.modeld.modeld')
  ctree, cfilename = source('openpilot.selfdrive.controls.controlsd')
  cp = mazda(CAR.MAZDA_CX5 if ti else CAR.MAZDA_CX5_2022, ti)
  env = run_production(tree, filename, NAMES, modeld_env(cp, lambda cp, params: M.seconds(cp, True), .3, 18.), modeld_join(tree))
  assert env['mazda_tau'] == .1
  assert env['lat_delay'] == pytest.approx(.4) and env['lat_action_t'] == pytest.approx(.475)
  assert env['action'].desiredCurvature == pytest.approx(float(old_smooth_value(.001, 0., .1)), rel=1e-6)   # float32
  lane = next(n for n in ast.walk(ctree) if isinstance(n, ast.Assign) and
              any(isinstance(t, ast.Name) and t.id == 'lane_delay' for t in n.targets) and isinstance(n.value, ast.Call))
  guard = next(n for n in ast.walk(ctree) if isinstance(n, ast.If) and ast.unparse(n.test) == 'lane_delay is not None')
  cenv = {'self': SimpleNamespace(kona_smoothing=0., mazda_smoothing=.1, sm={'lateralDelay': SimpleNamespace(lateralDelay=.3)}),
          'CS': SimpleNamespace(vEgo=18.), 'LAT_SMOOTH_SECONDS': 0., 'subaru_smooth_seconds': lambda *args: 0.,
          'control_maximum': lambda *args: 0., 'current_delay': lambda *args: .3, 'lane_now_ns': 1}
  run_production(ctree, cfilename, {'kona_tau', 'mazda_tau', 'lat_delay'}, cenv, [lane, guard])
  assert cenv['lat_delay'] == pytest.approx(.4) and cenv['lane_delay'] == pytest.approx(.4)
