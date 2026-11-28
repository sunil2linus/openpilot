"""Mazda Torque Interceptor AOL in the real SelfdriveD / Controls runtime (the AOL authority harness), and the card intent.

From the owner's first drive with the TI profile (drive 00000002):
* a stop with the cruise engaged raised Controls Mismatch (369.4 s): the panda withdraws lateral at a standstill;
* the host latch was lost on the first moving carState after a stop (128.2 / 446.7 / 567.4 / 671.2 s): the native
  status still showed the stop;
* a host source expiry during a brake cancel raised Controls Mismatch (961.6 s): the brake's pedalPressed was not
  accepted as the stock cruise's own user event.
"""
import os
from contextlib import ExitStack
from types import SimpleNamespace
from unittest import mock

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import prepare
from opendbc.car.mazda.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import AolSettings, read_settings
from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import authority_case
from openpilot.starpilot.aol.tests.test_ioniq_availability import car_state
from openpilot.starpilot.aol.vehicle import create_intent, policy_for
from openpilot.starpilot.car.mazda import aol as mazda_aol
from openpilot.starpilot.car.mazda.stock_intent import STOP_NATIVE_GRACE_NS

GearShifter = structs.CarState.GearShifter


def ti_cp():
  cp = prepare(CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False), True)
  cp.fingerprintSource = structs.CarParams.FingerprintSource.fixed
  return cp


def stock_cp():
  return CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], False, False, False)


def marked(cp):
  policy = policy_for(cp)
  cp.safetyConfigs[0].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  return cp


def runtime(cleanup, cp, engaged):
  cleanup.enter_context(OpenpilotPrefix())
  cleanup.enter_context(mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}))
  Params().put_bool('AlwaysOnLateral', True, block=True)
  case = authority_case()
  cleanup.callback(case.doCleanups)
  sd, controls, drive = case._authority_callers(engaged, cp=cp, genuine_events=True)
  sd.aol_settings = read_settings(Params())
  return sd, drive


def state(*, stopped=False, engaged=False, brake=False, speed=8.0):
  cs = car_state(brake=brake)
  cs.cruiseState.available = True
  cs.cruiseState.enabled = engaged
  cs.vEgo = cs.vEgoRaw = 0.0 if stopped else speed
  cs.standstill = stopped
  return cs


def names(sd):
  return [str(e.name) for e in sd.events.to_msg()]


@pytest.mark.parametrize('engaged', [True, False])
def test_stop_pauses_and_resumes_without_mismatch(engaged):
  with ExitStack() as cleanup:
    sd, drive = runtime(cleanup, ti_cp(), engaged)
    for tick in (0, 1, 2):
      axis, st, _ = drive(tick, native_request_long=False, observed_state=state(engaged=engaged))
    assert axis.lateralActive
    for tick in (3, 4, 5):   # stopped: the panda withdraws lateral (vehicle_moving)
      axis, st, _ = drive(tick, native_request_long=False, allowed=False, observed_state=state(stopped=True, engaged=engaged))
      assert not axis.lateralActive and axis.desiredLateral and str(axis.faultReason) == 'none'
      assert 'controlsMismatch' not in names(sd) and not sd.aol_authority_lost
      assert st.enabled == engaged
    for tick in (6, 7):      # rolling again
      axis, st, _ = drive(tick, native_request_long=False, observed_state=state(engaged=engaged))
      assert axis.lateralActive and str(axis.faultReason) == 'none' and st.enabled == engaged


def test_denial_while_moving_is_still_lost_permission():
  with ExitStack() as cleanup:
    sd, drive = runtime(cleanup, ti_cp(), True)
    for tick in (0, 1, 2):
      axis, _, _ = drive(tick, native_request_long=False, observed_state=state(engaged=True))
    axis, st, _ = drive(3, native_request_long=False, allowed=False, observed_state=state(engaged=True, speed=0.31))
    assert str(axis.faultReason) == 'critical' and 'controlsMismatch' in names(sd) and not st.enabled


def test_stock_profile_stop_unchanged():
  # the 2022 CX-5 (stock profile) keeps 7.0's behaviour: the new rule is for the TI profile only
  with ExitStack() as cleanup:
    sd, drive = runtime(cleanup, stock_cp(), True)
    for tick in (0, 1, 2):
      drive(tick, native_request_long=False, observed_state=state(engaged=True))
    axis, _, _ = drive(3, native_request_long=False, allowed=False, observed_state=state(stopped=True, engaged=True))
    assert str(axis.faultReason) == 'critical'


@pytest.mark.parametrize('pedals', ['none', 'brake', 'brake_steering', 'gas'])
def test_source_expiry_during_aol_with_or_without_the_brake_is_a_pause(pedals):
  # 961.6 s: AOL steering, the brake (and the driver's hands) on, one host step 32 ms after its carState
  brake, steering, gas = pedals.startswith('brake'), pedals == 'brake_steering', pedals == 'gas'
  with ExitStack() as cleanup:
    sd, drive = runtime(cleanup, ti_cp(), False)
    for tick in (0, 1, 2):
      last = tick == 2
      axis, _, _ = drive(tick, native_request_long=False, brake=brake and last, steering=steering and last, gas=gas and last)
    assert axis.lateralActive or steering
    axis, _, _ = drive(6, timeout=True, brake=brake, steering=steering, gas=gas, native_request_long=False)   # 40 ms old
    assert not axis.lateralActive and str(axis.faultReason) == 'availabilityPause'
    assert not sd.aol_authority_lost and 'controlsMismatch' not in names(sd)


def test_stock_profile_source_expiry_with_the_brake_unchanged():
  with ExitStack() as cleanup:
    sd, drive = runtime(cleanup, stock_cp(), False)
    for tick in (0, 1, 2):
      drive(tick, native_request_long=False, brake=tick == 2)
    axis, _, _ = drive(6, timeout=True, brake=True, native_request_long=False)
    assert str(axis.faultReason) == 'critical'


# ---- card intent: a native denial that belongs to a stop is no rejection (TI profile only) ----

def cs(**fields):
  s = structs.CarState(canValid=True, gearShifter=GearShifter.drive, **fields)
  s.cruiseState.available = True
  return s.as_reader()


def armed_intent(cp):
  intent = create_intent(cp, AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0)), policy_for(cp))
  intent.startup_main_pending = False
  intent.allowed_latch = True
  intent._last_latch_edge_ns = 1
  return intent


@pytest.mark.parametrize('ti', [True, False])
def test_rejection_from_a_stop_time_native_status(ti):
  intent = armed_intent(marked(ti_cp() if ti else stock_cp()))
  now = 1_000_000_000
  for _ in range(5):   # stopped
    now += 10_000_000
    intent.update(cs(standstill=True), fault_active=False, now_ns=now)
  stopped_native = now - 60_000_000           # the native status read while the car was stopped
  now += 10_000_000                           # the first moving carState: the status still shows the stop
  intent.update(cs(vEgo=0.15), fault_active=False, now_ns=now, native_rejection_ns=stopped_native)
  assert intent.allowed_latch == ti           # stock profile: unchanged (cleared)


def test_a_later_denial_still_rejects():
  intent = armed_intent(marked(ti_cp()))
  now = 1_000_000_000
  for _ in range(5):
    now += 10_000_000
    intent.update(cs(standstill=True), fault_active=False, now_ns=now)
  stop_end = now
  for _ in range(20):
    now += 10_000_000
    intent.update(cs(vEgo=1.0), fault_active=False, now_ns=now)
  intent.update(cs(vEgo=1.0), fault_active=False, now_ns=now + 10_000_000, native_rejection_ns=stop_end + STOP_NATIVE_GRACE_NS + 1)
  assert not intent.allowed_latch


def test_full_engagement_ignores_a_stop_time_rejection():
  intent = armed_intent(marked(ti_cp()))
  intent.allowed_latch = False
  now = 1_000_000_000
  for _ in range(3):
    now += 10_000_000
    intent.update(cs(standstill=True), fault_active=False, now_ns=now)
  kwargs = dict(enabled=False, active=False, source_ns=now, now_ns=now, fault_active=False, observation_valid=True)
  intent.observe_full_engagement(cs(standstill=True), **kwargs)
  now += 10_000_000
  engaged = structs.CarState(canValid=True, gearShifter=GearShifter.drive, standstill=True)
  engaged.cruiseState.available = engaged.cruiseState.enabled = True
  intent.observe_full_engagement(engaged.as_reader(), enabled=True, active=True, source_ns=now, now_ns=now, fault_active=False,
                                 observation_valid=True, native_rejection_ns=now - 50_000_000)
  assert intent.allowed_latch


# ---- the stop rule used by SelfdriveD ----

def native(requested=True, allowed=False):
  return SimpleNamespace(requestedLateral=requested, lateralAllowed=allowed, pandaInventory=[object()])


@pytest.mark.parametrize('change, expected', [
  ({}, True), ({'vEgoRaw': 0.31}, False), ({'gearShifter': GearShifter.reverse}, False),
  ({'steerFaultTemporary': True}, False), ({'steerFaultPermanent': True}, False), ({'accFaulted': True}, False),
  ({'canValid': False}, False), ({'native_allowed': True}, False), ({'native_requested': False}, False), ({'stock': True}, False),
])
def test_stop_rule(change, expected):
  change = dict(change)
  cp = marked(stock_cp() if change.pop('stock', False) else ti_cp())
  n = native(requested=change.pop('native_requested', True), allowed=change.pop('native_allowed', False))
  fields = dict(canValid=True, gearShifter=GearShifter.drive, vEgoRaw=0.0, standstill=True)
  fields.update(change)
  state = structs.CarState(**fields).as_reader()
  with mock.patch('openpilot.starpilot.aol.transport_pause._configured_transport_native', return_value=n):
    assert mazda_aol.ti_expected_stopped_lateral_denial(cp, state, n, sm=None, axis_session_id='s', now_ns=1) == expected
  with mock.patch('openpilot.starpilot.aol.transport_pause._configured_transport_native', return_value=None):
    assert not mazda_aol.ti_expected_stopped_lateral_denial(cp, state, n, sm=None, axis_session_id='s', now_ns=1)
