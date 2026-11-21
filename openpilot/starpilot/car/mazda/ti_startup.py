"""Startup of a Gen1 Mazda with a Torque Interceptor (TI): route panda bus 1 to the OBD-II pins before ControlsReady.

The TI talks on the OBD-II CAN pins. The panda only routes bus 1 there in SAFETY_MAZDA with param bit 8 (main.c), but
pandad switches to the car safety mode only after card sets ControlsReady, and card sets ControlsReady only once
carState is valid, which with a TI needs TI_FEEDBACK on bus 1. After fingerprinting the panda is in ELM327 param 1
(normal CAN wiring), so without help TI_FEEDBACK never arrives and the car never leaves ELM327.

So, after the car interface exists and before the first control step, card asks for OBD multiplexing again (the
existing ObdMultiplexingEnabled / ObdMultiplexingChanged handshake): pandad sets ELM327 param 0 = CAN_MODE_OBD_CAN2,
relay still open and no car messages allowed out. TI_FEEDBACK then arrives, carState becomes valid, card sets
ControlsReady, and pandad sets SAFETY_MAZDA param 8, which keeps OBD_CAN2. Once pandaStates shows that mode,
ObdMultiplexingEnabled is cleared again (non-blocking; pandad no longer reads it while its safety mode is set), so a
later card restart in the same drive does not wait in fingerprint()'s set_obd_multiplexing(False).

Fail closed: without a TI on the OBD pins carState never becomes valid, ControlsReady is never set and the panda
stays in ELM327 (relay open, the car's own LKAS untouched); the driver sees the CAN error alert and cannot engage.
"""
import time

from opendbc.car import structs
from opendbc.car.mazda.torque_interceptor import installed
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.common.swaglog import cloudlog

MAZDA_NAME = 'mazda'
MAZDA_RAW = int(structs.CarParams.SafetyModel.mazda)  # 13
HANDSHAKE_TIMEOUT_S = 2.0


def panda_ti_configured(panda_states) -> bool:
  """The internal panda (index 0) runs the Mazda safety with the TI bit."""
  try:
    if not panda_states:
      return False
    state = panda_states[0]
    model = state.safetyModel
    mazda = str(model) == MAZDA_NAME or getattr(model, 'raw', model) == MAZDA_RAW
    return bool(mazda and int(state.safetyParam) & int(MazdaSafetyFlags.TORQUE_INTERCEPTOR))
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


def required(cp) -> bool:
  return bool(installed(cp) and not cp.passive and not cp.notCar)


class TorqueInterceptorStartup:
  def __init__(self, params, *, timeout_s=HANDSHAKE_TIMEOUT_S, clock=time.monotonic, sleep=time.sleep):
    self.params = params
    self.timeout_s = timeout_s
    self.clock = clock
    self.sleep = sleep
    self.pending = False
    self.outcome = None

  def request(self, cp, panda_states=None) -> str:
    """Called once after get_car(), before the control loop. Never waits longer than timeout_s."""
    if not required(cp):
      self.outcome = 'not_required'
    elif panda_ti_configured(panda_states) or (panda_states is None and self.params.get_bool("ControlsReady")):
      # card restarted while pandad already runs SAFETY_MAZDA with the TI (OBD_CAN2 kept): change nothing,
      # pandad does not read the multiplexing request in that state.
      self.outcome = 'already_configured'
      self.pending = True  # still clear a stale request once pandaStates confirms
    else:
      self.params.remove("ObdMultiplexingChanged")
      self.params.put_bool("ObdMultiplexingEnabled", True, block=True)
      deadline = self.clock() + self.timeout_s
      while not self.params.get_bool("ObdMultiplexingChanged"):
        if self.clock() >= deadline:
          break
        self.sleep(0.01)
      self.outcome = 'requested' if self.params.get_bool("ObdMultiplexingChanged") else 'timeout'
      self.pending = True
    cloudlog.warning(f"Mazda TI startup: OBD CAN2 request {self.outcome}")
    return self.outcome

  def maintain(self, panda_states) -> None:
    """Every card step: once the panda runs SAFETY_MAZDA with the TI, withdraw the multiplexing request."""
    if self.pending and panda_ti_configured(panda_states):
      self.pending = False
      if self.params.get_bool("ObdMultiplexingEnabled"):
        self.params.put_bool("ObdMultiplexingEnabled", False)
      cloudlog.warning("Mazda TI startup: panda in Mazda safety with the TI, OBD multiplexing request withdrawn")
