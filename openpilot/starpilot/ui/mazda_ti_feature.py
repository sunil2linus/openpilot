"""Saved Torque Interceptor selection for Gen1 Mazda vehicles (hardware declaration, applies after the next startup)."""
from opendbc.car import structs
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.ui.feature_settings_state import FeatureRow


KEYS = frozenset(('TorqueInterceptorEnabled',))
ROWS = (('TorqueInterceptorEnabled', 'Torque Interceptor (Mazda)',
         'Turn on only if a MoreTorque Torque Interceptor is installed: StarPilot then also steers below 28 mph. '
         'Applies after the next startup.'),)


# The saved CarParams of a 2017-21 Gen1 Mazda without the TI are dashcam/passive with [noOutput, 0] (card.py replaces the
# safety config), so the row also accepts that configuration, not only the live (mazda, 0). The Stage 3 bits (gas
# auto-resume, steering test; opendbc/car/mazda/speed_flags.py) come from these settings and do not change the car.
CONFIGURATIONS = ((int(structs.CarParams.SafetyModel.mazda), 0), (int(structs.CarParams.SafetyModel.noOutput), 0))


def _model_raw(model):
  return int(getattr(model, 'raw', model))  # capnp enum reader or plain int


def capable(cp) -> bool:
  from opendbc.car.mazda.torque_interceptor import TI_CARS, installed
  from opendbc.car.mazda.values import MazdaFlags
  try:
    if cp is None or cp.brand != 'mazda' or cp.notCar:
      return False
    if installed(cp):
      return True
    from opendbc.car.mazda.speed_flags import STAGE3_BITS
    configurations = tuple((_model_raw(c.safetyModel), int(c.safetyParam) & ~STAGE3_BITS) for c in cp.safetyConfigs)
    return bool(cp.carFingerprint in TI_CARS and int(cp.flags) == int(MazdaFlags.GEN1) and
                not cp.openpilotLongitudinalControl and cp.pcmCruise and
                len(configurations) == 1 and configurations[0] in CONFIGURATIONS)
  except (AttributeError, TypeError, ValueError):
    return False


class MazdaTorqueInterceptorFeature:
  def __init__(self, owner):
    self.params = owner.params
    self.authority = owner.authority
    self.vehicle_params = owner.vehicle_params
    self.vehicle_fingerprint = owner.vehicle_fingerprint

  def capability(self):
    cp = self.vehicle_params()
    # The same answer for the dashcam (TI off) and the running TI CarParams.
    if not capable(cp):
      return None
    return (str(cp.carFingerprint), str(cp.brand), 'mazda-gen1-ti')

  def rows(self, parked):
    capability = self.capability()
    if capability is None:
      return ()
    rows = []
    for key, label, reason in ROWS:
      raw, readable = read_saved(self.params, key, 8)
      valid = readable and raw in (None, b'0', b'1')
      # An invalid saved value (e.g. "1\n" from a shell echo) is treated as Off at startup and can be repaired to Off.
      rows.append(FeatureRow(key, label, ('On' if raw == b'1' else 'Off') if valid else 'Invalid saved choice',
                             source=raw, choices=('Off', 'On') if valid else (), default_value='Off',
                             repair_value='' if valid or not readable else 'Off',
                             available=parked and self.authority('parked_preferences') and readable,
                             reason=reason if valid else 'Invalid saved choice; reset it to Off', capability=capability))
    return tuple(rows)

  def apply(self, request):
    if (request.key not in KEYS or request.value not in ('Off', 'On') or not request.confirmation or
        request.dependencies or request.related_source is not None or request.display_unit or request.direction):
      return False
    # On only from a valid saved value; an invalid one can only be repaired to Off
    if request.expected not in (None, b'0', b'1') and (request.value != 'Off' or not isinstance(request.expected, bytes)):
      return False

    def authorized():
      return (self.authority('parked_preferences') and bool(request.vehicle_fingerprint) and
              self.vehicle_fingerprint() == request.vehicle_fingerprint and request.capability is not None and
              self.capability() == request.capability)

    result = commit_exact(self.params, key=request.key, max_bytes=8, expected=request.expected,
                          raw=b'1' if request.value == 'On' else b'0', authorized=authorized, temp_prefix='.mazda-ti-')
    return result.committed and result.verified
