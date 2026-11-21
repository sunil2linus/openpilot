"""Saved Torque Interceptor selection for Gen1 Mazda vehicles (hardware declaration, applies after the next startup)."""
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.ui.feature_settings_state import FeatureRow


KEYS = frozenset(('TorqueInterceptorEnabled',))
ROWS = (('TorqueInterceptorEnabled', 'Torque Interceptor (Mazda)',
         'Turn on only if a MoreTorque Torque Interceptor is installed: StarPilot then also steers below 28 mph. '
         'Applies after the next startup.'),)


class MazdaTorqueInterceptorFeature:
  def __init__(self, owner):
    self.params = owner.params
    self.authority = owner.authority
    self.vehicle_params = owner.vehicle_params
    self.vehicle_fingerprint = owner.vehicle_fingerprint

  def capability(self):
    from opendbc.car.mazda.torque_interceptor import installed, supported
    cp = self.vehicle_params()
    try:
      # The same answer before and after the setting is applied to the running CarParams.
      if cp is None or cp.brand != 'mazda' or cp.notCar or not (supported(cp) or installed(cp)):
        return None
      return (str(cp.carFingerprint), str(cp.brand), 'mazda-gen1-ti')
    except (AttributeError, TypeError, ValueError):
      return None

  def rows(self, parked):
    capability = self.capability()
    if capability is None:
      return ()
    rows = []
    for key, label, reason in ROWS:
      raw, readable = read_saved(self.params, key, 8)
      valid = readable and raw in (None, b'0', b'1')
      rows.append(FeatureRow(key, label, ('On' if raw == b'1' else 'Off') if valid else 'Invalid saved choice',
                             source=raw, choices=('Off', 'On') if valid else (), default_value='Off',
                             available=parked and self.authority('parked_preferences') and valid,
                             reason=reason if valid else 'Invalid saved choice', capability=capability))
    return tuple(rows)

  def apply(self, request):
    if (request.key not in KEYS or request.value not in ('Off', 'On') or not request.confirmation or
        request.expected not in (None, b'0', b'1') or request.dependencies or request.related_source is not None or
        request.display_unit or request.direction):
      return False

    def authorized():
      return (self.authority('parked_preferences') and bool(request.vehicle_fingerprint) and
              self.vehicle_fingerprint() == request.vehicle_fingerprint and request.capability is not None and
              self.capability() == request.capability)

    result = commit_exact(self.params, key=request.key, max_bytes=8, expected=request.expected,
                          raw=b'1' if request.value == 'On' else b'0', authorized=authorized, temp_prefix='.mazda-ti-')
    return result.committed and result.verified
