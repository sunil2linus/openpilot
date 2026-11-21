"""Mazda speed control (buttons): one saved JSON document (MazdaSpeedControlConfig), shown as a master switch and,
while it is on, a few sub-options. Gen1 Mazda with the stock radar cruise only (the same cars as the Torque
Interceptor row). Saved parked; read when the drive starts."""
from openpilot.starpilot.car.mazda.speed import config as C
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

PREFIX = "mazda_speed:"
MASTER = PREFIX + "enabled"
RESET = PREFIX + "reset"
MODE_LABELS = {"off": "Off", "test": "Test (log only)", "on": "On"}
MODE_VALUES = {v: k for k, v in MODE_LABELS.items()}

# field, label, reason, step
SUB_ROWS = (
  ("curves", "Curves from the map",
   "Lowers the cruise set speed before curves found in the offline map (not yet before exit ramps: slow for those " +
   "yourself). Test computes and logs, never presses.", 0.),
  ("camera_curves", "Curves from the camera",
   "Lowers the set speed carefully for curves missing from the map, from what the camera sees (about 2 s ahead).", 0.),
  ("curve_offset_mph", "Curve speed offset",
   "Plans the tight part of map curves this many mph faster, never above the offset side force. Removed per curve " +
   "after a hard pass; on ramps only after two clean passes.", 1.),
  ("curve_offset_lat", "Offset side force", "The most side force (m/s²) the curve speed offset may plan.", .1),
  ("plan_lat", "Road curve side force", "Side force (m/s²) map curves on roads are planned at.", .1),
  ("ramp_lat", "Ramp curve side force", "Side force (m/s²) curves on ramps and links are planned at.", .1),
  ("ti_only_lat", "Interceptor-only side force",
   "Side force (m/s²) used when the stock lane keeping is blocked before a curve and only the interceptor steers.", .1),
)
UNITS = {"curve_offset_mph": "mph", "curve_offset_lat": "m/s²", "plan_lat": "m/s²", "ramp_lat": "m/s²", "ti_only_lat": "m/s²"}


def capable(cp) -> bool:
  from openpilot.starpilot.ui.mazda_ti_feature import capable as ti_capable
  return ti_capable(cp)


def _shown(value):
  if isinstance(value, bool):
    return "On" if value else "Off"
  if isinstance(value, float):
    return f"{value:.1f}"
  if isinstance(value, str):
    return MODE_LABELS[value]
  return str(value)


def parse_value(field, text):
  """The displayed / requested text -> the saved value for `field`, or raise ConfigError."""
  kind, _, lim = C.SCHEMA[field]
  if kind == "bool":
    if text not in ("Off", "On"):
      raise C.ConfigError(field)
    return text == "On"
  if kind == "mode":
    if text not in MODE_VALUES:
      raise C.ConfigError(field)
    return MODE_VALUES[text]
  try:
    number = float(text)
  except (TypeError, ValueError):
    raise C.ConfigError(field) from None
  if kind == "int":
    if number != int(number):
      raise C.ConfigError(field)
    return int(number)
  return round(number, 1)


class MazdaSpeedFeature:
  def __init__(self, owner):
    self.params = owner.params
    self.authority = owner.authority
    self.vehicle_params = owner.vehicle_params
    self.vehicle_fingerprint = owner.vehicle_fingerprint

  def capability(self):
    cp = self.vehicle_params()
    if not capable(cp):
      return None
    return (str(cp.carFingerprint), str(cp.brand), 'mazda-gen1-speed-buttons')

  def _source(self):
    raw, readable = read_saved(self.params, C.KEY, C.MAX_BYTES)
    try:
      cfg = C.decode(raw) if readable else None
    except C.ConfigError:
      cfg = None
    return raw, readable, cfg

  def rows(self, parked):
    capability = self.capability()
    if capability is None:
      return ()
    raw, readable, cfg = self._source()
    available = bool(parked and readable and self.authority('parked_preferences'))
    common = {"source": raw, "available": available, "capability": capability, "vehicle_fingerprint": self.vehicle_fingerprint()}
    if cfg is None:
      return (FeatureRow(RESET, "Mazda speed control (buttons)", "Invalid saved settings", repair_value="Off",
                         reason="Invalid saved settings: everything is off. Reset turns it off with default options."
                         if readable else "Saved settings cannot be read.", **common),)
    rows = [FeatureRow(MASTER, "Mazda speed control (buttons)", _shown(cfg.enabled), choices=("Off", "On"),
                       default_value="Off",
                       reason="Presses the cruise SET+/SET- buttons for curves. Off by default; applies at the next drive.",
                       **common)]
    if not cfg.enabled:
      return tuple(rows)
    for field, label, reason, step in SUB_ROWS:
      kind, default, lim = C.SCHEMA[field]
      value = getattr(cfg, field)
      if kind == "mode":
        rows.append(FeatureRow(PREFIX + field, label, _shown(value), choices=tuple(MODE_LABELS.values()),
                               default_value=_shown(default), reason=reason + " Applies at the next drive.", **common))
      else:
        rows.append(FeatureRow(PREFIX + field, label, _shown(value), step=step, minimum=float(lim[0]), maximum=float(lim[1]),
                               unit=UNITS.get(field, ""), default_value=_shown(default),
                               reason=reason + " Applies at the next drive.", **common))
    return tuple(rows)

  def apply(self, request):
    key = request.key
    if (not key.startswith(PREFIX) or request.dependencies or request.related_source is not None or
        request.display_unit or request.direction):
      return False
    raw, readable, cfg = self._source()
    if not readable or raw != request.expected:
      return False
    try:
      if key == RESET:
        if request.value != "Off":
          return False
        new = C.Config()
      else:
        field = key[len(PREFIX):]
        if field not in C.SCHEMA or cfg is None:
          return False
        value = parse_value(field, request.value)
        if field == "enabled" and value and not request.confirmation:
          return False   # turning the button presses on needs the confirmation question
        new = C.with_value(cfg, field, value)
      encoded = C.encode(new)
    except C.ConfigError:
      return False

    def authorized():
      return (self.authority('parked_preferences') and bool(request.vehicle_fingerprint) and
              self.vehicle_fingerprint() == request.vehicle_fingerprint and request.capability is not None and
              self.capability() == request.capability)

    result = commit_exact(self.params, key=C.KEY, max_bytes=C.MAX_BYTES, expected=request.expected, raw=encoded,
                          authorized=authorized, temp_prefix='.mazda-speed-')
    return result.committed and result.verified
