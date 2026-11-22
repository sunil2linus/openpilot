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
   "Lowers the cruise set speed before curves found in the offline map. Exit ramp curves before the split need " +
   "Exit ramps. Test computes and logs, never presses.", 0.),
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
  ("speed_limit", "Follow the speed limit",
   "Moves a speed we set (or one you set at the limit) to the posted limit plus the offset; never lowers a higher " +
   "speed you chose yourself, and your SET+/SET- make the speed yours. Needs Show Speed Limits on and Map Data in " +
   "the speed limit sources (offline maps).", 0.),
  ("limit_offset_mph", "Speed limit offset", "Follow the limit at the limit plus this many mph (also the merge's " +
   "and the exit street's target).", 1.),
  ("merge", "On-ramp merge",
   "On a ramp onto a highway, once its curves are behind, raises the set speed (a boost) to reach the highway's " +
   "limit plus the offset. Waits for a stop, a slow car ahead or blocked lane keeping.", 0.),
  ("exits", "Exit ramps",
   "Slows as much as the ramp needs while your signal toward the exit is on (within about 400 m); a short signal " +
   "slows at most 12 mph, so keep it on to the split for a tight exit or brake yourself. Hands the speed to the " +
   "street's limit after; lets go at once if you stay on the highway, press a button or hold the gas.", 0.),
)
UNITS = {"limit_offset_mph": "mph", "curve_offset_mph": "mph", "curve_offset_lat": "m/s²", "plan_lat": "m/s²", "ramp_lat": "m/s²", "ti_only_lat": "m/s²"}


MAP_ROWS = ("speed_limit", "merge", "exits")
CURVE_FILE_ROWS = ("curves", "merge", "exits")     # use the curve files mazda_curve_mapd builds parked
GPS_ROWS = ("curves", "speed_limit", "merge", "exits")   # need mazda_mapviewd's GPS position
MAP_STATUS_CACHE = 5.0          # s


def daemon_status():
  """(curve-map build text, last drive's GPS text) from the two helpers' status files (speed/status.py), "" each when
  there is nothing to say. Never raises."""
  try:
    from openpilot.starpilot.car.mazda.speed import status as ST
    return ST.build_text(ST.read(ST.BUILD_NAME)), ST.gps_text(ST.read(ST.GPS_NAME))
  except Exception:
    return "", ""


def map_service_status(params, cp):
  """"" when 7.0's map service (the mapd provider: map speed limits and the map's preview limit) will run on a drive,
  else what to do, in plain words. The checks 7.0 itself uses (maps/shadow_lifecycle.enabled and the offline-map
  part of preflight): Show Speed Limits (or the SLC) on, Map Data in the speed limit sources (or Highest / Lowest),
  and a downloaded, selected offline map. Never raises."""
  try:
    from openpilot.starpilot.maps import shadow_lifecycle as S
    from openpilot.starpilot.speed_limits.runtime_settings import read_params
    from openpilot.starpilot.speed_limits.selection import SelectionMode, Source
    todo = []
    settings = read_params(params)
    if settings.errors:
      todo.append("fix the speed limit settings")
    else:
      if not (settings.enabled or settings.display):
        todo.append("turn on Show Speed Limits")
      if not (Source.MAP in settings.selection.slots or settings.selection.mode is not SelectionMode.ORDERED):
        todo.append("add Map Data to the speed limit sources")
    if not (S._owned_root(S.OFFLINE_ROOT, S.STORAGE_ANCHOR) and S._selected_snapshot(S.OFFLINE_ROOT)):
      todo.append("download offline maps in Maps settings")
    if not todo:
      return ""
    return "no map speed limits: " + ", ".join(todo) + "."
  except Exception:
    return "the map service's state cannot be read."


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
    self._map_status = None         # (monotonic s, text)
    self._daemon_status = None      # (monotonic s, (build text, gps text))

  def map_status(self, cp):
    import time
    now = time.monotonic()
    if self._map_status is None or now - self._map_status[0] > MAP_STATUS_CACHE:
      self._map_status = (now, map_service_status(self.params, cp))
    return self._map_status[1]

  def helper_status(self):
    import time
    now = time.monotonic()
    if self._daemon_status is None or now - self._daemon_status[0] > MAP_STATUS_CACHE:
      self._daemon_status = (now, daemon_status())
    return self._daemon_status[1]

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
                       reason="Presses the cruise SET+/SET- buttons for curves, speed limits, merges and exits. Off by " +
                       "default; applies at the next drive.",
                       **common)]
    if not cfg.enabled:
      return tuple(rows)
    for field, label, reason, step in SUB_ROWS:
      kind, default, lim = C.SCHEMA[field]
      value = getattr(cfg, field)
      now_says = []
      if field in MAP_ROWS:
        now_says.append(self.map_status(self.vehicle_params()))
      if field in CURVE_FILE_ROWS or field in GPS_ROWS:
        build, gps = self.helper_status()
        now_says += [build if field in CURVE_FILE_ROWS else "", gps if field in GPS_ROWS else ""]
      now_says = [t for t in now_says if t]
      if now_says:
        reason = reason + " Now: " + " ".join(now_says)
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
