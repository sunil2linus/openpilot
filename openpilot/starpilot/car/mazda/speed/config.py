"""The one saved setting of the Mazda speed control (buttons): `MazdaSpeedControlConfig`, a small versioned JSON
document (the ConditionalModeConfig / CurveComfortData pattern).

Rules:
  * Missing key, unreadable, not JSON, wrong version, unknown field, wrong type or out of bounds -> the whole
    document is INVALID and everything is OFF (`Config.off()`). There is no partial reading.
  * Every field is optional; a missing field takes its default. Defaults are all off.
  * Sub-features only run with `enabled` (the master switch) on.
  * Modes: "off" / "test" (computes and logs, never presses) / "on".
  * Only what is built and tested has a field: version 2 added the curve speed offset (with its per-curve memory),
    version 3 the speed limit follow (+ its offset), the on-ramp merge and the exit plan, version 4 the stop
    slowdown and the on-screen notices (+ their sound), version 5 (Stage 3) the gas auto-resume and the steering
    limit test, version 6 the curve map build range, version 7 the Tag button on the driving screen. A field of a
    later version in an older document is invalid (unknown fields are invalid).
  * Stage 3: `auto_resume` (with the master switch) and `ti_steer_700` (without it: it is a steering setting, used only
    with the Torque Interceptor) also set panda safety param bits at car start (vehicle_preferences.py ->
    opendbc/car/mazda/speed_flags.py), so they apply from the next drive (the next car start).
No openpilot imports beyond the saved-value reader, so it is unit tested offline.
"""
import json
import math
from dataclasses import dataclass, fields, replace

KEY = "MazdaSpeedControlConfig"
VERSION = 7                     # 2: + curve_offset_mph / curve_offset_lat; 3: + speed_limit / limit_offset_mph /
VERSIONS = (1, 2, 3, 4, 5, 6, 7)  # merge / exits; 4: + stops / notices / notice_sound; 5: + auto_resume / ti_steer_700;
                                #    6: + curve_range_mi; 7: + tag_button (older documents still read)
MAX_BYTES = 4096

MODES = ("off", "test", "on")
WHOLE_MAP = 0                   # curve_range_mi: build every map area of the selected offline map
CURVE_RANGES = (30, 50, 100, 200, 300, WHOLE_MAP)   # miles (the old fork's MazdaCurveMapRange choices + the whole map)

# field -> (kind, default, bounds or choices); "choice": an int out of the listed ones
SCHEMA = {
  "enabled": ("bool", False, None),
  "curves": ("mode", "off", MODES),                 # curves from the map (+ camera veto / camera-only curves)
  "camera_curves": ("mode", "off", MODES),          # camera-only target for curves the map does not have
  "plan_lat": ("float", 2.2, (1.8, 3.5)),           # m/s^2: roads
  "ramp_lat": ("float", 2.6, (1.8, 3.5)),           # m/s^2: ramps / links
  "ti_only_lat": ("float", 1.8, (1.8, 3.5)),        # m/s^2: stock LKAS blocked before the curve / camera fault
  "curve_offset_mph": ("int", 0, (0, 5)),           # (v2) plan map curves' tight points this much faster ...
  "curve_offset_lat": ("float", 3.0, (2.0, 4.0)),   # (v2) ... never above this side force; removed per curve by memory
  "speed_limit": ("mode", "off", MODES),            # (v3) follow the posted limit (7.0's display-only reading)
  "limit_offset_mph": ("int", 3, (0, 10)),          # (v3) ... at the limit + this (dash mph; km/h dash: the same number)
  "merge": ("mode", "off", MODES),                  # (v3) on-ramp merge: up to the highway's limit + offset
  "exits": ("mode", "off", MODES),                  # (v3) exit plan: pre-slow for an exit ramp's curves before the split
  "stops": ("mode", "off", MODES),                  # (v4) stop slowdown: a stopped car / a stop ahead (stops.py)
  "notices": ("bool", False, None),                 # (v4) short on-screen notices (notices.py -> selfdrived)
  "notice_sound": ("bool", False, None),            # (v4) ... the stop / brake notices with a chime
  "auto_resume": ("bool", False, None),             # (v5) after his brake, his gas presses RES for him (auto_resume.py)
  "ti_steer_700": ("bool", False, None),            # (v5) TI steering limit test 600 -> 700 (not under the master switch)
  "curve_range_mi": ("choice", 100, CURVE_RANGES),  # (v6) the parked curve-map builder's range (curve_mapd.py), 0 = all
  "tag_button": ("bool", False, None),              # (v7) a TAG button on the driving screen (a Bookmark: our tag line)
}
V2_FIELDS = frozenset(("curve_offset_mph", "curve_offset_lat"))
V3_FIELDS = frozenset(("speed_limit", "limit_offset_mph", "merge", "exits"))
V4_FIELDS = frozenset(("stops", "notices", "notice_sound"))
V5_FIELDS = frozenset(("auto_resume", "ti_steer_700"))
V6_FIELDS = frozenset(("curve_range_mi",))
V7_FIELDS = frozenset(("tag_button",))


class ConfigError(ValueError):
  pass


@dataclass(frozen=True)
class Config:
  enabled: bool = False
  curves: str = "off"
  camera_curves: str = "off"
  plan_lat: float = 2.2
  ramp_lat: float = 2.6
  ti_only_lat: float = 1.8
  curve_offset_mph: int = 0
  curve_offset_lat: float = 3.0
  speed_limit: str = "off"
  limit_offset_mph: int = 3
  merge: str = "off"
  exits: str = "off"
  stops: str = "off"
  notices: bool = False
  notice_sound: bool = False
  auto_resume: bool = False
  ti_steer_700: bool = False
  curve_range_mi: int = 100
  tag_button: bool = False
  valid: bool = True            # False: the saved document was invalid (everything is off)

  @classmethod
  def off(cls, valid=True):
    return cls(valid=valid)

  def mode(self, name):
    """The effective mode of a sub-feature: "off" unless the master switch is on."""
    if not self.enabled:
      return "off"
    return getattr(self, name)

  def active(self, name):
    return self.mode(name) != "off"

  def presses(self, name):
    return self.mode(name) == "on"

  @property
  def needs_view(self):
    """The map view daemon (and its curve files) are needed: map curves, the limit (road / GPS trust), merge, exits."""
    return self.enabled and (any(self.mode(n) != "off" for n in ("curves", "speed_limit", "merge", "exits")) or
                             self.auto_resume)

  @property
  def shows_notices(self):
    """The on-screen notices: the master switch, notices on and at least one feature Test or On (they come from it)."""
    return bool(self.enabled and self.notices and self.any_active)

  @property
  def any_active(self):
    return self.enabled and (any(self.mode(n) != "off" for n, (k, _, _) in SCHEMA.items() if k == "mode") or
                             self.auto_resume)

  @property
  def tag_on(self):
    """(v7) the Tag button on the driving screen: the master switch and its own switch. It also runs the speed
    control's event log for the tag lines when no other feature is on (caller.make: tag only, it never presses)."""
    return bool(self.enabled and self.tag_button)

  @property
  def auto_resume_on(self):
    """(Stage 3) the gas auto-resume: the master switch and its own switch (its panda bit is set from this too)."""
    return bool(self.enabled and self.auto_resume)

  @property
  def steer_700_on(self):
    """(Stage 3) the TI steering limit test: its own switch only (applied only on a car with the TI)."""
    return bool(self.ti_steer_700)

  @property
  def curve_range_miles(self):
    """(v6) miles around the last known position the curve maps are built for, or None: the whole map."""
    return None if self.curve_range_mi == WHOLE_MAP else float(self.curve_range_mi)


def _unique(pairs):
  out = {}
  for k, v in pairs:
    if k in out:
      raise ConfigError("duplicate key")
    out[k] = v
  return out


def _bad_constant(_):
  raise ConfigError("NaN/Infinity not allowed")


def decode(raw):
  """bytes -> Config. Raises ConfigError for anything not exactly valid."""
  if raw is None:
    return Config.off()
  if not isinstance(raw, (bytes, bytearray)) or len(raw) > MAX_BYTES:
    raise ConfigError("size")
  try:
    doc = json.loads(bytes(raw).decode("utf-8"), object_pairs_hook=_unique, parse_constant=_bad_constant)
  except (UnicodeDecodeError, ValueError, RecursionError) as e:
    raise ConfigError(f"json: {e}") from None
  if type(doc) is not dict:
    raise ConfigError("not an object")
  if doc.get("version") not in VERSIONS or type(doc.get("version")) is not int:
    raise ConfigError("version")
  if ((doc["version"] < 2 and V2_FIELDS & set(doc)) or (doc["version"] < 3 and V3_FIELDS & set(doc)) or
      (doc["version"] < 4 and V4_FIELDS & set(doc)) or (doc["version"] < 5 and V5_FIELDS & set(doc)) or
      (doc["version"] < 6 and V6_FIELDS & set(doc)) or (doc["version"] < 7 and V7_FIELDS & set(doc))):
    raise ConfigError("field of a later version")
  values = {}
  for k, v in doc.items():
    if k == "version":
      continue
    if k not in SCHEMA:
      raise ConfigError(f"unknown field {k}")
    kind, _, lim = SCHEMA[k]
    if kind == "bool":
      if type(v) is not bool:
        raise ConfigError(k)
    elif kind == "mode":
      if type(v) is not str or v not in lim:
        raise ConfigError(k)
    elif kind == "int":
      if type(v) is not int or not lim[0] <= v <= lim[1]:
        raise ConfigError(k)
    elif kind == "choice":
      if type(v) is not int or v not in lim:
        raise ConfigError(k)
    elif kind == "float":
      if type(v) not in (int, float) or not math.isfinite(v) or not lim[0] <= v <= lim[1]:
        raise ConfigError(k)
      v = float(v)
    values[k] = v
  return Config(**values)


def encode(cfg: Config) -> bytes:
  """Config -> canonical bytes (only non-default fields, sorted, compact)."""
  doc = {"version": VERSION}
  for f in fields(Config):
    if f.name == "valid":
      continue
    v = getattr(cfg, f.name)
    if v != SCHEMA[f.name][1]:
      doc[f.name] = v
  raw = json.dumps(doc, sort_keys=True, separators=(",", ":")).encode()
  decode(raw)  # never write what we would not read back
  return raw


def with_value(cfg: Config, name, value) -> Config:
  if name not in SCHEMA:
    raise ConfigError(name)
  new = replace(cfg, **{name: value}, valid=True)
  decode(encode(new))
  return new


def read(params) -> Config:
  """The saved Config, never raises: invalid / unreadable -> everything off (valid=False)."""
  try:
    from openpilot.starpilot.saved_source import read_saved
    raw, readable = read_saved(params, KEY, MAX_BYTES)
  except Exception:
    return Config.off(valid=False)
  if not readable:
    return Config.off(valid=False)
  try:
    return decode(raw)
  except ConfigError:
    return Config.off(valid=False)
