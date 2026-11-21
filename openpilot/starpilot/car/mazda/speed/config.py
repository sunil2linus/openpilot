"""The one saved setting of the Mazda speed control (buttons): `MazdaSpeedControlConfig`, a small versioned JSON
document (the ConditionalModeConfig / CurveComfortData pattern).

Rules:
  * Missing key, unreadable, not JSON, wrong version, unknown field, wrong type or out of bounds -> the whole
    document is INVALID and everything is OFF (`Config.off()`). There is no partial reading.
  * Every field is optional; a missing field takes its default. Defaults are all off.
  * Sub-features only run with `enabled` (the master switch) on.
  * Modes: "off" / "test" (computes and logs, never presses) / "on".
No openpilot imports beyond the saved-value reader, so it is unit tested offline.
"""
import json
import math
from dataclasses import dataclass, fields, replace

KEY = "MazdaSpeedControlConfig"
VERSION = 1
MAX_BYTES = 4096

MODES = ("off", "test", "on")

# field -> (kind, default, bounds or choices)
SCHEMA = {
  "enabled": ("bool", False, None),
  "curves": ("mode", "off", MODES),                 # curves from the map (+ camera veto / camera-only curves)
  "camera_curves": ("mode", "off", MODES),          # camera-only target for curves the map does not have
  "curve_offset_mph": ("int", 0, (0, 5)),           # plan curves this much faster ...
  "side_force_cap": ("float", 3.0, (2.0, 4.0)),     # ... but never above this lateral accel (m/s^2)
  "plan_lat": ("float", 2.2, (1.8, 3.5)),           # m/s^2: roads
  "ramp_lat": ("float", 2.6, (1.8, 3.5)),           # m/s^2: ramps / links
  "ti_only_lat": ("float", 1.8, (1.8, 3.5)),        # m/s^2: stock LKAS blocked before the curve / camera fault
}


class ConfigError(ValueError):
  pass


@dataclass(frozen=True)
class Config:
  enabled: bool = False
  curves: str = "off"
  camera_curves: str = "off"
  curve_offset_mph: int = 0
  side_force_cap: float = 3.0
  plan_lat: float = 2.2
  ramp_lat: float = 2.6
  ti_only_lat: float = 1.8
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
  def any_active(self):
    return self.enabled and any(self.mode(n) != "off" for n, (k, _, _) in SCHEMA.items() if k == "mode")


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
  if doc.get("version") != VERSION or type(doc.get("version")) is not int:
    raise ConfigError("version")
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
