"""Mazda steering smoothing: the previous build's 0.10 s low-pass on the driving model's steering path.

The owner's previous build ran LAT_SMOOTH_SECONDS = 0.1 for every car: modeld filtered modelV2.action.desiredCurvature
with smooth_value(new, previous, 0.1) at 20 Hz and added the same 0.1 s to the lateral delay (so the model's
lat_action_t looks 0.1 s further ahead and makes up for the filter). 7.0 has LAT_SMOOTH_SECONDS = 0.0 and restores
the filter only for Subaru, Volkswagen and the Hyundai Kona (filter_action in subaru/smoothing.py), so on a Mazda the
raw 20 Hz curvature steps reach the torque controller. This module gives the Mazda the same 0.10 s again.

Where the 0.10 s goes (all four places the Kona's tau goes):
  * modeld: the filter_action(...) tau sum and lat_delay (lat_action_t and the model's lateral_control_params);
  * controlsd: lat_delay (LatControlTorque's lookahead) and lane_delay (Lane Centering).
lagd needs nothing: it measures the car's lag after controlsState.desiredCurvature, which is downstream of the filter.

Which cars: a Gen1 Mazda (every fingerprint of this port, with or without the Torque Interceptor) that steers with
the torque controller: brand mazda, a Mazda platform, the GEN1 flag, one Mazda safety config, not passive,
dashcam-only or notCar. Every other car gets 0.0 and this module reads and writes nothing for it.

The setting: `steer_smoothing` in MazdaSpeedControlConfig (speed/config.py, version 8), on by default, its own
switch (not under the speed control's master switch). Card reads it once at car start (vehicle_preferences.py).

Frozen for the drive, so modeld and controlsd always use the same value: card decides once and writes a small
context bound to the sha256 of the CarParams it publishes (the Subaru SubaruSmoothingContext pattern), before it
publishes CarParams. modeld and controlsd read that context once, after they got CarParams. The context lives in a
volatile file next to modeld's load receipt (/dev/shm), not in a new Params key: a new key would need the committed
aarch64 libparams_c.so / libcommon.a rebuilt. A card restart in the same drive (CarParams still published with the
same bytes) keeps the context it finds, so a later modeld or controlsd restart reads the same value. Anything
missing, unreadable or not matching the CarParams reads as 0.0 (7.0's behavior) in both processes.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re

from opendbc.car import structs
from opendbc.car.mazda.values import CAR, MazdaFlags

SECONDS = 0.1                 # the previous build's LAT_SMOOTH_SECONDS
VERSION = 1
MAX_BYTES = 256
MAX_CP_BYTES = 1_048_576
MAZDA_CARS = frozenset(CAR)
_PREFIX = re.compile(r"[A-Za-z0-9_-]{1,32}\Z")
_DIGEST = re.compile(r"[0-9a-f]{64}\Z")


def supported(cp) -> bool:
  """A Gen1 Mazda of this port (with or without the TI) steering with the torque controller."""
  try:
    if cp.brand != 'mazda':
      return False
    owners = [c for c in cp.safetyConfigs if c.safetyModel not in
              (structs.CarParams.SafetyModel.silent, structs.CarParams.SafetyModel.noOutput)]
    return bool(cp.carFingerprint in MAZDA_CARS and int(cp.flags) & int(MazdaFlags.GEN1) and
                not cp.passive and not cp.dashcamOnly and not cp.notCar and
                cp.steerControlType == structs.CarParams.SteerControlType.torque and
                cp.lateralTuning.which() == 'torque' and
                len(owners) == 1 and owners[0].safetyModel == structs.CarParams.SafetyModel.mazda)
  except (AttributeError, TypeError, ValueError, IndexError):
    return False


def seconds(cp, requested) -> float:
  """0.10 for a supported Mazda with the setting on, else 0.0."""
  return SECONDS if requested is True and supported(cp) else 0.


def context_path() -> Path:
  from openpilot.common.hardware.hw import Paths
  prefix = os.environ.get("OPENPILOT_PREFIX", "d")
  if _PREFIX.fullmatch(prefix) is None:
    raise ValueError("invalid IPC namespace")
  return Path(Paths.shm_path()) / f"starpilot_mazda_{prefix}" / "smoothing-v1.json"


def encode(cp_bytes: bytes, value: float) -> bytes:
  if type(cp_bytes) is not bytes or value not in (0., SECONDS):
    raise ValueError("invalid Mazda smoothing context")
  return json.dumps({"version": VERSION, "carParamsSha256": hashlib.sha256(cp_bytes).hexdigest(),
                     "seconds": "0.1" if value == SECONDS else "0"}, sort_keys=True, separators=(",", ":")).encode("ascii")


def decode(cp_bytes, raw) -> float | None:
  """The frozen seconds when `raw` is a context for exactly these CarParams bytes, else None."""
  if type(cp_bytes) is not bytes or not 16 <= len(cp_bytes) <= MAX_CP_BYTES or type(raw) is not bytes or not 0 < len(raw) <= MAX_BYTES:
    return None
  try:
    doc = json.loads(raw.decode("ascii"))
  except (UnicodeDecodeError, ValueError, RecursionError):
    return None
  if (type(doc) is not dict or set(doc) != {"version", "carParamsSha256", "seconds"} or type(doc["version"]) is not int or
      doc["version"] != VERSION or type(doc["carParamsSha256"]) is not str or _DIGEST.fullmatch(doc["carParamsSha256"]) is None or
      doc["carParamsSha256"] != hashlib.sha256(cp_bytes).hexdigest() or doc["seconds"] not in ("0", "0.1")):
    return None
  value = SECONDS if doc["seconds"] == "0.1" else 0.
  return value if encode(cp_bytes, value) == raw else None


def _read(path: Path) -> bytes | None:
  try:
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
  except OSError:
    return None
  try:
    data = os.read(fd, MAX_BYTES + 1)
  except OSError:
    return None
  finally:
    os.close(fd)
  return data if 0 < len(data) <= MAX_BYTES else None


def _write(path: Path, raw: bytes) -> None:
  try:
    os.mkdir(path.parent, 0o700)
  except FileExistsError:
    pass
  tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
  fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
  try:
    os.write(fd, raw)
  finally:
    os.close(fd)
  os.replace(tmp, path)


def _remove(path: Path) -> None:
  try:
    os.unlink(path)
  except OSError:
    pass


def install(cp, cp_bytes: bytes, requested, published=None, *, path: Path | None = None) -> float:
  """Card, once per start, before it publishes `cp_bytes` as CarParams. Returns the frozen seconds.

  `published` is the CarParams Params value read just before this publication: when it is these same bytes (a card
  restart in the same drive) a valid context already there is kept, so the drive's value never changes.
  """
  if not supported(cp):
    return 0.
  from openpilot.common.swaglog import cloudlog
  try:
    if path is None:
      path = context_path()
  except ValueError as e:
    cloudlog.warning(f"Mazda steering smoothing: no context path ({e}); 0.0 s")
    return 0.
  try:
    if published == cp_bytes:
      kept = decode(cp_bytes, _read(path))
      if kept is not None:
        cloudlog.warning(f"Mazda steering smoothing: kept {kept:.2f} s for this drive")
        return kept
    value = seconds(cp, requested)
    raw = encode(cp_bytes, value)
    _write(path, raw)
    if decode(cp_bytes, _read(path)) != value:
      raise OSError("Mazda smoothing context did not read back")
    cloudlog.warning(f"Mazda steering smoothing: {value:.2f} s for this drive")
    return value
  except (OSError, ValueError) as e:
    _remove(path)   # no older context may stay readable for these CarParams
    cloudlog.warning(f"Mazda steering smoothing: context not written ({e}); 0.0 s")
    return 0.


def read_frozen(cp, params, *, path: Path | None = None) -> float:
  """modeld and controlsd, once after CarParams: the drive's frozen seconds; 0.0 for any other car or any doubt."""
  if not supported(cp):
    return 0.
  from openpilot.common.swaglog import cloudlog
  from openpilot.starpilot.saved_source import read_saved
  try:
    cp_bytes, readable = read_saved(params, 'CarParams', MAX_CP_BYTES)
    value = decode(cp_bytes, _read(context_path() if path is None else path)) if readable else None
    if value is not None:
      with structs.CarParams.from_bytes(cp_bytes, traversal_limit_in_words=MAX_CP_BYTES // 8, nesting_limit=16) as published:
        if published.to_dict() != cp.to_dict():
          value = None
  except Exception as e:  # never stop modeld / controlsd over the smoothing: 0.0 is 7.0's behavior
    cloudlog.warning(f"Mazda steering smoothing: context unreadable ({e}); 0.0 s")
    return 0.
  if value is None:
    cloudlog.warning("Mazda steering smoothing: no context for these CarParams; 0.0 s")
    return 0.
  return value
