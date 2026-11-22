"""When the manager runs the two Mazda speed control helpers (system/manager/process_config.py): only on a Mazda
with the master switch on and any option that needs the map (map curves, speed limit, merge or exits) Test or On.
mazda_mapviewd runs onroad, mazda_curve_mapd parked. Never raises (a broken setting is simply off).

Which car: the CarParams the manager has (sm['carParams'], card's message). Before the first drive after a boot (parked
on power) and for the first moments of a drive that message has not come yet (brand ""): then the car of the last
drive, CarParamsPersistent (7.0's checked cache, schema_cache.get_cache, as ui_state and athenad read it). Its bytes
are re-read at most every PERSISTENT_RECHECK s and decoded only when they changed, never on every manager tick."""
import time

from openpilot.starpilot.car.mazda.speed import config as C

PERSISTENT_RECHECK = 30.0       # s
_cache = {"t": None, "raw": None, "car": ("", False)}


def _persistent(params, now=None):
  """(brand, notCar) of CarParamsPersistent, ("", False) when there is none or it does not check out."""
  now = time.monotonic() if now is None else now
  if _cache["t"] is not None and 0.0 <= now - _cache["t"] < PERSISTENT_RECHECK:
    return _cache["car"]
  _cache["t"] = now
  try:
    raw = params.get("CarParamsPersistent")
  except Exception:
    raw = None
  if raw == _cache["raw"]:
    return _cache["car"]
  car = ("", False)
  try:
    if raw:
      from opendbc.car.structs import car as car_capnp
      from openpilot.starpilot.schema_cache import inspect_cache
      payload = inspect_cache("CarParamsPersistent", raw).payload
      if payload is not None:
        with car_capnp.CarParams.from_bytes(payload) as cp:
          car = (str(cp.brand), bool(cp.notCar))
  except Exception:
    car = ("", False)
  _cache["raw"], _cache["car"] = raw, car
  return car


def car_brand(params, CP):
  """(brand, notCar): the manager's CarParams when it names a brand, else CarParamsPersistent."""
  brand = str(getattr(CP, "brand", "") or "") if CP is not None else ""
  if brand:
    return brand, bool(getattr(CP, "notCar", False))
  return _persistent(params)


def _curves(params, CP):
  try:
    brand, not_car = car_brand(params, CP)
    return bool(brand == "mazda" and not not_car and C.read(params).needs_view)
  except Exception:
    return False


def mapview(started, params, CP) -> bool:
  return bool(started) and _curves(params, CP)


def curve_build(started, params, CP) -> bool:
  return not started and _curves(params, CP)
