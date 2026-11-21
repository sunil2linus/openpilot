"""When the manager runs the two Mazda speed control helpers (system/manager/process_config.py): only on a Mazda
(the CarParams the manager has: the last drive's offroad) with the master switch on and map curves Test or On.
mazda_mapviewd runs onroad, mazda_curve_mapd parked. Never raises (a broken setting is simply off)."""
from openpilot.starpilot.car.mazda.speed import config as C


def _curves(params, CP):
  try:
    return bool(CP is not None and CP.brand == "mazda" and not CP.notCar and C.read(params).active("curves"))
  except Exception:
    return False


def mapview(started, params, CP) -> bool:
  return bool(started) and _curves(params, CP)


def curve_build(started, params, CP) -> bool:
  return not started and _curves(params, CP)
