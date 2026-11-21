"""What the stock MRCC does with a set speed, as measured on the owner's 2018 CX-5, and the braking envelope every
slowdown is planned with.

Measured (set speed vs true speed, mph):
  set above the car by  2-5 -> +0.39 m/s^2,  5-10 -> +0.50,  10-20 -> +0.71,  >20 -> +0.82
  car above the set by  3-5 -> -0.54 m/s^2,  5-8  -> -0.67,  8-11  -> -0.80,  11-15 -> -0.92
  never more than ~1 m/s^2 of braking; about 1.5 s from a set speed change to the car answering.
The dash (and the set speed) reads about DASH_OFFSET above the true speed (car_profile.DASH_OFFSET, owner's car).
"""
import math

MPH = 0.44704
KPH = 1 / 3.6

ACCEL_BANDS_MPH = ((2, 5, 0.39), (5, 10, 0.50), (10, 20, 0.71), (20, 1e9, 0.82))
DECEL_BANDS_MPH = ((3, 5, 0.54), (5, 8, 0.67), (8, 11, 0.80), (11, 1e9, 0.92))
LAG = 1.5                      # s from a set speed change to the cruise answering
BUTTON_TIME = 0.3              # s for a press to land on the set speed (the actuator learns the real one)
DASH_OFFSET = 2.0 * MPH        # m/s: set speed (dash) minus the true speed the cruise holds
PLAN_DECEL = 0.54              # m/s^2 every slowdown is planned with: the 3-5 mph band, which UNDERSHOOT reaches
UNDERSHOOT = 4.0 * MPH         # m/s below the target the set goes while the car is still faster than ...
UNDERSHOOT_ABOVE = 1.5 * MPH   # ... the target + this (so the gentle cruise slows in time); released by itself after
LKAS_FLOOR_FROM = 30.0 * MPH   # a curve whose own target is at least this ...
LKAS_FLOOR = (29.0 * MPH, 47.0 * KPH)   # ... never gets a set below this (keeps the stock LKAS above its cut-off)


def cruise_decel(gap):
  """m/s^2 of braking the cruise gives with the car `gap` m/s above the true set speed (0 under 3 mph)."""
  g = gap / MPH
  for lo, hi, a in DECEL_BANDS_MPH:
    if lo <= g < hi:
      return a
  return 0.0


def cruise_accel(gap):
  """m/s^2 the cruise gives with the true set speed `gap` m/s above the car (0 under 2 mph)."""
  g = gap / MPH
  for lo, hi, a in ACCEL_BANDS_MPH:
    if lo <= g < hi:
      return a
  return 0.0


def envelope(v_point, distance, v_ego, decel=PLAN_DECEL, lag=LAG + BUTTON_TIME):
  """The highest speed (m/s, true) the car may have now so that it is at v_point by `distance` m ahead, when the
  cruise only starts braking `lag` s after the set speed moves and then brakes at `decel`. Behind or within the lag
  distance: v_point itself."""
  room = distance - max(v_ego, 0.0) * lag
  if room <= 0:
    return v_point
  return math.sqrt(v_point * v_point + 2.0 * decel * room)


def set_command(target_true, v_ego, metric=False, lkas_floor=False):
  """The set speed (dash units, m/s) to command for a true-speed target: the dash offset added, the undershoot while
  the car is still well faster than the target, and the LKAS floor for curves whose own target allows it."""
  cmd = target_true + DASH_OFFSET
  if v_ego > target_true + UNDERSHOOT_ABOVE:
    cmd -= UNDERSHOOT
  if lkas_floor:
    cmd = max(cmd, LKAS_FLOOR[1] if metric else LKAS_FLOOR[0])
  return cmd
