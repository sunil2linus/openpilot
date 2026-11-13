"""Architect, round n: after OUR RES (auto_resume.py) the existing rules act exactly as after HIS RES. The real card-side
wrappers in carcontroller's order (AutoResume, then CurveSlowdown with other_button_active = ours busy) drive a model of
the car (test_curve_slowdown.Car, the measured stock cruise). Each scenario runs twice: his RES on the wheel (CRZ_BTNS
bus 0, CS.crz_resume) at the frame ours engaged, and ours (no wheel button: our frames are never read back). The
traces after the engagement must match. Needs the native-module stubs (-p stub_native); skipped without them."""
import importlib.util
import os
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)

import cereal.messaging as messaging
from opendbc.car import structs
from opendbc.car.mazda import auto_resume as ar
from opendbc.car.mazda import car_profile as cp
from opendbc.car.mazda import curve_slowdown as cs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import Buttons, MazdaSafetyFlags

HERE = os.path.dirname(__file__)


def _load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


W = _load("tcpw_ar", os.path.join(HERE, "test_car_profile_wrapper.py"))
T = _load("tcs_ar", os.path.join(HERE, "test_curve_slowdown.py"))
MPH = W.MPH
F = 100
DRIVE = structs.CarState.GearShifter.drive


def run(monkeypatch, tmp_path, who, stored_mph, plan_fn, frames=6000, brake_at=1000, stop_s=5.0, gas_until_mph=15,
        v0_mph=40, prime=None, mapd_fn=None, engage_frame=None):
  """A drive: cruise on at `stored_mph` (car at v0), his brake at `brake_at` (cruise off) down to a stop, `stop_s` at it,
  his gas up to `gas_until_mph`; the cruise back by `who` ("ours": our RES; "his": his RES at `engage_frame`).
  plan_fn(frame) -> starpilotPlan fields; prime(curve_slowdown) sets up the controller's state before the brake."""
  sm = W.FakeSM()
  sm.msgs["selfdriveState"] = SimpleNamespace(engageable=True)
  sm.msgs["mapdOut"] = SimpleNamespace(roadName="A Road", nextSpeedLimit=0.0, nextSpeedLimitDistance=0.0)
  for k in ("selfdriveState", "mapdOut"):
    sm.alive[k], sm.recv_frame[k], sm.updated[k] = True, 1, False
  monkeypatch.setattr(messaging, "SubMaster", lambda services: sm)
  monkeypatch.setattr(cp, "PROFILE_PATH", str(tmp_path / f"profile_{who}.json"))
  monkeypatch.setattr(cp.CarProfile.__init__, "__defaults__", (str(tmp_path / f"profile_{who}.json"), "", False, time.monotonic))
  monkeypatch.setattr(mazdacan, "create_button_cmd", lambda packer, CP, counter, btn: (btn, counter))
  CP = SimpleNamespace(flags=MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.AUTO_RESUME,
                       carFingerprint="MAZDA_CX5", maxLateralAccel=1.09, steerRatio=15.5, wheelbase=2.7,
                       openpilotLongitudinalControl=False)
  params = W.FakeParams({"IsMetric": False})
  slow = cs.CurveSlowdown(CP, params, curve_enabled=False, stop_mode=2, learning=False, map_mode=2, limit_mode=2)
  auto = ar.AutoResume(CP, params)
  for rec in (slow.recorder, auto.recorder):
    if rec is not None:
      rec.root = str(tmp_path / "events")
      monkeypatch.setattr(rec, "record", lambda *a, **k: True)
  car = T.Car(stored_mph, dash_offset_mph=0.0, lag=10, accel=0.6, step=MPH, cruise_gain=0.19, cruise_dead=1.0)
  car.v_ego = v0_mph * MPH
  r = dict(sets=[], vs=[], presses=[], ours=[], engage=None, on=[])
  on, brake, gas, gas_done, engage_at, stop_until = True, False, False, False, None, None
  plan = sm.msgs["starpilotPlan"]
  for fr in range(frames):
    for k, v in plan_fn(fr).items():
      setattr(plan, k, v)
    if mapd_fn is not None:
      for k, v in mapd_fn(fr).items():
        setattr(sm.msgs["mapdOut"], k, v)
    if fr == brake_at:
      brake = True
    if brake and fr == brake_at + 3:
      on = False                                           # the cruise falls 30 ms after his brake
    if not on and r["engage"] is None:
      if brake:
        car.v_ego = max(car.v_ego - 3.0 / F, 0.0)
        if car.v_ego == 0.0:
          stop_until = stop_until or fr + int(stop_s * F)
          if fr >= stop_until:
            brake = False
      elif stop_until is not None and not gas_done:
        gas = True
    if gas:
      if car.v_ego >= gas_until_mph * MPH:
        gas, gas_done = False, True                        # his foot off once up to speed (cruise on or not)
      else:
        car.v_ego += 1.5 / F
    if engage_at is not None and fr >= engage_at:
      on, engage_at = True, None
      r["engage"] = fr
    his_res = who == "his" and engage_frame is not None and engage_frame - 15 <= fr < engage_frame
    if who == "his" and engage_frame is not None and fr == engage_frame - 1 and r["engage"] is None:
      engage_at = fr + 1
    if prime is not None and fr == brake_at + 5:
      prime(slow)                                          # (with the cruise off: as left by a stop slowdown)
    out = SimpleNamespace(cruiseState=SimpleNamespace(enabled=on, available=True, speed=car.set_speed), vEgo=car.v_ego,
                          aEgo=0.0, gasPressed=gas, brakePressed=brake, standstill=car.v_ego < 0.1, leftBlinker=False,
                          rightBlinker=False, steeringPressed=False, steeringAngleDeg=0.0, steeringRateDeg=0.0,
                          gearShifter=DRIVE, doorOpen=False, seatbeltUnlatched=False, canValid=True)
    CS = SimpleNamespace(out=out, crz_set_minus=False, crz_set_plus=False, crz_cancel=False, crz_resume=his_res,
                         crz_btns_counter=(fr // 10) % 16, radar_has_lead=False, radar_lead_distance=0,
                         crz_set_allowed=not brake, crz_not_allowed=False)
    CC = SimpleNamespace(enabled=on, cruiseControl=SimpleNamespace(cancel=False, resume=False), latActive=True,
                         actuators=SimpleNamespace(torque=0.0))
    sent = auto.update(CC, CS, None, CP, slow, False) if who == "ours" else []
    if sent:
      r["ours"].append(fr)
      if not on and engage_at is None:
        engage_at = fr + 3                                  # the car takes our RES: on at the stored speed
    btns = slow.update(CC, CS, None, CP, bool(auto.busy) if who == "ours" else False)
    for btn, _ in btns:
      b = cs.SET_MINUS if btn == Buttons.SET_MINUS else cs.SET_PLUS
      r["presses"].append((fr, b, slow.logic.last_action))
      if slow.logic.last_action == "hold":
        car.hold_frame(b)
      else:
        car.press(b)
    if on:
      car.tick()
    else:
      car.frame += 1
      car._hold_tick()
    r["sets"].append(car.set_speed / MPH)
    r["vs"].append(car.v_ego / MPH)
    r["on"].append(on)
  r["slow"], r["auto"] = slow, auto
  return r


def both(monkeypatch, tmp_path, **kw):
  ours = run(monkeypatch, tmp_path, "ours", **kw)
  assert ours["engage"] is not None and ours["ours"], "our RES did not engage"
  his = run(monkeypatch, tmp_path, "his", engage_frame=ours["engage"], **kw)
  assert his["engage"] == ours["engage"]
  return ours, his


def first_reach(r, target, start, below=True):
  for k in range(start, len(r["sets"])):
    if (r["sets"][k] <= target + 1e-6) if below else (r["sets"][k] >= target - 1e-6):
      return k
  return None


def summary(r):
  e = r["engage"]
  p = [(round((f - e) / F, 2), b[4:], a) for f, b, a in r["presses"] if f >= e]
  return p


def street(limit_mph, rc=7, before=None, until=900, **extra):
  """The street from frame `until` on (just before his brake); `before`: (limit, class) of the road before it (a highway
  at his speed: nothing lowered before the brake), None: the street all along."""
  d = dict(slcSpeedLimit=limit_mph * MPH, slcSpeedLimitSource="Map Data" if limit_mph else "None", mapGpsTrust=0,
           mapRoadClass=rc, mapCurveMode=2, redLight=False)
  d.update(extra)
  if before is None:
    return lambda f: d
  b = dict(d, slcSpeedLimit=before[0] * MPH, slcSpeedLimitSource="Map Data" if before[0] else "None", mapRoadClass=before[1])
  return lambda f: b if f < until else d


def test_6f_815_rule_8_lowers_64_to_50_on_a_45_street_as_after_his_res(monkeypatch, tmp_path):
  ours, his = both(monkeypatch, tmp_path, stored_mph=64, v0_mph=64, plan_fn=street(45, before=(0, 1)))
  e = ours["engage"]
  assert summary(ours) == summary(his), (summary(ours)[:6], summary(his)[:6])
  assert ours["sets"][e:] == his["sets"][e:]
  # rule 8: down to 45 + 5 (Street Speed default) with a SET- hold first (the gap >= 2 steps), never under 50
  assert abs(ours["sets"][-1] - 50) < 1e-6 and min(ours["sets"][e:]) >= 50 - 1e-6
  acts = [a for _, b, a in summary(ours) if b == "MINUS"]
  assert acts and acts[0] == "hold", summary(ours)[:5]
  t50 = first_reach(ours, 50, e)
  print("6f 815: 64 -> 50 first press", summary(ours)[0][0], "s, at 50 after", round((t50 - e) / F, 1), "s;",
        "presses", len(acts), acts[:6])
  assert t50 is not None and (t50 - e) / F < 15.0


def test_no_limit_known_street_only_to_43_and_only_with_the_car_at_50_or_less(monkeypatch, tmp_path):
  # no limit on the map, a street: rule 8's fallback, down to 43, only with the car at most 50 mph
  ours, his = both(monkeypatch, tmp_path, stored_mph=60, v0_mph=60, plan_fn=street(0, before=(0, 1)), gas_until_mph=25)
  e = ours["engage"]
  assert summary(ours) == summary(his) and ours["sets"][e:] == his["sets"][e:]
  assert abs(ours["sets"][-1] - 43) < 1e-6, ours["sets"][-1]
  print("no limit, 60 stored, car at 25 when on: ->", ours["sets"][-1], "at", round((first_reach(ours, 43, e) - e) / F, 1), "s")


def test_res_restore_after_our_stop_lowering(monkeypatch, tmp_path):
  # our stop slowdown had lowered his 45 to 19 (the anchor 45) when he braked to the stop: the RES brings 19 back, the
  # RES restore takes it to 45 (a 40 street: rule 8's 40 + 5 is his 45, no match needed)
  def prime(slow):
    lg = slow.logic
    lg.anchor, lg.stop_episode, lg.last_set_speed, lg.cruise_gap = 45 * MPH, True, 19 * MPH, True
  ours, his = both(monkeypatch, tmp_path, stored_mph=19, v0_mph=19, plan_fn=street(40), prime=prime)
  e = ours["engage"]
  assert summary(ours) == summary(his) and ours["sets"][e:] == his["sets"][e:]
  assert abs(ours["sets"][-1] - 45) < 1e-6, ours["sets"][-1]
  t45 = first_reach(ours, 45, e, below=False)
  print("RES restore 19 -> 45:", summary(ours)[:3], "at 45 after", round((t45 - e) / F, 1), "s")


def test_ramp_start_on_a_highway_link(monkeypatch, tmp_path):
  # a street, then a link to a highway (70 ahead) where he braked to a stop at its light and gassed: the merge raises
  def plan_fn(f):
    if f < 800:
      return street(45)(f)
    if f >= 5000:      # onto the highway
      return dict(slcSpeedLimit=70 * MPH, slcSpeedLimitSource="Map Data", mapGpsTrust=0, mapRoadClass=1, mapCurveMode=2,
                  redLight=False, mapOnRamp=False, mapRampToHighway=False)
    return dict(slcSpeedLimit=45 * MPH, slcSpeedLimitSource="Map Data", mapGpsTrust=0, mapRoadClass=2, mapCurveMode=2,
                redLight=False, mapOnRamp=True, mapRampToHighway=True)

  def mapd_fn(f):
    return dict(nextSpeedLimit=70 * MPH, nextSpeedLimitDistance=600.0) if f >= 800 else {}
  ours, his = both(monkeypatch, tmp_path, stored_mph=45, plan_fn=plan_fn, mapd_fn=mapd_fn, gas_until_mph=25,
                   frames=9000)
  e = ours["engage"]
  assert summary(ours) == summary(his) and ours["sets"][e:] == his["sets"][e:]
  top = max(ours["sets"][e:5000])
  t70 = first_reach(ours, 70, e, below=False)
  print("ramp start: 45 stored -> up to", round(top), "on the link (merge boost), first SET+", summary(ours)[0][0],
        "s, at 70 after", round((t70 - e) / F, 1), "s; on the highway ->", round(ours["sets"][-1]))
  assert top > 45 + 1e-6 and abs(ours["sets"][-1] - 75) < 1e-6


def test_the_limit_cap_stops_the_restore_at_a_lower_limit(monkeypatch, tmp_path):
  # as above, but the street's limit fell 40 -> 30 while he was stopped (a steady lower limit, rule 3's cap): the RES
  # restore stops at it plus the Street Speed offset (35), not at his old 45
  def prime(slow):
    lg = slow.logic
    lg.anchor, lg.stop_episode, lg.last_set_speed, lg.cruise_gap = 45 * MPH, True, 19 * MPH, True

  def plan_fn(f):
    return street(40 if f < 1500 else 30)(f)
  ours, his = both(monkeypatch, tmp_path, stored_mph=19, v0_mph=19, plan_fn=plan_fn, prime=prime)
  e = ours["engage"]
  assert summary(ours) == summary(his) and ours["sets"][e:] == his["sets"][e:]
  print("limit cap: 19 stored, his 45, the limit 40 -> 30 at the stop ->", round(ours["sets"][-1]), summary(ours)[:3])
  assert abs(ours["sets"][-1] - 35) < 1e-6, ours["sets"][-1]
