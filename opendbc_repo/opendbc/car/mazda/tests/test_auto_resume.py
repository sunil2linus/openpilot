"""The gas brings the cruise back after a brake (auto_resume.py, MazdaBrakeAutoResume): the pure logic, driven frame by
frame with a small model of the car (RES brings the stored speed back, SET- engages at the current speed)."""
from opendbc.car.mazda import auto_resume as ar
from opendbc.car.mazda.auto_resume import AutoResumeInputs, AutoResumeLogic, RES

MPH = 0.44704
F = 100   # frames per second


class Sim:
  """A drive at 100 Hz. The wheel's CRZ_BTNS counter steps every 10 frames. The car takes our RES when `accepts` (the
  cruise on CAR_DELAY frames later at the stored set speed)."""
  CAR_DELAY = 3

  def __init__(self, metric=False, set_mph=45, v_mph=45, accepts=True, **kw):
    self.lg = AutoResumeLogic(metric)
    self.i = dict(cruise_enabled=True, cruise_available=True, set_speed=set_mph * MPH, v_ego=v_mph * MPH,
                  gas_pressed=False, brake_pressed=False, plan_alive=True, road_class=5, gps_trust=0, engageable=True,
                  set_allowed=True, btn_counter=0, can_valid=True)
    self.i.update(kw)
    self.accepts = accepts
    self.sent = []          # (frame, button)
    self.events = []
    self.engage_at = None
    self.frame = 0

  def step(self, n=1, **kw):
    self.i.update(kw)
    out = []
    for _ in range(n):
      self.frame += 1
      self.i["btn_counter"] = (self.frame // 10) % 16
      if self.engage_at is not None and self.frame >= self.engage_at[0]:
        self.i["cruise_enabled"] = True
        self.engage_at = None
      b = self.lg.update(AutoResumeInputs(**self.i))
      self.events += self.lg.events
      self.lg.events.clear()
      if b is not None:
        self.sent.append((self.frame, b))
        out.append(b)
        if self.accepts and not self.i["cruise_enabled"] and self.engage_at is None:
          self.engage_at = (self.frame + self.CAR_DELAY, b)
    return out

  def kinds(self, kind):
    return [d for k, d in self.events if k == kind]

  def brake_cancel(self, on_s=10, v_after_mph=None, brake_s=1.0):
    """Cruise on `on_s`, his brake turns it off (the cruise falls 3 frames after the brake), brake held `brake_s`."""
    self.step(on_s * F, cruise_enabled=True)
    self.step(3, brake_pressed=True)
    self.step(1, cruise_enabled=False)
    if v_after_mph is not None:
      self.i["v_ego"] = v_after_mph * MPH
    self.step(int(brake_s * F))
    self.step(1, brake_pressed=False)


def test_highway_brake_then_gas_resumes():
  s = Sim(set_mph=70, v_mph=70, road_class=1)
  s.brake_cancel(v_after_mph=62)
  assert s.lg.state == ar.ARMED and s.kinds("autoResumeArmed")[0]["map_ok"]
  s.step(3 * F)                                  # coasting, no gas: nothing
  assert s.sent == []
  s.step(int(0.29 * F), gas_pressed=True)        # under GAS_HOLD: not a press yet
  assert s.sent == []
  s.step(F)
  assert s.sent and s.sent[0][1] == RES and s.i["cruise_enabled"]
  assert s.lg.state == ar.DONE and s.kinds("autoResumeSent")[0]["ok"] and s.kinds("autoResumeSent")[0]["button"] == RES
  # the burst stopped once the cruise was on, and the frames went on the wheel's new frames, >= 0.1 s apart
  assert 1 <= len(s.sent) <= ar.BURST_FRAMES
  assert all(b - a >= ar.FRAME_GAP for (a, _), (b, _) in zip(s.sent, s.sent[1:], strict=False))
  assert all(f % 10 == 0 for f, _ in s.sent)
  assert s.lg.busy


def test_not_armed_unless_his_brake_after_two_seconds_of_cruise():
  # his CANCEL (no brake)
  s = Sim()
  s.step(10 * F)
  s.step(1, cruise_enabled=False, wheel_cancel=True)
  s.step(10, wheel_cancel=False)
  s.step(2 * F, gas_pressed=True)
  assert s.lg.state == ar.OFF and s.sent == []
  # the cruise on under ARM_CRUISE_ON
  s = Sim()
  s.step(F, cruise_enabled=False)
  s.brake_cancel(on_s=1)
  s.step(2 * F, gas_pressed=True)
  assert s.lg.state == ar.OFF and s.sent == [] and s.kinds("autoResumeRefused")[0]["reason"] == "cruise on too short"
  # a CANCEL frame of ours just before (openpilot cancelled, the brake came with it)
  s = Sim()
  s.step(10 * F)
  s.step(5, cancel_sent=True)
  s.step(1, cancel_sent=False)
  s.step(2, brake_pressed=True)
  s.step(1, cruise_enabled=False)
  s.step(10, brake_pressed=False)
  s.step(2 * F, gas_pressed=True)
  assert s.lg.state == ar.OFF and s.sent == []
  # the main switch off with it
  s = Sim()
  s.step(10 * F)
  s.step(3, brake_pressed=True)
  s.step(1, cruise_enabled=False, cruise_available=False)
  assert s.lg.state == ar.OFF


def test_stop_sign_with_a_turn_waits_for_the_wheel():
  s = Sim(set_mph=25, v_mph=25)
  s.brake_cancel(v_after_mph=0, brake_s=2.0)
  # pulls away turning right: the gas down, the wheel at 300 deg at 12 mph (~2.9 m/s^2)
  s.step(2 * F, gas_pressed=True, v_ego=12 * MPH, steering_angle=-300.0)
  assert s.sent == [] and s.lg.press is not None
  # the wheel unwinds (not winding up: |angle| falling) and is straight; the gas still down
  s.step(int(0.4 * F), steering_angle=0.0, v_ego=16 * MPH)
  assert s.sent == []                          # not calm CALM_TIME yet
  s.step(int(0.3 * F))
  assert s.sent and s.sent[0][1] == RES
  assert s.lg.state == ar.DONE


def test_turn_with_the_gas_lifted_more_than_three_seconds_drops_that_press():
  s = Sim(set_mph=25, v_mph=25)
  s.brake_cancel(v_after_mph=0, brake_s=2.0)
  s.step(int(0.5 * F), gas_pressed=True, v_ego=10 * MPH, steering_angle=300.0)
  s.step(4 * F, gas_pressed=False)
  s.step(2 * F, steering_angle=0.0)
  assert s.sent == [] and s.lg.state == ar.ARMED and s.kinds("autoResumeRefused")[-1]["reason"] == "gas released"
  # the panda only passes a frame with the gas down: a press made in the turn, the gas lifted when the wheel is straight
  # (within GAS_RECENT): we wait for the gas, then it goes at once
  s.step(int(0.4 * F), gas_pressed=True, steering_angle=300.0, v_ego=16 * MPH)
  s.step(F, gas_pressed=False, steering_angle=0.0)
  assert s.sent == [] and s.lg.press is not None and s.lg.wait_why == "gas"
  s.step(int(0.15 * F), gas_pressed=True)
  assert s.sent and s.sent[0][1] == RES and s.sent[0][0] - s.frame > -int(0.15 * F) - 1


def test_red_light_five_minutes_then_gas():
  s = Sim(set_mph=45, v_mph=45)
  s.brake_cancel(v_after_mph=0, brake_s=3.0)
  s.step(300 * F, brake_pressed=False)          # 5 minutes at the light (brake released, held by the car)
  assert s.lg.state == ar.ARMED and s.lg.dist == 0.0
  s.step(int(0.3 * F), gas_pressed=True, v_ego=2 * MPH)
  s.step(2 * F, v_ego=5 * MPH)
  # under 10 mph with no car ahead: wait (review G7: a creep away from a light, the car ahead not moving yet)
  assert s.sent == [] and s.lg.wait_why == "slow, no moving lead" and s.lg.state == ar.ARMED
  s.step(int(0.5 * F), v_ego=11 * MPH)
  # 45 stored, 11 now: RES all the same (the stored speed comes back; the street rules take it down from there)
  assert [b for _, b in s.sent] == [RES]
  assert s.i["cruise_enabled"] and abs(s.i["set_speed"] - 45 * MPH) < 1e-6
  assert s.lg.state == ar.DONE and s.kinds("autoResumeSent")[0]["button"] == RES


def test_parking_lot_then_a_public_road_within_the_distance():
  s = Sim(set_mph=30, v_mph=30, road_class=12)
  s.brake_cancel(v_after_mph=8)
  # into a parking lot: no road on the map
  s.step(6 * F, road_class=0)
  s.step(F, gas_pressed=True)
  assert s.sent == [] and s.lg.state == ar.ARMED and s.kinds("autoResumeRefused")[-1]["reason"] == "map"
  # the same press held on: still not used
  s.step(2 * F)
  assert s.sent == []
  # back on a residential street (still < 800 m), gas again
  s.step(F, gas_pressed=False, road_class=12)
  s.step(int(0.9 * F), gas_pressed=True, v_ego=22 * MPH)
  assert s.sent == [] and s.lg.wait_why == "road just changed"     # the road only 1.9 s old
  s.step(int(0.5 * F))
  assert s.sent and s.sent[0][1] == RES and s.lg.state == ar.DONE


def test_unknown_road_or_low_trust_is_off():
  for kw in (dict(road_class=0), dict(gps_trust=1), dict(plan_alive=False), dict(road_class=13)):
    s = Sim(**kw)
    s.brake_cancel(v_after_mph=40)
    s.step(2 * F, gas_pressed=True)
    assert s.sent == [], kw


def test_the_map_must_be_good_at_the_send_and_the_road_settled():
  # (review of round n, B1) no allowance for a lost map: good at the brake, lost when he presses the gas: no RES
  s = Sim(set_mph=45, v_mph=45, road_class=5)
  s.brake_cancel(v_after_mph=30)
  s.step(3 * F, road_class=0)
  s.step(F, gas_pressed=True)
  assert s.sent == [] and s.kinds("autoResumeRefused")[-1]["reason"] == "map"
  # a press made on the road, the map lost before the frame (turning into a driveway): dropped
  s = Sim(set_mph=45, v_mph=45, road_class=5)
  s.brake_cancel(v_after_mph=30)
  s.step(int(0.4 * F), gas_pressed=True, steering_angle=200.0)
  assert s.lg.press is not None
  s.step(F, road_class=0, steering_angle=0.0)
  assert s.sent == [] and s.lg.press is None and s.kinds("autoResumeRefused")[-1]["reason"] == "map"
  # the class (or mapd's name) changed less than 2 s before: wait for it to settle
  for kw in (dict(road_class=7), dict(road_name="Other Rd")):
    s = Sim(set_mph=45, v_mph=45, road_class=5)
    s.brake_cancel(v_after_mph=30)
    s.step(F)
    s.step(int(1.5 * F), gas_pressed=True, **kw)
    assert s.sent == [] and s.lg.wait_why == "road just changed", kw
    s.step(F)
    assert s.sent, kw
  # standing still on a good road never counts against it
  s = Sim(set_mph=45, v_mph=45)
  s.brake_cancel(v_after_mph=0)
  s.step(60 * F)
  s.step(int(0.3 * F), gas_pressed=True, v_ego=12 * MPH)
  s.step(F)
  assert s.sent


def test_the_wheel_angle_must_be_small():
  # (review B2) at junction speed the lateral accel is ~0 with the wheel at 150 deg: still turning
  s = Sim(set_mph=45, v_mph=45)
  s.brake_cancel(v_after_mph=11)
  s.step(2 * F, gas_pressed=True, steering_angle=110.0)
  assert abs(s.lg.lat) < 1.0 and s.sent == [] and s.lg.wait_why == "turning"
  s.step(int(0.4 * F), steering_angle=40.0)
  assert s.sent == []                        # under 45 deg, not for CALM_TIME yet
  s.step(int(0.2 * F))
  assert s.sent


def test_creep_guard_needs_a_moving_lead_under_10_mph():
  base = dict(set_mph=45, v_mph=45)
  for lead, ok in ((dict(lead_status=True, lead_d=15.0, lead_v=3.0), True),      # moving, ahead, not slower
                   (dict(lead_status=True, lead_d=15.0, lead_v=1.0), False),     # under 3 mph
                   (dict(lead_status=True, lead_d=60.0, lead_v=3.0), False),     # too far
                   (dict(lead_status=True, lead_d=15.0, lead_v=1.5), False),     # 1.5 < v 3.1 - 1
                   (dict(lead_status=False, lead_d=15.0, lead_v=3.0), False)):   # no lead
    s = Sim(**base)
    s.brake_cancel(v_after_mph=0)
    s.step(F)
    s.step(2 * F, gas_pressed=True, v_ego=7 * MPH, **lead)
    assert bool(s.sent) == ok, lead
  # openpilot's CAN not valid: no frame
  s = Sim(**base)
  s.brake_cancel(v_after_mph=20)
  s.step(2 * F, gas_pressed=True, can_valid=False)
  assert s.sent == [] and s.lg.wait_why == "can invalid"


def test_his_res_while_we_send_is_his():
  s = Sim(accepts=False)
  s.brake_cancel(v_after_mph=30)
  s.step(int(0.4 * F), gas_pressed=True)
  assert s.sent and s.lg.state == ar.SENDING
  s.step(1, wheel_resume=True, cruise_enabled=True)
  assert s.lg.state == ar.OFF and not s.kinds("autoResumeSent")


def test_his_buttons_and_the_car_state_disarm():
  for kw in (dict(wheel_cancel=True), dict(wheel_set=True), dict(wheel_resume=True), dict(gear_drive=False),
             dict(door_open=True), dict(seatbelt_unlatched=True), dict(cruise_available=False), dict(set_speed=0.0)):
    s = Sim()
    s.brake_cancel(v_after_mph=30)
    s.step(1, **kw)
    s.step(1, **{k: type(v)(not v) if isinstance(v, bool) else 45 * MPH for k, v in kw.items()})
    s.step(2 * F, gas_pressed=True)
    assert s.sent == [] and s.lg.state == ar.OFF, kw
    assert s.kinds("autoResumeRefused")[-1]["disarmed"]
  # the car's "not allowed" state: a blip does not disarm, NOT_ALLOWED_TIME does
  s = Sim()
  s.brake_cancel(v_after_mph=30)
  s.step(int(0.3 * F), not_allowed=True)
  s.step(1, not_allowed=False)
  assert s.lg.state == ar.ARMED
  s.step(int(0.6 * F), not_allowed=True)
  assert s.lg.state == ar.OFF and s.kinds("autoResumeRefused")[-1]["reason"] == "car not allowed"
  # ACC_SET_ALLOWED off is only "not now"
  s = Sim()
  s.brake_cancel(v_after_mph=30)
  s.step(2 * F, gas_pressed=True, set_allowed=False)
  assert s.sent == [] and s.lg.state == ar.ARMED and s.lg.wait_why == "car not ready"
  s.step(F, set_allowed=True)
  assert s.sent
  # openpilot's no-entry: not now either
  s = Sim()
  s.brake_cancel(v_after_mph=30)
  s.step(2 * F, gas_pressed=True, engageable=False)
  assert s.sent == [] and s.lg.wait_why == "openpilot no entry"


def test_distance_limit():
  s = Sim(set_mph=45, v_mph=45, road_class=0)       # never a mapped road: his presses refused, then too far
  s.brake_cancel(v_after_mph=40)
  s.step(int(800 / (40 * MPH) * F) + 50)
  assert s.lg.state == ar.OFF and s.kinds("autoResumeRefused")[-1]["reason"] == "distance"


def test_moving_time_limit_mirrors_the_panda():
  # a crawl: 5 mph for 175 s is ~390 m (under the distance) but past the panda's 180 s window less a margin
  s = Sim(set_mph=45, v_mph=45, road_class=0)
  s.brake_cancel(v_after_mph=5)
  s.step(int(ar.AUTO_RESUME_MAX_MOVING * F) - 150)    # (the 1 s with the brake down after the fall counts too)
  assert s.lg.state == ar.ARMED
  s.step(100)
  assert s.lg.state == ar.OFF and s.kinds("autoResumeRefused")[-1]["reason"] == "moving time"
  # the brake released a frame before the cruise fell: the panda opens no window, nor do we
  s = Sim()
  s.step(10 * F)
  s.step(3, brake_pressed=True)
  s.step(1, brake_pressed=False, cruise_enabled=False)
  s.step(2 * F, gas_pressed=True)
  assert s.lg.state == ar.OFF and s.sent == []


def test_big_stored_speed_still_sends_res():
  # (architect: the SET- path was dropped) 70 stored at 40 mph: RES, never a SET-
  s = Sim(set_mph=70, v_mph=70, road_class=1)
  s.brake_cancel(v_after_mph=40)
  s.step(2 * F, gas_pressed=True, v_ego=45 * MPH)
  assert [b for _, b in s.sent] == [RES] and abs(s.i["set_speed"] - 70 * MPH) < 1e-6
  assert not hasattr(ar, "AUTO_RESUME_MAX_JUMP") and not hasattr(ar, "SET_MINUS")


def test_no_response_one_retry_then_give_up():
  s = Sim(accepts=False)
  s.brake_cancel(v_after_mph=40)
  s.step(5 * F, gas_pressed=True)
  assert [b for _, b in s.sent] == [RES] * 8 and s.lg.state == ar.GAVE_UP
  assert [d["reason"] for d in s.kinds("autoResumeRefused")] == ["no response, retry", "no response"]
  # never again until the cruise comes on by other means, whatever he does
  s.step(F, gas_pressed=False)
  s.step(3 * F, gas_pressed=True)
  assert len(s.sent) == 8
  # his own engagement later clears it; a new brake cancel arms again
  s.step(1, cruise_enabled=True)
  assert s.lg.state == ar.OFF
  s.brake_cancel(v_after_mph=30)
  assert s.lg.state == ar.ARMED


def test_openpilot_refusing_gives_up_no_loop():
  s = Sim()
  s.brake_cancel(v_after_mph=40)
  s.step(F, gas_pressed=True)
  assert s.lg.state == ar.DONE
  n = len(s.sent)
  # openpilot does not engage: it asks for CANCEL, carcontroller sends it, the cruise goes off
  s.step(5, op_cancel=True)
  assert s.lg.state == ar.GAVE_UP and s.kinds("autoResumeRefused")[-1]["reason"] == "openpilot cancel"
  s.step(1, cancel_sent=True)
  s.step(1, cruise_enabled=False, cancel_sent=False, op_cancel=False)
  s.step(3 * F, gas_pressed=True)
  assert len(s.sent) == n and s.lg.state == ar.GAVE_UP
  # a one-frame cancel request on the rise (openpilot's own lag) is not a refusal
  s = Sim()
  s.brake_cancel(v_after_mph=40)
  s.step(int(0.5 * F), gas_pressed=True)
  s.step(1, op_cancel=True)
  s.step(3 * F, op_cancel=False)
  assert s.lg.state == ar.DONE
  # his brake again later (after REFUSE_WINDOW) arms again: a new stop, a new resume
  s.step(1, gas_pressed=False)
  s.brake_cancel(on_s=3, v_after_mph=30)
  assert s.lg.state == ar.ARMED


def test_hand_off_with_curve_slowdown():
  s = Sim()
  s.brake_cancel(v_after_mph=40)
  s.step(2 * F, gas_pressed=True, slowdown_busy=True)
  assert s.sent == [] and s.lg.wait_why == "slowdown busy"
  s.step(F, slowdown_busy=False)
  assert s.sent
  # and ours keeps curve_slowdown quiet while sending and BUSY_AFTER / OWN_ENGAGE_TIME after
  assert s.lg.busy
  s.step(int(ar.OWN_ENGAGE_TIME * F) + 5)
  assert not s.lg.busy
  # openpilot's own RESUME / CANCEL this frame: none of ours
  s = Sim()
  s.brake_cancel(v_after_mph=40)
  s.step(2 * F, gas_pressed=True, other_button_active=True)
  assert s.sent == []


def test_brake_during_the_burst_stops_it():
  s = Sim(accepts=False)
  s.brake_cancel(v_after_mph=40)
  s.step(int(0.35 * F), gas_pressed=True)
  k = len(s.sent)
  assert 1 <= k < 4
  s.step(F, gas_pressed=False, brake_pressed=True)
  assert len(s.sent) == k and s.lg.state == ar.ARMED
  s.step(1, brake_pressed=False)
  s.step(3 * F, gas_pressed=True)
  # the second (last) attempt, then given up: never more than the panda's budget
  assert len(s.sent) <= ar.MAX_FRAMES and s.lg.state == ar.GAVE_UP


def test_gas_down_at_the_brake_is_not_his_press():
  s = Sim()
  s.step(10 * F)
  s.step(3, brake_pressed=True, gas_pressed=True)
  s.step(1, cruise_enabled=False)
  s.step(10, brake_pressed=False)
  s.step(2 * F)
  assert s.sent == []
  s.step(1, gas_pressed=False)
  s.step(F, gas_pressed=True)
  assert s.sent


def test_wrapper_factory_off_unless_the_toggle_and_the_flag():
  from types import SimpleNamespace
  from opendbc.car.mazda.values import MazdaSafetyFlags

  class P:
    def __init__(self, on):
      self.on = on

    def get_bool(self, k):
      return self.on if k == "MazdaBrakeAutoResume" else False
  gen1 = SimpleNamespace(flags=MazdaSafetyFlags.GEN1, openpilotLongitudinalControl=False)
  flagged = SimpleNamespace(flags=MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.AUTO_RESUME, openpilotLongitudinalControl=False)
  assert ar.enabled(gen1, P(True)) is False          # the panda flag missing: never
  assert ar.enabled(flagged, P(False)) is False       # the toggle off: never
  assert ar.enabled(flagged, P(True)) is True
  ri = SimpleNamespace(flags=flagged.flags | MazdaSafetyFlags.RADAR_INTERCEPTOR, openpilotLongitudinalControl=True)
  assert ar.enabled(ri, P(True)) is False


def test_wrapper_reads_the_plan_and_selfdrive_state_and_builds_the_frame(tmp_path, monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  from types import SimpleNamespace
  import cereal.messaging as messaging
  from opendbc.car import structs
  from opendbc.car.mazda import mazdacan
  from opendbc.car.mazda.values import Buttons, MazdaSafetyFlags

  class SM:
    def __init__(self):
      self.msgs = {"starpilotPlan": SimpleNamespace(mapRoadClass=5, mapGpsTrust=0),
                   "selfdriveState": SimpleNamespace(engageable=True),
                   "radarState": SimpleNamespace(leadOne=SimpleNamespace(status=False, dRel=0.0, vLead=0.0)),
                   "mapdOut": SimpleNamespace(roadName="A Road")}
      self.alive = dict.fromkeys(self.msgs, True)
      self.recv_frame = dict.fromkeys(self.msgs, 1)

    def update(self, timeout):
      pass

    def __getitem__(self, k):
      return self.msgs[k]
  sm = SM()
  monkeypatch.setattr(messaging, "SubMaster", lambda services: sm)
  frame = [0]
  monkeypatch.setattr(mazdacan, "create_button_cmd", lambda packer, CP, counter, btn: ("frame", counter, btn, frame[0]))
  CP = SimpleNamespace(flags=MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.AUTO_RESUME, steerRatio=15.5, wheelbase=2.7,
                       openpilotLongitudinalControl=False)

  class Params:
    def get_bool(self, k):
      return False
  w = ar.AutoResume(CP, Params())
  records = []
  if w.recorder is not None:
    w.recorder.root = str(tmp_path / "events")
    monkeypatch.setattr(w.recorder, "record", lambda kind, t, data, **kw: records.append((kind, data)))
  st = dict(enabled=True, brake=False, gas=False, v=40 * MPH)

  def run(n, slowdown=None, **kw):
    st.update(kw)
    out = []
    for _ in range(n):
      frame[0] += 1
      cs = SimpleNamespace(out=SimpleNamespace(
        cruiseState=SimpleNamespace(enabled=st["enabled"], available=True, speed=45 * MPH), vEgo=st["v"],
        gasPressed=st["gas"], brakePressed=st["brake"], steeringAngleDeg=0.0, steeringRateDeg=0.0,
        gearShifter=structs.CarState.GearShifter.drive, doorOpen=False, seatbeltUnlatched=False, canValid=True),
        crz_cancel=False, crz_resume=False, crz_set_plus=False, crz_set_minus=False, crz_set_allowed=True,
        crz_not_allowed=False, crz_btns_counter=(frame[0] // 10) % 16)
      cc = SimpleNamespace(cruiseControl=SimpleNamespace(cancel=False, resume=False))
      out += w.update(cc, cs, None, CP, slowdown)
    return out
  run(3 * F)
  run(3, brake=True)
  run(1, enabled=False)
  run(F)
  run(1, brake=False)
  assert w.state == ar.ARMED
  # curve_slowdown with a press going holds ours back
  busy = SimpleNamespace(logic=SimpleNamespace(pending=None, hold=object(), frame=0, last_send_frame=-10_000, inflight=[]))
  assert run(2 * F, slowdown=busy, gas=True) == []
  # the map lost (and long enough): no frame either
  sm.msgs["starpilotPlan"].mapRoadClass = 0
  run(F, gas=False)
  run(6 * F)
  assert run(2 * F, gas=True) == []
  sm.msgs["starpilotPlan"].mapRoadClass = 5
  run(1, gas=False)
  out = run(3 * F, gas=True)               # (the road settled MAP_STABLE_TIME first)
  assert out and out[0][0] == "frame" and out[0][2] == Buttons.RESUME and w.logic.state == ar.SENDING
  # sent on the cycle a new wheel frame came, with its counter (create_button_cmd adds 1: CTR = wheel + 1)
  assert all(f % 10 == 0 and counter == (f // 10) % 16 for _, counter, _, f in out)
  if w.recorder is not None:
    kinds = [k for k, _ in records]
    assert "autoResumeArmed" in kinds and "autoResumeRefused" in kinds


def test_a_frozen_press_of_curve_slowdown_does_not_block_ours():
  # a press of curve_slowdown in flight when he braked stays `pending` (its timers frozen) while the cruise is off: only a
  # hold or a frame of its within BUSY_AFTER makes it busy
  from types import SimpleNamespace
  lg = SimpleNamespace(pending=object(), hold=None, frame=5000, last_send_frame=1000)
  assert ar.AutoResume.slowdown_busy(SimpleNamespace(logic=lg)) is False
  lg.last_send_frame = 4950
  assert ar.AutoResume.slowdown_busy(SimpleNamespace(logic=lg)) is True
  lg.last_send_frame, lg.hold = 1000, object()
  assert ar.AutoResume.slowdown_busy(SimpleNamespace(logic=lg)) is True
  assert ar.AutoResume.slowdown_busy(None) is False


def test_carcontroller_builds_it_only_with_the_toggle_and_the_flag(monkeypatch):
  import pytest
  pytest.importorskip("msgq.ipc_pyx", exc_type=ImportError)
  from types import SimpleNamespace
  import cereal.messaging as messaging
  from opendbc.car import Bus, structs
  from opendbc.car.mazda import carcontroller
  from opendbc.car.mazda.values import MazdaSafetyFlags

  class Params:
    def __init__(self, *a, **k):
      pass

    def get(self, key, encoding=None):
      return None

    def get_bool(self, key):
      return on[0] and key == "MazdaBrakeAutoResume"
  on = [False]
  monkeypatch.setattr(carcontroller, "Params", Params)
  monkeypatch.setattr(messaging, "SubMaster", lambda services: SimpleNamespace(update=lambda t: None))
  for toggle, flag, expect in ((False, False, False), (False, True, False), (True, False, False), (True, True, True)):
    on[0] = toggle
    CP = structs.CarParams()
    CP.flags = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR | (MazdaSafetyFlags.AUTO_RESUME if flag else 0))
    CP.openpilotLongitudinalControl = False
    c = carcontroller.CarController({Bus.pt: "mazda_2017"}, CP)
    assert (c.auto_resume is not None) == expect, (toggle, flag)


def test_crz_info_never_makes_can_invalid_and_a_stale_value_is_not_allowed():
  # (review) a missing 0x21B must not raise a CAN error: CRZ_INFO is registered with frequency 0 (only with the flag)
  from opendbc.can import CANPacker
  from opendbc.car import Bus, structs
  from opendbc.car.mazda.carstate import CarState
  from opendbc.car.mazda.values import CAR, MazdaSafetyFlags
  CP = structs.CarParams()
  CP.carFingerprint = CAR.MAZDA_CX5
  for flag in (0, MazdaSafetyFlags.AUTO_RESUME):
    CP.flags = int(MazdaSafetyFlags.GEN1 | flag)
    cp = CarState.get_can_parsers(CP)[Bus.pt]
    assert ((0x21B in cp.message_states) == bool(flag))
    if flag:
      assert cp.message_states[0x21B].ignore_alive
  packer = CANPacker("mazda_2017")
  addr, dat, _ = packer.make_can_msg("CRZ_INFO", 0, {"ACC_SET_ALLOWED": 1})
  t = 1_000_000_000
  cp.update([(t, [(addr, dat, 0)])])
  assert cp.vl["CRZ_INFO"]["ACC_SET_ALLOWED"] == 1 and cp.ts_nanos["CRZ_INFO"]["ACC_SET_ALLOWED"] == t
  # nothing on the bus for a long time: still valid (no alive check for it)
  for k in range(1, 200):
    cp.update([(t + k * 10_000_000, [])])
  assert cp.can_valid
