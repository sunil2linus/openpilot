"""(Stage 3) The gas brings the cruise back after a brake (speed/auto_resume.py, setting `auto_resume`): the pure logic,
driven frame by frame with a small model of the car (RES brings the stored speed back). Ported unchanged from the
owner's previous fork (opendbc/car/mazda/tests/test_auto_resume.py, the logic part); "curve_slowdown" there is the speed
control's actuator here (slowdown_busy)."""
from openpilot.starpilot.car.mazda.speed import auto_resume as ar
from openpilot.starpilot.car.mazda.speed.auto_resume import AutoResumeInputs, AutoResumeLogic, RES

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
  s.step(int(1.5 * F))
  assert s.sent == [] and s.lg.wait_why == "after a big turn"   # (a 300 deg turn: AUTO_RESUME_BIG_TURN_WAIT, 2 s)
  s.step(int(0.2 * F))
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
  s.step(int(0.4 * F), gas_pressed=True, steering_angle=80.0, v_ego=16 * MPH)     # (under the big turn's 90 deg)
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
  assert s.sent == [] and s.lg.state == ar.ARMED and s.kinds("autoResumeRefused")[-1]["reason"] == "map (waiting)"
  # the same press held on: never sent there (round Q: the map is a wait while his gas is held, the send needs it good)
  s.step(2 * F)
  assert s.sent == [] and s.lg.wait_why == "map"
  # back on a residential street (still < 800 m), gas again
  s.step(int(0.5 * F), gas_pressed=False, road_class=12)
  s.step(int(0.4 * F), gas_pressed=True, v_ego=22 * MPH)
  assert s.sent == [] and s.lg.wait_why == "road just changed"     # the road only 0.9 s old (MAP_STABLE_TIME 1 s)
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
  assert s.sent == [] and s.kinds("autoResumeRefused")[-1]["reason"] == "map (waiting)"
  s.step(3 * F)
  assert s.sent == []
  # a press made on the road, the map lost before the frame (turning into a driveway): never sent while it is lost
  s = Sim(set_mph=45, v_mph=45, road_class=5)
  s.brake_cancel(v_after_mph=30)
  s.step(int(0.4 * F), gas_pressed=True, steering_angle=200.0)
  assert s.lg.press is not None
  s.step(3 * F, road_class=0, steering_angle=0.0)
  assert s.sent == [] and s.lg.wait_why == "map"
  # the class (or mapd's name) changed less than MAP_STABLE_TIME (1 s) before: wait for it to settle
  for kw in (dict(road_class=7), dict(road_name="Other Rd")):
    s = Sim(set_mph=45, v_mph=45, road_class=5)
    s.brake_cancel(v_after_mph=30)
    s.step(F)
    s.step(int(0.8 * F), gas_pressed=True, **kw)
    assert s.sent == [] and s.lg.wait_why == "road just changed", kw
    s.step(F)
    assert s.sent, kw
  # (round Q) a class 0 blip under MAP_BLIP on the same road is no road change: sent once the map is good again
  s = Sim(set_mph=45, v_mph=45, road_class=5)
  s.brake_cancel(v_after_mph=30)
  s.step(3 * F)
  s.step(int(0.5 * F), gas_pressed=True, road_class=0)
  assert s.sent == [] and s.lg.wait_why == "map"
  s.step(int(0.5 * F), gas_pressed=True, road_class=5)
  assert s.sent == []                       # (review M6: the class good MAP_STABLE_TIME continuously first)
  s.step(int(0.7 * F), gas_pressed=True, road_class=5)
  assert s.sent
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
  s.step(2 * F, gas_pressed=True, steering_angle=80.0)
  assert abs(s.lg.lat) < 1.0 and s.sent == [] and s.lg.wait_why == "turning"
  s.step(int(0.4 * F), steering_angle=40.0)
  assert s.sent == []                        # under 45 deg, not for CALM_TIME yet
  s.step(int(0.2 * F))
  assert s.sent


def test_a_big_turn_of_the_wheel_waits_two_seconds():
  # (re-review, 6c t 1698.8) a U-turn on the same road: 200 deg, then straight, the gas held throughout
  def run(angle, turn_s=2.0):
    s = Sim(set_mph=53, v_mph=53)
    s.brake_cancel(v_after_mph=15)
    s.step(F)
    s.step(int(turn_s * F), gas_pressed=True, steering_angle=angle, v_ego=12 * MPH)
    t_below = s.frame                              # the angle drops below 90 (and 45) here
    s.step(5 * F, steering_angle=0.0, v_ego=18 * MPH)
    return s, t_below
  s, t0 = run(200.0)
  assert s.sent and abs((s.sent[0][0] - t0) / F - ar.AUTO_RESUME_BIG_TURN_WAIT) <= 0.15, (s.sent[0][0] - t0) / F
  # a 90+ deg junction turn: the same 2 s wait
  s, t0 = run(100.0)
  assert s.sent and (s.sent[0][0] - t0) / F >= ar.AUTO_RESUME_BIG_TURN_WAIT - 0.01
  # a 30 deg lane-position correction: only the 0.5 s calm (it never was over 45 deg: no wait at all)
  s, t0 = run(30.0)
  assert s.sent and (s.sent[0][0] - t0) / F < 0.2
  # the wait is time, not samples: 2 s whatever the wheel does under 90 deg meanwhile
  s = Sim(set_mph=53, v_mph=53)
  s.brake_cancel(v_after_mph=15)
  s.step(F)
  s.step(F, gas_pressed=True, steering_angle=-120.0, v_ego=12 * MPH)
  t0 = s.frame
  s.step(int(1.0 * F), steering_angle=60.0)
  s.step(int(0.8 * F), steering_angle=0.0, v_ego=18 * MPH)
  assert s.sent == [] and s.lg.wait_why == "after a big turn"
  s.step(F)
  assert s.sent and (s.sent[0][0] - t0) / F >= 2.0 - 0.01


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




def test_review_m6_into_a_parking_lot_a_flicker_back_to_a_public_class_never_sends():
  # (review of round Q, M6) a public road, his brake, into a lot (class 0) with his gas held, the class flickers back
  # to a public one for 0.5 s: no send (the class must be good MAP_STABLE_TIME continuously)
  s = Sim(set_mph=30, v_mph=30, road_class=7)
  s.brake_cancel(v_after_mph=12)
  s.step(int(1.0 * F), road_class=0, gas_pressed=True)
  s.step(int(0.5 * F), road_class=7, gas_pressed=True)
  s.step(int(2.0 * F), road_class=0, gas_pressed=True)
  assert s.sent == []
  # the same flicker shorter than MAP_BLIP inside the lot: none either
  s = Sim(set_mph=30, v_mph=30, road_class=7)
  s.brake_cancel(v_after_mph=12)
  s.step(int(3.0 * F), road_class=0, gas_pressed=True)
  s.step(int(0.8 * F), road_class=7, gas_pressed=True)
  s.step(int(1.0 * F), road_class=0, gas_pressed=True)
  assert s.sent == []
  # a name received long ago and another than at his brake: not the same public road for sure
  s = Sim(set_mph=30, v_mph=30, road_class=7, road_name="Main St")
  s.brake_cancel(v_after_mph=25)
  s.step(int(2.0 * F), gas_pressed=True, road_name="Lot Rd", road_name_last="Lot Rd", road_name_age=5.0)
  assert s.sent == [] and s.lg.wait_why == "road name not fresh"


def test_rereview_r6_an_old_name_equal_to_the_one_at_his_brake_still_sends():
  # (75 t 1298: mapdOut 11.8 s old, the same road as at his brake) the class public MAP_STABLE_TIME: sent
  s = Sim(set_mph=30, v_mph=30, road_class=7, road_name="Main St", road_name_last="Main St")
  s.brake_cancel(v_after_mph=25)
  s.step(int(2.0 * F), gas_pressed=True, road_name="", road_name_last="Main St", road_name_age=11.8)
  assert s.sent


def test_round2_no_res_on_a_link_under_25_mph_it_waits_for_a_proper_road():
  # (r2 rt 1166) his 46 came back at 11 mph inside Hall Rd's turnaround link (trunk_link, class 4): the car surged
  s = Sim(set_mph=46, v_mph=30, road_class=3)
  s.brake_cancel(v_after_mph=6)
  s.step(int(1.5 * F), road_class=4)                 # (the road settles on the link)
  s.step(2 * F, gas_pressed=True, v_ego=11 * MPH)
  assert s.sent == [] and s.lg.wait_why == "slow on a link"
  s.step(int(1.5 * F), gas_pressed=True, v_ego=20 * MPH, road_class=3)   # off the link, gas still held: it goes
  assert s.sent and s.sent[0][1] == RES
  # a ramp at 30 mph is no crawl in a turn: it sends
  s = Sim(set_mph=60, v_mph=40, road_class=2)
  s.brake_cancel(v_after_mph=30)
  s.step(int(1.5 * F))
  s.step(F, gas_pressed=True)
  assert s.sent
