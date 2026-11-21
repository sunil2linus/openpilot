"""The CAN pieces the speed control adds: SET+/SET- in create_button_cmd (CANCEL/RESUME frames unchanged) and the raw
wheel buttons / LKAS block / radar lead bit kept as carstate attributes (the CarState message itself unchanged)."""
from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.torque_interceptor import prepare
from opendbc.car.mazda.values import CAR, Buttons


def stage1_button_cmd(packer, counter, button):
  """create_button_cmd as it is on s7-mazda-ti (CANCEL / RESUME only)."""
  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  values = {"CAN_OFF": can, "CAN_OFF_INV": (can + 1) % 2, "SET_P": 0, "SET_P_INV": 1, "RES": res, "RES_INV": (res + 1) % 2,
            "SET_M": 0, "SET_M_INV": 1, "DISTANCE_LESS": 0, "DISTANCE_LESS_INV": 1, "DISTANCE_MORE": 0,
            "DISTANCE_MORE_INV": 1, "MODE_X": 0, "MODE_X_INV": 1, "MODE_Y": 0, "MODE_Y_INV": 1,
            "BIT1": 1, "BIT2": 1, "BIT3": 1, "CTR": (counter + 1) % 16}
  return packer.make_can_msg("CRZ_BTNS", 0, values)


def cp():
  return prepare(CarInterface.get_params(CAR.MAZDA_CX5, gen_empty_fingerprint(), [], False, False, False), True)


def test_cancel_and_resume_frames_are_unchanged():
  packer = CANPacker("mazda_2017")
  for button in (Buttons.CANCEL, Buttons.RESUME):
    for counter in range(16):
      assert mazdacan.create_button_cmd(packer, cp(), counter, button) == stage1_button_cmd(packer, counter, button)


def test_set_plus_and_set_minus_frames():
  from opendbc.can import CANParser
  packer = CANPacker("mazda_2017")
  for button, sig in ((Buttons.SET_PLUS, "SET_P"), (Buttons.SET_MINUS, "SET_M")):
    addr, data, bus = mazdacan.create_button_cmd(packer, cp(), 6, button)
    assert addr == 0x09D and bus == 0
    parser = CANParser("mazda_2017", [("CRZ_BTNS", 0)], 0)
    parser.update([(1, [(addr, data, bus)])])
    v = parser.vl["CRZ_BTNS"]
    assert v[sig] == 1 and v[sig + "_INV"] == 0 and v["CTR"] == 7
    other = "SET_M" if sig == "SET_P" else "SET_P"
    assert v[other] == 0 and v[other + "_INV"] == 1 and v["CAN_OFF"] == 0 and v["RES"] == 0
    assert data[0] != 0x1                       # never mistaken for a cancel by mazda.h


def test_carstate_keeps_raw_buttons_lkas_block_and_radar_lead():
  params = cp()
  ci = CarInterface(params)
  packer = CANPacker("mazda_2017")
  states = []
  ci.update([(1, [])])                        # the parser subscribes on first read: one empty update first
  for tick, (p, m, r, c, block, lead) in enumerate(((0, 0, 0, 0, 0, 0), (1, 0, 0, 0, 1, 1), (0, 1, 0, 0, 0, 0),
                                                     (0, 0, 1, 0, 0, 1), (0, 0, 0, 1, 0, 0)), 1):
    frames = [packer.make_can_msg("CRZ_BTNS", 0, {"SET_P": p, "SET_M": m, "RES": r, "CAN_OFF": c, "CTR": tick}),
              packer.make_can_msg("STEER_RATE", 0, {"LKAS_BLOCK": block}),
              packer.make_can_msg("CRZ_CTRL", 0, {"RADAR_HAS_LEAD": lead})]
    ci.update([(tick * 10_000_000, frames)])
    cs = ci.CS
    states.append((cs.crz_set_plus, cs.crz_set_minus, cs.crz_resume, cs.crz_cancel, cs.lkas_blocked, cs.radar_has_lead,
                   cs.crz_btns_counter))
  assert states == [(False, False, False, False, False, False, 1), (True, False, False, False, True, True, 2),
                    (False, True, False, False, False, False, 3), (False, False, True, False, False, True, 4),
                    (False, False, False, True, False, False, 5)]
