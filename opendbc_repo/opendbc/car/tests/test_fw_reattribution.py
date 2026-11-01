"""FW responses obtained through another brand's byte-identical query must still fingerprint the car.

The data below is the carFw list logged on a 2018 Mazda CX-5 (route 0000002f, StarPilot ff8714609):
the Mazda queries timed out while the panda was stalled, Ford's identical requests got the Mazda
ECUs' versions, and the car fell to MOCK although every version is in the Mazda database.
"""
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import match_fw_to_car, reattribute_fw_versions

Ecu = CarParams.Ecu

TP_F188 = [b'>\x00', b'"\xf1\x88']

# (ecu, fwVersion, address, subAddress, responseAddress, request, brand, bus, logging, obdMultiplexing)
CX5_FORD_TAGGED_FW = [
  (Ecu.engine, b'PYFD-188K2-J\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x7e0, 0, 0x7e8, TP_F188, 'ford', 1, True, True),
  (Ecu.fwdRadar, b'K131-67XK2-A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x764, 0, 0x76c, TP_F188, 'ford', 1, True, True),
  (Ecu.eps, b'KJ01-3210X-G-00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x730, 0, 0x738, TP_F188, 'ford', 1, True, True),
  (Ecu.hud, b'KL2L-554K2-I\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x720, 0, 0x728, TP_F188, 'ford', 1, True, True),
  (Ecu.fwdCamera, b'B61L-67XK2-R\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x706, 0, 0x70e, TP_F188, 'ford', 1, True, True),
  (Ecu.abs, b'KL2K-437K2-A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x760, 0, 0x768, TP_F188, 'ford', 1, True, True),
  (Ecu.engine, b'PYFD-188K2-J\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x7e0, 0, 0x7e8, TP_F188, 'ford', 0, True, True),
  (Ecu.fwdRadar, b'K131-67XK2-A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x764, 0, 0x76c, TP_F188, 'ford', 0, False, True),
  (Ecu.fwdCamera, b'B61L-67XK2-R\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x706, 0, 0x70e, TP_F188, 'ford', 0, False, True),
  (Ecu.hud, b'KL2L-554K2-I\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x720, 0, 0x728, TP_F188, 'ford', 0, True, True),
  (Ecu.eps, b'KJ01-3210X-G-00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x730, 0, 0x738, TP_F188, 'ford', 0, False, True),
  (Ecu.abs, b'KL2K-437K2-A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 0x760, 0, 0x768, TP_F188, 'ford', 0, False, True),
  (Ecu.fwdCamera, b'l\x80\x90\x00r\x84$\x00\x00\x00``\x00\x00\x00\x00\x00', 0x706, 0, 0x70e, [b'>\x00', b'"\xde\x00'], 'ford', 0, True, True),
  (Ecu.hud, b'\xbe\xff\x90\x80\x00', 0x720, 0, 0x728, [b'>\x00', b'"\xde\x01'], 'ford', 0, True, True),
  (Ecu.hud, b'\x03', 0x720, 0, 0x728, [b'>\x00', b'"\xde\x02'], 'ford', 0, True, True),
  (Ecu.abs, b'\x00', 0x760, 0, 0x768, [b'>\x00', b'"\xde\x02'], 'ford', 0, True, True),
  (Ecu.transmission, b'\xf1\x10' + b'\x00' * 24, 0x7e1, 0, 0x7e9, [b'"\xf1\x10'], 'hyundai', 0, True, True),
]

VIN = "JM3KFBDM1J0338578"


def build_car_fw(rows):
  out = []
  for ecu, version, addr, sub_addr, resp_addr, request, brand, bus, logging, obd in rows:
    f = CarParams.CarFw()
    f.ecu = ecu
    f.fwVersion = version
    f.address = addr
    f.subAddress = sub_addr
    f.responseAddress = resp_addr
    f.request = request
    f.brand = brand
    f.bus = bus
    f.logging = logging
    f.obdMultiplexing = obd
    out.append(f)
  return out


def test_ford_tagged_mazda_responses_do_not_match_alone():
  car_fw = build_car_fw(CX5_FORD_TAGGED_FW)
  _, matches = match_fw_to_car(car_fw, VIN, log=False)
  assert matches == set()


def test_reattribution_recovers_mazda_cx5():
  car_fw = build_car_fw(CX5_FORD_TAGGED_FW)
  extra = reattribute_fw_versions(car_fw)

  mazda = [f for f in extra if f.brand == "mazda"]
  # Mazda's bus-0 [TESTER_PRESENT, F188] request (eps, abs, fwdRadar, fwdCamera) and its bus-1
  # engine request; Ford's bus-1 request is logging-only for Ford, but the bytes are Mazda's query.
  assert {(str(f.ecu), f.address, f.bus) for f in mazda} == {
    ("engine", 0x7e0, 1), ("eps", 0x730, 0), ("abs", 0x760, 0), ("fwdRadar", 0x764, 0), ("fwdCamera", 0x706, 0),
  }
  # Mazda does not query the engine with TESTER_PRESENT on bus 0 and does not know 0x720.
  assert not any(f.address == 0x720 for f in mazda)
  assert all(not f.logging for f in mazda)
  # Ford's logging-only DE00/DE01/DE02 data identifiers are never re-tagged.
  assert all(list(f.request) == TP_F188 for f in mazda)

  exact, matches = match_fw_to_car(car_fw + extra, VIN, log=False)
  assert exact
  assert matches == {"MAZDA_CX5"}


def test_reattribution_is_idempotent_and_skips_known_tags():
  car_fw = build_car_fw(CX5_FORD_TAGGED_FW)
  extra = reattribute_fw_versions(car_fw)
  again = reattribute_fw_versions(car_fw + extra)
  keys = {(f.brand, f.address, f.subAddress, bytes(f.fwVersion), f.bus) for f in car_fw + extra}
  assert all((f.brand, f.address, f.subAddress, bytes(f.fwVersion), f.bus) not in keys for f in again)


def test_reattribution_does_not_create_other_brand_matches():
  car_fw = build_car_fw(CX5_FORD_TAGGED_FW)
  extra = reattribute_fw_versions(car_fw)
  for brand in {f.brand for f in extra} - {"mazda"}:
    brand_fw = [f for f in car_fw + extra if f.brand == brand]
    _, matches = match_fw_to_car(brand_fw, VIN, log=False)
    assert matches == set(), (brand, matches)
