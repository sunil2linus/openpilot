# Read-only copy of mapd_repo/cereal/offline/offline.capnp (the 7.0 offline map tile schema, same file id) for
# the Mazda curve builder: without the Go annotations, highwayClass read as its UInt16 wire value.
@0xda3a0d9284ca402f;


struct Way {
  name @0 :Text;
  ref @1 :Text;
  maxSpeed @2 :Float64;
  minLat @3 :Float64;
  minLon @4 :Float64;
  maxLat @5 :Float64;
  maxLon @6 :Float64;
  nodes @7 :List(Coordinates);
  lanes @8 :UInt8;
  advisorySpeed @9 :Float64;
  hazard @10 :Text;
  oneWay @11 :Bool;
  maxSpeedForward @12 :Float64;
  maxSpeedBackward @13 :Float64;
  id @14 :Int64;
  highwayClass @15 :UInt16;   # HighwayClass enum in mapd_repo/cereal/offline/offline.capnp: same wire value
  maxSpeedConditional @16 :Text;
  maxSpeedForwardConditional @17 :Text;
  maxSpeedBackwardConditional @18 :Text;
}

struct Coordinates {
  latitude @0 :Float64;
  longitude @1 :Float64;
}

struct Offline {
  minLat @0 :Float64;
  minLon @1 :Float64;
  maxLat @2 :Float64;
  maxLon @3 :Float64;
  ways @4 :List(Way);
  overlap @5 :Float64;
}
