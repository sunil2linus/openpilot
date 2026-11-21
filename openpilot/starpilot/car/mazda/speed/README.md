# Mazda speed control (buttons) — Stage 2 design note

Gen1 CX-5 with stock MRCC (`pcmCruise`, no openpilot longitudinal). The only way to change speed is the cruise
buttons (CRZ_BTNS 0x09D: SET+, SET-, RES). Everything here is OFF by default. With the master switch off the car
behaves exactly like the Stage 1 branch (`s7-mazda-ti`); `tests/test_off_is_stage1.py` checks it.

## Modules (all Python, `openpilot/starpilot/car/mazda/speed/`)

| module | job | origin |
|---|---|---|
| `config.py` | the one JSON setting `MazdaSpeedControlConfig` (version 1): strict decode, bounds, defaults (all off) | new |
| `cruise_model.py` | measured MRCC response, dash offset, the one braking envelope, undershoot, LKAS floor | new |
| `car_profile.py` | per-car learned button latency + hold overrun (same file as the old fork) | ported |
| `actuator.py` | ONE button actuator: taps, phase-locked holds, attribution, "his buttons win" | rewritten, old rules |
| `planner.py` | ONE arbitrator: `min(his anchor, contributors)`, anchor rules, test mode | new |
| `curve_plan.py` | map curve contributor: profile points -> envelope target, lateral limits, TI-only latch | new |
| `camera.py` | phantom veto of map curves; camera-only curve target | rewritten |
| `contributors.py` | builds the contributors for the drive; private modelV2 SubMaster + view reader | new |
| `curve_builder.py` + `offline_tiles.capnp` | map builder: spline, profiles, doublets, ramps, road shapes | ported, geometry rewritten |
| `curve_mapd.py` | parked daemon `mazda_curve_mapd`: builds curve files from 7.0's selected map generation | ported + 4 fixes |
| `map_match.py` | GpsTrust, RoadIndex/RoadMatcher, MapCurveStore | ported |
| `map_view.py` | onroad daemon `mazda_mapviewd`: curves ahead -> view file for card | new (curves-ahead ported) |
| `curve_memory.py`, `curve_memory_tool.py` | per curve+direction memory (way id key, no positions), cap + offset on the tight points, list/clear tool | simplified port |
| `events.py` | JSONL feature-event log under `/data/media/0/mazda_speed/` | simplified |
| `caller.py` | the hook card.py calls; off = returns the frames list untouched | new |
| `processes.py` | when the manager runs the two daemons | new |

**Exit ramps are NOT pre-slowed:** a ramp's curves only count once the road match has moved onto the ramp
(typically 50-150 m after the split), which can mean a late, firm slowdown or none in time (the old fork's 35 mph
exit entered at 52 mph). The driver must slow for exits himself until the exit plan (part f) exists.

Not in Stage 2 (absent, not half-wired): speed limit
follow / merge / exit handover (f), stop slowdown (g), on-screen notices (h: log only), exit pre-plan.

## Data flow (no capnp changes)

```
 parked:   7.0 offline tiles (selected generation)  --curve_mapd-->  /data/media/0/mazda_curve_map/<lat>/<lon>/*.json
 onroad:   gpsLocationExternal + curve files --mazda_mapviewd (map_view.py, 10 Hz)--> /dev/shm/mazda_speed_view.json
 card:     CarState + CRZ_BTNS + modelV2 + slcState + the view file --caller.py--> planner -> actuator -> CRZ_BTNS frame
```

* The view file is a small JSON (curve profile points as distance-ahead + radius at a monotonic timestamp, road
  class, ramp flags, GPS trust state). It is written atomically (tmp + rename) on tmpfs. card reads it at 10 Hz and
  moves the points by the distance driven since the timestamp; a view older than 0.5 s is "no map".
* card subscribes to modelV2/slcState/gpsLocationExternal through a private SubMaster created only when the
  master switch is on (card's own subscription list is unchanged).
* The button frame is appended after `CI.apply()` and only when that frame has no CRZ_BTNS of its own
  (openpilot's CANCEL/RESUME always win). Panda safety is unchanged: mazda.h already lets CRZ_BTNS through while
  controls are allowed (cruise engaged).

## Hook points in 7.0

* `selfdrive/car/card.py`: one block next to the Subaru software-cruise hook (`brand == "mazda"`).
* `opendbc/car/mazda/carstate.py`: stores the raw CRZ_BTNS buttons (SET_P/SET_M/RES/CAN_OFF), LKAS_BLOCK and the
  radar lead bit as attributes (the CarState message is unchanged). `mazdacan.create_button_cmd` learns SET+/SET-.
* `system/manager/process_config.py`: `mazda_curve_mapd` (offroad) and `mazda_mapviewd` (onroad), both only on a
  Mazda (the manager's CarParams) with the master switch on and map curves Test or On.
* Settings: `common/params_keys.h` (+ rebuilt libparams), `state_migration.VEHICLE_PREFERENCE_KEYS`,
  `ui/mazda_speed_feature.py` wired into FeatureSettingsOwner like `mazda_ti_feature.py`, Galaxy description.

## Ported / rewritten / dropped

* Ported: car_profile, GpsTrust, road matcher, curve store, curves-ahead, split_doublets, one-way rule, ramp
  records, the builder's chain joining, curve_mapd (+ the four review fixes: version bump, fsync + broken-file
  rebuild, MemoryError back-off, in-folder squares only). Old-format curve files are ignored until rebuilt.
* Rewritten: the actuator (one class, not the old 4,700-line controller), the planner, curve geometry (spline +
  distance profiles), the camera's role (veto + camera-only target only).
* Dropped on purpose: curve_learner, model_follow, in-curve camera taps/keep, post-apex taps, RampTail,
  CURVE_OWN_RAISE, MERGE_MODEL_OPEN, lead-in-curve follow, auto resume (Stage 3), steering 700 test (Stage 3),
  any panda change (Stage 3).

## Open questions (need the car)

See the Stage 2 report: button cadence/phase lock on 7.0's card timing, CRZ_SPEED step in mph, LKAS_BLOCK
meaning with the TI, view-file latency, the planning decel/lag numbers, and the lateral limits after 7.0's new
lateral controller.
