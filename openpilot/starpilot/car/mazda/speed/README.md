# Mazda speed control (buttons) — Stage 2 design note

Gen1 CX-5 with stock MRCC (`pcmCruise`, no openpilot longitudinal). The only way to change speed is the cruise
buttons (CRZ_BTNS 0x09D: SET+, SET-, RES). Everything here is OFF by default. With the master switch off the car
behaves exactly like the Stage 1 branch (`s7-mazda-ti`); `tests/test_off_is_stage1.py` checks it.

## Modules (all Python, `openpilot/starpilot/car/mazda/speed/`)

| module | job | origin |
|---|---|---|
| `config.py` | the one JSON setting `MazdaSpeedControlConfig` (version 1): decode, bounds, defaults | new |
| `cruise_model.py` | measured MRCC response (gap -> accel/decel, lag) and the braking envelope | new, from the owner's measurements |
| `car_profile.py` | per-car learned button latency, hold overrun, coast decel (JSON file) | ported (old `car_profile.py`) |
| `actuator.py` | ONE button actuator: moves the set speed toward the planner's command | rewritten, old rules kept |
| `planner.py` | ONE target arbitrator: `min(driver/limit, curve, stop)` + modes | new (replaces the old 7-layer stack) |
| `curve_plan.py` | curve contributor: profile points ahead -> target (braking envelope, lateral limits, LKAS floor) | new |
| `camera.py` | modelV2 helpers: phantom veto of map curves, camera-only target for unmapped curves | rewritten (old `camera_curve.py`/veto) |
| `curve_builder.py` | map builder: chains, Catmull-Rom spline, curvature, RDP profiles, `split_doublets`, ramps, road shapes | ported + rewritten geometry |
| `curve_mapd.py` | parked-only daemon that builds curve files on the device | ported (old `starpilot/system/curve_mapd.py`) |
| `curve_store.py`, `road_match.py`, `gps_trust.py` | onroad: load curve files around the car, match the road, GPS trust | ported (old `map_curves.py` parts) |
| `map_view.py` | the onroad map daemon `mazda_mapviewd` + the small file protocol to card | new |
| `events.py` | JSONL feature-event log under `/data/media/0/mazda_speed/` | simplified port |
| `caller.py` | the hook card.py calls; reads CarState/sockets, runs planner + actuator, returns CAN frames | new |

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
  Mazda with the master switch and map curves on.
* Settings: `common/params_keys.h` (+ rebuilt libparams), `state_migration.VEHICLE_PREFERENCE_KEYS`,
  `ui/mazda_speed_feature.py` wired into FeatureSettingsOwner like `mazda_ti_feature.py`, Galaxy description.

## Ported / rewritten / dropped

* Ported: car_profile, GpsTrust, road matcher, curve store, split_doublets, one-way rule, ramp records, the
  builder's chain joining, curve_mapd (+ the four review fixes: version bump, fsync, MemoryError back-off,
  in-folder squares only).
* Rewritten: the actuator (one class, not the old 4,700-line controller), the planner, curve geometry (spline +
  distance profiles), the camera's role (veto + camera-only target only).
* Dropped on purpose: curve_learner, model_follow, in-curve camera taps/keep, post-apex taps, RampTail,
  CURVE_OWN_RAISE, MERGE_MODEL_OPEN, lead-in-curve follow, auto resume (Stage 3), steering 700 test (Stage 3),
  any panda change (Stage 3).

## Open questions (need the car)

See the Stage 2 report: button cadence/phase lock on 7.0's card timing, CRZ_SPEED step in mph, LKAS_BLOCK
meaning with the TI, view-file latency, the planning decel/lag numbers, and the lateral limits after 7.0's new
lateral controller.
