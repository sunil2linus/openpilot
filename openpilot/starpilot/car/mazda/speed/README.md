# Mazda speed control (buttons) — Stage 2 design note

Gen1 CX-5 with stock MRCC (`pcmCruise`, no openpilot longitudinal). The only way to change speed is the cruise
buttons (CRZ_BTNS 0x09D: SET+, SET-, RES). Everything here is OFF by default. With the master switch off the car
behaves exactly like the Stage 1 branch (`s7-mazda-ti`); `tests/test_off_is_stage1.py` checks it.

## Modules (all Python, `openpilot/starpilot/car/mazda/speed/`)

| module | job | origin |
|---|---|---|
| `config.py` | the one JSON setting `MazdaSpeedControlConfig` (version 3): strict decode, bounds, defaults (all off) | new |
| `cruise_model.py` | measured MRCC response, dash offset, the one braking envelope, undershoot, LKAS floor | new |
| `car_profile.py` | per-car learned button latency + hold overrun (same file as the old fork) | ported |
| `actuator.py` | ONE button actuator: taps, phase-locked holds, attribution, "his buttons win" | rewritten, old rules |
| `planner.py` | ONE arbitrator: `min(his anchor, contributors)`, anchor owner + proposal rules, test mode | new |
| `limits.py` | (f) speed limit follow: 7.0's slcState reading, steady time with GPS trust, the limit proposal, 43 mph fallback | ported rules |
| `merge.py` | (f) on-ramp merge: highway limit (or the ramp's destination speed), boost / taper / end, guards | ported rules |
| `exit_plan.py` | (f) exit plan: evidence, reach bound, firm window, release, bounds, exit handover; LaneView | ported + rewritten |
| `curve_plan.py` | map curve contributor: profile points -> envelope target, lateral limits, TI-only latch | new |
| `camera.py` | phantom veto of map curves; camera-only curve target | rewritten |
| `contributors.py` | builds the contributors for the drive; private modelV2 SubMaster + view reader | new |
| `curve_builder.py` + `offline_tiles.capnp` | map builder: spline, profiles, doublets, ramps, road shapes | ported, geometry rewritten |
| `curve_mapd.py` | parked daemon `mazda_curve_mapd`: builds curve files from 7.0's selected map generation | ported + 4 fixes |
| `map_match.py` | GpsTrust, RoadIndex/RoadMatcher, MapCurveStore | ported |
| `map_view.py` | onroad daemon `mazda_mapviewd`: curves ahead, the matched link's ramp, exits ahead -> view file for card | new (curves-ahead, shapes ported) |
| `curve_memory.py`, `curve_memory_tool.py` | per curve+direction memory (way id key, no positions), cap + offset on the tight points, list/clear tool | simplified port |
| `events.py` | JSONL feature-event log under `/data/media/0/mazda_speed/` | simplified |
| `caller.py` | the hook card.py calls; off = returns the frames list untouched | new |
| `processes.py` | when the manager runs the two daemons | new |

## Part (f): speed limit, merge, exits (each off / test / on; test computes and logs, never presses)

* **Where the limit comes from.** 7.0's Speed Limit Controller runs display-only on this car (no openpilot
  longitudinal) and plannerd publishes its arbitration as `slcState` (20 Hz). card reads it through the speed
  control's private SubMaster (created only when speed limit, merge or exits is on): `speedLimit` when the selected
  source is the map or the dash, else that source's row in `sourceReadings`; `upcomingSpeedLimit` as the preview.
  Vision readings are not followed. It needs **Show Speed Limits** on (the default) and **Map Data** among the speed
  limit sources (the offline maps provider running). Navigation is **not** used: 7.0's `navInstruction` (navd, 1 Hz,
  an online Mapbox route with NavigationEnabled) and `starpilotNavigation` are no qualified offline exit signal.
* **Anchor owner.** The planner knows who owns the anchor: engaged (his SET / RES), his (his SET+/SET-) or ours.
  Contributors only *propose* anchors (limit, fallback, merge, handover); the planner takes them by its rules
  (planner.py): ours in any band, his engaged speed only within the band of the limit that changed, his own buttons
  never. All pressing is still the one actuator.
* **Limit**: steady 3 s (lower) / 5 s (higher, first) with GPS trust 0 on every frame (restarts at a bad frame);
  limit + offset (default 3 mph), whole dash steps, no margin; nothing on links or while merge / exit are busy; a raise
  waits for a lower preview (mapdOut's previewSpeedLimit; slcState's upcoming value is never filled on this car)
  seen on the same road; engaging inside the band of a known limit adopts limit + offset; 43 mph fallback only with
  no GPS position and the limit unknown (3 s at >= 35 mph, within 120 s of engaging, up only).
* **Merge**: on a link to a highway with trust 0 (or the start-of-drive trust on that link with GPS error <= 15 m);
  the highway limit from the preview / reading, else the ramp's destination speed from the map; boost to target + 20
  (car max 90), taper to target + 5 within 10 mph, end at the target. Waits for: a curve ahead planning below
  v + 5 mph, a short model path, a slow / closing / unknown radar lead, LKAS blocked. His SET- with a curve ahead is a
  curve speed; without, it declines the merge for that link.
* **Exits**: the map view gives each exit ahead (link start distance, side, the ramp's profile, to-highway flag,
  destination speed, the match and the map shapes past the split). With his blinker toward the exit within 400 m the
  car is kept on a reach profile (0.7 m/s^2 planned, so the ramp's first tight point stays reachable at <= 0.9); at the
  split with his blinker (or the camera's edge lane and the model path toward the link together) the full ramp plan
  runs for at most 3 s without a "link" confirmation (the edge lane or the model alone only hold the speed); until confirmed the cruise's set never goes more than 12 mph under the speed where our slowing began (the
  reach profile taking over, else the split). Released at once on "highway" (shapes, trusted match, model path), his
  gas held 1.5 s, any button; a confirmation is overturned by the same highway evidence or his gas held (unless the
  car is matched on the ramp with trust 0), and our handover then gives his speed back. His blinker to the other
  side cancels the exit blinker. **That 12 mph bound wins over the reach profile: from highway speed a tight exit
  curve is NOT reached by the cruise alone** (the unit "exit 48" case: 72 set, 70 mph, his blinker 370 m out,
  35 mph curve 250 m past the split: ~61 mph at the split, ~52 mph at the curve; he must brake ~1.4 m/s^2 over the
  last ~180 m).
  **Without his blinker an exit is only slowed for from the split, mostly once the ramp is confirmed (about 2-3 s past
  it)**: signal early and brake for a tight exit. Exit handover to the street's limit + offset on a trust-0 match, never
  more than 10 mph under the car until the street is matched; his gas / buttons / brake cancel it.
* **No GPS**: the view says "none"; the camera-only curve uses its more careful side force; an unconfirmed exit plan
  is dropped.

Not in Stage 2 (absent, not half-wired): stop slowdown (g), on-screen notices (h: log only).

## Data flow (no capnp changes)

```
 parked:   7.0 offline tiles (selected generation)  --curve_mapd-->  /data/media/0/mazda_curve_map/<lat>/<lon>/*.json
 onroad:   gpsLocationExternal + curve files --mazda_mapviewd (map_view.py, 10 Hz)--> /dev/shm/mazda_speed_view.json
 card:     CarState + CRZ_BTNS + modelV2 + slcState + radarState + the view file --caller.py--> planner -> actuator -> CRZ_BTNS
```

* The view file is a small JSON (curve profile points as distance-ahead + radius at a monotonic timestamp, road
  class, ramp flags, GPS trust state). It is written atomically (tmp + rename) on tmpfs. card reads it at 10 Hz and
  moves the points by the distance driven since the timestamp; a view older than 0.5 s is "no map".
* card subscribes to modelV2 / slcState / radarState through a private SubMaster created only when an option that
  needs it is on (card's own subscription list is unchanged). The view file is version 2 (part f: ramp, exits).
* The button frame is appended after `CI.apply()` and only when that frame has no CRZ_BTNS of its own
  (openpilot's CANCEL/RESUME always win). Panda safety is unchanged: mazda.h already lets CRZ_BTNS through while
  controls are allowed (cruise engaged).

## Hook points in 7.0

* `selfdrive/car/card.py`: one block next to the Subaru software-cruise hook (`brand == "mazda"`).
* `opendbc/car/mazda/carstate.py`: stores the raw CRZ_BTNS buttons (SET_P/SET_M/RES/CAN_OFF), LKAS_BLOCK and the
  radar lead bit as attributes (the CarState message is unchanged). `mazdacan.create_button_cmd` learns SET+/SET-.
* `system/manager/process_config.py`: `mazda_curve_mapd` (offroad) and `mazda_mapviewd` (onroad), both only on a
  Mazda (the manager's CarParams) with the master switch on and map curves, speed limit, merge or exits Test or On.
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
