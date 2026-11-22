# Mazda speed control (buttons) — design note (Stage 2 + parts g, h, t; Stage 3)

Gen1 CX-5 with stock MRCC (`pcmCruise`, no openpilot longitudinal). The only way to change speed is the cruise
buttons (CRZ_BTNS 0x09D: SET+, SET-, RES). Everything here is OFF by default. With the master switch off the car
behaves exactly like the Stage 1 branch (`s7-mazda-ti`); `tests/test_off_is_stage1.py` checks it.

## Modules (all Python, `openpilot/starpilot/car/mazda/speed/`)

| module | job | origin |
|---|---|---|
| `config.py` | the one JSON setting `MazdaSpeedControlConfig` (version 5; 1-4 still read): strict decode, bounds, defaults (all off) | new |
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
| `processes.py` | when the manager runs the two daemons (CarParamsPersistent before the first drive) | new |
| `view_age.py` | the one rule for how old a map view may be (alive / current / gone), for every reader | new |
| `status.py` | the two helpers' status files the settings rows read (curve-map build, last drive's GPS) | new |
| `stops.py` | (g) stop slowdown: stopped car (radard's lead), stop ahead (short model path), dash stop sign, RES restore | ported rules |
| `notices.py` | (h) card side of the notices: triggers, once-per-event + gap rules, the tmpfs notice file, camera fault | new (+ old MazdaCamNotice) |
| `notice_alert.py` | (h) selfdrived side: reads the notice file at <= 10 Hz, one low-priority Alert per new notice | new |
| `tag_tool.py` | (t) `list` / `pack` the Bookmark tags of the event log | new |
| `auto_resume.py` | (Stage 3) gas auto-resume after a brake cancel: the old fork's logic + the map view adapter | ported |

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
  side cancels the exit blinker. **Trust a blinker that is still on**: while his signal toward the exit is on (1.0 s grace for the lamp's
  flash gaps / a self-cancel), or it was on for this exit and the camera says the car is in the lane next to the road
  edge on that side, the reach profile slows as much as the ramp needs (no 12 mph bound). Once it is off without the
  ramp confirmed, the 12 mph bound applies again and the set is held where it is (no SET+ back up inside the exit
  area) until the exit is resolved. Unit numbers (72 set, 70 mph, 35 mph curve 250 m past the split): blinker on
  370 m -> split ~56 / curve ~38 mph; 320-270 m + edge lane ~57 / ~38; 370-300 m (or 320-270 m) without the edge
  lane ~60 / ~48: **he must brake**; a lane-change blink 150-60 m that stays on the highway dips ~10 mph; a blinker
  held 350 m to the split that stays on dips ~21 mph (the price of trusting it), then his speed comes back.
  **Without his blinker an exit is only slowed for from the split, mostly once the ramp is confirmed (about 2-3 s past
  it)**: keep the signal on to the split for a tight exit. Exit handover to the street's limit + offset on a trust-0 match, never
  more than 10 mph under the car until the street is matched; his gas / buttons / brake cancel it.
* **No GPS**: the view says "none"; the camera-only curve uses its more careful side force; an unconfirmed exit plan
  is dropped.

### What must be on in 7.0 (and what degrades without it)

* **Map speed limits and the map's preview limit** come from 7.0's map service (the packaged `mapd`, manager process
  `mapd_shadow`). It starts on a drive (maps/shadow_lifecycle.py `enabled` + `preflight`) when:
  1. **Show Speed Limits** is on (`ShowSpeedLimits`, default on);
  2. **Map Data** is one of the speed limit sources (`SLCPriority1..3`; the defaults are Vision, Map Data, Off);
  3. an **offline map is downloaded** in Maps settings (parked; `map_snapshot_operations` writes it and selects it:
     `/data/media/0/starpilot/maps/offline/current.json`).

  Check over ssh: `cat /data/params/d/ShowSpeedLimits; for k in 1 2 3; do cat /data/params/d/SLCPriority$k; echo; done`
  (a missing file is the default), `ls -l /data/media/0/starpilot/maps/offline/current.json`, and on a drive
  `ps aux | grep -c "[m]apd --shadow"`. The settings rows for speed limit / merge / exits say "Now: ..." with what to
  turn on when it will not run.
* Without the map service running (items 1 or 2 off): **speed limit follow** has no map limit (only a dash reading
  if 7.0 has one for this car; vision is not followed), so it does nothing; the **merge** uses the ramp's
  destination speed from our curve files (no preview); the **exit handover** uses the ramp's destination speed (no
  posted street limit). **Map curves, the exit plan and the merge's curve guard still work**: our curve files are built
  by our own parked daemon (`mazda_curve_mapd`) straight from the downloaded offline tiles of the selected region; they
  need the download (item 3), not a running map service. Without the download (item 3) nothing map-based works: only
  the camera-only curves remain.
* **Curve map build range** (config v6 `curve_range_mi`: 30 / 50 / 100 / 200 / 300 miles or 0 = Whole map, default
  100 = the old fixed range; shown next to "Curves from the map"): the builder only builds the map areas (2 x 2 degree
  folders) within this range of the last known position, nearest first; Whole map builds every downloaded area,
  nearest first. Each area takes about 2-3 minutes, built only while parked with power. The builder reads the range
  every pass (every minute while parked) and before each area, so a change applies after the area being built, with
  no reboot. The status says both counts (in range and all downloaded) so a short range is not mistaken for the whole
  map (the owner had 33 areas downloaded and saw "5 of 5 built").

## Part (g): stop slowdown (`stops`: off / test / on)

**What it can and cannot do.** The stock cruise brakes at most about 1 m/s^2 and never sets below 19 mph (30 km/h).
It cannot stop the car, and for a stop seen late it cannot even slow enough. This feature takes speed off EARLY and
warns ("Stopped car ahead" / "Stop ahead"); **the driver brakes**. Test mode computes, logs and warns, never presses.

* **Stopped car (G1)**: radard's lead (camera-based on this car; the stock radar's "has lead" bit only lowers the
  needed camera probability, it is never required) stopped (<= 2 m/s) or slower than half our speed, closing by
  >= 3 m/s, within 160 m, for 0.5 s without a break (a 20 m distance jump is another car). Target: the lead's speed
  (the floor for a stopped car) 15 m behind it with the planning decel after the cruise's lag; a stopped car within
  130 m: the floor at once (one hold).
* **Stop ahead (G2)**: the old fork's stop-light flag, computed here from modelV2 (old starpilot planner, CEM
  `stop_sign_and_light`): planned path `position.x[-1]` shorter than v x 8 s (x1.2 from 45 mph, capped 9 s), on with
  2.5 m / off with 4 m of hysteresis, not above 75 mph, not in a slow turn, not with a lead inside the path (G1's job).
  Confirmed after 1.0 s without a break -> the floor at the path's end (at once within 130 m). Above 45 mph with the
  stop >= 140 m away only 1 mph guard taps (from 0.5 s, one per 0.5 s, at most 3 mph) until it is nearer: a far
  model-only guess never drops the set more than 3 mph (the old fork's 53 -> 30 drop with nothing ahead).
  7.0's own stop-light detector (conditional_mode/stop.py) is not used: its host only runs with openpilot
  longitudinal (planner_host.py checks `CP.openpilotLongitudinalControl`), which this car does not have.
* **Stop sign (G3)**: carstate's `dashboard_stop_sign` (CAM_TRAFFIC_SIGNS STOP_SIGN == 9; its parser runs for Gen1
  and Gen1 + TI) seen within 2 s shortens G2's confirmation to 0.5 s. Never a trigger without a short model path.
  No map stop sign: neither our curve files nor 7.0's `mapdOut` have a stop node (not added).
* **Release / restore**: the lead moving away (>= 1.5 m/s, not closing, 0.5 s) or gone 2 s with the path long; the
  path long again for 1 s. The planner then raises toward the anchor as always. **His RES** with the set left low:
  the episode ends and the set comes back at once (hold, then taps) - but only step by step (a planner ceiling one
  step above the set: taps, no hold) while the model path is shorter than v x 4 s or a lead within 80 m is closing;
  no new episode for 3 s after his RES. His SET+ during a slowdown ends that episode (planner block + declined until
  the cause is gone); his gas held 1 s too; his SET (a new engage) / main off forget it. The target only goes down
  during an episode.
* **Planner ceiling**: `Contribution.ceiling` (dash units) caps the command; the stop guard taps and the RES restore
  use it (a ceiling is a wish like a target: the lowest command wins, his overrule blocks it).

## Part (h): on-screen notices (`notices` off / on, `notice_sound` off / on)

No capnp change. card (notices.py) writes `/dev/shm/mazda_speed_notice.json` atomically **only when a notice fires**:
`{v, sid, seq, kind, text1, text2, sound, until}` (never a position). selfdrived has one small, marked hook
(`selfdrived.py`: 2 lines in `__init__`, 2 lines in `update_alerts` after the departure advisories, the same style as
the blindspot / departure / Tesla extension alerts): `notice_alert.make()` is `None` unless a Mazda with the speed
control, notices and one feature Test / On (read once per drive); `poll()` stats the file at most every 0.1 s, reads
it only when its mtime / size changed, and turns a new (sid, seq) into one `Alert` (Priority.LOWER, small or mid,
alert type `mazdaSpeedNotice/notice`); any error -> no alert. It is never put up while another alert is on screen
(it waits for a free screen until its `until`, 3 s), and every stock alert of LOW or above replaces it.
Sound: only `AudibleAlert.prompt`, only for the warnings, only with `notice_sound` on.

| notice | when | sound |
|---|---|---|
| Curve ahead — slowing to NN | a map / camera curve starts to lower OUR set (once per curve; NN = its tightest point's set) | no |
| Exit ramp — slowing | the exit plan starts to lower the set (once per exit) | no |
| Exit ramp — brake for the curve / Cruise cannot slow enough | the 12 mph bound keeps the set >= 3 mph above what the ramp needs | yes |
| Stop ahead / Cruise may not stop · brake | G2 confirmed (also in `stops` Test) | yes |
| Stopped car ahead / Cruise may not stop · brake | G1 confirmed (also in `stops` Test) | yes |
| Speed limit NN — set to NN | the limit rule changes OUR anchor (Speed limit unknown — set to 43: the fallback) | no |
| Merging — set speed high | the merge boost starts (once per ramp) | no |
| Mazda speed control: no GPS position | 15 s moving without a position (once per drive; map features only) | no |
| Curve maps not built here | 10 s moving with a position but no curve files here (once per drive) | no |
| Car's camera reports a fault / Lane keep may be off · steering weaker | TI cars: CAM_LKAS ERR_BIT_1/2 (the old MazdaCamNotice, again at most every 60 s) | yes |
| Car's lane keep is off / Steering is weaker in curves until it clears | TI cars: LANE_LINES 0 for 5 s above 35 mph (latched) | no |
| Car's camera fault cleared | 2 s without either fault | no |
| Tagged | a Bookmark tag was written (part t) | no |

At least 3 s between two notices (1 s before a warning); a notice inside the gap waits in one slot (a warning wins
it) for at most 2 s. Notices need the master switch and at least one feature Test / On.

## Part (t): the Bookmark tag

The speed control's private SubMaster also reads `userBookmark` (only while the speed control runs). Each new one
writes a `tag` line into the event log: wall time, `CurrentRoute` (the route name, no position), seconds since the
drive started (+ an approximate segment), vEgo, set, anchor, owner, the plan (and its test-mode side), each
contributor's state and proposal, the view summary (GPS state, trust, road class, link, ramp, curve / exit counts,
whether a road name is present - not the name), the lead, and our button presses of the last 10 s. With notices on,
"Tagged" shows.

**Put the Bookmark button on the onroad screen (7.0, comma 3X / tizi, e.g. the Konik A1M):** while driving (a drive
started, no alert on screen), tap the small round **Quick Select** button in the bottom-left corner of the driving
screen (or swipe up-right from that corner); the Quick Select shelf opens with three slots. Tap an empty slot, then
choose **Bookmark** in the picker (8 per page: Next / Previous). From then on one tap on that slot saves a bookmark
(7.0 shows "Bookmark Saved") and writes our tag. (7.0's small corner bookmark icon is the comma four / compact
screen's only; on tizi Quick Select is the way. Galaxy lists Bookmark but cannot press it.)

Tools (over ssh, parked):
`python -m openpilot.starpilot.car.mazda.speed.tag_tool list` lists the tags with times;
`... tag_tool pack` says what it would pack (dry run), `... tag_tool pack --write [--out FILE] [--window 60]`
writes the log lines within 60 s of each tag into one file (default `/data/media/0/mazda_speed/tags-<time>.jsonl`;
position-like keys are dropped even though the log has none).

## Maintenance

* **Reset the learned car profile** (button latency, hold overrun): parked, `rm /data/media/0/mazda_car_profile.json`;
  it starts fresh at the next drive (car_profile.py also ignores it by itself when the fingerprint or the TI flag
  changes). No settings row.

## Data flow (no capnp changes)

```
 parked:   7.0 offline tiles (selected generation)  --curve_mapd-->  /data/media/0/mazda_curve_map/<lat>/<lon>/*.json
 onroad:   gpsLocation | gpsLocationExternal + curve files --mazda_mapviewd (map_view.py, 10 Hz)--> /dev/shm/mazda_speed_view.json
 card:     CarState + CRZ_BTNS + modelV2 + slcState + radarState + userBookmark + the view file --caller.py--> planner -> actuator -> CRZ_BTNS
 card:     notices.py --(only when a notice fires)--> /dev/shm/mazda_speed_notice.json --notice_alert.py (selfdrived)--> Alert
```

* **GPS**: 7.0 runs one GPS daemon per device: `qcomgpsd` publishes `gpsLocation` (comma 3X / tizi, e.g. the Konik
  A1M), `ubloxd` publishes `gpsLocationExternal`. mazda_mapviewd reads both and uses the newest valid fix (7.0's rule
  in starpilot/gps/source.py, without its 25 m accuracy cut: GpsTrust pauses on a bad accuracy itself). The first Stage 2
  drives read only `gpsLocationExternal`, so on the A1M every map feature had "no GPS" (only the camera curve worked).
  The parked builder's nearest-first order uses `LastGPSPosition` (either service) or, on a tizi where only ubloxd would
  write that param, 7.0's navigation last-position store.
* **Status the owner can see** (no Params key; `status.py`, files in `/data/media/0/mazda_speed/`, never a position):
  `curve_build_status.json` (mazda_curve_mapd: "curve maps: within 100 miles 5 of 5 built; all downloaded 5 of 33.",
  "building… within 100 miles 3 of 5 built; …", with Whole map "12 of 33 map areas built", "no offline map
  downloaded"; an older file without the range counts still reads) and `gps_status.json` (mazda_mapviewd: "last drive: no GPS position: no message from the
  GPS" / "the GPS has no fix" / "the GPS messages stopped"). The settings rows for map curves, speed limit, merge and
  exits add them after "Now:". The event log gets a `gps` line whenever the view's GPS state changes (ok + service,
  or why: no_messages / no_fix / stale / no_view = the view file is missing).
* **Which car before the first drive**: the manager's `carParams` is empty (brand "") until card runs, so parked on
  power after a reboot the builder did not start. processes.py then uses CarParamsPersistent (7.0's checked cache),
  re-read at most every 30 s and decoded only when its bytes change.
* The view file is a small JSON (curve profile points as distance-ahead + radius at a monotonic timestamp, road
  class, ramp flags, GPS trust state). It is written atomically (tmp + rename) on tmpfs. card reads it at 10 Hz and
  moves the points by the distance driven since the timestamp.
* **When the view counts (view_age.py, one rule for every reader).** The A1M's GPS (qcomgpsd) gives 1 fix per second;
  ubloxd up to 10. The view's time is the fix minus GPS_LAG (0.85 s), so the age is counted from the GPS message
  (`now - (t + GPS_LAG)`). **Alive** up to 2.5 s: one missed 1 Hz fix is fine, a stopped mazda_mapviewd or a lost GPS
  is "no GPS" within about 2.5-3.4 s (the old fork: 3 s; mazda_mapviewd also writes "none" after 2.5 s without a fix).
  **Current** for one GPS period after the message (the period seen + 0.25 s, at most 1.5 s). States hold while the
  view is alive (no_gps / gps_ok, the road, the trust, the exit plan, the merge); places move by the distance driven.
  Evidence timers (exit link / match / highway, the limit's steady time, the merge's steady times, the exit handover's
  street) grow only on current frames and hold (no growth, no restart) between current and alive; every map
  confirmation or release of an exit also needs 2 different views. Older than alive: exactly the old "no GPS".
  (Before this rule a view older than 1.0 s from its stamp, 0.5 s for the auto-resume, was "no map": with the 1 Hz
  GPS the view was usable only ~10% of the time, exits never confirmed, the auto-resume never ran and the 43 mph
  fallback fired with a good fix.)
* card subscribes to modelV2 / slcState / radarState through a private SubMaster created only when an option that
  needs it is on (card's own subscription list is unchanged). The view file is version 2 (part f: ramp, exits).
* The button frame is appended after `CI.apply()` and only when that frame has no CRZ_BTNS of its own
  (openpilot's CANCEL/RESUME always win). mazda.h lets CRZ_BTNS through while controls are allowed (cruise engaged);
  with the cruise off only CANCEL, and (Stage 3, bit 256 only) RES inside the panda's auto-resume window.

## Hook points in 7.0

* `selfdrive/car/card.py`: one block next to the Subaru software-cruise hook (`brand == "mazda"`).
* `selfdrive/selfdrived/selfdrived.py` (part h): the notice hook, 2 lines in `__init__` + 2 lines in `update_alerts`.
* `opendbc/car/mazda/carstate.py`: stores the raw CRZ_BTNS buttons (SET_P/SET_M/RES/CAN_OFF), LKAS_BLOCK and the
  radar lead bit as attributes (the CarState message is unchanged). `mazdacan.create_button_cmd` learns SET+/SET-.
* `system/manager/process_config.py`: `mazda_curve_mapd` (offroad) and `mazda_mapviewd` (onroad), both only on a
  Mazda (the manager's CarParams, else CarParamsPersistent) with the master switch on and map curves, speed limit, merge or exits Test or On.
* Settings: `common/params_keys.h` (+ rebuilt libparams), `state_migration.VEHICLE_PREFERENCE_KEYS`,
  `ui/mazda_speed_feature.py` wired into FeatureSettingsOwner like `mazda_ti_feature.py`, Galaxy description.

## Ported / rewritten / dropped

* Ported: car_profile, GpsTrust, road matcher, curve store, curves-ahead, split_doublets, one-way rule, ramp
  records, the builder's chain joining, curve_mapd (+ the four review fixes: version bump, fsync + broken-file
  rebuild, MemoryError back-off, in-folder squares only). Old-format curve files are ignored until rebuilt.
* Rewritten: the actuator (one class, not the old 4,700-line controller), the planner, curve geometry (spline +
  distance profiles), the camera's role (veto + camera-only target only).
* Dropped on purpose: curve_learner, model_follow, in-curve camera taps/keep, post-apex taps, RampTail,
  CURVE_OWN_RAISE, MERGE_MODEL_OPEN, lead-in-curve follow. (The auto resume, the steering 700 test and the panda
  changes came in Stage 3, below.)

## Open questions (need the car)

See the Stage 2 report: button cadence/phase lock on 7.0's card timing, CRZ_SPEED step in mph, LKAS_BLOCK
meaning with the TI, view-file latency, the planning decel/lag numbers, and the lateral limits after 7.0's new
lateral controller.

## Stage 3: resume after brake, steering limit test, one steering check (all off by default)

Three items, each OFF by default. With all of them off the safety param, the CarParams, the parsers and every frame
are the Stage 2 ones (`tests/test_stage3.py::test_off_the_safety_param_and_the_limits_are_stage2`,
`tests/test_off_is_stage1.py`). Both settings are in `MazdaSpeedControlConfig` (version 5; no new Params key). They set
panda safety param bits **when the car starts** (`starpilot/vehicle_preferences.py` -> `opendbc/car/mazda/speed_flags.py`,
right after the Torque Interceptor's bit), so a change applies at the **next drive** (the next car start).

| bit | C (`mazda.h`) | Python (`values.py`) | set when |
|---|---|---|---|
| 8 | `MAZDA_PARAM_TORQUE_INTERCEPTOR` | `MazdaSafetyFlags.TORQUE_INTERCEPTOR` | TorqueInterceptorEnabled (Stage 1) |
| 256 | `MAZDA_PARAM_AUTO_RESUME` | `MazdaSafetyFlags.AUTO_RESUME` | master switch + `auto_resume` |
| 512 | `MAZDA_PARAM_TI_STEER_700` | `MazdaSafetyFlags.TI_STEER_700` | `ti_steer_700`, only with bit 8 |

The bits are only in the safety param, never in CP.flags (`opendbc/safety/tests/test_mazda.py::test_safety_flags_match_mazda_h`).

### A. Resume after brake (gas) (`auto_resume`, under the master switch)

When his brake drops the stock cruise to standby, his gas press makes us press RES, so the cruise comes back at his
set speed. **Panda window** (mazda.h, only with bit 256): while controls are not allowed, a CRZ_BTNS frame with byte 0
== 0x04 (RES alone) passes only if
* the panda saw the cruise's falling edge (CRZ_CTRL CRZ_ACTIVE 1 -> 0) with the brake pressed (PEDALS 0x10) and the
  main switch on (CRZ_CTRL bit 17) in that same frame (this opens the window, with a fresh budget and timer);
* now: the main switch on, the brake released, the gas pressed (ENGINE_DATA), the RX checks valid;
* fewer than 8 frames so far in this window; at most 180 s of moving time since the edge (standing still not counted).
The window closes when the cruise comes on, the main switch goes off, 180 s of moving time pass, or the safety mode
is set again. A new brake press never refills the 8 frames; only a new cruise-on -> brake-cancel does. SET+ / SET-
never pass. **Car side** (`auto_resume.py`, the old fork's rules): armed by his brake cancel (cruise on >= 2 s, no
cancel of ours / openpilot's / his within 0.5 s, a set speed), his gas held 0.3 s, then RES only on a public road on
the map view (class 1-12, GPS trust 0, the road unchanged 1 s and in 2 views in a row; the view alive, view_age.py), the wheel calm (0.5 s; 2 s after > 90 deg), under 10
mph only behind a moving car, the car's ACC_SET_ALLOWED, openpilot engageable, nothing else pressing. 4 frames right
after new wheel frames (CTR = wheel + 1), >= 0.1 s apart; the cruise must be on 0.6 s later; one retry; 8 frames, 170 s
moving or 800 m at most; openpilot's cancel after it: given up until the cruise comes on by other means.
The RES goes out through the one actuator (its only press with the cruise off) and is booked as his RES: the planner
keeps his anchor ("resumed"), the actuator waits DRIVER_QUIET, the stop slowdown treats it as his RES (restore,
RES quiet), the exit plan lets go. It needs the map view (an offline map, curve maps built, GPS), like the map features.
Event log: `auto_resume` lines (armed / sent / refused + why) and `auto_resume_state` on each state / wait change.

**Why "not always works" on the old fork (from reading the rules; no logs here):** most misses would be the car side
holding a press until the gas has been up 3 s (GAS_RECENT): after a stop at a junction the wheel is turned (wait for
calm, 2 s after a big turn), the road changes (1 s), the GPS trust must be 0 (it is 1 "shaky" early in a drive and
after a GPS pause, 2 "low" next to a parallel road or with a poor fix), ACC_SET_ALLOWED stays 0 up to ~10 s after a
stop, and under 10 mph a moving car ahead is needed. If
he lifts the gas during those waits the press is dropped and a new gas press is needed. A brake tap that drops the
cruise without PEDALS BRAKE_ON in the CRZ_CTRL frame of the fall (a different switch) would open no window at all (the
panda and the car side both need it). Nothing of this was changed (no logs to justify it); the new
`auto_resume_state` lines show the rule that held a press. Checked and fine: both sides read the same brake / gas bits,
the frame is phase-locked to the wheel's CRZ_BTNS counter, the car stays inside the panda's budget and time.

### B. Steering limit test (700) (`ti_steer_700`, TI cars only, not under the master switch)

With the TI and bit 512 the panda uses MAZDA_TI_STEERING_LIMITS_700 (max 700; rates 10 / 25, rt delta 300, driver
allowance 15 as with 600) for CAM_LKAS and CAM_LKAS2, and carcontroller's STEER_MAX is 700. latAccelFactor x 700/600
and friction x 600/700, so the torque for a lateral acceleration below the old ceiling is unchanged: only the ceiling
moves. A test: watch for steering faults.

### C. One steering check for the TI command

Stage 1 checked CAM_LKAS2 as a second torque command, which moved the one shared rate limiter twice per frame (the two
messages taking turns could ramp twice as fast; the rt delta still capped it). Now CAM_LKAS2 must repeat the last
CAM_LKAS value the checks accepted (carcontroller sends CAM_LKAS first, the same value), 0 without controls, and stay
within max_torque; it no longer touches the limiter.

### Needs the car

The panda window on the bus (the fall comes with BRAKE_ON in the same CRZ_CTRL frame; the 16 logged brake cancels of
the old fork say yes), RES accepted from standby at 7.0's card timing, openpilot engaging on our RES with the gas down,
the 700 limit without TI / EPS faults.
