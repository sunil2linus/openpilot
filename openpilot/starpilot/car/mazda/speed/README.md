# Mazda speed control (buttons) — design note (Stage 2 + parts g, h, t; Stage 3)

Gen1 CX-5 with stock MRCC (`pcmCruise`, no openpilot longitudinal). The only way to change speed is the cruise
buttons (CRZ_BTNS 0x09D: SET+, SET-, RES). Everything here is OFF by default. With the master switch off the car
behaves exactly like the Stage 1 branch (`s7-mazda-ti`); `tests/test_off_is_stage1.py` checks it.

## Modules (all Python, `openpilot/starpilot/car/mazda/speed/`)

| module | job | origin |
|---|---|---|
| `config.py` | the one JSON setting `MazdaSpeedControlConfig` (version 7; 1-6 still read): strict decode, bounds, defaults (all off) | new |
| `cruise_model.py` | measured MRCC response, dash offset, the one braking envelope, undershoot, LKAS floor | new |
| `car_profile.py` | per-car learned button latency + hold overrun (same file as the old fork) | ported |
| `actuator.py` | ONE button actuator: taps, phase-locked holds, attribution, "his buttons win" | rewritten, old rules |
| `planner.py` | ONE arbitrator: `min(his anchor, contributors)`, anchor owner + proposal rules, test mode | new |
| `limits.py` | (f) speed limit follow: 7.0's slcState reading (our map's tagged limit as the fallback, MapLimit), steady time with GPS trust, the limit proposal, 43 mph fallback | ported rules |
| `merge.py` | (f) on-ramp merge: highway limit (or the ramp's destination speed), boost / taper / end, guards | ported rules |
| `exit_plan.py` | (f) exit plan: evidence, reach bound, firm window, release, bounds, exit handover; LaneView | ported + rewritten |
| `curve_plan.py` | map curve contributor: profile points -> envelope target, lateral limits, TI-only latch | new |
| `camera.py` | phantom veto of map curves; camera-only curve target | rewritten |
| `contributors.py` | builds the contributors for the drive; private modelV2 SubMaster + view reader | new |
| `curve_builder.py` + `offline_tiles.capnp` | map builder: spline, profiles, doublets, ramps, road shapes | ported, geometry rewritten |
| `curve_mapd.py` | parked daemon `mazda_curve_mapd`: builds curve files from 7.0's selected map generation | ported + 4 fixes |
| `map_diag.py` | over ssh: counts of the built curve maps (formats, tagged limits by class, link joints, named ways) | new |
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
| `street_cap.py` | (round 2) a highway's speed carried onto a street is capped (the old fork's "exit street") | ported rules |

## Part (f): speed limit, merge, exits (each off / test / on; test computes and logs, never presses)

* **Our map as the fallback limit (round 4, format 6).** The curve files carry each way's tagged maxspeed; where 7.0
  has no valid reading the limit rules use it (see "Round 4" below for the precedence).
* **Where the limit comes from.** 7.0's Speed Limit Controller runs display-only on this car (no openpilot
  longitudinal) and plannerd publishes its arbitration as `slcState` (20 Hz). card reads it through the speed
  control's private SubMaster (created only when speed limit, merge or exits is on): `speedLimit` when the selected
  source is the map or the dash, else that source's row in `sourceReadings`; `upcomingSpeedLimit` as the preview.
  Vision readings are not followed. It needs **Show Speed Limits** on (the default) and **Map Data** among the speed
  limit sources (the offline maps provider running). Navigation is **not** used: 7.0's `navInstruction` (navd, 1 Hz,
  an online Mapbox route with NavigationEnabled) and `starpilotNavigation` are no qualified offline exit signal.
  Known (drive 0000000c): slcState's map row is "valid" exactly when 7.0's mapdOut says matchedLimit with way
  selection "current" (every sample). mapd says "unknown" (ErrAmbiguousWay: competing roads) for many whole GPS fixes
  and "possible" for the first fix of a new match, so on some streets the row is valid only ~18% of the time (the 1 Hz
  fix makes each answer last 1 s). That is 7.0's own selection, not our reading (slcState comes at 20 Hz; the 0.5 s
  staleness never cut a reading). So the steady time **rides through those seconds**: it holds (neither grows nor
  restarts) through a gap of readings up to 2.5 s (two missed 1 Hz fixes, like view_age's LATE) and restarts only on
  a different value or a longer gap; mapdOut's own limit on a "possible" fix (read through the same private
  SubMaster) counts as the same reading when it equals the candidate, never as a new value on its own. GPS trust
  still counts in a gap (a bad-trust frame restarts it). Recorded drive: the longest steady run on the 2235-2295
  stretch went from 8 s to 17 s; the times each limit became steady did not change.
* **Anchor owner.** The planner knows who owns the anchor: engaged (his SET / RES), his (his SET+/SET-) or ours.
  Contributors only *propose* anchors (limit, fallback, merge, handover); the planner takes them by its rules
  (planner.py): ours in any band, his engaged speed within the band of the limit that changed. All pressing is still
  the one actuator.
  * **The press that engages is an engagement.** SET+, SET- or RES from standby engage the stock cruise. The actuator
    reports that press as "engaged" (another set speed) or "resumed" (the same set speed), also when the car shows
    the new set speed a few frames after the cruise came on (ENGAGE_SETTLE 0.5 s, his press within 1.5 s before and
    no new press since). "resumed" keeps the anchor (we restore from there); one of ours stays ours. An anchor his
    SET+/SET- had made his becomes engaged again **on a street** (logged "resumed (was his)"); on a highway, a link
    or a road of unknown class it **stays his** (logged "resumed (his kept: not a street)"): he SET+ to 75 on a 70
    highway, taps the brake, RES: 75 stays, and only a limit change after the RES re-takes it (the old fork's rule
    8). With no GPS position at all the RES is an engagement as before. The gas auto-resume's RES is the same
    "resumed". (Drive 0000000c t 1872: his RES from standby kept "his" from an earlier SET+, so that engagement had
    no limit proposals and no fallback.) His SET+ / SET- after the cruise is on are his, as before.
  * **The limit change rule** (the old fork's speed_limit_follow rules 1 and 9). His own speed (his SET+ / SET-, or
    his engaged speed out of the band) is re-taken only by a NEW limit: one that became steady after his last press
    or engagement (the limit `seq`). A higher one needs his speed in the band of the limit it replaces (his 50 on a
    45 road -> 55 + offset; his 62 there stays). A lower one (the limit went down) re-takes his speed in any band,
    only on a street (road class known, not a highway, not a link), after 5 s unchanged with GPS trust 0 (8 s for a
    drop of more than 20 mph). Fresh for 30 s after it became steady. Never while he overrules a slowdown (a reason
    he blocked), and a raise never while a slowdown is under way: it waits, and is gone after the 30 s. Once taken
    the speed is ours; his next SET+ / SET- is his again, until the next limit change. On a highway his own speed
    above a lower limit is never pressed down (80 on a 70 road, 78 on a new 55). "The limit it replaces" survives a
    gap in the readings (7.0's map provider says "ambiguous road" for whole stretches): the last steady limit, and
    the one his speed was at, are kept 90 s; the same limit back after a gap is no change. The one his speed was at
    replaces the known old limit only when no old limit is known or the change is itself a raise (his 38 on a 35
    road, then his own 57 on a 50 road, then 50 -> 40 is a drop from 50, not a raise from 35).
  * **Who is pressed down on a highway** (checked): a highway limit drop lowers an anchor that is ours, or an engaged
    one inside the old limit's band, after the lower limit's 3 s steady time, to the new posted limit; an anchor his
    SET+ / SET- made his is never lowered on a highway (only a raise in the old limit's band re-takes it).
* **Limit**: steady 3 s (lower) / 5 s (higher, first) with GPS trust 0 on every frame (restarts at a bad frame);
  For the limit rules only a motorway (road class 1) is a highway: a trunk road (class 3, e.g. a divided surface road
  with lights such as Hall Rd) is a street, so it gets the offset, match on engage and the lower-limit re-take. Exits
  and merges still treat trunk roads as highways (map_view). Off a street a LOWER limit needs 5 s (8 s for a drop of
  more than 20 mph) before it lowers even a speed of ours (a wrong reading under an overpass); on a street 3 s.
  the target is the limit + offset (default 3 mph) **on a street only** (the last known road class of the map view
  is known and not a highway nor a link; the owner's "street speed: limit + 3", the old build's Street Speed); on a
  highway, a link or a road whose class was never known it is the posted limit itself. Whole dash steps, no margin;
  nothing on links or while merge / exit are busy; a raise waits for a lower preview (mapdOut's previewSpeedLimit;
  slcState's upcoming value is never filled on this car) seen on the same road; engaging (SET or RES) inside the
  band of a known limit adopts limit + offset **only on a street** with GPS trust 0 (the old fork's rule 8;
  elsewhere his SET speed stays); 43 mph fallback only with no GPS position and the limit unknown (3 s at >= 35 mph,
  within 120 s of engaging, up only).
* **Merge**: on a link to a highway with trust 0 (or the start-of-drive trust on that link with GPS error <= 15 m);
  the highway limit from the preview / reading, else the ramp's destination speed from the map; the target is that
  posted limit (no offset: a highway's, so the limit follow carries on at the same speed); boost to target + 20
  (car max 90), taper to target + 5 within 10 mph, end at the target. Waits for: a curve ahead planning below
  v + 5 mph, a short model path, a slow / closing / unknown radar lead, LKAS blocked. His SET- with a curve ahead is a
  curve speed; without, it declines the merge for that link.
* **Exits**: the map view gives each exit ahead (link start distance, side, the ramp's profile, to-highway flag,
  destination speed, the match and the map shapes past the split). With his blinker toward the exit within 400 m the
  car is kept on a reach profile (0.7 m/s^2 planned, so the ramp's first tight point stays reachable at <= 0.9); at the
  split with his blinker (or the camera's edge lane and the model path toward the link together) the full ramp plan
  runs for at most 3 s without a "link" confirmation (the edge lane or the model alone only hold the speed); until confirmed the cruise's set never goes more than 12 mph under the speed where our slowing began (the
  reach profile taking over, else the split). Released at once on "highway" (shapes, trusted match, model path), his
  gas held 1.5 s, any button - his gas / button / brake only once the plan acts (it had a target, his exit-side blinker,
  the reach profile, confirmed, or the car past the split): before that the exit is only looked at, his press there is
  about his own speed and the exit stays planned, and his gas held then does not count (r3: his SET+ ~500 m before an
  exit released it for good; round 3: also within 400 m, r2 rt 1044 his SET+ at 227 m, r2 rt 987 his gas held across
  the 400 m line); a confirmation is overturned by the same highway evidence or his gas held (unless the
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
* Without the map service running (items 1 or 2 off): **speed limit follow** has only our curve files' tagged limits
  (round 4, the fallback; areas rebuilt in format 6) and a dash reading if 7.0 has one for this car (vision is not
  followed); the **merge** uses the ramp's
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
  (Round 2) Above 45 mph the flag also needs the model's planned end speed (velocity.x[-1]) at most 75 % of ours
  (80 % to stay on); a lead inside the path no longer blocks it when the model plans a stop (end speed <= 4.5 m/s:
  the car ahead at our speed stops too); no guard taps while the exit plan or a curve explains the short path; a
  guard within 5 s of the last goes on from it (never more than 3 mph in all). G1 needs only 0.5 confidence within
  60 m closing by 4 m/s or more, holds its window through a 0.5 s dropout, and takes a stopping car (braking at
  -1.5 m/s^2 or harder, 1 m/s slower than us, the model planning a stop). Replay (true = the car stops or goes under
  8 mph within 20 s): r2 2 true / 7 false -> 5 true / 4 false (3 of the 4 slowed to 11-30 mph); r3 1 / 6 -> 1 / 1;
  0000000c 2 / 0 -> 2 / 0.
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
(it waits for a free screen until its `until`, 3 s), and every stock alert of LOW or above replaces it. "Another alert
on screen" is one with a size: the silent override alerts (steerOverride, gasPressedOverride: no text, AlertSize.none)
never hold a notice back (r2 rt 992: a "Stop ahead" with sound waited 3 s behind his steering and was never shown).
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

The speed control's private SubMaster also reads `userBookmark` (only while the speed control runs: any feature Test
/ On, or the Tag button setting). Each new one writes a `tag` line into the event log: wall time, `CurrentRoute` (the route name, no position), seconds since the
drive started (+ an approximate segment), vEgo, set, anchor, owner, the plan (and its test-mode side), each
contributor's state and proposal, the view summary (GPS state, trust, road class, link, ramp, curve / exit counts,
whether a road name is present - not the name), the lead, and our button presses of the last 10 s. With notices on,
"Tagged" shows.

**The TAG button (Large screen: comma 3X / tizi, e.g. the Konik A1M):** parked, in the Mazda speed control rows turn
on **Tag button on the driving screen** (the master switch must be on; no other option is needed). While driving a
TAG pill (208 x 112 px) shows at the top left, directly under the set speed / speed limit group, left-aligned with it
(default layout: x 88, y 400; it sits 16 px under the lowest thing that group can draw - its card, the sign, the
speed limit confirmation and its Accept / Reject prompt - and slides right or lower when another enabled widget is in
the way, e.g. right of the CEM road gauge when the speed-limit sources row makes the group taller; mirrored to the
right with right-hand drive). One tap saves one bookmark through the same Bookmark action Quick Select runs (7.0
shows "Bookmark Saved"; the pill flashes SAVED) and writes our tag. It stays on screen during alerts and our
notices; it hides while Quick Select is open. With only the master switch and this setting on, the speed control runs
"tag only": it writes the tag lines and never presses a button (no car profile, no notices). The setting is
`tag_button` in `MazdaSpeedControlConfig` (version 7); the button is `ui/mazda_tag_button.py` (Quick Select's
Bookmark slot still works too).

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
* **Message fields are 7.0's** (`tests/test_cereal_fields.py` builds real capnp messages of every service read here -
  radarState, modelV2, slcState, mapdOut, selfdriveState, userBookmark, gpsLocation(External), carState, carControl,
  carParams, pandaStates - and runs our readers on them). A field missing from the schema is never silent: one
  `schema` line per field per drive in the event log (else one cloudlog warning). Until this check the radar lead was
  never read on the car: LeadData's "a lead is there" is `present` in 7.0 (`status` in older schemas, still read as a
  fallback), so G1, the "Stop ahead" lead block, the auto-resume creep guard, the merge's lead wait and the RES restore
  never saw a car.
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
keeps his anchor ("resumed"; an anchor his SET+/SET- made his is an engaged one again), the actuator waits DRIVER_QUIET, the stop slowdown treats it as his RES (restore,
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

## Round 2 (drives r2 / r3 of 2026-11-22)

### Where a link really leads (map_view.LinkShapes, read time: no curve map rebuild)

The builder joins link ways of the same class without a name into one chain, straight through a junction with a
street. At M-53 / 23 Mile Rd the exit ramp and the on-ramp across the street are ONE chain (4 ways, 1.1 km) whose
record says "motorway to motorway": no exit handover, the merge armed toward 70 on the exit, and his 70 stayed the
anchor on 23 Mile Rd. The map view now reads the road shapes already in the files: a joint between two of the link's
ways with a street shape POINT (class 5-12) within 1 m is a junction (round 3: within 3 m of the street's line was
also a bridge whose way splits right above a street). Before it the link leads to that street (the view's
ramp: `hwy` false, `spd` the street class's default speed, `street` its class; an exit ahead likewise, its profile ends
at the junction); past it the link comes from that street (`from_hwy` false) and its record holds again (the on-ramp
merges). Home area (folder 42/-84): 90 interchange, 23 entrance, 29 exit and 31 other links have such a junction
(round 2's 3 m line test: 119 / 25 / 33 / 33; the 29 interchanges no longer cut were taken for long motorway-to-motorway
ramps over a street, e.g. M-59 490860778, I-94 17963358, I-69 443254109). Known limit of this read-time rule: the shapes
are simplified per way (2 m), so a street node in a straight run can be gone (at link ends onto a street ~40 % have no
street point within 1 m); such a junction inside a chain is not seen and the record holds (M-53 -> 23 Mile Rd,
486581349, keeps its point and its cut). **Round 4 (format 6): the builder decides the junction by a shared node**
and stores it ("j"); this read-time rule is only used for areas not rebuilt yet (see "Round 4"). The link's ways are followed along their own shapes (round 3: round 2
looked only in the cells along the chord between the link's ends, which missed the middle ways of a wide loop: 60
home-area links had missing segments, 25 a whole way missing; 5 change: 4 long I-275 / I-675 interchanges and an M-14
exit now see their street junction). The builder itself is unchanged (no version bump, no rebuild).

### Turnarounds and slow trunk roads are no merge (map_view.LinkShapes.leads)

r2 rt 1085-1170: the merge armed (target 50, a boost toward 70) inside Hall Rd's Michigan-left turnaround (a
trunk_link from one carriageway onto the other). A link now leads "to a highway" (the merge; an exit without a
handover) only when it ends on a motorway, or on a trunk road whose map speed is 55 mph or more, without first
meeting a street, and it is no turnaround. A turnaround: the link starts and ends on roads of the same class whose
one-way shapes there run more than 120 deg apart (read from the shapes in the files). The link's own heading is no
test: Hall Rd's turnaround itself turns only ~45 deg (the rest is the turn onto the other carriageway at its end),
and ~90 real on-ramps in the home area start and end more than 120 deg apart. Round 3: a turnaround is also short,
at most 250 m (400 m when both ends are the same road by name): two freeways can run more than 120 deg apart where a
long interchange joins them (I-75 -> Fisher Fwy 739 m, Herb Gray Pkwy -> E.C. Row 396 m, M-14 <-> M-153 593 / 1057 m
were flagged). Home area: 488 interchange-type (round 2: 493) and 2592 street-to-street turnarounds found (3078 of
round 2's 3085 were 250 m or shorter; Hall Rd's 1214344938 is 80 m); 38 interchanges and 28 entrances end on a trunk
road under 55.

### The street cap (street_cap.py, kind "street")

A safety net that does not need the ramp marking (the old fork's "exit to a street, completed" + its street
speed): after a motorway, or a link entered from a highway (50 m on it), a street matched with GPS trust 0 for 2 s
(class 5-12, or a trunk road whose posted limit is known and below 55) opens a 60 s window. Its cap is the street's
limit + the offset (unchanged 2 s), or 43 mph with no limit (2 s), no cap for a limit above 55. Once per street, with
the car under 55 mph and the anchor above the cap, the anchor is proposed down, never more than 2.4 m/s under the
car's dash speed (the old pacing), until the planner has it. The planner takes "street" only down, from any owner;
the anchor is then ours ("Street - set to NN"). His SET+ / SET- is his speed again (not capped again on that street);
after his brake and RES it goes on (old rule). While it is open on a street the merge drops a pending raise, and a
limit raise takes our street anchor (or a handover's) only for a limit that became steady after it (the highway's
old limit kept through a gap of readings never raises it back), or for one that is being read now on this street
(round 3, `live`: a valid reading this frame equal to the steady limit, clean trust, a matched street, at most 55, read
3 s in all on this road chain; so the trunk road's 55 read again after a cap at his RES, or a street's 45 after the
43 "limit unknown" cap, takes the set to the limit + offset). Differences from the old fork: trunk roads count only
with a known limit under 55 (the old fork never treated a trunk as a street); a street must come within 1 km after
the highway / link (the old fork waited without a bound); the pacing is an anchor floor (the old fork pressed SET-
taps or one hold itself).

### After any RES the set is never far above the road (street_cap.py, auto_resume.py)

r3 rt 402: the auto-resume brought back 55 at 21 mph on a 25-35 mph street (he pressed SET- x8); r2 rt 1167: 46 back at
11 mph inside the turnaround link (the car surged 6 -> 28 mph in the turn); r2 rt 1044: 53 carried onto Hall Rd (45).
* **Ownership (one rule for his RES and ours):** a RES keeps the anchor as before ("resumed"); then, on a trusted
  street (GPS trust 0, matched, not a link; class 5-12 or a trunk road), the street cap proposes an anchor above what
  the street allows down, as an anchor of ours (kind "street", paced like the exit cap): to the limit + offset when the
  limit is known (read now, or last read on this road chain with at most 400 m driven since: a red light of any length
  keeps it; a limit above 55: no cap), else to min(his anchor, max(v, 25 mph) + 20 mph). His SET+ / SET- afterwards is his speed as always (not capped again until the next RES). On a
  motorway, a link or an unknown road nothing changes (his speed back, as before). It is the same mechanism as the
  exit cap (one contributor, one kind, the lower of the two).
* **No auto-resume on a link under 25 mph** (a ramp, a turnaround): the press waits while his gas is held and goes once
  the car is on a proper road (or faster than 25 mph); his own RES there is his.
* Difference from the request: the limit-unknown cap uses at least 25 mph as "the speed" (a RES from a stop at 10 mph
  would otherwise cap at 30, under every street limit here); so the cap is never under 45 mph.

### A map jog is no curve (map_view.jogs, read time: no rebuild)

r2 rt 668-686 on Schoenherr Rd: the set went 53 -> 32 for a "curve" of radius 97 m where the road is straight (the GPS
heading changed under 1 deg/s). The cause is in the built file: where a two-way way meets a one-way way of the same
road (the road splits into its carriageways) the map line jogs 8 m sideways, read as a 26 deg left and a 30.2 deg right
bend 4 m apart. The builder's split_doublets drops exactly such pairs, but only up to 30 deg per bend: this one missed by
0.2 deg. The map view now leaves out two opposite bends of one road within 20 m whose heading changes cancel (net
under 8 deg), each at most 35 deg, the road moved sideways at most 12 m, where a one-way way meets a two-way one;
never on a ramp. Home area (folder 42/-84): 166 of 16068 curves left out (1 %), no ramp curve, no curve under 60 m;
the West Utica Rd curves (r 117 / 222 / 227 / 229 m) are kept. The camera veto still looks ~2 s ahead: the model's
curvature under-reads beyond that (camera.py), so a longer veto would also veto real curves; not changed.

## Round 4: our map's speed limit as a fallback (curve map format 6)

**Why.** 7.0's map service says "unknown" for whole seconds where ways compete (ErrAmbiguousWay on divided roads). On
Hall Rd (a divided trunk road tagged 45 / 50) slcState's map row was valid on ~5-18 % of the samples over several
drives, so limit follow, match on engage, the lower-limit re-take and the "limit known" street caps hardly ever worked
there, and after a stop the RES cap used the "limit unknown" rule (45). Our own map view matched Hall Rd with trust 0.

**The files (curve_builder.py, FORMAT_VERSION 6 / BUILDER_VERSION 4).** Each road shape carries its way's TAGGED
maxspeed as `"l"`: N for N mph, -N for N km/h (the tag's own unit and whole number), `[forward, backward]` when a
two-way way's directions differ, no field when untagged. Never the class default. Conditional / variable limits are
ignored. Size: +3.2 % on a full build of the home area from the old-format tiles (16.52 -> 17.05 MB). Each ramp record
also carries `"j"` (the joints of its ways where a street shares the node: an at-grade junction) and its `to` / `from`
roads `"h"` (their travel heading at the link's own end node, one-way roads): map_view.LinkShapes then needs no shape
search (cuts, turnaround); the read-time 1 m / 3 m rules remain for format 5 files.

**The rebuild.** curve_mapd sees format 5 / builder 3 in each area's state and rebuilds every area (within the build
range, nearest first, ~2-3 min each, parked on power; the owner's 33 areas about 1-2 hours with "Whole map"; with the default 100 miles only the areas in range, the rest keep working as format 5). Until an
area is rebuilt its old files keep working on the car exactly as before (curves, ramps, exits, shapes; only our
fallback limit is missing there; map_match reads formats 5 and 6). The settings status says so while it runs:
"curve maps: building… within 100 miles 3 of 5 built, 2 older ones still in use until rebuilt; all downloaded 3 of 33
(parked, on power)." (whole map: "building… 12 of 33 map areas built, 21 older ones still in use until rebuilt").
Check over ssh afterwards: `python -m openpilot.starpilot.car.mazda.speed.map_diag` (counts and ids only).

**The view.** `"lim"` (m/s) + `"limu"` ("mph" / "kmh"): the matched way's tag for the direction of travel, only with
GPS trust 0 and a match; on a link only when the link itself is tagged.

**Precedence (limits.MapLimit, once per frame in contributors.Sources -> ctx["limit_in"]; SpeedLimit and StreetCap
read it):**

| 7.0 (slcState map / dash row) | our map | the reading used |
|---|---|---|
| valid | any | 7.0's |
| valid, a different value | - | 7.0's; our map is not used on that road chain for 15 s after (no flipping in 7.0's gaps); one `map_limit` "disagree" line per road chain |
| missing / unknown | tagged, trust 0, matched, view alive | ours, as a reading of the same kind: the same steady times, trust and view rules, GAP_HOLD, `live`, seq / re-take, match on engage |
| only "possible" (mapdOut) with a different value | tagged | none this frame (a gap: the steady time holds up to GAP_HOLD); the same value: one reading |
| missing | untagged / not trusted | none (as before; a class default is never a limit) |

* street_cap's "limit known" (the exit cap and the RES cap) uses the same reading: a red light on Hall Rd resumes to
  the limit + 3, not the unknown-limit 45.
* The lower-limit preview ahead stays 7.0's only (our map would need the next ways along the road: not cheap; left).
* The merge's highway limit and the exit handover's street limit still read 7.0 only (unchanged).
* Event log: `map_limit` lines: `start` (value, unit, road class) when our map starts supplying the limit or its
  value changes, `stop` after 10 s without it (why: 7.0 / no tag / no view), `disagree` (both values). No position.

**Recorded drives (open loop; the home-area files here are format 5, so the limits were joined by way id from the
OLD-format offline tiles: an older OSM snapshot; 91 % of the device files' ways found).** Hall Rd: a reading (7.0 or
ours) on r2 5 % -> 99 % of the Hall Rd frames, r3 4 % -> 69 % (the rest: the first minute's trust 1), route 0000000c
1930-1985 18 % -> 98 %. 7.0's valid values never disagreed with the tags. r2 rt 987 Hall Rd's 45 steady (was never) ->
our 53 to 48; r2 rt 1030 the RES after the red light keeps 48 (was the unknown-limit 45); 0000000c t 1872 the RES on
Hall Rd matches 45 -> 48 (was his 45 kept); M-53's exit cap onto 23 Mile Rd 70 -> 48 (was 43, the limit unknown); the
M-59 raise to 55 2 s earlier; no curve plan changed.

**Junctions by node (old-format tiles of the home area; the device's own tiles may differ).** Cuts stored vs the
read-time rule: interchange 100 / 84, entrance 25 / 23, exit 33 / 29, other links 37 / 31; "to a highway" interchange
280 / 296, entrance 593 / 595; turnarounds 3033 / 3034 (4 only stored, 5 only read-time). M-53 486581349 -> 23 Mile Rd
cut (both); Hall Rd 1214344938 turnaround (both). **Changed:** 490860778 (M-59), 17963358 (I-94), 443260504 and
443254109 (I-69), 8727544 (I-94) and 8729433 (Lodge) share a NODE with a street at a joint of their ways (a diamond's
off-ramp running straight across the street onto the on-ramp, like M-53 / 23 Mile Rd), so they are cut now; 8727340
and 8728850 (Southfield / Jeffries) are not. The two "entrances" 402416597 and 8727616 are short crossovers (23 m /
51 m) from one carriageway of a divided street to the other, where the real on-ramp starts: cut at that joint.
