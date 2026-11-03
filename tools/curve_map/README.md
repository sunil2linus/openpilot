# Curve map

Finds the curves on the roads in the offline OpenStreetMap tiles (FrogPilot's and StarPilot's map download) and writes a small curve file per
0.25° square. Each curve has its tightest radius, both ends with the direction of travel, the apex, the road's
speed limit (or a default for the road type), any curve-sign advisory speed in OSM, and the speed it needs at
2.5 / 3.0 / 3.5 m/s² of cornering. Curves are found along a road; a turn from one road into another at an
intersection is never a curve. Residential streets are left out.

Checked against curves the car measured: the map radius was within about 10 % (117 m vs 104–116 m measured,
131 m vs 138–164 m, 216 m vs 231 m). A 2° x 2° area (about 140 x 100 miles) takes about 10 s and gives about
14,000 curves, about 14 MB with the road shapes.

## Built on the device (no laptop needed)

With Map Curves set to Test or On, the device builds the curve files itself while the car is parked
(`starpilot/system/curve_mapd.py`, started by manager offroad, lowest priority): every map folder under
`/data/media/0/osm/offline` that has no curve files, newer map tiles than its curve files were built from, or files
from an older builder is built into `/data/media/0/curve_map/<lat>/<lon>/`. Files built on a laptop are kept as they
are. A new city: download its map in the StarPilot map settings, park, and wait. Progress and timings are in
`/data/media/0/curve_map/build_log.txt` (a metro folder takes a few minutes on the device). The car side picks the
new files up at the start of the next drive (and every 2 minutes while driving).

Which folders get built (the Mazda tiles in the vehicle settings):

* **Map Build Range** (30–300 mi): folders within that distance of where the car is parked, nearest first.
* **Map Build States**: a tick per state of the map selection. A ticked state is built in full, wherever the car is
  parked and whatever the range says, and kept up to date after map updates. For a trip, tick its states while parked
  at home (on Wi-Fi, after downloading them in the map settings), untick them after. The dialog shows what each state
  has ("3 of 9 areas built", "Map not downloaded") and the build's progress with a time estimate.
* Both only build folders of the regions selected in StarPilot's map settings. Folders from an older whole-US download
  that are outside the selection are counted in the log ("378 map folders (4.1 GB) are outside your selected maps")
  and left alone; nothing under `osm/` is ever deleted here. Where a state is comes from
  `starpilot/common/map_region_boxes.py` (a bounding box per US state; US, Canada and Mexico as countries).

## Run it on a Windows laptop

1. Install Python 3.12 from https://www.python.org/downloads/ (tick "Add python.exe to PATH").
2. In PowerShell, install the two libraries:
   ```
   py -m pip install pycapnp numpy
   ```
3. Download the script and the map schema into one folder:
   ```
   mkdir $HOME\curve_map; cd $HOME\curve_map
   $base = "https://raw.githubusercontent.com/sunil2linus/openpilot/starpilot-mazda"
   Invoke-WebRequest "$base/tools/curve_map/build_curve_map.py" -OutFile build_curve_map.py
   Invoke-WebRequest "$base/tools/curve_map/offline_maps.capnp" -OutFile offline_maps.capnp
   ```
4. Copy the map folder you need from the device. Folders are named by the even degree below the latitude and
   longitude (Shelby Township, 42.6 / -83.1, is in `42/-84`):
   ```
   scp -r comma@192.168.88.13:/data/media/0/osm/offline/42/-84 $HOME\curve_map\osm_42_-84
   ```
5. Build the curves:
   ```
   py build_curve_map.py --tiles osm_42_-84 --out curves_42_-84
   ```
6. Copy the result to the device (the car side reads `/data/media/0/curve_map/<lat>/<lon>/`):
   ```
   ssh comma@192.168.88.13 "mkdir -p /data/media/0/curve_map/42"
   scp -r curves_42_-84 comma@192.168.88.13:/data/media/0/curve_map/42/-84
   ```

Repeat 4–6 for each 2° folder you drive in.

## Another city

1. Find the city's coordinates: in Google Maps, right-click the city centre and click the numbers at the top of
   the menu (they are copied, e.g. `41.8781, -87.6298` for Chicago).
2. Ask the script which folders cover the area and print the commands (latitude, longitude, miles):
   ```
   py build_curve_map.py --which 41.8781 -87.6298 50
   ```
   A city near a 2° line needs 2–4 folders; each takes about 10–40 s to build.
3. Run the printed commands in PowerShell, from the folder with the script, with the car parked and the device on
   the same Wi-Fi (or hotspot). Add `--device <ip>` to the step-2 command if the device's address changed.

The device already has every US folder under `/data/media/0/osm/offline` (from the map download in FrogPilot or
StarPilot), so no new download is needed.

## Output

`<out>/<lat0>_<lon0>_<lat1>_<lon1>.json`, by the 0.25° square of each curve's apex:

| field | meaning |
|---|---|
| `a`, `a_hdg` | entry point and heading (deg) when travelling a → b |
| `b`, `b_hdg` | entry point and heading when travelling b → a |
| `apex`, `r`, `r_map` | tightest point; radius the car needs (m: the map radius over 60 m of road, widened for short sharp bends a car can cut within its lane) and the map radius |
| `len`, `turn`, `dir` | curve length (m), total turn (deg), L/R when travelling a → b |
| `speed`, `adv` | speed limit (m/s; road-type default if OSM has none), OSM advisory speed (m/s, 0 = none) |
| `cls`, `name`, `oneway`, `ways` | road class, name or ref, one-way, OSM way ids |
| `v` | speed the curve needs (m/s) at 2.5 / 3.0 / 3.5 m/s² |
| `wdir` | `[way id, 1/0]`: travelling a → b follows that way's node order (FrogPilot's map match `is_forward`) |
| `ramp` | on ramps only: `type` (exit / entrance / interchange / link / two-way), `travel`, the road it leads `to` and comes `from` (class, speed limit, name), `end_dist` from the curve's end to the end of the ramp, `len` |

Each file also has `ramps`: every ramp (by its start square) with its `type`, `start`, `end`, `len`, `to`, `from`
and `wdir`, also for ramps without a curve, so the car knows what road a ramp merges into.

Format 4 (this version) adds `roads`: the shape of every road the curves use (no residential streets) in that
square, `{"w": way id, "p": [lat, lon, dlat, dlon, ...]}` in micro-degrees (first point, then differences), in the
way's node order and simplified to 2 m. The car matches its GPS position to these when its map program does not
say which road it is on (StarPilot). Format 3 files still work on FrogPilot; StarPilot needs format 4. Files are
about twice as big (about 14 MB per 2° folder).

Tests: `python -m pytest tools/curve_map/test_build_curve_map.py`.
