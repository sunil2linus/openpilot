# Curve map

Finds the curves on the roads in FrogPilot's offline OpenStreetMap tiles and writes a small curve file per
0.25° square. Each curve has its tightest radius, both ends with the direction of travel, the apex, the road's
speed limit (or a default for the road type), any curve-sign advisory speed in OSM, and the speed it needs at
2.5 / 3.0 / 3.5 m/s² of cornering. Curves are found along a road; a turn from one road into another at an
intersection is never a curve. Residential streets are left out.

Checked against curves the car measured: the map radius was within about 10 % (117 m vs 104–116 m measured,
131 m vs 138–164 m, 216 m vs 231 m). A 2° x 2° area (about 140 x 100 miles) takes about 10 s and gives about
14,000 curves, 4 MB.

## Run it on a Windows laptop

1. Install Python 3.12 from https://www.python.org/downloads/ (tick "Add python.exe to PATH").
2. In PowerShell, install the two libraries:
   ```
   py -m pip install pycapnp numpy
   ```
3. Download the script and the map schema into one folder:
   ```
   mkdir $HOME\curve_map; cd $HOME\curve_map
   $base = "https://raw.githubusercontent.com/sunil2linus/openpilot/frogpilot-dev"
   Invoke-WebRequest "$base/tools/curve_map/build_curve_map.py" -OutFile build_curve_map.py
   Invoke-WebRequest "$base/frogpilot/system/offline_maps.capnp" -OutFile offline_maps.capnp
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

Repeat 4–6 for each 2° folder you drive in. The device already has every US folder under
`/data/media/0/osm/offline` (from FrogPilot's map download), so no new download is needed.

## Output

`<out>/<lat0>_<lon0>_<lat1>_<lon1>.json`, by the 0.25° square of each curve's apex:

| field | meaning |
|---|---|
| `a`, `a_hdg` | entry point and heading (deg) when travelling a → b |
| `b`, `b_hdg` | entry point and heading when travelling b → a |
| `apex`, `r` | tightest point and its radius (m, over 60 m of road) |
| `len`, `turn`, `dir` | curve length (m), total turn (deg), L/R when travelling a → b |
| `speed`, `adv` | speed limit (m/s; road-type default if OSM has none), OSM advisory speed (m/s, 0 = none) |
| `cls`, `name`, `oneway`, `ways` | road class, name or ref, one-way, OSM way ids |
| `v` | speed the curve needs (m/s) at 2.5 / 3.0 / 3.5 m/s² |

Tests: `python -m pytest tools/curve_map/test_build_curve_map.py`.
