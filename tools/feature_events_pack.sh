#!/usr/bin/env bash
# Pack the per-event logs of the optional driving features (stop warnings, curve slowdown, stopped
# car slowdown) together with only the qcamera.ts files of the segments they refer to.
#
#   ssh comma 'bash /data/openpilot/tools/feature_events_pack.sh'   ->  /data/feature_events.tgz
#
# Each event folder in /data/media/0/feature_events/ has a summary.txt with the route name, the
# segment number and the seconds into the segment (filled in by the device at write time) plus the
# wall-clock time. For events whose segment could not be determined on the device, the segment is
# re-derived here from the modification times of the segment folders in /data/media/0/realdata*/.
# The qcamera.ts of each referenced segment (and the one before it, for the 15 s lead-in) is added.
# No rlog / fcamera is ever included.
set -euo pipefail

EVENTS_DIR="${EVENTS_DIR:-/data/media/0/feature_events}"
LOG_ROOTS="${LOG_ROOTS:-/data/media/0/realdata:/data/media/0/realdata_HD:/data/media/0/realdata_konik}"
OUT="${1:-/data/feature_events.tgz}"
STAGE="$(mktemp -d /tmp/feature_events_pack.XXXXXX)"
trap 'rm -rf "$STAGE"' EXIT

if [ ! -d "$EVENTS_DIR" ] || [ -z "$(ls -A "$EVENTS_DIR" 2>/dev/null)" ]; then
  echo "no events in $EVENTS_DIR"
  exit 0
fi

mkdir -p "$STAGE/feature_events" "$STAGE/qcamera"
cp -r "$EVENTS_DIR"/. "$STAGE/feature_events/"

python3 - "$STAGE" "$LOG_ROOTS" <<'EOF'
import os, re, shutil, sys

stage = sys.argv[1]
events = os.path.join(stage, "feature_events")
roots = sys.argv[2].split(":")

# segment folders: {(route, n): (path, start_time, end_time)}; start = folder mtime (files created at
# segment start), end = qlog mtime (written until the segment ends)
segs = {}
for root in roots:
  try:
    names = os.listdir(root)
  except OSError:
    continue
  for name in names:
    m = re.match(r"^(.+)--(\d+)$", name)
    if not m:
      continue
    path = os.path.join(root, name)
    try:
      start = os.stat(path).st_mtime
      end = max((os.stat(os.path.join(path, f)).st_mtime for f in ("qlog", "rlog", "qcamera.ts")
                 if os.path.exists(os.path.join(path, f))), default=start + 60)
    except OSError:
      continue
    segs[(m.group(1), int(m.group(2)))] = (path, start, end)

wanted = set()
for ev in sorted(os.listdir(events)):
  summ = os.path.join(events, ev, "summary.txt")
  if not os.path.isfile(summ):
    continue
  kv = {}
  with open(summ) as f:
    for line in f:
      if ":" in line:
        k, v = line.split(":", 1)
        kv[k.strip()] = v.strip()
  route, seg, t = kv.get("route", ""), kv.get("segment", ""), kv.get("time_unix", "")
  if not seg and route and t:
    # re-derive from the folder times
    try:
      t = float(t)
      cands = [(n, s, e) for (r, n), (p, s, e) in segs.items() if r == route and s - 5 <= t <= e + 5]
      if cands:
        n, s, e = min(cands, key=lambda c: abs(c[1] - t))
        seg = str(n)
        with open(summ, "a") as f:
          f.write(f"segment_from_mtime: {n}\nsegment_seconds_from_mtime: {t - s:.1f}\n")
    except ValueError:
      pass
  if route and seg:
    n = int(seg)
    wanted.add((route, n))
    wanted.add((route, n - 1))      # the 15 s lead-in may be in the previous minute
  else:
    print(f"  {ev}: no route/segment (time {kv.get('time', '')})")

for route, n in sorted(wanted):
  info = segs.get((route, n))
  if info is None:
    continue
  src = os.path.join(info[0], "qcamera.ts")
  if os.path.exists(src):
    dst = os.path.join(stage, "qcamera", f"{route}--{n}--qcamera.ts")
    shutil.copy2(src, dst)
    print(f"  qcamera {route}--{n} ({os.path.getsize(src) // 1024} KB)")
print(f"{len(os.listdir(events))} events, {len(os.listdir(os.path.join(stage, 'qcamera')))} qcamera files")
EOF

tar -C "$STAGE" -czf "$OUT" feature_events qcamera
echo "wrote $OUT ($(du -h "$OUT" | cut -f1))"
