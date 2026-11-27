#!/usr/bin/env bash
# Build pandad on the comma device (AGNOS) for the Mazda TI Always On Lateral change, in a SEPARATE checkout.
#
#   bash /data/aolbuild/src/tools/mazda_ti_aol/pandad_device_build.sh build
#
# It never touches /data/openpilot (the running build) or the shared scons cache (/data/scons_cache), and refuses to
# run while the car is on (IsOnroad). It builds the pandad target twice in /data/aolbuild/src:
#   1. "base": the commit before the pandad source change (unchanged source), to compare with the committed binary;
#   2. "new":  the branch head (the new Mazda AOL profile).
# Each time every tracked file the build changed is saved under /data/aolbuild/out/<phase>/, with logs and
# sha256 lists, and everything is packed into /data/media/0/aolbuild.tar (to copy to the PC with scp).
# Check the tar with tools/mazda_ti_aol/check_pandad.py before committing anything.
set -euo pipefail

WORK=/data/aolbuild
SRC=$WORK/src
RUN=$WORK/run.sh
OUT=$WORK/out
REPORT=$WORK/report
TAR=/data/media/0/aolbuild.tar
TARGET=openpilot/selfdrive/pandad/pandad
PROFILE=openpilot/starpilot/car/mazda/aol_policy.h
JOBS=${JOBS:-4}

die() { echo "ERROR: $*" >&2; exit 1; }

if [[ "${1:-}" == "build" ]]; then
  # The base checkout removes this script from the tree: run a copy outside it.
  [[ -d "$SRC/.git" ]] || die "no checkout at $SRC"
  cp "$SRC/tools/mazda_ti_aol/pandad_device_build.sh" "$RUN"
  exec bash "$RUN" run
fi
[[ "${1:-}" == "run" ]] || die "usage: $0 build"

# ---- guards ----
[[ -f /AGNOS ]] || die "not an AGNOS device"
[[ "$(readlink -f "$SRC")" != "$(readlink -f /data/openpilot)" ]] || die "refusing to build in /data/openpilot"
if [[ -f /data/params/d/IsOnroad ]] && [[ "$(cat /data/params/d/IsOnroad 2>/dev/null)" == "1" ]]; then
  die "the car is on (IsOnroad): park and turn the car off first"
fi
avail_kb=$(df -Pk /data | awk 'NR==2{print $4}')
[[ "$avail_kb" -gt 4000000 ]] || die "less than 4 GB free on /data"

cd "$SRC"
NEW=$(git rev-parse HEAD)
CHANGE=$(git log -1 --format=%H -- "$PROFILE")
BASE=$(git rev-parse "$CHANGE^")
git cat-file -e "$BASE^{commit}" || die "base commit not fetched (clone with more depth)"
grep -q '1024U' "$PROFILE" || die "this checkout does not have the new Mazda AOL profile"

rm -rf "$OUT" "$REPORT"
mkdir -p "$OUT" "$REPORT" "$WORK/scons_cache"
export SCONS_CACHE=$WORK/scons_cache      # never the shared /data/scons_cache (its prune would delete files there)
# scons runs through the repo's own native AGNOS entry point (scripts/native_device_build.sh scons: the device venv's
# scons, PYTHONPATH bound to this checkout), as a device build of this tree is meant to run
[[ -x /usr/local/venv/bin/scons ]] || die "no /usr/local/venv/bin/scons"
VENV=/usr/local/venv
# Debug info paths as in the committed objects (built in the laptop container), so unchanged code can match
MAP="-fdebug-prefix-map=$SRC=/audit/nov20-turn-settings-private/build-source"
MAP+=" -fdebug-prefix-map=$VENV=/work/.venv-linux-arm64"
MAP+=" -fdebug-prefix-map=/usr/include=/opt/tici-sysroot/usr/include"
MAP+=" -fdebug-prefix-map=/usr/local/include=/opt/tici-sysroot/usr/local/include"

{
  echo "date $(date -u +%FT%TZ)"
  echo "agnos $(cat /VERSION 2>/dev/null)"
  echo "uname $(uname -srm)"
  echo "clang $(clang++ --version | head -1)"
  echo "scons $(/usr/local/venv/bin/scons --version 2>/dev/null | grep -m1 -i 'engine' || true)"
  echo "python $($VENV/bin/python3 --version 2>&1) prefix $VENV"
  $VENV/bin/python3 - <<'PY'
import importlib.metadata as m
for name in ("capnproto", "libusb", "json11", "zstd", "ncurses", "acados", "ffmpeg", "numpy", "scons"):
  try:
    print("pkg", name, m.version(name))
  except Exception:
    try:
      print("pkg", name, m.version("comma-deps-" + name))
    except Exception:
      print("pkg", name, "?")
PY
  echo "base $BASE"
  echo "new $NEW"
  echo "ccflags $MAP"
  echo "jobs $JOBS"
} > "$REPORT/env.txt"
cat "$REPORT/env.txt"

build_phase() {
  local phase=$1 commit=$2
  echo "==> $phase: $commit"
  git checkout -f -q --detach "$commit"
  local started=$(date +%s)
  if ! bash scripts/native_device_build.sh scons -j"$JOBS" --cache-disable --verbose --ccflags="$MAP" "$TARGET" \
       > "$REPORT/$phase.scons.log" 2>&1; then
    tail -40 "$REPORT/$phase.scons.log"
    die "$phase build failed (log: $REPORT/$phase.scons.log)"
  fi
  echo "    built in $(( $(date +%s) - started )) s"
  git status --porcelain --untracked-files=no > "$REPORT/$phase.status"
  : > "$REPORT/$phase.sha256"
  while read -r _ path; do
    mkdir -p "$OUT/$phase/$(dirname "$path")"
    cp -p "$path" "$OUT/$phase/$path"
    echo "$(sha256sum "$path" | cut -d' ' -f1) $(git show "$commit:$path" | sha256sum | cut -d' ' -f1) $path" >> "$REPORT/$phase.sha256"
  done < "$REPORT/$phase.status"
  # the pandad outputs are always saved, also when identical to the committed ones
  for f in pandad pandad.o main.o panda_safety.o libpanda.a panda.o panda_usb.o spi.o; do
    p=openpilot/selfdrive/pandad/$f
    mkdir -p "$OUT/$phase/openpilot/selfdrive/pandad"
    cp -p "$p" "$OUT/$phase/$p"
  done
  echo "    tracked files changed by the build:"
  awk '{ printf "      %s %s\n", ($1 == $2 ? "same   " : "CHANGED"), $3 }' "$REPORT/$phase.sha256"
}

build_phase base "$BASE"
build_phase new "$NEW"
git checkout -f -q --detach "$NEW"

rm -f "$TAR"
tar -C "$WORK" -cf "$TAR" out report
echo "==> packed $TAR ($(du -h "$TAR" | cut -f1))"
base_same=$(awk '$3 == "openpilot/selfdrive/pandad/pandad" { print ($1 == $2) ? "IDENTICAL" : "DIFFERENT" }' "$REPORT/base.sha256")
echo "==> unchanged source rebuilt vs committed pandad: ${base_same:-IDENTICAL (not changed by the build)}"
echo "==> DONE. Copy $TAR to the PC, then run the cleanup command."
