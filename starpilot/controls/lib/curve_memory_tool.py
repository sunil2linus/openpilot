#!/usr/bin/env python3
"""Curve memory maintenance (round U, U8): list the curve memory's records by road name, and clear the speed caps and / or
the Curve Speed Offset's "no offset" marks of one road. Run on the device over ssh with the car off (the planner reads the
file when a drive starts and writes it after passes):

  python3 /data/openpilot/starpilot/controls/lib/curve_memory_tool.py                       # list every road
  python3 /data/openpilot/starpilot/controls/lib/curve_memory_tool.py --road "W Utica"      # one road (name contains it)
  python3 /data/openpilot/starpilot/controls/lib/curve_memory_tool.py --road "W Utica" --clear-caps --clear-marks
  ... the same with --apply to write it (a copy of the file is kept as curve_memory.json.bak first)

Without --apply nothing is written (a dry run: what would change is printed). No GPS position is ever printed.
"""
import argparse
import json
import os
import shutil
import sys
import time

MEMORY_PATH = "/data/media/0/curve_memory.json"
MPH = 0.44704
CAP_KEYS = ("cap", "cap_t", "cap_sat_t", "cap_v", "cap_hits", "cap_lift")
CAP_EXPIRE_DAYS = 30.0          # (map_curves.CAP_EXPIRE_DAYS)


def _days(t, now):
  try:
    return (now - float(t)) / 86400.0
  except (TypeError, ValueError):
    return None


def _passes(e):
  ps = [p for p in e.get("passes", []) if isinstance(p, list) and len(p) >= 2]
  op = [p for p in ps if len(p) >= 4 and "op" in (p[3] or []) and "alert" not in (p[3] or []) and p[2] is not None]
  fastest = max((float(p[2]) for p in op), default=0.0)
  return len(ps), len(op), fastest


def describe(e, now):
  """One line for record `e` (no position)."""
  parts = []
  d = e.get("dir")
  parts.append(f"dir {float(d):5.1f} deg" if d is not None else "dir (none)")
  if e.get("r_map"):
    parts.append(f"map R {float(e['r_map']):.0f} m")
  if e.get("ramp"):
    parts.append("ramp")
  n, n_op, fastest = _passes(e)
  parts.append(f"passes {n} (clean op {n_op}" + (f", fastest {fastest / MPH:.1f} mph)" if n_op else ")"))
  if e.get("cap"):
    age = _days(e.get("cap_sat_t", e.get("cap_t")), now)
    state = " EXPIRED" if (age is not None and age > CAP_EXPIRE_DAYS) else ""
    parts.append(f"cap {float(e['cap']) / MPH:.1f} mph" + (f" (saturated at {float(e['cap_v']) / MPH:.1f})" if e.get("cap_v") else
                                                           " (saturation speed unknown)") +
                 (f", {age:.0f} days old{state}" if age is not None else "") + f", hits {int(e.get('cap_hits', 0) or 0)}")
  m = e.get("no_offset")
  if isinstance(m, list) and len(m) >= 3:
    age = _days(m[0], now)
    parts.append(f"no offset: {m[2] or '?'}" + (f" ({age:.0f} days old)" if age is not None else ""))
  g = e.get("ghost")
  if isinstance(g, list) and g:
    parts.append(f"ghost marks {len(g)}")
  return ", ".join(parts)


def load(path):
  with open(path) as f:
    data = json.load(f)
  if not isinstance(data, dict) or not isinstance(data.get("curves"), list):
    raise ValueError("not a curve memory file")
  return data


def by_road(curves, road=None):
  """{name: [record, ...]} (case-insensitive 'contains' match on `road` when given)."""
  out = {}
  for e in curves:
    if not isinstance(e, dict):
      continue
    name = str(e.get("name", "") or "") or "(no name)"
    if road is not None and road.lower() not in name.lower():
      continue
    out.setdefault(name, []).append(e)
  return out


def clear(records, caps, marks):
  """Removes the caps / "no offset" marks of `records` in place; returns (caps cleared, marks cleared)."""
  nc = nm = 0
  for e in records:
    if caps and any(k in e for k in CAP_KEYS):
      nc += 1 if e.get("cap") else 0
      for k in CAP_KEYS:
        e.pop(k, None)
    if marks and "no_offset" in e:
      e.pop("no_offset", None)
      nm += 1
  return nc, nm


def main(argv=None, out=sys.stdout):
  ap = argparse.ArgumentParser(description="List / clean the curve memory by road name (no GPS printed).")
  ap.add_argument("--file", default=MEMORY_PATH, help=f"the curve memory file (default {MEMORY_PATH})")
  ap.add_argument("--road", help="only roads whose name contains this (case does not matter)")
  ap.add_argument("--clear-caps", action="store_true", help="remove the speed caps of the matched road(s)")
  ap.add_argument("--clear-marks", action="store_true", help='remove the "no offset" marks of the matched road(s)')
  ap.add_argument("--apply", action="store_true", help="write the change (default: a dry run)")
  a = ap.parse_args(argv)
  if (a.clear_caps or a.clear_marks) and not a.road:
    print("--clear-caps / --clear-marks need --road", file=out)
    return 2
  try:
    data = load(a.file)
  except (OSError, ValueError) as ex:
    print(f"cannot read {a.file}: {ex}", file=out)
    return 1
  now = time.time()
  roads = by_road(data["curves"], a.road)
  if not roads:
    print("no record matches" if a.road else "the curve memory is empty", file=out)
    return 0
  for name in sorted(roads):
    print(f"{name}: {len(roads[name])} record(s)", file=out)
    for k, e in enumerate(roads[name]):
      print(f"  {k + 1}. {describe(e, now)}", file=out)
  if not (a.clear_caps or a.clear_marks):
    return 0
  records = [e for rs in roads.values() for e in rs]
  nc, nm = clear(records, a.clear_caps, a.clear_marks)
  what = f"{nc} cap(s) and {nm} \"no offset\" mark(s) on {', '.join(sorted(roads))}"
  if not a.apply:
    print(f"dry run: would clear {what}; add --apply to write", file=out)
    return 0
  if nc == 0 and nm == 0:
    print("nothing to clear", file=out)
    return 0
  bak = a.file + ".bak"
  shutil.copy2(a.file, bak)
  tmp = a.file + ".tmp"
  with open(tmp, "w") as f:
    json.dump(data, f)
  os.replace(tmp, a.file)
  print(f"cleared {what}; the old file is {bak}", file=out)
  return 0


if __name__ == "__main__":
  sys.exit(main())
