#!/usr/bin/env python3
"""Curve memory maintenance (run on the device, parked):

  python -m openpilot.starpilot.car.mazda.speed.curve_memory_tool list [--name TEXT]
  python -m openpilot.starpilot.car.mazda.speed.curve_memory_tool clear --name TEXT [--apply]

Lists / clears the remembered curves by road name (case-insensitive part of the name). clear is a dry run unless
--apply; with --apply the file is first copied to <file>.bak. Prints road names, way ids, passes, caps; never a position.
"""
import argparse
import json
import os
import shutil
import sys
import time

from openpilot.starpilot.car.mazda.speed.curve_memory import MEMORY_PATH, MPH, VERSION


def load(path):
  with open(path) as f:
    d = json.load(f)
  if not isinstance(d, dict) or d.get("version") != VERSION:
    raise ValueError("not a curve memory file of this version")
  return d


def matches(entry, text):
  return text is None or text.lower() in str(entry.get("name", "")).lower()


def describe(key, e, now):
  cap = e.get("cap")
  cap_s = f"cap {cap['v'] / MPH:.0f} mph ({(now - cap['t']) / 86400:.0f} d ago)" if cap else "no cap"
  mark = f", no offset ({(now - e['no_offset']) / 86400:.0f} d ago)" if e.get("no_offset") else ""
  clean = sum(1 for p in e.get("passes", []) if p.get("clean"))
  return f"{e.get('name') or '(unnamed)'} [{key}{' ramp' if e.get('ramp') else ''}]: {len(e.get('passes', []))} passes " + \
         f"({clean} clean), {cap_s}{mark}"


def main(argv=None, out=print, now=None):
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("command", choices=("list", "clear"))
  p.add_argument("--name", help="part of the road name")
  p.add_argument("--apply", action="store_true", help="clear: really change the file (a .bak is written first)")
  p.add_argument("--file", default=MEMORY_PATH)
  a = p.parse_args(argv)
  now = time.time() if now is None else now  # noqa: TID251
  if not os.path.exists(a.file):
    out("no curve memory yet")
    return 0
  try:
    d = load(a.file)
  except (OSError, ValueError) as e:
    out(f"cannot read {a.file}: {e}")
    return 1
  hits = {k: e for k, e in d["curves"].items() if matches(e, a.name)}
  if a.command == "list":
    for k in sorted(hits, key=lambda k: (str(hits[k].get("name")), k)):
      out(describe(k, hits[k], now))
    out(f"{len(hits)} of {len(d['curves'])} curves")
    return 0
  if not a.name:
    out("clear needs --name")
    return 2
  for k in sorted(hits):
    out(("clearing " if a.apply else "would clear ") + describe(k, hits[k], now))
  if not a.apply:
    out(f"dry run: {len(hits)} curves would be cleared (add --apply)")
    return 0
  shutil.copy2(a.file, a.file + ".bak")
  for k in hits:
    del d["curves"][k]
  tmp = a.file + ".tmp"
  with open(tmp, "w") as f:
    json.dump(d, f, indent=1)
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, a.file)
  out(f"cleared {len(hits)} curves (backup: {a.file}.bak)")
  return 0


if __name__ == "__main__":
  sys.exit(main())
