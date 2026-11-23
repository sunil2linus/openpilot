"""
Counts of the built curve maps (format 6: tagged limits, link joints), for checking a rebuild over ssh, parked:

  python -m openpilot.starpilot.car.mazda.speed.map_diag [--ways 486581349,1214344938] [--name "Hall Road"] [--folder 42/-84]

Prints counts and ids only, never a position: the map areas by format, the road shapes with a tagged limit by road
class, the link records with stored joints / street cuts / turnarounds / "to a highway" (map_view.LinkShapes on the
stored answers), the given ways' link flags, and the tagged limits found on the ways with the given name (their ids
read from 7.0's selected offline tiles of that folder). Read-only.
"""
import argparse
import collections
import glob
import json
import os

from openpilot.starpilot.car.mazda.speed import curve_builder as B
from openpilot.starpilot.car.mazda.speed import curve_mapd as D

NAMED = (486581349, 490860778, 17963358, 443260504, 443254109, 8727340, 8728850, 8727544, 8729433, 1214344938,
         402416597, 8727616)


class _Store:
  def __init__(self, chains):
    self.chains, self.grid = chains, []


def _code(c):
  return f"{c} mph" if c > 0 else f"{-c} km/h"


def folder_counts(path, ways=(), name_ids=frozenset()):
  """Counts of one built folder (dict)."""
  from openpilot.starpilot.car.mazda.speed import map_view as V
  out = {"files": 0, "format": collections.Counter(), "tagged": collections.Counter(), "untagged": collections.Counter(),
         "ramps": 0, "with_j": 0, "cut": 0, "turn": 0, "to_hwy": 0, "named": {}, "name_limits": collections.Counter()}
  chains, ramps, seen = {}, [], set()
  for p in sorted(glob.glob(os.path.join(path, "*.json"))):
    try:
      with open(p) as f:
        d = json.load(f)
    except (OSError, ValueError):
      continue
    out["files"] += 1
    out["format"][(d.get("version"), d.get("builder"))] += 1
    for r in d.get("roads", []):
      w = int(r.get("w", 0))
      if w in seen:
        continue
      seen.add(w)
      lim = r.get("l")
      (out["tagged"] if lim is not None else out["untagged"])[int(r.get("k", 0))] += 1
      if w in name_ids and lim is not None:
        for c in ({lim} if isinstance(lim, int) else set(lim)) - {0}:
          out["name_limits"][_code(int(c))] += 1
    for ch in d.get("chains", []):
      chains[ch["id"]] = ch
    ramps += d.get("ramps", [])
  links = V.LinkShapes(_Store(chains))
  for r in ramps:
    if r.get("travel") == "both":
      continue
    out["ramps"] += 1
    out["with_j"] += "j" in r
    inf = links.info(r)
    hwy = links.leads(r)[0]
    out["cut"] += bool(inf["cuts"])
    out["turn"] += bool(inf["turn"])
    out["to_hwy"] += bool(hwy)
    ws = [int(w) for w, _f, _s in (chains.get(r.get("chain")) or {"w": []})["w"]]
    for w in ways:
      if w in ws:
        out["named"].setdefault(w, []).append({"type": r.get("type"), "ways": len(ws), "stored": "j" in r,
                                               "cuts": [list(c) for c in inf["cuts"]], "turn": bool(inf["turn"]),
                                               "to_highway": bool(hwy)})
  return out


def name_ids(folder, name):
  """Way ids with this name (or ref) in 7.0's selected offline tiles of `folder` (empty when not found)."""
  try:
    from openpilot.starpilot.maps.storage import offline_root
    sel = D.selected_folders(offline_root()) or {}
  except Exception:
    return set()
  info = sel.get(folder)
  if info is None:
    return set()
  schema = B.load_schema()
  out = set()
  for tile in info["tiles"]:
    t = B.read_tile(schema, os.path.join(info["dir"], tile))
    if t is None:
      continue
    for w in t.ways:
      if name and (w.name == name or w.ref == name):
        out.add(int(w.id))
  return out


def main(argv=None):
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--root", default=D.CURVE_ROOT)
  p.add_argument("--ways", default=",".join(map(str, NAMED)))
  p.add_argument("--name", default="Hall Road")
  p.add_argument("--folder", default="42/-84", help="the folder whose offline tiles give the named ways' ids")
  a = p.parse_args(argv)
  ways = [int(w) for w in a.ways.split(",") if w.strip()]
  ids = name_ids(a.folder, a.name) if a.name else set()
  folders = sorted(os.path.relpath(os.path.dirname(s), a.root) for s in glob.glob(os.path.join(a.root, "*", "*", D.STATE_NAME)))
  states = collections.Counter()
  total = collections.defaultdict(collections.Counter)
  named = {}
  for f in folders:
    st = D.read_state(os.path.join(a.root, f)) or {}
    states[(st.get("format"), st.get("builder"))] += 1
    c = folder_counts(os.path.join(a.root, f), ways, ids if f == a.folder else frozenset())
    for k in ("tagged", "untagged", "name_limits", "format"):
      total[k].update(c[k])
    for k in ("files", "ramps", "with_j", "cut", "turn", "to_hwy"):
      total["n"][k] += c[k]
    for w, lst in c["named"].items():
      named.setdefault(w, []).extend(lst)
  print(f"map areas: {len(folders)}; by (format, builder): {dict(states)}; files by (format, builder): {dict(total['format'])}")
  print("road shapes (ways) with a tagged limit, by class:")
  for cls in sorted(set(total["tagged"]) | set(total["untagged"])):
    print(f"  class {cls:2d} {B.CLASS_NAMES.get(cls, '?'):14s} tagged {total['tagged'][cls]:6d}  untagged {total['untagged'][cls]:6d}")
  n = total["n"]
  print(f"one-way link records {n['ramps']}: stored joints {n['with_j']}, street cuts {n['cut']}, turnarounds {n['turn']}, "
        f"to a highway {n['to_hwy']}")
  for w in ways:
    print(f"way {w}: {named.get(w, 'not in a link record')}")
  print(f"'{a.name}' ways in {a.folder}'s tiles: {len(ids)}; tagged limits on their shapes: {dict(total['name_limits'])}")


if __name__ == "__main__":
  main()
