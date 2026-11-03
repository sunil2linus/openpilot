#!/usr/bin/env python3
"""
List (and with --delete, remove) downloaded offline map folders that overlap none of the regions selected in
StarPilot's map settings: leftovers of an older download (FrogPilot and StarPilot share /data/media/0/osm).

  python3 tools/curve_map/osm_leftovers.py            # list only: what would go, and how much space
  python3 tools/curve_map/osm_leftovers.py --delete   # remove those folders

Safety: nothing is removed when the selection is empty or holds a region this tool cannot place on the map (it
could not tell that region's folders from leftovers). A folder is kept when it overlaps any selected region, with a
margin, using the states' real outlines (starpilot/common/map_region_shapes.json), so nothing
near a state line goes. Only <lat>/<lon> folders under the offline root are ever touched.
"""
import argparse
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from starpilot.common.map_region_boxes import box_overlaps, place_tokens

OFFLINE_ROOT = "/data/media/0/osm/offline"
PARAM_PATH = "/data/params/d/MapsSelected"
FOLDER_RE = re.compile(r"^-?\d+/-?\d+$")


def read_selection(path=PARAM_PATH):
  try:
    with open(path) as f:
      raw = f.read()
  except OSError:
    return []
  return [t.strip() for t in raw.split(",") if t.strip()]


def folder_size(path):
  total = 0
  for dirpath, _dirs, files in os.walk(path):
    for name in files:
      try:
        total += os.path.getsize(os.path.join(dirpath, name))
      except OSError:
        pass
  return total


def downloaded_folders(root=OFFLINE_ROOT):
  out = []
  try:
    for la in os.listdir(root):
      for lo in os.listdir(os.path.join(root, la)):
        name = f"{la}/{lo}"
        if FOLDER_RE.match(name) and os.path.isdir(os.path.join(root, la, lo)):
          out.append(name)
  except OSError:
    pass
  return sorted(out)


def find_leftovers(folders, tokens):
  """(leftover folder names, kept folder names, problem text or None)."""
  if not tokens:
    return [], list(folders), "no maps selected in StarPilot's map settings: nothing can be told apart"
  placed, unknown = place_tokens(tokens)
  if unknown:
    return [], list(folders), f"cannot place {', '.join(unknown)} on the map: not removing anything"
  keep = [f for f in folders if any(box_overlaps(f, region) for region in placed.values())]
  leftovers = [f for f in folders if f not in keep]
  return leftovers, keep, None


def main(argv=None, root=OFFLINE_ROOT, param_path=PARAM_PATH, out=print):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--delete", action="store_true", help="remove the leftover folders (default: list only)")
  args = ap.parse_args(argv)
  tokens = read_selection(param_path)
  folders = downloaded_folders(root)
  leftovers, keep, problem = find_leftovers(folders, tokens)
  out(f"selected maps: {', '.join(tokens) or '(none)'}")
  out(f"{len(folders)} downloaded map folders: {len(keep)} overlap your selected maps, {len(leftovers)} do not")
  if problem:
    out(problem)
    return 1
  total = 0
  for f in leftovers:
    total += folder_size(os.path.join(root, f))
  out(f"leftovers: {len(leftovers)} folders, {total / 1e9:.1f} GB")
  if not args.delete:
    out("list only: run again with --delete to remove them")
    return 0
  for f in leftovers:
    shutil.rmtree(os.path.join(root, f), ignore_errors=True)
  for la in os.listdir(root):
    try:
      os.rmdir(os.path.join(root, la))  # only removes now-empty latitude folders
    except OSError:
      pass
  out(f"removed {len(leftovers)} folders ({total / 1e9:.1f} GB); kept {len(keep)}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
