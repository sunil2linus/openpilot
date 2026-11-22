"""The speed control's tags (part T): every press of 7.0's onroad Bookmark button writes one "tag" line into the event
log (caller.py). This tool lists them and packs the log lines around each tag into one file to send.

  python -m openpilot.starpilot.car.mazda.speed.tag_tool list
  python -m openpilot.starpilot.car.mazda.speed.tag_tool pack            (dry run: says what it would write)
  python -m openpilot.starpilot.car.mazda.speed.tag_tool pack --write [--out FILE] [--window 60]

pack keeps the lines within --window s (default 60) of each tag, from the same drive's file, in time order, each
tag's lines after a {"kind": "pack_tag"} header line. The log never has a position; pack drops any position-like key
anyway (POSITION_KEYS). Never changes the logs.
"""
import argparse
import json
import os
import sys
import time

from openpilot.starpilot.car.mazda.speed.events import LOG_DIR

WINDOW = 60.0
POSITION_KEYS = frozenset(("lat", "lon", "lng", "latitude", "longitude", "position", "positions"))
MAX_LINE = 200_000


def _clean(row):
  if isinstance(row, dict):
    return {k: _clean(v) for k, v in row.items() if str(k).lower() not in POSITION_KEYS}
  if isinstance(row, list):
    return [_clean(v) for v in row]
  return row


def read_file(path):
  rows = []
  try:
    with open(path, encoding="utf-8", errors="replace") as f:
      for line in f:
        if len(line) > MAX_LINE:
          continue
        try:
          row = json.loads(line)
        except ValueError:
          continue
        if isinstance(row, dict) and isinstance(row.get("t"), (int, float)):
          rows.append(row)
  except OSError:
    pass
  return rows


def log_files(directory=LOG_DIR):
  try:
    names = sorted(n for n in os.listdir(directory) if n.startswith("events-") and n.endswith(".jsonl"))
  except OSError:
    return []
  return [os.path.join(directory, n) for n in names]


def tags(directory=LOG_DIR):
  """[(file, tag row)] in file and time order."""
  out = []
  for path in log_files(directory):
    out.extend((path, row) for row in read_file(path) if row.get("kind") == "tag")
  return out


def describe(path, row):
  wall = row.get("wall")
  when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(wall)) if isinstance(wall, (int, float)) else "?"
  v = row.get("v")
  v_txt = f"{v / 0.44704:.0f} mph" if isinstance(v, (int, float)) else "?"
  return (f"{os.path.basename(path)}  tag {row.get('n', '?')}  {when}  drive {row.get('drive_s', '?')} s  " +
          f"route {row.get('route') or '?'}  {v_txt}  reason {row.get('reason') or '-'}")


def pack(directory=LOG_DIR, window=WINDOW):
  """The packed lines (dicts), each tag's window after its header."""
  out = []
  for path in log_files(directory):
    rows = read_file(path)
    for tag in (r for r in rows if r.get("kind") == "tag"):
      t = tag["t"]
      out.append({"kind": "pack_tag", "file": os.path.basename(path), "n": tag.get("n"), "t": t, "window": window})
      out.extend(_clean(r) for r in rows if abs(r["t"] - t) <= window)
  return out


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("command", choices=("list", "pack"))
  parser.add_argument("--dir", default=LOG_DIR)
  parser.add_argument("--out", default=None, help="pack: the file to write (default: tags-<time>.jsonl in --dir)")
  parser.add_argument("--window", type=float, default=WINDOW)
  parser.add_argument("--write", action="store_true", help="pack: really write the file (default: dry run)")
  args = parser.parse_args(argv)
  if args.command == "list":
    found = tags(args.dir)
    for path, row in found:
      print(describe(path, row))
    print(f"{len(found)} tag(s)")
    return 0
  lines = pack(args.dir, args.window)
  n_tags = sum(1 for r in lines if r.get("kind") == "pack_tag")
  out = args.out or os.path.join(args.dir, time.strftime("tags-%Y%m%d-%H%M%S.jsonl"))
  if not args.write:
    print(f"dry run: would write {len(lines)} line(s) for {n_tags} tag(s) to {out} (add --write)")
    return 0
  tmp = out + ".tmp"
  with open(tmp, "w") as f:
    for r in lines:
      f.write(json.dumps(r, separators=(",", ":"), default=str) + "\n")
  os.replace(tmp, out)
  print(f"wrote {len(lines)} line(s) for {n_tags} tag(s) to {out}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
