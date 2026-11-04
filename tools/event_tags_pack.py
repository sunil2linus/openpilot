#!/usr/bin/env python3
"""Pack only the logs around the moments tagged with the on-road TAG button.

  ssh comma@<device ip> 'python3 /data/openpilot/tools/event_tags_pack.py --list'     list the tags
  ssh comma@<device ip> 'python3 /data/openpilot/tools/event_tags_pack.py'            pack tags not packed yet
  scp comma@<device ip>:/data/event_tags_<stamp>.tgz .                                 (the exact line is printed)

For each tag: qlog + qcamera.ts of every segment covering 10 s before to 20 s after the tap (--pre / --post),
the feature_events in that window, and the tag records. --rlog adds the full rlogs (big). Choose tags with
--last N, --since N, --tags 3,5, --hours H, --route NAME or --all; by default only tags newer than the last
pack (remembered in /data/media/0/event_tags/last_packed.txt, not updated with --no-mark or --dry-run).
"""
import argparse
import importlib.util
import json
import os
import shutil
import sys
import tarfile
import tempfile
import time

_here = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("event_tags", os.path.join(_here, "..", "starpilot", "common", "event_tags.py"))
et = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(et)

LEGEND = """Fields (speeds m/s, distances m, set speeds km/h as the car reports them):
  car.v_ego speed, car.v_cruise_cluster set speed (255 = not set), car.gear (raw enum, see gear_name)
  op.enabled openpilot engaged, op.aol always-on lateral steering, op.curve_offset our curve SET- offset
  lead.* radar lead (d_rel distance, v_lead speed), lead.adj_stopped stopped car in the next lane
  plan.red_light / forcing_stop / stop_sign, plan.map_curve.* map curve state (status 4 = paused, 5 = held)
  gps.* position, h_acc horizontal accuracy (m), age_s how old the GPS fix was at the tap
  segment / segment_s: the minute (segment) and seconds into it; segment_from lock = exact, mtime = guess
"""


def _parse_numbers(text):
  out = []
  for part in (text or "").split(","):
    part = part.strip()
    if not part:
      continue
    if "-" in part:
      a, b = part.split("-", 1)
      out.extend(range(int(a), int(b) + 1))
    else:
      out.append(int(part))
  return out


def _read_mark(root):
  try:
    with open(os.path.join(root, et.MARK_FILE)) as f:
      return int(f.read().strip() or 0)
  except (OSError, ValueError):
    return 0


def _write_mark(root, n):
  try:
    with open(os.path.join(root, et.MARK_FILE), "w") as f:
      f.write(f"{n}\n")
  except OSError as e:
    print(f"could not update {et.MARK_FILE}: {e}")


def choose(tags, args, mark):
  numbers = _parse_numbers(args.tags) if args.tags else None
  explicit = args.all or numbers or args.since is not None or args.last is not None or args.hours is not None
  return et.filter_tags(tags, last=args.last, since=args.since, numbers=numbers, hours=args.hours, route=args.route,
                        after_n=None if explicit else mark)


def pack(tags, tags_path, out, roots=et.LOG_ROOTS, events_root=et.EVENTS_ROOT, pre=et.PRE_S, post=et.POST_S,
         rlog=False, now=None, log=print):
  """Write the tgz; returns (out, [(route, n) packed], [(route, n) missing])."""
  spans = et.scan_segments(roots, now)
  wanted, missing = set(), set()
  per_tag = []
  for t in tags:
    found, miss = et.select_segments(t, spans, pre, post)
    wanted.update(found)
    missing.update(miss)
    per_tag.append((t, found, miss))
  events = et.select_feature_events(events_root, tags, pre, post)

  stage = tempfile.mkdtemp(prefix="event_tags_pack.")
  try:
    base = os.path.join(stage, "event_tags")
    os.makedirs(os.path.join(base, "tags"))
    if os.path.exists(tags_path):
      shutil.copy2(tags_path, os.path.join(base, "tags", "tags.jsonl"))
    with open(os.path.join(base, "tags", "selected.jsonl"), "w") as f:
      for t in tags:
        f.write(json.dumps(t, separators=(",", ":")) + "\n")
    with open(os.path.join(base, "tags", "README.txt"), "w") as f:
      for t, found, miss in per_tag:
        f.write(et.summary_line(t) + "\n")
        f.write("    segments: " + (", ".join(f"{r}--{n}" for r, n in found) or "none") + "\n")
        if miss:
          f.write("    not on the device any more: " + ", ".join(f"{r}--{n}" for r, n in miss) + "\n")
      f.write("\n" + LEGEND)

    names = ["qlog.zst", "qlog", "qcamera.ts"] + (["rlog.zst", "rlog"] if rlog else [])
    total = 0
    for route, n in sorted(wanted):
      sp = spans[(route, n)]
      dst = os.path.join(base, "logs", f"{route}--{n}")
      os.makedirs(dst, exist_ok=True)
      for name in names:
        src = os.path.join(sp.path, name)
        if os.path.exists(src):
          shutil.copy2(src, os.path.join(dst, name))
          total += os.path.getsize(src)
      log(f"  {route}--{n}{'  (still being written)' if sp.in_progress else ''}")
    for ev in events:
      shutil.copytree(ev, os.path.join(base, "feature_events", os.path.basename(ev)))
    mb = total // 1_000_000
    log(f"{len(tags)} tags, {len(wanted)} segments ({mb} MB before compression), {len(events)} feature events, " +
        f"{len(missing)} segments missing")

    with tarfile.open(out, "w:gz") as tar:
      tar.add(base, arcname="event_tags")
  finally:
    shutil.rmtree(stage, ignore_errors=True)
  return out, sorted(wanted), sorted(missing)


def _device_ip():
  """This device's address on the local network (no traffic is sent), for the printed scp line."""
  import socket
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      s.connect(("10.255.255.255", 1))
      return s.getsockname()[0]
  except OSError:
    return "<device ip>"


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--list", action="store_true", help="only list the tags")
  ap.add_argument("--all", action="store_true")
  ap.add_argument("--last", type=int)
  ap.add_argument("--since", type=int, help="tags with this number or higher")
  ap.add_argument("--tags", help="tag numbers, e.g. 3,5,8-10")
  ap.add_argument("--hours", type=float, help="tags from the last H hours")
  ap.add_argument("--route", help="only tags of this route (part of the name is enough)")
  ap.add_argument("--pre", type=float, default=et.PRE_S)
  ap.add_argument("--post", type=float, default=et.POST_S)
  ap.add_argument("--rlog", action="store_true", help="also the full rlogs (large)")
  ap.add_argument("--no-mark", action="store_true", help="do not remember these tags as packed")
  ap.add_argument("--dry-run", action="store_true", help="show what would be packed")
  ap.add_argument("--root", default=et.TAGS_ROOT)
  ap.add_argument("--out", help="output .tgz (default /data/event_tags_<time>.tgz)")
  args = ap.parse_args(argv)

  tags_path = os.path.join(args.root, et.TAGS_FILE)
  tags = et.read_tags(tags_path)
  if not tags:
    print(f"no tags in {tags_path}")
    return 0
  mark = _read_mark(args.root)

  if args.list:
    for t in tags:
      print(("   " if (t.get("n") or 0) <= mark else " * ") + et.summary_line(t))
    print(f"{len(tags)} tags; * = not packed yet")
    return 0

  sel = choose(tags, args, mark)
  if not sel:
    print("no tags selected (all packed already? use --all, --last N or --tags N)")
    return 0

  if args.dry_run:
    spans = et.scan_segments()
    for t in sel:
      found, miss = et.select_segments(t, spans, args.pre, args.post)
      print(et.summary_line(t))
      print("    segments: " + (", ".join(f"{r}--{n}" for r, n in found) or "none") +
            (f"   missing: {', '.join(f'{r}--{n}' for r, n in miss)}" if miss else ""))
    return 0

  out = args.out or time.strftime("/data/event_tags_%Y%m%d_%H%M%S.tgz")
  pack(sel, tags_path, out, pre=args.pre, post=args.post, rlog=args.rlog)
  if not args.no_mark:
    top = max((t.get("n") or 0) for t in sel)
    if top > mark:
      _write_mark(args.root, top)
  size = os.path.getsize(out) // 1_000_000
  print(f"wrote {out} ({size} MB)")
  print("copy it to your computer with:")
  print(f"  scp comma@{_device_ip()}:{out} .")
  return 0


if __name__ == "__main__":
  sys.exit(main())
