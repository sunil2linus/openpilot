#!/usr/bin/env python3
"""Report on the Mazda model follower (TEST ONLY: opendbc/car/mazda/model_follow.py logs the set speed the driving model
would want, it never presses a button).

  python3 tools/model_follow_report.py                          the device's logs (/data/media/0/feature_events)
  python3 tools/model_follow_report.py --events ./feature_events   a copy (tools/feature_events_pack.sh packs them)
  python3 tools/model_follow_report.py --log rlog.zst [...]       replay the follower over a route's rlogs / qlogs

What it reads: every <time>_modelFollow/summary.txt (one episode: the follower's set speed and the real one were 3 mph
or more apart for 1 s or more) and model_follow_drives.csv (one line per cruise off / 5 min of cruise; the last line of
a drive has its totals). With --log it runs the follower itself on carState, radarState, modelV2 (rlogs; qlogs only have
drivingModelData's accel, no plan) and starpilotPlan.redLight; the driver's speed is taken as the highest set speed of
the last 30 s (the logs do not have our own anchor).

What it prints, in plain words:
  * every episode: what happened (the driver braked, the car slowed by itself, our own features lowered the set speed)
    and when the follower would have started lowering against when the real set speed went down / the driver braked:
    "earlier by 2.5 s" means the model would have started 2.5 s before what really happened
  * would-be false slowdowns: the follower lowered, but the car did not slow, the driver did not brake and nothing of
    ours lowered the set speed
  * the opposite: our features lowered the set speed and the model did not want to
  * would-be button presses per hour of cruise (1 mph taps, and press actions with long presses)
"""
import argparse
import csv
import glob
import os
import sys

MPH = 0.44704
DEFAULT_EVENTS = "/data/media/0/feature_events"
SLOWED_MPH = 5.0           # the car slowed this much (or to 3/4 of its speed) in or after the episode: a real slowdown


def _num(v):
  if v is None or v == "":
    return None
  try:
    return float(v)
  except (TypeError, ValueError):
    return v


def read_summary(path):
  """summary.txt -> dict (numbers as floats, '' as None)."""
  out = {}
  with open(path) as f:
    for line in f:
      if ": " in line:
        k, v = line.rstrip("\n").split(": ", 1)
        out[k] = _num(v)
      elif line.rstrip("\n").endswith(":"):
        out[line.rstrip("\n")[:-1]] = None
  return out


def load_events(root):
  eps = []
  for folder in sorted(glob.glob(os.path.join(root, "*_modelFollow*"))):
    p = os.path.join(folder, "summary.txt")
    if os.path.isfile(p):
      try:
        ep = read_summary(p)
        ep["folder"] = os.path.basename(folder)
        eps.append(ep)
      except OSError:
        pass
  return eps


def load_drives(root):
  """The last line of every drive in model_follow_drives.csv (and its .1)."""
  drives = {}
  for name in ("model_follow_drives.csv.1", "model_follow_drives.csv"):
    p = os.path.join(root, name)
    if not os.path.isfile(p):
      continue
    with open(p) as f:
      for row in csv.DictReader(f):
        drives[row.get("drive_start_wall", "")] = row
  return list(drives.values())


def _true(v):
  return v in (1, 1.0, "1", True)


def classify(ep):
  """(what happened, was it a would-be false slowdown)."""
  v0 = ep.get("vEgo_mph") or 0.0
  v_min = min(x for x in (ep.get("v_min_mph"), ep.get("v_min_after_mph"), v0) if isinstance(x, float))
  slowed = v_min <= max(v0 - SLOWED_MPH, 0.75 * v0)
  braked = _true(ep.get("driver_braked")) or _true(ep.get("driver_braked_after"))
  feats = str(ep.get("features_any") or ep.get("features") or "")
  feats = "" if feats == "none" else feats
  ours = ep.get("actual_lower_at_s") is not None or "stop:" in feats or "ours" in feats
  what = []
  if braked:
    what.append("driver braked")
  if slowed:
    what.append(f"car slowed {v0:.0f} -> {v_min:.0f} mph")
  if ours:
    what.append("our features lowered the set" + (f" ({feats})" if feats else ""))
  if _true(ep.get("red_light_any")) or _true(ep.get("red_light")):
    what.append("red light seen")
  false = ep.get("kind") == "follower_lower" and not (braked or slowed or ours)
  return ("; ".join(what) or "nothing (no brake, no slowdown, nothing of ours)"), false


def timing(ep):
  """Plain words: the follower's first lowering against the real set speed's drop and the driver's brake."""
  f = ep.get("follow_lower_at_s")
  refs = [(n, ep.get(k)) for n, k in (("our set speed drop", "actual_lower_at_s"), ("driver brake", "brake_at_s"))
          if isinstance(ep.get(k), float)]
  if not refs:
    return "nothing to compare with (no set speed drop, no brake)"
  name, ref = min(refs, key=lambda x: x[1])
  if not isinstance(f, float):
    return f"the follower did not lower ({name} at {ref:+.1f} s)"
  d = ref - f
  return f"follower {'earlier' if d >= 0 else 'later'} by {abs(d):.1f} s than the {name}"


def print_report(eps, drives, out=sys.stdout):
  w = out.write
  hours = taps = actions = 0.0
  if drives:
    w("Drives (the last summary line of each)\n")
    for d in sorted(drives, key=lambda r: r.get("drive_start_wall", "")):
      h = (_num(d.get("cruise_on_s")) or 0.0) / 3600
      t, a = _num(d.get("would_taps")) or 0.0, _num(d.get("would_actions")) or 0.0
      hours, taps, actions = hours + h, taps + t, actions + a
      w(f"  {d.get('route') or d.get('drive_start_wall')}: cruise {h * 60:.0f} min, lowered {d.get('lowers')} times, " +
        f"would-be taps {t:.0f} ({t / h if h else 0:.0f}/h), press actions {a:.0f} ({a / h if h else 0:.0f}/h), " +
        f"episodes {d.get('episodes')} (written {d.get('episodes_written')}), model: modelV2 {d.get('src_modelV2_s')} s, " +
        f"drivingModelData {d.get('src_drivingModelData_s')} s, none {d.get('no_model_s')} s; " +
        f"cost {d.get('call_avg_us')} us avg, {d.get('call_max_us')} us max" +
        (f"; ERROR {d.get('error')}" if d.get("error") else "") + "\n")
    if hours:
      w(f"  all drives: {hours * 60:.0f} min of cruise, would-be taps {taps / hours:.0f}/h, press actions {actions / hours:.0f}/h\n")
  w(f"\nEpisodes: {len(eps)}\n")
  false = []
  missed = []
  for ep in eps:
    what, is_false = classify(ep)
    if is_false:
      false.append(ep)
    if ep.get("kind") == "actual_lower":
      missed.append(ep)
    w(f"  {ep.get('folder') or ep.get('t0', '')}: {ep.get('kind')}, {ep.get('vEgo_mph') or 0:.0f} mph, set {ep.get('set_mph') or 0:.0f}, " +
      f"follower min {ep.get('follow_min_mph') or 0:.0f}, real set min {ep.get('set_min_mph') or 0:.0f}, " +
      f"model accel min {ep.get('model_accel_min') or 0:.2f}, lead {'d ' + format(ep['lead_d'], '.0f') + ' m' if ep.get('lead_d') else 'none'}, " +
      f"{ep.get('episode_s') or 0:.1f} s, would-be taps {ep.get('would_taps') or 0:.0f}\n")
    w(f"      what happened: {what}\n      timing: {timing(ep)}\n")
  w(f"\nWould-be false slowdowns (the follower lowered, nothing followed, no brake): {len(false)}\n")
  for ep in false:
    w(f"  {ep.get('folder') or ep.get('t0', '')}: {ep.get('vEgo_mph') or 0:.0f} mph, follower down to {ep.get('follow_min_mph') or 0:.0f}, " +
      f"model accel min {ep.get('model_accel_min') or 0:.2f}\n")
  w(f"Our features lowered and the model did not want to: {len(missed)}\n")
  for ep in missed:
    w(f"  {ep.get('folder') or ep.get('t0', '')}: {ep.get('features_any') or ep.get('features')}, real set down to " +
      f"{ep.get('set_min_mph') or 0:.0f}, {classify(ep)[0]}\n")
  return false, missed


# ---- replay over logs ----
def replay(samples, is_metric=False, dt=0.05, driver_window=30.0):
  """Run the follower and the episode tracker over samples, dicts at a fixed rate of 1/dt with: t, v, a_ego, cruise,
  set (m/s), brake, gas, model_a (None: no model), plan_t / plan_v (or None), lead (status, d, v, prob), red_light.
  Returns (episodes as merged open+close summaries, the follower, the timeline [(t, follow set, real set, reason)])."""
  sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "opendbc_repo"))
  from opendbc.car.mazda.model_follow import ModelFollower, EpisodeTracker
  f = ModelFollower(is_metric, dt=dt)
  tr = EpisodeTracker()
  eps, cur, timeline = [], None, []
  recent = []                                   # (t, set) while cruise is on
  for s in samples:
    t = s["t"]
    if s["cruise"] and s["set"] > 0:
      recent.append((t, s["set"]))
    else:
      recent = []
    while recent and recent[0][0] < t - driver_window:
      recent.pop(0)
    driver = max(x[1] for x in recent) if recent else 0.0
    lead = s.get("lead") or (False, 0.0, 0.0, 0.0)
    radar_lead = s.get("radar_lead")
    if radar_lead is None:      # not in the logs: a confident car in reach of the stock radar's following
      radar_lead = bool(lead[0]) and lead[3] >= 0.5 and lead[1] <= 2.5 * s["v"] + 10.0
    cmd = f.update(s["v"], driver, s["cruise"], s["model_a"], s.get("plan_t"), s.get("plan_v"), radar_lead=radar_lead)
    acts = tr.update(t, s["cruise"], s["v"], s["a_ego"], cmd, s["set"] if s["cruise"] else 0.0, driver,
                     s["model_a"] if s["model_a"] is not None else 0.0, f.reason, f.presses, f.press_actions, lead[0], lead[1],
                     lead[2], lead[3], s.get("red_light", False), "", s["brake"], s["gas"], False)
    for kind, summary in acts or ():
      if kind == "open":
        cur = dict(summary, t0=round(tr.ep["t0"], 2))
      elif cur is not None:
        cur.update(summary)
        eps.append(cur)
        cur = None
    timeline.append((t, cmd, s["set"] if s["cruise"] else 0.0, f.reason))
  return eps, f, timeline


def samples_from_logs(paths, dt=0.05):
  """Samples for replay() from rlogs / qlogs (needs openpilot's LogReader)."""
  try:
    from openpilot.tools.lib.logreader import LogReader
  except ImportError:
    sys.exit("--log needs openpilot's tools.lib.logreader (run from an openpilot checkout)")
  last = {"carState": None, "radarState": None, "modelV2": None, "drivingModelData": None, "starpilotPlan": None}
  t0, next_t = None, None
  for p in paths:
    for m in LogReader(p):
      w = m.which()
      if w not in last:
        continue
      if t0 is None:
        t0 = m.logMonoTime
        next_t = 0.0
      t = (m.logMonoTime - t0) / 1e9
      last[w] = getattr(m, w)
      while next_t is not None and t >= next_t and last["carState"] is not None:
        c = last["carState"]
        model_a, pt, pv = None, None, None
        if last["modelV2"] is not None:
          model_a = float(last["modelV2"].action.desiredAcceleration)
          pt, pv = list(last["modelV2"].velocity.t), list(last["modelV2"].velocity.x)
        elif last["drivingModelData"] is not None:
          model_a = float(last["drivingModelData"].action.desiredAcceleration)
        lead = (False, 0.0, 0.0, 0.0)
        if last["radarState"] is not None:
          ld = last["radarState"].leadOne
          lead = (bool(ld.status), float(ld.dRel), float(ld.vLead), float(ld.modelProb))
        red = bool(getattr(last["starpilotPlan"], "redLight", False)) if last["starpilotPlan"] is not None else False
        yield {"t": next_t, "v": float(c.vEgo), "a_ego": float(c.aEgo), "cruise": bool(c.cruiseState.enabled), "radar_lead": None,
               "set": float(c.cruiseState.speed), "brake": bool(c.brakePressed), "gas": bool(c.gasPressed), "model_a": model_a,
               "plan_t": pt, "plan_v": pv, "lead": lead, "red_light": red}
        next_t += dt


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--events", default=DEFAULT_EVENTS, help="the feature_events folder (default: the device's)")
  ap.add_argument("--log", nargs="*", default=None, help="rlog / qlog files of one route, in order: replay the follower")
  ap.add_argument("--metric", action="store_true", help="with --log: the car's set speed is in km/h")
  args = ap.parse_args(argv)
  if args.log:
    eps, f, timeline = replay(samples_from_logs(args.log), args.metric)
    cruise_h = sum(1 for _, _, s, _ in timeline if s > 0) * 0.05 / 3600
    print(f"Replay: {cruise_h * 60:.0f} min of cruise, lowered {f.lowers} times, would-be taps {f.taps} " +
          f"({f.taps / cruise_h if cruise_h else 0:.0f}/h), press actions {f.actions} ({f.actions / cruise_h if cruise_h else 0:.0f}/h)")
    print_report(eps, [])
    return
  print_report(load_events(args.events), load_drives(args.events))


if __name__ == "__main__":
  main()
