"""Offline tests for the per-event logs (ring buffer, writer, pruning, failure safety)."""
import importlib.util
import os
import tempfile

_spec = importlib.util.spec_from_file_location(
  "feature_events", os.path.join(os.path.dirname(__file__), "..", "feature_events.py"))
fe = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fe)

COLS = ["vEgo", "set", "press"]


class FakeParams:
  def __init__(self, route="00000040--abcdef1234"):
    self.route = route

  def get(self, key, encoding=None):
    return self.route if key == "CurrentRoute" else None


def make(root, **kw):
  kw.setdefault("hz", 20)
  kw.setdefault("write_in_thread", False)
  kw.setdefault("params", FakeParams())
  return fe.EventRecorder(COLS, root=root, **kw)


def feed(rec, t0, t1, hz=20, press_at=()):
  t = t0
  while t <= t1 + 1e-9:
    rec.add_row(t, (10.0 + t, 20.0, "M" if round(t, 2) in press_at else ""))
    rec.tick(t)
    t = round(t + 1.0 / hz, 4)


def folders(root):
  return sorted(e for e in os.listdir(root) if os.path.isdir(os.path.join(root, e)))


def read_rows(root, folder):
  with open(os.path.join(root, folder, "signals.csv")) as f:
    lines = f.read().strip().split("\n")
  return lines[0].split(","), [ln.split(",") for ln in lines[1:]]


def test_point_event_window_and_contents():
  with tempfile.TemporaryDirectory() as d:
    rec = make(d)
    feed(rec, 0.0, 40.0)                      # 40 s of rows: only the last 20 s are kept
    rec.record("stopAhead", 40.0, {"speed_mph": 45.2, "mode": "test", "fired": True})
    assert folders(d) == []                   # nothing written during the post window
    feed(rec, 40.05, 44.9)
    assert folders(d) == []
    feed(rec, 44.95, 45.5)
    fs = folders(d)
    assert len(fs) == 1 and fs[0].endswith("_stopAhead"), fs
    header, rows = read_rows(d, fs[0])
    assert header == ["t_rel"] + COLS
    t_rel = [float(r[0]) for r in rows]
    assert abs(t_rel[0] - (-15.0)) < 0.06 and abs(t_rel[-1] - 5.0) < 0.06, (t_rel[0], t_rel[-1])
    assert len(rows) == 401, len(rows)
    with open(os.path.join(d, fs[0], "summary.txt")) as f:
      s = f.read()
    assert "kind: stopAhead" in s and "speed_mph: 45.200" in s and "fired: 1" in s and "mode: test" in s
    assert "route: 00000040--abcdef1234" in s and "time_monotonic: 40.000" in s


def test_episode_event_keeps_rows_while_open():
  with tempfile.TemporaryDirectory() as d:
    rec = make(d)
    feed(rec, 0.0, 30.0)
    ev = rec.open("curveSlowdown", 30.0, {"set_before": 50})
    feed(rec, 30.05, 70.0, press_at=(31.0, 31.25, 60.0))     # a 40 s episode, longer than the ring buffer
    rec.close(ev, 70.0, {"presses": 3})
    feed(rec, 70.05, 75.2)
    fs = folders(d)
    assert len(fs) == 1 and fs[0].endswith("_curveSlowdown")
    _, rows = read_rows(d, fs[0])
    t_rel = [float(r[0]) for r in rows]
    assert abs(t_rel[0] + 15.0) < 0.06 and abs(t_rel[-1] - 45.0) < 0.06, (t_rel[0], t_rel[-1])
    assert sum(1 for r in rows if r[3] == "M") == 3
    with open(os.path.join(d, fs[0], "summary.txt")) as f:
      s = f.read()
    assert "duration_s: 40.00" in s and "presses: 3" in s and "set_before: 50" in s


def test_overlapping_point_events_each_get_a_folder():
  with tempfile.TemporaryDirectory() as d:
    rec = make(d)
    feed(rec, 0.0, 20.0)
    rec.record("stopAhead", 20.0, {})
    feed(rec, 20.05, 22.0)
    rec.record("stoppedLead", 22.0, {})
    feed(rec, 22.05, 28.0)
    fs = folders(d)
    assert len(fs) == 2 and fs[0].endswith("_stopAhead") and fs[1].endswith("_stoppedLead"), fs


def test_same_second_events_do_not_overwrite():
  with tempfile.TemporaryDirectory() as d:
    rec = make(d, clock=lambda: 1_700_000_000.0)
    feed(rec, 0.0, 10.0)
    rec.record("stopAhead", 10.0, {})
    rec.record("stopAhead", 10.0, {})
    feed(rec, 10.05, 16.0)
    assert len(folders(d)) == 2


def test_pruning_by_count_and_size():
  with tempfile.TemporaryDirectory() as d:
    for k in range(12):
      p = os.path.join(d, f"2026-01-01_00-00-{k:02d}_x")
      os.makedirs(p)
      with open(os.path.join(p, "signals.csv"), "w") as f:
        f.write("x" * 1000)
    assert fe.prune(d, max_bytes=10_000_000, max_events=10) == 2
    assert folders(d)[0] == "2026-01-01_00-00-02_x"            # the oldest went first
    assert fe.prune(d, max_bytes=4500, max_events=100) == 6
    assert len(folders(d)) == 4
    # the recorder prunes after each write
    rec = make(d, max_bytes=10_000_000, max_events=3)
    feed(rec, 0.0, 20.0)
    rec.record("stopAhead", 20.0, {})
    feed(rec, 20.05, 26.0)
    assert len(folders(d)) == 3 and folders(d)[-1].endswith("_stopAhead")


def test_writer_failure_never_raises():
  with tempfile.TemporaryDirectory() as d:
    bad = os.path.join(d, "not", "writable")
    rec = make(bad)
    os.makedirs(os.path.join(d, "not"), mode=0o500)
    feed(rec, 0.0, 20.0)
    rec.record("stopAhead", 20.0, {})
    feed(rec, 20.05, 26.0)
    assert rec.failed >= 1 and rec.written == 0 or os.geteuid() == 0   # root can write anywhere
    os.chmod(os.path.join(d, "not"), 0o700)
  # a params object that raises, and a broken clock: still no exception
  class BadParams:
    def get(self, *a, **k):
      raise RuntimeError("no params")
  with tempfile.TemporaryDirectory() as d:
    rec = make(d, params=BadParams())
    feed(rec, 0.0, 20.0)
    rec.record("stopAhead", 20.0, {"n": None, "f": 1.5, "b": False})
    feed(rec, 20.05, 26.0)
    assert len(folders(d)) == 1
    with open(os.path.join(d, folders(d)[0], "summary.txt")) as f:
      s = f.read()
    assert "route: \n" in s and "segment: \n" in s and "n: \n" in s and "f: 1.500" in s and "b: 0" in s
  rec = make("/nonexistent/x", clock=lambda: (_ for _ in ()).throw(RuntimeError("clock")))
  assert rec.open("x", 0.0, {}) is None
  rec.close(None, 1.0)
  rec.tick(2.0)
  rec.add_row(3.0, (1, 2, 3))


def test_locate_segment_from_realdata_layout():
  with tempfile.TemporaryDirectory() as d:
    route = "00000041--0123456789"
    for k in range(3):
      os.makedirs(os.path.join(d, f"{route}--{k}"))
      os.utime(os.path.join(d, f"{route}--{k}"), (1_000_000 + 60 * k, 1_000_000 + 60 * k))
    os.makedirs(os.path.join(d, "00000040--ffffffffff--7"))
    seg, secs, root = fe.locate_segment(route, 1_000_000 + 60 * 2 + 23.4, roots=(d, "/nonexistent"))
    assert (seg, root) == (2, d) and abs(secs - 23.4) < 1e-6
    assert fe.locate_segment("", 0.0, roots=(d,)) == (None, None, None)
    assert fe.locate_segment("00000099--0000000000", 0.0, roots=(d,)) == (None, None, None)


def test_near_miss_rate_limit_and_pruned_first():
  with tempfile.TemporaryDirectory() as d:
    rec = make(d)
    feed(rec, 0.0, 20.0)
    assert rec.record("stopAhead_nearMiss", 20.0, {"blocked_by": "prob 0.78 < 0.85"})
    assert not rec.record("stopAhead_nearMiss", 25.0, {})            # within 20 s: skipped
    assert rec.record("stoppedLead_nearMiss", 25.0, {})               # other kind: fine
    assert rec.record("stopAhead", 26.0, {})                          # real events are not limited
    assert rec.record("stopAhead", 26.5, {})
    feed(rec, 20.05, 46.0)
    assert rec.record("stopAhead_nearMiss", 46.0, {})
    feed(rec, 46.05, 52.0)
    fs = folders(d)
    assert len(fs) == 5, fs
    with open(os.path.join(d, [f for f in fs if f.endswith("stopAhead_nearMiss")][0], "summary.txt")) as f:
      assert "blocked_by: prob 0.78 < 0.85" in f.read()
    # over the cap: near-misses go first, oldest first, then the oldest real event
    assert fe.prune(d, max_events=3) == 2
    assert not any("nearMiss" in f for f in folders(d)[:1]) and len(folders(d)) == 3
    assert fe.prune(d, max_events=2) == 1
    assert [f.endswith("_stopAhead") or f.endswith("_stopAhead_2") for f in folders(d)] == [True, True], folders(d)


def test_long_open_episode_is_written_anyway():
  with tempfile.TemporaryDirectory() as d:
    rec = make(d)
    feed(rec, 0.0, 10.0)
    rec.open("curveSlowdown", 10.0, {})
    feed(rec, 10.05, 10.0 + fe.MAX_OPEN_SECONDS + fe.POST_SECONDS + 2, hz=2)
    assert len(folders(d)) == 1
    with open(os.path.join(d, folders(d)[0], "summary.txt")) as f:
      assert "MAX_OPEN_SECONDS" in f.read()


if __name__ == "__main__":
  import sys
  failed = 0
  for name, fn in sorted(globals().items()):
    if name.startswith("test_"):
      try:
        fn()
        print("PASS", name)
      except Exception as e:  # noqa: BLE001
        failed += 1
        print("FAIL", name, repr(e))
  print(f"{failed} failed")
  sys.exit(1 if failed else 0)


def test_long_events_are_thinned_evenly():
  rows = [(i * 0.05, (i,)) for i in range(5000)]
  thin = fe.EventRecorder._thin(rows, 1200)
  assert len(thin) == 1200 and thin[0] == rows[0] and thin[-1] == rows[-1]
  gaps = {round(thin[i + 1][0] - thin[i][0], 2) for i in range(len(thin) - 1)}
  assert max(gaps) <= 0.25                      # evenly spread, no big holes


def test_threaded_writer_never_blocks_the_caller(tmp_path):
  # drive 3b: a 2,000-row map curve event written in one go froze card for 0.1-0.3 s. The caller now only
  # hands the event to one writer thread; tick() returns at once and the file still appears.
  import time as _t
  cols = [f"c{i}" for i in range(50)]
  rec = fe.EventRecorder(cols, hz=20, root=str(tmp_path), write_in_thread=True)
  t = 0.0
  ev = rec.open("mapCurve", t, {"a": 1})
  for k in range(2400):
    t = k * 0.05
    rec.add_row(t, [float(k)] * 50)
  rec.close(ev, t)
  worst = 0.0
  for _ in range(200):
    t += 0.05
    s = _t.perf_counter()
    rec.tick(t)
    worst = max(worst, _t.perf_counter() - s)
  assert worst < 0.01, worst
  for _ in range(400):
    if rec.written:
      break
    _t.sleep(0.05)
  assert rec.written == 1
  folder = os.listdir(tmp_path)[0]
  with open(os.path.join(tmp_path, folder, "signals.csv")) as f:
    n = sum(1 for _ in f) - 1
  assert n == fe.MAX_ROWS
