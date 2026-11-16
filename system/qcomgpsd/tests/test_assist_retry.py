"""Round T, T1a: the background retry of a failed GPS assistance injection (assist_retry.py). Synthetic clock only."""
import os
import time

from openpilot.system.qcomgpsd import assist_retry as ar


class FakeClock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


class FakeEvent:
  """threading.Event stand-in whose wait() advances the fake clock (and can set itself at a time)."""
  def __init__(self, clock, set_at=None):
    self.clock, self.set_at, self.flag = clock, set_at, False

  def is_set(self):
    if self.set_at is not None and self.clock.t >= self.set_at:
      self.flag = True
    return self.flag

  def set(self):
    self.flag = True

  def wait(self, timeout=None):
    if self.set_at is not None and self.clock.t + timeout >= self.set_at > self.clock.t:
      self.clock.t = self.set_at
    else:
      self.clock.t += timeout
    return self.is_set()


def _file(tmp_path):
  p = tmp_path / "xtra.bin"
  p.write_bytes(b"x")
  return str(p)


def test_every_30_s_for_5_minutes_then_stops(tmp_path):
  clock = FakeClock()
  path = _file(tmp_path)
  times = []

  def inject(p):
    times.append(clock.t)
    return False
  st = ar.AssistRetry(clock.t)
  ar.retry_loop(st, FakeEvent(clock), inject, path=path, clock=clock)
  assert times == [100.0 + 30.0 * k for k in range(1, 11)]   # 30 .. 300 s after start
  assert st.done and not os.path.exists(path)                  # the kept data is cleaned up


def test_stops_at_first_success(tmp_path):
  clock = FakeClock()
  path = _file(tmp_path)
  calls = []

  def inject(p):
    calls.append(clock.t)
    return len(calls) == 2
  st = ar.AssistRetry(clock.t)
  ar.retry_loop(st, FakeEvent(clock), inject, path=path, clock=clock)
  assert len(calls) == 2 and st.done


def test_stops_when_a_fix_is_reported(tmp_path):
  clock = FakeClock()
  path = _file(tmp_path)
  calls = []
  st = ar.AssistRetry(clock.t)
  ar.retry_loop(st, FakeEvent(clock, set_at=175.0), lambda p: calls.append(clock.t) or False, path=path, clock=clock)
  assert calls == [130.0, 160.0]       # the fix at 75 s: no attempt after it


def test_an_inject_that_raises_never_escapes(tmp_path):
  clock = FakeClock()
  path = _file(tmp_path)

  def inject(p):
    raise RuntimeError("modem gone")
  st = ar.AssistRetry(clock.t)
  ar.retry_loop(st, FakeEvent(clock), inject, path=path, clock=clock)   # no exception
  assert st.done


def test_keep_moves_the_file_and_starts_one_thread(tmp_path):
  src = _file(tmp_path)
  dst = str(tmp_path / "retry.bin")
  started = []
  r = ar.AssistRetryThread(0.0, inject=lambda p: False)
  r.fix()                               # a fix already: nothing kept, the data file removed
  assert r.keep(src, dst) is False and not os.path.exists(src) and not os.path.exists(dst)

  r = ar.AssistRetryThread(time.monotonic(), inject=lambda p: started.append(p) or False)   # the first try is 30 s away
  src = _file(tmp_path)
  assert r.keep(src, dst) is True and os.path.exists(dst) and not os.path.exists(src)
  t = r.thread
  src = _file(tmp_path)
  assert r.keep(src, dst) is True and r.thread is t      # the same thread
  r.fix()
  t.join(timeout=5)
  assert not t.is_alive() and started == []
