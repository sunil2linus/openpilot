"""StarPilot (round T, T1a): retry a failed GPS assistance (XTRA) injection in the background.

Drive 77: "mmcli --location-inject-assistance-data" failed on all its attempts 21-23 s after start and qcomgpsd gave up
(the data file was deleted), so the first fix came only at 86 s. When the injection at setup fails the data file is kept
under ASSIST_RETRY_FILE and one background thread tries the injection again every ASSIST_RETRY_INTERVAL, for the first
ASSIST_RETRY_WINDOW after qcomgpsd started, and stops at the first success or once a fix is reported. The thread only runs
mmcli (a separate process, with a timeout); it never touches the diag port, the AT port or the main loop's state, and any
error is logged and swallowed."""
import os
import subprocess
import threading
import time

ASSIST_RETRY_FILE = '/tmp/xtra3grc.retry.bin'
ASSIST_RETRY_INTERVAL = 30.0   # s between attempts
ASSIST_RETRY_WINDOW = 300.0    # s after qcomgpsd started: no attempt after this
ASSIST_RETRY_TIMEOUT = 45.0    # s: the mmcli process is killed after this (mmcli's own --timeout is 30)


def inject_once(path):
  """One injection attempt (no retries, no sleeps). True when mmcli reported success."""
  cmd = ["mmcli", "-m", "any", "--timeout", "30", f"--location-inject-assistance-data={path}"]
  try:
    subprocess.run(cmd, check=True, capture_output=True, timeout=ASSIST_RETRY_TIMEOUT)
    return True
  except Exception:
    return False


class AssistRetry:
  """When the next attempt is due. Pure: the caller gives the time; fed by the thread below."""

  def __init__(self, start, interval=ASSIST_RETRY_INTERVAL, window=ASSIST_RETRY_WINDOW):
    self.start, self.interval, self.window = float(start), float(interval), float(window)
    self.next_t = self.start + self.interval
    self.done = False              # succeeded, a fix came, or the window ended
    self.attempts = 0

  def expired(self, now):
    return now > self.start + self.window

  def wait_time(self, now):
    """s until the next attempt is due (0: now), None: no more attempts."""
    if self.done or self.expired(now):
      self.done = True
      return None
    return max(self.next_t - now, 0.0)

  def attempted(self, now, ok):
    self.attempts += 1
    self.next_t = now + self.interval
    if ok:
      self.done = True


def retry_loop(state, fix_event, inject, path=ASSIST_RETRY_FILE, clock=time.monotonic, log=None):
  """Runs in the background thread. fix_event: set by the main loop at the first fix (also used to wait)."""
  try:
    while not fix_event.is_set():
      w = state.wait_time(clock())
      if w is None:
        break
      if w > 0 and fix_event.wait(timeout=w):
        break
      if fix_event.is_set() or state.expired(clock()) or not os.path.exists(path):
        break
      ok = bool(inject(path))
      state.attempted(clock(), ok)
      if log is not None:
        log(f"assistance data injection retry {state.attempts}: {'ok' if ok else 'failed'}")
  except Exception:
    pass
  finally:
    state.done = True
    try:
      if os.path.exists(path):
        os.remove(path)
    except OSError:
      pass


class AssistRetryThread:
  """Starts the retry thread at most once per qcomgpsd run."""

  def __init__(self, start, log=None, inject=inject_once):
    self.state = AssistRetry(start)
    self.fix_event = threading.Event()
    self.log, self.inject = log, inject
    self.thread = None

  def fix(self):
    self.fix_event.set()

  def keep(self, src, dst=ASSIST_RETRY_FILE):
    """The injection at setup failed: keep the data for the retries (moved, so the setup path sees it as used) and start
    the thread if it is not running. Never raises."""
    try:
      if self.fix_event.is_set() or self.state.done:
        os.remove(src)
        return False
      os.replace(src, dst)
      if self.thread is None:
        self.thread = threading.Thread(target=retry_loop, args=(self.state, self.fix_event, self.inject, dst),
                                       kwargs={"log": self.log}, daemon=True, name="assist_retry")
        self.thread.start()
      return True
    except Exception:
      return False
