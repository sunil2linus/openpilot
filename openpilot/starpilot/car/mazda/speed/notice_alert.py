"""
The selfdrived side of the Mazda speed control's notices (part H): turns each new notice card wrote
(notices.NOTICE_PATH) into an ordinary Alert for selfdrived's AlertManager, so the text and the sound travel in
selfdriveState as always (no capnp change; the same way 7.0 adds its blindspot / departure advisories).

  * make(CP, params) -> None (the hook is inert) unless a Mazda with the speed control on, notices on and at least one
    feature Test or On. Read once when selfdrived starts (the setting applies at the next drive, like card's).
  * poll(current) at most every POLL_EVERY: one os.stat; the file is read and parsed only when its mtime / size
    changed. Anything wrong (missing, too big, not JSON, an unknown field or kind, a wrong type, text too long, past
    its "until") -> no alert. A new (sid, seq) becomes ONE alert; the AlertManager then keeps it for its duration.
  * Low priority (Priority.LOWER: every stock alert of LOW or above wins) and, on top, never put up while any other
    alert is on screen: it waits for a free screen until its "until" (a stock alert never gets covered).
  * Sound: an existing AudibleAlert only (prompt), only when the notice asks for it AND notice_sound is on.
Never raises.
"""
import json
import math
import os
import time

from openpilot.starpilot.car.mazda.speed import notices as N

ALERT_TYPE = "mazdaSpeedNotice/notice"
EVENT_TYPE = "mazdaSpeedNotice"
POLL_EVERY = 0.1               # s (10 Hz)
MAX_BYTES = 1024
FIELDS = frozenset(("v", "sid", "seq", "kind", "text1", "text2", "sound", "until"))
DURATION = {"tagged": 2.0}     # s on screen; warnings WARN_DURATION, the rest INFO_DURATION
WARN_DURATION = 4.0
INFO_DURATION = 3.0


def make(CP, params):
  """The reader for this drive, or None (not a Mazda / the feature or its notices off / any error)."""
  try:
    if CP is None or CP.brand != "mazda":
      return None
    from openpilot.starpilot.car.mazda.speed import config as C
    cfg = C.read(params)
    if not cfg.shows_notices:
      return None
    return NoticeReader(sound=cfg.notice_sound)
  except Exception:
    return None


def parse(raw, now):
  """bytes -> the notice dict, or None when it is not exactly valid (or already past its "until")."""
  try:
    if len(raw) > MAX_BYTES:
      return None
    doc = json.loads(raw)
  except (ValueError, TypeError, RecursionError):
    return None
  if type(doc) is not dict or set(doc) != FIELDS or doc["v"] != N.VERSION or type(doc["v"]) is not int:
    return None
  if (type(doc["sid"]) is not str or not 0 < len(doc["sid"]) <= 32 or type(doc["seq"]) is not int or doc["seq"] <= 0 or
      doc["kind"] not in N.KINDS or type(doc["text1"]) is not str or type(doc["text2"]) is not str or
      not 0 < len(doc["text1"]) <= 48 or len(doc["text2"]) > 64 or type(doc["sound"]) is not bool or
      type(doc["until"]) not in (int, float) or not math.isfinite(doc["until"])):
    return None
  if doc["until"] < now:
    return None
  return doc


class NoticeReader:
  def __init__(self, path=N.NOTICE_PATH, sound=False, clock=time.monotonic):
    self.path = path
    self.sound = bool(sound)
    self.clock = clock
    self.next_poll = -1e9
    self.stamp = None              # (mtime_ns, size) of the file last read
    self.seen = None               # (sid, seq) of the last notice taken
    self.pending = None            # the notice waiting for a free screen
    self.made = 0

  def _read(self, now):
    try:
      st = os.stat(self.path)
    except OSError:
      return
    stamp = (st.st_mtime_ns, st.st_size)
    if stamp == self.stamp:
      return
    self.stamp = stamp
    try:
      with open(self.path, "rb") as f:
        raw = f.read(MAX_BYTES + 1)
    except OSError:
      return
    doc = parse(raw, now)
    if doc is None:
      return
    key = (doc["sid"], doc["seq"])
    if self.seen is not None and key[0] == self.seen[0] and key[1] <= self.seen[1]:
      return
    self.seen = key
    self.pending = doc

  def poll(self, current=None, now=None):
    """[] or [the Alert] for selfdrived's update_alerts. current: the AlertManager's current alert (last frame)."""
    try:
      now = self.clock() if now is None else now
      if now >= self.next_poll:
        self.next_poll = now + POLL_EVERY
        self._read(now)
      doc = self.pending
      if doc is None:
        return []
      if doc["until"] < now:
        self.pending = None
        return []
      busy = current is not None and getattr(current, "alert_type", "") not in ("", ALERT_TYPE)
      if busy:
        return []                  # never over a stock alert: wait for a free screen (until its "until")
      self.pending = None
      self.made += 1
      return [self.alert(doc)]
    except Exception:
      return []

  def alert(self, doc):
    from openpilot.selfdrive.selfdrived.events import Alert, AlertSize, AlertStatus, AudibleAlert, Priority, VisualAlert
    warn = doc["kind"] in N.WARNINGS
    duration = DURATION.get(doc["kind"], WARN_DURATION if warn else INFO_DURATION)
    sound = AudibleAlert.prompt if (doc["sound"] and self.sound) else AudibleAlert.none
    alert = Alert(doc["text1"], doc["text2"], AlertStatus.userPrompt if warn else AlertStatus.normal,
                  AlertSize.mid if doc["text2"] else AlertSize.small, Priority.LOWER, VisualAlert.none, sound, duration)
    alert.alert_type, alert.event_type = ALERT_TYPE, EVENT_TYPE
    return alert
