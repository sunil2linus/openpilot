"""
One rule for every reader of the map view (map_view.py, mazda_mapviewd): is the view alive, and does this frame count
as evidence? Pure (no imports; tests/test_view_age.py).

The view's "t" is the GPS sample's time minus GPS_LAG (the position is that old when the message arrives), so
`now - t` starts at GPS_LAG already. The age here is counted from the GPS message: age = now - (t + GPS_LAG).
The GPS rate differs by device: 1 Hz on the comma 3X / tizi (qcomgpsd, e.g. the Konik A1M), up to 10 Hz with ubloxd.

  * CURRENT: the newest view, within one GPS period of its message (the period seen between views + SPAN_SLACK for
    the 10 Hz file read and jitter, at most VIEW_SPAN). Evidence timers (confirmations, steady times) grow only here.
  * LATE: older than that but at most VIEW_ALIVE: a fix was missed. The view is still alive: what it said holds (the
    road, the trust, the states built on it); places are followed by the distance driven; evidence timers neither
    grow nor restart.
  * GONE: older than VIEW_ALIVE, no view, or no GPS position ("gps": "none"): exactly the old "no GPS".

VIEW_ALIVE tolerates one missed 1 Hz fix and still finds a stopped mazda_mapviewd or a lost GPS within ~2.5-3.4 s (the
old fork: 3 s); mazda_mapviewd writes "none" after the same time without a fix (map_view.GPS_STALE).
A NEW view (its "t" changed) is the evidence step: a confirmation also needs VIEWS_MIN different views saying it
(ViewCount), so one 1 Hz view alone never confirms anything.
"""

GPS_LAG = 0.85                # s: the position is this old when its GPS message arrives (0.7-1.0 s measured)
VIEW_ALIVE = 2.5              # s since the GPS message: an older view is no map (one missed 1 Hz fix is tolerated)
VIEW_SPAN = 1.5               # s since the GPS message: a view is CURRENT at most this long ...
SPAN_SLACK = 0.25             # s ... and at most one GPS period + this (the 10 Hz file read, jitter)
PERIOD_DEFAULT = 1.0          # s: the GPS period before two views were seen (the slowest GPS)
VIEWS_MIN = 2                 # different views a confirmation needs

GONE, LATE, CURRENT = 0, 1, 2


def view_age(view, now):
  """s since the GPS message of the view (None: no usable view)."""
  if view is None:
    return None
  t = view.get("t")
  if not isinstance(t, (int, float)):
    return None
  return now - (float(t) + GPS_LAG)


def view_live(view, now, span=VIEW_SPAN):
  """GONE / LATE / CURRENT (see the module doc). span: how long a view is CURRENT (ViewClock gives it per GPS rate)."""
  if view is None or view.get("gps") != "ok":
    return GONE
  age = view_age(view, now)
  if age is None or age > VIEW_ALIVE:
    return GONE
  return CURRENT if age <= span else LATE


def view_alive(view, now):
  return view_live(view, now) != GONE


class ViewClock:
  """One reader's view of the view: whether this frame brought a NEW view (`new`), the GPS period seen between views,
  and the view's state now (update). Calling update again in the same frame with the same view changes nothing."""

  def __init__(self):
    self.t = None                 # "t" of the newest view seen
    self.period = PERIOD_DEFAULT
    self.new = False

  @property
  def span(self):
    return min(VIEW_SPAN, self.period + SPAN_SLACK)

  def update(self, view, now):
    t = view.get("t") if view is not None and view.get("gps") == "ok" else None
    self.new = False
    if isinstance(t, (int, float)) and t != self.t:
      if self.t is not None and 0.0 < t - self.t <= VIEW_ALIVE:
        self.period = t - self.t
      self.t, self.new = t, True
    return view_live(view, now, self.span)


class ViewCount:
  """How many different views (by their "t") have said something in a row."""

  def __init__(self):
    self.n, self.t = 0, None

  def add(self, t):
    if t != self.t:
      self.n, self.t = self.n + 1, t

  def reset(self):
    self.n, self.t = 0, None

  def __bool__(self):
    return self.n >= VIEWS_MIN
