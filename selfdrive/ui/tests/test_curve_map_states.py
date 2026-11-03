"""The Map Build States dialog (Mazda tiles): its texts and tick handling, with pyray and the UI stack stubbed."""
import importlib
import sys
import types

import pytest


class MagicMock:
  """Enough of a stand-in for the drawing helpers: any attribute, callable, returns itself."""

  def __getattr__(self, name):
    return self

  def __call__(self, *args, **kwargs):
    return self

MODULE = "openpilot.selfdrive.ui.layouts.settings.starpilot.curve_map_states"
STUBBED = (
  "pyray",
  "openpilot.common.params",
  "openpilot.system.ui.lib.application",
  "openpilot.system.ui.lib.multilang",
  "openpilot.system.ui.lib.scroll_panel",
  "openpilot.system.ui.lib.text_measure",
  "openpilot.system.ui.widgets",
  "openpilot.system.ui.widgets.label",
  "openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid",
)


class FakeParams:
  def __init__(self, vals=None):
    self.vals = dict(vals or {})

  def get(self, key, encoding=None, **kw):
    return self.vals.get(key)

  def put(self, key, value):
    self.vals[key] = value


class StubWidget:
  def __init__(self):
    self._touch_valid_callback = None

  def set_touch_valid_callback(self, cb):
    self._touch_valid_callback = cb


def _stub(**attrs):
  m = types.ModuleType("stub")
  for k, v in attrs.items():
    setattr(m, k, v)
  return m


@pytest.fixture
def C():
  saved = {n: sys.modules.get(n) for n in STUBBED + (MODULE,)}
  rl = MagicMock()
  rl.Rectangle = lambda x, y, w, h: types.SimpleNamespace(x=x, y=y, width=w, height=h)
  rl.check_collision_point_rec = lambda p, r: r.x <= p.x <= r.x + r.width and r.y <= p.y <= r.y + r.height
  sys.modules["pyray"] = rl
  sys.modules["openpilot.common.params"] = _stub(Params=FakeParams)
  sys.modules["openpilot.system.ui.lib.application"] = _stub(FontWeight=MagicMock(), gui_app=MagicMock())
  sys.modules["openpilot.system.ui.lib.multilang"] = _stub(tr=lambda s: s)
  sys.modules["openpilot.system.ui.lib.scroll_panel"] = _stub(GuiScrollPanel=MagicMock)
  sys.modules["openpilot.system.ui.lib.text_measure"] = _stub(measure_text_cached=MagicMock())
  sys.modules["openpilot.system.ui.widgets"] = _stub(Widget=StubWidget)
  sys.modules["openpilot.system.ui.widgets.label"] = _stub(gui_text_box=MagicMock())
  sys.modules["openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid"] = _stub(
    AetherListColors=MagicMock(), DEFAULT_PANEL_STYLE=MagicMock(), aether_begin_scissor_mode=MagicMock(),
    aether_end_scissor_mode=MagicMock(), draw_rounded_fill=MagicMock(), draw_rounded_stroke=MagicMock(),
    draw_selection_list_row=MagicMock(), snap_rect=lambda r: r, with_alpha=lambda c, a: c)
  sys.modules.pop(MODULE, None)
  mod = importlib.import_module(MODULE)
  yield mod
  for n, m in saved.items():
    if m is None:
      sys.modules.pop(n, None)
    else:
      sys.modules[n] = m


def test_texts(C):
  assert C.region_line(9, 3, True, True) == "3 of 9 areas built · whole state, always"
  assert C.region_line(9, 9, True, False) == "Built — 9 areas · only within range"
  assert C.region_line(1, 1, True, False) == "Built — 1 area · only within range"
  assert C.region_line(0, 0, True, True) == "Map not downloaded"
  assert C.region_line(0, 0, False, True).startswith("Not on the device's map")
  assert C.tile_value([], "") == "None" and C.tile_value(["us_state.MI"], "") == "Michigan"
  assert C.tile_value(["us_state.MI", "us_state.IL", "us_state.WA"], "Curve files up to date: ...") == "Michigan + 2"
  assert C.tile_value(["us_state.MI"], "Building area 4 of 9 (now Illinois), about 8 min left (avg 1.4 min per area)") == "Area 4 of 9 (now Illinois)"
  assert C.token_label("nation.US") == "Whole U.S." and C.token_label("us_state.ZZ") == "ZZ" and C.token_label("nation.CA") == "Canada"
  assert C.listed_tokens(["us_state.IL", "us_state.MI"], ["us_state.WA", "us_state.MI"]) == ["us_state.IL", "us_state.MI", "us_state.WA"]
  assert C.tick_summary([], ["us_state.IL"]).startswith("No state ticked")
  assert C.tick_summary(["us_state.MI"], ["us_state.IL", "us_state.MI"]).startswith("1 of 2 ticked")
  assert C.parse_tokens(b"us_state.MI, us_state.IL,,us_state.MI") == ["us_state.MI", "us_state.IL"]


def test_dialog_ticks_save_at_once_in_the_selection_order(C):
  params = FakeParams({C.SELECTED_PARAM: "us_state.IL,us_state.MI,us_state.WI", C.TICKED_PARAM: "us_state.WI"})
  d = C.CurveMapStatesDialog(params, FakeParams(), scan=lambda tokens: dict.fromkeys(tokens, (2, 1, True)))
  assert d._ticked == ["us_state.WI"]
  d._toggle("us_state.IL")
  assert params.vals[C.TICKED_PARAM] == "us_state.IL,us_state.WI"          # kept in the list's order
  d._toggle("us_state.WI")
  assert params.vals[C.TICKED_PARAM] == "us_state.IL"
  # a ticked state no longer in the map selection is listed (so it can be unticked) and saved
  params.vals[C.SELECTED_PARAM] = "us_state.IL"
  d = C.CurveMapStatesDialog(params, FakeParams(), scan=lambda tokens: {})
  d._ticked = ["us_state.MI"]
  assert C.listed_tokens(d._selected, d._ticked) == ["us_state.IL", "us_state.MI"]
  d._toggle("us_state.MI")
  assert params.vals[C.TICKED_PARAM] == ""


def test_dialog_scans_in_the_background_and_reads_the_status(C):
  params = FakeParams({C.SELECTED_PARAM: "us_state.IL,us_state.MI"})
  mem = FakeParams({C.STATUS_PARAM: "Building area 4 of 9 (now Illinois), about 8 min left (avg 1.4 min per area)"})
  seen = []

  def scan(tokens):
    seen.append(list(tokens))
    return dict.fromkeys(tokens, (9, 3, True))
  d = C.CurveMapStatesDialog(params, mem, scan=scan)
  d._refresh()
  d._scan_thread.join(2.0)
  assert seen == [["us_state.IL", "us_state.MI"]] and d._region_info["us_state.MI"] == (9, 3, True)
  assert d._status_text.startswith("Building area 4 of 9 (now Illinois)")
  d._refresh()                                                                 # too soon for another scan
  assert len(seen) == 1
