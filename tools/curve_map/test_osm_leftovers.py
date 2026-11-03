import importlib.util
import os

_spec = importlib.util.spec_from_file_location("osm_leftovers", os.path.join(os.path.dirname(__file__), "osm_leftovers.py"))
L = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(L)


def _make(tmp_path, folders, selection):
  root = tmp_path / "offline"
  for f in folders:
    d = root / f
    d.mkdir(parents=True)
    (d / "tile").write_bytes(b"x" * 1000)
  param = tmp_path / "MapsSelected"
  param.write_text(selection)
  return str(root), str(param)


def test_lists_then_deletes_only_folders_outside_the_selection(tmp_path):
  folders = ["42/-84", "42/-86", "40/-74", "26/-82"]          # Michigan x2, New York, Florida
  root, param = _make(tmp_path, folders, "us_state.MI")
  lines = []
  assert L.main([], root=root, param_path=param, out=lines.append) == 0
  assert sorted(os.listdir(os.path.join(root, "40"))) == ["-74"]   # list only: nothing removed
  assert any("2 do not" in s for s in lines), lines
  assert L.main(["--delete"], root=root, param_path=param, out=lines.append) == 0
  left = sorted(f"{la}/{lo}" for la in os.listdir(root) for lo in os.listdir(os.path.join(root, la)))
  assert left == ["42/-84", "42/-86"]
  assert not os.path.exists(os.path.join(root, "26"))               # empty latitude folder gone too


def test_refuses_with_no_selection_or_an_unknown_region(tmp_path):
  for selection in ("", "us_state.MI,nation.XX"):
    root, param = _make(tmp_path / (selection or "empty"), ["42/-84", "40/-74"], selection)
    assert L.main(["--delete"], root=root, param_path=param, out=lambda s: None) == 1
    assert os.path.isdir(os.path.join(root, "40", "-74"))


def test_keeps_a_folder_that_only_touches_a_state_line(tmp_path):
  # Indiana's eastern edge is at -84.78: the Ohio folder 40/-84 (lon -84..-82) does not overlap; 40/-86 does
  root, param = _make(tmp_path, ["40/-86", "40/-84", "40/-82"], "us_state.IN")
  L.main(["--delete"], root=root, param_path=param, out=lambda s: None)
  assert sorted(os.listdir(os.path.join(root, "40"))) == ["-86"]
