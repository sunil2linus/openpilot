"""starpilotSelfdriveState.alertSound is typed with the stock AudibleAlert enum, but StarPilot puts its own sounds
(random events, map and stop chimes) in it. str() on such a value raises ("Member was null"), which crashed the UI
on every Stop Ahead chime (drive 3e). The UI must only compare .raw values."""
import os
import re

from cereal import custom, messaging
from openpilot.common.basedir import BASEDIR

RENDERERS = ("selfdrive/ui/onroad/alert_renderer.py", "selfdrive/ui/mici/onroad/alert_renderer.py")


def test_a_starpilot_chime_is_outside_the_stock_enum_but_raw_works():
  msg = messaging.new_message("starpilotSelfdriveState")
  msg.starpilotSelfdriveState.alertSound = int(custom.StarPilotCarControl.HUDControl.AudibleAlert.stopChime)
  sound = msg.starpilotSelfdriveState.alertSound
  assert sound.raw == 25
  try:
    str(sound)
    raised = False
  except Exception:
    raised = True
  assert raised, "if str() works now, the schema changed: this test can be relaxed"


def test_the_ui_never_converts_an_alert_sound_to_text():
  text_forms = (r"(str|repr|format)\([^)]*alertSound(?!\.raw)", r"f[\"'][^\"']*\{[^}]*alertSound(?!\.raw)",
                r"alertSound\.to_dict", r"alertSound\)?\.name\b")
  for path in RENDERERS:
    src = open(os.path.join(BASEDIR, path)).read()
    for form in text_forms:
      assert not re.search(form, src), (path, form)
