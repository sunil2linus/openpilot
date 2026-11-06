"""Reading the LiveTorqueParameters cache param (no openpilot runtime dependencies, so it can be unit tested)."""
from cereal import log


def read_live_torque_cache(msg_bytes):
  """(useParams, liveValid) from the LiveTorqueParameters param, else (False, False).

  torqued stores the whole log.Event (messaging.new_message("liveTorqueParameters").to_bytes()), so it must be read
  as an Event; read as a bare LiveTorqueParametersData the bits came from the Event header instead (useParams always
  False, liveValid always True here), so Force Auto Tune's "no auto tune" condition was decided by noise (found in
  ladelisandwich/openpilot 190f74d0)."""
  if not msg_bytes:
    return False, False
  try:
    with log.Event.from_bytes(msg_bytes) as evt:
      if evt.which() != "liveTorqueParameters":
        return False, False
      ltp = evt.liveTorqueParameters
      return bool(ltp.useParams), bool(ltp.liveValid)
  except Exception:
    return False, False
