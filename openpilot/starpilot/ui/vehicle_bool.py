"""Confirmation text for parked vehicle preferences."""

VEHICLE_BOOL_KEYS = frozenset(("AlphaLongitudinalEnabled", "SubaruAvhStartup", "SubaruStopStartOff", "SubaruSNG", "SubaruRedneckCruise",
                              "GMPedalLongitudinal", "DisableOpenpilotLongitudinal", "LongPitch", "VoltSNG", "SNGHack", "GMAutoHold", "VoltOnePedalMode",
                              "TeslaCoopSteering", "TeslaAOLScreenTap", "TeslaAOLDisengageOnBrake", "TorqueInterceptorEnabled",
                              "mazda_speed:enabled", "mazda_speed:auto_resume", "mazda_speed:ti_steer_700",
                              "RemoteStartBootsComma", "HKGRemoteStartBootsComma", "IgnoreIgnitionLine", "TeslaWakeOnCAN"))


def confirmation_question(request):
  if request.value not in ("On", "Off"):
    raise ValueError("Unsupported vehicle preference value")
  verb = "Enable" if request.value == "On" else "Disable"
  from openpilot.starpilot.ui.panda_wake_feature import LABELS
  if request.key in LABELS:
    return f"{verb} {LABELS[request.key][0].lower()}? This changes Panda firmware after the next device restart."
  if request.key == "AlphaLongitudinalEnabled":
    return (f"{verb} Alpha Longitudinal after the next startup? StarPilot will request speed control instead of factory ACC. " +
            "Automatic Emergency Braking may be disabled; exact startup checks still apply.")
  if request.key == "GMPedalLongitudinal":
    return (f"{verb} Pedal Speed Control after the next startup? " +
            "A compatible connected interceptor must be detected before StarPilot can control it.")
  if request.key == "DisableOpenpilotLongitudinal":
    verb = "Turn off" if request.value == "On" else "Turn on"
    return f"{verb} StarPilot speed control after the next startup? Steering stays unchanged."
  if request.key == "SubaruAvhStartup":
    return f"{verb} AVH ON at the next startup? A bounded request runs once in Park with the engine running."
  if request.key == "SubaruStopStartOff":
    return f"{verb} Stop/Start OFF at the next startup? A bounded request runs once while stationary in Park or Neutral."
  if request.key == "SubaruSNG":
    return f"{verb} Subaru stop-and-go assistance after the next startup? Factory ACC retains speed control."
  if request.key == "SubaruRedneckCruise":
    return f"{verb} software cruise after the next startup? Stock ACC keeps pedal control."
  if request.key == "SNGHack":
    return f"{verb} Toyota stop-and-go assistance after the next startup?"
  if request.key == "VoltSNG":
    return f"{verb} Volt stop-and-go assistance after the next startup?"
  if request.key == "GMAutoHold":
    from opendbc.car.gm.values import CAR
    release = "gas" if request.vehicle_fingerprint == CAR.BUICK_LACROSSE else "gas or regen paddle"
    return f"{verb} Automatic Brake Hold after the next startup? Press the {release} to release a hold."
  if request.key == "VoltOnePedalMode":
    return f"{verb} Volt one-pedal driving after the next startup? Lifting off the accelerator can apply the brakes with cruise disengaged and cruise main on."
  if request.key == "LongPitch":
    return f"{verb} compensation for road grade? This refreshes during the drive."
  if request.key == "TorqueInterceptorEnabled":
    return (f"{verb} the Mazda Torque Interceptor after the next startup? Turn this on only if a MoreTorque " +
            "Torque Interceptor is installed.")
  if request.key == "mazda_speed:enabled":
    return (f"{verb} Mazda speed control at the next drive? StarPilot then presses the cruise SET+ and SET- buttons " +
            "to slow for curves. Your own button presses, the brake and the gas always win.")
  if request.key == "mazda_speed:auto_resume":
    return (f"{verb} the gas auto-resume at the next drive? After your brake turns the cruise off, your gas press " +
            "makes StarPilot press RES, so the cruise comes back at your set speed. It changes a Panda safety setting " +
            "when the car starts.")
  if request.key == "mazda_speed:ti_steer_700":
    return (f"{verb} the steering limit test at the next drive? This is a test: with the Torque Interceptor it raises " +
            "the maximum steering command from 600 to 700. Turn it off if a steering fault appears.")
  if request.key == "TeslaCoopSteering":
    return f"{verb} Tesla cooperative steering after the next startup? Light steering input adjusts the assisted steering angle within its limits."
  if request.key == "TeslaAOLScreenTap":
    return f"{verb} three-finger screen taps for Always On Lateral after the next startup? Cruise control stays independent."
  if request.key == "TeslaAOLDisengageOnBrake":
    return f"{verb} disarming Always On Lateral with the brake after the next startup? This applies with either Tesla harness."
  raise ValueError("Unsupported vehicle preference")
