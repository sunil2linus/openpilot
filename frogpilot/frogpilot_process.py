#!/usr/bin/env python3
import datetime
import time

from pathlib import Path

from cereal import messaging
from openpilot.common.realtime import DT_MDL, Priority, set_realtime_priority
from openpilot.common.time import system_time_valid

from openpilot.frogpilot.system.model_manager import ModelManager
from openpilot.frogpilot.assets.theme_manager import ThemeManager
from openpilot.frogpilot.common.frogpilot_api import FrogPilotAPI
from openpilot.frogpilot.selfdrive.car.toyota.door_control import lock_doors, send_door_command
from openpilot.frogpilot.selfdrive.navd.map_scheduler import update_maps
from openpilot.frogpilot.selfdrive.pandad.panda_flashing import flash_panda
from openpilot.frogpilot.system.reporting import capture_report
from openpilot.frogpilot.system.speed_limit_reader_models import SpeedLimitReaderModels
from openpilot.frogpilot.system.updated.update_helpers import update_openpilot
from openpilot.frogpilot.common import frogpilot_functions, frogpilot_paths, frogpilot_utilities, frogpilot_variables
from openpilot.frogpilot.controls.frogpilot_planner import FrogPilotPlanner
from openpilot.frogpilot.controls.lib.speed_limit_filler import upload_speed_limits
from openpilot.frogpilot.system.frogpilot_stats import send_stats
from openpilot.frogpilot.system.frogpilot_tracking import FrogPilotTracking

ASSET_CHECK_RATE = (1 / DT_MDL)

class FrogPilotRequests:
  def __init__(self, model_manager, theme_manager):
    self.model_manager = model_manager
    self.theme_manager = theme_manager

    self.the_pond_request_sock = messaging.sub_sock("thePondRequest")
    self.ui_request_sock = messaging.sub_sock("frogpilotUIRequest")

    self.downloading_models = False
    self.downloading_themes = False

    self.door_command = ""

    self.pending = {}
    self.request_times = {}

  def take(self, kind):
    pending = self.pending.pop(kind, None)
    if pending is None:
      return None

    self.request_times[kind], request = pending
    return request

  def send_door_command(self, lock):
    if send_door_command(lock):
      if lock:
        self.door_command = "locked"
      else:
        self.door_command = "unlocked"
    else:
      self.door_command = "failed"

  def update(self, sm):
    for msg in messaging.drain_sock(self.ui_request_sock):
      self.pending[msg.frogpilotUIRequest.which()] = (msg.logMonoTime, msg.frogpilotUIRequest)

    for msg in messaging.drain_sock(self.the_pond_request_sock):
      self.pending[msg.thePondRequest.request.which()] = (msg.logMonoTime, msg.thePondRequest.request)

    if not frogpilot_utilities.is_thread_alive("download_models"):
      if self.downloading_models:
        self.model_manager.download_lock.release()
        self.downloading_models = False

      if ("downloadAllModels" in self.pending or "downloadModels" in self.pending) and self.model_manager.download_lock.acquire(blocking=False):
        self.downloading_models = True

        request = self.take("downloadModels")
        if request is not None:
          models = list(request.downloadModels)
          finished_message = "Downloaded!"
        else:
          self.take("downloadAllModels")
          models = self.model_manager.missing_models()
          finished_message = "All models downloaded!"

        self.model_manager.download_state.cancelled = False
        self.model_manager.download_state.progress = "Downloading..."
        frogpilot_utilities.run_thread_with_lock("download_models", self.model_manager.download_models, (models, finished_message))

    if "downloadAllModels" not in self.pending and "downloadModels" not in self.pending and self.take("cancelModelDownload") is not None:
      self.model_manager.download_state.cancelled = True

    if sm["deviceState"].networkMetered and self.model_manager.downloading_automatically:
      self.model_manager.download_state.cancelled = True

    if not frogpilot_utilities.is_thread_alive("download_themes"):
      if self.downloading_themes:
        self.theme_manager.download_state.cancelled = False
        self.theme_manager.download_lock.release()
        self.downloading_themes = False

      if "downloadTheme" in self.pending and self.theme_manager.download_lock.acquire(blocking=False):
        self.downloading_themes = True

        request = self.take("downloadTheme")
        themes = [(theme.component, theme.theme) for theme in request.downloadTheme]

        self.theme_manager.download_count = len(themes)
        self.theme_manager.download_failed_count = 0
        self.theme_manager.download_success_count = 0
        self.theme_manager.download_state.progress = "Downloading..."
        frogpilot_utilities.run_thread_with_lock("download_themes", self.theme_manager.download_themes, (themes,))

    if "downloadTheme" not in self.pending and self.take("cancelThemeDownload") is not None:
      if frogpilot_utilities.is_thread_alive("download_themes"):
        self.theme_manager.download_state.cancelled = True

    if self.take("refreshTheme") is not None:
      self.theme_manager.previous_asset_mappings = {}
      self.theme_manager.theme_update_count += 1
      self.theme_manager.theme_updated = True

    if not frogpilot_utilities.is_thread_alive("send_door_command"):
      request = self.take("lockDoors")
      if request is not None:
        if sm["deviceState"].started or frogpilot_utilities.is_thread_alive("lock_doors"):
          self.door_command = "busy"
        else:
          self.door_command = "pending"
          frogpilot_utilities.run_thread_with_lock("send_door_command", self.send_door_command, (request.lockDoors,))

    return self.take("updateToggles") is not None

  def publish(self, pm, stats_saved_count):
    frogpilot_process_state_send = messaging.new_message("frogpilotProcessState")
    frogpilotProcessState = frogpilot_process_state_send.frogpilotProcessState

    frogpilotProcessState.doorCommand = self.door_command
    frogpilotProcessState.doorCommandRequestTime = self.request_times.get("lockDoors", 0)
    frogpilotProcessState.downloadingModels = frogpilot_utilities.is_thread_alive("download_models") or self.model_manager.downloading_automatically
    frogpilotProcessState.downloadThemeRequestTime = self.request_times.get("downloadTheme", 0)
    frogpilotProcessState.modelDownloadProgress = self.model_manager.download_state.progress
    frogpilotProcessState.modelDownloadRequestTime = max(self.request_times.get("downloadAllModels", 0), self.request_times.get("downloadModels", 0))
    frogpilotProcessState.statsSavedCount = stats_saved_count
    frogpilotProcessState.themeDownloadCount = self.theme_manager.download_count
    frogpilotProcessState.themeDownloadFailedCount = self.theme_manager.download_failed_count
    frogpilotProcessState.themeDownloadProgress = self.theme_manager.download_state.progress
    frogpilotProcessState.themeDownloadSuccessCount = self.theme_manager.download_success_count

    pm.send("frogpilotProcessState", frogpilot_process_state_send)

def serve_settings_request(key, function):
  path = Path(frogpilot_variables.params_memory.get(key, encoding="utf-8"))

  try:
    function(path)
  except Exception:
    frogpilot_variables.params_memory.put_bool(key, False)
    raise

  frogpilot_variables.params_memory.remove(key)

def assets_checks(time_validated, frogpilot_toggles, frogpilot_api):
  if frogpilot_variables.params_memory.get_bool("FlashPanda"):
    frogpilot_utilities.run_thread_with_lock("flash_panda", flash_panda)

  if frogpilot_variables.params_memory.get("BackupSettings") not in (None, b"0"):
    frogpilot_utilities.run_thread_with_lock("backup_settings", serve_settings_request, ("BackupSettings", frogpilot_functions.backup_settings))

  if frogpilot_variables.params_memory.get("RestoreSettings") not in (None, b"0"):
    frogpilot_utilities.run_thread_with_lock("restore_settings", serve_settings_request, ("RestoreSettings", frogpilot_functions.restore_settings))

  issue_reported = frogpilot_variables.params_memory.get("IssueReported", encoding="utf-8")
  if issue_reported is not None and time_validated:
    frogpilot_utilities.run_thread_with_lock("capture_report", capture_report, (issue_reported, dict(vars(frogpilot_toggles)), frogpilot_api))

def update_checks(model_manager, now, speed_limit_reader_models, theme_manager, sm, frogpilot_toggles, boot_run=False):
  while not (frogpilot_utilities.is_url_pingable("https://github.com") or frogpilot_utilities.is_url_pingable("https://gitlab.com")):
    time.sleep(60)

  with model_manager.download_lock:
    model_manager.update_models(not sm["deviceState"].networkMetered and frogpilot_toggles.automatically_download_models, sm["deviceState"].started)

  if frogpilot_toggles.show_speed_limits or frogpilot_toggles.speed_limit_controller or frogpilot_toggles.speed_limit_filler:
    speed_limit_reader_models.update(sm)

  if not sm["deviceState"].networkMetered:
    with theme_manager.download_lock:
      theme_manager.update_themes(boot_run)

  if not sm["deviceState"].networkMetered:
    frogpilot_utilities.run_thread_with_lock("update_maps", update_maps, (now,))

  if not sm["deviceState"].networkMetered and frogpilot_toggles.automatic_updates:
    frogpilot_utilities.run_thread_with_lock("update_openpilot", update_openpilot)

def update_toggles(variables, started, model_manager, theme_manager, time_validated):
  frogpilot_toggles = variables.frogpilot_toggles

  previous_holiday_themes = frogpilot_toggles.holiday_themes
  previous_random_themes = frogpilot_toggles.random_themes

  theme_manager.theme_updated = False

  if not started and frogpilot_variables.params.get_bool("ModelRandomizer"):
    model_manager.randomize_model(new_drive=False)

  variables.update(theme_manager.holiday_theme, started)

  if not started:
    frogpilot_functions.sync_flag_file(frogpilot_paths.HD_PATH, frogpilot_toggles.use_higher_bitrate)
    frogpilot_functions.sync_flag_file(frogpilot_paths.KONIK_PATH, frogpilot_toggles.use_konik_server)

  randomize_theme = frogpilot_toggles.holiday_themes != previous_holiday_themes
  randomize_theme |= frogpilot_toggles.random_themes != previous_random_themes

  theme_manager.update_active_theme(time_validated, frogpilot_toggles, randomize_theme=randomize_theme)

  if time_validated:
    frogpilot_utilities.run_thread_with_lock("backup_toggles", frogpilot_functions.backup_toggles, (frogpilot_variables.params_cache,), report=False)

def frogpilot_thread():
  set_realtime_priority(Priority.CTRL_LOW)

  frogpilot_paths.ERROR_FILE_PATH.unlink(missing_ok=True)

  frogpilot_api = FrogPilotAPI(frogpilot_variables.params)
  variables = frogpilot_variables.FrogPilotVariables()
  model_manager = ModelManager()
  speed_limit_reader_models = SpeedLimitReaderModels(frogpilot_variables.params)
  theme_manager = ThemeManager(frogpilot_variables.params)

  frogpilot_requests = FrogPilotRequests(model_manager, theme_manager)

  frogpilot_toggles = variables.frogpilot_toggles

  pm = messaging.PubMaster(["frogpilotPlan", "frogpilotProcessState"])
  sm = messaging.SubMaster(["carControl", "carState", "controlsState", "deviceState",
                            "liveLocationKalman", "liveParameters", "longitudinalPlan", "modelV2",
                            "radarState", "frogpilotCarParams", "frogpilotCarState", "frogpilotControlsState",
                            "frogpilotDeviceState", "frogpilotModelV2", "frogpilotNavigation", "frogpilotSignReading",
                            "navInstruction"],
                            poll="modelV2")

  manual_update_previously = False
  run_update_checks = False
  share_speed_limits = True
  started_previously = False
  time_validated = False
  waiting_for_car_params = False

  force_onroad_cleared_count = 0
  stats_saved_count = 0

  previous_update_checks_period = None

  while True:
    sm.update()

    now = datetime.datetime.now(datetime.timezone.utc)

    started = sm["deviceState"].started

    if started and not started_previously:
      frogpilot_paths.ERROR_FILE_PATH.unlink(missing_ok=True)

      frogpilot_planner = FrogPilotPlanner(theme_manager, frogpilot_api)
      frogpilot_tracking = FrogPilotTracking(frogpilot_planner, frogpilot_toggles)

      waiting_for_car_params = True

    elif not started and started_previously:
      run_update_checks = True

      frogpilot_planner.frogpilot_vcruise.slc.close()
      frogpilot_planner.frogpilot_weather.close()

      frogpilot_planner.speed_limit_filler.save()

      share_speed_limits = True

      frogpilot_tracking.save_stats(blocking=True)
      stats_saved_count += 1

      if frogpilot_toggles.model_randomizer:
        model_manager.randomize_model(new_drive=True)

      variables.update(theme_manager.holiday_theme, started)

      frogpilot_functions.sync_flag_file(frogpilot_paths.HD_PATH, frogpilot_toggles.use_higher_bitrate)
      frogpilot_functions.sync_flag_file(frogpilot_paths.KONIK_PATH, frogpilot_toggles.use_konik_server)

      if frogpilot_toggles.lock_doors_timer:
        frogpilot_utilities.run_thread_with_lock("lock_doors", lock_doors, (frogpilot_toggles.lock_doors_timer,), report=False)

      theme_manager.restore_wheel_image()

      if frogpilot_toggles.random_themes:
        theme_manager.update_active_theme(time_validated, frogpilot_toggles, randomize_theme=True)

      if time_validated:
        frogpilot_utilities.run_thread_with_lock("send_stats", send_stats, (frogpilot_variables.params, frogpilot_toggles, frogpilot_api))

    if theme_manager.theme_updated:
      update_toggles(variables, started, model_manager, theme_manager, time_validated)

    if started and sm.updated["modelV2"]:
      frogpilot_planner.update(now, time_validated, sm, frogpilot_toggles)
      frogpilot_planner.publish(theme_manager.theme_update_count, sm, pm, frogpilot_toggles, variables.toggles_update_time)

      frogpilot_tracking.update(now, time_validated, sm)
    elif not started:
      frogpilot_plan_send = messaging.new_message("frogpilotPlan")
      frogpilot_plan_send.frogpilotPlan.themeUpdateCount = theme_manager.theme_update_count
      frogpilot_plan_send.frogpilotPlan.togglesUpdateTime = variables.toggles_update_time
      pm.send("frogpilotPlan", frogpilot_plan_send)

    started_previously = started

    if share_speed_limits and not started and frogpilot_utilities.on_unmetered_network(sm["deviceState"]):
      if frogpilot_toggles.speed_limit_filler and frogpilot_toggles.frogpilot_telemetry and not frogpilot_toggles.uploads_disabled:
        frogpilot_utilities.run_thread_with_lock("upload_speed_limits", upload_speed_limits, (frogpilot_variables.params, frogpilot_api))

      share_speed_limits = False

    if sm.frame % ASSET_CHECK_RATE == 0:
      assets_checks(time_validated, frogpilot_toggles, frogpilot_api)

    toggles_updated = frogpilot_requests.update(sm)
    frogpilot_requests.publish(pm, stats_saved_count)

    toggles_updated |= sm["frogpilotDeviceState"].forceOnroadClearedCount > force_onroad_cleared_count
    toggles_updated |= waiting_for_car_params and sm.updated["frogpilotCarParams"]

    force_onroad_cleared_count = sm["frogpilotDeviceState"].forceOnroadClearedCount
    waiting_for_car_params &= not sm.updated["frogpilotCarParams"]

    if toggles_updated:
      update_toggles(variables, started, model_manager, theme_manager, time_validated)

    manual_update = frogpilot_variables.params_memory.get("ManualUpdateInitiated") is not None
    run_update_checks |= manual_update and not manual_update_previously
    manual_update_previously = manual_update

    if frogpilot_utilities.frogs_go_moo():
      update_checks_period = now.minute // 5
    else:
      update_checks_period = now.hour

    run_update_checks |= update_checks_period != previous_update_checks_period
    run_update_checks &= time_validated

    previous_update_checks_period = update_checks_period

    if run_update_checks:
      theme_manager.update_active_theme(time_validated, frogpilot_toggles)
      frogpilot_utilities.run_thread_with_lock("update_checks", update_checks, (model_manager, now, speed_limit_reader_models, theme_manager, sm, frogpilot_toggles))

      run_update_checks = False
    elif not time_validated:
      time_validated = system_time_valid()
      if not time_validated:
        continue

      theme_manager.update_active_theme(time_validated, frogpilot_toggles)
      frogpilot_utilities.run_thread_with_lock("update_checks", update_checks, (model_manager, now, speed_limit_reader_models, theme_manager, sm, frogpilot_toggles, True))

      frogpilot_utilities.run_thread_with_lock("backup_toggles", frogpilot_functions.backup_toggles, (frogpilot_variables.params_cache,), report=False)
      frogpilot_utilities.run_thread_with_lock("send_stats", send_stats, (frogpilot_variables.params, frogpilot_toggles, frogpilot_api))

def main():
  frogpilot_thread()

if __name__ == "__main__":
  main()
