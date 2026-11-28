from opendbc.car import structs
from opendbc.car.mazda import ti_aol
from opendbc.car.mazda.stock_aol import aol_qualified
from openpilot.selfdrive.car.cruise import CRUISE_LONG_PRESS
from openpilot.starpilot.aol.intent import AolCardIntent, AOL_TOGGLE, ButtonType


# Torque Interceptor profile: the panda withdraws lateral at a standstill (vehicle_moving) and reports it in a native
# status read up to ~100 ms later. A denial observed while the car was stopped, or within this time after the stop
# ended, is that stop, not a rejection of the driver's latch (drive 00000002: 128.2 / 446.7 / 567.4 / 671.2 s, the
# latch was lost on the first moving carState). Intent only: the panda still decides every command.
STOP_NATIVE_GRACE_NS = 100_000_000


class MazdaStockCardIntent(AolCardIntent):
  observe_stock_engagement = True

  def __init__(self, cp, settings):
    super().__init__(settings, explicit_latch=True)
    self.cp = cp
    self.main_seen = False
    self.startup_main_pending = True
    self.startup_main_source_ns = 0
    self.startup_native_ready = False
    self.startup_binding = None
    self.main_available = False
    self.main_neutral = False
    self.main_pulse = False
    self.cancel_held = False
    self.cancel_ticks = 0
    self.ti_stop_ns = 0

  @property
  def ti(self):
    # read at call time: card creates this intent before it adds the AOL bits to the safety param (card.py), so a value
    # cached in __init__ would be False for the whole drive
    return ti_aol.qualified(self.cp, marked_only=True)

  def stop_native_rejection(self, native_rejection_ns):
    """A native denial that belongs to a stop (TI profile only): not a rejection."""
    return bool(self.ti and native_rejection_ns and self.ti_stop_ns and
                native_rejection_ns <= self.ti_stop_ns + STOP_NATIVE_GRACE_NS)

  def observe_full_engagement(self, CS, **kwargs):
    if self.stop_native_rejection(kwargs.get('native_rejection_ns', 0)):
      kwargs['native_rejection_ns'] = 0
    return super().observe_full_engagement(CS, **kwargs)

  def observe_startup_readiness(self, sm, CS, *, native, panda_ready, now_ns, producer):
    from openpilot.starpilot.aol.intent import disarming_fault
    from openpilot.starpilot.aol.runtime import AXIS_MAX_AGE_NS
    from openpilot.starpilot.aol.transport_pause import _healthy_transport_native, availability_only

    self.startup_native_ready = False
    if not self.startup_main_pending or not aol_qualified(self.cp, marked_only=True):
      return
    if CS.steerFaultPermanent or CS.accFaulted:
      self.startup_main_pending = False
      return
    try:
      event_ns = int(sm.logMonoTime['onroadEvents'])
      events_current = bool(sm.seen['onroadEvents'] and sm.valid['onroadEvents'] and sm.alive['onroadEvents'] and
        0 < event_ns <= now_ns and now_ns - event_ns <= 1_500_000_000)
      events = sm['onroadEvents']
      faults = [event for event in events if disarming_fault([event], CS)]
      if events_current and faults and not availability_only(faults):
        self.startup_main_pending = False
        return
      axis = sm['aolAxisState']
      native = (_healthy_transport_native(sm, self.cp, CS, str(axis.sessionId), now_ns, native)
                if native is not None else None)
      self.startup_native_ready = bool(events_current and panda_ready and native is not None and native.pandaInventory and
        not native.requestedLateral and not native.lateralAllowed and not native.requestedLongitudinal and not native.longitudinalAllowed and
        all(not slot.controlsAllowed for slot in native.pandaInventory) and
        sm.seen['aolAxisState'] and sm.valid['aolAxisState'] and sm.alive['aolAxisState'] and
        int(sm.logMonoTime['aolAxisState']) == axis.observedMonoTime and
        axis.validUntilMonoTime - axis.observedMonoTime <= AXIS_MAX_AGE_NS and
        axis.qualified and axis.nativeAcknowledged and str(axis.faultReason) == 'none' and
        str(axis.sessionId) == native.axisSessionId and producer and axis.intentProducerSessionId == producer and
        not axis.desiredLateral and not axis.desiredLongitudinal and not axis.lateralActive and not axis.longitudinalActive and
        0 < axis.sourceCarStateMonoTime <= axis.observedMonoTime <= now_ns <= axis.validUntilMonoTime and
        now_ns - axis.sourceCarStateMonoTime <= AXIS_MAX_AGE_NS)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      self.startup_native_ready = False
    if self.startup_native_ready:
      binding = (str(axis.sessionId), producer, int(axis.intentIntegrityEpoch), native.pandaSerial,
                 tuple(slot.hardwareSerial for slot in native.pandaInventory))
      if binding != self.startup_binding:
        self.startup_main_source_ns = 0
        self.startup_binding = binding
    else:
      self.startup_binding = None
      self.startup_main_source_ns = 0

  def cancel_action(self, action):
    if action == AOL_TOGGLE and self.settings.enabled:
      self.allowed_latch = not self.allowed_latch
      if self.allowed_latch:
        self.pause_lateral = False
    else:
      self._perform(action)

  def update(self, CS, **kwargs):
    if self.ti and CS.standstill and CS.canValid and not CS.canTimeout:
      self.ti_stop_ns = kwargs.get('now_ns', 0)
    if self.stop_native_rejection(kwargs.get('native_rejection_ns', 0)):
      kwargs['native_rejection_ns'] = 0
    state = structs.CarState(**CS.to_dict())
    events = []
    for event in CS.buttonEvents:
      if event.type == ButtonType.cancel:
        if event.pressed:
          self.cancel_held = True
          self.cancel_ticks = 0
        else:
          if self.cancel_held and 0 < self.cancel_ticks < CRUISE_LONG_PRESS:
            self.cancel_action(self.settings.cancel_actions[0])
          self.cancel_held = False
          self.cancel_ticks = 0
      elif event.type in (ButtonType.mainCruise, ButtonType.lkas):
        action = self.settings.main_action if event.type == ButtonType.mainCruise else self.settings.lkas_action
        if action == AOL_TOGGLE:
          events.append(structs.CarState.ButtonEvent(**event.to_dict()))
        elif event.pressed:
          self._perform(action)
      else:
        events.append(structs.CarState.ButtonEvent(**event.to_dict()))
    available = bool(state.cruiseState.available)
    healthy = state.canValid and not state.canTimeout
    if healthy:
      if self.main_pulse:
        events.append(structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=False))
        self.main_pulse = False
      if not available:
        self.main_neutral = True
      elif self.main_seen and self.main_neutral and not self.main_available:
        events.append(structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=True))
        self.main_pulse = True
        self.main_neutral = False
      self.main_seen = True
      self.main_available = available
    else:
      self.cancel_held = False
      self.cancel_ticks = 0
      self.main_neutral = False
    startup_main = False
    if self.startup_main_pending and healthy:
      eligible = bool(available and not state.cruiseState.enabled and self.settings.enabled and
        self.main_action != AOL_TOGGLE and self.lkas_action != AOL_TOGGLE and
        not state.steerFaultPermanent and not state.accFaulted and
        not (self.cancel_held and any(self.settings.cancel_actions)))
      source_ns = kwargs.get('now_ns', 0)
      if not eligible:
        self.startup_main_pending = False
      elif self.startup_native_ready and kwargs.get('fault_active'):
        self.startup_main_pending = False
      elif self.startup_native_ready and self.startup_main_source_ns and source_ns > self.startup_main_source_ns:
        # The first valid sample initializes the transport feedback pipeline.
        # Confirm MAIN on a subsequent fresh sample before admitting startup intent.
        self.startup_main_pending = False
        startup_main = True
      elif self.startup_native_ready and source_ns > 0:
        self.startup_main_source_ns = source_ns
    state.buttonEvents = events
    super().update(state.as_reader(), **kwargs)
    if startup_main and not self._fault_inhibit:
      self.allowed_latch = True
      self._last_latch_edge_ns = kwargs.get('now_ns', 0)
    if not available:
      self.allowed_latch = False
    if self.cancel_held:
      self.cancel_ticks += 1
      if self.cancel_ticks == CRUISE_LONG_PRESS:
        self.cancel_action(self.settings.cancel_actions[1])
      elif self.cancel_ticks == CRUISE_LONG_PRESS * 5:
        self.cancel_action(self.settings.cancel_actions[2])
