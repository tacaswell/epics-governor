import time
import logging
import epics
import copy

from threading import Thread, Lock
from queue import Queue, Empty

class Target:
    """
    Represents a Target entity. A Target is part of a State: it has a name (which is a named position of a Device),
    limits and a flag 'updateAfter' if its value must be updated after leaving the State.
    """
    def __init__(self, config):
        self._config = config

    def __repr__(self):
        return "Target('{}', limits={}, updateAfter={})".format(self.target, self.limits, self.update_after)

    def get_target(self):
        return self._config['target']

    def get_limits(self):
        return tuple(self._config['limits'])

    def set_limits(self, new_limits):
        self._config['limits'] = new_limits

    def get_update_after(self):
        try:
            return self._config['updateAfter']
        except KeyError:
            return False

    target = property(get_target)
    limits = property(get_limits, set_limits)
    update_after = property(get_update_after)


class State:
    """
    Represents a State entity. A State has a name, a full name and a map of Devices to their desired Targets.
    """
    def __init__(self, name, config):
        self._name = name
        self._config = config
        self._targets = {device: Target(target) for device, target in config.get('targets', {}).items()}

    def __repr__(self):
        return "State({}[{}], targets={})".format(self.name, self.fullname, self.targets)

    def get_name(self):
        return self._name

    def get_fullname(self):
        return self._config.get('name', self._name)

    def get_targets(self):
        return self._targets

    name = property(get_name)
    fullname = property(get_fullname)
    targets = property(get_targets)

device_types_registry = {}


class MetaDevice(type):
    """ Automatic registry for new Device types """
    def __new__(meta, name, bases, class_dict):
        cls = type.__new__(meta,  name, bases, class_dict)
        device_types_registry[cls.__name__] = cls
        return cls


class Device(metaclass=MetaDevice):
    """
    Base class for all Device types. Actual devices of the type 'Device' act like dummies: all moves complete
    immediately and successfully, no hardware is touched.
    """
    REQUIRED_FIELDS = ('name', 'timeout')

    def __init__(self, name, config, governor, logger_name="Device"):
        self._name = name
        self._config = config
        self._governor = governor
        self._limits = None
        self._target = None
        self._homed = False

        self._logger_name = "{}.{}[{}]".format(governor.logger_name, logger_name, name)
        self._logger = logging.getLogger(self._logger_name)

    def __repr__(self):
        return "Device({}[{}], pv='{}' positions={})".format(self.name, self.fullname, self.pvname, self.positions)

    def move(self, target, wait=False):
        """
        Moves this Device to the desired target position.
        :param target: str|float
            A position name OR a position value
        :param wait: bool
            Whether to wait for the Device finish its move or not. Default: False.
        :return: True if the move was successful, False otherwise
        """
        self.target = None
        target_pos = self.positions.get(target.target, target.target)
        self._logger.info("Issued move to '%s' (%f)", target, target_pos)
        return self.wait() if wait else True

    def wait(self):
        """
        Waits for this Device to finish moving
        :return: True if the move was successful, False otherwise
        """
        return True

    def stop(self):
        """ Immediately stops the motion """
        self._logger.info("Issued STOP")

    @property
    def connected(self):
        return True

    @property
    def alarmed(self):
        return False

    @property
    def homed(self):
        return True

    @property
    def done(self):
        return True

    @property
    def name(self):
        return self._name

    @property
    def fullname(self):
        return self._config['name']

    @property
    def pvname(self):
        return self._config['pv']

    @property
    def positions(self):
        return self._config.get('positions', {})

    @property
    def target(self):
        return self._target

    @target.setter
    def target(self, target):
        self._logger.info("Target set to %s", target)
        self._target = target

    @property
    def target_pos(self):
        if self.target is None:
            return None
        return self.positions.get(self._target.target, self._target.target)

    @property
    def limits(self):
        return self._target.limits

    @property
    def timeout(self):
        return self._config['timeout']

    @property
    def val(self):
        return self.target_pos


class Motor(Device):
    """ A Motor device that represents a real motor. It communicates with an EPICS motor record. """
    REQUIRED_FIELDS = Device.REQUIRED_FIELDS + ('pv', 'tolerance', 'positions')

    def __init__(self, name, config, governor):
        super(Motor, self).__init__(name, config, governor, logger_name="Motor")

        self._val = epics.PV(self.pvname, connection_callback=self._connection_changed)
        self._rbv = epics.PV(self.pvname + ".RBV", callback=self._value_changed)
        self._dmov = epics.PV(self.pvname + ".DMOV")
        self._msta = epics.PV(self.pvname + ".MSTA", callback=self._status_changed)
        self._stop = epics.PV(self.pvname + ".STOP")

    def __repr__(self):
        return "Motor({}[{}], pv='{}' positions={})".format(self.name, self.fullname, self.pvname, self.positions)

    def _connection_changed(self, **kwargs):
        """ Emits a disconnect event to the Governor. Called by pcaspy. """
        if not kwargs['conn']:
            self._logger.warn('disconnected')
            self._governor.device_event(self._governor.DISCONNECT_EVENT, self)

    def _value_changed(self, value, **kwargs):
        """ Checks if the new value is within limits. If not, emits a limits violation event. Called by pcaspy. """
        if self.target_pos is not None:
            lower = self.target_pos + self.limits[0] - self.tolerance
            upper = self.target_pos + self.limits[1] + self.tolerance
            if value < lower or value > upper:
                msg = "limits violated: position=%f target=%f abs limits=%s"
                self._logger.warn(msg, value, self.target_pos, (lower, upper))
                self._governor.device_event(self._governor.LIMITS_VIOLATED_EVENT, self)

    def _status_changed(self, value, **kwargs):
        """ Checks if the homing status changed. """
        self._homed = bool(int(value) & 0x4000)
        if not self._homed:
            self._logger.error("not homed")

    def move(self, target, wait=False):
        """
        Moves motor to the desired target position.
        :param target: str|float
            A position name OR a position value
        :param wait: bool
            Whether to wait for the motor to finish moving or not. Default: False.
        :return: True if the move was successful, False otherwise
        """
        self.target = None
        target_pos = self.positions.get(target.target, target.target)
        self._logger.info("Issued move to '%s' (%f)", target, target_pos)
        self._val.put(target_pos)
        return self.wait() if wait else True

    def wait(self):
        """
        Waits for the motor to finish moving
        :return: True if the move was successful, False otherwise
        """
        start_pos = self._rbv.get()
        start = time.time()
        self._logger.info("Waiting movement completion")
        time.sleep(0.1)
        while not self.done:
            if time.time() - start > self.timeout and self._rbv.get() == start_pos:
                self._governor.device_event(self._governor.TIMEOUT_EVENT)
                self._logger.warn("Movement timed out")
                return False
            time.sleep(0.1)
        return True

    def stop(self):
        """ Sends a stop command to the motor """
        self._logger.info("Issued STOP")
        self._stop.put(1)

    @property
    def tolerance(self):
        return self._config['tolerance']

    @property
    def connected(self):
        return self._val.wait_for_connection(0)

    @property
    def homed(self):
        return self._homed

    @property
    def done(self):
        return bool(self._dmov.get())

    @Device.target.setter
    def target(self, target):
        Device.target.fset(self, target)
        if target is not None:
            self._value_changed(value=self._rbv.get())

    @property
    def val(self):
        return self._val.get()


class Valve(Device):
    """
    Represents a valve-like device. It communicates with three EPICS PVs:
        <prefix>Pos-Sts
        <prefix>Cmd:Opn-Cmd
        <prefix>Cmd:Cls-Cmd

    It has two implicit positions: Open and Closed.
    """
    REQUIRED_FIELDS = Device.REQUIRED_FIELDS + ('pv', )
    TGT_OPEN = "Open"
    TGT_CLOSED = "Closed"
    TGTS = [TGT_CLOSED, TGT_OPEN]

    def __init__(self, name, config, governor):
        super(Valve, self).__init__(name, config, governor, logger_name="Valve")

        self._val = epics.PV(self.pvname + "Pos-Sts",
                             connection_callback=self._connection_changed,
                             callback=self._value_changed)
        self._open = epics.PV(self.pvname + "Cmd:Opn-Cmd")
        self._close = epics.PV(self.pvname + "Cmd:Cls-Cmd")

        self._current_setpoint = self.val

    def __repr__(self):
        return "Valve({}[{}], pv='{}' positions={})".format(self.name, self.fullname, self.pvname, self.positions)

    def _connection_changed(self, **kwargs):
        """ Emits a disconnect event to the Governor. Called by pcaspy. """
        if not kwargs['conn']:
            self._logger.warn('disconnected')
            self._governor.device_event(self._governor.DISCONNECT_EVENT, self)

    def _value_changed(self, value, **kwargs):
        """ Checks if the new value is still the expected position. If not, emits a limits violation event. """
        if self.target_pos is not None:
            lower = self.target_pos + self.limits[0]
            upper = self.target_pos + self.limits[1]
            if value < lower or value > upper:
                msg = "position violated: position=%s target=%s"
                self._logger.warn(msg, self.TGTS[value], self.TGTS[self.target_pos])
                self._governor.device_event(self._governor.LIMITS_VIOLATED_EVENT, self)

    def move(self, target, wait=False):
        """
        Moves the valve to the desired target position.
        :param target: str
            A position name (either Open or Closed)
        :param wait: bool
            Whether to wait for the valve to finish moving or not. Default: False.
        :return: True if the move was successful, False otherwise
        """
        self.target = None
        target_pos = self.positions.get(target.target, target.target)
        self._logger.info("Issued move to '%s' (%s)", target, target_pos)
        if target.target == self.TGT_OPEN:
            self._open.put(1)
        elif target.target == self.TGT_CLOSED:
            self._close.put(1)
        else:
            raise ValueError("Invalid target {}".format(target.target))
        self._current_setpoint = target_pos

        return self.wait() if wait else True

    def wait(self):
        """
        Waits for the valve to reach the desired position.
        :return: True if the move was successful, False otherwise
        """
        start = time.time()
        self._logger.info("Waiting movement completion. Timeout=%.2f", self.timeout)
        time.sleep(0.1)
        while not self.done:
            if time.time() - start > self.timeout:
                self._governor.device_event(self._governor.TIMEOUT_EVENT)
                self._logger.warn("Movement timed out")
                return False
            time.sleep(0.1)
        return True

    def stop(self):
        self._logger.info("Issued STOP (nothing to do)")

    @property
    def positions(self):
        return {self.TGT_OPEN: 1, self.TGT_CLOSED: 0}

    @property
    def connected(self):
        return self._val.wait_for_connection(0)

    @property
    def done(self):
        return self.val == self._current_setpoint

    @Device.target.setter
    def target(self, target):
        Device.target.fset(self, target)
        if target is not None:
            #raise ValueError
            self._value_changed(value=self._val.get())

    @property
    def val(self):
        return self._val.get()


class Governor:
    DISCONNECT_EVENT = 'disconnect'
    ALARM_EVENT = 'alarm'
    LIMITS_VIOLATED_EVENT = 'limits_violated'
    TIMEOUT_EVENT = 'timeout'
    ABORT_EVENT = 'abort'
    LIMIT_LOW = 0
    LIMIT_HIGH = 1

    STATUS_IDLE = 0
    STATUS_BUSY = 1
    STATUS_DISABLED = 2
    STATUS_FAULT = 3

    def __init__(self, name, config):
        self._name = name
        self._logger_name = "Governor[{}]".format(self.name)
        self._logger = logging.getLogger(self._logger_name)
        self._observer = None
        self._status = self.STATUS_IDLE
        self._disconnected_devices = []
        self._alarmed_devices = []
        self._not_homed_devices = []
        self._current_state = None
        self._next_state = None
        self._init_state = None
        self._devices = {}
        self._states = {}
        self._transitions = {}
        self._config = config
        self._parse_config(config)
        self._queue = Queue()
        self._abort_transition = False
        self._busy = False
        self.set_state(self._init_state)
        self._running = True

        # Lock to prevent multiple transitions from happening at once
        self._worker_lock = Lock()
        # Thread to deal with events
        self._worker_thread = Thread(target=self._worker)
        self._worker_thread.start()
        self._logger.info("created")

    @property
    def name(self):
        return self._name

    @property
    def state(self):
        return self._current_state

    @property
    def idle(self):
        return self._status == self.STATUS_IDLE

    @property
    def busy(self):
        return self._status == self.STATUS_BUSY

    @property
    def fault(self):
        return self._status == self.STATUS_FAULT

    @property
    def logger_name(self):
        return self._logger_name

    @property
    def enabled(self):
        return self.status != self.STATUS_DISABLED

    @property
    def observer(self):
        return self._observer

    @observer.setter
    def observer(self, observer):
        self._observer = observer
        self._notify_observer()

    @property
    def devices(self):
        return self._devices

    @property
    def states(self):
        return self._states

    @property
    def transitions(self):
        return self._transitions

    @property
    def status(self):
        return self._status

    @property
    def status_message(self):
        """ Build the status message string and returns it. """
        if self.fault:
            disconn = ("disconn", self._disconnected_devices)
            alarmed = ("alarm", self._alarmed_devices)
            not_homed = ("!homed", self._not_homed_devices)

            err = []
            for name, device_list in (disconn, alarmed, not_homed):
                if device_list:
                    err.append("{}({})".format(name, ",".join(device_list)))

            return " ".join(err)
        elif self._next_state == self._current_state:
            return "state {}".format(self._current_state)
        else:
            return "transition {} to {}".format(self._current_state, self._next_state)

    def kill(self):
        self._running = False

    def abort(self):
        """ Put an abort event on the events queue. Called by GovernorDriver when an abort command is received. """
        self.device_event(self.ABORT_EVENT)
        self._logger.info("Abort event sent")

    def set_enabled(self, enabled):
        """
        Enable or disable this Governor
        :param enabled: bool
            True if this Governor must be enabled, False otherwise.
        :return: True if the enabled state was successfully changed, False otherwise
        """
        if self.enabled == enabled:
            self._logger.info("Governor is already %sabled", ("dis","en")[int(enabled)])
            return False

        if self.busy:
            self._logger.warn("Can't change enabled state while busy")
            return False

        if enabled:
            self._status = self.STATUS_IDLE
        else:
            self._status = self.STATUS_DISABLED

        self.set_state(self._init_state)
        self._notify_observer()
        return True

    def _notify_observer(self):
        """ Notify the pcaspy server that the PVs have new values. """
        if self._observer is None:
            return

        states = {}
        for state_name, state in self._states.items():
            states[state_name] = {
                "active": state_name == self._current_state,
                "reachable": state_name in self.reachable_states() and self.enabled and self.idle,
                "limits": {dev_name: target.limits for dev_name, target in state.targets.items()}
            }

        transitions = {}
        for origin in self._transitions:
            for dest in self._transitions[origin]:
                transitions[(origin, dest)] = (
                    # Active
                    (origin, dest) == (self._current_state, self._next_state),
                    # Reachable
                    origin == self._current_state and dest in self.reachable_states() and self.enabled and self.idle
                )

        devices = {dev_name: copy.copy(dev.positions) for dev_name, dev in self._devices.items()}

        self._observer.update(self.name, states, transitions, devices)

    def device_event(self, event, device=None):
        """
        Put an event from a device in the events queue. Called by a Device or by abort.

        :param event: str
            The name of the event
        :param device: str
            The source of the event. Default: None
        """
        self._queue.put((event, device))

    def set_state(self, state, force=False):
        """
        Set the Governor's current state.

        :param state: str
            Name of the state to be set to.
        :param force: bool
            Whether to force going to the new state. Default: False
        :return:
        """
        if self._current_state != state or force:
            self._current_state = state
            self._next_state = state
            if state == self._init_state:
                for device in self._devices.values():
                    device.target = None
        self._notify_observer()

    def set_state_device_limit(self, state, device, limit, value):
        """
        Set a new limit value to a State's Device's limit. The new value is written back to the configuration file.

        :param state: str
            The name of the state
        :param device: str
            The name of the device
        :param limit: str
            Which limit to change (either Governor.LIMIT_LOW or Governor.LIMIT_HIGH)
        :param value: float
            The new limit value
        :return: True if the new limit is valid and was successfully set, False otherwise
        """
        old_limits = self._states[state].targets[device].limits
        if limit == self.LIMIT_LOW:
            new_limits = (value, old_limits[1])
        else:
            new_limits = (old_limits[0], value)

        if new_limits[0] > new_limits[1]:
            msg = "state '%s' device: '%s', lower limit exceeds upper limit '%s'"
            self._logger.error(msg, state, device, new_limits)
            return False

        self._states[state].targets[device].limits = new_limits
        self._config.commit()
        return True

    def set_device_position(self, device, position, value):
        """
        Set a new position value to a Device's position. The new value is written back to the configuration file.

        :param device: str
            The name of the device
        :param position: str
            The name of the device's position
        :param value: float
            The new value for the position. Cannot be None
        :return: True if successful, False otherwise
        """
        if value is not None:
            self._logger.info("Setting device '%s' position '%s' to value '%s'", device, position, value)
            self._devices[device].positions[position] = value
            self._config.commit()
        else:
            msg = "Won't set device '%s' position '%s' to None. Ensure '%s' participates in the transition sequence"
            self._logger.warning(msg, device, position, device)
        return True

    def _parse_config(self, config):
        """
        Create Python objects that represent the entities defined in the configuration file.
        :param config: YAML config object
            The configuration object created from the YAML file.
        """

        # Create Device obkects
        self._devices = {}
        for name, device in config['devices'].items():
            self._devices[name] = device_types_registry[device['type']](name, device, self)

        # Create State objects
        self._states = {name: State(name, state) for name, state in config['states'].items()}

        # Sets the initial state
        self._init_state = config['init_state']

        # Read transition sequences
        self._transitions = {}
        for origin, transition in config['transitions'].items():
            self._transitions[origin] = {}
            if origin != self._init_state:
                self._transitions[origin][self._init_state] = {}
            for destination, sequence in transition.items():
                self._transitions[origin][destination] = sequence

    def reachable_states(self, origin=None):
        """
        Returns a list of reachable states from a certain origin, or from the current state if origin is None
        :param origin: str
            Name of the state to get from which to get the other reachable states.
        :return: A list of state names that are reachable from origin
        """
        if origin is None:
            origin = self._current_state
        return (origin,) + tuple(self._transitions[origin].keys())

    def _do_transition(self, dest):
        """
        Performs the Transition from the current state to a desired destination state
        :param dest: str
            The destination state.
        """

        origin = self._current_state

        if not self.enabled:
            msg = "attempted transition [%s]->[%s] while disabled"
            self._logger.warning(msg, origin, dest)
            return

        # Check that the transition is valid
        if dest not in self.reachable_states():
            msg = "Can't transition from [%s] to [%s]"
            self._logger.error(msg, origin, dest)
            return

        msg = "Transition [%s] -> [%s]"
        self._logger.info(msg, self._states[origin].fullname, self._states[dest].fullname)

        self._next_state = dest

        # Special case: if already in dest state, do nothing
        if origin == dest:
            return

        # Update all positions marked with "updateAfter"
        for device, target in self._states[origin].targets.items():
            if target.update_after:
                self.set_device_position(device, target.target, self._devices[device].val)

        # Read sequence of devices
        device_sequence = self._transitions[origin][dest]
        targets = self._states[dest].targets

        self._notify_observer()

        # Start transition
        all_devices = set(self._devices.keys())
        moved_devices = set()
        for item in device_sequence:
            if self.fault:
                return

            # Move one device
            if isinstance(item, str):
                devices = [(self._devices[item], targets[item])]
            # Move several devices at once
            else:
                devices = [(self._devices[i], targets[i]) for i in item]

            for device, target in devices:
                moved_devices.add(device.name)
                if not self._abort_transition:
                    device.move(target)

            for device, target in devices:
                if not self._abort_transition:
                    device.wait()
                if not self._abort_transition:
                    device.target = target

        # Flip current state to destination state
        if not self.fault and not self._abort_transition:
            self.set_state(dest)

        # Any device that was not moved by this transition will *NOT* be monitored
        unmoved_devices = all_devices - moved_devices
        for device in unmoved_devices:
            self._devices[device].target = None

    def do_transition(self, dest):
        # Take lock. Only one Transition can happen at a time.
        with self._worker_lock:
            start_time = time.time()

            self._status = self.STATUS_BUSY
            self._notify_observer()

            self._abort_transition = False

            try:
                self._do_transition(dest)
            except Exception as e:
                self._logger.exception("Exception during transition: %s", e)

            if self._status == self.STATUS_BUSY:
                self._status = self.STATUS_IDLE

            self._abort_transition = False
            self._notify_observer()

            self._logger.debug("Transition took %.3f seconds", time.time() - start_time)

    def check_fault_state(self):
        """ Checks if still in fault state """
        self._disconnected_devices = [d for d in self._devices if not self._devices[d].connected]
        self._alarmed_devices = [d for d in self._devices if self._devices[d].alarmed]
        self._not_homed_devices = [d for d in self._devices if not self._devices[d].homed]

        if self.enabled:
            if any((self._disconnected_devices, self._alarmed_devices, self._not_homed_devices)):
                self._status = self.STATUS_FAULT
                self.set_state(self._init_state)
            elif self.fault:
                self._status = self.STATUS_IDLE
                self.set_state(self._init_state)

    def _worker(self):
        """ Runs in a separate thread. Deals with events coming from the Devices or from an abort command."""
        while self._running:
            try:
                message = self._queue.get(timeout=0.5)
            except Empty:
                # No messages, check if we're in a fault state
                self.check_fault_state()
                continue

            # A device disconnected, major alarmed, moved out of limits or timed out: go to fault state
            event, device = message
            if event in (self.DISCONNECT_EVENT, self.ALARM_EVENT, self.LIMITS_VIOLATED_EVENT, self.TIMEOUT_EVENT,
                         self.ABORT_EVENT):
                self._abort_transition = True
                self.set_state(self._init_state, force=True)
                if event == self.ABORT_EVENT:
                    for dev in self.devices.values():
                        dev.stop()
                self.check_fault_state()
