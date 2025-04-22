import logging

import components
import ruamel.yaml as yaml


class ConfigManager:
    """ Represents a YAML configuration file that describes a Governor. Acts like a dictionary. """

    def __init__(self, filename):
        """
        :param filename: str
            Backing YAML configuration file.
        """
        self._logger = logging.getLogger("ConfigManager")
        self._filename = filename

        with open(filename) as f:
            # round_trip_load allows us to write back to the config file when settings change
            self._raw_config = yaml.round_trip_load(f)

        self.ok = self.check_config()
        if not self.ok:
            self._logger.error("Invalid config file '%s'", filename)

    def get(self, key, default=None):
        return self._raw_config.get(key, default)

    def __getitem__(self, item):
        return self._raw_config[item]

    @property
    def filename(self):
        return self._filename

    def commit(self):
        """
        Writes changes back to the backing configuration file
        """
        with open(self._filename, 'w') as f:
            yaml.round_trip_dump(self._raw_config, f)

    def check_config(self):
        """
        Checks that the configuration file has the right schema

        :return: True if the configuration file is OK, False otherwise
        """
        checks = [self._check_root_mandatory, self._check_init_state,
                  self._check_devices, self._check_states, self._check_limits,
                  self._check_transitions]

        for check in checks:
            if not check():
                return False
        return True

    def _check_mandatory(self, mandatory_params, config, where):
        """
        Checks that all parameters in mandatory_parameters are present in config.

        :param mandatory_params: list of str
            List of parameter names that must be present in config.
        :param config: YAML config object
            The configuration object to be checked
        :param where: str
            Description of the configuration location. It will be printed if the check fails.
        :return: True if all parameters in mandatory_parameters are in config, False otherwise.
        """
        ok = True
        for param in mandatory_params:
            if param not in config:
                self._logger.error("Missing mandatory parameter '%s' in '%s'", param, where)
                ok = False
        return ok

    def _check_root_mandatory(self):
        """
        Checks that the root of the configuraiton file has certain required parameters.

        :return: True if it has, False otherwise.
        """
        return self._check_mandatory(('init_state', 'devices', 'states'), self._raw_config, "root")

    def _check_type(self, name, device):
        """
        Checks that a Device's type is valid

        :param name: str
            The name of the Device being checked
        :param device: YAML config object for a Device
        :return: True if the Device's type is valid, False otherwise
        """
        if device['type'] not in components.device_types_registry:
            self._logger.error("device[{}] type can only be one of {}".format(name, tuple(components.device_types_registry.keys())))
            return False
        return True

    def _check_devices(self):
        """
        Checks all Devices configuration for validity.

        :return: True if all Devices have valid configurations, False otherwise.
        """
        ok = True
        for name, device in self._raw_config['devices'].items():
            fmt_name = "device[{}]".format(name)
            # Every Device must have a 'type'
            if not self._check_mandatory(('type', ), device, fmt_name):
                ok = False
            else:
                # The Device's type must be valid
                if not self._check_type(name, device):
                    ok = False

                # Each Device type requires different mandatory parameters
                required_fields = components.device_types_registry[device['type']].REQUIRED_FIELDS
                if not self._check_mandatory(required_fields, device, fmt_name):
                    ok = False
        return ok

    def _check_states(self):
        """
        Checks all State's configuration for validity.

        :return: True if all States have valid configurations, False otherwise.
        """
        ok = True
        for name, state in self._raw_config['states'].items():
            for device_name, device_target in state.get('targets', {}).items():
                # Check that all Devices mentioned by the State were declared before
                if device_name not in self._raw_config['devices']:
                    msg = "State '%s' mentions unknown device '%s'"
                    self._logger.error(msg, name, device_name)
                    ok = False
                    continue

                # Every State definition must contain a 'target' and a 'limits' parameter
                target_name = ""
                where = "state[{}] device[{}]".format(name, device_name)
                if not self._check_mandatory(('target', 'limits'), device_target, where):
                    ok = False
                else:
                    target_name = device_target['target']

                # The Target defined by this State must have been declared before in the Device definition
                cfg = self._raw_config['devices'][device_name]
                if 'positions' in cfg and target_name not in cfg['positions']:
                    msg = "State '%s' device '%s' invalid target: %s"
                    self._logger.error(msg, name, device_name, device_target['target'])
                    ok = False
        return ok

    def _check_limits(self):
        """
        Checks if all States limits are valid. The limits are *relative* to the target position.

        :return: True if all States limits are valid, False otherwise.
        """
        ok = True
        for state_name, state in self._raw_config['states'].items():
            if 'targets' not in state:
                continue
            for device, device_target in state['targets'].items():
                target_name, limits = device_target['target'], tuple(device_target['limits'])
                if limits[0] > limits[1]:
                    msg = "State '%s' device '%s' target '%s' lower limit[%f] > upper limit[%f]"
                    self._logger.error(msg, state_name, device, target_name, *limits)
                    ok = False
                    continue
        return ok

    def _check_transitions(self):
        """
        Checks if all Transitions are valid.

        :return: True if all Transitions are valid, False otherwise.
        """
        ok = True
        for origin, transition in self._raw_config['transitions'].items():
            # The origin of a Transition must have been declared before
            if origin not in self._raw_config['states']:
                msg = "Invalid transition origin '%s'"
                self._logger.error(msg, origin)
                ok = False

            for destination, sequence in transition.items():
                # The destination of a Transition must have been declared before
                if destination not in self._raw_config['states']:
                    msg = "Invalid transition destination '%s'"
                    self._logger.error(msg, destination)
                    ok = False

                # Can't have a Transition from and to the same State
                if origin == destination:
                    msg = "Transition to '%s'"
                    self._logger.error(msg, origin)
                    ok = False

                # All devices mentioned in a Transition must have been declared before
                sequence_devices = set()
                for device_name in sequence:
                    if isinstance(device_name, str):
                        sequence_devices.add(device_name)
                    else:
                        sequence_devices |= set(device_name)

                msg = "Transition %s->%s contains invalid device '%s'"
                for device_name in sequence_devices:
                    if device_name not in self._raw_config['devices']:
                        self._logger.error(msg, origin, destination, device_name)
                        ok = False

                # All devices mentioned in a Transition must be part of the target State
                msg = "Transition %s->%s sequence moves a device '%s' that is not part of the destination"
                destination_state = self._raw_config['states'][destination]
                for device_name in sequence_devices:
                    if device_name not in destination_state['targets']:
                        self._logger.error(msg, origin, destination, device_name)
                        ok = False

                # All devices in the destination must be moved to get there
                # This might be too strict, leave it out for now
                #msg = "Transition %s->%s sequence must move device '%s', but it's missing"
                #for device_name in destination_state['targets']:
                #    if device_name not in sequence_devices:
                #        self._logger.error(msg, origin, destination, device_name)
                #        ok = False

        return ok

    def _check_init_state(self):
        """
        Checks that an initial State was defined.

        :return: True if an initial State was properly defined, False otherwise.
        """
        if self._raw_config['init_state'] not in self._raw_config['states']:
            msg = "Invalid init state: '%s'"
            self._logger.error(msg, self._raw_config['init_state'])
            return False
        return True
