import argparse
import logging
import re
import time
from threading import Thread
from queue import Queue
from collections import OrderedDict
from functools import partial

from pcaspy import SimpleServer, Driver, Severity

from cfgmanager import ConfigManager
from components import Governor

import ruamel.yaml as yaml


class GovernorDriver(Driver):
    """
    The GovernorDriver deals with EPICS requests. It manages different "governors" (only one "governor" is active at
    any given time) and routes the requests to the appropriate governor.
    """
    def __init__(self, governors, active_governor, sync_targets={}):
        """
        :param governors: list of Governor:
            A list of governors to be managed
        :param active_governor: object of type Governor:
            Which of the governors in the list will be the active one at first
        :param sync_targets: dict:
            Targets to be kept in sync between all governors. A dictionary of the form
                { device_name [str]: list of target_name [list of str] }
        """
        # pcaspy's base class constructor
        Driver.__init__(self)

        # Internal state
        self._active = True
        self._governors = governors
        self._active_governor = active_governor
        self._sync_targets = sync_targets
        self._logger = logging.getLogger("GovernorDriver")

        # Initialize PV values
        self.setParam("{Gov}Active-Sel", self._active)
        self.setParam("{Gov}Config-Sel", list(governors.keys()).index(active_governor))
        self.setParam("{Gov}Cmd:Abort-Cmd", 0)
        self.setParam("{Gov}Cmd:Kill-Cmd", 0)
        for gov_name, governor in self._governors.items():
            governor.observer = self
            governor.set_enabled(gov_name == active_governor)
            prefix = '{{Gov:{}}}'.format(gov_name)
            self.setParam(prefix+'Sts:Status-Sts', governor.status)
            self.setParam(prefix+'Sts:Msg-Sts', governor.status_message)

        self._worker_task = Thread(target=self._worker)
        self._worker_q = Queue()

        self._worker_task.start()

    def _worker(self):
        """ Handles transitions in its own thread"""
        while True:
            cmd = self._worker_q.get()

            # Check for poison
            if cmd is None:
                self._logger.info("stopping worker")
                break

            action, after = cmd
            action()
            after()

    def update(self, gov_name, states, transitions, devices):
        """
        Updates all relevant PV values. This method is called by each individual Governor when there is a change in its
        whole operating state.

        :param gov_name: str
            The name of the governor issuing the update
        :param states: dict
            The current operating state of all governor States. A dictionary of the form
                {
                    state_name [str] : {
                        'reachable': reachable [bool],
                        'active': active [bool],
                        'limits': {
                            device_name [str]: (low_limit, high_limit) [(float, float) tuple]
                    }
                }
        :param transitions: dict
            The current operating state of all governor Transitions. A dictionary of the form:
                { (from, to) [(str, str) tuple]: (active, reachable) [(bool, bool) tuple]
        :param devices: dict
            The current target values for all governor Devices. A dictionary of the form:
                { device_name [str]: list of (target_name [str], target_value [float])
        """
        prefix = '{{Gov:{}}}'.format(gov_name)

        # Sorted list of reachable state names
        self.setParam(prefix + 'Sts:Reach-I', sorted(list(set(
            state_to
            for (_, state_to), (_, reachable) in transitions.items()
            if reachable
        ))))

        # All active states (there should be only one, pick first)
        self.setParam(prefix + 'Sts:State-I', next((
            state_name
            for state_name, state_data in states.items()
            if state_data['active']
        ), ''))

        self.setParam(prefix + 'Sts:Busy-Sts', self._governors[gov_name].busy)

        for state_name, state_updates in states.items():
            prefix = '{{Gov:{}-St:{}}}'.format(gov_name, state_name)
            self.setParam(prefix + 'Sts:Active-Sts', state_updates["active"])
            self.setParam(prefix + 'Sts:Reach-Sts', state_updates["reachable"])

            for device_name, (low_lim, high_lim) in state_updates["limits"].items():
                low_param = '{{Gov:{}-Dev:{}}}{}:LLim-Pos'.format(gov_name, device_name, state_name)
                high_param = '{{Gov:{}-Dev:{}}}{}:HLim-Pos'.format(gov_name, device_name, state_name)
                self.setParam(low_param, low_lim)
                self.setParam(high_param, high_lim)

        for transition, (active, reachable) in transitions.items():
            prefix = '{{Gov:{}-Tr:{}-{}}}'.format(gov_name, *transition)
            self.setParam(prefix + 'Sts:Active-Sts', active)
            self.setParam(prefix + 'Sts:Reach-Sts', reachable)

        for device_name, positions in devices.items():
            for position_name, position_value in positions.items():
                param = '{{Gov:{}-Dev:{}}}Pos:{}-Pos'.format(gov_name, device_name, position_name)
                self.setParam(param, position_value)

        self.updatePVs()

    def read(self, reason):
        """
        Returns the current value of a PV. This is called by pcaspy code when it receives a GET request for a PV.

        :param reason: str
            The PV name
        :return: The value of the PV
        """
        gov_status = re.match(r'\{Gov:(.+)\}Sts:Status-Sts', reason)
        gov_status_message = re.match(r'\{Gov:(.+)\}Sts:Msg-Sts', reason)

        if reason == "{Gov}Active-Sel":
            return self._active
        elif reason == "{Gov}Config-Sel":
            return list(self._governors.keys()).index(self._active_governor)
        elif reason == "{Gov}Cmd:Abort-Cmd":
            return 0
        elif gov_status is not None:
            return self._governors[gov_status.group(1)].status
        elif gov_status_message is not None:
            return self._governors[gov_status_message.group(1)].status_message
        else:
            self._logger.debug("couldn't match reason %s", reason)
            return self.getParam(reason)

    def write(self, reason, value):
        """
        Writes a value to a PV. This is called by pcaspy code when it receives a PUT request for a PV.

        :param reason: str
            The PV name
        :param value: any
            The value to be written to the PV
        """
        self._logger.debug("write(reason=%s,value=%s)", reason, value)
        status = True

        gov_abort = re.match(r'\{Gov:(?P<gov_name>.+)\}Cmd:Abort-Cmd', reason)
        gov_go = re.match(r'\{Gov:(?P<gov_name>.+)\}Cmd:Go-Cmd', reason)
        # Limit reason is something like: {Gov:Human-Dev:bsz}SE:HLim-Pos
        dev_limit = re.match(r'\{Gov:(?P<gov_name>.+)-Dev:(?P<dev_name>.*)\}(?P<state_name>.*):(?P<limit>.*)-Pos', reason)
        # Position reason is something like {Gov:Human-Dev:bsy}Pos:Down-Pos
        dev_pos = re.match(r'\{Gov:(?P<gov_name>.+)-Dev:(?P<dev_name>.*)\}Pos:(?P<pos_name>.*)-Pos', reason)

        if reason == "{Gov}Active-Sel":
            self._active = bool(value)
        elif reason == "{Gov}Cmd:Abort-Cmd":
            self._logger.info("Got abort command")
            self._governors[self._active_governor].abort()
        elif reason == "{Gov}Cmd:Kill-Cmd":
            self._logger.info("Got kill command. Exiting...")
            global running
            running = False
        elif self._active:
            if reason == "{Gov}Config-Sel":
                next_governor = tuple(self._governors.keys())[value]
                if next_governor != self._active_governor:
                    self._logger.info("Changing governor from [%s] to [%s]", self._active_governor, next_governor)
                    if not self._governors[self._active_governor].set_enabled(False):
                        status = False
                    else:
                        self._active_governor = next_governor
                        self._governors[self._active_governor].set_enabled(True)
                else:
                    status = False

            elif gov_abort is not None:
                governor = self._governors[gov_abort.group('gov_name')]
                governor.abort()

            elif gov_go is not None:
                governor = self._governors[gov_go.group('gov_name')]
                if value in governor.reachable_states():
                    # Do the transition on our worker
                    self._worker_q.put((
                        partial(governor.do_transition, value),
                        partial(self.callbackPV, reason)
                    ))
                else:
                    status = False
            elif dev_limit is not None:
                governor = self._governors[dev_limit.group('gov_name')]
                state = dev_limit.group('state_name')
                limit = dev_limit.group('limit')
                device = dev_limit.group('dev_name')
                limit = {"LLim": governor.LIMIT_LOW, "HLim": governor.LIMIT_HIGH}[limit]
                status = governor.set_state_device_limit(state, device, limit, value)
            elif dev_pos is not None:
                governor = self._governors[dev_pos.group('gov_name')]
                device, position = dev_pos.group('dev_name'), dev_pos.group('pos_name')

                if device in self._sync_targets and position in self._sync_targets[device]:
                    govs = self._governors.values()
                else:
                    govs = [governor]

                for gov in govs:
                    status = status and gov.set_device_position(device, position, value)
                    if status:
                        r = '{{Gov:{}-Dev:{}}}Pos:{}-Pos'.format(gov.name, device, position)
                        self.setParam(r, value)
            else:
                status = False
        else:
            status = False

        if status:
            self.setParam(reason, value)

        return status

running = True
if __name__ == '__main__':
    # Accepted arguments
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-c", "--config", help="configuration files to load", nargs="+",
                        required=True)
    parser.add_argument("--check_config", help="check configuration file and exit", action="store_true")
    parser.add_argument("-l", "--log_level", help="set log level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"])
    parser.add_argument("--prefix", help="prefix to Gov's PV names", default="")
    parser.add_argument("-s", "--sync", help="synchronization config file")
    args = parser.parse_args()

    logging.basicConfig(level=args.log_level, format='[%(asctime)s]%(levelname)s:%(name)s:%(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    logging.info("The Governor")

    # Check configuration files for errors
    configs = [ConfigManager(config) for config in args.config]
    for config in configs:
        if not config.ok:
            logging.error("Invalid config file %s", config.filename)
            exit(1)

    logging.info("Configuration files %s loaded", args.config)
    if args.check_config:
        exit(0)

    # Create one Governor per config file
    governors = OrderedDict()
    for config in configs:
        governors[config['name']] = Governor(config['name'], config)

    # Read in sync targets
    sync_targets = {}
    if args.sync:
        with open(args.sync) as f:
            sync_targets = yaml.load(f)

    # Crude check on sync file
    for dev_name, dev_targets in sync_targets.items():
        if not all([dev_name in gov.devices for gov in governors.values()]):
            logging.error("Device {} not present in all configurations".format(dev_name))
            exit(0)

        for target in dev_targets:
            if not all([target in gov.devices[dev_name].positions for gov in governors.values()]):
                logging.error("Device {} target {} not present in all configurations".format(dev_name, target))
                exit(0)

    # Create the pcaspy server (IOC) and all of its PVs
    server = SimpleServer()

    # General PVs that control all Governors

    # Select whether governors are active (can transition to a different state)
    server.createPV(args.prefix, {
        '{Gov}Active-Sel': {
            'type': 'enum',
            'enums': ['Inactive', 'Active'],
            'value': 1
        }
    })

    # Select which governor to use
    server.createPV(args.prefix, {
        '{Gov}Config-Sel': {
            'type': 'enum',
            'enums': [config['name'] for config in configs],
            'value': 0
        }
    })

    # List of all existing governors
    server.createPV(args.prefix, {
        '{Gov}Sts:Configs-I': {
            'type': 'string',
            'value': sorted(config['name'] for config in configs),
            'count': len([config for config in configs]),
        }
    })

    # Abort all governors
    server.createPV(args.prefix, {
        '{Gov}Cmd:Abort-Cmd': {
            'type': 'int',
            'value': 0
        }
    })

    # Kill entire IOC
    server.createPV(args.prefix, {
        '{Gov}Cmd:Kill-Cmd': {
            'type': 'int',
            'value': 0
        }
    })

    # Governor-specific PVs
    for gov_name, governor in governors.items():
        gov_prefix = "{{Gov:{}}}".format(gov_name)

        # Abort this governor
        server.createPV(args.prefix, {
            gov_prefix+'Cmd:Abort-Cmd': {
                'type': 'int',
                'value': 0
            }
        })

        # Command: write the desired destination state name
        # to this PV to start a transition
        server.createPV(args.prefix, {
            gov_prefix+'Cmd:Go-Cmd': {
                'type': 'string',
                'value': "",
                'asyn': True
            }
        })

        # Governor's status
        server.createPV(args.prefix, {
            gov_prefix+'Sts:Status-Sts': {
                'type': 'enum',
                'enums': ['Idle', 'Busy', 'Disabled', 'FAULT'],
                'states': [
                    Severity.NO_ALARM, Severity.NO_ALARM,
                    Severity.NO_ALARM, Severity.MAJOR_ALARM
                ],
                'value': 0,
                'scan': 0.5
            }
        })

        # Governor's message
        server.createPV(args.prefix, {
            gov_prefix+'Sts:Msg-Sts': {
                'type': 'string',
                'value': "",
                'scan': 0.5
            }
        })

        # All existing states
        server.createPV(args.prefix, {
            gov_prefix+'Sts:States-I': {
                'type': 'string',
                'value': sorted(governor.states),
                'count': len(governor.states),
            }
        })

        # All existing devices
        server.createPV(args.prefix, {
            gov_prefix+'Sts:Devs-I': {
                'type': 'string',
                'value': sorted(governor.devices),
                'count': len(governor.devices),
            }
        })

        # Current state
        server.createPV(args.prefix, {
            gov_prefix+'Sts:State-I': {
                'type': 'string',
                'value': ''
            }
        })

        # States reachable from current state
        server.createPV(args.prefix, {
            gov_prefix+'Sts:Reach-I': {
                'type': 'string',
                'value': [''],
                'count': len(governor.states),
            }
        })

        # Busy transitioning
        server.createPV(args.prefix, {
            gov_prefix+'Sts:Busy-Sts': {
                'type': 'enum',
                'enums': ['No', 'Yes'],
                'value': 0,
            }
        })

        # All existing devices
        server.createPV(args.prefix, {
            gov_prefix+'Sts:Devs-I': {
                'type': 'string',
                'value': list(governor.devices),
                'count': len(governor.devices)
            }
        })

        for device_name, device in governor.devices.items():
            server.createPV(args.prefix, {
                '{{Gov:{}-Dev:{}}}Sts:Tgts-I'.format(gov_name, device_name): {
                    'type': 'string',
                    'value': list(device.positions),
                    'count': len(device.positions),
                }
            })

            for target in device.positions:
                name = '{{Gov:{}-Dev:{}}}Pos:{}-Pos'.format(gov_name, device_name, target)
                server.createPV(args.prefix, {name: {'type': 'float', 'value': 0}})

            for state_name in governor.states:
                for lim in ('LLim', 'HLim'):
                    name = '{{Gov:{}-Dev:{}}}{}:{}-Pos'.format(gov_name, device_name, state_name, lim)
                    server.createPV(args.prefix, {name: {'type': 'float', 'value': 0}})

        for state_name in governor.states:
            dev = '{{Gov:{}-St:{}}}'.format(gov_name, state_name)
            server.createPV(args.prefix, {dev + 'Sts:Reach-Sts': {'type': 'int', 'value': 0}})
            server.createPV(args.prefix, {dev + 'Sts:Active-Sts': {'type': 'int', 'value': 0}})

            for next_state in governor.reachable_states(state_name):
                dev = '{{Gov:{}-Tr:{}-{}}}'.format(gov_name, state_name, next_state)
                server.createPV(args.prefix, {dev + 'Sts:Active-Sts': {'type': 'int', 'value': 0}})
                server.createPV(args.prefix, {dev + 'Sts:Reach-Sts': {'type': 'int', 'value': 0}})

    # Create a Driver that will manage all Governors
    driver = GovernorDriver(governors, configs[0]["name"], sync_targets)

    # Process incoming EPICS GETs and PUTs every 100 ms
    while running:
        server.process(0.1)

    for gov in governors.values():
        gov.kill()

    time.sleep(0.5)
