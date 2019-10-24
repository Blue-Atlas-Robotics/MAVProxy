#!/usr/bin/env python
'''
Fishi Module
Oleksandr Slovak, August 2019
'''

import os
import sys
import time
from pprint import pprint

from importlib import reload
from types import ModuleType

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
import pyfishi.manager as mng
from pymavlink import mavutil

from .joystick import key_map

reloaded = set()

skip_modules = (
    "__",
    "collections",
    "abc",
    "sys",
    "os",
    "mp",
    "pickle",
    "np",
    "quat",
    "threading",
    "py_trees",
    "ABCMeta",
    "colr",
    "traceback",
    "linecache",
    "functools",
    "xmltd",
    "splinalg",
    "dsp",
)

msg_types_master = {
    # 'RC_CHANNELS',
    'SCALED_PRESSURE2',
    # 'POWER_STATUS',
    # 'SYSTEM_TIME',
    # 'GPS_RAW_INT',
    # 'SCALED_PRESSURE',
    # 'SCALED_IMU3',
    # 'VFR_HUD',
    # 'SIMSTATE',
    # 'VIBRATION',
    # 'HWSTATUS',
    # 'NAMED_VALUE_FLOAT',
    # 'MISSION_CURRENT',
    # 'SCALED_IMU2',
    # 'STATUSTEXT',
    # 'NAV_CONTROLLER_OUTPUT',
    # 'COMMAND_ACK',
    'HEARTBEAT',
    'ATTITUDE',
    'ATTITUDE_QUATERNION',
    'CONTROL_SYSTEM_STATE',
    'RC_CHANNELS_SCALED',
    'EKF_STATUS_REPORT',
    # 'AHRS3',
    'SENSOR_OFFSETS',
    # 'GPS_GLOBAL_ORIGIN',
    'SYS_STATUS',
    # 'AHRS2',
    # 'SERVO_OUTPUT_RAW',
    # 'AHRS',
    'RAW_IMU',
    # 'GLOBAL_POSITION_INT',
    # 'HOME_POSITION',
    # 'BATTERY_STATUS',
    # 'TIMESYNC',
    # 'MEMINFO'
}

msg_types_gcs = {
    # 'COMMAND_LONG',
    'MANUAL_CONTROL',
    # 'HEARTBEAT'
}


def rreload(module, name="", depth=0, max_depth=10, skip_names=skip_modules):
    """Recursively reload modules."""
    global reloaded
    reload(module)
    reloaded = reloaded.union({name})
    for attribute_name in dir(module):
        skip = False
        for skip_name in skip_names:
            if skip_name in attribute_name:
                skip = True
        if skip:
            continue

        if attribute_name in sys.builtin_module_names:
            continue

        if depth > max_depth:
            raise Exception('Too deep recursion for rreload, please, revisit project structure')

        if attribute_name in reloaded:
            continue

        attribute = getattr(module, attribute_name)
        if type(attribute) is ModuleType:
            # print("reloaded: " + attribute_name)
            rreload(attribute, name=attribute_name, depth=depth + 1)


class Fishi(mp_module.MPModule):
    live_log_toggle = False
    one_button_pressed = False
    two_buttons_pressed = False

    prev_time = time.time()
    prev_seq = 0

    trim_presets = {
        "camera": {
            "index": 0,
            "presets": (
                ("", "cam_auto", "0", "1"),
                ("", "cam_exp", "0", "1"),
                ("", "cam_gain", "0", "1"),
                ("", "cam_cbal", "0", "64"),
                ("", "cam_cbal", "1", "64"),
                ("", "cam_cbal", "2", "64"),
            )
        },
        "ctrl": {
            "index": 0,
            "presets": (
                ("", "off_t", "2", "0.01"),
                ("", "trim_yaw", "0", "1"),
            )
        },
    }

    messages = {
        "master": {t: None for t in msg_types_master},
        "GCS": {t: None for t in msg_types_gcs},
        "opt": {
            "set_seq": 0,
            "get_seq": 0,
            "trim_seq": 0,
            "name": "off_t",
            "idx": 2,
            "value": None,
            "trim": 0.01,
        },
        "cmd": {"terminate": False},
    }

    messages_seq = {
        "master": {t: 0 for t in msg_types_master},
        "GCS": {t: 0 for t in msg_types_gcs}
    }

    def __init__(self, mpstate, **kwargs):
        """Initialise module"""
        super(Fishi, self).__init__(mpstate, "fishi", public=True, multi_vehicle=True)

        self.fishi_settings = mp_settings.MPSettings([
            ('verbose', bool, False),
        ])
        self.add_command('fishi', self.cmd_fishi, "fishi module", ['status', 'set (LOGSETTING)'])

        rreload(mng)
        self.control_loop = mng.Manager(log_folder_path=mpstate.status.logdir, **kwargs)

        # TODO, arguments to fihsi module for auto arm
        # self.master.set_mode(19)  # MANUAL, it will work even if pymavlink does not have a mode mapping updated

        # self.master.arducopter_arm()

        # for _ in range(4):
        #     self.master.mav.manual_control_send(
        #         self.master.target_system, 0, 0, 0, 0,
        #         key_map["LEFT_STICK_PRESS"]
        #     )

    def unload(self):
        print("\n")
        # TODO, arguments to fihsi module for auto arm
        # self.master.arducopter_disarm()  # Do not do disarm to prevent EKF to reinitialize.

        # for _ in range(4):
        #     self.master.mav.manual_control_send(
        #         self.master.target_system, 0, 0, 0, 0,
        #         key_map["RIGHT_STICK_PRESS"]
        #     )

        self.master.set_mode(19)
        self.messages["cmd"] = {"terminate": True}
        self.control_loop.set_input(self.messages)
        self.control_loop.external_end.close()
        self.control_loop.stop_log.set()
        while self.control_loop.rotate_log.is_set():
            pass

    def usage(self):
        '''show help on command line options'''
        return "Usage: fishi <status|set>"

    def cmd_fishi(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.fishi_settings.command(args[1:])
        elif args[0] == "live":
            self.cmd_toggle()
        elif args[0] == "opt":
            self.cmd_opt(args)
        elif args[0] == "do_trim":
            self.cmd_do_trim(args)
        elif args[0] == "set_trim":
            self.cmd_set_trim(args)
        elif args[0] == "rotate":
            self.cmp_rotate()
        else:
            print(self.usage())

    def cmp_rotate(self):
        print("\n")
        self.control_loop.rotate_log.set()

    def cmd_do_trim(self, args):

        if len(args) == 4:
            self.messages["opt"]["name"] = args[1]
            self.messages["opt"]["idx"] = int(args[2])
            self.messages["opt"]["trim"] = float(args[3])

            self.messages["opt"]["trim_seq"] = self.messages["opt"]["trim_seq"] + 1
        else:
            pass

    def cmd_set_trim(self, args):
        if len(args) == 4:
            self.messages["opt"]["name"] = args[1]
            self.messages["opt"]["idx"] = int(args[2])
            self.messages["opt"]["trim"] = float(args[3])

            print("trim: " + self.messages["opt"]["name"] + "[" + str(self.messages["opt"]["idx"]) +
                  "] : (+-)" + str(self.messages["opt"]["trim"]))
        else:
            pass

    def cmd_joy_trim(self, direction):

        if direction == "up":
            if self.messages["opt"]["trim"] < 0:
                self.messages["opt"]["trim"] = -self.messages["opt"]["trim"]

            self.messages["opt"]["trim_seq"] = self.messages["opt"]["trim_seq"] + 1

        elif direction == "down":
            if self.messages["opt"]["trim"] > 0:
                self.messages["opt"]["trim"] = -self.messages["opt"]["trim"]

            self.messages["opt"]["trim_seq"] = self.messages["opt"]["trim_seq"] + 1
        else:
            pass

    def cmd_opt(self, args):

        if len(args) != 4:
            self.messages["opt"]["name"] = None
            self.messages["opt"]["set_seq"] = self.messages["opt"]["set_seq"] + 1
        else:

            self.messages["opt"]["name"] = args[1]
            self.messages["opt"]["idx"] = int(args[2])
            self.messages["opt"]["value"] = float(args[3])

            self.messages["opt"]["set_seq"] = self.messages["opt"]["set_seq"] + 1

    def status(self):
        '''returns information about module'''
        return ""

    def cmd_toggle(self):
        '''returns information about module'''
        self.live_log_toggle = not self.live_log_toggle

    def idle_task(self):
        '''called rapidly by mavproxy'''
        # TODO, get log chunks from ctrl_log queue pickle and append to file
        pass

    def mavlink_packet(self, m):
        '''handle mavlink packets'''

        msg_type = m.get_type()
        if m.get_type() in self.messages["master"].keys():
            self.messages["master"][msg_type] = m.to_dict()

        if m.get_type() == "HEARTBEAT" and self.live_log_toggle:
            pass

        if m.get_type() == "SENSOR_OFFSETS" and self.live_log_toggle:
            self.control_loop.dump_tail.set()

        if m.get_type() == "ATTITUDE_QUATERNION":
            self.control_loop.set_input(self.messages)
            self.communicate(self.control_loop.get_output())

    def mavlink_packet_slave(self, m):
        '''handle mavlink packets from slave connections'''

        seq_now = m.get_seq()
        msg_type = m.get_type()
        msg_dict = m.to_dict()

        if msg_type in self.messages["GCS"].keys() and seq_now != self.messages_seq["GCS"][msg_type]:
            self.messages["GCS"][msg_type] = msg_dict
            self.messages_seq["GCS"][msg_type] = seq_now

        if msg_type == "MANUAL_CONTROL":
            self.__handle_manual_control(msg_dict)

    def __handle_manual_control(self, msg_dict):
        if (msg_dict["buttons"] == (key_map["LB"] | key_map["RB"])) and not self.two_buttons_pressed:
            self.live_log_toggle = not self.live_log_toggle
            self.two_buttons_pressed = True

        if (msg_dict["buttons"] == key_map["A"]) and not self.one_button_pressed:
            self.master.set_mode(20)
            self.one_button_pressed = True

        # if (msg_dict["buttons"] == key_map["DIGITAL_UP"]) and not self.one_button_pressed:
        #     self.cmd_joy_trim("up")
        #     self.one_button_pressed = True
        #
        # if (msg_dict["buttons"] == key_map["DIGITAL_DOWN"]) and not self.one_button_pressed:
        #     self.cmd_joy_trim("down")
        #     self.one_button_pressed = True

        if (msg_dict["buttons"] == key_map["DIGITAL_UP"]) and not self.one_button_pressed:
            self.cmd_do_trim(("", "depth_trim", "0", "-0.2"))
            self.one_button_pressed = True

        if (msg_dict["buttons"] == key_map["DIGITAL_DOWN"]) and not self.one_button_pressed:
            self.cmd_do_trim(("", "depth_trim", "0", "0.2"))
            self.one_button_pressed = True

        if (msg_dict["buttons"] == key_map["X"]) and not self.one_button_pressed:
            self.cmd_set_trim(("", "off_t", "2", "0.01"))
            self.one_button_pressed = True

        if (msg_dict["buttons"] == key_map["Y"]) and not self.one_button_pressed:
            self.cmd_set_trim(("", "trim_yaw", "0", "1"))
            self.one_button_pressed = True

        if (msg_dict["buttons"] == key_map["DIGITAL_RIGHT"]) and not self.one_button_pressed:
            self.cmd_do_trim(("", "trim_f_t", "1", "5"))
            self.one_button_pressed = True

        if (msg_dict["buttons"] == key_map["DIGITAL_LEFT"]) and not self.one_button_pressed:
            self.cmd_do_trim(("", "trim_f_t", "1", "-5"))
            self.one_button_pressed = True

        if (msg_dict["buttons"] == (key_map["RB"] | key_map["DIGITAL_UP"])) and not self.two_buttons_pressed:
            self.trim_presets["camera"]["index"] += 1
            if self.trim_presets["camera"]["index"] >= len(self.trim_presets["camera"]["presets"]):
                self.trim_presets["camera"]["index"] = len(self.trim_presets["camera"]["presets"]) - 1
            self.cmd_set_trim(self.trim_presets["camera"]["presets"][self.trim_presets["camera"]["index"]])
            self.two_buttons_pressed = True

        if (msg_dict["buttons"] == (key_map["RB"] | key_map["DIGITAL_DOWN"])) and not self.two_buttons_pressed:
            self.trim_presets["camera"]["index"] -= 1
            if self.trim_presets["camera"]["index"] < 0:
                self.trim_presets["camera"]["index"] = 0
            self.cmd_set_trim(self.trim_presets["camera"]["presets"][self.trim_presets["camera"]["index"]])
            self.two_buttons_pressed = True

        if (msg_dict["buttons"] == (key_map["LB"] | key_map["DIGITAL_UP"])) and not self.two_buttons_pressed:
            self.trim_presets["ctrl"]["index"] += 1
            if self.trim_presets["ctrl"]["index"] >= len(self.trim_presets["ctrl"]["presets"]):
                self.trim_presets["ctrl"]["index"] = len(self.trim_presets["ctrl"]["presets"]) - 1
            self.cmd_set_trim(self.trim_presets["ctrl"]["presets"][self.trim_presets["ctrl"]["index"]])
            self.two_buttons_pressed = True

        if (msg_dict["buttons"] == (key_map["LB"] | key_map["DIGITAL_DOWN"])) and not self.two_buttons_pressed:
            self.trim_presets["ctrl"]["index"] -= 1
            if self.trim_presets["ctrl"]["index"] < 0:
                self.trim_presets["ctrl"]["index"] = 0
            self.cmd_set_trim(self.trim_presets["ctrl"]["presets"][self.trim_presets["ctrl"]["index"]])
            self.two_buttons_pressed = True

        # Idle joy reset
        if not msg_dict["buttons"] and self.one_button_pressed:
            self.one_button_pressed = False

        if (msg_dict["buttons"] == 0) or (msg_dict["buttons"] == key_map["RB"]) or (msg_dict["buttons"] == key_map["LB"]) and self.two_buttons_pressed:
            self.two_buttons_pressed = False

    def communicate(self, outputs):
        for o in outputs:
            if o["type"] == "pwm":

                time_boot_ms = 0
                port = 0
                (chan1_raw, chan2_raw,
                 chan3_raw, chan4_raw,
                 chan5_raw, chan6_raw,
                 chan7_raw, chan8_raw) = o["value"]
                rssi = 0

                self.master.mav.rc_channels_raw_send(
                    time_boot_ms,
                    port,
                    chan1_raw,
                    chan2_raw,
                    chan3_raw,
                    chan4_raw,
                    chan5_raw,
                    chan6_raw,
                    chan7_raw,
                    chan8_raw,
                    rssi
                )

            else:
                pprint(o)


def init(mpstate, **kwargs):
    '''initialise module'''
    return Fishi(mpstate, **kwargs)
