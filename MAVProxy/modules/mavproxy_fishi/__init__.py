#!/usr/bin/env python
'''
Fishi Module
Oleksandr Slovak, August 2019
'''

import os
import time
from pprint import pprint

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pyfishi import ctrl
from pymavlink import mavutil

config_path = os.path.abspath(
    os.path.join(os.path.dirname(ctrl.__file__), "..", "config", "brov2_original.urdf"))

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
    'EKF_STATUS_REPORT',
    # 'AHRS3',
    'SENSOR_OFFSETS',
    # 'GPS_GLOBAL_ORIGIN',
    # 'SYS_STATUS',
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


class Fishi(mp_module.MPModule):
    live_log_toggle = False
    button_pressed = False

    prev_time = time.time()
    prev_seq = 0

    messages = {
        "master": {t: None for t in msg_types_master},
        "GCS": {t: None for t in msg_types_gcs},
        "opt": {"seq": 0, "name": None, "idx": None, "value": None},
        "cmd": {"terminate": False},
    }

    messages_seq = {
        "master": {t: 0 for t in msg_types_master},
        "GCS": {t: 0 for t in msg_types_gcs}
    }

    def __init__(self, mpstate):
        """Initialise module"""
        super(Fishi, self).__init__(mpstate, "fishi", public=True, multi_vehicle=True)

        self.fishi_settings = mp_settings.MPSettings([
            ('verbose', bool, False),
        ])
        self.add_command('fishi', self.cmd_fishi, "fishi module", ['status', 'set (LOGSETTING)'])

        # self.master.set_mode(20)  # RAW, it will work even if pymavlink does not have a mode mapping updated
        self.master.set_mode(19)  # RAW, it will work even if pymavlink does not have a mode mapping updated

        self.control_loop = ctrl.Control(config_path, log_folder_path=mpstate.status.logdir)

    def unload(self):
        print("\n")
        self.messages["cmd"]["terminate"] = True
        self.control_loop.set_input(self.messages)
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
        elif args[0] == "toggle":
            self.cmd_toggle()
        elif args[0] == "opt":
            self.cmd_opt(args)
        elif args[0] == "rotate":
            self.control_loop.rotate_log.set()
        else:
            print(self.usage())

    def cmd_opt(self, args):

        if len(args) != 4:
            self.messages["opt"]["name"] = "test"
            self.messages["opt"]["idx"] = 0
            self.messages["opt"]["value"] = 0.0

            self.messages["opt"]["seq"] = self.messages["opt"]["seq"] + 1
        else:

            self.messages["opt"]["name"] = args[1]
            self.messages["opt"]["idx"] = int(args[2])
            self.messages["opt"]["value"] = float(args[3])

            self.messages["opt"]["seq"] = self.messages["opt"]["seq"] + 1

        print(self.messages["opt"])

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

        if m.get_type() == "ATTITUDE":
            self.control_loop.set_input(self.messages)
            self.communicate(self.control_loop.get_outputs())

    def mavlink_packet_slave(self, m):
        '''handle mavlink packets from slave connections'''

        seq_now = m.get_seq()
        msg_type = m.get_type()
        msg_dict = m.to_dict()

        if msg_type in self.messages["GCS"].keys() and seq_now != self.messages_seq["GCS"][msg_type]:
            self.messages["GCS"][msg_type] = msg_dict
            self.messages_seq["GCS"][msg_type] = seq_now

        if msg_type == "MANUAL_CONTROL":
            if (msg_dict["buttons"] & 1 << 9) and not self.button_pressed:
                self.live_log_toggle = not self.live_log_toggle
                self.button_pressed = True

            if (msg_dict["buttons"] & 1) and not self.button_pressed:
                self.master.set_mode(20)
                self.button_pressed = True

            if not msg_dict["buttons"] and self.button_pressed:
                self.button_pressed = False

    def communicate(self, outputs):
        for o in outputs:
            if o["type"] == "pwm":
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_LAST,
                    False,  # confirmation
                    0, *o["pwms"])
            else:
                pprint(o)


def init(mpstate):
    '''initialise module'''
    return Fishi(mpstate)
