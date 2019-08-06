#!/usr/bin/env python
'''
Fishi Module
Oleksandr Slovak, August 2019
'''

import time

from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

msg_types_master = {
    # 'RC_CHANNELS',
    # 'SCALED_PRESSURE3',
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
    # 'EKF_STATUS_REPORT',
    # 'AHRS3',
    'SENSOR_OFFSETS',
    # 'GPS_GLOBAL_ORIGIN',
    # 'SYS_STATUS',
    # 'AHRS2',
    # 'SERVO_OUTPUT_RAW',
    # 'AHRS',
    # 'RAW_IMU',
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


class Loop:

    message_handlers = {}

    def set_input(self, msgs):
        pass

    def get_output(self):
        return []


class fishi(mp_module.MPModule):
    toggle = True

    prev_time = time.time()
    prev_seq = 0

    messages = {
        "robot": {t: None for t in msg_types_master},
        "qgroundcontrol": {t: None for t in msg_types_gcs}
    }

    loop = Loop()

    def __init__(self, mpstate):
        """Initialise module"""
        super(fishi, self).__init__(mpstate, "fishi", public=True, multi_vehicle=True)

        self.msgs = {}

        self.fishi_settings = mp_settings.MPSettings(
            [('verbose', bool, False),
             ])
        self.add_command('fishi', self.cmd_fishi, "fishi module", ['status', 'set (LOGSETTING)'])

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
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        return str(self.mpstate.sysid_outputs)

    def cmd_toggle(self):
        '''returns information about module'''
        self.toggle = not self.toggle

    def idle_task(self):
        '''called rapidly by mavproxy'''
        # TODO, think how to utilize that
        pass

    def mavlink_packet(self, m):
        '''handle mavlink packets'''

        msg_type = m.get_type()
        if m.get_type() in self.messages["robot"].keys():
            self.messages["robot"][msg_type] = m.to_dict()

        if m.get_type() == "HEARTBEAT":
                print(self.messages)

        if m.get_type() == "ATTITUDE":
            self.loop.set_input(self.messages)
            self.communicate(self.loop.get_output())

    def mavlink_packet_slave(self, m):
        '''handle mavlink packets from slave connections'''

        msg_type = m.get_type()
        if m.get_type() in self.messages["qgroundcontrol"].keys():
            self.messages["qgroundcontrol"][msg_type] = m.to_dict()

        # TODO handle doubling messages correctly
        # if self.toggle:
        #     if m.get_type() == "MANUAL_CONTROL":
        #         time_now = time.time()
        #         seq_now = m.get_seq()
        #         if seq_now == self.prev_seq:
        #             pass  # Skip dublicates if any
        #         else:
        #             print(1/(time_now - self.prev_time))
        #             self.prev_time = time_now
        #             self.prev_seq = seq_now

    def communicate(self, outputs):
        for o in outputs:
            if o["type"] == "pwm":
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_LAST,
                    0,  # confirmation
                    0, *o["pwms"])


def init(mpstate):
    '''initialise module'''
    return fishi(mpstate)
