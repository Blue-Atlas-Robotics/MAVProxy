#!/usr/bin/env python
'''
Fishi Module
Oleksandr Slovak, August 2019
'''

import time
import traceback

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

class fishi(mp_module.MPModule):

    toggle = True

    prev_time = time.time()

    def __init__(self, mpstate):
        """Initialise module"""
        super(fishi, self).__init__(mpstate, "fishi", public=True, multi_vehicle=True)

        self.msgs = {}

        self.fishi_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('fishi', self.cmd_fishi, "fishi module", ['status','set (LOGSETTING)'])

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

        # self.master.mav.command_long_send(
        #     self.target_system,  # target_system
        #     self.target_component,
        #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
        #     0, # confirmation
        #     1, # param1 (1 to indicate arm)
        #     p2, # param2  (all other params meaningless)
        #     0, # param3
        #     0, # param4
        #     0, # param5
        #     0, # param6
        #     0) # param7

        pass

    def mavlink_packet_slave(self, m):
        '''handle mavlink packets from slave connections'''
        if self.toggle:
            if m.get_type() == "MANUAL_CONTROL":
                time_now = time.time()
                print(1/(time_now - self.prev_time), end="\t")
                print([m.get_srcSystem(), m.get_srcComponent(), m.get_seq()])

                self.prev_time = time_now

                for l in traceback.format_stack():
                    print(l.strip())

def init(mpstate):
    '''initialise module'''
    return fishi(mpstate)
