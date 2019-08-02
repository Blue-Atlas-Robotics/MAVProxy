#!/usr/bin/env python
'''
Fishi Module
Peter Barker, September 2016
'''

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from pprint import pprint


class fishi(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(fishi, self).__init__(mpstate, "fishi", "")

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
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        return ""

    def boredom_message(self):
        if self.fishi_settings.verbose:
            return ("I'm very bored")
        return ("I'm bored")

    def idle_task(self):
        '''called rapidly by mavproxy'''
        pass

    def mavlink_packet(self, m):
        '''handle mavlink packets'''

        if m.get_type() not in self.msgs.keys():
            self.msgs[m.get_type()] = None

        elif m.get_type() == 'HEARTBEAT':
            print(self.msgs)


def init(mpstate):
    '''initialise module'''
    return fishi(mpstate)
