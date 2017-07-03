#!/usr/bin/env python
'''
Survey Manager module
George Zogopoulos
June 2017

Module for managing multiple survey missions and POIs

'''

import os
import os.path
import sys
from pymavlink import mavutil, mavwp
import errno
import time

try:
    from ..lib.wx_loader import wx
except ImportError:
    import wx
except ValueError:
    import wx
else:
    print("Unknown error code")

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

import multiprocessing
from MAVProxy.modules.lib.multiprocessing_queue import makeIPCQueue

from surveyManagerFrame import SurveyFrame
import sm_event as sme
from surveyCreator import SurveyCreator

class SurveyManagerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(SurveyManagerModule, self).__init__(mpstate, "survey_manager", "Survey Manager", public=True)

        self.verbose = False

        self.mission_memory = [mavwp.MAVWPLoader() for count in xrange(10)]
        self.current_waypoint_memory = 10*[1]
        self.input_mission_buffer = []  # Fills up with a temporary mission, as a list of waypoints
        self.current_waypoint_temp = 1
        self.num_wps_expected = 0
        self.recv_wp_index = {}
        self.event_queue = makeIPCQueue()  # Create the event queue, between GUI and backend
        self.gui_queue = makeIPCQueue()  # Create a queue from the backend to the GUI
        self.GUI = multiprocessing.Process(target=self.create_gui, args=(self.event_queue, self.gui_queue))  # Create and start the GUI in a separate process
        self.survey_creator = SurveyCreator(self.target_system, self.target_component)

        # GUI interface
        self.selected_memory_slot = 0
        self.loaded_mission_name = None
        self.survey_altitude = 100
        self.POI_coordinates = None
        self.survey_name = "MyPOI"
        self.survey_pattern = "Clover"

        self.update_interval = 1  # seconds
        self.last_update = time.time()

        self.module_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
              ('scale', float, 50),
          ])
        self.add_command('survey', self.cmd_example, "survey manager module", ['status', 'set (parameter pair)', 'show'])

        self.GUI.start()

    def usage(self):
        '''show help on command line options'''
        return "Usage: example <status|set>"

    def cmd_example(self, args):
        """control behaviour of the module"""
        if len(args) == 0:
            print self.usage()
        elif args[0] == "status":
            print self.status()
        elif args[0] == "set":
            self.module_settings.command(args[1:])
        elif args[0] == "show":
            print "Queried memory slot %d" % args[1]
        else:
            print self.usage()

    def status(self):
        """returns information about module"""
        self.last_update = time.time() # status entertains us
        return("""Stored missions: %d\n
            Currently selected memory slot %d\n
            """ % (len(self.mission_memory), self.selected_memory_slot))

    def unload(self):
        '''unload module'''
        if self.GUI.is_alive():
            self.GUI.join(1)
        self.GUI.terminate()
        self = None

    def create_gui(self, event_queue, gui_queue):
        mp_util.child_close_fds()

        self.app = wx.App(False)
        self.app.SetExitOnFrameDelete(True)
        self.app.frame_1 = SurveyFrame(event_queue, gui_queue)
        self.app.SetTopWindow(self.app.frame_1)
        self.app.frame_1.Show()
        self.app.MainLoop()

    def event_count(self):
        """return number of events waiting to be processed"""
        return self.event_queue.qsize()

    def get_event(self):
        '''return next event or None'''
        if self.event_queue.qsize() == 0:
            return None
        return self.event_queue.get()

    def process_event(self, event):
        '''Top-level method for processing events'''
        event_type = event.get_type()
        print("processing new event of type %d" % event_type)
        if event_type == sme.SM_DOWNLOAD:
            self.mission_memory[self.selected_memory_slot].clear()  # Clear the current memory for re-filling
            self.mpstate.module('wp').cmd_wp(['list'])
            # means I'm doing a read & don't know how many wps to expect:
            self.num_wps_expected = -1
            self.recv_wp_index = {}
            self.current_waypoint_memory[self.selected_memory_slot] = self.current_waypoint_temp

        elif event_type == sme.SM_UPLOAD:
            self.mpstate.module('wp').wploader.clear()
            print "Clearing wps in wp module"
            num_wps = self.mission_memory[self.selected_memory_slot].count()

            for i in xrange(num_wps):
                print "Adding wp %d to wp module" % i
                self.mpstate.module('wp').wploader.add(self.mission_memory[self.selected_memory_slot].wp(i))

            print("Telling wp module to send all waypoints")
            self.mpstate.module('wp').send_all_waypoints()

            # Restore the current waypoint
            self.master.waypoint_set_current_send(self.current_waypoint_memory[self.selected_memory_slot])

            self.num_wps_expected = num_wps  # Is this needed?
            self.wps_received = {}

            # Inform the local SmartCamera module of the new POI
            # This is called separately because MAVProxy modules do not intercept MAVLink messages derived from the same
            # MAVProxy instance
            if self.module('SmartCamera') is not None:
                self.module('SmartCamera').update_POI(self.POI_coordinates)

        elif event_type == sme.SM_CREATE:
            self.input_mission_buffer = self.survey_creator.create_survey(self.POI_coordinates, self.survey_altitude, self.survey_pattern)
            if self.input_mission_buffer is None:
                self.console.error("Invalid survey pattern type")
            self.mission_memory[self.selected_memory_slot].clear()
            home = self.mpstate.module('wp').get_home()
            self.mission_memory[self.selected_memory_slot].add(home)
            for wp in self.input_mission_buffer:
                self.mission_memory[self.selected_memory_slot].add(wp)
            self.gui_queue.put(sme.SurveyManagerEvent(sme.SM_SET_SLOT_NAME, index=self.selected_memory_slot, value=self.survey_name))

        elif event_type == sme.SM_SET_MEMORY_SLOT:
            self.selected_memory_slot = event.get_arg('value')
            print (self.mission_memory[self.selected_memory_slot].view_list())

        elif event_type == sme.SM_SET_SURVEY_NAME:
            self.survey_name = event.get_arg('value')

        elif event_type == sme.SM_SET_ALTITUDE:
            self.survey_altitude = event.get_arg('value')

        elif event_type == sme.SM_SET_COORDINATES:
            self.POI_coordinates = (event.get_arg('lat'), event.get_arg('lon'))
            self.gui_queue.put(sme.SurveyManagerEvent(sme.SM_SET_POI, coords=self.POI_coordinates))

        elif event_type == sme.SM_SET_SURVEY_PATTERN:
            self.survey_pattern = event.get_arg('value')
        else:
            print "Unhandled event type %d!" % event_type

    def update_POI(self, click_position):
        '''update the center point for new surveys'''
        # print "Got new center position"
        # print click_position
        self.POI_coordinates = click_position
        self.gui_queue.put(sme.SurveyManagerEvent(sme.SM_SET_POI, coords=self.POI_coordinates))

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_update > self.update_interval:
            self.last_update = now
            # Check the event queue
            # Forward events to the corresponding processor

            while self.event_count() > 0:
                # Iterate over the event queue
                new_event = self.event_queue.get()
                if new_event:
                    self.process_event(new_event)

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        mtype = m.get_type()

        if mtype in ['WAYPOINT_COUNT', 'MISSION_COUNT']:
            # I haven't asked for WPs, or duplicates have been transmitted
            if (self.num_wps_expected == 0):
                self.console.error("Received unexpected MISSION_COUNT, perhaps download triggered by another module")
            # I was expecting the mission item count
            elif (self.num_wps_expected == -1):
                self.num_wps_expected = m.count
                self.recv_wp_index = {}
                print "Received number of mission items (%d)" % self.num_wps_expected
            # write has been sent by the mission editor (?)
            elif (self.num_wps_expected > 1):
                if (m.count != self.num_wps_expected):
                    self.console.error("Unexpected waypoint count from APM after write (Editor)")
                # since this is a write operation from the Editor there
                # should be no need to update number of table rows

        elif mtype in ['WAYPOINT', 'MISSION_ITEM']:
            # Handle mission_item mavlink packets
            if (len(self.recv_wp_index) < self.num_wps_expected):  # TODO: A better test is needed
                #if we haven't already received this wp, store it to memory
                if (m.seq not in self.recv_wp_index.keys()):
                    print "Received an expected mission item"
                    if m.seq==0:
                        # The HOME position is transmitted
                        print "The home position has been received"
                    else:
                        print "A regular mission item has been received"
                    self.mission_memory[self.selected_memory_slot].add(m)
                    # self.gui_event_queue.put(MissionEditorEvent(
                    #     me_event.MEGE_SET_MISS_ITEM,
                    #     num=m.seq,command=m.command,param1=m.param1,
                    #     param2=m.param2,param3=m.param3,param4=m.param4,
                    #     lat=m.x,lon=m.y,alt=m.z,frame=m.frame))

                    self.recv_wp_index[m.seq] = True
            else:
                print "Received mission item without requesting it (type=%s, seq=%d)" % (m.get_type(), m.seq)

        elif mtype in ['WAYPOINT_CURRENT', 'MISSION_CURRENT']:
        # Store the current waypoint for this mission, so as to resume there
            self.current_waypoint_temp = m.seq

def init(mpstate):
    '''initialise module'''
    return SurveyManagerModule(mpstate)
