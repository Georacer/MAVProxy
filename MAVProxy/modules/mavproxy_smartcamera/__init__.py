#!/usr/bin/env python
#***************************************************************************
#                      Copyright Jaime Machuca
#***************************************************************************
# Title        : mavproxy_smartcamera.py
#
# Description  : This file is intended to be added as a module to MAVProxy,
#                it is intended to be used to control smart cameras that are
#                connected to a companion computer. It reads MAVlink commands
#                and uses them to control the cameras attached. The module
#                reads a configuration file called smart_camera.cnf that tells
#                it what cameras are connected, it then tries to connect to the
#                cameras and populates a list of available cameras.
#
# Environment  : Python 2.7 Code. Intended to be included in MAVproxy as a Module
#
# Responsible  : Jaime Machuca
#
# License      : GNU GPL version 3
#
# Editor Used  : Xcode 6.1.1 (6A2008a)
#
#****************************************************************************

#****************************************************************************
# HEADER-FILES (Only those that are needed in this file)
#****************************************************************************

# System Header files and Module Headers
import time, math, sched, threading, os

# Module Dependent Headers
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib.mp_settings import MPSetting
from MAVProxy.modules.lib.mp_util import gps_distance

# Own Headers
from sc_webcam import SmartCameraWebCam
from sc_SonyQX1 import SmartCamera_SonyQX
from sc_fakecamera import SmartCameraFakeCamera
from sc_Sequoia import SmartCameraSequoia
import sc_config

#****************************************************************************
# LOCAL DEFINES
#****************************************************************************


#****************************************************************************
# Class name       : SmartCameraModule
#
# Public Methods   : init
#                    mavlink_packet
#
# Private Methods  : __vRegisterCameras
#                    __vCmdCamTrigger
#
#****************************************************************************
class SmartCameraModule(mp_module.MPModule):

#****************************************************************************
#   Method Name     : __init__ Class Initializer
#
#   Description     : Initializes the class
#
#   Parameters      : mpstate
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __init__(self, mpstate):
        super(SmartCameraModule, self).__init__(mpstate, "SmartCamera", "SmartCamera commands", public=True)
        self.add_command('camtrigger', self.__vCmdCamTrigger, "Trigger camera")
        self.add_command('connectcams', self.__vCmdConnectCameras, "Connect to Cameras")
        self.add_command('setCamISO', self.__vCmdSetCamISO, "Set Camera ISO")
        self.add_command('setCamAperture', self.__vCmdSetCamAperture, "Set Camera Aperture")
        self.add_command('setCamShutterSpeed', self.__vCmdSetCamShutterSpeed, "Set Camera Shutter Speed")
        self.add_command('setCamExposureMode', self.__vCmdSetCamExposureMode, "Set Camera Exposure Mode")
        self.add_command('getAllPictures', self.__vCmdGetAllPictures, "Download all flight pictures, filename as argument optional")
        self.CamRetryScheduler = sched.scheduler(time.time, time.sleep)
        self.ProgramAuto = 1
        self.Aperture = 2
        self.Shutter = 3
        self.Manual = 4
        self.IntelligentAuto = 5
        self.SuperiorAuto = 6
        self.WirelessPort = sc_config.config.get_string("general", 'WirelessPort', "wlan0")
        self.u8RetryTimeout = 0
        self.u8MaxRetries = 5
        self.outputPath = self.__vConfigureOutputPath() # Set where images should be saved
        self.tLastCheckTime = time.time()
        self.tLastTriggerTime = time.time()
        self.u8KillHeartbeatTimer = 100
        self.__vRegisterCameras()

        self.POI_coordinates = 2*[None]
        self.trigger_distance_threshold = 100
        self.lat_lon_alt = 3*[None]

        self.mpstate = mpstate

        self.debug = False

        # Start a 10 second timer to kill heartbeats as a workaround
        # threading.Timer(10, self.__vKillHeartbeat).start()

#****************************************************************************
#   Method Name     : __vKillHeartbeat
#
#   Description     : Sets heartbeat setting to 0 to stop sending heartbeats
#                     this is a temporary workaround for systems that do not
#                     properly interpret the heartbeat contents like 3DR Solo
#                     in such systems the heartbeats from the camera controller
#                     and the main system are confused causing potential issues
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vKillHeartbeat(self):
        print("Killing Heartbeat - Solo Workaround")
        self.mpstate.settings.heartbeat = 0

#****************************************************************************
#   Method Name     : __vConfigureOutputPath
#
#   Description     : Configure the output path for saving captured images, if
#                     one is provided in the configuration file
#
#   Parameters      : None
#
#   Return Value    : Image output path
#
#   Author          : George Zogopoulos
#
#****************************************************************************
    def __vConfigureOutputPath(self):
        print("Reading output path from smart_camera.cnf")
        path = sc_config.config.get_string("general","outputPath",None)
        if path == None: # No path specified
            print("No output path specified")
        # nop
        else:
            path = os.path.expanduser(path)
            if not os.path.exists(path):
                os.makedirs(path)
        print("Output path set to %s" % path)
        return path

#****************************************************************************
#   Method Name     : __vRegisterQXCamera
#
#   Description     : Tries to connect to a QX camera on the specified Wireless
#                     port. If no camera is found it will retry every 5 seconds
#                     until u8MaxRetries is reached.
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vRegisterQXCamera(self,u8CamNumber):
        if (self.u8RetryTimeout < self.u8MaxRetries):
            new_camera = SmartCamera_SonyQX(u8CamNumber, self.WirelessPort)
            if new_camera.boValidCameraFound() is True:
                self.camera_list = self.camera_list + [new_camera]
                print("Found QX Camera")
                self.master.mav.statustext_send(6,"Camera Controller: Found QX Camera, Ready to Fly")
            else:
                print("No Valid Camera Found, retry in 5 sec")
                self.u8RetryTimeout = self.u8RetryTimeout + 1
                self.CamRetryScheduler.enter(5, 1, self.__vRegisterQXCamera, [u8CamNumber])
                self.CamRetryScheduler.run()
        else:
            print("Max retries reached, No QX Camera Found")
            self.master.mav.statustext_send(3,"Camera Controller: Warning! Camera not found")
            self.u8RetryTimeout = 0

#****************************************************************************
#   Method Name     : __vRegisterCameras
#
#   Description     : Creates camera objects based on camera-type configuration
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vRegisterCameras(self):

        # initialise list
        self.camera_list = []

        #look for up to 2 cameras
        for i in range(0,2):
            config_group = "camera%d" % i
            camera_type = sc_config.config.get_integer(config_group, 'type', 0)
            # webcam
            if camera_type == 1:
                new_camera = SmartCameraWebCam(i)
                self.camera_list = self.camera_list + [new_camera]

            # Sony QX1
            if camera_type == 2:
                self.__vRegisterQXCamera(i)

            # fake camera
            if camera_type == 3:
                new_camera = SmartCameraFakeCamera(i)
                self.camera_list = self.camera_list + [new_camera]

            # Parrot Sequoia
            if camera_type == 4:
                new_camera = SmartCameraSequoia(i)
                self.camera_list = self.camera_list + [new_camera]


        # display number of cameras found
        print ("cameras found: %d" % len(self.camera_list))

#****************************************************************************
#   Method Name     : __vCmdCamTrigger
#
#   Description     : Triggers all the cameras and stores Geotag information.
#                    Also saves the images if an output directory has been
#                    provided
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamTrigger(self, args=[]):
        '''Trigger Camera'''
        #print(self.camera_list)
        for cam in self.camera_list:
            cam.take_picture()
            if self.debug:
                print("Trigger Cam %s" % cam)
            if self.outputPath is not None:
                try:
                    cam.save_picture(self.outputPath)
                    if self.debug:
                        print("Saved image from Cam %s" % cam)
                except:
                    print("Could not save image from Cam %s" % cam)

#****************************************************************************
#   Method Name     : __vCmdConnectCameras
#
#   Description     : Initiates connection to cameras
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdConnectCameras(self, args):
        '''ToDo: Validate the argument as a valid port'''
        if len(args) >= 1:
            self.WirelessPort = args[0]
        print ("Connecting to Cameras on %s" % self.WirelessPort)
        self.__vRegisterCameras()

#****************************************************************************
#   Method Name     : __vCmdSetCamExposureMode
#
#   Description     : Sets the camera exposure mode
#
#   Parameters      : Exposure Mode, Cam number
#                     Valid values are Program Auto, Aperture, Shutter, Manual
#                     Intelligent Auto, Superior Auto
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamExposureMode(self, args):
        '''ToDo: Validate CAM number and Valid Mode Values'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetExposureMode(args[0])
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetExposureMode(args[0])
        else:
            print ("Usage: setCamExposureMode MODE [CAMNUMBER], Valid values for MODE: Program Auto, Aperture, Shutter, Manual, Intelligent Auto, Superior Auto")

#****************************************************************************
#   Method Name     : __vCmdSetCamAperture
#
#   Description     : Sets the camera aperture
#
#   Parameters      : Aperture Value, Cam number
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamAperture(self, args):
        '''ToDo: Validate CAM number and Valid Aperture Value'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetAperture(int(args[0]))
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetAperture(int(args[0]))
        else:
            print ("Usage: setCamAperture APERTURE [CAMNUMBER], APERTURE is value x10")

#****************************************************************************
#   Method Name     : __vCmdSetCamShutterSpeed
#
#   Description     : Sets the shutter speed for the camera
#
#   Parameters      : Shutter speed, Cam Number
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamShutterSpeed(self, args):
        '''ToDo: Validate CAM number and Valid Shutter Speed'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetShutterSpeed(int(args[0]))
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetShutterSpeed(int(args[0]))
        else:
            print ("Usage: setCamShutterSpeed SHUTTERVALUE [CAMNUMBER], Shutter value is the devisor in 1/x (only works for values smaller than 1)")

#****************************************************************************
#   Method Name     : __vCmdSetCamISO
#
#   Description     : Sets the ISO value for the camera
#
#   Parameters      : ISO Value, Cam Number
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdSetCamISO(self, args):
        '''ToDo: Validate CAM number and Valid ISO Value'''
        if len(args) == 1:
            for cam in self.camera_list:
                cam.boSetISO(args[0])
        elif len(args) == 2:
            cam = self.camera_list[int(args[1])]
            cam.boSetISO(args[0])
        else:
            print ("Usage: setCamISO ISOVALUE [CAMNUMBER]")

#****************************************************************************
#   Method Name     : __vCmdCamZoomIn
#
#   Description     : Commands the Camera to Zoom In
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamZoomIn(self):
        for cam in self.camera_list:
            cam.boZoomIn()

#****************************************************************************
#   Method Name     : __vCmdCamZoomOut
#
#   Description     : Commands the Camera to Zoom In
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdCamZoomOut(self):
        for cam in self.camera_list:
            cam.boZoomOut()

#****************************************************************************
#   Method Name     : __vCmdGetAllPictures
#
#   Description     : Downloads all the pics taken during this flight
#
#   Parameters      : None
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vCmdGetAllPictures(self, args):

        #Download Pictures
        if len(args) >= 1:
            slogFileName = args[0]
            for cam in self.camera_list:
                print("Init Picture Download for Cam %s from file %s" % cam, slogFileName)
                cam.boGetAllSessionPictures(slogFileName)
        else:
            for cam in self.camera_list:
                print("Init Picture Download for Cam %s" % cam)
                cam.boGetAllSessionPictures(0)

#****************************************************************************
#   Method Name     : __vDecodeDIGICAMConfigure
#
#   Description     : Decode and process the camera configuration Messages
#
#   Parameters      : CommandLong Message
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vDecodeDIGICAMConfigure(self, mCommand_Long):
        if mCommand_Long.param1 != 0:
            print ("Exposure Mode = %d" % mCommand_Long.param1)

            if mCommand_Long.param1 == self.ProgramAuto:
                self.__vCmdSetCamExposureMode(["Program Auto"])

            elif mCommand_Long.param1 == self.Aperture:
                self.__vCmdSetCamExposureMode(["Aperture"])

            elif mCommand_Long.param1 == self.Shutter:
                self.__vCmdSetCamExposureMode(["Shutter"])

        '''Shutter Speed'''
        if mCommand_Long.param2 != 0:
            print ("Shutter Speed= %d" % mCommand_Long.param2)
            self.__vCmdSetCamShutterSpeed([mCommand_Long.param2])

        '''Aperture'''
        if mCommand_Long.param3 != 0:
            print ("Aperture = %d" % mCommand_Long.param3)
            self.__vCmdSetCamAperture([mCommand_Long.param3])

        '''ISO'''
        if mCommand_Long.param4 != 0:
            print ("ISO = %d" % mCommand_Long.param4)
            self.__vCmdSetCamISO([mCommand_Long.param4])

        '''Exposure Type'''
        if mCommand_Long.param5 != 0:
            print ("Exposure type= %d" % mCommand_Long.param5)


#****************************************************************************
#   Method Name     : __vDecodeDIGICAMControl
#
#   Description     : Decode and process the camera control Messages
#
#   Parameters      : CommandLong Message
#
#   Return Value    : None
#
#   Author           : Jaime Machuca
#
#****************************************************************************

    def __vDecodeDIGICAMControl(self, mCommand_Long):
        '''Session'''
        if mCommand_Long.param1 != 0:
            print ("Session = %d" % mCommand_Long.param1)

        '''Zooming Step Value'''
        if mCommand_Long.param2 != 0:
            print ("Zooming Step = %d" % mCommand_Long.param2)

        '''Zooming Step Value'''
        if mCommand_Long.param3 != 0:
            print ("Zooming Value = %d" % mCommand_Long.param3)

            if (mCommand_Long.param3 == 1):
                self.__vCmdCamZoomIn()
            elif (mCommand_Long.param3 == -1):
                self.__vCmdCamZoomOut()
            else:
                print ("Invalid Zoom Value")

        '''Focus 0=Unlock/1=Lock/2=relock'''
        if mCommand_Long.param4 != 0:
            print ("Focus = %d" % mCommand_Long.param4)

        '''Trigger'''
        if mCommand_Long.param5 != 0:
            print ("Trigger = %d" % mCommand_Long.param5)
            self.__vCmdCamTrigger(mCommand_Long)

# ****************************************************************************
#   Method Name     : update_POI
#
#   Description     : Called to set the POI coordinates
#
#   Parameters      : (lat,lon) float tuple
#
#   Return Value    : None
#
#   Author          : George Zogopoulos
#
# ****************************************************************************

    def update_POI(self, coordinates):
        '''update the Point Of Interest'''
        self.POI_coordinates[0] = coordinates[0]
        self.POI_coordinates[1] = coordinates[1]

#****************************************************************************
#   Method Name     : mavlink_packet
#
#   Description     : MAVProxy required callback function used to receive MAVLink
#                     packets
#
#   Parameters      : MAVLink Message
#
#   Return Value    : None
#
#   Author           : Jaime Machuca, George Zogopoulos
#
#****************************************************************************

    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        mtype = m.get_type()
        if self.debug: print("Got message %s" % mtype)
        if mtype == "GLOBAL_POSITION_INT":
            # Update the UAV position
            self.lat_lon_alt[0] = m.lat*1.0e-7
            self.lat_lon_alt[1] = m.lon*1.0e-7
            self.lat_lon_alt[2] = m.alt*0.001
            if self.debug: print("Got new position: " + "(%g, %g, %g)" % (self.lat_lon_alt[0], self.lat_lon_alt[1], self.lat_lon_alt[2]))
            for cam in self.camera_list:
                try:
                    cam.boSet_GPS(m)
                except:
                    pass
        if mtype == "ATTITUDE":
            for cam in self.camera_list:
                try:
                    cam.boSet_Attitude(m)
                except:
                    pass
        if mtype == "CAMERA_STATUS":
            if self.debug: print ("Got Message camera_status")
        if mtype == "CAMERA_FEEDBACK":
            if self.debug: print ("Got Message camera_feedback")
            '''self.__vCmdCamTrigger(m)'''
        if mtype == "COMMAND_LONG":
            if m.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE:
                print ("Got Message Digicam_configure")
                self.__vDecodeDIGICAMConfigure(m)
            elif m.command == mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL:
                print ("Got Message Digicam_control")
                self.__vDecodeDIGICAMControl(m)
        if mtype == "NAV_CONTROLLER_OUTPUT":
            if self.debug: print("%dm away from next waypoint" % m.wp_dist)
        if mtype in ['WAYPOINT', 'MISSION_ITEM']:
            if m.command == mavutil.mavlink.MAV_CMD_DO_SET_ROI: # We intercepted a ROI message
                if self.debug: print("Intercepted ROI mission item with coordinates (%g, %g)" % (m.x, m.y))
                POI = (m.x, m.y)
                self.update_POI(POI)

#****************************************************************************
#   Method Name     : idle_task
#
#   Description     : used for heartbeat work around timer
#
#   Parameters      : none
#
#   Return Value    : none
#
#   Author           : Jaime Machuca, George Zogopoulos
#
#****************************************************************************

    def idle_task(self):
        now = time.time()
        # Kill heartbeat routine
        if not self.u8KillHeartbeatTimer == 0 and self.tLastCheckTime > 1:
            self.tLastCheckTime = now
            self.u8KillHeartbeatTimer -= 1
            if self.u8KillHeartbeatTimer == 0:
                self.__vKillHeartbeat()

        # Check distance from POI and trigger routine
        if (now-self.tLastTriggerTime > 2) and (self.POI_coordinates[0] is not None):
            self.tLastTriggerTime = now
            # Calculate distance from POI-target
            distance = gps_distance(self.POI_coordinates[0], self.POI_coordinates[1], self.lat_lon_alt[0], self.lat_lon_alt[1])
            if distance < self.trigger_distance_threshold:
                self.__vCmdCamTrigger()
            if self.debug: print("Dinstance from target: %gm" % distance)



#****************************************************************************
#   Method Name     : init
#
#   Description     :
#
#   Parameters      : mpstate
#
#   Return Value    : SmartCameraModule Instance
#
#   Author           : Jaime Machuca
#
#****************************************************************************

def init(mpstate):
    '''initialise module'''
    return SmartCameraModule(mpstate)
