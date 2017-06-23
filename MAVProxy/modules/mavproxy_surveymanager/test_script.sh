#!/bin/sh

# Have an instance of ArduPlane SITL running and outputing a MAVLink stream @localhost:14550

cd /home/george/EMSA/MAVProxy
python setup.py build install --user --record installed_files.txt
mavproxy.py --master=udp:127.0.0.1:14550 --moddebug=3 --load-module surveymanager --load-module map --cmd="wp load /home/george/EMSA/sm_mission.txt"