#!/usr/bin/env bash

#only needed if a program asks for X11
#export XAUTHORITY=/home/joseph/.Xauthority 
#export DISPLAY=:0.0
source /root/RoboCar/ROS_Workspace/devel/setup.bash
exec "$@"
