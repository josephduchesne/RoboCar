#!/usr/bin/env bash

export ROS_MASTER_URI=http://pi:11311/
export ROS_PACKAGE_PATH=/root/RoboCar/ROS_Workspace/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
source /root/RoboCar/ROS_Workspace/devel/setup.bash
exec "$@"
