#!/usr/bin/env bash

export ROS_MASTER_URI=http://pi:11311/
source /root/robocar/ROS_Workspace/devel/setup.bash
exec "$@"
