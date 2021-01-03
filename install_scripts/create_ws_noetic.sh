#!/usr/bin/env bash

WS_PATH=~/catkin_ws

source /opt/ros/noetic/setup.bash
mkdir -p $WS_PATH/src
cd $WS_PATH
catkin_init_workspace
catkin build
source devel/setup.bash

