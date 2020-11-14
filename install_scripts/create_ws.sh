#!/usr/bin/env bash

WS_PATH=~/catkin_ws

source /opt/ros/melodic/setup.bash
mkdir -p $WS_PATH/src
cd $WS_PATH
catkin build
source devel/setup.bash
catkin_init_workspace

