#!/usr/bin/env bash

WS_PATH=~/catkin_ws

mkdir -p $WS_PATH/src
cd $WS_PATH
catkin_init_workspace
catkin build
echo "source $WS_PATH/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
