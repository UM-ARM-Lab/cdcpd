#!/usr/bin/env bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/neotic/setup.bash" >> ~/.bashrc
sudo apt-get install -y librospack-dev libclass-loader-dev libroslz4-dev libactionlib-dev libxmlrpcpp-dev librosconsole-dev
source ~/.bashrc
source /opt/ros/noetic/setup.bash
