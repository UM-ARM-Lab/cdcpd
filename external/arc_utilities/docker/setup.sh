#/bin/bash
ROS_VERSION=${1:-kinetic}
BUILD_TYPE=${2:-Release}

. /opt/ros/$ROS_VERSION/setup.bash

echo $(rosversion -d)

cd ~/catkin_ws

# Note: As this script is intended for the CI image, things are run without sudo
apt update
rosdep init
rosdep update
rosdep install --as-root "apt:false pip:false" --from-paths src --ignore-src -r -y

catkin_make_isolated -DCMAKE_BUILD_TYPE=$BUILD_TYPE
