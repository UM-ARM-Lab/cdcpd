#/bin/bash
ROS_VERSION=${1:-kinetic}
BUILD_TYPE=${2:-Release}

. /opt/ros/$ROS_VERSION/setup.bash
. /root/catkin_ws/devel_isolated/setup.bash

cd ~/catkin_ws
catkin_make_isolated -DCMAKE_BUILD_TYPE=$BUILD_TYPE --catkin-make-args run_tests && catkin_test_results
