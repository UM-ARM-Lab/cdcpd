#!/bin/bash

FAKE_ROBOT_URDF_PATH="/home/dylan/catkin_ws/src/cdcpd/cdcpd/fake_robot.urdf"
FAKE_ROBOT_SRDF_PATH="/home/dylan/catkin_ws/src/cdcpd/cdcpd/fake_robot.srdf"

# Set rosparams.
rosparam set fake_robot_description "$(cat ${FAKE_ROBOT_URDF_PATH})"
rosparam set fake_robot_description_semantic "$(cat ${FAKE_ROBOT_SRDF_PATH})"

# Change directory so that we're working from the same working directory everytime
cd /home/dcolli23/catkin_ws/src/cdcpd/cdcpd
echo "Executing from this directory: $(pwd)"

# Launch RVIZ
rviz -d rviz/mesh_collision_debugging.rviz &
sleep 2

/home/dylan/catkin_ws/devel/lib/cdcpd/sdformat_to_planning_scene
