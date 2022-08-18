#!/bin/bash

DEMO_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BAGFILE="${DEMO_DIR}/rosbags/demo_1_static_rope.bag"
RVIZ_FILE="${DEMO_DIR}/../cdcpd/rviz/kinect_tripodB.rviz"

# Launch ROS core
#roscore &

# Launch rviz
# We launch in the background so it doesn't prevent the script from executing the rosbag play
# command
rviz -d "${RVIZ_FILE}" &

# Start playback of rosbag
# The -d 3 option adds a 3 second delay to playing the rosbag. This compensates for rviz startup
# time
rosbag play $BAGFILE -d 7 -l
