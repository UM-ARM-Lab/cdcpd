#!/bin/bash
source /home/deformtrack/miniconda3/bin/activate /home/deformtrack/miniconda3/envs/catkin_cdcpd
rosrun cdcpd cdcpd_node.py
exec "$@"