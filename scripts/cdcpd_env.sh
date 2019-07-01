#!/bin/bash
source ~/miniconda3/bin/activate ~/miniconda3/envs/catkin_cdcpd
rosrun cdcpd cdcpd_visualize.py
exec "$@"