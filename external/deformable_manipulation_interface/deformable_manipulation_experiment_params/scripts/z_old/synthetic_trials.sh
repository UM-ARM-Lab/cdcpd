#!/bin/bash
LOG_FOLDER=/home/dmcconachie/Dropbox/catkin_ws/src/smmap/logs/kalman_synthetic_trials
rosrun smmap kalman_filter_synthetic_trials 100 1000 10    3  2 > $LOG_FOLDER/SMALL_result_100_trials_1000_pulls.txt
rosrun smmap kalman_filter_synthetic_trials 100 1000 60  147  6 > $LOG_FOLDER/MEDIUM_result_100_trials_1000_pulls.txt
rosrun smmap kalman_filter_synthetic_trials 100 1000 60 6075 12 > $LOG_FOLDER/LARGE_result_100_trials_1000_pulls.txt
