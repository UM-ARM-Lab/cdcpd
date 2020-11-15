#!/bin/bash

LOG_FOLDER=~/Dropbox/catkin_ws/src/smmap/logs/cloth_table/
TEST_ID=optimization_time_runs
mkdir ${LOG_FOLDER}${TEST_ID}
roslaunch smmap generic_experiment.launch \
    task_type:=cloth_table \
    test_id:=${TEST_ID}\
    logging_enabled:=true \
    start_bullet_viewer:=false \
    screenshots_enabled:=false \
    multi_model:=true \
    bandit_algoritm:=KFMANDB \
    calculate_regret:=true \
    use_random_seed:=false \
    static_seed_override:=true \
    static_seed:=147ba3bc969ecbf4 \
    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

LOG_FOLDER=~/Dropbox/catkin_ws/src/smmap/logs/rope_cylinder/
TEST_ID=optimization_time_runs
mkdir ${LOG_FOLDER}${TEST_ID}
roslaunch smmap generic_experiment.launch \
    task_type:=rope_cylinder \
    test_id:=${TEST_ID}\
    logging_enabled:=true \
    start_bullet_viewer:=false \
    screenshots_enabled:=false \
    multi_model:=true \
    bandit_algoritm:=KFMANDB \
    calculate_regret:=true \
    use_random_seed:=false \
    static_seed_override:=true \
    static_seed:=147ba3bc969ecbf4 \
    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt