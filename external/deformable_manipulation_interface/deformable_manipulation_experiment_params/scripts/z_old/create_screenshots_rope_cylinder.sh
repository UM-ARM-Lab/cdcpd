#!/bin/bash

LOG_FOLDER=~/Dropbox/catkin_ws/src/smmap/logs/rope_cylinder/

# Rope Cylinder - regret 0
TEST_ID=wafr_final_submission/rope_KFMANDB_regret_0
mkdir ${LOG_FOLDER}${TEST_ID}
roslaunch smmap generic_experiment.launch \
    task_type:=rope_cylinder \
    test_id:=${TEST_ID}\
    logging_enabled:=true \
    start_bullet_viewer:=true \
    screenshots_enabled:=true \
    multi_model:=true \
    bandit_algoritm:=KFMANDB \
    calculate_regret:=false \
    use_random_seed:=false \
    static_seed_override:=true \
    static_seed:=147ce69cf5beff3e \
    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

# Rope Cylinder - trans 0 rot 24 - failure
TEST_ID=wafr_final_submission/rope_single_model_failure_trans_0_rot_24
mkdir ${LOG_FOLDER}${TEST_ID}
roslaunch smmap generic_experiment.launch \
    task_type:=rope_cylinder \
    test_id:=${TEST_ID} \
    logging_enabled:=true \
    start_bullet_viewer:=true \
    screenshots_enabled:=true \
    multi_model:=false \
    bandit_algoritm:=KFMANDB \
    calculate_regret:=false \
    deformability_override:=true \
    translational_deformability:=0 \
    rotational_deformability:=24 \
    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

