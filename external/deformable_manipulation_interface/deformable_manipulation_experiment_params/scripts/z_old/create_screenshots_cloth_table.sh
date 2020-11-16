#!/bin/bash

LOG_FOLDER=~/Dropbox/catkin_ws/src/smmap/logs/cloth_table/

# Cloth Table - regret 0
TEST_ID=wafr_final_submission/cloth_KFMANDB_regret_0
mkdir ${LOG_FOLDER}${TEST_ID}
roslaunch smmap generic_experiment.launch \
    task_type:=cloth_table \
    test_id:=${TEST_ID}\
    logging_enabled:=true \
    start_bullet_viewer:=true \
    screenshots_enabled:=true \
    multi_model:=true \
    bandit_algoritm:=KFMANDB \
    calculate_regret:=false \
    use_random_seed:=false \
    static_seed_override:=true \
    static_seed:=147b6f7f94c9b78f \
    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

# Cloth Table - trans 24 rot 0 - eventual success
TEST_ID=wafr_final_submission/cloth_single_model_eventual_success_trans_24_rot_0
mkdir ${LOG_FOLDER}${TEST_ID}
roslaunch smmap generic_experiment.launch \
    task_type:=cloth_table \
    test_id:=${TEST_ID} \
    logging_enabled:=true \
    start_bullet_viewer:=true \
    screenshots_enabled:=true \
    multi_model:=false \
    bandit_algoritm:=KFMANDB \
    calculate_regret:=false \
    deformability_override:=true \
    translational_deformability:=24 \
    rotational_deformability:=0 \
    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

