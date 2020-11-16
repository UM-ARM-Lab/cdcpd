#!/bin/bash

LOG_FOLDER=~/Dropbox/catkin_ws/src/smmap/logs/cloth_wafr/

#TEST_ID=wafr_final_submission/test_vs_noise_data_process_0.01_obs_0.1_regret_0_with_actual_noise_process_0.1_obs_0.01
#mkdir ${LOG_FOLDER}${TEST_ID}
#roslaunch smmap generic_experiment.launch \
#    task_type:=cloth_wafr \
#    test_id:=${TEST_ID}\
#    logging_enabled:=true \
#    start_bullet_viewer:=true \
#    screenshots_enabled:=true \
#    multi_model:=true \
#    bandit_algoritm:=KFMANDB \
#    calculate_regret:=false \
#    use_random_seed:=false \
#    static_seed_override:=true \
#    static_seed:=147c6fc183e8c118 \
#    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

#TEST_ID=wafr_final_submission/test_vs_noise_data_process_0.1_obs_0.01_with_actual_noise_process_0.1_obs_0.01_KFMANB_regret_0
#mkdir ${LOG_FOLDER}${TEST_ID}
#roslaunch smmap generic_experiment.launch \
#    task_type:=cloth_wafr \
#    test_id:=${TEST_ID}\
#    logging_enabled:=true \
#    start_bullet_viewer:=false \
#    screenshots_enabled:=false \
#    multi_model:=true \
#    bandit_algoritm:=KFMANB \
#    calculate_regret:=false \
#    use_random_seed:=false \
#    static_seed_override:=true \
#    static_seed:=147ba3bc969ecbf4 \
#    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

## Cloth Wafr - regret 9 with cloth pulled through cylinder
#TEST_ID=wafr_final_submission/cloth_pulled_through_cylinder_KFMANDB_regret_9
#mkdir ${LOG_FOLDER}${TEST_ID}
#roslaunch smmap generic_experiment.launch \
#    task_type:=cloth_wafr \
#    test_id:=${TEST_ID}\
#    logging_enabled:=true \
#    start_bullet_viewer:=true \
#    screenshots_enabled:=true \
#    multi_model:=true \
#    bandit_algoritm:=KFMANDB \
#    calculate_regret:=false \
#    use_random_seed:=false \
#    static_seed_override:=true \
#    static_seed:=147c6b8fa975bf7a \
#    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

## Cloth Wafr - regret 7 with cloth pulled off first cylinder, then moved back on
#TEST_ID=wafr_final_submission/cloth_pulled_off_cylinder_KFMANDB_regret_7
#mkdir ${LOG_FOLDER}${TEST_ID}
#roslaunch smmap generic_experiment.launch \
#    task_type:=cloth_wafr \
#    test_id:=${TEST_ID} \
#    logging_enabled:=true \
#    start_bullet_viewer:=true \
#    screenshots_enabled:=true \
#    multi_model:=true \
#    bandit_algoritm:=KFMANDB \
#    calculate_regret:=false \
#    use_random_seed:=false \
#    static_seed_override:=true \
#    static_seed:=147c635059c5febe \
#    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt
#
## Cloth Wafr - regret 6 with cloth pulled just far enough
#TEST_ID=wafr_final_submission/cloth_pulled_just_right_KFMANDB_regret_6
#mkdir ${LOG_FOLDER}${TEST_ID}
#roslaunch smmap generic_experiment.launch \
#    task_type:=cloth_wafr \
#    test_id:=${TEST_ID} \
#    logging_enabled:=true \
#    start_bullet_viewer:=true \
#    screenshots_enabled:=true \
#    multi_model:=true \
#    bandit_algoritm:=KFMANDB \
#    calculate_regret:=false \
#    use_random_seed:=false \
#    static_seed_override:=true \
#    static_seed:=147c5f3314b46581 \
#    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

## Cloth Wafr - trans 8 rot 4 - eventual success
#TEST_ID=wafr_final_submission/cloth_single_model_eventual_success_trans_8_rot_4
#mkdir ${LOG_FOLDER}${TEST_ID}
#roslaunch smmap generic_experiment.launch \
#    task_type:=cloth_wafr \
#    test_id:=${TEST_ID} \
#    logging_enabled:=true \
#    start_bullet_viewer:=true \
#    screenshots_enabled:=true \
#    multi_model:=false \
#    bandit_algoritm:=KFMANDB \
#    calculate_regret:=false \
#    deformability_override:=true \
#    translational_deformability:=8 \
#    rotational_deformability:=4 \
#    --screen > ${LOG_FOLDER}${TEST_ID}/output_log.txt

