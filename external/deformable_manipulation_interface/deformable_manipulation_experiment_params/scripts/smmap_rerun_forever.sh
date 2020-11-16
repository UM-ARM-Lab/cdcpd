#!/usr/bin/env bash

TASKTYPE=rope_hooks_simple
TESTID=generate_training_examples

TERMINATE_FLAG_FILE=`rospack find deformable_manipulation_experiment_params`/scripts/rerun_terminate_flag
LOG_DIR=`rospack find smmap`/logs/$TASKTYPE/$TESTID
mkdir -p $LOG_DIR

while [[ ! -f "$TERMINATE_FLAG_FILE" ]];
do
    STAMP=`date +"%s%N"`
    roslaunch deformable_manipulation_experiment_params generic_experiment.launch \
        task_type:=$TASKTYPE \
        start_bullet_viewer:=false \
        disable_smmap_visualizations:=true \
        use_random_seed:=true \
        classifier_type:=none \
        test_id:=$TESTID \
        rrt_num_trials:=1 \
        test_paths_in_bullet:=true \
        rerun_forever:=true \
        "$@" --screen &> $LOG_DIR/$STAMP.log
done
