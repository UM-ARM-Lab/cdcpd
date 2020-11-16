#!/usr/bin/env bash

#test_id=voxnet_old_labels_10_trials
#test_id=svm_knn_label_based_on_correct_10_trials
#test_id=label_based_on_correct_100_trials__single_planning_instance__planning_limit_600
test_id=label_based_on_correct_30_trials__single_planning_instance__planning_limit_600

environments[0]=rope_hooks_simple_short_rope
environments[1]=rope_hooks_simple
environments[2]=rope_hooks_simple_long_rope
environments[3]=rope_hooks_simple_super_long_rope
environments[4]=rope_hooks_multi
environments[5]=rope_hooks
environments[6]=engine_assembly
#environments[7]=cloth_hooks_complex

for exp in ${environments[@]}; do
    echo "${exp}"
    cd /tmp
    cd ~/catkin_ws/src/smmap/logs/${exp}/${test_id}/basic__normalized_lengths__raw_connected_components/
    data=`cat */smmap_output.log | grep "Total successful"`
    echo "${data}"
    echo ""
done
