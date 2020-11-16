#!/bin/bash

function calc { 
    awk "BEGIN { print "$*" }"
}

function kalman_bandit_trial
{
    roslaunch smmap $base_environment.launch test_id:=$base_experiment"/process_noise_"$1"_observation_noise_"$2 \
        multi_model:=1 planning_horizion:=$planning_horizion \
        kalman_parameters_override:=1 process_noise_factor:=$1 observation_noise_factor:=$2
}

function single_model_trial_baseline_noise
{
    variance=$1
    roslaunch smmap $base_environment.launch test_id:=$base_experiment"/variance_"$variance \
        multi_model:=0 planning_horizion:=$planning_horizion \
        feedback_variance:=$variance
}

function single_model_trial_multiple_deform_values
{
    stepsize=$3
    min=`calc $1/$3`
    max=`calc $2/$3`
    for trans in `seq $min $max`;
    do
        trans_deform=`calc $trans*$3`
        for rot in `seq $min $max`;
        do
            rot_deform=`calc $rot*$3`
            test_id=$base_experiment"/trans_"$trans_deform"_rot_"$rot_deform
            echo $test_id
            roslaunch smmap $base_environment.launch test_id:=$test_id \
                multi_model:=0 planning_horizion:=$planning_horizion \
                deformability_override:=1 translational_deformability:=$trans_deform rotational_deformability:=$rot_deform
        done
    done
}
