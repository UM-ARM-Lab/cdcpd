#!/bin/bash

source trial_types.sh

base_environment="rope_cylinder"
planning_horizion=1
base_experiment=$planning_horizion"_step_kalman_bandit_time_seed"
kalman_bandit_trial 10 0.1
kalman_bandit_trial 10 1
kalman_bandit_trial 10 10
kalman_bandit_trial 100 1
kalman_bandit_trial 100 10
kalman_bandit_trial 100 100
kalman_bandit_trial 1 1
kalman_bandit_trial 1 0.1
kalman_bandit_trial 1 0.01
kalman_bandit_trial 0.1 1
kalman_bandit_trial 0.1 0.1
kalman_bandit_trial 0.1 0.01
kalman_bandit_trial 0.01 1
kalman_bandit_trial 0.01 0.1
kalman_bandit_trial 0.01 0.01
