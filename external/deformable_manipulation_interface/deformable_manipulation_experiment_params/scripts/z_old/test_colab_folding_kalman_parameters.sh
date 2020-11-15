#!/bin/bash

source trial_types.sh

base_environment="colab_folding"
planning_horizion=1
base_experiment=$planning_horizion"_step_kalman_bandit"
kalman_bandit_trial 1 1
kalman_bandit_trial 1 0.1
kalman_bandit_trial 1 0.01
kalman_bandit_trial 0.1 1
kalman_bandit_trial 0.1 0.1
kalman_bandit_trial 0.1 0.01
kalman_bandit_trial 0.01 1
kalman_bandit_trial 0.01 0.1
kalman_bandit_trial 0.01 0.01
