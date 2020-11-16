#!/bin/bash

source trial_types.sh

base_environment="colab_folding"
planning_horizion=1
base_experiment=$planning_horizion"_step_observation_noise"
single_model_trial_baseline_noise 0
single_model_trial_baseline_noise `calc 0.00625/20`
single_model_trial_baseline_noise `calc 0.0125/20`
single_model_trial_baseline_noise `calc 0.01875/20`
single_model_trial_baseline_noise `calc 0.025/20`
