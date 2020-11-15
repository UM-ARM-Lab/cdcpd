#!/bin/bash

source trial_types.sh

base_environment="rope_cylinder"
planning_horizion=1
base_experiment=$planning_horizion"_step_observation_noise"
single_model_trial_baseline_noise 0
single_model_trial_baseline_noise `calc 0.025/20`
single_model_trial_baseline_noise `calc 0.05/20`
single_model_trial_baseline_noise `calc 0.1/20`
single_model_trial_baseline_noise `calc 0.5/20`
