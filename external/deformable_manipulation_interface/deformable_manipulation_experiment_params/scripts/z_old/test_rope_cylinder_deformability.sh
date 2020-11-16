#!/bin/bash

source trial_types.sh

base_environment="rope_cylinder"
planning_horizion=1
base_experiment=$planning_horizion"_step_baseline"
single_model_trial_multiple_deform_values 6 14 4
