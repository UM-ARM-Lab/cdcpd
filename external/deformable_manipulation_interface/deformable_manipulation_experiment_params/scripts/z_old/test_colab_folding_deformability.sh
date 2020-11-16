#!/bin/bash

source trial_types.sh

base_environment="colab_folding"
planning_horizion=1
base_experiment=$planning_horizion"_step_baseline"
single_model_trial_multiple_deform_values 10 18 4
