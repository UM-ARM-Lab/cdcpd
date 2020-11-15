#!/usr/bin/python

from run_trial import *


# Note that this is 0 to 16 as range does [start, stop), thus we get 0:1:15 in Matlab speak
adaptive_range = range(0, 16, 1)

# Run the single model baseline
for adaptive_exponent in adaptive_range:
    adaptive_model_learning_rate = 10.0**(-adaptive_exponent)
    run_trial(experiment="cloth_wafr",
              logging_enabled="true",
              test_id="single_model_baseline/"
                      + "adaptive_1e-" + str(adaptive_exponent),
              planning_horizon=1,
              multi_model="false",
              use_adaptive_model="true",
              adaptive_model_learning_rate=adaptive_model_learning_rate,
              start_bullet_viewer="false")

# Note that this is 0 to 25 as range does [start, stop), thus we get 0:2:24 in Matlab speak
deform_range = range(0, 25, 2)

# Run the single model baseline
for translational_deform in deform_range:
    # for rotational_deform in deform_range:
        rotational_deform=translational_deform
        run_trial(experiment="cloth_wafr",
                  logging_enabled="true",
                  test_id="single_model_baseline/"
                          + "trans_" + str(translational_deform)
                          + "_rot_" + str(rotational_deform),
                  planning_horizon=1,
                  multi_model="false",
                  deformability_override="true",
                  translational_deformability=translational_deform,
                  rotational_deformability=rotational_deform,
                  start_bullet_viewer="false")