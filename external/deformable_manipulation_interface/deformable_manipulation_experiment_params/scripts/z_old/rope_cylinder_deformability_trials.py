#!/usr/bin/python

from run_trial import *

# Note that this is 6 to 15 as range does [start, stop)
deform_range = range(2, 43, 4)
planning_horizion = 1

for translational_deform in deform_range:
    for rotational_deform in deform_range:
        run_trial(experiment = "rope_cylinder",
                  logging_enabled = "true",
                  test_id = "presentation_trials_baseline_rigidity"
                        + "trans_" + str(translational_deform)
                        + "_rot_" + str(rotational_deform),
                  planning_horizon= planning_horizion,
                  multi_model = "false",
                  deformability_override = "true",
                  translational_deformability = translational_deform,
                  rotational_deformability = rotational_deform)
