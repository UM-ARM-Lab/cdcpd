#!/usr/bin/python

from run_trial import *

# Note that this is 10 to 19 as range does [start, stop)
deform_range = range(10, 19, 4)

process_noise_factor_range = [0.01, 0.1, 1.0, 10.0, 100.0]
# feedback_covariance_range = [0.0, 0.025 / 20.0, 0.05 / 20.0, 0.1 / 20, 0.5 / 20]
feedback_covariance_range = [0.0, 0.00625 / 20.0, 0.0125 / 20.0, 0.01875 / 20, 0.025 / 20]
planning_horizion = 1

for feedback_covariance in feedback_covariance_range:

    # Run the single model baseline
    # for translational_deform in deform_range:
    #     for rotational_deform in deform_range:
    #
    #         run_trial(experiment="cloth_cylinder",
    #                   logging_enabled="true",
    #                   test_id=str(planning_horizion) + "_step_simulator_noise_vs_kalman_parameters/"
    #                           + "feedback_covariance_" + str(feedback_covariance) + "/"
    #                           + "single_model_baseline/"
    #                           + "trans_" + str(translational_deform)
    #                           + "_rot_" + str(rotational_deform),
    #                   planning_horizion=planning_horizion,
    #                   multi_model="false",
    #                   deformability_override="true",
    #                   translational_deformability=translational_deform,
    #                   rotational_deformability=rotational_deform)

    # Run the multi model trials
    for process_noise_factor in process_noise_factor_range:

        if process_noise_factor == process_noise_factor_range[0]:
            observation_noise_factor_range = \
                [process_noise_factor, process_noise_factor * 10.0, process_noise_factor_range[-1]]
        elif process_noise_factor == process_noise_factor_range[-1]:
            observation_noise_factor_range = \
                [process_noise_factor_range[0], process_noise_factor / 10.0, process_noise_factor]
        else:
            observation_noise_factor_range = \
                [process_noise_factor / 10.0, process_noise_factor, process_noise_factor * 10.0]

        for observation_noise_factor in observation_noise_factor_range:
            run_trial(experiment = "cloth_cylinder",
                      logging_enabled = "true",
                      test_id = str(planning_horizion) + "_step_simulator_noise_vs_kalman_parameters/"
                            + "feedback_covariance_" + str(feedback_covariance) + "/"
                            + "process_" + str(process_noise_factor)
                            + "_observation_" + str(observation_noise_factor),
                      planning_horizon= planning_horizion,
                      multi_model = "true",
                      kalman_parameters_override = "true",
                      process_noise_factor = process_noise_factor,
                      observation_noise_factor = observation_noise_factor)
