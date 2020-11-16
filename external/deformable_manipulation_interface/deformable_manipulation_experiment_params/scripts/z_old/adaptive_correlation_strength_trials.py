#!/usr/bin/python

from run_trial import *

# for experiment in ['cloth_table', 'cloth_wafr', 'rope_cylinder']:
for experiment in ['rope_cylinder']:
    # for process_noise_factor in [0.1, 0.01]:
    for process_noise_factor in [0.1]:
        # for observation_noise_factor in [0.1, 0.01]:
        for observation_noise_factor in [0.01]:
            # for algorithm in ['KFMANB', 'KFMANDB']:
            for algorithm in ['KFMANB']:
                for i in range(0, 10):
                    run_trial(
                        experiment=experiment,
                        start_bullet_viewer='true',
                        screenshots_enabled='true',
                        logging_enabled='true',
                        test_id='adaptive_correlation_trials_noise_trials/' 'process_' + str(process_noise_factor) + '_obs_' + str(observation_noise_factor) + '/' + algorithm + "_regret_" + str(i),
                        optimization_enabled='true',
                        bandit_algorithm=algorithm,
                        multi_model='true',
                        calculate_regret='true',
                        use_random_seed='true',
                        max_correlation_strength_factor=0.9,
                        process_noise_factor=process_noise_factor,
                        observation_noise_factor=observation_noise_factor)
