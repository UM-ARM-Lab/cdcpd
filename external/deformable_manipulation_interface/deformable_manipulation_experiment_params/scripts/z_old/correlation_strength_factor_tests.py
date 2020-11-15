#!/usr/bin/python

from run_trial import *

for correlation_strength_factor in [0.01, 0.1, 0.5, 0.9, 0.99]:
    run_trial(
        experiment="cloth_table",
        start_bullet_viewer='true',
        screenshots_enabled='true',
        logging_enabled='true',
        test_id='correlation_strength_factor_trials/KFMANDB_factor_' + str(correlation_strength_factor),
        optimization_enabled='true',
        bandit_algorithm='KFMANDB',
        multi_model='true',
        calculate_regret='true',
        use_random_seed='false',
        correlation_strength_factor=correlation_strength_factor)

for correlation_strength_factor in [0.01, 0.1, 0.5, 0.9, 0.99]:
    run_trial(
        experiment="rope_cylinder",
        start_bullet_viewer='true',
        screenshots_enabled='true',
        logging_enabled='true',
        test_id='correlation_strength_factor_trials/KFMANDB_factor_' + str(correlation_strength_factor),
        optimization_enabled='true',
        bandit_algorithm='KFMANDB',
        multi_model='true',
        calculate_regret='true',
        use_random_seed='false',
        correlation_strength_factor=correlation_strength_factor)

for correlation_strength_factor in [0.01, 0.1, 0.5, 0.9, 0.99]:
    run_trial(
        experiment="cloth_wafr",
        start_bullet_viewer='true',
        screenshots_enabled='true',
        logging_enabled='true',
        test_id='correlation_strength_factor_trials/KFMANDB_factor_' + str(correlation_strength_factor),
        optimization_enabled='true',
        bandit_algorithm='KFMANDB',
        multi_model='true',
        calculate_regret='true',
        use_random_seed='false',
        correlation_strength_factor=correlation_strength_factor)
