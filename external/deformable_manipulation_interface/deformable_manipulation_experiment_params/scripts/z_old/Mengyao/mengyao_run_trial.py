import sys
import subprocess


def mengyao_run_trial(experiment,
              start_bullet_viewer = None,
              screenshots_enabled = None,
              logging_enabled = None,
              test_id = None,
              planning_horizon = None,
              optimization_enabled = None,
              bandit_algorithm = None,
              multi_model = None,
              deformability_override = None,
              translational_deformability = None,
              rotational_deformability = None,
              use_adaptive_model = None,
              adaptive_model_learning_rate = None,

              use_diminishing_rigidity_model = None,
              use_constraint_model = None,
              use_diminishing_random_sample_model = None,

              desired_down_scale = None,

              translational_dir_deformability = None,
              translational_dis_deformability = None,
              rotational_dis_deformability = None,
              stretching_cosine_threshold = None,

              cloth_leading_edge_x = None,
              cloth_leading_edge_y = None,

              process_noise_factor = None,
              observation_noise_factor = None,

              feedback_covariance = None,
              calculate_regret = None,
              use_random_seed = None,

              correlation_strength_factor = None,
              max_correlation_strength_factor = None):
    # Constant values that we need
    # roslaunch_command = ["roslaunch", "smmap", "generic_experiment.launch task_type:=" + experiment]
    roslaunch_command = ["roslaunch", "deformable_manipulation_experiment_params", "generic_experiment_mengyao.launch task_type:=" + experiment, "--screen"]

    # Setup logging parameters
    if logging_enabled is not None:
        roslaunch_command.append('logging_enabled:=' + logging_enabled)
    if test_id is not None:
        roslaunch_command.append('test_id:=' + test_id)

    # Setup planner parameters
    if planning_horizon is not None:
        roslaunch_command.append('planning_horizion:=' + str(planning_horizon))

    if optimization_enabled is not None:
        roslaunch_command.append('optimization_enabled:=' + str(optimization_enabled))

    # Setup model parameters
    if deformability_override is not None:
        assert(translational_deformability is not None and rotational_deformability is not None)
        roslaunch_command.append('deformability_override:=' + str(deformability_override))
        roslaunch_command.append('translational_deformability:=' + str(translational_deformability))
        roslaunch_command.append('rotational_deformability:=' + str(rotational_deformability))

    if use_adaptive_model is not None:
        roslaunch_command.append('use_adaptive_model:=' + str(use_adaptive_model))

    if adaptive_model_learning_rate is not None:
        roslaunch_command.append('adaptive_model_learning_rate:=' + str(adaptive_model_learning_rate))

    if desired_down_scale is not None:
        roslaunch_command.append('desired_down_scale:=' + str(desired_down_scale))
        
    if translational_dir_deformability is not None:
        roslaunch_command.append('translational_dir_deformability:=' + str(translational_dir_deformability))

    if translational_dis_deformability is not None:
        roslaunch_command.append('translational_dis_deformability:=' + str(translational_dis_deformability))

    if rotational_dis_deformability is not None:
        roslaunch_command.append('rotational_dis_deformability:=' + str(rotational_dis_deformability))

    if stretching_cosine_threshold is not None:
        roslaunch_command.append('stretching_cosine_threshold:=' + str(stretching_cosine_threshold))

    # Experimental setup Params
    if cloth_leading_edge_x is not None:
        roslaunch_command.append('cloth_leading_edge_x:=' + str(cloth_leading_edge_x))

    if cloth_leading_edge_y is not None:
        roslaunch_command.append('cloth_leading_edge_y:=' + str(cloth_leading_edge_y))

    # Setup multi-model parameters
    if bandit_algorithm is not None:
        roslaunch_command.append('bandit_algorithm:=' + bandit_algorithm)

    if multi_model is not None:
        roslaunch_command.append('multi_model:=' + multi_model)

    if process_noise_factor is not None or observation_noise_factor is not None:
        assert(process_noise_factor is not None and observation_noise_factor is not None)
        roslaunch_command.append('kalman_parameters_override:=true')
        roslaunch_command.append('process_noise_factor:=' + str(process_noise_factor))
        roslaunch_command.append('observation_noise_factor:=' + str(observation_noise_factor))

    # Setup simulator parameters
    if feedback_covariance is not None:
        roslaunch_command.append('feedback_covariance:=' + str(feedback_covariance))

    if start_bullet_viewer is not None:
        roslaunch_command.append('start_bullet_viewer:=' + str(start_bullet_viewer))

    if calculate_regret is not None:
        roslaunch_command.append('calculate_regret:=' + str(calculate_regret))

    if screenshots_enabled is not None:
        roslaunch_command.append('screenshots_enabled:=' + str(screenshots_enabled))

    if use_random_seed is not None:
        roslaunch_command.append('use_random_seed:=' + str(use_random_seed))

    if correlation_strength_factor is not None:
        roslaunch_command.append('correlation_strength_factor_override:=true')
        roslaunch_command.append('correlation_strength_factor:=' + str(correlation_strength_factor))

    if max_correlation_strength_factor is not None:
        roslaunch_command.append('max_correlation_strength_factor:=' + str(max_correlation_strength_factor))

    # Add any extra parameters that have been added
    roslaunch_command += sys.argv[1:]

    log_folder = '~/Dropbox/catkin_ws/src/smmap/logs/' + experiment + '/' + test_id
    roslaunch_command.append(' --screen > ' + log_folder + '/output_log.txt')

    cmd = ' '.join(roslaunch_command)
    print cmd, '\n'

    subprocess.call(args='mkdir -p ' + log_folder, shell=True)
    subprocess.call(args=cmd, shell=True)
