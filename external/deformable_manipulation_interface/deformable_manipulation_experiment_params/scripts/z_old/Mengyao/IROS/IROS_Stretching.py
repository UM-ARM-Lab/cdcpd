#!/usr/bin/python

from mengyao_run_trial import *

from numpy import arange as frange


def mengyao_run_trials(experiment, log_prefix=""):

        screenshots_enabled="true"

        trans_dir_deformability = 4
        rotation_deformability = 20
        trans_dis_deformability = 10


        ####### TODO: Another Variable(s) should be defined (in mengyao_run_trial.py) to set the type of model controller 
        ####### for each of the functions below:
        ####### For each function below, only one of the bench mark or new model should be run. 
        ####### The resulting "realtime_stretching_factor.txt" will have only one column recording the data for 
        ####### the one controller.

        ####### TODO: Put the Matlab scripts in the Matlab_Script folder into the corresponding folder
        ####### e.g: smmap/log/cloth_single_pole/control_error_IROS_cloth_single_pole.m
        #######      smmap/log/cloth_single_pole/single_stretching_factor_IROS_cloth_single_pole.m

        ####### TODO: Double check the log setting here. 

        # rope_cylinder on BenchMark model and controller
        # down_scale = 200
        # stretching_threshold = 0.4
        # mengyao_run_trial(experiment = "rope_cylinder_two_grippers",
        #                   start_bullet_viewer = "false",
        #                   screenshots_enabled = screenshots_enabled,
        #                   disable_all_visualizations="true",
        #                   controller_logging_enabled = "true",
        #                   test_id = log_prefix + ("IROS_screenshots/BM"),
        #                   planner_trial_type = "diminishing_rigidity_single_model_least_squares_controller",
        #                   desired_motion_scale_factor= down_scale,
        #                   translational_dir_deformability = trans_dir_deformability,
        #                   translational_dis_deformability = trans_dis_deformability,
        #                   rotational_dis_deformability = rotation_deformability,
        #                   stretching_cosine_threshold = stretching_threshold,
        #                   )
        #
        # rope_cylinder on new model and controller
        down_scale = 200
        stretching_threshold = 0.4
        mengyao_run_trial(experiment = "rope_cylinder_two_grippers",
                          start_bullet_viewer = "false",
                          screenshots_enabled = screenshots_enabled,
                          disable_all_visualizations="true",
                          controller_logging_enabled = "true",
                          test_id = log_prefix + ("IROS_screenshots/NM"),
                          planner_trial_type = "constraint_single_model_constraint_controller",
                          desired_motion_scale_factor= down_scale,
                          translational_dir_deformability = trans_dir_deformability,
                          translational_dis_deformability = trans_dis_deformability,
                          rotational_dis_deformability = rotation_deformability,
                          stretching_cosine_threshold = stretching_threshold,
                          )

        # # cloth_single_pole on Bench mark model and controller
        # down_scale = 100
        # stretching_threshold = 0.4
        # mengyao_run_trial(experiment = "cloth_single_pole",
        #                   start_bullet_viewer = "false",
        #                   screenshots_enabled=screenshots_enabled,
        #                   disable_all_visualizations="true",
        #                   controller_logging_enabled = "true",
        #                   test_id = log_prefix + ("stretching_status/BM"),
        #                   planner_trial_type = "diminishing_rigidity_single_model_least_squares_controller",
        #                   desired_motion_scale_factor= down_scale,
        #                   translational_dir_deformability = trans_dir_deformability,
        #                   translational_dis_deformability = trans_dis_deformability,
        #                   rotational_dis_deformability = rotation_deformability,
        #                   stretching_cosine_threshold = stretching_threshold,
        #                   )
        #
        # # cloth_single_pole on new model and controller with cos = 0.4 (stretching parameter)
        # down_scale = 100
        # stretching_threshold = 0.4
        # mengyao_run_trial(experiment = "cloth_single_pole",
        #                   start_bullet_viewer = "false",
        #                   screenshots_enabled = screenshots_enabled,
        #                   disable_all_visualizations="true",
        #                   controller_logging_enabled = "true",
        #                   test_id = log_prefix + ("stretching_status/NM/cos_04"),
        #                   planner_trial_type = "constraint_single_model_constraint_controller",
        #                   desired_motion_scale_factor= down_scale,
        #                   translational_dir_deformability = trans_dir_deformability,
        #                   translational_dis_deformability = trans_dis_deformability,
        #                   rotational_dis_deformability = rotation_deformability,
        #                   stretching_cosine_threshold = stretching_threshold,
        #                   )
        #
        # # cloth_single_pole on new model and controller with cos = 0.6 (stretching parameter)
        # down_scale = 100
        # stretching_threshold = 0.6
        # mengyao_run_trial(experiment = "cloth_single_pole",
        #                   start_bullet_viewer = "false",
        #                   screenshots_enabled = screenshots_enabled,
        #                   disable_all_visualizations="true",
        #                   controller_logging_enabled = "true",
        #                   test_id = log_prefix + ("stretching_status/NM/cos_06"),
        #                   planner_trial_type = "constraint_single_model_constraint_controller",
        #                   desired_motion_scale_factor= down_scale,
        #                   translational_dir_deformability = trans_dir_deformability,
        #                   translational_dis_deformability = trans_dis_deformability,
        #                   rotational_dis_deformability = rotation_deformability,
        #                   stretching_cosine_threshold = stretching_threshold,
        #                   )
        #
        # # cloth_single_pole on new model and controller with cos = 0.8 (stretching parameter)
        # down_scale = 100
        # stretching_threshold = 0.8
        # mengyao_run_trial(experiment = "cloth_single_pole",
        #                   start_bullet_viewer = "false",
        #                   screenshots_enabled = screenshots_enabled,
        #                   disable_all_visualizations="true",
        #                   controller_logging_enabled = "true",
        #                   test_id = log_prefix + ("stretching_status/NM/cos_08"),
        #                   planner_trial_type = "constraint_single_model_constraint_controller",
        #                   desired_motion_scale_factor= down_scale,
        #                   translational_dir_deformability = trans_dir_deformability,
        #                   translational_dis_deformability = trans_dis_deformability,
        #                   rotational_dis_deformability = rotation_deformability,
        #                   stretching_cosine_threshold = stretching_threshold,
        #                   )




if __name__ == '__main__':
  mengyao_run_trials("***")        
