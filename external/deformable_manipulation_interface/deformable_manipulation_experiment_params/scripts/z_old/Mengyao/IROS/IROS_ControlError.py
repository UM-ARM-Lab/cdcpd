#!/usr/bin/python

from mengyao_run_trial import *

from numpy import arange as frange

### This script will run all the four tests in one time. 

def mengyao_run_trials(generate_screenshots="true", log_prefix=""):

        trans_dir_deformability = 900.0
        rotation_deformability = 20.0
        trans_dis_deformability = 10.0

        ####### for each of the functions below:
        ####### For each function below, both the bench mark model and the new model should be tested
        ####### at each simulation step. The "control_error_realtime.txt" should contain 2 columns of errors.
        ####### The first column is the control error of the new model, and the second for the bench mark


        ####### The corresponding Matlab scripts to generate plots are in the log folder.
        ####### e.g: smmap/log/cloth_single_pole/control_error_IROS_cloth_single_pole.m
        #######      smmap/log/cloth_single_pole/single_stretching_factor_IROS_cloth_single_pole.m

        desired_motion_scale_factor = 200.0
        stretching_threshold = 0.3
        mengyao_run_trial(experiment = "rope_cylinder_two_grippers",
                          start_bullet_viewer = "false",
                          screenshots_enabled = generate_screenshots,
                          disable_all_visualizations = "true",
                          controller_logging_enabled = "true",
                          test_id = log_prefix + ("controll_error"),
                          planner_trial_type = "multi_model_controller_test",
                          desired_motion_scale_factor= desired_motion_scale_factor,
                          translational_dir_deformability = trans_dir_deformability,
                          translational_dis_deformability = trans_dis_deformability,
                          rotational_dis_deformability = rotation_deformability,
                          stretching_cosine_threshold = stretching_threshold,
                          )

        desired_motion_scale_factor = 5.0
        stretching_threshold = 0.3
        mengyao_run_trial(experiment = "rope_zig_match",
                          start_bullet_viewer = "false",
                          screenshots_enabled=generate_screenshots,
                          disable_all_visualizations="true",
                          controller_logging_enabled = "true",
                          test_id = log_prefix + ("controll_error"),
                          planner_trial_type = "multi_model_controller_test",
                          desired_motion_scale_factor= desired_motion_scale_factor,
                          translational_dir_deformability = trans_dir_deformability,
                          translational_dis_deformability = trans_dis_deformability,
                          rotational_dis_deformability = rotation_deformability,
                          stretching_cosine_threshold = stretching_threshold,
                          )


        desired_motion_scale_factor = 400.0
        stretching_threshold = 0.4
        mengyao_run_trial(experiment = "cloth_wafr",
                          start_bullet_viewer = "false",
                          screenshots_enabled=generate_screenshots,
                          disable_all_visualizations="true",
                          controller_logging_enabled = "true",
                          test_id = log_prefix + ("controll_error"),
                          planner_trial_type = "multi_model_controller_test",
                          desired_motion_scale_factor= desired_motion_scale_factor,
                          translational_dir_deformability = trans_dir_deformability,
                          translational_dis_deformability = trans_dis_deformability,
                          rotational_dis_deformability = rotation_deformability,
                          stretching_cosine_threshold = stretching_threshold,
                          )

        desired_motion_scale_factor = 100.0
        stretching_threshold = 0.4
        mengyao_run_trial(experiment = "cloth_single_pole",
                          start_bullet_viewer = "false",
                          screenshots_enabled=generate_screenshots,
                          disable_all_visualizations="true",
                          controller_logging_enabled = "true",
                          test_id = log_prefix + ("controll_error"),
                          planner_trial_type = "multi_model_controller_test",
                          desired_motion_scale_factor= desired_motion_scale_factor,
                          translational_dir_deformability = trans_dir_deformability,
                          translational_dis_deformability = trans_dis_deformability,
                          rotational_dis_deformability = rotation_deformability,
                          stretching_cosine_threshold = stretching_threshold,
                          )



if __name__ == '__main__':
  mengyao_run_trials()
