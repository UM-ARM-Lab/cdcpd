#!/usr/bin/python

from mengyao_run_trial import *

from numpy import arange as frange


def mengyao_run_trials(experiment, generate_screenshots="false", log_prefix=""):

        # Note that this is 0 to 25 as range does [start, stop), thus we get 0:4:24 in Matlab speak
        # deform_range = range(0, 25, 4)
        # 5:5:25
        # stretching_threshold_range = frange(0.35, 0.7, 0.05)
        down_scale_range = [400]
        stretching_threshold_range = [0.4, 0.5, 0.3, 0.6]
        trans_dir_deformability_range = [900]
        trans_dis_deformability_range = [10]
        rotation_deformability_range = [10]
        cloth_leading_edge_x_range = [-0.012, -0.01, -0.05, 0.0]
        #cloth_leading_edge_x_range = [-0.005]
        cloth_leading_edge_y_range = [0.0, 0.02, 0.05, 0.07, 0.085]

        rotation_deformability = 20
        trans_dis_deformability = 10;

        # Run the single model baseline
      #  for rotation_deformability in rotation_deformability_range:
      #  for stretching_threshold in stretching_threshold_range:
        for cloth_x in cloth_leading_edge_x_range:
            for cloth_y in cloth_leading_edge_y_range:
                for stretching_threshold in stretching_threshold_range:
                    for trans_dir_deformability in trans_dir_deformability_range:
                        for down_scale in down_scale_range:
                            mengyao_run_trial(experiment=experiment,
                                              start_bullet_viewer="false",
                                              logging_enabled="true",
                                              test_id=log_prefix + ("MM_test_stretching/DM/" + "desired_down_" + str(down_scale) +
                                      "/cos_" + str(stretching_threshold) + "/dir" + str(trans_dir_deformability) + 
                                      "_dis" + str(trans_dis_deformability) + "_rot" + str(rotation_deformability) + 
                                      "/cloth_x" + str(cloth_x) + "cloth_y" + str(cloth_y)),
                                              desired_motion_scale_factor = down_scale,
                                              translational_dir_deformability = trans_dir_deformability,
                                              translational_dis_deformability = trans_dis_deformability,
                                              rotational_dis_deformability=rotation_deformability,
                                              stretching_cosine_threshold=stretching_threshold,
                                              cloth_leading_edge_x = cloth_x,
                                              cloth_leading_edge_y = cloth_y)

        # Note that this is 0 to 16 as range does [start, stop), thus we get 0:1:10 in Matlab speak





if __name__ == '__main__':
  mengyao_run_trials("cloth_single_pole")        
