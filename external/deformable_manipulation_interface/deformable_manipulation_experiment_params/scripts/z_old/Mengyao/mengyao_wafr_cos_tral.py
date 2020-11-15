#!/usr/bin/python

from mengyao_run_trial import *

from numpy import arange as frange


def mengyao_run_trials(experiment, generate_screenshots="false", log_prefix=""):

        # Note that this is 0 to 25 as range does [start, stop), thus we get 0:4:24 in Matlab speak
        # deform_range = range(0, 25, 4)
        # 5:5:25
        # stretching_threshold_range = frange(0.35, 0.7, 0.05)
        stretching_threshold_range = [0.4]
        trans_dir_deformability_range = [900]
        trans_dis_deformability_range = [10, 3]
        rotation_deformability_range = [20, 5]
        down_scaling_range = [300]
        stretching_threshold = 0.4

        # Run the single model baseline
        for down_scaling in down_scaling_range:
            for trans_dir_deformability in trans_dir_deformability_range:
                for trans_dis_deformability in trans_dis_deformability_range:
                    for rotation_deformability in rotation_deformability_range:
                        mengyao_run_trial(experiment=experiment,
                                          start_bullet_viewer="false",
                                          logging_enabled="true",
                                          test_id=log_prefix + ("MM_test/desired_down" + str(down_scaling) +
                                    "/cos_" + str(stretching_threshold) + "_dir" + 
                                    str(trans_dir_deformability) + "_dis" + str(trans_dis_deformability) + 
                                    "_rot" + str(rotation_deformability)),
                                          desired_motion_scale_factor = down_scaling,
                                          translational_dir_deformability = trans_dir_deformability,
                                          translational_dis_deformability = trans_dis_deformability,
                                          rotational_dis_deformability=rotation_deformability,
                                          stretching_cosine_threshold=stretching_threshold)
        # Note that this is 0 to 16 as range does [start, stop), thus we get 0:1:10 in Matlab speak






if __name__ == '__main__':
  mengyao_run_trials("cloth_wafr")        
