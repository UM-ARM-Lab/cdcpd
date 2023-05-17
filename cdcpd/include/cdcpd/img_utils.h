#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>

cv::Mat getHsvMask(cv::Mat const& rgb, double const hue_min=340.0, double const sat_min=0.3,
    double const val_min=0.4, double const hue_max=20.0, double const sat_max=1.0,
    double const val_max=1.0);