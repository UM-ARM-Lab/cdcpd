#pragma once

#include <random>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "opencv2/features2d.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PastTemplateMatcher {
public:
    PastTemplateMatcher(int _sample_size);

    size_t size();

    // TODO returning vectors of big matrices is very inefficient. Perhaps return pointers, or something, instead.
    std::vector<Eigen::Matrix3Xf> query_template(pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_points, int k=8);

    void add_template(pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points, Eigen::Matrix3Xf tracking_result); 

private:
    cv::Mat downsampled_feature(pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_points);
    cv::Mat vfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_points);

    cv::FlannBasedMatcher matcher;
    std::vector<Eigen::Matrix3Xf> recovery_templates;
    std::vector<cv::Mat> recovery_features;
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<> index_generator;
    int sample_size;
};
