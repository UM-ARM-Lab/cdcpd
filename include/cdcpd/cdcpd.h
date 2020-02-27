#pragma once

#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class CDCPD {
public:
    CDCPD(const cv::Mat& _intrinsics);

    pcl::PointCloud<pcl::PointXYZ>::Ptr operator()(
         const cv::Mat& rgb,
         const cv::Mat& depth,
         const cv::Mat& mask,
         const pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud,
         const Eigen::MatrixXi& template_edges);
private:
    const cv::Mat intrinsics;
    Eigen::Vector4f last_lower_bounding_box;
    Eigen::Vector4f last_upper_bounding_box;
    const double tolerance;
    const double alpha;
    const double beta;
    const double w;
    const double initial_sigma_scale;
    const int max_iterations;
}; 