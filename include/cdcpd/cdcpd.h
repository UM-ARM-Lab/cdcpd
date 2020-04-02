#pragma once

#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class CDCPD {
public:
    struct Output {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr masked_point_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr last_template;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cpd_iterations;
        pcl::PointCloud<pcl::PointXYZ>::Ptr gurobi_output;
    };

    CDCPD(pcl::PointCloud<pcl::PointXYZ>::ConstPtr template_cloud,
          const cv::Mat& _P_matrix);

    Output operator()(
         const cv::Mat& rgb,
         const cv::Mat& depth,
         const cv::Mat& mask,
         const pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud,
         const Eigen::MatrixXi& template_edges);

private:
    const cv::Mat P_matrix;
    Eigen::Vector4f last_lower_bounding_box;
    Eigen::Vector4f last_upper_bounding_box;
    const int lle_neighbors;
    Eigen::MatrixXf m_lle;
    const double tolerance;
    const double alpha;
    const double beta;
    const double w;
    const double initial_sigma_scale;
    const double start_lambda;
    const double annealing_factor;
    const int max_iterations;
}; 
