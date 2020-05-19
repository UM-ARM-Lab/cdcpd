#pragma once

#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "cdcpd/past_template_matcher.h"

void test_cylinder();

class CDCPD {
public:
    struct Output {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr masked_point_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cpd_output;
        pcl::PointCloud<pcl::PointXYZ>::Ptr gurobi_output;
    };

    struct FixedPoint {
        Eigen::Vector3f position;
        int template_index;
    };

    CDCPD(pcl::PointCloud<pcl::PointXYZ>::ConstPtr template_cloud,
          const cv::Mat& _P_matrix,
          bool _use_recovery = true);

    Output operator()(
         const cv::Mat& rgb, // RGB image
         const cv::Mat& depth, // Depth image
         const cv::Mat& mask,
         const pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud,
         const Eigen::Matrix2Xi& template_edges,
         const std::vector<FixedPoint>& fixed_points = std::vector<FixedPoint>()
         );

private:
    Eigen::VectorXf visibility_prior(const Eigen::Matrix3Xf vertices, 
                                            const Eigen::Matrix3f& intrinsics,
                                            const cv::Mat& depth,
                                            const cv::Mat& mask,
                                            float k=1e1); // TODO this should be configurable
    // TODO instead of transforming the P matrix continually, we should just store P as an Eigen matrix
    // and not have to pass around intr in here
    Eigen::Matrix3Xf cpd(pcl::PointCloud<pcl::PointXYZ>::ConstPtr downsampled_cloud,
                         const Eigen::Matrix3Xf& Y,
                         const cv::Mat& depth,
                         const cv::Mat& mask,
                         const Eigen::Matrix3f& intr);

    PastTemplateMatcher template_matcher;
    Eigen::Matrix3Xf original_template;

    // P_matrix: (3, 4) camera matrix
    const cv::Mat P_matrix;
    Eigen::Vector3f last_lower_bounding_box;
    Eigen::Vector3f last_upper_bounding_box;
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
    bool use_recovery;
}; 
