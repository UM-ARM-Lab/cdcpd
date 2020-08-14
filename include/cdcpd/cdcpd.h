#ifndef CDCPD_H
#define CDCPD_H

#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <smmap/constraint_jacobian_model.h>
#include <smmap/deformable_model.h>
#include <smmap_utilities/grippers.h>

#include "cdcpd/past_template_matcher.h"

// #ifndef DEBUG
// #define DEBUG
// #endif

#ifndef ENTIRE
#define ENTIRE
#endif

// #ifndef ROPE
// #define ROPE
// #endif

// #ifndef COMP
// #define COMP
// #endif

#ifndef COMP_PRED1
#define COMP_PRED1
#endif

#ifndef COMP_PRED2
#define COMP_PRED2
#endif

// #ifndef CPDLOG
// #define CPDLOG
// #endif

// #ifndef CYLINDER_INTER
// #define CYLINDER_INTER
// #endif

// #ifndef CYL_CLOTH4
// #define CYL_CLOTH4
// #endif

#ifndef SIMULATION
#define SIMULATION
#endif

#ifndef PREDICT
#define PREDICT
#endif

// #ifndef COMP_NOPRED
// #define COMP_NOPRED
// #endif

// void test_nearest_line();

Eigen::MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                                            int lle_neighbors,
                                            double reg);

Eigen::MatrixXf locally_linear_embedding(pcl::PointCloud<pcl::PointXYZ>::ConstPtr template_cloud,
                                         int lle_neighbors,
                                         double reg);

class CDCPD {
public:
    struct Output {
        #ifdef ENTIRE
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud;
        #endif
        pcl::PointCloud<pcl::PointXYZ>::Ptr masked_point_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cpd_output;
        #ifdef PREDICT
        pcl::PointCloud<pcl::PointXYZ>::Ptr cpd_predict;
        #endif
        pcl::PointCloud<pcl::PointXYZ>::Ptr gurobi_output;
    };

    struct FixedPoint {
        Eigen::Vector3f position;
        int template_index;
    };

    CDCPD(pcl::PointCloud<pcl::PointXYZ>::ConstPtr template_cloud,
          const Eigen::Matrix2Xi& _template_edges,
          #ifdef PREDICT
          std::shared_ptr<ros::NodeHandle> nh,
          const double translation_dir_deformability,
          const double translation_dis_deformability,
          const double rotation_deformability,
          const Eigen::MatrixXi& gripper_idx,
          #endif
          const bool _use_recovery = true,
          const double alpha = 0.5,
          const double beta = 1.0,
          const double lambda = 1.0,
          const double k = 100.0);

    Output operator()(const cv::Mat& rgb, // RGB image
                      const cv::Mat& depth, // Depth image
                      const cv::Mat& mask,
                      const cv::Matx33d& intrinsics,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud,
#ifdef PREDICT
                      const smmap::AllGrippersSinglePoseDelta& q_dot,
                      const smmap::AllGrippersSinglePose& q_config,
#endif
                      const bool self_intersection = true,
                      const bool interation_constrain = true,
                      const bool is_prediction = true,
					  const int pred_choice = 0,
                      const std::vector<FixedPoint>& fixed_points = {});

private:
    Eigen::VectorXf visibility_prior(const Eigen::Matrix3Xf& vertices,
                                     const cv::Mat& depth,
                                     const cv::Mat& mask,
                                     const Eigen::Matrix3f& intrinsics,
                                     const float kvis);

    Eigen::VectorXi is_occluded(const Eigen::Matrix3Xf& vertices,
                                const cv::Mat& depth,
                                const cv::Mat& mask,
                                const Eigen::Matrix3f& intrinsics);

    Eigen::Matrix3Xf blend_result(const Eigen::Matrix3Xf& Y_pred,
                                  const Eigen::Matrix3Xf& Y_cpd,
                                  const Eigen::VectorXi& is_occluded);

    // TODO instead of transforming the P matrix continually, we should just store P as an Eigen matrix
    // and not have to pass around intr in here
    Eigen::MatrixXf calcP(const int N,
                          const int M,
                          const int D,
                          const double sigma2,
                          const Eigen::Matrix3Xf& X,
                          const Eigen::Matrix3Xf& TY,
                          const Eigen::Matrix3Xf& Y_emit_prior);
    Eigen::Matrix3Xf cpd(const Eigen::Matrix3Xf& X,
                         const Eigen::Matrix3Xf& Y,
                         const Eigen::Matrix3Xf& Y_pred,
                         const cv::Mat& depth,
                         const cv::Mat& mask);

    Eigen::Matrix3Xf cpd(const Eigen::Matrix3Xf& X,
                         const Eigen::Matrix3Xf& Y,
                         const cv::Mat& depth,
                         const cv::Mat& mask);
#ifdef PREDICT
    Eigen::Matrix3Xd predict(const Eigen::Matrix3Xd& P,
                             const smmap::AllGrippersSinglePoseDelta& q_dot,
                             const smmap::AllGrippersSinglePose& q_config,
							 const int pred_choice);

    std::shared_ptr<smmap::ConstraintJacobianModel> model;
	std::shared_ptr<smmap::DeformableModel> deformModel;
#endif

    PastTemplateMatcher template_matcher;
    Eigen::Matrix3Xf original_template;
    const Eigen::Matrix2Xi template_edges;

    Eigen::Vector3f last_lower_bounding_box;
    Eigen::Vector3f last_upper_bounding_box;
    const int lle_neighbors;
    Eigen::MatrixXf m_lle;
    Eigen::MatrixXf L_lle;
    const double tolerance;
    const double alpha;
    const double beta;
    const double w;
    const double initial_sigma_scale;
    const double start_lambda;
    const double annealing_factor;
    const double k;
    const int max_iterations;
    const float kvis;
    bool use_recovery;
    // std::vector<Eigen::MatrixXf> Q;
    double last_sigma2;
    #ifdef PREDICT
    const Eigen::MatrixXi& gripper_idx;
    #endif
};

#endif

