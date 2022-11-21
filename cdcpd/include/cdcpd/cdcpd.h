#ifndef CDCPD_H
#define CDCPD_H

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <smmap_models/constraint_jacobian_model.h>
#include <smmap_models/diminishing_rigidity_model.h>
#include <smmap_utilities/grippers.h>

#include <arc_utilities/ostream_operators.hpp>
#include <arc_utilities/ros_helpers.hpp>
#include <opencv2/core.hpp>

#include "cdcpd/obs_util.h"
#include "cdcpd/optimizer.h"
#include "cdcpd/past_template_matcher.h"

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZHSV PointHSV;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointHSV> PointCloudHSV;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_3 Point_3;
typedef K::Ray_3 Ray_3;
typedef K::Vector_3 Vector;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> AABB_face_graph_primitive;
typedef CGAL::AABB_traits<K, AABB_face_graph_primitive> AABB_face_graph_traits;

namespace PMP = CGAL::Polygon_mesh_processing;

typedef PMP::Face_location<Mesh, FT> Face_location;

Eigen::MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, int lle_neighbors,
                                            double reg);

Eigen::MatrixXf locally_linear_embedding(PointCloud::ConstPtr template_cloud, int lle_neighbors, double reg);

static std::ostream &operator<<(std::ostream &out, FixedPoint const &p) {
  out << "[" << p.template_index << "] " << p.position;
  return out;
}

enum OutputStatus {
  NoPointInFilteredCloud,
  ObjectiveTooHigh,
  Success,
};

class CDCPD_Parameters
{
public:
    CDCPD_Parameters();
    CDCPD_Parameters(ros::NodeHandle& ph);

    // TODO: describe each parameter in words (and a pointer to an equation/section of paper)
    double const objective_value_threshold;
    double const alpha;
    double const lambda;
    double const k_spring;
    double const beta;
    double const zeta;
    double const min_distance_threshold;
    double const obstacle_cost_weight;
    double const fixed_points_weight;
    // NOTE: original cdcpd recovery not implemented
    bool const use_recovery;
    int const lle_neighbors;
    double const tolerance;
    double const w;
    double const initial_sigma_scale;
    int const max_iterations;
    double const kvis;
};

class CDCPD {
 public:
  struct Output {
    PointCloudRGB::Ptr original_cloud;
    PointCloud::Ptr masked_point_cloud;
    PointCloud::Ptr downsampled_cloud;
    PointCloud::Ptr cpd_output;
    PointCloud::Ptr cpd_predict;
    PointCloud::Ptr gurobi_output;
    OutputStatus status;
  };

  // TODO: This could be further refactored to break dependency on ROS to allow for running CDCPD
  // as a standalone library.
  CDCPD(ros::NodeHandle ph, PointCloud::ConstPtr template_cloud_initial,
      Eigen::Matrix2Xi const& template_edges_initial, CDCPD_Parameters const& params_in);

  CDCPD(PointCloud::ConstPtr template_cloud_initial,
      Eigen::Matrix2Xi const& template_edges_initial, CDCPD_Parameters const& params_in);

  // If you have want gripper constraints to be added and removed automatically based on is_grasped
  // and distance
  Output operator()(const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask,
      const cv::Matx33d &intrinsics, const PointCloud::Ptr template_cloud,
      ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},
      const smmap::AllGrippersSinglePose &q_config = {}, const std::vector<bool> &is_grasped = {},
      int pred_choice = 0);

  // If you want to used a known correspondence between grippers and node indices (gripper_idx)
  Output operator()(const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask,
      const cv::Matx33d &intrinsics, const PointCloud::Ptr template_cloud,
      ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},
      const smmap::AllGrippersSinglePose &q_config = {}, const Eigen::MatrixXi &gripper_idx = {},
      int pred_choice = 0);

  Output operator()(const PointCloudRGB::Ptr &points, const PointCloud::Ptr template_cloud,
      ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},
      const smmap::AllGrippersSinglePose &q_config = {}, const Eigen::MatrixXi &gripper_idx = {},
      int pred_choice = 0);

  // The common implementation that the above overloads call
  Output operator()(const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask,
      const cv::Matx33d &intrinsics, const PointCloud::Ptr template_cloud,
      ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},  // TODO: this should be one data structure
      const smmap::AllGrippersSinglePose &q_config = {}, int pred_choice = 0);

  Eigen::VectorXf visibility_prior(const Eigen::Matrix3Xf &vertices, const cv::Mat &depth,
      const cv::Mat &mask, const Eigen::Matrix3f &intrinsics, float kvis);

  Eigen::Matrix3Xf cpd(const Eigen::Matrix3Xf &X, const Eigen::Matrix3Xf &Y,
      const Eigen::Matrix3Xf &Y_pred, const Eigen::VectorXf &Y_emit_prior);

  Eigen::Matrix3Xd predict(const Eigen::Matrix3Xd &P,
      const smmap::AllGrippersSinglePoseDelta &q_dot,
      const smmap::AllGrippersSinglePose &q_config, int pred_choice);

  // ros::NodeHandle nh;
  ros::NodeHandle ph;

  std::unique_ptr<smmap::ConstraintJacobianModel> constraint_jacobian_model;
  std::unique_ptr<smmap::DiminishingRigidityModel> diminishing_rigidity_model;

  // PastTemplateMatcher template_matcher;
  Eigen::Matrix3Xf original_template;
  Eigen::Matrix2Xi template_edges;

  // CdcpdParameters params;
  Eigen::Vector3f last_lower_bounding_box;
  Eigen::Vector3f last_upper_bounding_box;
  int const lle_neighbors;
  Eigen::MatrixXf m_lle;
  Eigen::MatrixXf L_lle;
  double const tolerance;
  double const alpha;
  double const beta;
  double const w;
  double const initial_sigma_scale;
  float const start_lambda;
  double const k;
  int const max_iterations;
  float const kvis;
  float const zeta;
  float const obstacle_cost_weight;
  float const fixed_points_weight;
  bool const use_recovery = false;
  float const objective_value_threshold_;
  Eigen::MatrixXi gripper_idx;
  std::shared_ptr<const sdf_tools::SignedDistanceField> sdf_ptr;
  std::vector<bool> last_grasp_status;
  int total_frames_ = 0;
};

#endif
