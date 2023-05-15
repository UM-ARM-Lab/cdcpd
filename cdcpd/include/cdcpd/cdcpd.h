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

#include "cdcpd/cpd.h"
#include "cdcpd/obs_util.h"
#include "cdcpd/optimizer.h"
#include "cdcpd/past_template_matcher.h"
#include "cdcpd/stopwatch.h"
#include "cdcpd/segmenter.h"
#include "cdcpd/tracking_map.h"

#include <iostream>
#include <string>

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

// Perform VoxelGrid filter downsampling on an Eigen matrix representing a point cloud.
Eigen::Matrix3Xf downsampleMatrixCloud(Eigen::Matrix3Xf mat_in);

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
};

class CDCPDIterationInputs
{
public:
    // CDCPDIterationInputs();

    // Eigen::Matrix3Xf Y;
    Eigen::VectorXf Y_emit_prior;
    Eigen::Matrix3Xf X;
    ObstacleConstraints obstacle_constraints;
    // Eigen::RowVectorXd max_segment_length;
    std::vector<FixedPoint> pred_fixed_points;
    TrackingMap tracking_map;
    smmap::AllGrippersSinglePoseDelta q_dot;
    smmap::AllGrippersSinglePose q_config;
    int pred_choice;
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

    Eigen::Matrix3Xf get_cpd_output()
    {
        return cpd_output->getMatrixXfMap().topRows(3);
    }

    Eigen::Matrix3Xf get_gurobi_output()
    {
        return gurobi_output->getMatrixXfMap().topRows(3);
    }

    Eigen::Matrix3Xf get_downsampled_cloud()
    {
        return downsampled_cloud->getMatrixXfMap().topRows(3);
    }
  };

  CDCPD(TrackingMap const& tracking_map, float objective_value_threshold, bool use_recovery = false,
      double alpha = 0.5, double beta = 1.0, double lambda = 1.0, double k = 100.0,
      float zeta = 10.0, float obstacle_cost_weight = 1.0, float fixed_points_weight = 10.0);

  CDCPD(PointCloud::ConstPtr template_cloud, const Eigen::Matrix2Xi &_template_edges,
      float objective_value_threshold, bool use_recovery = false, double alpha = 0.5,
      double beta = 1.0, double lambda = 1.0, double k = 100.0, float zeta = 10.0,
      float obstacle_cost_weight = 1.0, float fixed_points_weight = 10.0);

//   CDCPD(ros::NodeHandle nh, ros::NodeHandle ph, PointCloud::ConstPtr template_cloud,
//       const Eigen::Matrix2Xi &_template_edges, float objective_value_threshold,
//       bool use_recovery = false, double alpha = 0.5, double beta = 1.0, double lambda = 1.0,
//       double k = 100.0, float zeta = 10.0, float obstacle_cost_weight = 1.0,
//       float fixed_points_weight = 10.0);

  // If you have want gripper constraints to be added and removed automatically based on is_grasped
  // and distance
  Output operator()(const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask,
      const cv::Matx33d &intrinsics, TrackingMap const& tracking_map,
      ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},
      const smmap::AllGrippersSinglePose &q_config = {}, const std::vector<bool> &is_grasped = {},
      int pred_choice = 0);

  // If you want to used a known correspondence between grippers and node indices (gripper_idx)
  Output operator()(const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask,
      const cv::Matx33d &intrinsics, TrackingMap const& tracking_map,
      ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},
      const smmap::AllGrippersSinglePose &q_config = {}, const Eigen::MatrixXi &gripper_idx = {},
      int pred_choice = 0);

  Output operator()(const PointCloudRGB::Ptr &points, TrackingMap const& tracking_map,
      ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},
      const smmap::AllGrippersSinglePose &q_config = {}, const Eigen::MatrixXi &gripper_idx = {},
      int pred_choice = 0);

  // TODO(Dylan): Figure out a better (more uniform) way to organize the call structure. As it
  // stands, this should be an operator overload but it can't be as it has the same arguments as
  // the above point cloud method, but this method does not do the segmentation.
//   Output doCdcpdPointCloudAlreadySegmented(const PointCloudRGB::Ptr &filtered_points,
//       const PointCloud::Ptr template_cloud,
//       ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
//       const smmap::AllGrippersSinglePoseDelta &q_dot = {},
//       const smmap::AllGrippersSinglePose &q_config = {}, const Eigen::MatrixXi &gripper_idx = {},
//       int pred_choice = 0);

  // The common implementation that the above overloads call
  // TODO(Dylan): This should call the point cloud method (without segmentation) as it repeats the
  // same code.
  Output operator()(const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask,
      const cv::Matx33d &intrinsics, TrackingMap const& tracking_map,
      ObstacleConstraints points_normals, Eigen::RowVectorXd const max_segment_length,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},  // TODO: this should be one data structure
      const smmap::AllGrippersSinglePose &q_config = {}, int pred_choice = 0);

  // Runs the most basic implementation of CDCPD without passing all input args via method call.
  Output run(CDCPDIterationInputs const& inputs);

  // This is the most basic implementation of CDCPD.
  // No input manipulation is done for you, you're expected to pass in the inputs to the algorithm
  // as indicated by the paper.
  // TODO(Dylan): Clean up just passing in the TrackingMap. There's redundant information passing
  // in both Y and the TrackingMap since the latter contains Y.
  Output operator()(Eigen::Matrix3Xf const& Y, Eigen::VectorXf const& Y_emit_prior,
      Eigen::Matrix3Xf const& X, ObstacleConstraints obstacle_constraints,
      Eigen::RowVectorXd const max_segment_length,
      std::vector<FixedPoint> pred_fixed_points,
      TrackingMap const& tracking_map,
      const smmap::AllGrippersSinglePoseDelta &q_dot = {},  // TODO: this should be one data structure
      const smmap::AllGrippersSinglePose &q_config = {}, int pred_choice = 0);

  Eigen::Vector3f get_last_lower_bounding_box() { return last_lower_bounding_box_; }

  Eigen::Vector3f get_last_upper_bounding_box() { return last_upper_bounding_box_; }

  Eigen::VectorXf visibility_prior(const Eigen::Matrix3Xf &vertices, const cv::Mat &depth,
      const cv::Mat &mask, const Eigen::Matrix3f &intrinsics, float kvis);

  // TODO(Dylan): Pass in something that differentiates templates and their Gaussians as we want to
  // apply dynamics to the templates distinctly.
  Eigen::Matrix3Xd predict(const Eigen::Matrix3Xd &P,
      const smmap::AllGrippersSinglePoseDelta &q_dot,
      const smmap::AllGrippersSinglePose &q_config, int pred_choice);


    // Temporarily moving to public so that I can change point associations externally.
    std::shared_ptr<CPDInterface> cpd_runner_;

    // Convert CDCPD object to string representation.
    operator std::string() const
    {
        std::stringstream ss;

        ss << "CDCPD Object:" << std::endl
           << "\tlle_neighbors_: " << lle_neighbors_ << std::endl
           << "\tw_: " << w_ << std::endl
           << "\tstart_lambda_: " << start_lambda_ << std::endl
           << "\tk_: " << k_ << std::endl
           << "\tkvis_: " << kvis_ << std::endl
           << "\tobstacle_cost_weight_: " << obstacle_cost_weight_ << std::endl
           << "\tfixed_points_weight_: " << fixed_points_weight_ << std::endl
           << "\tuse_recovery_: " << use_recovery_ << std::endl
           << "\tobjective_value_threshold_: " << objective_value_threshold_ << std::endl
           << "\ttotal_frames_: " << total_frames_ << std::endl
           << "\tgripper_idx_: " << gripper_idx_ << std::endl
           << "\tlast_grasp_status_: " << last_grasp_status_ << std::endl
           << "\toriginal_template_:" << std::endl << original_template_ << std::endl
           << "\ttemplate_edges_:" << std::endl << template_edges_ << std::endl
           << "\tm_lle_:" << std::endl << m_lle_ << std::endl
           << "\tL_lle_:" << std::endl << L_lle_ << std::endl
           << "\tlast_lower_bounding_box_: " << last_lower_bounding_box_ << std::endl
           << "\tlast_upper_bounding_box_: " << last_upper_bounding_box_ << std::endl;

        return ss.str();
    }

protected:
    // ros::NodeHandle nh_;
    // ros::NodeHandle ph_;

    std::unique_ptr<smmap::ConstraintJacobianModel> constraint_jacobian_model_;
    std::unique_ptr<smmap::DiminishingRigidityModel> diminishing_rigidity_model_;

    // PastTemplateMatcher template_matcher;
    Eigen::Matrix3Xf original_template_;
    Eigen::Matrix2Xi template_edges_;



    // CdcpdParameters params;
    Eigen::Vector3f last_lower_bounding_box_;
    Eigen::Vector3f last_upper_bounding_box_;
    int lle_neighbors_;
    Eigen::MatrixXf m_lle_;
    Eigen::MatrixXf L_lle_;
    double w_;
    float start_lambda_;
    double k_;
    float kvis_;
    float obstacle_cost_weight_;
    float fixed_points_weight_;
    bool use_recovery_ = false;
    Eigen::MatrixXi gripper_idx_;
    std::shared_ptr<const sdf_tools::SignedDistanceField> sdf_ptr_;
    std::vector<bool> last_grasp_status_;
    float objective_value_threshold_;
    int total_frames_ = 0;

    // Temporary solution for decoupling ROS node handlers from CDCPD library for pybind11
    std::unique_ptr<SegmenterHSV> segmenter;

    // Perform VoxelGrid filter downsampling.
    PointCloud::Ptr downsamplePointCloud(PointCloud::Ptr cloud_in);

    // Returns a vector of `FixedPoint`s based on the gripper's point correspondance.
    std::vector<FixedPoint> getPredictedFixedPoints(Eigen::MatrixXi const& gripper_idx,
        smmap::AllGrippersSinglePose const& q_config);
};

#endif
