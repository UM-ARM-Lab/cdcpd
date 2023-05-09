#include "cdcpd/cdcpd.h"

#include <arc_utilities/enumerate.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types_conversion.h>

#include <Eigen/Dense>
#include <algorithm>
#include <arc_utilities/ros_helpers.hpp>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fgt.hpp>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sdf_tools/sdf.hpp>
#include <string>

#include "cdcpd/obs_util.h"

#include <iostream>

std::string const LOGNAME = "cdcpd";

using cv::Mat;
using cv::Vec3b;
using Eigen::ArrayXd;
using Eigen::ArrayXf;
using Eigen::Isometry3d;
using Eigen::Matrix2Xi;
using Eigen::Matrix3f;
using Eigen::Matrix3Xd;
using Eigen::Matrix3Xf;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::RowVectorXf;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXd;
using Eigen::VectorXf;
using Eigen::VectorXi;

class CompHSV : public pcl::ComparisonBase<PointHSV>
{
  using ComparisonBase<PointHSV>::capable_;
  using ComparisonBase<PointHSV>::op_;

 public:
  /** \brief Construct a CompHSV
   * \param component_name either "h", "s" or "v"
   * \param op the operator to use when making the comparison
   * \param compare_val the constant value to compare the component value too
   */
  CompHSV(const std::string &component_name, pcl::ComparisonOps::CompareOp op, double compare_val)
      : component_offset_(),
        compare_val_(compare_val)
  {
    // Verify the component name
    if (component_name == "h") {
      component_offset_ = 16;
    } else if (component_name == "s") {
      component_offset_ = 20;
    } else if (component_name == "v") {
      component_offset_ = 24;
    } else {
      PCL_WARN("[pcl::CompHSV::CompHSV] unrecognized component name!\n");
      capable_ = false;
      return;
    }

    // save the rest of the context
    capable_ = true;
    op_ = op;
  }

  [[nodiscard]] bool evaluate(const PointHSV &point) const override {
    // extract the component value
    const auto *pt_data = reinterpret_cast<const std::uint8_t *>(&point);
    std::uint8_t my_val = *(pt_data + component_offset_);

    // now do the comparison
    switch (this->op_) {
      case pcl::ComparisonOps::GT:
        return (my_val > this->compare_val_);
      case pcl::ComparisonOps::GE:
        return (my_val >= this->compare_val_);
      case pcl::ComparisonOps::LT:
        return (my_val < this->compare_val_);
      case pcl::ComparisonOps::LE:
        return (my_val <= this->compare_val_);
      case pcl::ComparisonOps::EQ:
        return (my_val == this->compare_val_);
      default:
        PCL_WARN("[pcl::CompHSV::evaluate] unrecognized op_!\n");
        return (false);
    }
  }

 protected:
  std::uint32_t component_offset_;
  double compare_val_;

 private:
  CompHSV() : component_offset_(), compare_val_() {}  // not allowed
};

static PointCloud::Ptr mat_to_cloud(const Eigen::Matrix3Xf &mat)
{
  PointCloud::Ptr cloud(new PointCloud);
  cloud->points.reserve(mat.cols());
  for (ssize_t i = 0; i < mat.cols(); ++i) {
    cloud->push_back(pcl::PointXYZ(mat(0, i), mat(1, i), mat(2, i)));
  }
  return cloud;
}

MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
    int lle_neighbors, double reg)
{
  // calculate L in Eq. (15) and Eq. (16)
  // ENHANCE: use tapkee lib to accelarate
  // kdtree: kdtree from Y^0
  // lle_neighbors: parameter for lle calculation
  // reg: regularization term (necessary when lle_neighbor > dimension)
  PointCloud::ConstPtr cloud = kdtree.getInputCloud();
  uint32_t num_tracked_verts = cloud->width;

  // std::cout << "Cloud height: " << cloud->height << std::endl;
  // std::cout << "Cloud width: " << cloud->width << std::endl;
  assert(cloud->height == 1);
  // std::cout << "After assert\n";
  // adjacencies: save index of adjacent points
  MatrixXi adjacencies = MatrixXi(num_tracked_verts, lle_neighbors);
  // B: save weight W_ij
  MatrixXf B = MatrixXf::Zero(num_tracked_verts, lle_neighbors);
  MatrixXf v = VectorXf::Ones(lle_neighbors);
  // algorithm: see https://cs.nyu.edu/~roweis/lle/algorithm.html
  // std::cout << "before loop\n";
  for (size_t i = 0; i < num_tracked_verts; ++i) {
    std::vector<int> neighbor_inds(lle_neighbors + 1);
    std::vector<float> neighbor_dists(lle_neighbors + 1);
    kdtree.nearestKSearch(i, lle_neighbors + 1, neighbor_inds, neighbor_dists);
    // C: transpose of Z in Eq [d] and [e]
    MatrixXf C(lle_neighbors, 3);
    for (size_t j = 1; j < neighbor_inds.size(); ++j) {
      C.row(j - 1) = cloud->points[neighbor_inds[j]].getVector3fMap() - cloud->points[i].getVector3fMap();
      adjacencies(i, j - 1) = neighbor_inds[j];
    }
    // G: C in Eq [f]
    MatrixXf G = C * C.transpose();
    // ???: why += R for G
    auto R = reg;
    auto const tr = G.trace();
    if (tr > 0) {
      R *= tr;
    }
    G.diagonal().array() += R;
    VectorXf w = G.llt().solve(v);
    B.row(i) = w / w.sum();
  }
  // std::cout << "Loop done\n";
  MatrixXf graph = MatrixXf::Zero(num_tracked_verts, num_tracked_verts);
  for (ssize_t i = 0; i < graph.rows(); ++i) {
    for (ssize_t j = 0; j < lle_neighbors; ++j) {
      graph(i, adjacencies(i, j)) = B(i, j);
    }
  }
  return graph;
}

MatrixXf locally_linear_embedding(PointCloud::ConstPtr template_cloud, int lle_neighbors,
    double reg)
{
  // std::cout << "LLE begin\n";
  // calculate H in Eq. (18)
  // template_cloud: Y^0 in Eq. (15) and (16)
  // lle_neighbors: parameter for lle calculation
  // reg: regularization term (seems unnecessary)
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(template_cloud);
  // std::cout << "after setting input cloud\n";
  // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
  MatrixXf W = barycenter_kneighbors_graph(kdtree, lle_neighbors, reg);
  // M: (M, M) matrix, corresponding to H in Eq. (18)
  // std::cout << "after barycenter_kneighbors_graph\n";
  MatrixXf M = (W.transpose() * W) - W.transpose() - W;
  M.diagonal().array() += 1;
  // std::cout << "LLE end\n";
  return M;
}

CDCPD_Parameters::CDCPD_Parameters(ros::NodeHandle& ph)
  : objective_value_threshold(ROSHelpers::GetParam<double>(ph, "objective_value_threshold", 1.0)),
    alpha(ROSHelpers::GetParam<double>(ph, "alpha", 0.5)),
    lambda(ROSHelpers::GetParam<double>(ph, "lambda", 1.0)),
    k_spring(ROSHelpers::GetParam<double>(ph, "k", 100.0)),
    beta(ROSHelpers::GetParam<double>(ph, "beta", 1.0)),
    zeta(ROSHelpers::GetParam<double>(ph, "zeta", 10.0)),
    min_distance_threshold(ROSHelpers::GetParam<double>(ph, "min_distance_threshold", 0.01)),
    obstacle_cost_weight(ROSHelpers::GetParam<double>(ph, "obstacle_cost_weight", 0.001)),
    fixed_points_weight(ROSHelpers::GetParam<double>(ph, "fixed_points_weight", 10.0)),
    use_recovery(ROSHelpers::GetParam<bool>(ph, "use_recovery", false))
{}

/*
 * Return a non-normalized probability that each of the tracked vertices produced any detected point.
 * Implement Eq. (7) in the paper
 */
VectorXf CDCPD::visibility_prior(const Matrix3Xf &vertices, const Mat &depth, const Mat &mask,
                                 const Matrix3f &intrinsics, const float kvis)
{
  // vertices: (3, M) matrix Y^t (Y in IV.A) in the paper
  // depth: CV_16U depth image
  // mask: CV_8U mask for segmentation
  // kvis: k_vis in the paper

  // Project the vertices to get their corresponding pixel coordinate
  // ENHANCE: replace P_eigen.leftCols(3) with intrinsics
  Matrix3Xf image_space_vertices = intrinsics * vertices;

  // Homogeneous coords: divide by third row
  // the for-loop is unnecessary
  image_space_vertices.row(0).array() /= image_space_vertices.row(2).array();
  image_space_vertices.row(1).array() /= image_space_vertices.row(2).array();
  for (int i = 0; i < image_space_vertices.cols(); ++i) {
    float x = image_space_vertices(0, i);
    float y = image_space_vertices(1, i);
    image_space_vertices(0, i) = std::min(std::max(x, 0.0f), static_cast<float>(depth.cols));
    image_space_vertices(1, i) = std::min(std::max(y, 0.0f), static_cast<float>(depth.rows));
  }

  // Get image coordinates
  Eigen::Matrix2Xi coords = image_space_vertices.topRows(2).cast<int>();

  for (int i = 0; i < coords.cols(); ++i) {
    coords(0, i) = std::min(std::max(coords(0, i), 0), depth.cols - 1);
    coords(1, i) = std::min(std::max(coords(1, i), 1), depth.rows - 1);
  }

  // Find difference between point depth and image depth
  // depth_diffs: (1, M) vector
  Eigen::VectorXf depth_diffs = Eigen::VectorXf::Zero(vertices.cols());
  for (int i = 0; i < vertices.cols(); ++i) {
    // std::cout << "depth at " << coords(1, i) << " " << coords(0, i) << std::endl;
    uint16_t raw_depth = depth.at<uint16_t>(coords(1, i), coords(0, i));
    if (raw_depth != 0) {
      depth_diffs(i) = vertices(2, i) - static_cast<float>(raw_depth) / 1000.0;
    } else {
      depth_diffs(i) = 0.02;  // prevent numerical problems; taken from the Python code
    }
  }
  // ENHANCE: repeating max
  depth_diffs = depth_diffs.array().max(0);

  Eigen::VectorXf depth_factor = depth_diffs.array().max(0.0);
  cv::Mat dist_img(depth.rows, depth.cols, CV_32F);  // TODO haven't really tested this but seems right
  int maskSize = 5;
  cv::distanceTransform(~mask, dist_img, cv::noArray(), cv::DIST_L2, maskSize);
  // ???: why cv::normalize is needed
  cv::normalize(dist_img, dist_img, 0.0, 1.0, cv::NORM_MINMAX);

  Eigen::VectorXf dist_to_mask(vertices.cols());
  for (int i = 0; i < vertices.cols(); ++i) {
    dist_to_mask(i) = dist_img.at<float>(coords(1, i), coords(0, i));
  }
  VectorXf score = (dist_to_mask.array() * depth_factor.array()).matrix();
  VectorXf prob = (-kvis * score).array().exp().matrix();
  // ???: unnormalized prob
  return prob;
}

// NOTE: based on the implementation here:
// https://github.com/ros-perception/image_pipeline/blob/melodic/depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp
// we expect that cx, cy, fx, fy are in the appropriate places in P
static std::tuple<PointCloudRGB::Ptr, PointCloud::Ptr> point_clouds_from_images(
    const cv::Mat &depth_image, const cv::Mat &rgb_image, const cv::Mat &mask,
    const Eigen::Matrix3f &intrinsics, const Eigen::Vector3f &lower_bounding_box,
    const Eigen::Vector3f &upper_bounding_box)
{
  // depth_image: CV_16U depth image
  // rgb_image: CV_8U3C rgb image
  // mask: CV_8U mask for segmentation
  // intrinsic matrix of Kinect using the Pinhole camera model
  //  [[fx 0  px];
  //   [0  fy py];
  //   [0  0  1 ]]
  // lower_bounding_box_vec: bounding for mask
  // upper_bounding_box_vec: bounding for mask

  float pixel_len;
  cv::Mat local_depth_image;
  depth_image.convertTo(local_depth_image, CV_32F);
  pixel_len = 0.0002645833;

  // Use correct principal point from calibration
  auto const center_x = intrinsics(0, 2);
  auto const center_y = intrinsics(1, 2);

  auto const unit_scaling = 0.001;
  auto const constant_x = 1.0f / (intrinsics(0, 0) * pixel_len);
  auto const constant_y = 1.0f / (intrinsics(1, 1) * pixel_len);
  auto constexpr bad_point = std::numeric_limits<float>::quiet_NaN();

  PointCloud::Ptr filtered_cloud(new PointCloud);
  PointCloudRGB::Ptr unfiltered_cloud(new PointCloudRGB(depth_image.cols, depth_image.rows));
  auto unfiltered_iter = unfiltered_cloud->begin();

  for (int v = 0; v < depth_image.rows; ++v) {
    for (int u = 0; u < depth_image.cols; ++u) {
      float depth = local_depth_image.at<float>(v, u);

      // Assume depth = 0 is the standard was to note invalid
      if (std::isfinite(depth)) {
        float x = (float(u) - center_x) * pixel_len * float(depth) * unit_scaling * constant_x;
        float y = (float(v) - center_y) * pixel_len * float(depth) * unit_scaling * constant_y;
        float z = float(depth) * unit_scaling;
        // Add to unfiltered cloud
        // ENHANCE: be more concise
        unfiltered_iter->x = x;
        unfiltered_iter->y = y;
        unfiltered_iter->z = z;
        unfiltered_iter->r = rgb_image.at<Vec3b>(v, u)[0];
        unfiltered_iter->g = rgb_image.at<Vec3b>(v, u)[1];
        unfiltered_iter->b = rgb_image.at<Vec3b>(v, u)[2];

        Eigen::Array<float, 3, 1> point(x, y, z);
        if (mask.at<bool>(v, u) && point.min(upper_bounding_box.array()).isApprox(point) &&
            point.max(lower_bounding_box.array()).isApprox(point)) {
          filtered_cloud->push_back(pcl::PointXYZ(x, y, z));
        }
      } else {
        unfiltered_iter->x = unfiltered_iter->y = unfiltered_iter->z = bad_point;
      }
      ++unfiltered_iter;
    }
  }

  assert(unfiltered_iter == unfiltered_cloud->end());
  return {unfiltered_cloud, filtered_cloud};
}

Matrix3Xd CDCPD::predict(const Matrix3Xd &P, const smmap::AllGrippersSinglePoseDelta &q_dot,
                         const smmap::AllGrippersSinglePose &q_config, const int pred_choice)
{
  if (pred_choice == 0) {
    return P;
  } else {
    smmap::WorldState world;
    world.object_configuration_ = P;
    world.all_grippers_single_pose_ = q_config;
    if (pred_choice == 1) {
      return constraint_jacobian_model_->getObjectDelta(world, q_dot) + P;
    } else {
      return diminishing_rigidity_model_->getObjectDelta(world, q_dot) + P;
    }
  }
}

// TEMPORARY FOR PYTHON BINDING.
// However, I might change to using just a TrackingMap and CDCPD_Parameters instance for
// initialization.
CDCPD::CDCPD(TrackingMap const& tracking_map, const float objective_value_threshold,
    const bool use_recovery, const double alpha, const double beta, const double lambda,
    const double k, const float zeta, const float obstacle_cost_weight,
    const float fixed_points_weight)
    : CDCPD(tracking_map.form_vertices_cloud(), tracking_map.form_edges_matrix(),
    objective_value_threshold, use_recovery, alpha, beta, lambda, k, zeta, obstacle_cost_weight,
    fixed_points_weight)
{}

// This is for the case where the gripper indices are unknown (in real experiment)
CDCPD::CDCPD(PointCloud::ConstPtr template_cloud,  // this needs a different data-type for python
    const Matrix2Xi &_template_edges, const float objective_value_threshold, const bool use_recovery,
    const double alpha, const double beta, const double lambda, const double k, const float zeta,
    const float obstacle_cost_weight, const float fixed_points_weight)
//     : CDCPD(ros::NodeHandle(), ros::NodeHandle("~"), template_cloud, template_edges,
//         objective_value_threshold, use_recovery, alpha, beta, lambda, k, zeta, obstacle_cost_weight,
//         fixed_points_weight)
// {}

// CDCPD::CDCPD(ros::NodeHandle nh, ros::NodeHandle ph, PointCloud::ConstPtr template_cloud,
//     const Matrix2Xi &_template_edges, const float objective_value_threshold,
//     const bool use_recovery, const double alpha, const double beta, const double lambda,
//     const double k, const float zeta, const float obstacle_cost_weight,
//     const float fixed_points_weight)
//     : nh_(nh),
//       ph_(ph),
      : original_template_(template_cloud->getMatrixXfMap().topRows(3)),
        template_edges_(_template_edges),
        last_lower_bounding_box_(original_template_.rowwise().minCoeff()),       // TODO make configurable?
        last_upper_bounding_box_(original_template_.rowwise().maxCoeff()),       // TODO make configurable?
        // lle_neighbors_(std::min(8, static_cast<int>(original_template_.cols()))), // TODO make
        // configurable?
        lle_neighbors_(2),
        // m_lle_(locally_linear_embedding(template_cloud, lle_neighbors_, 1e-3)),  // TODO make
        m_lle_(MatrixXf::Zero(8, 8)),
        // configurable?
        // m_lle_(Eigen::MatrixXf::Zero(original_template_.cols(), original_template_.cols())),
        w_(0.1),                                                                // TODO make configurable?
        start_lambda_(lambda),
        k_(k),
        kvis_(1e3),
        obstacle_cost_weight_(obstacle_cost_weight),
        fixed_points_weight_(fixed_points_weight),
        use_recovery_(use_recovery),
        last_grasp_status_({false, false}),
        objective_value_threshold_(objective_value_threshold)
{
  // Temporary for Python binding testing.
  ros::Time::init();
  // std::cout << "Past initializer list\n";
  last_lower_bounding_box_ = last_lower_bounding_box_ - bounding_box_extend;
  last_upper_bounding_box_ = last_upper_bounding_box_ + bounding_box_extend;

  // TODO(Dylan): Make these parameters configurable via ROS params?
  double const tolerance_cpd = 1e-4;
  double const initial_sigma_scale = 1.0 / 8.0;
  int const max_cpd_iterations = 100;

  bool const doing_multitemplate_tracking = false;

  if (doing_multitemplate_tracking)
  {
    std::cout << "Make multi-template LLE more general? Depends on how we proceed with algorithm"
      << std::endl;
    // Need to make LLE respect separate templates. For right now I'm going to hard code in a split in
    // the original template but this should likely be recomputed at every time step.
    // 1. split the template cloud into two separate clouds.
    PointCloud::Ptr template_cloud1(new PointCloud);
    PointCloud::Ptr template_cloud2(new PointCloud);
    // = template_cloud->
    for (int i = 0; i < 4; ++i)
    {
      template_cloud1->push_back(template_cloud->points[i]);
    }
    for (int i = 4; i < 8; ++i)
    {
      template_cloud2->push_back(template_cloud->points[i]);
    }

    // ****** m_lle_ ?? This appears to be the same thing as L_lle_ below but I'm pressed for time and
    // can't look into it.
    auto m_lle_1 = locally_linear_embedding(template_cloud1, lle_neighbors_, 1e-3);
    auto m_lle_2 = locally_linear_embedding(template_cloud2, lle_neighbors_, 1e-3);
    m_lle_ = MatrixXf::Zero(8, 8);
    m_lle_.block(0, 0, 4, 4) = m_lle_1;
    m_lle_.block(4, 4, 4, 4) = m_lle_2;

    // ****** Weight Matrix
    // for template 1
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1;
    kdtree1.setInputCloud(template_cloud1);
    // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
    auto L_lle_1 = barycenter_kneighbors_graph(kdtree1, lle_neighbors_, 0.001);

    // for template 2
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree2;
    kdtree2.setInputCloud(template_cloud2);
    // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
    auto L_lle_2 = barycenter_kneighbors_graph(kdtree2, lle_neighbors_, 0.001);

    // Combine the two LLEs by using matrix blocks. The matrix should be partioned into a
    // block-diagonal matrix where there are zeros in the upper right and lower left blocks.
    L_lle_ = MatrixXf::Zero(8, 8);
    L_lle_.block(0, 0, 4, 4) = L_lle_1;
    L_lle_.block(4, 4, 4, 4) = L_lle_2;
  }
  else
  {
    // Need to also make changes to how original CPD is run since it's trying to do an LLE embedding
    // with more neighbors than actual vertices in the template.
    int const num_vertices = template_cloud->width;
    if (num_vertices <= lle_neighbors_)
    {
      std::cout << "Skipping LLE since number of LLE neighbors (" << lle_neighbors_
        << ") > than number of template vertices (" << num_vertices << ")" << std::endl;
      m_lle_ = MatrixXf::Zero(num_vertices, num_vertices);
      L_lle_ = MatrixXf::Zero(num_vertices, num_vertices);
    }
    else
    {
      m_lle_ = locally_linear_embedding(template_cloud, lle_neighbors_, 1e-3);

      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud(template_cloud);
      // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
      L_lle_ = barycenter_kneighbors_graph(kdtree, lle_neighbors_, 0.001);
    }

  }

  // std::cout << "Full L_lle_:" << L_lle_ << std::endl;

  // TODO(Dylan): Make choice of CPD algorithm here based on ROS params or using a CDCPD builder.
  // std::cout << "Before CPD construction\n";
  if (doing_multitemplate_tracking)
  {
    cpd_runner_ = std::make_shared<CPDMultiTemplateExternalPointAssignment>(LOGNAME, tolerance_cpd,
        max_cpd_iterations, initial_sigma_scale, w_, alpha, beta, zeta, start_lambda_, m_lle_);
  }
  else
  {
    cpd_runner_ = std::make_shared<CPD>(LOGNAME, tolerance_cpd, max_cpd_iterations,
      initial_sigma_scale, w_, alpha, beta, zeta, start_lambda_, m_lle_);
  }
  // std::cout << "Before CPD casting\n";

  // Upcast the cpd_runner_choice to the CPDInterface that we'll interact with to run CPD.
  // cpd_runner_ = cpd_runner_choice;
}

CDCPD::Output CDCPD::operator()(const Mat &rgb, const Mat &depth, const Mat &mask,
    const cv::Matx33d &intrinsics, TrackingMap const& tracking_map,
    ObstacleConstraints obstacle_constraints, Eigen::RowVectorXd const max_segment_length,
    const smmap::AllGrippersSinglePoseDelta &q_dot, const smmap::AllGrippersSinglePose &q_config,
    const std::vector<bool> &is_grasped, const int pred_choice)
{
  std::vector<int> idx_map;
  for (auto const &[j, is_grasped_j] : enumerate(is_grasped)) {
    if (j < q_config.size() and j < q_dot.size()) {
      idx_map.push_back(j);
    } else {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "is_grasped index " << j << " given but only "
          << q_config.size() << " gripper configs and " << q_dot.size()
          << " gripper velocities given.");
    }
  }

  // associate each gripper with the closest point in the current estimate
  if (is_grasped != last_grasp_status_) {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "grasp status changed, recomputing correspondences");

    // get the previous tracking result
    PointCloud::ConstPtr const template_cloud = tracking_map.form_vertices_cloud();
    const Matrix3Xf Y = template_cloud->getMatrixXfMap().topRows(3);

    auto const num_gripper = idx_map.size();
    MatrixXi grippers(1, num_gripper);
    for (auto g_idx = 0u; g_idx < num_gripper; g_idx++) {
      Vector3f gripper_pos = q_config[idx_map[g_idx]].matrix().cast<float>().block<3, 1>(0, 3);
      MatrixXf dist = (Y.colwise() - gripper_pos).colwise().norm();
      // FIXME: this is missing a "min" of some kind
      MatrixXf::Index minCol;
      grippers(0, g_idx) = static_cast<int>(minCol);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "closest point index: " << minCol);
    }

    gripper_idx_ = grippers;

    {
      std::vector<smmap::GripperData> grippers_data;

      // format grippers_data
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "gripper data when constructing CDCPD:" << grippers);
      for (int g_idx = 0; g_idx < grippers.cols(); g_idx++) {
        std::vector<long> grip_node_idx;
        for (int node_idx = 0; node_idx < grippers.rows(); node_idx++) {
          grip_node_idx.push_back(long(grippers(node_idx, g_idx)));
          ROS_DEBUG_STREAM_NAMED(LOGNAME, "grasp point: " << grippers(node_idx, g_idx));
        }
        std::string gripper_name;
        gripper_name = "gripper" + std::to_string(g_idx);
        smmap::GripperData gripper(gripper_name, grip_node_idx);
        grippers_data.push_back(gripper);
      }
    }
  }

  auto const cdcpd_out = operator()(rgb, depth, mask, intrinsics, tracking_map,
                                    obstacle_constraints, max_segment_length, q_dot, q_config,
                                    pred_choice);

  last_grasp_status_ = is_grasped;

  return cdcpd_out;
}

// NOTE: this is the one I'm current using for rgb + depth
CDCPD::Output CDCPD::operator()(const Mat &rgb, const Mat &depth, const Mat &mask,
    const cv::Matx33d &intrinsics, TrackingMap const& tracking_map,
    ObstacleConstraints obstacle_constraints, Eigen::RowVectorXd const max_segment_length,
    const smmap::AllGrippersSinglePoseDelta &q_dot, const smmap::AllGrippersSinglePose &q_config,
    const Eigen::MatrixXi &gripper_idx, const int pred_choice)
{
  this->gripper_idx_ = gripper_idx;
  auto const cdcpd_out = operator()(rgb, depth, mask, intrinsics, tracking_map,
                                    obstacle_constraints,
                                    max_segment_length, q_dot, q_config, pred_choice);
  return cdcpd_out;
}

// NOTE: for point cloud inputs that need to be segmented/filtered.
CDCPD::Output CDCPD::operator()(const PointCloudRGB::Ptr &points,
    TrackingMap const& tracking_map, ObstacleConstraints obstacle_constraints,
    Eigen::RowVectorXd const max_segment_length, const smmap::AllGrippersSinglePoseDelta &q_dot,
    const smmap::AllGrippersSinglePose &q_config, const Eigen::MatrixXi &gripper_idx,
    const int pred_choice)
{
  Stopwatch stopwatch_cdcpd("CDCPD");
  // FIXME: this has a lot of duplicate code
  this->gripper_idx_ = gripper_idx;

  // template_cloud: point clouds corresponding to Y^t (Y in IV.A) in the paper
  // template_edges: (2, K) matrix corresponding to E in the paper

  // Perform HSV segmentation
  boost::shared_ptr<PointCloud> cloud_segmented;
  PointCloud::Ptr cloud_downsampled(new PointCloud);
  {
    Stopwatch stopwatch_segmentation("Segmentation");
    // auto segmenter = std::make_unique<SegmenterHSV>(ph_, last_lower_bounding_box_,
    //     last_upper_bounding_box_);
    segmenter->set_last_lower_bounding_box(last_lower_bounding_box_);
    segmenter->set_last_upper_bounding_box(last_upper_bounding_box_);
    segmenter->segment(points);

    // Drop color info from the point cloud.
    // NOTE: We use a boost pointer here because that's what our version of pcl is expecting in
    // the `setInputCloud` function call.
    cloud_segmented = boost::make_shared<PointCloud>();
    pcl::copyPointCloud(segmenter->get_segmented_cloud(), *cloud_segmented);

    cloud_downsampled = downsamplePointCloud(cloud_segmented);
  }

  PointCloud::ConstPtr const template_cloud = tracking_map.form_vertices_cloud();
  const Matrix3Xf Y = template_cloud->getMatrixXfMap().topRows(3);

  // TODO: check whether the change is needed here for unit conversion
  auto const Y_emit_prior = Eigen::VectorXf::Ones(template_cloud->size());
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Y_emit_prior " << Y_emit_prior);

  if (cloud_downsampled->width == 0) {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "No points in the filtered point cloud");
    std::cout << "No points in filtered cloud" << std::endl;
    PointCloud::Ptr cdcpd_out = mat_to_cloud(Y);
    PointCloud::Ptr cdcpd_cpd = mat_to_cloud(Y);
    PointCloud::Ptr cdcpd_pred = mat_to_cloud(Y);

    return CDCPD::Output{points, cloud_segmented, cloud_downsampled, cdcpd_cpd, cdcpd_pred,
        cdcpd_out, OutputStatus::NoPointInFilteredCloud};
  }

  // Add points to X according to the previous template
  Matrix3Xf X = cloud_downsampled->getMatrixXfMap().topRows(3);

  std::vector<FixedPoint> pred_fixed_points = getPredictedFixedPoints(gripper_idx, q_config);

  Output output = operator()(Y, Y_emit_prior, X, obstacle_constraints, max_segment_length,
    pred_fixed_points, tracking_map, q_dot, q_config, pred_choice);
  output.original_cloud = points;
  output.masked_point_cloud = cloud_segmented;
  output.downsampled_cloud = cloud_downsampled;

  return output;
}

CDCPD::Output CDCPD::operator()(const Mat &rgb, const Mat &depth, const Mat &mask,
    const cv::Matx33d &intrinsics, TrackingMap const& tracking_map,
    ObstacleConstraints obstacle_constraints, Eigen::RowVectorXd const max_segment_length,
    const smmap::AllGrippersSinglePoseDelta &q_dot, const smmap::AllGrippersSinglePose &q_config,
    const int pred_choice)
{
  // rgb: CV_8U3C rgb image
  // depth: CV_16U depth image
  // mask: CV_8U mask for segmentation
  // template_cloud: point clouds corresponding to Y^t (Y in IV.A) in the paper
  // template_edges: (2, K) matrix corresponding to E in the paper

  assert(rgb.type() == CV_8UC3);
  assert(depth.type() == CV_16U);
  assert(mask.type() == CV_8U);
  assert(rgb.rows == depth.rows && rgb.cols == depth.cols);

  Eigen::Matrix3d intrinsics_eigen_tmp;
  cv::cv2eigen(intrinsics, intrinsics_eigen_tmp);
  Eigen::Matrix3f intrinsics_eigen = intrinsics_eigen_tmp.cast<float>();

  // entire_cloud: pointer to the entire point cloud
  // cloud: pointer to the point clouds selected
  auto [entire_cloud, cloud] = point_clouds_from_images(depth, rgb, mask, intrinsics_eigen,
      last_lower_bounding_box_ - bounding_box_extend,
      last_upper_bounding_box_ + bounding_box_extend);
  ROS_INFO_STREAM_THROTTLE_NAMED(1, LOGNAME + ".points",
      "Points in filtered: (" << cloud->height << " x " << cloud->width << ")");

  PointCloud::ConstPtr const template_cloud = tracking_map.form_vertices_cloud();
  const Matrix3Xf Y = template_cloud->getMatrixXfMap().topRows(3);
  // TODO: check whether the change is needed here for unit conversion
  Eigen::VectorXf Y_emit_prior = visibility_prior(Y, depth, mask, intrinsics_eigen, kvis_);
  PointCloud::Ptr cloud_downsampled = downsamplePointCloud(cloud);

  if (cloud_downsampled->width == 0) {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "No points in the filtered point cloud");
    PointCloud::Ptr cdcpd_out = mat_to_cloud(Y);
    PointCloud::Ptr cdcpd_cpd = mat_to_cloud(Y);
    PointCloud::Ptr cdcpd_pred = mat_to_cloud(Y);

    return CDCPD::Output{entire_cloud, cloud, cloud_downsampled, cdcpd_cpd, cdcpd_pred, cdcpd_out,
      OutputStatus::NoPointInFilteredCloud};
  }
  Matrix3Xf X = cloud_downsampled->getMatrixXfMap().topRows(3);
  // Add points to X according to the previous template

  const Matrix3Xf &entire = entire_cloud->getMatrixXfMap().topRows(3);

  std::vector<FixedPoint> pred_fixed_points = getPredictedFixedPoints(gripper_idx_, q_config);

  Output output = operator()(Y, Y_emit_prior, X, obstacle_constraints, max_segment_length,
    pred_fixed_points, tracking_map, q_dot, q_config, pred_choice);
  output.original_cloud = entire_cloud;
  output.masked_point_cloud = cloud;
  output.downsampled_cloud = cloud_downsampled;

  return output;
}

CDCPD::Output CDCPD::run(CDCPDIterationInputs const& in)
{
  Eigen::Matrix3Xf const Y = in.tracking_map.form_vertices_cloud()->getMatrixXfMap().topRows(3);
  Eigen::RowVectorXd max_segment_lengths = in.tracking_map.form_max_segment_length_matrix();
  return (*this)(Y, in.Y_emit_prior, in.X, in.obstacle_constraints, max_segment_lengths,
    in.pred_fixed_points, in.tracking_map, in.q_dot, in.q_config, in.pred_choice);
}

CDCPD::Output CDCPD::operator()(Eigen::Matrix3Xf const& Y, Eigen::VectorXf const& Y_emit_prior,
      Eigen::Matrix3Xf const& X, ObstacleConstraints obstacle_constraints,
      Eigen::RowVectorXd const max_segment_length,
      std::vector<FixedPoint> pred_fixed_points,
      TrackingMap const& tracking_map,
      const smmap::AllGrippersSinglePoseDelta &q_dot,  // TODO: this should be one data structure
      const smmap::AllGrippersSinglePose &q_config, int pred_choice)
{
  total_frames_ += 1;

  // CPD and prediction using dynamics model.
  Matrix3Xf TY, TY_pred;
  {
    Stopwatch stopwatch_cpd("CPD");
    TY_pred = predict(Y.cast<double>(), q_dot, q_config, pred_choice).cast<float>();
    TY = (*cpd_runner_)(X, Y, TY_pred, Y_emit_prior, tracking_map);
  }

  ROS_DEBUG_STREAM_NAMED(LOGNAME, "fixed points" << pred_fixed_points);

  // Next step: optimization.
  Matrix3Xf Y_opt;
  double objective_value;
  {
    Stopwatch stopwatch_optimization("Optimization");

    // NOTE: seems like this should be a function, not a class? is there state like the gurobi env?
    // ???: most likely not 1.0
    Optimizer opt(original_template_, Y, start_lambda_, obstacle_cost_weight_, fixed_points_weight_);
    // std::cout << "Past Optimizer constructor" << std::endl;
    auto const opt_out = opt(TY, template_edges_, pred_fixed_points, obstacle_constraints,
        max_segment_length);
    Y_opt = opt_out.first;
    objective_value = opt_out.second;

    ROS_DEBUG_STREAM_NAMED(LOGNAME + ".objective", "objective: " << objective_value);
    // std::cout << "objective: " << objective_value << std::endl;
  }

  // NOTE: set stateful member variables for next time
  last_lower_bounding_box_ = Y_opt.rowwise().minCoeff();
  last_upper_bounding_box_ = Y_opt.rowwise().maxCoeff();

  PointCloud::Ptr cdcpd_out = mat_to_cloud(Y_opt);
  PointCloud::Ptr cdcpd_cpd = mat_to_cloud(TY);
  PointCloud::Ptr cdcpd_pred = mat_to_cloud(TY_pred);

  auto status = OutputStatus::Success;
  // std::cout << "Output status = success" << std::endl;
  if (total_frames_ > 10 and objective_value > objective_value_threshold_) {
    ROS_WARN_STREAM_NAMED(LOGNAME + ".objective", "Objective too high!");
    status = OutputStatus::ObjectiveTooHigh;
  }

  Output output;
  output.cpd_output = cdcpd_cpd;
  output.cpd_predict = cdcpd_pred;
  output.gurobi_output = cdcpd_out;
  output.status = status;

  return output;
}

Eigen::Matrix3Xf CDCPD::downsampleMatrixCloud(Eigen::Matrix3Xf mat_in)
{
    PointCloud::Ptr cloud_in = mat_to_cloud(mat_in);
    PointCloud::Ptr cloud_downsampled = downsamplePointCloud(cloud_in);
    return cloud_downsampled->getMatrixXfMap().topRows(3);
}

// Perform VoxelGrid filter downsampling.
PointCloud::Ptr CDCPD::downsamplePointCloud(PointCloud::Ptr cloud_in)
{
    PointCloud::Ptr cloud_downsampled(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME + ".points", "Points in cloud before leaf: "
        << cloud_in->width);
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*cloud_downsampled);
    ROS_INFO_STREAM_THROTTLE_NAMED(1, LOGNAME + ".points",
                                  "Points in filtered point cloud: " << cloud_downsampled->width);
    return cloud_downsampled;
}

std::vector<FixedPoint> CDCPD::getPredictedFixedPoints(Eigen::MatrixXi const& gripper_idx,
    smmap::AllGrippersSinglePose const& q_config)
{
    std::vector<FixedPoint> pred_fixed_points;
    auto const num_grippers = std::min(
        static_cast<size_t>(gripper_idx.cols()), static_cast<size_t>(q_config.size()));
    for (auto col = 0u; col < num_grippers; ++col) {
        FixedPoint pt;
        pt.template_index = gripper_idx(0, col);
        pt.position(0) = q_config[col](0, 3);
        pt.position(1) = q_config[col](1, 3);
        pt.position(2) = q_config[col](2, 3);
        pred_fixed_points.push_back(pt);
    }
    return pred_fixed_points;
}