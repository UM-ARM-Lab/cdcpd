#include <algorithm>
#include <cmath>
#include <cassert>
#include <chrono>
#include <string>

#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <sdf_tools/collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/enumerate.h>
#include <fgt.hpp>

#include "cdcpd/obs_util.h"
#include "cdcpd/cdcpd.h"

#include <iostream>
#include <fstream>
#include <random>

auto constexpr const LOGNAME = "cdcpd";

using cv::Mat;
using cv::Vec3b;
using Eigen::ArrayXf;
using Eigen::ArrayXd;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Matrix3Xf;
using Eigen::Matrix3Xd;
using Eigen::Matrix3f;
using Eigen::MatrixXi;
using Eigen::Matrix2Xi;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::RowVectorXf;
using Eigen::Isometry3d;

static double calculate_prob_reg(const Matrix3Xf &X,
                                 const Matrix3Xf &TY,
                                 const MatrixXf &G,
                                 const MatrixXf &W,
                                 const double sigma2,
                                 const VectorXf &Y_emit_prior)
{
  int M = TY.cols();
  int N = X.cols();
  int D = X.rows();
  MatrixXf P(M, N); // end = std::chrono::system_clock::now(); std::cout << "526: " << (end-start).count() << std::endl;
  for (int i = 0; i < M; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      P(i, j) = (X.col(j) - TY.col(i)).squaredNorm();
    }
  }

  float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
  float w = 0.1;
  c *= w / (1 - w);
  c *= static_cast<float>(M) / N;

  P = (-P / (2 * sigma2)).array().exp().matrix();
  P.array().colwise() *= Y_emit_prior.array();

  RowVectorXf den = P.colwise().sum();
  den.array() += c;

  P = P.array().rowwise() / den.array();

  double reg = 0.0;
  for (int i = 0; i < M; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      reg += P(i, j) * ((X.col(j) - (TY.col(i) + (G * W).transpose().col(i))).squaredNorm());
    }
  }
  reg = reg / (2 * sigma2);
  reg = reg + P.sum() * 3.0 * log(sigma2) / 2.0;
  return reg;
}

static double calculate_prob_reg(const Matrix3Xf &X,
                                 const Matrix3Xf &TY,
                                 const MatrixXf &G,
                                 const MatrixXf &W,
                                 const double sigma2,
                                 const VectorXf Y_emit_prior,
                                 const MatrixXf &P)
{
  int M = TY.cols();
  int N = X.cols();

  double reg = 0.0;
  Matrix3Xf Y = TY + (G * W).transpose();
  for (int i = 0; i < M; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      reg += P(i, j) * ((X.col(j) - Y.col(i))).squaredNorm();
    }
  }
  reg = reg / (2 * sigma2);
  reg = reg + P.sum() * 3.0 * log(sigma2) / 2.0;
  return reg;
}

static PointCloud::Ptr mat_to_cloud(const Eigen::Matrix3Xf &mat)
{
  PointCloud::Ptr cloud(new PointCloud);
  cloud->points.reserve(mat.cols());
  for (ssize_t i = 0; i < mat.cols(); ++i)
  {
    cloud->push_back(pcl::PointXYZ(mat(0, i), mat(1, i), mat(2, i)));
  }
  return cloud;
}

static double initial_sigma2(const MatrixXf &X, const MatrixXf &Y)
{
  // X: (3, N) matrix, X^t in Algorithm 1
  // Y: (3, M) matrix, Y^(t-1) in Algorithm 1
  // Implement Line 2 of Algorithm 1
  double total_error = 0.0;
  assert(X.rows() == Y.rows());
  for (int i = 0; i < X.cols(); ++i)
  {
    for (int j = 0; j < Y.cols(); ++j)
    {
      total_error += (X.col(i) - Y.col(j)).squaredNorm();
    }
  }
  return total_error / (X.cols() * Y.cols() * X.rows());
}

static MatrixXf gaussian_kernel(const MatrixXf &Y, double beta)
{
  // Y: (3, M) matrix, corresponding to Y^(t-1) in Eq. 13.5 (Y^t in VI.A)
  // beta: beta in Eq. 13.5 (between 13 and 14)
  MatrixXf diff(Y.cols(), Y.cols());
  diff.setZero();
  for (int i = 0; i < Y.cols(); ++i)
  {
    for (int j = 0; j < Y.cols(); ++j)
    {
      diff(i, j) = (Y.col(i) - Y.col(j)).squaredNorm();
    }
  }
  // ???: beta should be beta^2
  MatrixXf kernel = (-diff / (2 * beta * beta)).array().exp();
  return kernel;
}

static std::vector<smmap::CollisionData> fake_collision_check(const smmap::AllGrippersSinglePose &)
{
  return {};
}

MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
                                     int lle_neighbors,
                                     double reg)
{
  // calculate L in Eq. (15) and Eq. (16)
  // ENHANCE: use tapkee lib to accelarate
  // kdtree: kdtree from Y^0
  // lle_neighbors: parameter for lle calculation
  // reg: regularization term (necessary when lle_neighbor > dimension)
  PointCloud::ConstPtr cloud = kdtree.getInputCloud();
  assert(cloud->height == 1);
  // adjacencies: save index of adjacent points
  MatrixXi adjacencies = MatrixXi(cloud->width, lle_neighbors);
  // B: save weight W_ij
  MatrixXf B = MatrixXf::Zero(cloud->width, lle_neighbors);
  MatrixXf v = VectorXf::Ones(lle_neighbors);
  // algorithm: see https://cs.nyu.edu/~roweis/lle/algorithm.html
  for (size_t i = 0; i < cloud->width; ++i)
  {
    std::vector<int> neighbor_inds(lle_neighbors + 1);
    std::vector<float> neighbor_dists(lle_neighbors + 1);
    kdtree.nearestKSearch(i, lle_neighbors + 1, neighbor_inds, neighbor_dists);
    // C: transpose of Z in Eq [d] and [e]
    MatrixXf C(lle_neighbors, 3);
    for (size_t j = 1; j < neighbor_inds.size(); ++j)
    {
      C.row(j - 1) = cloud->points[neighbor_inds[j]].getVector3fMap()
                     - cloud->points[i].getVector3fMap();
      adjacencies(i, j - 1) = neighbor_inds[j];
    }
    // G: C in Eq [f]
    MatrixXf G = C * C.transpose();
    // ???: why += R for G
    auto R = reg;
    auto const tr = G.trace();
    if (tr > 0)
    {
      R *= tr;
    }
    G.diagonal().array() += R;
    VectorXf w = G.llt().solve(v);
    B.row(i) = w / w.sum();
  }
  MatrixXf graph = MatrixXf::Zero(cloud->width, cloud->width);
  for (ssize_t i = 0; i < graph.rows(); ++i)
  {
    for (ssize_t j = 0; j < lle_neighbors; ++j)
    {
      graph(i, adjacencies(i, j)) = B(i, j);
    }
  }
  return graph;
}

MatrixXf locally_linear_embedding(PointCloud::ConstPtr template_cloud,
                                  int lle_neighbors,
                                  double reg)
{
  // calculate H in Eq. (18)
  // template_cloud: Y^0 in Eq. (15) and (16)
  // lle_neighbors: parameter for lle calculation
  // reg: regularization term (seems unnecessary)
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(template_cloud);
  // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
  MatrixXf W = barycenter_kneighbors_graph(kdtree, lle_neighbors, reg);
  // M: (M, M) matrix, corresponding to H in Eq. (18)
  MatrixXf M = (W.transpose() * W) - W.transpose() - W;
  M.diagonal().array() += 1;
  return M;
}

/*
 * Return a non-normalized probability that each of the tracked vertices produced any detected point.
 * Implement Eq. (7) in the paper
 */
VectorXf CDCPD::visibility_prior(const Matrix3Xf &vertices,
                                 const Mat &depth,
                                 const Mat &mask,
                                 const Matrix3f &intrinsics,
                                 const float kvis)
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
  for (int i = 0; i < image_space_vertices.cols(); ++i)
  {
    float x = image_space_vertices(0, i);
    float y = image_space_vertices(1, i);
    image_space_vertices(0, i) = std::min(std::max(x, 0.0f), static_cast<float>(depth.cols));
    image_space_vertices(1, i) = std::min(std::max(y, 0.0f), static_cast<float>(depth.rows));
  }

  // Get image coordinates
  Eigen::Matrix2Xi coords = image_space_vertices.topRows(2).cast<int>();

  for (int i = 0; i < coords.cols(); ++i)
  {
    coords(0, i) = std::min(std::max(coords(0, i), 0), depth.cols - 1);
    coords(1, i) = std::min(std::max(coords(1, i), 1), depth.rows - 1);
  }

  // Find difference between point depth and image depth
  // depth_diffs: (1, M) vector
  Eigen::VectorXf depth_diffs = Eigen::VectorXf::Zero(vertices.cols());
  for (int i = 0; i < vertices.cols(); ++i)
  {
    // std::cout << "depth at " << coords(1, i) << " " << coords(0, i) << std::endl;
    uint16_t raw_depth = depth.at<uint16_t>(coords(1, i), coords(0, i));
    if (raw_depth != 0)
    {
      depth_diffs(i) = vertices(2, i) - static_cast<float>(raw_depth) / 1000.0;
    } else
    {
      depth_diffs(i) = 0.02; // prevent numerical problems; taken from the Python code
    }
  }
  // ENHANCE: repeating max
  depth_diffs = depth_diffs.array().max(0);

  Eigen::VectorXf depth_factor = depth_diffs.array().max(0.0);
  cv::Mat dist_img(depth.rows, depth.cols, CV_32F); // TODO haven't really tested this but seems right
  int maskSize = 5;
  cv::distanceTransform(~mask, dist_img, cv::noArray(), cv::DIST_L2, maskSize);
  // ???: why cv::normalize is needed
  cv::normalize(dist_img, dist_img, 0.0, 1.0, cv::NORM_MINMAX);

  Eigen::VectorXf dist_to_mask(vertices.cols());
  for (int i = 0; i < vertices.cols(); ++i)
  {
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
static std::tuple<PointCloudRGB::Ptr, PointCloud::Ptr>
point_clouds_from_images(const cv::Mat &depth_image,
                         const cv::Mat &rgb_image,
                         const cv::Mat &mask,
                         const Eigen::Matrix3f &intrinsics,
                         const Eigen::Vector3f &lower_bounding_box,
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

  // ENHANCE: change the way to get access to depth image and rgb image to be more readable
  // from "depth_row += row_step" to ".at<>()"
  /*
  const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_image.data[0]);
  int row_step = depth_image.step / sizeof(uint16_t); // related with accessing data in cv::Mat
  const uint8_t* rgb = &rgb_image.data[0];
  int rgb_step = 3; // TODO check on this
  int rgb_skip = rgb_image.step - rgb_image.cols * rgb_step;
  */

  PointCloud::Ptr filtered_cloud(new PointCloud);
  PointCloudRGB::Ptr unfiltered_cloud(new PointCloudRGB(depth_image.cols, depth_image.rows));
  auto unfiltered_iter = unfiltered_cloud->begin();

  for (int v = 0; v < depth_image.rows; ++v)
  {
    for (int u = 0; u < depth_image.cols; ++u)
    {
      float depth = local_depth_image.at<float>(v, u);

      // Assume depth = 0 is the standard was to note invalid
      if (std::isfinite(depth))
      {
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
        if (mask.at<bool>(v, u) &&
            point.min(upper_bounding_box.array()).isApprox(point) &&
            point.max(lower_bounding_box.array()).isApprox(point))
        {
          filtered_cloud->push_back(pcl::PointXYZ(x, y, z));
        }
      } else
      {
        unfiltered_iter->x = unfiltered_iter->y = unfiltered_iter->z = bad_point;
      }
      ++unfiltered_iter;
    }
  }

  assert(unfiltered_iter == unfiltered_cloud->end());
  return {unfiltered_cloud, filtered_cloud};
}

Matrix3Xf CDCPD::cpd(const Matrix3Xf &X,
                     const Matrix3Xf &Y,
                     const Matrix3Xf &Y_pred,
                     const cv::Mat &depth,
                     const cv::Mat &mask,
                     const Eigen::Matrix3f &intr)
{
  // downsampled_cloud: PointXYZ pointer to downsampled point clouds
  // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
  // depth: CV_16U depth image
  // mask: CV_8U mask for segmentation label

  Eigen::VectorXf Y_emit_prior = visibility_prior(Y, depth, mask, intr, kvis);

  /// CPD step

  // G: (M, M) Guassian kernel matrix
  MatrixXf G = gaussian_kernel(original_template, beta);//Y, beta);

  // TY: Y^(t) in Algorithm 1
  Matrix3Xf TY = Y;
  double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale;

  int iterations = 0;
  double error = tolerance + 1; // loop runs the first time

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

#ifdef CPDLOG
  std::cout << "\nCPD loop\n";
  std::cout << std::setw(20) << "loop" << std::setw(20) << "prob term" << std::setw(20) << "CPD term" << std::setw(20) << "LLE term" << std::endl;
#endif


  while (iterations <= max_iterations && error > tolerance)
  {
    double qprev = sigma2;
    // Expectation step
    int N = X.cols();
    int M = Y.cols();
    int D = Y.rows();

    // P: P in Line 5 in Algorithm 1 (mentioned after Eq. (18))
    // Calculate Eq. (9) (Line 5 in Algorithm 1)
    // NOTE: Eq. (9) misses M in the denominator

    MatrixXf P(M, N);
    {
      for (int i = 0; i < M; ++i)
      {
        for (int j = 0; j < N; ++j)
        {
          P(i, j) = (X.col(j) - TY.col(i)).squaredNorm();
        }
      }

      float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
      c *= w / (1 - w);
      c *= static_cast<double>(M) / N;

      P = (-P / (2 * sigma2)).array().exp().matrix();
      P.array().colwise() *= Y_emit_prior.array();

      RowVectorXf den = P.colwise().sum();
      den.array() += c;

      P = P.array().rowwise() / den.array();
    }

    // Fast Gaussian Transformation to calculate Pt1, P1, PX
    MatrixXf PX = (P * X.transpose()).transpose();

    // // Maximization step
    VectorXf Pt1 = P.colwise().sum();
    VectorXf P1 = P.rowwise().sum();
    float Np = P1.sum();

    // NOTE: lambda means gamma here
    // Corresponding to Eq. (18) in the paper
    auto const lambda = start_lambda;
    MatrixXf p1d = P1.asDiagonal();

    auto const current_zeta = ROSHelpers::GetParamDebugLog<float>(ph, "zeta", 10.0);
    MatrixXf A = (P1.asDiagonal() * G)
                 + alpha * sigma2 * MatrixXf::Identity(M, M)
                 + sigma2 * lambda * (m_lle * G)
                 + current_zeta * G;

    MatrixXf B = PX.transpose() - (p1d + sigma2 * lambda * m_lle) * Y.transpose()
                 + zeta * (Y_pred.transpose() - Y.transpose());

    MatrixXf W = (A).householderQr().solve(B);

    TY = Y + (G * W).transpose();

    // Corresponding to Eq. (19) in the paper
    VectorXf xPxtemp = (X.array() * X.array()).colwise().sum();
    double xPx = Pt1.dot(xPxtemp);
    VectorXf yPytemp = (TY.array() * TY.array()).colwise().sum();
    double yPy = P1.dot(yPytemp);
    double trPXY = (TY.array() * PX.array()).sum();
    sigma2 = (xPx - 2 * trPXY + yPy) / (Np * static_cast<double>(D));

    if (sigma2 <= 0)
    {
      sigma2 = tolerance / 10;
    }

#ifdef CPDLOG
    double prob_reg = calculate_prob_reg(X, TY, G, W, sigma2, Y_emit_prior, P);
    double lle_reg = start_lambda / 2 * ((TY*m_lle*TY.transpose()).trace() + 2*(W.transpose()*G*m_lle*TY).trace() + (W.transpose()*G*m_lle*G*W).trace());
    double cpd_reg = alpha * (W.transpose()*G*W).trace()/2;
    std::cout << std::setw(20) << iterations;
    std::cout << std::setw(20) << prob_reg;
    std::cout << std::setw(20) << cpd_reg;
    std::cout << std::setw(20) << lle_reg << std::endl;
#endif


    error = std::abs(sigma2 - qprev);
    iterations++;
  }
  return TY;
}

Matrix3Xd CDCPD::predict(const Matrix3Xd &P,
                         const smmap::AllGrippersSinglePoseDelta &q_dot,
                         const smmap::AllGrippersSinglePose &q_config,
                         const int pred_choice)
{
  // P: template
  // q_dot: velocity of gripper, a 6*G matrix
  // q_config: indices of points gripped, a X*G matrix (X: depends on the case)
  // pred_choice:
  // 	- 0: no movement
  // 	- 1: prediction from ConstraintJacobianModel
  //  - 2: prediction from DiminishingRigidityModel

  // NOTE:
  //      - WorldState definition
  //      struct WorldState
  //      {
  //      public:
  //          ObjectPointSet object_configuration_;
  //          EigenHelpers::VectorIsometry3d rope_node_transforms_;
  //          AllGrippersSinglePose all_grippers_single_pose_;
  //          Eigen::VectorXd robot_configuration_;
  //          bool robot_configuration_valid_;
  //          std::vector<CollisionData> gripper_collision_data_;
  //          double sim_time_;

  //          uint64_t serializeSelf(std::vector<uint8_t>& buffer) const;

  //          static uint64_t Serialize(const WorldState& state, std::vector<uint8_t>& buffer);

  //          static std::pair<WorldState, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current);

  //          bool operator==(const WorldState& other) const;
  //      };
  //      - Needed attribute:
  //          - object_configuration_
  //          - all_grippers_single_pose_
  //      - AllGrippersSinglePoseDelta: kinematics::VectorVector6d (std::vector of eigen's 6D vector)
  //      - ObjectPointSet: Eigen::Matrix3Xd
  //      - AllGrippersSinglePose: EigenHelpers::VectorIsometry3d (std::vector of Isometry3d)
  if (pred_choice == 0)
  {
    return P;
  } else
  {
    smmap::WorldState world;
    world.object_configuration_ = P;
    world.all_grippers_single_pose_ = q_config;
    smmap::AllGrippersSinglePoseDelta grippers_pose_delta = q_dot;
    // cout << "gripper configuration:" << endl;
    // for (int i = 0; i < q_config.size(); ++i)
    // {
    // cout << q_config[i].rotation() << endl << endl;
    // cout << q_config[i].translation() << endl << endl;
    // }
    // cout << "gripper velocity" << endl;
    // for (int i = 0; i < q_dot.size(); ++i)
    // {
    // cout << q_dot[i] << endl << endl;
    // }
    if (pred_choice == 1)
    {
      return constraint_jacobian_model->getObjectDelta(world, grippers_pose_delta) + P;
    } else
    {
      return diminishing_rigidity_model->getObjectDelta(world, grippers_pose_delta) + P;
    }
  }
}

// This is for the case where the gripper indices are unknown (in real experiment)
CDCPD::CDCPD(PointCloud::ConstPtr template_cloud, // this needs a different data-type for python
             const Matrix2Xi &template_edges,
             const bool use_recovery,
             const double alpha,
             const double beta,
             const double lambda,
             const double k,
             const float zeta,
             const float obstacle_cost_weight) :
    CDCPD(ros::NodeHandle(),
          ros::NodeHandle("~"),
          template_cloud,
          template_edges,
          use_recovery,
          alpha,
          beta,
          lambda,
          k,
          zeta,
          obstacle_cost_weight)
{}

CDCPD::CDCPD(ros::NodeHandle nh,
             ros::NodeHandle ph,
             PointCloud::ConstPtr template_cloud,
             const Matrix2Xi &_template_edges,
             const bool use_recovery,
             const double alpha,
             const double beta,
             const double lambda,
             const double k,
             const float zeta,
             const float obstacle_cost_weight) :
    nh(nh),
    ph(ph),
    original_template(template_cloud->getMatrixXfMap().topRows(3)),
    template_edges(_template_edges),
    last_lower_bounding_box(original_template.rowwise().minCoeff()), // TODO make configurable?
    last_upper_bounding_box(original_template.rowwise().maxCoeff()), // TODO make configurable?
    lle_neighbors(8), // TODO make configurable?
    m_lle(locally_linear_embedding(template_cloud, lle_neighbors, 1e-3)), // TODO make configurable?
    tolerance(1e-4), // TODO make configurable?
    alpha(alpha), // TODO make configurable?
    beta(beta), // TODO make configurable?
    w(0.1), // TODO make configurable?
    initial_sigma_scale(1.0 / 8), // TODO make configurable?
    start_lambda(lambda),
    k(k),
    max_iterations(100), // TODO make configurable?
    kvis(1e3),
    zeta(zeta),
    obstacle_cost_weight(obstacle_cost_weight),
    use_recovery(use_recovery),
    last_grasp_status({false, false})
{
  last_lower_bounding_box = last_lower_bounding_box - bounding_box_extend;
  last_upper_bounding_box = last_upper_bounding_box + bounding_box_extend;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(template_cloud);
  // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
  L_lle = barycenter_kneighbors_graph(kdtree, lle_neighbors, 0.001);
  // Qconstructor(template_edges, Q, original_template.cols());

  // TODO: how to configure nh so that the it can get correct sdf
  const double res = 1.0;
  const double size = 10.0;
  const Eigen::Isometry3d origin_transform
      = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond(
          Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));
//  auto map = sdf_tools::CollisionMapGrid(origin_transform, "world", res, size, size, 1.0,
//                                         sdf_tools::COLLISION_CELL(0.0));
//  const auto sdf = map.ExtractSignedDistanceField(1e6, true, false).first;
  sdf_ptr = std::make_shared<const sdf_tools::SignedDistanceField>(origin_transform, "world", res, size, size, 1.0,
                                                                   1e6);


  auto const translation_dir_deformability = 0.0;
  auto const translation_dis_deformability = 0.0;
  auto const rotation_deformability = 0.0;
  // FIXME: these models have horrible static nonsense
//  constraint_jacobian_model = std::make_unique<smmap::ConstraintJacobianModel>(std::make_shared<ros::NodeHandle>(nh),
//                                                                               translation_dir_deformability,
//                                                                               translation_dis_deformability,
//                                                                               rotation_deformability,
//                                                                               sdf_ptr);
//
//  diminishing_rigidity_model = std::make_unique<smmap::DiminishingRigidityModel>(std::make_shared<ros::NodeHandle>(nh),
//                                                                                 translation_dir_deformability,
//                                                                                 rotation_deformability);
}

CDCPD::Output CDCPD::operator()(const Mat &rgb,
                                const Mat &depth,
                                const Mat &mask,
                                const cv::Matx33d &intrinsics,
                                const PointCloud::Ptr template_cloud, ObstacleConstraints obstacle_constraints,
                                const double max_segment_length,
                                const smmap::AllGrippersSinglePoseDelta &q_dot,
                                const smmap::AllGrippersSinglePose &q_config,
                                const std::vector<bool> &is_grasped,
                                const int pred_choice)
{
  std::vector<int> idx_map;
  for (auto const &[j, is_grasped_j] : enumerate(is_grasped))
  {
    if (j < q_config.size() and j < q_dot.size())
    {
      idx_map.push_back(j);
    } else
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME,
                             "is_grasped index " << j << " given but only " << q_config.size()
                                                 << " gripper configs and " << q_dot.size()
                                                 << " gripper velocities given.");
    }
  }

  // associate each gripper with the closest point in the current estimate
  if (is_grasped != last_grasp_status)
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "grasp status changed, recomputing correspondences");

    // get the previous tracking result
    const Matrix3Xf Y = template_cloud->getMatrixXfMap().topRows(3);

    auto const num_gripper = idx_map.size();
    MatrixXi grippers(1, num_gripper);
    for (auto g_idx = 0u; g_idx < num_gripper; g_idx++)
    {
      Vector3f gripper_pos = q_config[idx_map[g_idx]].matrix().cast<float>().block<3, 1>(0, 3);
      MatrixXf dist = (Y.colwise() - gripper_pos).colwise().norm();
      MatrixXf::Index minRow, minCol;
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "closest point index: " << minCol);
      grippers(0, g_idx) = int(minCol);
    }

    gripper_idx = grippers;

    {
      std::vector<smmap::GripperData> grippers_data;

      // format grippers_data
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "gripper data when constructing CDCPD:" << grippers);
      for (int g_idx = 0; g_idx < grippers.cols(); g_idx++)
      {
        std::vector<long> grip_node_idx;
        for (int node_idx = 0; node_idx < grippers.rows(); node_idx++)
        {
          grip_node_idx.push_back(long(grippers(node_idx, g_idx)));
          ROS_DEBUG_STREAM_NAMED(LOGNAME, "grasp point: " << grippers(node_idx, g_idx));
        }
        std::string gripper_name;
        gripper_name = "gripper" + std::to_string(g_idx);
        smmap::GripperData gripper(gripper_name, grip_node_idx);
        grippers_data.push_back(gripper);
      }

      //      constraint_jacobian_model->SetGrippersData(grippers_data);
      //
      //      // set up collision check function
      //      constraint_jacobian_model->SetCallbackFunctions(fake_collision_check);
      //
      //      // set initial point configuration
      //      Matrix3Xf eigen_template_cloud = template_cloud->getMatrixXfMap().topRows(3);
      //      constraint_jacobian_model->SetInitialObjectConfiguration(eigen_template_cloud.cast<double>());
      //      diminishing_rigidity_model->SetInitialObjectConfiguration(eigen_template_cloud.cast<double>());
    }
  }

  auto const cdcpd_out = operator()(rgb, depth, mask, intrinsics, template_cloud, obstacle_constraints,
                                    max_segment_length, q_dot,
                                    q_config, pred_choice);

  last_grasp_status = is_grasped;

  return cdcpd_out;

}

CDCPD::Output CDCPD::operator()(const Mat &rgb,
                                const Mat &depth,
                                const Mat &mask,
                                const cv::Matx33d &intrinsics,
                                const PointCloud::Ptr template_cloud, ObstacleConstraints obstacle_constraints,
                                const double max_segment_length,
                                const smmap::AllGrippersSinglePoseDelta &q_dot,
                                const smmap::AllGrippersSinglePose &q_config,
                                const Eigen::MatrixXi &gripper_idx,
                                const int pred_choice)
{
  this->gripper_idx = gripper_idx;
  auto const cdcpd_out = operator()(rgb, depth, mask, intrinsics, template_cloud, obstacle_constraints,
                                    max_segment_length, q_dot,
                                    q_config, pred_choice);
  return cdcpd_out;

}

CDCPD::Output CDCPD::operator()(const Mat &rgb,
                                const Mat &depth,
                                const Mat &mask,
                                const cv::Matx33d &intrinsics,
                                const PointCloud::Ptr template_cloud, ObstacleConstraints obstacle_constraints,
                                const double max_segment_length,
                                const smmap::AllGrippersSinglePoseDelta &q_dot,
                                const smmap::AllGrippersSinglePose &q_config,
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

  Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");

  // Useful utility for outputting an Eigen matrix to a file
  auto to_file = [&np_fmt](const std::string &fname, const MatrixXf &mat)
  {
    std::ofstream(fname, std::ofstream::app) << mat.format(np_fmt) << "\n\n";
  };

  Eigen::Matrix3d intrinsics_eigen_tmp;
  cv::cv2eigen(intrinsics, intrinsics_eigen_tmp);
  Eigen::Matrix3f intrinsics_eigen = intrinsics_eigen_tmp.cast<float>();

  // entire_cloud: pointer to the entire point cloud
  // cloud: pointer to the point clouds selected
  auto[entire_cloud, cloud] = point_clouds_from_images(depth,
                                                       rgb,
                                                       mask,
                                                       intrinsics_eigen,
                                                       last_lower_bounding_box - bounding_box_extend,
                                                       last_upper_bounding_box + bounding_box_extend);
  ROS_INFO_STREAM_THROTTLE_NAMED(1, LOGNAME, "Points in filtered: (" << cloud->height << " x " << cloud->width << ")");

  /// VoxelGrid filter downsampling
  PointCloud::Ptr cloud_downsampled(new PointCloud);
  const Matrix3Xf Y = template_cloud->getMatrixXfMap().topRows(3);
  // TODO: check whether the change is needed here for unit conversion
  Eigen::VectorXf Y_emit_prior = visibility_prior(Y, depth, mask, intrinsics_eigen, kvis);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "Points in cloud before leaf: " << cloud->width);
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.02f, 0.02f, 0.02f);
  sor.filter(*cloud_downsampled);
  ROS_INFO_STREAM_THROTTLE_NAMED(1, LOGNAME, "Points in fully filtered: " << cloud_downsampled->width);
  if (cloud_downsampled->width == 0)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "No point in the filtered point cloud");
    PointCloud::Ptr cdcpd_out = mat_to_cloud(Y);
    PointCloud::Ptr cdcpd_cpd = mat_to_cloud(Y);
    PointCloud::Ptr cdcpd_pred = mat_to_cloud(Y);

    return CDCPD::Output{
        entire_cloud,
        cloud,
        cloud_downsampled,
        cdcpd_cpd,
        cdcpd_pred,
        cdcpd_out
    };
  }
  Matrix3Xf X = cloud_downsampled->getMatrixXfMap().topRows(3);
  // Add points to X according to the previous template

  const Matrix3Xf &entire = entire_cloud->getMatrixXfMap().topRows(3);

  std::vector<FixedPoint> pred_fixed_points;
  auto const num_grippers = std::min(static_cast<size_t>(gripper_idx.cols()), static_cast<size_t>(q_config.size()));
  for (auto col = 0u; col < num_grippers; ++col)
  {
    FixedPoint pt;
    pt.template_index = gripper_idx(0, col);
    pt.position(0) = q_config[col](0, 3);
    pt.position(1) = q_config[col](1, 3);
    pt.position(2) = q_config[col](2, 3);
    pred_fixed_points.push_back(pt);
  }

  Matrix3Xf TY, TY_pred;
  TY_pred = predict(Y.cast<double>(), q_dot, q_config, pred_choice).cast<float>();
  TY = cpd(X, Y, TY_pred, depth, mask, intrinsics_eigen);

  // Next step: optimization.

  ROS_DEBUG_STREAM_NAMED(LOGNAME, "fixed points" << pred_fixed_points);

  // NOTE: seems like this should be a function, not a class? is there state like the gurobi env?
  // ???: most likely not 1.0
  Optimizer opt(original_template, Y, 1.1, obstacle_cost_weight);
  Matrix3Xf Y_opt = opt(TY, template_edges, pred_fixed_points, obstacle_constraints, max_segment_length);

  // NOTE: set stateful member variables for next time
  last_lower_bounding_box = Y_opt.rowwise().minCoeff();
  last_upper_bounding_box = Y_opt.rowwise().maxCoeff();

  PointCloud::Ptr cdcpd_out = mat_to_cloud(Y_opt);
  PointCloud::Ptr cdcpd_cpd = mat_to_cloud(TY);
  PointCloud::Ptr cdcpd_pred = mat_to_cloud(TY_pred);

  return CDCPD::Output{
      entire_cloud,
      cloud,
      cloud_downsampled,
      cdcpd_cpd,
      cdcpd_pred,
      cdcpd_out
  };
}
