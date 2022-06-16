#include <arc_utilities/enumerate.h>
#include <cdcpd/cdcpd.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include "cdcpd_ros/kinect_sub.h"

#include <iostream>
#include <fstream>
#include <string>

constexpr auto const LOGNAME = "cdcpd_node";
constexpr auto const PERF_LOGGER = "perf";

Eigen::Vector3f extent_to_env_size(Eigen::Vector3f const& bbox_lower, Eigen::Vector3f const& bbox_upper) {
  return (bbox_upper - bbox_lower).cwiseAbs() + 2 * bounding_box_extend;
};

Eigen::Vector3f extent_to_center(Eigen::Vector3f const& bbox_lower, Eigen::Vector3f const& bbox_upper) {
  return (bbox_upper + bbox_lower) / 2;
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace gm = geometry_msgs;
namespace vm = visualization_msgs;
namespace ehc = EigenHelpersConversions;

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int num_points, float length);

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int num_points, const Eigen::Vector3f& start_position,
                                                               const Eigen::Vector3f& end_position);

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int const num_points, float const length) {
  Eigen::Vector3f start_position(-length / 2, 0, 1.0);
  Eigen::Vector3f end_position(length / 2, 0, 1.0);
  return makeRopeTemplate(num_points, start_position, end_position);
}

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int const num_points,
                                                               const Eigen::Vector3f& start_position,
                                                               const Eigen::Vector3f& end_position) {
  Eigen::Matrix3Xf template_vertices(3, num_points);  // Y^0 in the paper
  Eigen::VectorXf thetas = Eigen::VectorXf::LinSpaced(num_points, 0, 1);
  for (auto i = 0u; i < num_points; ++i) {
    auto const theta = thetas.row(i);
    template_vertices.col(i) = (end_position - start_position) * theta + start_position;
  }
  Eigen::Matrix2Xi template_edges(2, num_points - 1);
  template_edges(0, 0) = 0;
  template_edges(1, template_edges.cols() - 1) = num_points - 1;
  for (int i = 1; i <= template_edges.cols() - 1; ++i) {
    template_edges(0, i) = i;
    template_edges(1, i - 1) = i;
  }
  return {template_vertices, template_edges};
}

PointCloud::Ptr makeCloud(Eigen::Matrix3Xf const& points) {
  // TODO: Can we do this cleaner via some sort of data mapping?
  PointCloud::Ptr cloud(new PointCloud);
  for (int i = 0; i < points.cols(); ++i) {
    auto const& c = points.col(i);
    cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
  }
  return cloud;
}

cv::Mat getHsvMask(ros::NodeHandle const& ph, cv::Mat const& rgb) {
  auto const hue_min = ROSHelpers::GetParamDebugLog<double>(ph, "hue_min", 340.0);
  auto const sat_min = ROSHelpers::GetParamDebugLog<double>(ph, "saturation_min", 0.4);
  auto const val_min = ROSHelpers::GetParamDebugLog<double>(ph, "value_min", 0.4);
  auto const hue_max = ROSHelpers::GetParamDebugLog<double>(ph, "hue_max", 20.0);
  auto const sat_max = ROSHelpers::GetParamDebugLog<double>(ph, "saturation_max", 1.0);
  auto const val_max = ROSHelpers::GetParamDebugLog<double>(ph, "value_max", 1.0);

  cv::Mat rgb_f;
  rgb.convertTo(rgb_f, CV_32FC3);
  rgb_f /= 255.0;  // get RGB 0.0-1.0
  cv::Mat color_hsv;
  cvtColor(rgb_f, color_hsv, CV_RGB2HSV);

  cv::Mat mask1;
  cv::Mat mask2;
  cv::Mat hsv_mask;
  auto hue_min1 = hue_min;
  auto hue_max2 = hue_max;
  if (hue_min > hue_max) {
    hue_max2 = 360;
    hue_min1 = 0;
  }
  cv::inRange(color_hsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max2, sat_max, val_max), mask1);
  cv::inRange(color_hsv, cv::Scalar(hue_min1, sat_min, val_min), cv::Scalar(hue_max, sat_max, val_max), mask2);
  bitwise_or(mask1, mask2, hsv_mask);

  return hsv_mask;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cdcpd_node");

  // user-specified variable
  std::string data_dir = "/home/yixuan/blender_data/rope_simple/";
  std::string frame_id = "kinect2_rgb_optical_frame";
  int num_points = 11;
  double rope_length = 1.0;
  int frame_num = 251;
  Eigen::Vector3f const start_position(-1.0, 0.0, 3.0);
  Eigen::Vector3f const end_position(0.0, 0.0, 3.0);
  bool use_recovery = false;
  double alpha = 0.5;
  double beta = 1.0;
  double lambda = 1.0;
  double k_spring = 100.0;
  double zeta = 10.0;
  double obstacle_cost_weight = 0.001;

  // initialization for CDCPD
  ros::NodeHandle nh;
  ros::NodeHandle ph;
  auto const [template_vertices, template_edges] = makeRopeTemplate(num_points, start_position, end_position);
  // Construct the initial template as a PCL cloud
  auto tracked_points = makeCloud(template_vertices);
  auto cdcpd = CDCPD(nh, ph, tracked_points, template_edges, use_recovery, alpha, beta, lambda, k_spring, zeta,
                       obstacle_cost_weight);
  cv::Matx33d intrinsics;
  Eigen::MatrixXd intri_eigen = eigen_from_file(data_dir+"/cam_info.txt");
  cv::eigen2cv(intri_eigen, intrinsics);
  double max_segment_length = rope_length/static_cast<float>(num_points-1);
  ObstacleConstraints obstacle_constraints; // currently void

  // initialize publisher
  auto original_publisher = nh.advertise<PointCloud>("cdcpd/original", 10);
  auto masked_publisher = nh.advertise<PointCloud>("cdcpd/masked", 10);
  auto downsampled_publisher = nh.advertise<PointCloud>("cdcpd/downsampled", 10);
  auto template_publisher = nh.advertise<PointCloud>("cdcpd/template", 10);
  auto pre_template_publisher = nh.advertise<PointCloud>("cdcpd/pre_template", 10);
  auto output_publisher = nh.advertise<PointCloud>("cdcpd/output", 10);
  auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);

  for (int i = 0; i < frame_num; i++) {
    std::stringstream idx_ss;
    idx_ss << std::setw(3) << std::setfill('0') << i;
    std::string idx_str = idx_ss.str();
    cv::Mat rgb = cv::imread(data_dir+"render/rgb_"+idx_str+".png");
    cvtColor(rgb, rgb, CV_BGR2RGB);
    cv::Mat depth = cv::imread(data_dir+"render/depth_"+idx_str+".png", cv::IMREAD_ANYDEPTH);
    auto const hsv_mask = getHsvMask(ph, rgb);

    tracked_points->header.frame_id = frame_id;

    // gripper information
    // cuurent no motion model
    smmap::AllGrippersSinglePose q_config;
    auto const n_grippers = q_config.size();
    const smmap::AllGrippersSinglePoseDelta q_dot{n_grippers, kinematics::Vector6d::Zero()};
    Eigen::MatrixXi gripper_idx(1,2);
    gripper_idx << 0, 0;

    auto out = cdcpd(rgb, depth, hsv_mask, intrinsics, tracked_points, obstacle_constraints, max_segment_length,
                             q_dot, q_config, gripper_idx);
    tracked_points = out.gurobi_output;

    // publish
    // Update the frame ids
    {
      out.original_cloud->header.frame_id = frame_id;
      out.masked_point_cloud->header.frame_id = frame_id;
      out.downsampled_cloud->header.frame_id = frame_id;
      out.cpd_output->header.frame_id = frame_id;
      out.gurobi_output->header.frame_id = frame_id;
    }

    // Add timestamp information
    {
      auto time = ros::Time::now();
      pcl_conversions::toPCL(time, out.original_cloud->header.stamp);
      pcl_conversions::toPCL(time, out.masked_point_cloud->header.stamp);
      pcl_conversions::toPCL(time, out.downsampled_cloud->header.stamp);
      pcl_conversions::toPCL(time, out.cpd_output->header.stamp);
      pcl_conversions::toPCL(time, out.gurobi_output->header.stamp);
    }

    // Publish the point clouds
    {
      original_publisher.publish(out.original_cloud);
      masked_publisher.publish(out.masked_point_cloud);
      downsampled_publisher.publish(out.downsampled_cloud);
      template_publisher.publish(out.cpd_output);
      output_publisher.publish(out.gurobi_output);
    }
  }

  return EXIT_SUCCESS;
}
