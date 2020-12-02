#include <cdcpd/cdcpd.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/imgproc/types_c.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include "cdcpd_ros/kinect_sub.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace gm = geometry_msgs;
namespace vm = visualization_msgs;
namespace ehc = EigenHelpersConversions;

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int const num_points, float const length) {
  Eigen::Matrix3Xf template_vertices = Eigen::Matrix3Xf::Zero(3, num_points);  // Y^0 in the paper
  template_vertices.row(0).setLinSpaced(num_points, -length / 2, length / 2);
  template_vertices.row(2).array() += 1.4f;
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

cv::Mat ropeHsvMask(cv::Mat rgb) {
  cv::Mat rgb_f;
  rgb.convertTo(rgb_f, CV_32FC3);
  rgb_f /= 255.0;  // get RGB 0.0-1.0
  cv::Mat color_hsv;
  cvtColor(rgb_f, color_hsv, CV_RGB2HSV);

  cv::Mat mask1;
  cv::Mat mask2;
  cv::Mat hsv_mask;
  cv::inRange(color_hsv, cv::Scalar(0, 0.2, 0.2), cv::Scalar(20, 1.0, 1.0), mask1);
  cv::inRange(color_hsv, cv::Scalar(340, 0.2, 0.2), cv::Scalar(360, 1.0, 1.0), mask2);
  bitwise_or(mask1, mask2, hsv_mask);

  return hsv_mask;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cdcpd_node");
  auto nh = ros::NodeHandle();
  auto ph = ros::NodeHandle("~");

#ifndef ROPE
  static_assert("This node is only designed for rope right now");
#endif

  // Publsihers for the data, some visualizations, others consumed by other nodes
  auto original_publisher = nh.advertise<PointCloud>("cdcpd/original", 1);
  auto masked_publisher = nh.advertise<PointCloud>("cdcpd/masked", 1);
  auto downsampled_publisher = nh.advertise<PointCloud>("cdcpd/downsampled", 1);
  auto template_publisher = nh.advertise<PointCloud>("cdcpd/template", 1);
  auto output_publisher = nh.advertise<PointCloud>("cdcpd/output", 1);
  auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);

  // TF objects for getting gripper positions
  auto tf_buffer = tf2_ros::Buffer();
  auto tf_listener = tf2_ros::TransformListener(tf_buffer);

  // Initial connectivity model of rope
  auto const num_points = ROSHelpers::GetParam<int>(nh, "rope_num_points", 11);
  auto const length = ROSHelpers::GetParam<float>(nh, "rope_length", 1.0);
  auto const [template_vertices, template_edges] = makeRopeTemplate(num_points, length);
  // Construct the initial template as a PCL cloud
  auto template_cloud = makeCloud(template_vertices);

  // CDCPD parameters
  // ENHANCE: describe each parameter in words (and a pointer to an equation/section of paper)
  auto const alpha = ROSHelpers::GetParam<double>(ph, "alpha", 0.5);
  auto const lambda = ROSHelpers::GetParam<double>(ph, "lambda", 1.0);
  auto const k_spring = ROSHelpers::GetParam<double>(ph, "k", 100.0);
  auto const beta = ROSHelpers::GetParam<double>(ph, "beta", 1.0);
  auto const use_recovery = ROSHelpers::GetParam<bool>(ph, "use_recovery", false);
  auto const kinect_name = ROSHelpers::GetParam<std::string>(ph, "kinect_name", "kinect2");
  auto const kinect_channel = ROSHelpers::GetParam<std::string>(ph, "kinect_channel", "qhd");

  // For use with TF and "fixed points" for the constrain step
  auto const kinect_tf_name = kinect_name + "_rgb_optical_frame";
  auto const left_tf_name = ROSHelpers::GetParam<std::string>(ph, "left_tf_name", "cdcpd_ros/left_gripper_prior");
  auto const right_tf_name = ROSHelpers::GetParam<std::string>(ph, "right_tf_name", "cdcpd_ros/right_gripper_prior");
  auto const left_node_idx = ROSHelpers::GetParam<int>(ph, "left_node_idx", num_points - 1);
  auto const right_node_idx = ROSHelpers::GetParam<int>(ph, "right_node_idx", 1);

  auto cdcpd = CDCPD(template_cloud, template_edges, use_recovery, alpha, beta, lambda, k_spring);
  // TODO: Make these const references? Does this matter for CV types?
  auto const callback = [&](cv::Mat rgb, cv::Mat depth, cv::Matx33d cam) {
    std::vector<CDCPD::FixedPoint> fixed_points;
    // Left Gripper
    try {
      auto const gripper = tf_buffer.lookupTransform(kinect_tf_name, left_tf_name, ros::Time(0));
      auto const pos = ehc::GeometryVector3ToEigenVector3d(gripper.transform.translation);
      fixed_points.push_back({pos.cast<float>(), left_node_idx});

    } catch (tf2::TransformException const& ex) {
      ROS_WARN_STREAM_THROTTLE(
          10.0, "Unable to lookup transform from " << kinect_tf_name << " to " << left_tf_name << ": " << ex.what());
    }
    // Right Gripper
    try {
      auto const gripper = tf_buffer.lookupTransform(kinect_tf_name, right_tf_name, ros::Time(0));
      auto const pos = ehc::GeometryVector3ToEigenVector3d(gripper.transform.translation);
      fixed_points.push_back({pos.cast<float>(), right_node_idx});

    } catch (tf2::TransformException const& ex) {
      ROS_WARN_STREAM_THROTTLE(
          10.0, "Unable to lookup transform from " << kinect_tf_name << " to " << right_tf_name << ": " << ex.what());
    }

    // Perform and record the update
    auto hsv_mask = ropeHsvMask(rgb);
    auto out = cdcpd(rgb, depth, hsv_mask, cam, template_cloud, true, false, fixed_points);
    template_cloud = out.gurobi_output;

    // Update the frame ids
    {
#ifdef ENTIRE
      out.original_cloud->header.frame_id = kinect_tf_name;
#endif
      out.masked_point_cloud->header.frame_id = kinect_tf_name;
      out.downsampled_cloud->header.frame_id = kinect_tf_name;
      out.cpd_output->header.frame_id = kinect_tf_name;
      out.gurobi_output->header.frame_id = kinect_tf_name;
#ifdef COMP
      out_without_constrain.gurobi_output->header.frame_id = kinect_tf_name;
#endif
    }

    // Add timestamp information
    {
      auto time = ros::Time::now();
#ifdef ENTIRE
      pcl_conversions::toPCL(time, out.original_cloud->header.stamp);
#endif
      pcl_conversions::toPCL(time, out.masked_point_cloud->header.stamp);
      pcl_conversions::toPCL(time, out.downsampled_cloud->header.stamp);
      pcl_conversions::toPCL(time, out.cpd_output->header.stamp);
      pcl_conversions::toPCL(time, out.gurobi_output->header.stamp);
#ifdef COMP
      pcl_conversions::toPCL(time, out_without_constrain.gurobi_output->header.stamp);
#endif
    }

    // Publish the point clouds
    {
#ifdef ENTIRE
      original_publisher.publish(out.original_cloud);
#endif
      masked_publisher.publish(out.masked_point_cloud);
      downsampled_publisher.publish(out.downsampled_cloud);
      template_publisher.publish(out.cpd_output);
      output_publisher.publish(out.gurobi_output);
#ifdef COMP
      output_without_constrain_publisher.publish(out_without_constrain.gurobi_output);
#endif
    }

    // Publish markers indication the order of the points
    {
      auto rope_marker_fn = [&](PointCloud::ConstPtr cloud, std::string const& ns) {
        vm::Marker order;
        order.header.frame_id = kinect_tf_name;
        order.header.stamp = ros::Time();
        order.ns = ns;
        order.action = visualization_msgs::Marker::ADD;
        order.pose.orientation.w = 1.0;
        order.id = 1;
        order.scale.x = 0.002;
        order.color.r = 1.0;
        order.color.a = 1.0;

        for (auto pc_iter = cloud->begin(); pc_iter != cloud->end(); ++pc_iter) {
          geometry_msgs::Point p;
          p.x = pc_iter->x;
          p.y = pc_iter->y;
          p.z = pc_iter->z;
          order.points.push_back(p);
        }
        return order;
      };

      auto const rope_marker = rope_marker_fn(out.gurobi_output, "line_order");
      order_pub.publish(rope_marker);
    }
  };

  auto const options = KinectSub::SubscriptionOptions(kinect_name + "/" + kinect_channel);
  KinectSub sub(callback, options);

  ROS_INFO("Spinning...");
  ros::waitForShutdown();

  return EXIT_SUCCESS;
}
