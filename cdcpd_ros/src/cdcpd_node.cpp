#include <arc_utilities/enumerate.h>
#include <cdcpd/cdcpd.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <opencv2/imgproc/types_c.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include "cdcpd_ros/kinect_sub.h"

constexpr auto const LOGNAME = "cdcpd_node";
constexpr auto const COLLISION_BODY_NAME = "cdcpd_tracked_point";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace gm = geometry_msgs;
namespace vm = visualization_msgs;
namespace ehc = EigenHelpersConversions;

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int const num_points, float const length) {
  Eigen::Matrix3Xf template_vertices = Eigen::Matrix3Xf::Zero(3, num_points);  // Y^0 in the paper
  template_vertices.row(0).setLinSpaced(num_points, -length / 2, length / 2);
  template_vertices.row(2).array() += 1.0f;
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

Objects get_moveit_planning_scene_as_mesh(planning_scene_monitor::PlanningSceneMonitorPtr const& scene_monitor) {
  // get the latest scene
  planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor);

  Objects objects;
  planning_scene->getCollisionObjectMsgs(objects);
  return objects;
}

struct CDCPD_Moveit_Node {
  ros::NodeHandle nh;
  ros::NodeHandle ph;
  ros::Publisher contact_marker_pub;

  CDCPD_Moveit_Node() : ph("~") {
    std::string robot_namespace{"hdt_michigan"};
    auto scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    auto const scene_topic = ros::names::append(robot_namespace, "move_group/monitored_planning_scene");
    auto const service_name = ros::names::append(robot_namespace, "get_planning_scene");
    scene_monitor->startSceneMonitor(scene_topic);
    scene_monitor->requestPlanningSceneState(service_name);

    // Publsihers for the data, some visualizations, others consumed by other nodes
    auto original_publisher = nh.advertise<PointCloud>("cdcpd/original", 1);
    auto masked_publisher = nh.advertise<PointCloud>("cdcpd/masked", 1);
    auto downsampled_publisher = nh.advertise<PointCloud>("cdcpd/downsampled", 1);
    auto template_publisher = nh.advertise<PointCloud>("cdcpd/template", 1);
    auto output_publisher = nh.advertise<PointCloud>("cdcpd/output", 1);
    auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);
    contact_marker_pub = ph.advertise<vm::MarkerArray>("contacts", 10);

    // TF objects for getting gripper positions
    auto tf_buffer = tf2_ros::Buffer();
    auto tf_listener = tf2_ros::TransformListener(tf_buffer);

    // Initial connectivity model of rope
    auto const num_points = ROSHelpers::GetParam<int>(nh, "rope_num_points", 11);
    auto const length = ROSHelpers::GetParam<float>(nh, "rope_length", 1.0);
    auto const [template_vertices, template_edges] = makeRopeTemplate(num_points, length);
    // Construct the initial template as a PCL cloud
    auto tracked_points = makeCloud(template_vertices);

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
    auto const left_tf_name = ROSHelpers::GetParam<std::string>(ph, "left_tf_name", "");
    auto const right_tf_name = ROSHelpers::GetParam<std::string>(ph, "right_tf_name", "");
    auto const left_node_idx = ROSHelpers::GetParam<int>(ph, "left_node_idx", num_points - 1);
    auto const right_node_idx = ROSHelpers::GetParam<int>(ph, "right_node_idx", 1);

    auto cdcpd = CDCPD(nh, ph, tracked_points, template_edges, use_recovery, alpha, beta, lambda, k_spring);

    // TODO: Make these const references? Does this matter for CV types?
    auto const callback = [&](cv::Mat rgb, cv::Mat depth, cv::Matx33d intrinsics) {
      smmap::AllGrippersSinglePose q_config;
      // Left Gripper
      if (not left_tf_name.empty()) {
        try {
          auto const gripper = tf_buffer.lookupTransform(kinect_tf_name, left_tf_name, ros::Time(0));
          auto const config = ehc::GeometryTransformToEigenIsometry3d(gripper.transform);
          ROS_DEBUG_STREAM("left gripper: " << config.translation());
          q_config.push_back(config);

        } catch (tf2::TransformException const& ex) {
          ROS_WARN_STREAM_THROTTLE(10.0, "Unable to lookup transform from " << kinect_tf_name << " to " << left_tf_name
                                                                            << ": " << ex.what());
        }
      }
      // Right Gripper
      if (not right_tf_name.empty()) {
        try {
          auto const gripper = tf_buffer.lookupTransform(kinect_tf_name, right_tf_name, ros::Time(0));
          auto const config = ehc::GeometryTransformToEigenIsometry3d(gripper.transform);
          ROS_DEBUG_STREAM("right gripper: " << config.translation());
          q_config.push_back(config);

        } catch (tf2::TransformException const& ex) {
          ROS_WARN_STREAM_THROTTLE(10.0, "Unable to lookup transform from " << kinect_tf_name << " to " << right_tf_name
                                                                            << ": " << ex.what());
        }
      }

      // Perform and record the update
      auto const hsv_mask = getHsvMask(ph, rgb);
      auto const n_grippers = q_config.size();
      const smmap::AllGrippersSinglePoseDelta q_dot{n_grippers, kinematics::Vector6d::Zero()};

      //    auto const objects = get_moveit_planning_scene_as_mesh(scene_monitor);
      auto const points_normals = moveit_get_points_normals(scene_monitor, tf_buffer, kinect_tf_name, tracked_points);

      Eigen::MatrixXi gripper_idx(1, 2);
      gripper_idx << left_node_idx, right_node_idx;
      auto const out =
          cdcpd(rgb, depth, hsv_mask, intrinsics, tracked_points, points_normals, q_dot, q_config, gripper_idx);
      tracked_points = out.gurobi_output;

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

      // Publish markers indication the order of the points
      {
        auto rope_marker_fn = [&](PointCloud::ConstPtr cloud, std::string const& ns) {
          vm::Marker order;
          order.header.frame_id = kinect_tf_name;
          order.header.stamp = ros::Time();
          order.ns = ns;
          order.type = visualization_msgs::Marker::LINE_STRIP;
          order.action = visualization_msgs::Marker::ADD;
          order.pose.orientation.w = 1.0;
          order.id = 1;
          order.scale.x = 0.01;
          order.color.r = 1.0;
          order.color.a = 1.0;

          for (auto pc_iter : *cloud) {
            geometry_msgs::Point p;
            p.x = pc_iter.x;
            p.y = pc_iter.y;
            p.z = pc_iter.z;
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
  }

  std::optional<PointNormal> find_nearest_point_and_normal(planning_scene_monitor::LockedPlanningSceneRW planning_scene,
                                                           tf2_ros::Buffer const& tf_buffer, std::string kinect_tf_name,
                                                           pcl::PointXYZ const& point) {
    auto world = planning_scene->getWorldNonConst();
    auto sphere = std::make_shared<shapes::Sphere>(0.02);
    std::string moveit_frame{"robot_root"};

    try {
      auto const cdcpd_to_moveit = tf_buffer.lookupTransform(kinect_tf_name, moveit_frame, ros::Time(0));
      auto const cdcpd_to_moveit_transform = ehc::GeometryTransformToEigenIsometry3d(cdcpd_to_moveit.transform);

      auto const point_in_moveit_frame = cdcpd_to_moveit_transform * point.getVector3fMap().cast<double>();
      Eigen::Isometry3d pose_moveit_frame = Eigen::Isometry3d::Identity();
      pose_moveit_frame.translation() = point_in_moveit_frame;

      world->addToObject(COLLISION_BODY_NAME, sphere, pose_moveit_frame);

      collision_detection::CollisionRequest req;
      req.contacts = true;
      req.verbose = true;
      req.max_contacts_per_pair = 1;
      req.max_contacts = std::numeric_limits<typeof(collision_detection::CollisionRequest::max_contacts)>::max();
      collision_detection::CollisionResult res;
      planning_scene->checkCollisionUnpadded(req, res);

      PointsNormals points_normals;
      for (auto const& [contact_names, contacts] : res.contacts) {
        // FIXME: is this correct?
        if (contacts.empty()) {
          continue;
        }

        auto const contact = contacts[0];
        auto add_point_normal = [&](int body_idx) {
          // FIXME: the contact point in moveit frame seems to be wrong
          auto const contact_point_moveit_frame = contact.nearest_points[body_idx];
          auto const normal_moveit_frame = res.contacts.begin()->second.begin()->normal;
          auto const contact_point_cdcpd_frame = cdcpd_to_moveit_transform.inverse() * contact_point_moveit_frame;
          auto const normal_cdcpd_frame = cdcpd_to_moveit_transform.inverse() * normal_moveit_frame;
          points_normals.emplace_back(contact_point_cdcpd_frame.cast<float>(), normal_cdcpd_frame.cast<float>());

          // NOTE: debug & visualize
          {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "contact point in cdcpd frame: " << contact_point_cdcpd_frame);
            ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "normal in cdcpd frame: " << normal_cdcpd_frame);

            vm::MarkerArray markers;
            vm::Marker arrow;
            arrow.color.r = 1.0;
            arrow.color.g = 0.0;
            arrow.color.b = 1.0;
            arrow.color.a = 0.5;
            arrow.type = vm::Marker::ARROW;
            arrow.action = vm::Marker::ADD;
            arrow.header.frame_id = kinect_tf_name;
            arrow.header.stamp = ros::Time::now();
            arrow.scale.x = 0.01;
            arrow.scale.y = 0.02;
            arrow.scale.z = 0;
            arrow.pose.orientation.w = 1;
            Eigen::Vector3d arrow_end_point_cdcpd_frame = contact_point_cdcpd_frame + normal_cdcpd_frame;
            arrow.points.push_back(ConvertTo<geometry_msgs::Point>(contact_point_cdcpd_frame));
            arrow.points.push_back(ConvertTo<geometry_msgs::Point>(arrow_end_point_cdcpd_frame));

            markers.markers.push_back(arrow);
            /* Get the contact ponts and display them as markers */
            contact_marker_pub.publish(markers);
          }
        };

        if (contact_names.first == COLLISION_BODY_NAME) {
          add_point_normal(0);
        } else if (contact_names.second == COLLISION_BODY_NAME) {
          add_point_normal(1);
        } else {
          continue;
        }
      }

      if (points_normals.empty()) {
        return {};
      } else if (points_normals.size() != 1) {
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "Found multiple collisions for rope point at " << point);
      }
      return {points_normals[0]};
    } catch (tf2::TransformException const& ex) {
      ROS_WARN_STREAM_THROTTLE(
          10.0, "Unable to lookup transform from " << kinect_tf_name << " to " << moveit_frame << ": " << ex.what());
    }
    return {};
  }

  PointsNormals moveit_get_points_normals(planning_scene_monitor::PlanningSceneMonitorPtr const& scene_monitor,
                                          tf2_ros::Buffer const& tf_buffer, std::string kinect_tf_name,
                                          PointCloud::ConstPtr tracked_points) {
    // an alternative to manual + CGAL based nearest/normal, we could check check each point on the rope for collision
    // via moveit but moveit only knows how to check for collision betweeen the robot state and the world/itself so I'm
    // not sure how we'd do this

    planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor);

    PointsNormals points_normals;

    // add a point to the moveit world and collision check it
    for (auto const& point : *tracked_points) {
      auto const point_normal = find_nearest_point_and_normal(planning_scene, tf_buffer, kinect_tf_name, point);
      if (point_normal) {
        auto const& [contact_point, normal] = point_normal.value();
        points_normals.emplace_back(point_normal.value());
      }
    }

    return points_normals;
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cdcpd_node");

  CDCPD_Moveit_Node cmn;

  return EXIT_SUCCESS;
}
