#include <arc_utilities/enumerate.h>
#include <cdcpd/cdcpd.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/TransformStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
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

std::string const LOGNAME = "cdcpd_node";
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

void print_bodies(robot_state::RobotState const& state) {
  std::vector<robot_state::AttachedBody const*> bs;
  std::cout << "Attached Bodies:\n";
  state.getAttachedBodies(bs);
  for (auto const& b : bs) {
    std::cout << b->getName() << '\n';
  }
}

struct CDCPD_Moveit_Node {
  std::string collision_body_prefix{"cdcpd_tracked_point_"};
  std::string robot_namespace_;
  std::string robot_description_;
  ros::NodeHandle nh;
  ros::NodeHandle ph;
  ros::Publisher contact_marker_pub;
  ros::Publisher bbox_pub;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr model_loader_;
  robot_model::RobotModelPtr model_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  std::string moveit_frame{"robot_root"};
  std::string kinect_tf_name = "kinect2_rgb_optical_frame";
  double min_distance_threshold{0.01};
  bool moveit_ready{false};
  bool moveit_enabled{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  explicit CDCPD_Moveit_Node(std::string const& robot_namespace)
      : robot_namespace_(robot_namespace),
        robot_description_(robot_namespace + "/robot_description"),
        ph("~"),
        scene_monitor_(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description_)),
        model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>(robot_description_)),
        model_(model_loader_->getModel()),
        visual_tools_("robot_root", "cdcpd_moveit_node", scene_monitor_),
        tf_listener_(tf_buffer_) {
    auto const scene_topic = ros::names::append(robot_namespace, "move_group/monitored_planning_scene");
    auto const service_name = ros::names::append(robot_namespace, "get_planning_scene");
    scene_monitor_->startSceneMonitor(scene_topic);
    moveit_ready = scene_monitor_->requestPlanningSceneState(service_name);
    if (not moveit_ready) {
      ROS_WARN_NAMED(LOGNAME, "Could not get the moveit planning scene. This means no obstacle constraints.");
    }

    // Publishers for the data, some visualizations, others consumed by other nodes
    auto original_publisher = nh.advertise<PointCloud>("cdcpd/original", 10);
    auto masked_publisher = nh.advertise<PointCloud>("cdcpd/masked", 10);
    auto downsampled_publisher = nh.advertise<PointCloud>("cdcpd/downsampled", 10);
    auto template_publisher = nh.advertise<PointCloud>("cdcpd/template", 10);
    auto pre_template_publisher = nh.advertise<PointCloud>("cdcpd/pre_template", 10);
    auto output_publisher = nh.advertise<PointCloud>("cdcpd/output", 10);
    auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);
    contact_marker_pub = ph.advertise<vm::MarkerArray>("contacts", 10);
    bbox_pub = ph.advertise<jsk_recognition_msgs::BoundingBox>("cdcpd/bbox", 10);

    // Moveit Visualization
    auto const viz_robot_state_topic = "cdcpd_moveit_node/robot_state";
    visual_tools_.loadRobotStatePub(viz_robot_state_topic, false);

    auto const kinect_name = ROSHelpers::GetParam<std::string>(ph, "kinect_name", "kinect2");

    // For use with TF and "fixed points" for the constrain step
    kinect_tf_name = kinect_name + "_rgb_optical_frame";
    auto const left_tf_name = ROSHelpers::GetParam<std::string>(ph, "left_tf_name", "");
    auto const right_tf_name = ROSHelpers::GetParam<std::string>(ph, "right_tf_name", "");
    auto const num_points = ROSHelpers::GetParam<int>(nh, "rope_num_points", 11);
    auto const left_node_idx = ROSHelpers::GetParam<int>(ph, "left_node_idx", num_points - 1);
    auto const right_node_idx = ROSHelpers::GetParam<int>(ph, "right_node_idx", 1);
    auto const use_gripper_constraints = ROSHelpers::GetParam<bool>(ph, "use_gripper_constraints", true);
    Eigen::MatrixXi gripper_idx(1, 2);
    gripper_idx << left_node_idx, right_node_idx;

    // Initial connectivity model of rope
    auto const rope_length = ROSHelpers::GetParam<float>(nh, "rope_length", 1.0);
    auto const max_segment_length = rope_length / static_cast<float>(num_points);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "max segment length " << max_segment_length);

    wait_for_tf(left_tf_name, right_tf_name);

    auto const left_gripper = tf_buffer_.lookupTransform(kinect_tf_name, left_tf_name, ros::Time(0));
    auto const right_gripper = tf_buffer_.lookupTransform(kinect_tf_name, right_tf_name, ros::Time(0));

    Eigen::Vector3f const start_position =
        ehc::GeometryVector3ToEigenVector3d(left_gripper.transform.translation).cast<float>();
    Eigen::Vector3f const end_position =
        ehc::GeometryVector3ToEigenVector3d(right_gripper.transform.translation).cast<float>();

    auto const& initial_template_pair = makeRopeTemplate(num_points, start_position, end_position);
    auto const& initial_template_vertices = initial_template_pair.first;
    auto const& initial_template_edges = initial_template_pair.second;

    // Construct the initial template as a PCL cloud
    auto const& initial_tracked_points = makeCloud(initial_template_vertices);
    PointCloud::Ptr tracked_points = initial_tracked_points;  // non-const, modified each time

    // TODO: describe each parameter in words (and a pointer to an equation/section of paper)
    auto const objective_value_threshold = ROSHelpers::GetParam<double>(ph, "objective_value_threshold", 1.0);
    auto const alpha = ROSHelpers::GetParam<double>(ph, "alpha", 0.5);
    auto const lambda = ROSHelpers::GetParam<double>(ph, "lambda", 1.0);
    auto const k_spring = ROSHelpers::GetParam<double>(ph, "k", 100.0);
    auto const beta = ROSHelpers::GetParam<double>(ph, "beta", 1.0);
    auto const zeta = ROSHelpers::GetParam<double>(ph, "zeta", 10.0);
    min_distance_threshold = ROSHelpers::GetParam<double>(ph, "min_distance_threshold", 0.01);
    auto const obstacle_cost_weight = ROSHelpers::GetParam<double>(ph, "obstacle_cost_weight", 0.001);
    auto const fixed_points_weight = ROSHelpers::GetParam<double>(ph, "fixed_points_weight", 10.0);
    // NOTE: original cdcpd recovery not implemented
    auto const use_recovery = ROSHelpers::GetParam<bool>(ph, "use_recovery", false);
    auto const kinect_channel = ROSHelpers::GetParam<std::string>(ph, "kinect_channel", "qhd");
    auto const kinect_prefix = kinect_name + "/" + kinect_channel;

    auto cdcpd = CDCPD(nh, ph, initial_tracked_points, initial_template_edges, objective_value_threshold, use_recovery,
                       alpha, beta, lambda, k_spring, zeta, obstacle_cost_weight, fixed_points_weight);

    auto const callback = [&](cv::Mat const& rgb, cv::Mat const& depth, cv::Matx33d const& intrinsics) {
      auto const t0 = ros::Time::now();
      smmap::AllGrippersSinglePose q_config;
      // Left Gripper
      if (not left_tf_name.empty()) {
        try {
          auto const gripper = tf_buffer_.lookupTransform(kinect_tf_name, left_tf_name, ros::Time(0));
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
          auto const gripper = tf_buffer_.lookupTransform(kinect_tf_name, right_tf_name, ros::Time(0));
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

      // publish bbox
      {
        jsk_recognition_msgs::BoundingBox bbox_msg;
        bbox_msg.header.stamp = ros::Time::now();
        bbox_msg.header.frame_id = kinect_tf_name;

        auto const bbox_size = extent_to_env_size(cdcpd.last_lower_bounding_box, cdcpd.last_upper_bounding_box);
        auto const bbox_center = extent_to_center(cdcpd.last_lower_bounding_box, cdcpd.last_upper_bounding_box);
        bbox_msg.pose.position.x = bbox_center.x();
        bbox_msg.pose.position.y = bbox_center.y();
        bbox_msg.pose.position.z = bbox_center.z();
        bbox_msg.pose.orientation.w = 1;
        bbox_msg.dimensions.x = bbox_size.x();
        bbox_msg.dimensions.y = bbox_size.y();
        bbox_msg.dimensions.z = bbox_size.z();
        bbox_pub.publish(bbox_msg);
      }

      // publish the template before processing
      {
        auto time = ros::Time::now();
        tracked_points->header.frame_id = kinect_tf_name;
        pcl_conversions::toPCL(time, tracked_points->header.stamp);
        pre_template_publisher.publish(tracked_points);
      }

      ObstacleConstraints obstacle_constraints;
      if (moveit_ready and moveit_enabled) {
        obstacle_constraints = get_moveit_obstacle_constriants(tracked_points);
      }

      if (not use_gripper_constraints) {
        q_config = {};
        gripper_idx = {};
      }
      auto const out = cdcpd(rgb, depth, hsv_mask, intrinsics, tracked_points, obstacle_constraints, max_segment_length,
                             q_dot, q_config, gripper_idx);
      tracked_points = out.gurobi_output;

      // Update the frame ids
      {
        out.original_cloud->header.frame_id = kinect_tf_name;
        out.masked_point_cloud->header.frame_id = kinect_tf_name;
        out.downsampled_cloud->header.frame_id = kinect_tf_name;
        out.cpd_output->header.frame_id = kinect_tf_name;
        out.gurobi_output->header.frame_id = kinect_tf_name;
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
          order.color.r = 0.1;
          order.color.g = 0.6;
          order.color.b = 0.9;
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

      // compute length and print that for debugging purposes
      auto output_length{0.0};
      for (auto point_idx{0}; point_idx < tracked_points->size() - 1; ++point_idx) {
        Eigen::Vector3f const p = tracked_points->at(point_idx + 1).getVector3fMap();
        Eigen::Vector3f const p_next = tracked_points->at(point_idx).getVector3fMap();
        output_length += (p_next - p).norm();
      }
      ROS_DEBUG_STREAM_NAMED(LOGNAME + ".length", "length = " << output_length << " desired length = " << rope_length);

      auto const t1 = ros::Time::now();
      auto const dt = t1 - t0;
      ROS_DEBUG_STREAM_NAMED(PERF_LOGGER, "dt = " << dt.toSec() << "s");

      if (out.status == OutputStatus::NoPointInFilteredCloud or out.status == OutputStatus::ObjectiveTooHigh) {
        // recreating
        cdcpd = CDCPD(nh, ph, initial_tracked_points, initial_template_edges, objective_value_threshold, use_recovery,
                      alpha, beta, lambda, k_spring, zeta, obstacle_cost_weight, fixed_points_weight);
      }
    };

    auto kinect_sub_setup = KinectSubSetup(kinect_prefix);
    // wait a second so the TF buffer can fill
    ros::Duration(0.5).sleep();

    KinectSub sub(callback, kinect_sub_setup);

    ros::waitForShutdown();
  }
  void wait_for_tf(const std::string& left_tf_name, const std::string& right_tf_name) const {
    ROS_INFO_NAMED(LOGNAME, "Waiting for TF...");
    while (true) {
      if (tf_buffer_.canTransform(kinect_tf_name, left_tf_name, ros::Time(0)) and
          tf_buffer_.canTransform(kinect_tf_name, right_tf_name, ros::Time(0))) {
        break;
      }
    }
  }

  ObstacleConstraints find_nearest_points_and_normals(planning_scene_monitor::LockedPlanningSceneRW planning_scene,
                                                      Eigen::Isometry3d const& cdcpd_to_moveit) {
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.distance = true;
    req.max_contacts_per_pair = 1;
    collision_detection::CollisionResult res;
    planning_scene->checkCollisionUnpadded(req, res);

    vm::MarkerArray contact_markers;
    ObstacleConstraints obstacle_constraints;
    auto contact_idx = 0u;
    for (auto const& [contact_names, contacts] : res.contacts) {
      if (contacts.empty()) {
        continue;
      }

      auto const contact = contacts[0];
      auto add_interaction_constraint = [&](int contact_idx, int body_idx, std::string body_name,
                                            Eigen::Vector3d const& tracked_point_moveit_frame,
                                            Eigen::Vector3d const& object_point_moveit_frame) {
        // NOTE: if the tracked_point is inside the object, contact.depth will be negative. In this case, the normal
        // points in the opposite direction, starting at object_point and going _away_ from tracked_point.
        auto const normal_dir = contact.depth > 0.0 ? 1.0 : -1.0;
        Eigen::Vector3d const object_point_cdcpd_frame = cdcpd_to_moveit.inverse() * object_point_moveit_frame;
        Eigen::Vector3d const tracked_point_cdcpd_frame = cdcpd_to_moveit.inverse() * tracked_point_moveit_frame;
        Eigen::Vector3d const normal_cdcpd_frame =
            ((tracked_point_cdcpd_frame - object_point_cdcpd_frame) * normal_dir).normalized();
        auto get_point_idx = [&]() {
          unsigned int point_idx;
          sscanf(body_name.c_str(), (collision_body_prefix + "%u").c_str(), &point_idx);
          return point_idx;
        };
        auto const point_idx = get_point_idx();
        obstacle_constraints.emplace_back(
            ObstacleConstraint{point_idx, object_point_cdcpd_frame.cast<float>(), normal_cdcpd_frame.cast<float>()});

        // debug & visualize
        {
          ROS_DEBUG_STREAM_NAMED(
              LOGNAME, "nearest point: " << contact.nearest_points[0].x() << ", " << contact.nearest_points[0].y()
                                         << ", " << contact.nearest_points[0].z() << " on " << contact.body_name_1
                                         << " and " << contact.nearest_points[1].x() << ", "
                                         << contact.nearest_points[1].y() << ", " << contact.nearest_points[1].z()
                                         << " on " << contact.body_name_2 << " depth " << contact.depth
                                         << " (in moveit frame)");

          vm::Marker arrow;
          arrow.id = 100 * contact_idx + 0;
          arrow.action = vm::Marker::ADD;
          arrow.type = vm::Marker::ARROW;
          arrow.ns = "arrow";
          arrow.header.frame_id = moveit_frame;
          arrow.header.stamp = ros::Time::now();
          arrow.color.r = 1.0;
          arrow.color.g = 0.0;
          arrow.color.b = 1.0;
          arrow.color.a = 0.2;
          arrow.scale.x = 0.001;
          arrow.scale.y = 0.002;
          arrow.scale.z = 0.002;
          arrow.pose.orientation.w = 1;
          arrow.points.push_back(ConvertTo<geometry_msgs::Point>(object_point_moveit_frame));
          arrow.points.push_back(ConvertTo<geometry_msgs::Point>(tracked_point_moveit_frame));

          vm::Marker normal;
          normal.id = 100 * contact_idx + 0;
          normal.action = vm::Marker::ADD;
          normal.type = vm::Marker::ARROW;
          normal.ns = "normal";
          normal.header.frame_id = kinect_tf_name;
          normal.header.stamp = ros::Time::now();
          normal.color.r = 0.4;
          normal.color.g = 1.0;
          normal.color.b = 0.7;
          normal.color.a = 0.6;
          normal.scale.x = 0.0015;
          normal.scale.y = 0.0025;
          normal.scale.z = 0.0025;
          normal.pose.orientation.w = 1;
          normal.points.push_back(ConvertTo<geometry_msgs::Point>(object_point_cdcpd_frame));
          Eigen::Vector3d const normal_end_point_cdcpd_frame = object_point_cdcpd_frame + normal_cdcpd_frame * 0.02;
          normal.points.push_back(ConvertTo<geometry_msgs::Point>(normal_end_point_cdcpd_frame));

          contact_markers.markers.push_back(arrow);
          contact_markers.markers.push_back(normal);
        }
      };

      if (contact.depth > min_distance_threshold) {
        continue;
      }
      if (contact.body_name_1.find(collision_body_prefix) != std::string::npos) {
        add_interaction_constraint(contact_idx, 0, contact.body_name_1, contact.nearest_points[0],
                                   contact.nearest_points[1]);
      } else if (contact.body_name_2.find(collision_body_prefix) != std::string::npos) {
        add_interaction_constraint(contact_idx, 1, contact.body_name_2, contact.nearest_points[1],
                                   contact.nearest_points[0]);
      } else {
        continue;
      }

      ++contact_idx;
    }

    vm::MarkerArray clear_array;
    vm::Marker clear_marker;
    clear_marker.action = vm::Marker::DELETEALL;
    clear_array.markers.push_back(clear_marker);
    contact_marker_pub.publish(clear_array);
    contact_marker_pub.publish(contact_markers);
    return obstacle_constraints;
  }

  ObstacleConstraints get_moveit_obstacle_constriants(PointCloud::ConstPtr tracked_points) {
    Eigen::Isometry3d cdcpd_to_moveit;
    try {
      auto const cdcpd_to_moveit_msg = tf_buffer_.lookupTransform(moveit_frame, kinect_tf_name, ros::Time(0));
      cdcpd_to_moveit = ehc::GeometryTransformToEigenIsometry3d(cdcpd_to_moveit_msg.transform);
    } catch (tf2::TransformException const& ex) {
      ROS_WARN_STREAM_THROTTLE(
          10.0, "Unable to lookup transform from " << kinect_tf_name << " to " << moveit_frame << ": " << ex.what());
      return {};
    }

    planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor_);

    // customize by excluding some objects
    auto& world = planning_scene->getWorldNonConst();
    std::vector<std::string> objects_to_ignore{
        "collision_sphere.link_1",
        "ground_plane.link",
    };
    for (auto const& object_to_ignore : objects_to_ignore) {
      if (world->hasObject(object_to_ignore)) {
        auto success = world->removeObject(object_to_ignore);
        if (success) {
          ROS_DEBUG_STREAM_NAMED(LOGNAME, "Successfully removed " << object_to_ignore);
        } else {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to remove " << object_to_ignore);
        }
      }
    }

    auto& robot_state = planning_scene->getCurrentStateNonConst();

    // remove the attached "tool boxes"
    std::vector<std::string> objects_to_detach{"left_tool_box", "right_tool_box"};
    for (auto const& object_to_detach : objects_to_detach) {
      if (not robot_state.hasAttachedBody(object_to_detach)) {
        continue;
      }
      auto success = robot_state.clearAttachedBody(object_to_detach);
      if (not success) {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to detach " << object_to_detach);
      }
    }

    planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    // attach to the robot base link, sort of hacky but MoveIt only has API for checking robot vs self/world,
    // so we have to make the tracked points part of the robot, hence "attached collision objects"
    for (auto const& [tracked_point_idx, point] : enumerate(*tracked_points)) {
      Eigen::Vector3d const tracked_point_cdcpd_frame = point.getVector3fMap().cast<double>();
      Eigen::Vector3d const tracked_point_moveit_frame = cdcpd_to_moveit * tracked_point_cdcpd_frame;
      Eigen::Isometry3d tracked_point_pose_moveit_frame = Eigen::Isometry3d::Identity();
      tracked_point_pose_moveit_frame.translation() = tracked_point_moveit_frame;

      std::stringstream collision_body_name_stream;
      collision_body_name_stream << collision_body_prefix << tracked_point_idx;
      auto const collision_body_name = collision_body_name_stream.str();

      // FIXME: not moveit frame, but the base link_frame, could those be different?
      auto sphere = std::make_shared<shapes::Box>(0.01, 0.01, 0.01);

      robot_state.attachBody(collision_body_name, Eigen::Isometry3d::Identity(), {sphere},
                             {tracked_point_pose_moveit_frame}, std::vector<std::string>{}, "base");
    }

    // visualize
    visual_tools_.publishRobotState(robot_state, rviz_visual_tools::CYAN);

    return find_nearest_points_and_normals(planning_scene, cdcpd_to_moveit);
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cdcpd_node");
  CDCPD_Moveit_Node cmn("hdt_michigan");
  return EXIT_SUCCESS;
}
