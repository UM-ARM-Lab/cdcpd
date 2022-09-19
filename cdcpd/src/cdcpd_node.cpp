#include <arc_utilities/enumerate.h>
#include <cdcpd/SetGripperConstraints.h>
#include <cdcpd/cdcpd.h>
#include <cdcpd/sdformat_to_planning_scene.h>
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
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include "cdcpd_ros/camera_sub.h"

std::string const LOGNAME = "cdcpd_node";
constexpr auto const MAX_CONTACTS_VIZ = 500;
constexpr auto const PERF_LOGGER = "perf";

struct ConstraintPosDotIndices {
  smmap::AllGrippersSinglePose q_config;
  smmap::AllGrippersSinglePoseDelta q_dot;
  Eigen::MatrixXi gripper_indices;
};

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

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int num_points, const Eigen::Vector3f& start_position,
                                                               const Eigen::Vector3f& end_position);

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
  auto const sat_min = ROSHelpers::GetParamDebugLog<double>(ph, "saturation_min", 0.3);
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
  ros::Publisher scene_pub;
  ros::ServiceServer gripper_service_;
  std::vector<cdcpd::GripperConstraint> gripper_constraints_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr model_loader_;
  robot_model::RobotModelPtr model_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  std::string moveit_frame{"robot_root"};
  std::string camera_frame;
  double min_distance_threshold{0.01};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster br_;

  std::string sdf_filename = "/home/peter/catkin_ws/src/cdcpd/cdcpd/car5_real_cdcpd.world";
  planning_scene::PlanningScenePtr planning_scene_;
  moveit_msgs::PlanningScene planning_scene_msg_;

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

    // Publishers for the data, some visualizations, others consumed by other nodes
    auto original_publisher = nh.advertise<PointCloud>("cdcpd/original", 10);
    auto masked_publisher = nh.advertise<PointCloud>("cdcpd/masked", 10);
    auto downsampled_publisher = nh.advertise<PointCloud>("cdcpd/downsampled", 10);
    auto template_publisher = nh.advertise<PointCloud>("cdcpd/template", 10);
    auto pre_template_publisher = nh.advertise<PointCloud>("cdcpd/pre_template", 10);
    auto output_publisher = nh.advertise<PointCloud>("cdcpd/output", 10);
    auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);
    contact_marker_pub = ph.advertise<vm::MarkerArray>("contacts", 10);
    bbox_pub = ph.advertise<jsk_recognition_msgs::BoundingBox>("bbox", 10);
    scene_pub = ph.advertise<moveit_msgs::PlanningScene>("scene", 10);

    // point cloud input takes precedence. If points_name is not empty, we will use that and not RGB + Depth
    auto const points_name = ROSHelpers::GetParam<std::string>(ph, "points", "");
    auto const rgb_topic = ROSHelpers::GetParam<std::string>(ph, "rgb_topic", "/camera/color/image_raw");
    auto const depth_topic = ROSHelpers::GetParam<std::string>(ph, "depth_topic", "/camera/depth/image_rect_raw");
    auto const info_topic = ROSHelpers::GetParam<std::string>(ph, "info_topic", "/camera/depth/camera_info");
    camera_frame = ROSHelpers::GetParam<std::string>(ph, "camera_frame", "camera_color_optical_frame");

    auto const max_rope_length = ROSHelpers::GetParam<float>(nh, "max_rope_length", 1.0);

    planning_scene_ = sdf_to_planning_scene(sdf_filename, "mock_camera_link");
    planning_scene_->getPlanningSceneMsg(planning_scene_msg_);

    gripper_service_ = ph.advertiseService("set_gripper_constraints", &CDCPD_Moveit_Node::SetGripperConstraints, this);

    // For use with TF and "fixed points" for the constraint step
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Using frame " << camera_frame);
    auto const num_points = ROSHelpers::GetParam<int>(nh, "rope_num_points", 11);

    cdcpd::GripperConstraint left_gripper_constraint;
    left_gripper_constraint.frame_id = "mocap_left_hand_left_hand";
    left_gripper_constraint.node_index = 0;
    gripper_constraints_.push_back(left_gripper_constraint);
    cdcpd::GripperConstraint right_gripper_constraint;
    right_gripper_constraint.frame_id = "mocap_right_hand_right_hand";
    right_gripper_constraint.node_index = 24;
    gripper_constraints_.push_back(right_gripper_constraint);

    auto get_q_config = [&]() {
      smmap::AllGrippersSinglePose q_config;
      std::vector<int> gripper_indices_vec;
      for (auto const gripper_constraint : gripper_constraints_) {
        auto const tf_name = gripper_constraint.frame_id;
        gripper_indices_vec.push_back(gripper_constraint.node_index);
        if (not tf_name.empty()) {
          try {
            auto const gripper = tf_buffer_.lookupTransform(camera_frame, tf_name, ros::Time(0), ros::Duration(10));
            auto const config = ehc::GeometryTransformToEigenIsometry3d(gripper.transform);
            ROS_DEBUG_STREAM_NAMED(LOGNAME + ".grippers", "gripper: " << config.translation());
            q_config.push_back(config);
          } catch (tf2::TransformException const& ex) {
            ROS_WARN_STREAM_THROTTLE(
                10.0, "Unable to lookup transform from " << camera_frame << " to " << tf_name << ": " << ex.what());
          }
        }
      }

      const smmap::AllGrippersSinglePoseDelta q_dot{gripper_constraints_.size(), kinematics::Vector6d::Zero()};

      Eigen::MatrixXi gripper_indices_eigen;
      gripper_indices_eigen.resize(1, gripper_indices_vec.size());
      for (auto const [i, gripper_idx] : enumerate(gripper_indices_vec)) {
        gripper_indices_eigen(0, i) = gripper_idx;
      }
      return ConstraintPosDotIndices{q_config, q_dot, gripper_indices_eigen};
    };

    auto get_start_end_points = [&]() {
      // initialize start and end points for rope
      auto const gripper_constraints = get_q_config();
      auto const q_config = gripper_constraints.q_config;
      Eigen::Vector3f start_position{Eigen::Vector3f::Zero()};
      Eigen::Vector3f end_position{Eigen::Vector3f::Zero()};
      end_position[2] += max_rope_length;
      if (gripper_constraints_.size() == 2u) {
        start_position = q_config[0].translation().cast<float>();
        end_position = q_config[1].translation().cast<float>();
      } else if (gripper_constraints_.size() == 1u) {
        end_position = q_config[0].translation().cast<float>();
        start_position = end_position;
        start_position[1] += max_rope_length;
      } else if (gripper_constraints_.empty()) {
        start_position << -max_rope_length / 2, 0, 1.0;
        end_position << max_rope_length / 2, 0, 1.0;
      }

      return std::tuple{start_position, end_position};
    };

    // Initial connectivity model of rope
    auto const max_segment_length = max_rope_length / static_cast<float>(num_points - 1);
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "max segment length " << max_segment_length);

    auto const& [start_position, end_position] = get_start_end_points();
    auto const& initial_template = makeRopeTemplate(num_points, start_position, end_position);
    auto const& initial_template_vertices = initial_template.first;
    auto const& initial_template_edges = initial_template.second;
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
    auto const moveit_enabled = ROSHelpers::GetParam<bool>(ph, "moveit_enabled", false);

    auto cdcpd = CDCPD(nh, ph, initial_tracked_points, initial_template_edges, objective_value_threshold, use_recovery,
                       alpha, beta, lambda, k_spring, zeta, obstacle_cost_weight, fixed_points_weight);

    auto publish_bbox = [&]() {
      jsk_recognition_msgs::BoundingBox bbox_msg;
      bbox_msg.header.stamp = ros::Time::now();
      bbox_msg.header.frame_id = camera_frame;

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
    };

    auto publish_template = [&]() {
      auto time = ros::Time::now();
      tracked_points->header.frame_id = camera_frame;
      pcl_conversions::toPCL(time, tracked_points->header.stamp);
      pre_template_publisher.publish(tracked_points);
    };

    auto get_obstacle_constraints = [&]() {
      ObstacleConstraints obstacle_constraints;
      if (moveit_enabled) {
        obstacle_constraints = get_moveit_obstacle_constraints(tracked_points);
        ROS_DEBUG_NAMED(LOGNAME + ".moveit", "Got moveit obstacle constraints");
      }
      return obstacle_constraints;
    };

    auto publish_outputs = [&](ros::Time const& t0, CDCPD::Output const& out) {
      // Update the frame ids
      {
        out.original_cloud->header.frame_id = camera_frame;
        out.masked_point_cloud->header.frame_id = camera_frame;
        out.downsampled_cloud->header.frame_id = camera_frame;
        out.cpd_output->header.frame_id = camera_frame;
        out.gurobi_output->header.frame_id = camera_frame;
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
          order.header.frame_id = camera_frame;
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
      ROS_DEBUG_STREAM_NAMED(LOGNAME + ".length", "length = " << output_length << " max length = " << max_rope_length);

      auto const t1 = ros::Time::now();
      auto const dt = t1 - t0;
      ROS_DEBUG_STREAM_NAMED(PERF_LOGGER, "dt = " << dt.toSec() << "s");
    };

    auto reset_if_bad = [&](CDCPD::Output const& out) {
      if (out.status == OutputStatus::NoPointInFilteredCloud or out.status == OutputStatus::ObjectiveTooHigh) {
        ROS_INFO_STREAM_NAMED(LOGNAME + ".reset", "Resetting! status code " << out.status);
        auto const& [current_start_position, current_end_position] = get_start_end_points();
        auto const& current_template_pair = makeRopeTemplate(num_points, current_start_position, current_end_position);
        auto const& current_template_vertices = current_template_pair.first;
        auto const& current_template_edges = current_template_pair.second;
        auto const& current_tracked_points = makeCloud(current_template_vertices);
        ROS_INFO_STREAM_NAMED(LOGNAME + ".reset", "current points " << current_start_position << " " << current_end_position);
        tracked_points = current_tracked_points;
        cdcpd = CDCPD(nh, ph, current_tracked_points, current_template_edges, objective_value_threshold, use_recovery,
                      alpha, beta, lambda, k_spring, zeta, obstacle_cost_weight, fixed_points_weight);
      }
    };

    auto const callback = [&](cv::Mat const& rgb, cv::Mat const& depth, cv::Matx33d const& intrinsics) {
      auto const t0 = ros::Time::now();
      auto [q_config, q_dot, gripper_indices] = get_q_config();

      publish_bbox();

      auto obstacle_constraints = get_obstacle_constraints();

      auto const hsv_mask = getHsvMask(ph, rgb);
      auto const out = cdcpd(rgb, depth, hsv_mask, intrinsics, tracked_points, obstacle_constraints, max_segment_length,
                             q_dot, q_config, gripper_indices);
      tracked_points = out.gurobi_output;
      publish_outputs(t0, out);
      reset_if_bad(out);
    };

    auto const points_callback = [&](const sensor_msgs::PointCloud2ConstPtr& points_msg) {
      auto const t0 = ros::Time::now();
      auto [q_config, q_dot, gripper_indices] = get_q_config();

      publish_bbox();
      publish_template();
      auto obstacle_constraints = get_obstacle_constraints();

      pcl::PCLPointCloud2 points_v2;
      pcl_conversions::toPCL(*points_msg, points_v2);
      auto points = boost::make_shared<PointCloudRGB>();
      pcl::fromPCLPointCloud2(points_v2, *points);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "unfiltered points: " << points->size());

      auto const out =
          cdcpd(points, tracked_points, obstacle_constraints, max_segment_length, q_dot, q_config, gripper_indices);
      tracked_points = out.gurobi_output;
      publish_outputs(t0, out);
      reset_if_bad(out);
    };

    if (points_name.empty()) {
      ROS_INFO_NAMED(LOGNAME, "subscribing to RGB + Depth");
      auto camera_sub_setup = CameraSubSetup(rgb_topic, depth_topic, info_topic);
      // wait a second so the TF buffer can fill
      ros::Duration(0.5).sleep();

      KinectSub sub(callback, camera_sub_setup);
      ros::waitForShutdown();
    } else {
      ROS_INFO_NAMED(LOGNAME, "subscribing to points");
      auto sub = nh.subscribe<sensor_msgs::PointCloud2>(points_name, 10, points_callback);
      ros::spin();
    }
  }

  bool SetGripperConstraints(cdcpd::SetGripperConstraints::Request& req, cdcpd::SetGripperConstraints::Response& res) {
    gripper_constraints_ = req.constraints;
    return true;
  }

  ObstacleConstraints find_nearest_points_and_normals(planning_scene::PlanningScenePtr planning_scene) {
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.distance = false;

    req.max_contacts_per_pair = 1;
    collision_detection::CollisionResult res;
    auto const t0 = ros::Time::now();
    planning_scene->checkCollisionUnpadded(req, res);
    auto const t1 = ros::Time::now();
    auto const dt = t1 - t0;
    ROS_DEBUG_STREAM_NAMED(PERF_LOGGER, "checkCollision = " << dt.toSec() << "s");

    // first fill up the contact markers with "zero" markers
    // rviz makes deleting markers hard, so it's easier to just publish a fixed number of markers in the array
    vm::MarkerArray contact_markers;
    for (auto i{0}; i < MAX_CONTACTS_VIZ; ++i) {
      auto const [arrow, normal] =
          arrow_and_normal(i, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      contact_markers.markers.push_back(arrow);
      contact_markers.markers.push_back(normal);
    }

    ObstacleConstraints obstacle_constraints;
    auto contact_idx = 0u;
    for (auto const& [contact_names, contacts] : res.contacts) {
      if (contacts.empty()) {
        continue;
      }

      auto const contact = contacts[0];
      auto add_interaction_constraint = [&](int contact_idx, int body_idx, std::string body_name,
                                            Eigen::Vector3d const& tracked_point_cdcpd_frame,
                                            Eigen::Vector3d const& object_point_cdcpd_frame) {
        // NOTE: if the tracked_point is inside the object, contact.depth will be negative. In this case, the normal
        // points in the opposite direction, starting at object_point and going _away_ from tracked_point.
        auto const normal_dir = contact.depth > 0.0 ? 1.0 : -1.0;
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
              LOGNAME + ".moveit",
              "nearest point: " << contact.nearest_points[0].x() << ", " << contact.nearest_points[0].y() << ", "
                                << contact.nearest_points[0].z() << " on " << contact.body_name_1 << " and "
                                << contact.nearest_points[1].x() << ", " << contact.nearest_points[1].y() << ", "
                                << contact.nearest_points[1].z() << " on " << contact.body_name_2 << " depth "
                                << contact.depth << " (in camera frame)");
          auto const [arrow, normal] =
              arrow_and_normal(contact_idx, tracked_point_cdcpd_frame, object_point_cdcpd_frame, normal_cdcpd_frame);
          if (contact_idx < MAX_CONTACTS_VIZ) {
            contact_markers.markers[2 * contact_idx] = arrow;
            contact_markers.markers[2 * contact_idx + 1] = normal;
          }
        }
      };

      if (contact.depth > min_distance_threshold) {
        continue;
      }
      std::vector<std::string> bodies_to_ignore{
          "leftgripper_link", "leftgripper2_link", "left_tool", "rightgripper_link", "rightgripper2_link", "right_tool",
      };
      if (std::find(bodies_to_ignore.cbegin(), bodies_to_ignore.cend(), contact.body_name_1) !=
          bodies_to_ignore.cend()) {
        ROS_DEBUG_STREAM_NAMED(LOGNAME + ".moveit", "Ignoring " << contact.body_name_1);
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

    ROS_DEBUG_STREAM_NAMED(LOGNAME + ".contacts", "contacts: " << contact_idx);
    contact_marker_pub.publish(contact_markers);
    return obstacle_constraints;
  }
  std::pair<vm::Marker, vm::Marker> arrow_and_normal(int contact_idx, const Eigen::Vector3d& tracked_point_cdcpd_frame,
                                                     const Eigen::Vector3d& object_point_cdcpd_frame,
                                                     const Eigen::Vector3d& normal_cdcpd_frame) const {
    vm::Marker arrow, normal;
    arrow.id = 100 * contact_idx + 0;
    arrow.action = vm::Marker::ADD;
    arrow.type = vm::Marker::ARROW;
    arrow.ns = "arrow";
    arrow.header.frame_id = camera_frame;
    arrow.header.stamp = ros::Time::now();
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 1.0;
    arrow.color.a = 0.2;
    arrow.scale.x = 0.001;
    arrow.scale.y = 0.002;
    arrow.scale.z = 0.002;
    arrow.pose.orientation.w = 1;
    arrow.points.push_back(ConvertTo<geometry_msgs::Point>(object_point_cdcpd_frame));
    arrow.points.push_back(ConvertTo<geometry_msgs::Point>(tracked_point_cdcpd_frame));
    normal.id = 100 * contact_idx + 0;
    normal.action = vm::Marker::ADD;
    normal.type = vm::Marker::ARROW;
    normal.ns = "normal";
    normal.header.frame_id = camera_frame;
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

    return {arrow, normal};
  }

  ObstacleConstraints get_moveit_obstacle_constraints(PointCloud::ConstPtr tracked_points) {
    // planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(scene_monitor_);
    // auto planning_scene = locked_planning_scene.operator->();

    // planning_scene_ = sdf_to_planning_scene(sdf_filename, "mock_camera_link");
    // planning_scene_->getPlanningSceneMsg(planning_scene_msg_);

    scene_pub.publish(planning_scene_msg_);
    geometry_msgs::TransformStamped identity_transform_msg;
    identity_transform_msg.child_frame_id = "mock_camera_link";
    identity_transform_msg.header.frame_id = camera_frame;
    identity_transform_msg.header.stamp = ros::Time::now();
    identity_transform_msg.transform.rotation.w = 1;
    br_.sendTransform(identity_transform_msg);

    // customize by excluding some objects
    auto& world = planning_scene_->getWorldNonConst();
    std::vector<std::string> objects_to_ignore{
        "collision_sphere.link_1",
        "ground_plane.link",
    };
    for (auto const& object_to_ignore : objects_to_ignore) {
      if (world->hasObject(object_to_ignore)) {
        auto success = world->removeObject(object_to_ignore);
        if (success) {
          ROS_DEBUG_STREAM_NAMED(LOGNAME + ".moveit", "Successfully removed " << object_to_ignore);
        } else {
          ROS_ERROR_STREAM_NAMED(LOGNAME + ".moveit", "Failed to remove " << object_to_ignore);
        }
      }
    }

    auto& robot_state = planning_scene_->getCurrentStateNonConst();

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

    planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    // std::vector<pcl::PointXYZ> my_tracked_points;
    // for (double x = -0.1; x < 0.1; x += 0.05) {
    //   for (double y = -0.1; y < 0.1; y += 0.05) {
    //     for (double z = 1; z < 1.5; z += 0.05) {
    //       my_tracked_points.emplace_back(x, y, z);
    //     }
    //   }
    // }
    //
    // attach to the robot base link, sort of hacky but MoveIt only has API for checking robot vs self/world,
    // so we have to make the tracked points part of the robot, hence "attached collision objects"
    for (auto const& [tracked_point_idx, point] : enumerate(*tracked_points)) {
      Eigen::Isometry3d tracked_point_pose_cdcpd_frame = Eigen::Isometry3d::Identity();
      tracked_point_pose_cdcpd_frame.translation() = point.getVector3fMap().cast<double>();

      std::stringstream collision_body_name_stream;
      collision_body_name_stream << collision_body_prefix << tracked_point_idx;
      auto const collision_body_name = collision_body_name_stream.str();

      auto collision_shape = std::make_shared<shapes::Box>(0.01, 0.01, 0.01);

      robot_state.attachBody(collision_body_name, Eigen::Isometry3d::Identity(), {collision_shape},
                             {tracked_point_pose_cdcpd_frame}, std::vector<std::string>{}, "mock_camera_link");
    }

    // visualize
    // visual_tools_.publishRobotState(robot_state, rviz_visual_tools::CYAN);

    ROS_DEBUG_NAMED(LOGNAME + ".moveit", "Finding nearest points and normals");
    return find_nearest_points_and_normals(planning_scene_);
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cdcpd_node");
  CDCPD_Moveit_Node cmn("hdt_michigan");
  return EXIT_SUCCESS;
}
