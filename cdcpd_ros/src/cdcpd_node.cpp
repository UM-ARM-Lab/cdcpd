#include <arc_utilities/enumerate.h>
#include <cdcpd/cdcpd.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_fcl/collision_common.h>
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

constexpr auto const LOGNAME = "cdcpd_node";

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

void print_bodies(robot_state::RobotState const& state) {
  std::vector<robot_state::AttachedBody const*> bs;
  std::cout << "Attached Bodies:\n";
  state.getAttachedBodies(bs);
  for (auto const& b : bs) {
    std::cout << b->getName() << '\n';
  }
};

struct CDCPD_Moveit_Node {
  std::string collision_body_prefix{"cdcpd_tracked_point_"};
  ros::NodeHandle nh;
  ros::NodeHandle ph;
  ros::Publisher contact_marker_pub;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr model_loader_;
  robot_model::RobotModelPtr model_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  std::string moveit_frame{"robot_root"};

  CDCPD_Moveit_Node()
      : ph("~"),
        scene_monitor_(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description")),
        model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>()),
        model_(model_loader_->getModel()),
        visual_tools_("robot_root", "cdcpd_moveit_node", scene_monitor_) {
    std::string robot_namespace{"hdt_michigan"};
    auto const scene_topic = ros::names::append(robot_namespace, "move_group/monitored_planning_scene");
    auto const service_name = ros::names::append(robot_namespace, "get_planning_scene");
    scene_monitor_->startSceneMonitor(scene_topic);
    scene_monitor_->requestPlanningSceneState(service_name);

    // Publsihers for the data, some visualizations, others consumed by other nodes
    auto original_publisher = nh.advertise<PointCloud>("cdcpd/original", 1);
    auto masked_publisher = nh.advertise<PointCloud>("cdcpd/masked", 1);
    auto downsampled_publisher = nh.advertise<PointCloud>("cdcpd/downsampled", 1);
    auto template_publisher = nh.advertise<PointCloud>("cdcpd/template", 1);
    auto pre_template_publisher = nh.advertise<PointCloud>("cdcpd/pre_template", 1);
    auto output_publisher = nh.advertise<PointCloud>("cdcpd/output", 1);
    auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);
    contact_marker_pub = ph.advertise<vm::MarkerArray>("contacts", 10);

    // Moveit Visualization
    auto const viz_robot_state_topic = "cdcpd_moveit_node/robot_state";
    visual_tools_.loadRobotStatePub(viz_robot_state_topic, false);
    //    auto const viz_planning_scene_topic = "cdcpd_moveit_node/planning_scene";
    //    visual_tools_.setPlanningSceneTopic(viz_planning_scene_topic);
    //    visual_tools_.setManualSceneUpdating();
    //    auto const loaded_viz_psm = visual_tools_.loadPlanningSceneMonitor();
    //    if (not loaded_viz_psm) {
    //      ROS_WARN_STREAM_NAMED(LOGNAME, "Failed to load planning scene monitor on topic " <<
    //      viz_planning_scene_topic);
    //    }

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
    Eigen::MatrixXi gripper_idx(1, 2);
    gripper_idx << left_node_idx, right_node_idx;

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

      // publish the template before processing
      {
        auto time = ros::Time::now();
        tracked_points->header.frame_id = kinect_tf_name;
        pcl_conversions::toPCL(time, tracked_points->header.stamp);
        pre_template_publisher.publish(tracked_points);
      }

      //    auto const objects = get_moveit_planning_scene_as_mesh(scene_monitor);
      auto const points_normals = moveit_get_points_normals(scene_monitor_, tf_buffer, kinect_tf_name, tracked_points);

      ros::Duration(0.5).sleep();
      return;

      auto const out = cdcpd(rgb, depth, hsv_mask, intrinsics, tracked_points, points_normals, q_dot, q_config,
                             gripper_idx, false, false);
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
    // wait a second so the TF buffer can fill
    ros::Duration(0.5).sleep();
    KinectSub sub(callback, options);

    ROS_INFO("Spinning...");
    ros::waitForShutdown();
  }

  PointsNormals find_nearest_points_and_normals(planning_scene_monitor::LockedPlanningSceneRW planning_scene,
                                                tf2_ros::Buffer const& tf_buffer, std::string kinect_tf_name,
                                                Eigen::Isometry3d const& cdcpd_to_moveit) {
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.verbose = true;
    req.distance = true;
    req.max_contacts_per_pair = 1;
    req.max_contacts = 1;
    collision_detection::CollisionResult res;
    planning_scene->checkCollisionUnpadded(req, res);

    vm::MarkerArray contact_markers;
    PointsNormals points_normals;
    auto contact_idx = 0u;
    for (auto const& [contact_names, contacts] : res.contacts) {
      // FIXME: is this correct?
      if (contacts.empty()) {
        continue;
      }

      auto const contact = contacts[0];
      auto add_point_normal = [&](int contact_idx, int body_idx) {
        // FIXME: the contact point in moveit frame seems to be wrong
        auto const contact_point_moveit_frame = contact.pos;
        auto const normal_moveit_frame = res.contacts.begin()->second.begin()->normal;
        Eigen::Vector3d const contact_point_cdcpd_frame = cdcpd_to_moveit.inverse() * contact_point_moveit_frame;
        Eigen::Vector3d const normal_cdcpd_frame = cdcpd_to_moveit.inverse() * normal_moveit_frame;
        points_normals.emplace_back(contact_point_cdcpd_frame.cast<float>(), normal_cdcpd_frame.cast<float>());

        // NOTE: debug & visualize
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
          arrow.color.a = 1.0;
          arrow.scale.x = 0.005;
          arrow.scale.y = 0.01;
          arrow.scale.z = 0;
          arrow.pose.orientation.w = 1;
          auto const normal_cdcpd_frame_scaled = normal_cdcpd_frame.normalized() * 0.1;
          Eigen::Vector3d arrow_end_point_cdcpd_frame = contact_point_cdcpd_frame + normal_cdcpd_frame;
          arrow.points.push_back(ConvertTo<geometry_msgs::Point>(contact.nearest_points[0]));
          arrow.points.push_back(ConvertTo<geometry_msgs::Point>(contact.nearest_points[1]));

          contact_markers.markers.push_back(arrow);
        }
      };

      if (contact_names.first.find(collision_body_prefix) != std::string::npos) {
        add_point_normal(contact_idx, 0);
      } else if (contact_names.second.find(collision_body_prefix) != std::string::npos) {
        add_point_normal(contact_idx, 1);
      } else {
        continue;
      }

      ++contact_idx;
    }

    vm::MarkerArray clear_array;
    vm::Marker clear_marker;
    clear_marker.type = vm::Marker::DELETEALL;
    contact_marker_pub.publish(clear_array);
    contact_marker_pub.publish(contact_markers);
    return points_normals;
  }

  PointsNormals moveit_get_points_normals(planning_scene_monitor::PlanningSceneMonitorPtr const& scene_monitor,
                                          tf2_ros::Buffer const& tf_buffer, std::string kinect_tf_name,
                                          PointCloud::ConstPtr tracked_points) {
    Eigen::Isometry3d cdcpd_to_moveit;
    try {
      auto const cdcpd_to_moveit_msg = tf_buffer.lookupTransform(moveit_frame, kinect_tf_name, ros::Time(0));
      cdcpd_to_moveit = ehc::GeometryTransformToEigenIsometry3d(cdcpd_to_moveit_msg.transform);
    } catch (tf2::TransformException const& ex) {
      ROS_WARN_STREAM_THROTTLE(
          10.0, "Unable to lookup transform from " << kinect_tf_name << " to " << moveit_frame << ": " << ex.what());
      return {};
    }

    planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor);
    planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    auto robot_state = planning_scene->getCurrentStateNonConst();

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
      robot_state.attachBody(collision_body_name, {sphere}, {tracked_point_pose_moveit_frame},
                             std::vector<std::string>{}, "base");
    }

    // this shouldn't be necessary...!?
    planning_scene->setCurrentState(robot_state);

    // visualize
    visual_tools_.publishRobotState(robot_state, rviz_visual_tools::CYAN);

    return find_nearest_points_and_normals(planning_scene, tf_buffer, kinect_tf_name, cdcpd_to_moveit);
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cdcpd_node");

  CDCPD_Moveit_Node cmn;

  return EXIT_SUCCESS;
}
