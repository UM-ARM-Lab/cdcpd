#include <iostream>

#include <arc_utilities/enumerate.h>
#include <cdcpd/sdformat_to_planning_scene.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <arc_utilities/eigen_ros_conversions.hpp>

namespace vm = visualization_msgs;

vm::Marker makeSphere(vm::InteractiveMarker& msg) {
  vm::Marker marker;

  marker.type = vm::Marker::SPHERE;
  marker.scale.x = msg.scale * 0.2;
  marker.scale.y = msg.scale * 0.2;
  marker.scale.z = msg.scale * 0.2;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

vm::InteractiveMarkerControl& makeSphereControl(vm::InteractiveMarker& msg) {
  vm::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeSphere(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

int main(int argc, char* argv[]) {
  std::string sdf_filename = "car5_real_cdcpd_mesh.world";
  ros::init(argc, argv, "test_sdf_to_planning_scene");

  ros::NodeHandle nh;
  auto contact_marker_pub = nh.advertise<vm::Marker>("contacts", 10);
  auto scene_pub = nh.advertise<moveit_msgs::PlanningScene>("scene", 10);

  vm::InteractiveMarker int_marker;
  int_marker.header.frame_id = "kinect2_tripodA_rgb_optical_frame";
  int_marker.pose.position.z = 1;
  int_marker.scale = 0.1;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  auto server = std::make_shared<interactive_markers::InteractiveMarkerServer>("basic_controls", "", false);

  makeSphereControl(int_marker);

  int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;

  vm::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_z";
  control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  std::cout << "Before planning scene" << std::endl;
  auto const planning_scene = sdf_to_planning_scene(sdf_filename, "mock_camera_link");
  std::cout << "After planning scene" << std::endl;

  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene->getPlanningSceneMsg(planning_scene_msg);
  scene_pub.publish(planning_scene_msg);

  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

  server->insert(int_marker);
  auto processFeedback = [&](const vm::InteractiveMarkerFeedbackConstPtr& feedback) {
    if (feedback->event_type == vm::InteractiveMarkerFeedback::POSE_UPDATE) {
      auto& robot_state = planning_scene->getCurrentStateNonConst();
      std::vector<pcl::PointXYZ> my_tracked_points;
      my_tracked_points.emplace_back(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

      std::string collision_body_prefix = "test_point";

      // attach to the robot base link, sort of hacky but MoveIt only has API for checking robot vs self/world,
      // so we have to make the tracked points part of the robot, hence "attached collision objects"
      for (auto const& [tracked_point_idx, point] : enumerate(my_tracked_points)) {
        Eigen::Isometry3d tracked_point_pose_cdcpd_frame = Eigen::Isometry3d::Identity();
        tracked_point_pose_cdcpd_frame.translation() = point.getVector3fMap().cast<double>();

        std::stringstream collision_body_name_stream;
        collision_body_name_stream << collision_body_prefix << tracked_point_idx;
        auto const collision_body_name = collision_body_name_stream.str();

        auto collision_shape = std::make_shared<shapes::Sphere>(0.01);

        robot_state.attachBody(collision_body_name, Eigen::Isometry3d::Identity(), {collision_shape},
                               {tracked_point_pose_cdcpd_frame}, std::vector<std::string>{}, "mock_camera_link");
      }

      collision_detection::CollisionRequest req;
      req.contacts = true;
      req.distance = false;
      req.max_contacts_per_pair = 1;
      collision_detection::CollisionResult res;
      planning_scene->checkCollisionUnpadded(req, res);

      vm::Marker contact_marker;
      contact_marker.header.frame_id = "kinect2_tripodA_rgb_optical_frame";
      contact_marker.header.stamp = ros::Time::now();
      contact_marker.color.a = 1;
      contact_marker.color.r = 1;
      contact_marker.scale.x = 0.01;
      contact_marker.scale.y = 0.01;
      contact_marker.scale.z = 0.01;
      contact_marker.action = vm::Marker::ADD;
      contact_marker.type = vm::Marker::SPHERE_LIST;
      contact_marker.pose.orientation.w = 1;
      contact_marker.ns = "contact";
      geometry_msgs::Point zero_point;
      contact_marker.points.push_back(zero_point);

      vm::Marker normal_marker;
      normal_marker.header.frame_id = "kinect2_tripodA_rgb_optical_frame";
      normal_marker.header.stamp = ros::Time::now();
      normal_marker.color.a = 1;
      normal_marker.color.r = 1;
      normal_marker.scale.x = 0.001;
      normal_marker.scale.y = 0.002;
      normal_marker.scale.z = 0.002;
      normal_marker.action = vm::Marker::ADD;
      normal_marker.type = vm::Marker::ARROW;
      normal_marker.pose.orientation.w = 1;
      normal_marker.ns = "normal";
      normal_marker.points.push_back(zero_point);
      normal_marker.points.push_back(zero_point);

      for (auto const& [contact_names, contacts] : res.contacts) {
        if (contacts.empty()) {
          continue;
        }
        auto const contact = contacts[0];
        auto const start = ConvertTo<geometry_msgs::Point>(contact.pos);
        Eigen::Vector3d end_eigen = contact.pos + contact.normal * 0.02;
        auto const end = ConvertTo<geometry_msgs::Point>(end_eigen);
        normal_marker.points[0] = start;
        normal_marker.points[1] = end;

        if (contact.body_name_1.find(collision_body_prefix) != std::string::npos) {
          geometry_msgs::Point point;
          point.x = contact.nearest_points[1][0];
          point.y = contact.nearest_points[1][1];
          point.z = contact.nearest_points[1][2];
          contact_marker.points[0] = point;
        } else if (contact.body_name_2.find(collision_body_prefix) != std::string::npos) {
          geometry_msgs::Point point;
          point.x = contact.nearest_points[0][0];
          point.y = contact.nearest_points[0][1];
          point.z = contact.nearest_points[0][2];
          contact_marker.points[0] = point;
        }
      }

      scene_pub.publish(planning_scene_msg);
      contact_marker_pub.publish(contact_marker);
      contact_marker_pub.publish(normal_marker);
    }

    server->applyChanges();
  };
  server->setCallback(int_marker.name, processFeedback);
  server->applyChanges();

  ros::spin();

  return EXIT_SUCCESS;
}
