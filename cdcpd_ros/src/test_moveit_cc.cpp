#include <arc_utilities/enumerate.h>
#include <cdcpd/cdcpd.h>
//#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/ros_helpers.hpp>

constexpr auto const LOGNAME = "test_moveit_cc";
constexpr auto const COLLISION_BODY_NAME = "cdcpd_tracked_point";

PointNormal find_nearest_point_and_normal(planning_scene_monitor::LockedPlanningSceneRW planning_scene,
                                          pcl::PointXYZ const& point) {
  auto world = planning_scene->getWorldNonConst();
  auto sphere = std::make_shared<shapes::Sphere>(0.02);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = point.x;
  pose.translation().y() = point.y;
  pose.translation().z() = point.z;

  world->addToObject(COLLISION_BODY_NAME, sphere, pose);

  collision_detection::CollisionRequest req;
  req.distance = false;
  req.contacts = true;
  req.verbose = true;
  req.max_contacts = std::numeric_limits<typeof(collision_detection::CollisionRequest::max_contacts)>::max();
  req.max_contacts_per_pair = 1;
  collision_detection::CollisionResult res;
  planning_scene->checkCollisionUnpadded(req, res);

  for (auto const& [contact_names, contacts] : res.contacts) {
    if (contacts.empty()) {
      continue;
    }

    auto const contact = contacts[0];
    //    if (contact_names.first == COLLISION_BODY_NAME) {
    //      body_idx = 0;
    //    } else if (contact_names.second == COLLISION_BODY_NAME) {
    //      body_idx = 1;
    //    } else {
    //      continue;
    //    }
    //
    //    auto const contact_point = contact.nearest_points[body_idx];
    //    auto const normal = res.contacts.begin()->second.begin()->normal;
    //    return {contact_point, normal};
  }

  return {{0, 0, 0}, {0, 0, 1}};
}

PointsNormals moveit_get_points_normals(planning_scene_monitor::PlanningSceneMonitorPtr const& scene_monitor) {
  // an alternative to manual + CGAL based nearest/normal, we could check check each point on the rope for collision via
  // moveit but moveit only knows how to check for collision betweeen the robot state and the world/itself so I'm not
  // sure how we'd do this

  planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor);
//  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

  Points contact_points(3, 1);
  Normals normals(3, 1);

  // add a point to the moveit world and collision check it
  pcl::PointXYZ point{0, 0, -0.1};
  auto const& [contact_point, normal] = find_nearest_point_and_normal(planning_scene, point);

  ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "contact points: " << contact_points);
  ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "normals: " << normals);
  return {contact_points, normals};
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_moveit_cc");
  auto nh = ros::NodeHandle();
  auto ph = ros::NodeHandle("~");

  std::string robot_namespace{"hdt_michigan"};
  auto scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  auto const scene_topic = ros::names::append(robot_namespace, "move_group/monitored_planning_scene");
  auto const service_name = ros::names::append(robot_namespace, "get_planning_scene");
  scene_monitor->startSceneMonitor(scene_topic);
  scene_monitor->requestPlanningSceneState(service_name);

  auto const points_normals = moveit_get_points_normals(scene_monitor);

  ros::shutdown();
}
