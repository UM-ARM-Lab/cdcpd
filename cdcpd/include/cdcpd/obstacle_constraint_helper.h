#pragma once

#include <string>

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <arc_utilities/eigen_ros_conversions.hpp>
#include <arc_utilities/enumerate.h>
#include <tf2_ros/transform_broadcaster.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

#include "cdcpd/optimizer.h"
#include "cdcpd/sdformat_to_planning_scene.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace gm = geometry_msgs;
namespace vm = visualization_msgs;
constexpr auto const MAX_CONTACTS_VIZ = 25;
constexpr auto const PERF_LOGGER = "perf";
std::string const LOGNAME = "cdcpd_node";

class ObstacleConstraintHelper
{
public:
    // Could pass in a reference to parameter structs here which would narrow down argument number
    ObstacleConstraintHelper(bool moveit_enabled, ros::Publisher const& contact_marker_pub,
        ros::Publisher const& scene_pub, std::string const camera_frame,
        std::string const sdf_filename, double const min_distance_threshold);

    // Finds the nearest points and normals of all tracked points relative to the planning scene
    // pointed to by the planning_scene_ member variable.
    ObstacleConstraints find_nearest_points_and_normals();

    std::pair<vm::Marker, vm::Marker> arrow_and_normal(int contact_idx,
        const Eigen::Vector3d& tracked_point_cdcpd_frame,
        const Eigen::Vector3d& object_point_cdcpd_frame,
        const Eigen::Vector3d& normal_cdcpd_frame) const;

    ObstacleConstraints get_moveit_obstacle_constraints(PointCloud::ConstPtr tracked_points);

    // Returns a vector of ObstacleConstraint objects.
    ObstacleConstraints get_obstacle_constraints(pcl::shared_ptr<PointCloud> tracked_points);

    std::string sdf_filename_;
    planning_scene::PlanningScenePtr planning_scene_;
    moveit_msgs::PlanningScene planning_scene_msg_;
    std::string collision_body_prefix{"cdcpd_tracked_point_"};
    double min_distance_threshold_;

    // May refactor these out. If not, then just keep them as pointers and use constructor to make
    // a shared pointer from the CDCPD_Parameter struct for each publisher.
    std::shared_ptr<ros::Publisher> contact_marker_pub_;
    std::shared_ptr<ros::Publisher> scene_pub_;
    std::string const camera_frame_;
    tf2_ros::TransformBroadcaster br_;
    bool const moveit_enabled_;

private:
    // Loads the SDFormat file, given by sdf_filename_ member, into the planning_scene_ member.
    void load_sdformat_file();
};