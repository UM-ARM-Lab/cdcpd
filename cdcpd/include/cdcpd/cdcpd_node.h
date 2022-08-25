#pragma once

#include <string>

#include <arc_utilities/enumerate.h>
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
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include "cdcpd/cdcpd.h"
#include "cdcpd_ros/camera_sub.h"
#include "cdcpd/utils.h"
#include "cdcpd/deformable_object_configuration.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace gm = geometry_msgs;
namespace vm = visualization_msgs;
namespace ehc = EigenHelpersConversions;

struct CdcpdPublishers
{
    CdcpdPublishers(ros::NodeHandle& nh, ros::NodeHandle& ph);

    ros::Publisher original_publisher;
    ros::Publisher masked_publisher;
    ros::Publisher downsampled_publisher;
    ros::Publisher template_publisher;
    ros::Publisher pre_template_publisher;
    ros::Publisher output_publisher;
    ros::Publisher order_pub;
    ros::Publisher contact_marker_pub;
    ros::Publisher bbox_pub;
};

struct CdcpdNodeParameters
{
    CdcpdNodeParameters(ros::NodeHandle& nh, ros::NodeHandle& ph);

    // TODO: describe each parameter in words (and a pointer to an equation/section of paper)
    // point cloud input takes precedence. If points_name is not empty, we will use that and not RGB + Depth
    std::string const points_name;
    std::string const rgb_topic;
    std::string const depth_topic;
    std::string const info_topic;
    std::string const camera_frame;
    std::string const grippers_info_filename;
    int const num_points;
    float const max_rope_length;
    bool const moveit_enabled;
};

struct CDCPD_Moveit_Node {
public:
    explicit CDCPD_Moveit_Node(std::string const& robot_namespace);

    ObstacleConstraints find_nearest_points_and_normals(planning_scene_monitor::LockedPlanningSceneRW planning_scene,
                                                      Eigen::Isometry3d const& cdcpd_to_moveit);



    std::pair<vm::Marker, vm::Marker> arrow_and_normal(int contact_idx, const Eigen::Vector3d& tracked_point_moveit_frame,
                                                     const Eigen::Vector3d& object_point_moveit_frame,
                                                     const Eigen::Vector3d& object_point_cdcpd_frame,
                                                     const Eigen::Vector3d& normal_cdcpd_frame) const;

    ObstacleConstraints get_moveit_obstacle_constriants(PointCloud::ConstPtr tracked_points);

    // Previous lambda functions.
    // TODO(dylan.colli): Make as many of these const methods as possible.
    std::tuple<smmap::AllGrippersSinglePose,
               const smmap::AllGrippersSinglePoseDelta> get_q_config();
    void publish_bbox();
    void publish_template();
    ObstacleConstraints get_obstacle_constraints();
    void publish_outputs(ros::Time const& t0, CDCPD::Output const& out);
    void callback(cv::Mat const& rgb, cv::Mat const& depth, cv::Matx33d const& intrinsics);
    void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg);
    void reset_if_bad(CDCPD::Output const& out);



    std::string collision_body_prefix{"cdcpd_tracked_point_"};
    std::string robot_namespace_;
    std::string robot_description_;
    ros::NodeHandle nh;
    ros::NodeHandle ph;
    CdcpdPublishers publishers;
    CdcpdNodeParameters const node_params;
    CdcpdParameters const cdcpd_params;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
    robot_model_loader::RobotModelLoaderPtr model_loader_;
    robot_model::RobotModelPtr model_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    std::string moveit_frame{"robot_root"};
    double min_distance_threshold{0.01};
    bool moveit_ready{false};

    // Dylan added to get previous lambda functions working.
    std::unique_ptr<CDCPD> cdcpd;
    // PointCloud::Ptr initial_tracked_points;
    // Eigen::Matrix2Xi initial_template_edges;
    // PointCloud::Ptr tracked_points;
    float const max_segment_length;
    // TODO(dylan.colli): refactor gripper information into another class.
    YAML::Node grippers_info;
    unsigned int gripper_count;
    Eigen::MatrixXi gripper_indices;

    RopeConfiguration rope_configuration;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};