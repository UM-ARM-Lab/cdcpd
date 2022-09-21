#pragma once

#include <string>

#include <geometric_shapes/shapes.h>
#include <geometry_msgs/TransformStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <opencv2/imgproc/types_c.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include "cdcpd/cdcpd.h"
#include "cdcpd/deformable_object_configuration.h"
#include "cdcpd/obstacle_constraint_helper.h"
#include "cdcpd/sdformat_to_planning_scene.h"
#include "cdcpd_ros/camera_sub.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace gm = geometry_msgs;
namespace vm = visualization_msgs;
namespace ehc = EigenHelpersConversions;

struct CDCPD_Publishers
{
    CDCPD_Publishers(ros::NodeHandle& nh, ros::NodeHandle& ph);

    ros::Publisher original_publisher;
    ros::Publisher masked_publisher;
    ros::Publisher downsampled_publisher;
    ros::Publisher template_publisher;
    ros::Publisher pre_template_publisher;
    ros::Publisher output_publisher;
    ros::Publisher order_pub;
    ros::Publisher contact_marker_pub;
    ros::Publisher bbox_pub;
    ros::Publisher scene_pub;
};

struct CDCPD_Node_Parameters
{
    CDCPD_Node_Parameters(ros::NodeHandle& nh, ros::NodeHandle& ph);

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
    std::string const sdf_filename;
};

struct CDCPD_Moveit_Node {
public:
    explicit CDCPD_Moveit_Node(std::string const& robot_namespace);

    // Returns the gripper configuration
    std::tuple<smmap::AllGrippersSinglePose,
               const smmap::AllGrippersSinglePoseDelta> get_q_config();

    // Publishes the bounding box.
    void publish_bbox() const;

    // Publishes the tracked points of the deformable object.
    // NOTE: Meant to be called before CDCPD runs as CDCPD modifies the tracked points.
    void publish_template() const;

    // Publishes the CDCPD outputs.
    void publish_outputs(ros::Time const& t0, CDCPD::Output const& out);

    // Main callback for RGB and Depth Mat inputs.
    void callback(cv::Mat const& rgb, cv::Mat const& depth, cv::Matx33d const& intrinsics);

    // Main callback for point cloud inputs.
    void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg);

    // Resets CDCPD tracking to initial tracking configuration if OutputStatus indicates a problem.
    void reset_if_bad(CDCPD::Output const& out);

    std::string collision_body_prefix{"cdcpd_tracked_point_"};
    ros::NodeHandle nh;
    ros::NodeHandle ph;
    CDCPD_Publishers publishers;
    CDCPD_Node_Parameters const node_params;
    CDCPD_Parameters const cdcpd_params;
    std::unique_ptr<CDCPD> cdcpd;

    // TODO(dylan.colli): refactor gripper information into another class.
    YAML::Node grippers_info;
    unsigned int gripper_count;
    Eigen::MatrixXi gripper_indices;

    RopeConfiguration rope_configuration;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ObstacleConstraintHelper obstacle_constraint_helper_;
};