#pragma once

#include <string>

#include <arc_utilities/enumerate.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/TransformStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
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
#include "cdcpd/corner_candidate_detector.h"
#include "cdcpd_ros/camera_sub.h"
#include "cdcpd/deformable_object_configuration.h"
#include "cdcpd/segmenter.h"

// Separating as I'm not sure I'll keep this around.
#include <pcl/io/ply_io.h>

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
    ros::Publisher bbox_array_pub;
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
    float const length_initial_cloth;
    float const width_initial_cloth;
    float const grid_size_initial_guess_cloth;
    bool const moveit_enabled;
    DeformableObjectType const deformable_object_type;
};

struct CDCPD_Moveit_Node {
public:
    explicit CDCPD_Moveit_Node(std::string const& robot_namespace);

    // void initialize_deformable_object_configuration(Eigen::Vector3f const& rope_start_position,
    //     Eigen::Vector3f const& rope_end_position);
    // Returns initialized deformable object template based on type selected when launching CDCPD
    std::shared_ptr<DeformableObjectConfiguration> initialize_deformable_object_configuration(
        Eigen::Vector3f const& rope_start_position, Eigen::Vector3f const& rope_end_position);


    ObstacleConstraints find_nearest_points_and_normals(planning_scene_monitor::LockedPlanningSceneRW planning_scene,
                                                      Eigen::Isometry3d const& cdcpd_to_moveit);



    std::pair<vm::Marker, vm::Marker> arrow_and_normal(int contact_idx, const Eigen::Vector3d& tracked_point_moveit_frame,
                                                     const Eigen::Vector3d& object_point_moveit_frame,
                                                     const Eigen::Vector3d& object_point_cdcpd_frame,
                                                     const Eigen::Vector3d& normal_cdcpd_frame) const;

    ObstacleConstraints get_moveit_obstacle_constriants(PointCloud::ConstPtr tracked_points);

    // Returns the gripper configuration
    std::tuple<smmap::AllGrippersSinglePose,
               const smmap::AllGrippersSinglePoseDelta> get_q_config();

    // Publishes the bounding box.
    void publish_bboxes() const;

    // Publishes the tracked points of the deformable object.
    // NOTE: Meant to be called before CDCPD runs as CDCPD modifies the tracked points.
    void publish_template() const;

    // Return a vector of ObstacleConstraint objects for the given tracked object.
    ObstacleConstraints get_obstacle_constraints(int const deformable_object_id);

    // Publishes all CDCPD outputs in `cdcpd_outputs_` member.
    void publish_outputs(ros::Time const& t0); //, CDCPD::Output const& out);

    // Main callback for RGB and Depth Mat inputs.
    // void callback(cv::Mat const& rgb, cv::Mat const& depth, cv::Matx33d const& intrinsics);

    // Main callback for point cloud inputs.
    void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg);

    // Resets CDCPD tracking to initial tracking configuration if OutputStatus indicates a problem.
    void reset_if_bad(CDCPD::Output const& out);

    std::string collision_body_prefix{"cdcpd_tracked_point_"};
    std::string robot_namespace_;
    std::string robot_description_;
    ros::NodeHandle nh;
    ros::NodeHandle ph;
    CDCPD_Publishers publishers;
    CDCPD_Node_Parameters const node_params;
    CDCPD_Parameters const cdcpd_params;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
    robot_model_loader::RobotModelLoaderPtr model_loader_;
    robot_model::RobotModelPtr model_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    std::string moveit_frame{"robot_root"};
    double min_distance_threshold{0.01};
    bool moveit_ready{false};

    // TODO(dylan.colli): refactor gripper information into another class.
    YAML::Node grippers_info;
    unsigned int gripper_count;
    Eigen::MatrixXi gripper_indices;

    // Map with configuration ID and configuration object
    std::map<int const, std::shared_ptr<DeformableObjectConfiguration>> deformable_object_configurations_;

    // Map with configuration ID and the CDCPD instance used to track the configuration
    std::map<int const, std::shared_ptr<CDCPD>> cdcpd_instances_;

    std::map<int const, CDCPD::Output> cdcpd_outputs_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Temporarily storing the RGB and Depth images here for easier corner candidate detection
    cv::Mat rgb_img_;
    cv::Mat depth_img_;

protected:
    // Returns a new, unique ID to be associated to the new deformable object that's being tracked
    // and increments the internal variable to produce the next ID to be assigned.
    int get_new_deformable_object_configuration_id(){ return next_deformable_object_id_++; }

    // The deformable object ID for the next object we encounter.
    int next_deformable_object_id_;
};