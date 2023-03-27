#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <gtest/gtest.h>
#include <ros/package.h>

#include <iostream>

#include "cdcpd/deformable_object_configuration.h"
#include "cdcpd/cdcpd.h"
#include "test_resim_utils.h"

#define PRINT_DEBUG_MESSAGES false

RopeConfiguration getInitialTracking(float const max_rope_length, int const num_points)
{
    Eigen::Vector3f start_position{Eigen::Vector3f::Zero()};
    Eigen::Vector3f end_position{Eigen::Vector3f::Zero()};
    start_position << -max_rope_length / 2, 0, 1.0;
    end_position << max_rope_length / 2, 0, 1.0;

    RopeConfiguration rope_configuration(num_points, max_rope_length, start_position, end_position);
    rope_configuration.initializeTracking();

    return rope_configuration;
}

CDCPD initializeCdcpdSimulator(DeformableObjectTracking const& rope_tracking_initial)
{
    if (PRINT_DEBUG_MESSAGES)
    {
        std::stringstream ss;
        ss << rope_tracking_initial.edges_;
        ROS_WARN("Initial template edges");
        ROS_WARN(ss.str().c_str());
    }

    // Specify constant scalar and boolean parameters
    float const objective_value_threshold = 1.0;
    bool const use_recovery = false;
    double const alpha = 0.5;
    double const beta = 1.0;
    double const lambda = 1.0;
    double const k = 100.0;
    float const zeta = 10.0;
    float const obstacle_cost_weight = 0.001;
    float const fixed_points_weight = 10.0;

    CDCPD cdcpd = CDCPD(rope_tracking_initial.points_, rope_tracking_initial.edges_,
        objective_value_threshold, use_recovery, alpha, beta, lambda, k, zeta, obstacle_cost_weight,
        fixed_points_weight);

    return cdcpd;
}

// Resimulate CDCPD on a set of previously recorded input point clouds in a bag file.
PointCloud::Ptr resimulateCdcpd(CDCPD& cdcpd_sim,
    std::vector<boost::shared_ptr<PointCloudRGB>> const& input_clouds,
    PointCloud::Ptr const& initial_tracked_points, float const max_rope_length,
    int const num_points)
{
    // Do some setup of parameters and gripper configuration.
    PointCloud::Ptr tracked_points = initial_tracked_points;

    ObstacleConstraints obstacle_constraints;  // No need to specify anything with this demo.

    float const max_segment_length = max_rope_length / static_cast<float>(num_points);
    Eigen::RowVectorXd max_segment_lengths =
        Eigen::RowVectorXd::Ones(num_points) * max_segment_length;

    unsigned int gripper_count = 0U;
    smmap::AllGrippersSinglePose q_config;
    const smmap::AllGrippersSinglePoseDelta q_dot{gripper_count, kinematics::Vector6d::Zero()};
    Eigen::MatrixXi gripper_indices(1, gripper_count);

    // Read the CDCPD input from the rosbag and step through the execution until the end of input is
    // reached.
    for (auto & cloud : input_clouds)
    {
        // Run a single "iteration" of CDCPD mimicing the points_callback lambda function found in
        // cdcpd_node.cpp
        CDCPD::Output out = cdcpd_sim(cloud, tracked_points, obstacle_constraints,
            max_segment_lengths, q_dot, q_config, gripper_indices);
        tracked_points = out.gurobi_output;

        // Do a health check of CDCPD
        if (out.status == OutputStatus::NoPointInFilteredCloud ||
            out.status == OutputStatus::ObjectiveTooHigh)
        {
            std::string err_msg = "CDCPD failed for this iteration. We need to implement "
                "cdcpd_node.cpp reset_if_bad lambda function!";
            ROS_ERROR(err_msg.c_str());
        }
    }
    return tracked_points;
}

TEST(StaticRope, testResimPointEquivalency)
{
    // Read in the ros bagfile that we'll be resimulating and checking CDCPD performance against.
    rosbag::Bag bag;
    std::string package_path = ros::package::getPath("cdcpd");
    std::string bag_file_path = package_path + "/../demos/rosbags/demo_1_static_rope.bag";
    bag.open(bag_file_path, rosbag::bagmode::Read);
    auto pt_cloud_last = readLastCdcpdOutput(bag);

    // Run CDCPD on rosbag input so as to simulate a full run of CDCPD and test that the
    // functionality has not changed significantly since recording of the bag file.
    float const max_rope_length = 0.46F;  // Taken from kinect_tripodB.launch
    int const num_points = 15;  // Taken from kinect_tripodB.launch
    RopeConfiguration rope_configuration = getInitialTracking(max_rope_length, num_points);
    CDCPD cdcpd_sim = initializeCdcpdSimulator(rope_configuration.initial_);

    auto input_clouds = readCdcpdInputPointClouds(bag);
    PointCloud::Ptr tracked_points = resimulateCdcpd(cdcpd_sim, input_clouds,
        rope_configuration.initial_.points_, max_rope_length, num_points);

    expectPointCloudsEqual(*pt_cloud_last, *tracked_points);

    bag.close();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "StaticRopeTestNode");
    testing::InitGoogleTest(&argc, argv);
    auto nh = ros::NodeHandle();
    auto ph = ros::NodeHandle("~");

    auto res = RUN_ALL_TESTS();

    ros::shutdown();
    return res;
}