#include <cdcpd/cdcpd.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <gtest/gtest.h>

#include <iostream>

// TODO(dylan.colli): This was ripped from cdcpd_node.cpp and should be refactored at some point
// soon.
std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int const num_points,
    const Eigen::Vector3f& start_position,
    const Eigen::Vector3f& end_position)
{
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

// TODO(dylan.colli): This was ripped from cdcpd_node.cpp and should be refactored at some point
// soon.
pcl::PointCloud<pcl::PointXYZ>::Ptr makeCloud(Eigen::Matrix3Xf const& points) {
  // TODO: Can we do this cleaner via some sort of data mapping?
  PointCloud::Ptr cloud(new PointCloud);
  for (int i = 0; i < points.cols(); ++i) {
    auto const& c = points.col(i);
    cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
  }
  return cloud;
}

// NOTE: This is refactored code so that I can get the initial tracked points more than once to
// initialize the tracked points that are updated over time.
std::pair<PointCloud::Ptr const&, Eigen::Matrix2Xi const&> getInitialTracking(
    float const max_rope_length, int const num_points)
{
    Eigen::Vector3f start_position{Eigen::Vector3f::Zero()};
    Eigen::Vector3f end_position{Eigen::Vector3f::Zero()};
    start_position << -max_rope_length / 2, 0, 1.0;
    end_position << max_rope_length / 2, 0, 1.0;
    std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> const& initial_template_pair =
        makeRopeTemplate(num_points, start_position, end_position);
    Eigen::Matrix3Xf const& initial_template_vertices = initial_template_pair.first;
    Eigen::Matrix2Xi const& initial_template_edges = initial_template_pair.second;
    PointCloud::Ptr const& initial_tracked_points = makeCloud(initial_template_vertices);
    return {initial_tracked_points, initial_template_edges};
}

CDCPD* initializeCdcpdSimulator(float const max_rope_length, int const num_points)
{
    // Initialize the ROS node handlers?
    // Not necessary for one constructor but that just default constructs the node handlers to:
    //  ros::NodeHandle() and ros::NodeHandle("~").
    // I'm not sure if this is sufficient for testing purposes. I might have to separate the
    // ROS interfaces from the CDCPD math to properly test everything.

    // Likely need to remove these. Just here for debugging.
    // ROS_WARN("Before nh");
    // auto nh = ros::NodeHandle();
    // ROS_WARN("After nh");
    // auto ph = ros::NodeHandle("~");
    // ROS_WARN("After ph");

    // Initialize the template point clouds.
    // TODO(dylan.colli): There's a decent amount of repeated code between this and cdcpd_node.cpp
    // CDCPD constructor. Refactor.

    // TODO(dylan.colli): This is taken from a conditional in cdcpd_node.cpp CDCPD_Moveit_Node
    // constructor. This should be refactored.
    // Eigen::Vector3f start_position{Eigen::Vector3f::Zero()};
    // Eigen::Vector3f end_position{Eigen::Vector3f::Zero()};
    // start_position << -max_rope_length / 2, 0, 1.0;
    // end_position << max_rope_length / 2, 0, 1.0;
    // std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> const& initial_template_pair =
    //     makeRopeTemplate(num_points, start_position, end_position);
    // Eigen::Matrix3Xf const& initial_template_vertices = initial_template_pair.first;
    // Eigen::Matrix2Xi const& initial_template_edges = initial_template_pair.second;
    // PointCloud::Ptr const& initial_tracked_points = makeCloud(initial_template_vertices);
    std::pair<PointCloud::Ptr const&, Eigen::Matrix2Xi const&> initial_tracking = 
        getInitialTracking(max_rope_length, num_points);
    PointCloud::Ptr const& initial_tracked_points = initial_tracking.first;
    Eigen::Matrix2Xi const& initial_template_edges = initial_tracking.second;

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
    // TODO(dylan.colli): Use smart pointer here.
    ROS_WARN("Right before CDCPD object construction");
    CDCPD* cdcpd = new CDCPD(initial_tracked_points, initial_template_edges,
        objective_value_threshold, use_recovery, alpha, beta, lambda, k, zeta, obstacle_cost_weight,
        fixed_points_weight);
    ROS_WARN("Right after CDCPD object construction");

    return cdcpd;
}

TEST(StaticRope, testConvergence)
{
    rosbag::Bag bag;
    // Test executed from ~/.ros, so we have to do some funky path manipulation to get to the bag
    // files
    bag.open("../../../../../src/cdcpd/demos/rosbags/demo_1_static_rope.bag",
        rosbag::bagmode::Read);
    
    // List the topics we want to view
    std::vector<std::string> topics;
    topics.push_back(std::string("/cdcpd/output"));

    // Create the view so that we can iterate through all of the topic messages.
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Read the last CDCPD point cloud message that was output in the ROS bag.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud_last;
    for(rosbag::MessageInstance const m: view)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr i = m.instantiate<pcl::PointCloud<pcl::PointXYZ>>();
        if (i != nullptr)
        {
            pt_cloud_last = i;
        }
        else
        {
            ROS_WARN("No data!");
        }
    }

    // ONLY FOR DEBUGGING: Put that last message into a string to print
    // std::string full_msg;
    // full_msg.append("Printing points: ");
    // std::string pts;
    // for (auto const& point: pt_cloud_last->points)
    // {
    //     // char const * p = reinterpret_cast<char const *>(point.data);
    //     // std::string s(p, p + sizeof point.data);  // beginning + length constructor

    //     // pts.append(s);
    //     pts.append("x: ");
    //     pts.append(std::to_string(point.x));
    //     pts.append("y: ");
    //     pts.append(std::to_string(point.y));
    //     pts.append("z: ");
    //     pts.append(std::to_string(point.z));
    //     pts.append(std::string(", "));
    // }
    // full_msg.append(pts);
    // ROS_WARN("%s", full_msg.c_str());

    // Run CDCPD on rosbag input so as to simulate a full run of CDCPD and test that the
    // functionality has not changed significantly since recording of the bag file.
    // CDCPD cdcpd_sim();
    ROS_WARN("Before cdcpd initialization.");
    float const max_rope_length = 0.46F;  // Taken from kinect_tripodB.launch
    int const num_points = 15;  // Taken from kinect_tripodB.launch
    CDCPD* cdcpd_sim = initializeCdcpdSimulator(max_rope_length, num_points);
    
    
    
    std::pair<PointCloud::Ptr const&, Eigen::Matrix2Xi const&> initial_tracking = 
        getInitialTracking(max_rope_length, num_points);
    pcl::shared_ptr<PointCloud> tracked_points = initial_tracking.first;
    ObstacleConstraints obstacle_constraints;  // No need to specify anything with this demo.
    float const max_segment_length = max_rope_length / static_cast<float>(num_points);
    // TODO(dylan.colli): Do I need to do anything more involved here? I doubt it since there are no
    // grippers specified for this demo.
    unsigned int gripper_count = 0U;
    smmap::AllGrippersSinglePose q_config;
    const smmap::AllGrippersSinglePoseDelta q_dot{gripper_count, kinematics::Vector6d::Zero()};
    Eigen::MatrixXi gripper_indices(1, gripper_count);
    ROS_WARN("After CDCPD initialization");

    // Read the CDCPD input from the rosbag and step through the execution until the end of input is
    // reached.
    // The points_callback lambda function in cdcpd_node is probably what I need to look at for
    // feeding rosbag data into the object.
    std::vector<std::string> input_topics;
    input_topics.push_back(std::string("/cdcpd/original"));
    rosbag::View input_view(bag, rosbag::TopicQuery(input_topics));
    ROS_WARN("After view construction");
    for (rosbag::MessageInstance const m: input_view)
    {
        // Read the message.
        sensor_msgs::PointCloud2ConstPtr points_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (points_msg == nullptr)
        {
            ROS_ERROR("Point cloud pointer is empty!");
        }
        // Do any data transformation on the message to put into form that CDCPD can take as input.
        // TODO(dylan.colli): This mimics some of the functionality in points_callback that should
        // likely be refactored.
        pcl::PCLPointCloud2 points_v2;
        pcl_conversions::toPCL(*points_msg, points_v2);
        auto points = boost::make_shared<PointCloudRGB>();
        pcl::fromPCLPointCloud2(points_v2, *points);

        // Run a single "iteration" of CDCPD mimicing the points_callback lambda function found in
        // cdcpd_node.cpp
        CDCPD::Output out = (*cdcpd_sim)(points, tracked_points, obstacle_constraints,
            max_segment_length, q_dot, q_config, gripper_indices);
        tracked_points = out.gurobi_output;


        // Do a health check of CDCPD?
        if (out.status == OutputStatus::NoPointInFilteredCloud || 
            out.status == OutputStatus::ObjectiveTooHigh)
        {
            ROS_ERROR("CDCPD failed for this iteration. We need to implement cdcpd_node.cpp reset_if_bad lambda function!");
        }
        ROS_WARN("After health check");

    }


    // Read final output of CDCPD.
    // Compare output of test CDCPD to expected CDCPD.
    
    
    
    bag.close();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "StaticRopeTestNode");
    testing::InitGoogleTest(&argc, argv);
//   ros::NodeHandle nh;
    auto nh = ros::NodeHandle();
    auto ph = ros::NodeHandle("~");

//   thread t([]{while(ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();
    return res;
}