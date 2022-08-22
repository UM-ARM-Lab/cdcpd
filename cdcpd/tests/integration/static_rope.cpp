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
  {
      ROS_WARN("Template edges:");
      std::stringstream ss;
      ss << template_edges;
      ROS_WARN(ss.str().c_str());
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
std::pair<PointCloud::Ptr, Eigen::Matrix2Xi const> getInitialTracking(
    float const max_rope_length, int const num_points)
{
    Eigen::Vector3f start_position{Eigen::Vector3f::Zero()};
    Eigen::Vector3f end_position{Eigen::Vector3f::Zero()};
    start_position << -max_rope_length / 2, 0, 1.0;
    end_position << max_rope_length / 2, 0, 1.0;
    std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> const& initial_template_pair =
        makeRopeTemplate(num_points, start_position, end_position);
    Eigen::Matrix3Xf const& initial_template_vertices = initial_template_pair.first;
    Eigen::Matrix2Xi const initial_template_edges = initial_template_pair.second;
    pcl::PointCloud<pcl::PointXYZ>::Ptr initial_tracked_points = makeCloud(initial_template_vertices);
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
    std::pair<PointCloud::Ptr, Eigen::Matrix2Xi const> initial_tracking = 
        getInitialTracking(max_rope_length, num_points);
    PointCloud::Ptr const& initial_tracked_points = initial_tracking.first;
    Eigen::Matrix2Xi const& initial_template_edges = initial_tracking.second;
    
    {
        std::stringstream ss;
        ss << initial_template_edges;
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
    // TODO(dylan.colli): Use smart pointer here.
    ROS_WARN("Right before CDCPD object construction");
    // CDCPD cdcpd = CDCPD(initial_tracked_points, initial_template_edges,
    //     objective_value_threshold, use_recovery, alpha, beta, lambda, k, zeta, obstacle_cost_weight,
    //     fixed_points_weight);
    // ROS_WARN("Right after initial constrution");
    CDCPD* cdcpd_ptr = new CDCPD(initial_tracked_points, initial_template_edges,
        objective_value_threshold, use_recovery, alpha, beta, lambda, k, zeta, obstacle_cost_weight,
        fixed_points_weight);
    ROS_WARN("Right after CDCPD object construction");

    return cdcpd_ptr;
    // return cdcpd;
}

void expectPointCloudsEqual(pcl::PointCloud<pcl::PointXYZ> const& truth,
    pcl::PointCloud<pcl::PointXYZ> const& test)
{
    // Test same number of points.
    EXPECT_EQ(truth.size(), test.size());

    // Test individual points.
    for (int idx = 0; idx < truth.size(); ++idx)
    {
        EXPECT_FLOAT_EQ(truth.points[idx].x, test.points[idx].x);
        EXPECT_FLOAT_EQ(truth.points[idx].y, test.points[idx].y);
        EXPECT_FLOAT_EQ(truth.points[idx].z, test.points[idx].z);
    }
}

// Reads the last CDCPD output message from the specified bag file.
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> readLastCdcpdOutput(rosbag::Bag const& bag)
{
    // List the topics we want to view
    std::vector<std::string> topics;
    topics.push_back(std::string("/cdcpd/output"));

    // Create the view so that we can iterate through all of the topic messages.
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Read the last CDCPD point cloud message that was output in the ROS bag.
    sensor_msgs::PointCloud2Ptr pt_cloud_last_ptr;
    for(rosbag::MessageInstance const m: view)
    {
        auto i = m.instantiate<sensor_msgs::PointCloud2>();
        if (i != nullptr)
        {
            pt_cloud_last_ptr = i;
        }
        else
        {
            ROS_WARN("No data!");
        }
    }
    // Convert the last point cloud message to a usable point cloud type.
    auto pt_cloud_last = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    {
        pcl::PCLPointCloud2 points_v2;
        pcl_conversions::toPCL(*pt_cloud_last_ptr, points_v2);
        pcl::fromPCLPointCloud2(points_v2, *pt_cloud_last);
    }
    {
        std::stringstream ss_2;
        for (int nIndex = 0; nIndex < pt_cloud_last->points.size (); nIndex++)
        {
            float x = pt_cloud_last->points[nIndex].x;
            float y = pt_cloud_last->points[nIndex].y;
            float z = pt_cloud_last->points[nIndex].z;
            ss_2 << "(" << x << ", " << y << ", " << z << "), ";
        }
        ROS_WARN("Read pt_cloud_last points:");
        ROS_WARN(ss_2.str().c_str());
    }
    return pt_cloud_last;
}

// Read the CDCPD input point clouds from the specified bag file.
// These are the point clouds that come from the kinect in the demo and are ultimately used to run
// CDCPD.
std::vector<boost::shared_ptr<PointCloudRGB>> readCdcpdInputPointClouds(rosbag::Bag const& bag)
{
    // Create the view for the input point clouds.
    std::vector<std::string> input_topics;
    input_topics.push_back(std::string("/cdcpd/original"));
    rosbag::View input_view(bag, rosbag::TopicQuery(input_topics));

    // Iterate through the messages and store in the vector.
    std::vector<boost::shared_ptr<PointCloudRGB>> input_clouds;
    for (rosbag::MessageInstance const m: input_view)
    {
        // Read the message.
        auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (points_msg == nullptr)
        {
            ROS_ERROR("Point cloud pointer is empty!");
        }
        
        // Do any data transformation on the message to put into form that CDCPD can take as input.
        // TODO(dylan.colli): This mimics some of the functionality in points_callback that should
        // likely be refactored.
        auto points_input = boost::make_shared<PointCloudRGB>();
        {
            pcl::PCLPointCloud2 points_v2;
            pcl_conversions::toPCL(*points_msg, points_v2);   
            pcl::fromPCLPointCloud2(points_v2, *points_input);
        }
        input_clouds.push_back(points_input);
    }
    return input_clouds;
}

PointCloud::Ptr resimulateCdcpd(CDCPD& cdcpd_sim, std::vector<boost::shared_ptr<PointCloudRGB>> const& input_clouds, float const max_rope_length, int const num_points)
{
    std::pair<PointCloud::Ptr, Eigen::Matrix2Xi const> initial_tracking = 
        getInitialTracking(max_rope_length, num_points);
    PointCloud::Ptr tracked_points = initial_tracking.first;
    ObstacleConstraints obstacle_constraints;  // No need to specify anything with this demo.
    float const max_segment_length = max_rope_length / static_cast<float>(num_points);
    unsigned int gripper_count = 0U;
    smmap::AllGrippersSinglePose q_config;
    const smmap::AllGrippersSinglePoseDelta q_dot{gripper_count, kinematics::Vector6d::Zero()};
    Eigen::MatrixXi gripper_indices(1, gripper_count);
    ROS_WARN("After CDCPD initialization");

    // Read the CDCPD input from the rosbag and step through the execution until the end of input is
    // reached.
    // The points_callback lambda function in cdcpd_node is probably what I need to look at for
    // feeding rosbag data into the object.
    for (auto & cloud : input_clouds)
    {
        // Run a single "iteration" of CDCPD mimicing the points_callback lambda function found in
        // cdcpd_node.cpp
        CDCPD::Output out = cdcpd_sim(cloud, tracked_points, obstacle_constraints,
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
    return tracked_points;
}

TEST(StaticRope, testConvergence)
{
    rosbag::Bag bag;
    // Test executed from ~/.ros, so we have to do some funky path manipulation to get to the bag
    // files
    bag.open("../../../../../src/cdcpd/demos/rosbags/demo_1_static_rope.bag",
        rosbag::bagmode::Read);
    auto pt_cloud_last = readLastCdcpdOutput(bag);

    // Run CDCPD on rosbag input so as to simulate a full run of CDCPD and test that the
    // functionality has not changed significantly since recording of the bag file.
    // CDCPD cdcpd_sim();
    ROS_WARN("Before cdcpd initialization.");
    float const max_rope_length = 0.46F;  // Taken from kinect_tripodB.launch
    int const num_points = 15;  // Taken from kinect_tripodB.launch
    CDCPD* cdcpd_sim = initializeCdcpdSimulator(max_rope_length, num_points);
    ROS_WARN("After cdcpd_sim object initialization");

    auto input_clouds = readCdcpdInputPointClouds(bag);
    PointCloud::Ptr tracked_points = resimulateCdcpd(*cdcpd_sim, input_clouds, max_rope_length, num_points);

    expectPointCloudsEqual(*pt_cloud_last, *tracked_points);
    
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