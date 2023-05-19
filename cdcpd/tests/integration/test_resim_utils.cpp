#include "test_resim_utils.h"

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

void expectPointCloudsEqual(pcl::PointCloud<pcl::PointXYZ> const& truth,
    pcl::PointCloud<pcl::PointXYZ> const& test)
{
    // Test same number of points.
    EXPECT_EQ(truth.size(), test.size());

    // Test individual points.
    for (int idx = 0; idx < truth.size(); ++idx)
    {
        auto const& truth_pt = truth.points[idx];
        auto const& test_pt = test.points[idx];

        EXPECT_FLOAT_EQ(truth_pt.x, test_pt.x);
        EXPECT_FLOAT_EQ(truth_pt.y, test_pt.y);
        EXPECT_FLOAT_EQ(truth_pt.z, test_pt.z);

        // Uncomment this if you want high resolution info printed.
        // auto x_err = std::pow(truth_pt.x - test_pt.x, 2.0);
        // auto y_err = std::pow(truth_pt.y - test_pt.y, 2.0);
        // auto z_err = std::pow(truth_pt.z - test_pt.z, 2.0);

        // auto pt_err = std::pow(x_err + y_err + z_err, 0.5);

        // ROS_WARN_STREAM("Truth point: " << truth_pt.x << ", " << truth_pt.y << ", " << truth_pt.z);
        // ROS_WARN_STREAM("Test point: " << test_pt.x << ", " << test_pt.y << ", " << test_pt.z);
        // ROS_WARN_STREAM("Pointwise error: " << pt_err);
    }
}

// Reads the last CDCPD output message from the specified bag file.
std::vector<boost::shared_ptr<PointCloud> > readCdcpdOutput(rosbag::Bag const& bag)
{
    // List the topics we want to view
    std::vector<std::string> topics;
    topics.push_back(std::string("/cdcpd/output"));

    // Create the view so that we can iterate through all of the topic messages.
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Read the CDCPD output (template vertices).
    std::vector<boost::shared_ptr<PointCloud> > output_clouds;
    sensor_msgs::PointCloud2Ptr pt_cloud_last_ptr;
    for(rosbag::MessageInstance const m: view)
    {
        auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (points_msg == nullptr)
        {
            ROS_ERROR("Point cloud pointer is empty!");
        }

        // Do any data transformation on the message to put into form that we can compare with new
        // CDCPD output.
        // TODO(dylan.colli): This mimics some of the functionality in points_callback that should
        // likely be refactored.
        auto points_output = boost::make_shared<PointCloud>();
        {
            pcl::PCLPointCloud2 points_v2;
            pcl_conversions::toPCL(*points_msg, points_v2);
            pcl::fromPCLPointCloud2(points_v2, *points_output);
        }
        output_clouds.push_back(points_output);
    }

    return output_clouds;
}