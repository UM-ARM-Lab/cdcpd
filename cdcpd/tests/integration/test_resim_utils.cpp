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
    pcl::PCLPointCloud2 points_v2;
    pcl_conversions::toPCL(*pt_cloud_last_ptr, points_v2);
    pcl::fromPCLPointCloud2(points_v2, *pt_cloud_last);

    return pt_cloud_last;
}