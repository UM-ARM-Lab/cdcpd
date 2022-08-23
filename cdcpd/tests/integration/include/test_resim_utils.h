#pragma once

#include <vector>

#include <boost/shared_ptr.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <gtest/gtest.h>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

// Read the CDCPD input point clouds from the specified bag file.
// These are the point clouds that come from the kinect in the demo and are ultimately used to run
// CDCPD.
std::vector<boost::shared_ptr<PointCloudRGB>> readCdcpdInputPointClouds(rosbag::Bag const& bag);

// Use gtest EXPECT_FLOAT_EQ macro calls to check that each point in the point clouds are equivalent
void expectPointCloudsEqual(pcl::PointCloud<pcl::PointXYZ> const& truth,
    pcl::PointCloud<pcl::PointXYZ> const& test);

// Reads the last CDCPD output message from the specified bag file.
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> readLastCdcpdOutput(rosbag::Bag const& bag);