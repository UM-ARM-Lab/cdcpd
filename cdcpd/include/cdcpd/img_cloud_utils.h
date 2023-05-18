#pragma once

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <tuple>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

using cv::Vec3b;

cv::Mat getHsvMask(cv::Mat const& rgb, double const hue_min=340.0, double const sat_min=0.3,
    double const val_min=0.4, double const hue_max=20.0, double const sat_max=1.0,
    double const val_max=1.0);

// Perform VoxelGrid filter downsampling on an Eigen matrix representing a point cloud.
Eigen::Matrix3Xf downsampleMatrixCloud(Eigen::Matrix3Xf mat_in);

// Runs point cloud through a box filter to mitigate segmentation outliers.
Eigen::Matrix3Xf boxFilterMatrixCloud(const Eigen::Matrix3Xf &mat_in);

PointCloud::Ptr mat_to_cloud(const Eigen::Matrix3Xf &mat);

// NOTE: based on the implementation here:
// https://github.com/ros-perception/image_pipeline/blob/melodic/depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp
// we expect that cx, cy, fx, fy are in the appropriate places in P
std::tuple<PointCloudRGB::Ptr, PointCloud::Ptr> point_clouds_from_images(
    const cv::Mat &depth_image, const cv::Mat &rgb_image, const cv::Mat &mask,
    const Eigen::Matrix3f &intrinsics, const Eigen::Vector3f &lower_bounding_box,
    const Eigen::Vector3f &upper_bounding_box);

// Convert RGB and depth images to un/filtered point cloud matrices.
std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf> point_cloud_mats_from_images(
    const Eigen::MatrixXi &depth_image,
    const Eigen::MatrixXi &red_channel,
    const Eigen::MatrixXi &green_channel,
    const Eigen::MatrixXi &blue_channel,
    const Eigen::MatrixXi &mask,
    const Eigen::Matrix3f &intrinsics, const Eigen::Vector3f &lower_bounding_box,
    const Eigen::Vector3f &upper_bounding_box);
