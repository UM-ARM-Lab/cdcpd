#pragma once

#include <string>

#include <Eigen/Dense>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <ros/ros.h>

#include <arc_utilities/ros_helpers.hpp>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZHSV PointHSV;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointHSV> PointCloudHSV;

inline Eigen::Vector3f const bounding_box_extend(0.1, 0.2, 0.1);
std::string const LOGNAME_SEGMENTER = "cdcpd_segmenter";

// The base class that all Segmenter classes should inherit from. This defines the functions with
// which the user will interact with the Segmenter.
// Template argument here is for the type of the segmented point cloud.
template<typename T>
class Segmenter
{
public:
    Segmenter(){}
    virtual ~Segmenter(){}

    // Return a copy of the segmented point cloud. This prevents accidentally making changes to the
    // output of the segmentation.
    virtual T get_segmented_cloud() const = 0;

    // Do the segmentation and store output as member variables of this class.
    virtual void segment(const PointCloudRGB::Ptr & inputs_points) = 0;

protected:
    // The segmented point cloud.
    // Adding const here ensures that the segmented point cloud is not changed downstream of
    // segmentation, preventing unexpected behavior if the user modifies the segmented point cloud.
    std::unique_ptr<const T> segmented_points_;
};

class SegmenterHSV : public Segmenter<PointCloudHSV>
{
public:

    struct SegmenterParameters
    {
    public:
        SegmenterParameters(ros::NodeHandle& ph);

        float const hue_min;
        float const hue_max;
        float const sat_min;
        float const sat_max;
        float const val_min;
        float const val_max;
    };

    SegmenterHSV(ros::NodeHandle& ph, Eigen::Vector3f const last_lower_bounding_box,
        Eigen::Vector3f const last_upper_bounding_box);

    virtual PointCloudHSV get_segmented_cloud() const override;

    virtual void segment(const PointCloudRGB::Ptr & input_points);

    void set_last_lower_bounding_box(Eigen::Vector3f const& last_lower_bounding_box);

    void set_last_upper_bounding_box(Eigen::Vector3f const& last_upper_bounding_box);

protected:

    Eigen::Vector3f last_lower_bounding_box_;
    Eigen::Vector3f last_upper_bounding_box_;
    SegmenterParameters const params_;
};