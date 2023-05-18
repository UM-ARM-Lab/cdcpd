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

#include "cdcpd/img_cloud_utils.h"

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZHSV PointHSV;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointHSV> PointCloudHSV;

inline Eigen::Vector3f const bounding_box_extend(0.1, 0.2, 0.1);
std::string const LOGNAME_SEGMENTER = "cdcpd_segmenter";

// The base class that all Segmenter classes should inherit from. This defines the functions with
// which the user will interact with the Segmenter.
class Segmenter
{
public:
    Segmenter();
    virtual ~Segmenter(){}

    virtual void set_input_cloud(const PointCloudRGB::Ptr input_cloud) { input_cloud_ = input_cloud; }

    virtual void set_input_cloud_from_matrices(const Eigen::Matrix3Xf &xyz,
        const Eigen::Matrix3Xi &rgb);

    // Return a copy of the segmented point cloud. This prevents accidentally making changes to the
    // output of the segmentation.
    virtual PointCloud::Ptr get_segmented_cloud();

    virtual Eigen::Matrix3Xf get_segmented_cloud_matrix();

    // Do the segmentation and store output as member variables of this class.
    virtual void segment() = 0;

protected:
    // The cloud to be segmented.
    PointCloudRGB::Ptr input_cloud_;

    // The segmented point cloud.
    PointCloud::Ptr segmented_points_;
};

class SegmenterHSV : public Segmenter
{
public:

    struct SegmenterParameters
    {
    public:
        SegmenterParameters();
        // Opted to remove this as a means to decouple segmentation from ROS. This obviously means
        // that we can't get the parameters from the parameter server now. Need to find a good
        // compromise for this. Peter and I have discussed rewriting the CDCPD node in Python for a
        // better experience working with ROS.
        // SegmenterParameters(ros::NodeHandle& ph);

        float const hue_min;
        float const hue_max;
        float const sat_min;
        float const sat_max;
        float const val_min;
        float const val_max;
    };

    SegmenterHSV(Eigen::Vector3f const last_lower_bounding_box,
        Eigen::Vector3f const last_upper_bounding_box);

    virtual void segment();

    void set_last_lower_bounding_box(Eigen::Vector3f const& last_lower_bounding_box);

    void set_last_upper_bounding_box(Eigen::Vector3f const& last_upper_bounding_box);

protected:

    Eigen::Vector3f last_lower_bounding_box_;
    Eigen::Vector3f last_upper_bounding_box_;
    SegmenterParameters const params_;
};