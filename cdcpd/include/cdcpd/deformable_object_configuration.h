#pragma once

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct DeformableObjectTracking
{
    DeformableObjectTracking(){}

    // Turns the passed in Matrix of XYZ points to a point cloud.
    void makeCloud(Eigen::Matrix3Xf const& points);

    // Copy constructor.
    DeformableObjectTracking(DeformableObjectTracking const& other);

    Eigen::Matrix3Xf vertices{};
    Eigen::Matrix2Xi edges{};
    PointCloud::Ptr points;
};

class DeformableObjectConfiguration
{
public:
    DeformableObjectConfiguration(int const num_points_);

    // Forms the deformable object tracking template. This needs to be overriden by derived classes
    // since this is dependent upon the type of deformable object we're tracking.
    DeformableObjectTracking virtual makeTemplate(Eigen::Vector3f const& start_position,
        Eigen::Vector3f const& end_position) = 0;

    virtual ~DeformableObjectConfiguration(){}

    void initializeTracking(Eigen::Vector3f const& start_position,
        Eigen::Vector3f const& end_position);

    int const num_points;
    DeformableObjectTracking tracked;
    DeformableObjectTracking initial;
};

class RopeConfiguration : public DeformableObjectConfiguration
{
public:
    RopeConfiguration(int const num_points_, float const max_rope_length_);
    DeformableObjectTracking virtual makeTemplate(Eigen::Vector3f const& start_position,
        Eigen::Vector3f const& end_position);

    float const max_rope_length;
    float const max_segment_length;
};


// TODO(dylan.colli): Make a derived class for cloth configuration.
// class ClothConfiguration : public DeformableObjectConfiguration
// {

// };