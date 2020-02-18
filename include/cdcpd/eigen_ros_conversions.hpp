#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#ifndef LBV_EIGEN_ROS_CONVERSIONS_HPP
#define LBV_EIGEN_ROS_CONVERSIONS_HPP

////////////////////////////////////////////////////////////////////////////////

template<typename Output>
Output ConvertTo(geometry_msgs::Pose const& pose);

template<typename Output>
Output ConvertTo(Eigen::Affine3d const& transform);

template<typename Output>
Output ConvertTo(Eigen::Isometry3d const& transform);

template<>
inline Eigen::Isometry3d ConvertTo<Eigen::Isometry3d>(geometry_msgs::Pose const& pose)
{
    Eigen::Translation3d const trans(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond const quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    return trans * quat;
}

template<>
inline Eigen::Affine3d ConvertTo<Eigen::Affine3d>(geometry_msgs::Pose const& pose)
{
    Eigen::Translation3d const trans(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond const quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    return trans * quat;
}

template<>
inline geometry_msgs::Transform ConvertTo<geometry_msgs::Transform>(Eigen::Affine3d const& transform)
{
    geometry_msgs::Transform geo_transform;
    geo_transform.translation.x = transform.translation().x();
    geo_transform.translation.y = transform.translation().y();
    geo_transform.translation.z = transform.translation().z();
    const Eigen::Quaterniond quat(transform.rotation());
    geo_transform.rotation.x = quat.x();
    geo_transform.rotation.y = quat.y();
    geo_transform.rotation.z = quat.z();
    geo_transform.rotation.w = quat.w();
    return geo_transform;
}

template<>
inline geometry_msgs::Transform ConvertTo<geometry_msgs::Transform>(Eigen::Isometry3d const& transform)
{
    geometry_msgs::Transform geo_transform;
    geo_transform.translation.x = transform.translation().x();
    geo_transform.translation.y = transform.translation().y();
    geo_transform.translation.z = transform.translation().z();
    const Eigen::Quaterniond quat(transform.rotation());
    geo_transform.rotation.x = quat.x();
    geo_transform.rotation.y = quat.y();
    geo_transform.rotation.z = quat.z();
    geo_transform.rotation.w = quat.w();
    return geo_transform;
}

////////////////////////////////////////////////////////////////////////////////

template<typename Output>
Output ConvertTo(geometry_msgs::Transform const& transform);

template<>
inline Eigen::Isometry3d ConvertTo<Eigen::Isometry3d>(geometry_msgs::Transform const& transform)
{
    Eigen::Translation3d const trans(transform.translation.x, transform.translation.y, transform.translation.z);
    Eigen::Quaterniond const quat(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
    return trans * quat;
}

template<>
inline Eigen::Affine3d ConvertTo<Eigen::Affine3d>(geometry_msgs::Transform const& transform)
{
    Eigen::Translation3d const trans(transform.translation.x, transform.translation.y, transform.translation.z);
    Eigen::Quaterniond const quat(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
    return trans * quat;
}

#endif // LBV_EIGEN_ROS_CONVERSIONS_HPP