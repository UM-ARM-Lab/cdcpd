#ifndef UTILS_H
#define UTILS_H

#include <utility>

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int const num_points,
                                                               const Eigen::Vector3f& start_position,
                                                               const Eigen::Vector3f& end_position);

PointCloud::Ptr makeCloud(Eigen::Matrix3Xf const& points);


#endif