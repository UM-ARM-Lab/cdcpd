//
// Created by kunhuang on 6/3/19.
//

#pragma once

#include <moveit/version.h>
#include <Eigen/Geometry>
#include <vector>

#define MOVEIT_VERSION_AT_LEAST(x, y, z) \
  (MOVEIT_VERSION_MAJOR > x ||           \
   (MOVEIT_VERSION_MAJOR >= x &&         \
    (MOVEIT_VERSION_MINOR > y || (MOVEIT_VERSION_MINOR >= y && MOVEIT_VERSION_PATCH >= z))))

#if MOVEIT_VERSION_AT_LEAST(1, 0, 1)
using Pose = Eigen::Isometry3d;
#else
using Pose = Eigen::Affine3d;
#endif

using PoseSequence = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
using PointSequence = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
