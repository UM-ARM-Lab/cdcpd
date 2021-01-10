#ifndef LBV_EIGEN_TRANSFORMS_HPP
#define LBV_EIGEN_TRANSFORMS_HPP

#include <arc_utilities/eigen_typedefs.hpp>
#include <arc_utilities/moveit_pose_type.hpp>

template <typename T1, typename T2>
std::pair<T1, T2> Transform(Pose const& transform, std::pair<T1, T2> const& input) {
  return {transform * input.first, transform * input.second};
}

std::pair<Pose, Pose> Transform(Pose const& transform,
                                std::pair<Eigen::Translation3d, Eigen::Translation3d> const& input) {
  return {transform * input.first, transform * input.second};
}

template <typename T>
std::vector<T, Eigen::aligned_allocator<T>> Transform(Pose const& transform,
                                                      std::vector<T, Eigen::aligned_allocator<T>> const& input) {
  std::vector<T, Eigen::aligned_allocator<T>> output(input.size());
  for (auto idx = 0ul; idx < input.size(); ++idx) {
    output[idx] = transform * input[idx];
  }
  return output;
}

#endif
