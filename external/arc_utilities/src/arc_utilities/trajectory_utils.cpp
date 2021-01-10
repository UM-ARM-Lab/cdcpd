#include <arc_utilities/trajectory_utils.h>
#include <ros/console.h>

namespace trajectory_utils {

void append(trajectory_msgs::JointTrajectory &trajectory1, trajectory_msgs::JointTrajectory const &trajectory2) {
  // first check they have the same joint names
  if (trajectory1.joint_names.empty()) {
    trajectory1.joint_names = trajectory2.joint_names;
  } else if (not std::equal(trajectory1.joint_names.cbegin(), trajectory1.joint_names.cend(),
                            trajectory2.joint_names.cbegin())) {
    ROS_ERROR_STREAM("Tried to append trajectory but the joint names differ");
    ROS_ERROR_STREAM("joint 1 names:");
    for (auto const &name1 : trajectory1.joint_names) {
      ROS_ERROR_STREAM("\t" << name1);
    }
    ROS_ERROR_STREAM("joint 2 names:");
    for (auto const &name2 : trajectory2.joint_names) {
      ROS_ERROR_STREAM("\t" << name2);
    }
    throw std::runtime_error("Tried to append trajectory but the joint names differ");
  }

  if (trajectory1.header.frame_id.empty()) {
    trajectory1.header.frame_id = trajectory2.header.frame_id;
  }

  auto const last_trajectory1_time = [trajectory1]() {
    if (not trajectory1.points.empty()) {
      return trajectory1.points.back().time_from_start;
    }
    return ros::Duration(0);
  }();
  for (auto const &point2 : trajectory2.points) {
    auto point2_retimed = point2;
    point2_retimed.time_from_start = last_trajectory1_time + point2.time_from_start;
    trajectory1.points.emplace_back(point2_retimed);
  }
}

}  // namespace trajectory_utils