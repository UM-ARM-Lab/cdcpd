#pragma once

#include <trajectory_msgs/JointTrajectory.h>

namespace trajectory_utils {
void append(trajectory_msgs::JointTrajectory &trajectory1, trajectory_msgs::JointTrajectory const &trajectory2);
}
