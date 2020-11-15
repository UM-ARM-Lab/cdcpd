#ifndef SMMAP_CONVERSIONS_H
#define SMMAP_CONVERSIONS_H

#include <deformable_manipulation_msgs/TransitionTest.h>
#include <deformable_manipulation_msgs/TransitionTestResult.h>

#include "smmap/band_rrt.h"
#include "smmap/transition_estimation.h"

namespace smmap
{
    std::pair<TransitionEstimation::StateTrajectory, bool> ToTrajectory(
            const WorldState& initial_world_state,
            const RRTPath& path,
            const deformable_manipulation_msgs::TransitionTest& test,
            const deformable_manipulation_msgs::TransitionTestResult& test_result);

    std::pair<TransitionEstimation::StateTrajectory, bool> ToTrajectory(
            const WorldState& initial_world_state,
            const RRTPath& waypoints,
            const deformable_manipulation_msgs::RobotPathTestResult& test_result,
            const std::vector<size_t>& test_waypoint_indices);
}

#endif
