#include "smmap/straight_line_controller.h"

using namespace smmap;

StraightLineController::StraightLineController(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        RobotInterface::Ptr robot,
        Visualizer::Ptr vis,
        const DeformableModel::ConstPtr& model)
    : DeformableController(nh, ph, robot, vis, model)
{
    const auto grippers = robot_->getGrippersData();
    const size_t num_grippers = grippers.size();

    std::cout << std::endl;

    static_grippers_motions_.reserve(num_grippers);
    for (size_t idx = 0; idx < num_grippers; ++idx)
    {
        const std::string& gripper_name = grippers[idx].name_;
        static_grippers_motions_.push_back(GetGripperDeltaTrajectory(*ph_, gripper_name));
    }

    current_motion_idx_.clear();
    current_motion_idx_.resize(num_grippers, 0);
}

DeformableController::OutputData StraightLineController::getGripperMotion_impl(
        const InputData& input_data)
{
    const size_t num_grippers = static_grippers_motions_.size();

    AllGrippersSinglePoseDelta cmd(num_grippers, kinematics::Vector6d::Zero());
    for (size_t gripper_idx = 0; gripper_idx < num_grippers; ++gripper_idx)
    {
        const auto& current_gripper_motions = static_grippers_motions_[gripper_idx];
        const std::vector<double>& timing_points = current_gripper_motions.first;
        const std::vector<kinematics::Vector6d>& gripper_deltas = current_gripper_motions.second;

        // Check if we should move on to the next motion
        size_t& motion_idx = current_motion_idx_[gripper_idx];
        if (motion_idx + 1 < timing_points.size())
        {
            if (input_data.world_current_state_.sim_time_ >= timing_points[motion_idx + 1])
            {
                motion_idx++;
            }
        }

        cmd[gripper_idx] = gripper_deltas[motion_idx];
    }

    Eigen::VectorXd robot_motion;
    if (input_data.world_current_state_.robot_configuration_valid_)
    {
        robot_motion = robot_->mapGripperMotionToRobotMotion(cmd);
    }

    const auto prediction = model_->getObjectDelta(input_data.world_current_state_, cmd);
    return OutputData(cmd, prediction, robot_motion);
}
