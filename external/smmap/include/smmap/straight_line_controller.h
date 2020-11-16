#ifndef STRAIGHT_LINE_CONTROLLER_H
#define STRAIGHT_LINE_CONTROLLER_H

#include "smmap/deformable_controller.h"

namespace smmap
{
    class StraightLineController : public DeformableController
    {
        public:
            StraightLineController(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    RobotInterface::Ptr robot,
                    Visualizer::Ptr vis,
                    const DeformableModel::ConstPtr& model);

        private:
            virtual OutputData getGripperMotion_impl(const InputData& input_data) override final;

            std::vector<std::pair<std::vector<double>, std::vector<kinematics::Vector6d>>> static_grippers_motions_;
            std::vector<size_t> current_motion_idx_;
    };
}

#endif // LEAST_SQUARES_CONTROLLER_WITH_OBJECT_AVOIDANCE_H
