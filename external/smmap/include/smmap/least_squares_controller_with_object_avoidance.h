#ifndef LEAST_SQUARES_CONTROLLER_WITH_OBJECT_AVOIDANCE_H
#define LEAST_SQUARES_CONTROLLER_WITH_OBJECT_AVOIDANCE_H

#include "smmap/deformable_controller.h"
#include <smmap_models/jacobian_model.h>

namespace smmap
{
    class LeastSquaresControllerWithObjectAvoidance : public DeformableController
    {
        public:
            LeastSquaresControllerWithObjectAvoidance(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    RobotInterface::Ptr robot,
                    Visualizer::Ptr vis,
                    const JacobianModel::ConstPtr& model,
                    const double obstacle_avoidance_scale,
                    const bool optimize);

        private:
            virtual OutputData getGripperMotion_impl(const InputData& input_data) override final;

            const double obstacle_avoidance_scale_;
            const bool optimize_;
    };
}

#endif // LEAST_SQUARES_CONTROLLER_WITH_OBJECT_AVOIDANCE_H
