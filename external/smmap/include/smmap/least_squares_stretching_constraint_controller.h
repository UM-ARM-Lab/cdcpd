#ifndef LEAST_SQUARES_CONTROLLER_WITH_STRETCHING_CONSTRAINT_H
#define LEAST_SQUARES_CONTROLLER_WITH_STRETCHING_CONSTRAINT_H

#include "smmap/deformable_controller.h"
#include <smmap_models/jacobian_model.h>

namespace smmap
{
    class LeastSquaresControllerWithStretchingConstraint : public DeformableController
    {
        public:
            LeastSquaresControllerWithStretchingConstraint(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    RobotInterface::Ptr robot,
                    Visualizer::Ptr vis,
                    const JacobianModel::ConstPtr& model);

        private:
            virtual OutputData getGripperMotion_impl(const InputData& input_data) override final;

            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                    stretchingCorrectionVectorsAndPoints(const InputData& input_data) const;
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                    ropeTwoGrippersStretchingCorrectionVectorsAndPoints(const InputData& input_data) const;
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
                    clothTwoGrippersStretchingCorrectionVectorsAndPoints(const InputData& input_data) const;

            void visualizeCone(
                    const Eigen::Vector3d& cone_direction,
                    const double min_normalized_dot_product,
                    const Eigen::Isometry3d& pose,
                    const int marker_id);

            // Task Data
            const DeformableType deformable_type_;
            const std::vector<GripperData> grippers_data_;

            // Model/Task Data
            const Eigen::MatrixXd nominal_distance_;
            const Eigen::MatrixXd max_node_distance_;
            const Eigen::MatrixXd max_node_squared_distance_;

            // Controller parameters
            const double distance_to_obstacle_threshold_;
            const double stretching_cosine_threshold_;

            // Visualization
            const bool visualize_overstretch_cones_;
    };
}

#endif // LEAST_SQUARES_CONTROLLER_WITH_STRETCHING_CONSTRAINT_H
