#ifndef STRETCHINCONSTRAINTCONTROLLER_H
#define STRETCHINCONSTRAINTCONTROLLER_H

#include <sdf_tools/sdf.hpp>
#include <smmap_utilities/visualization_tools.h>

#include "smmap/deformable_controller.h"
#include "smmap/robot_interface.h"

namespace smmap
{
    class StretchingConstraintController : public DeformableController
    {
        public:
            StretchingConstraintController(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    RobotInterface::Ptr robot,
                    Visualizer::Ptr vis,
                    const DeformableModel::ConstPtr& deformable_model,
                    const sdf_tools::SignedDistanceField::ConstPtr sdf,
                    const std::shared_ptr<std::mt19937_64>& generator,
                    const StretchingConstraintControllerSolverType gripper_controller_type,
                    const int max_count);

        private:
            /////////////////////////////////////////////////////////////////////////////////////////
            // Optimization functions
            /////////////////////////////////////////////////////////////////////////////////////////

            OutputData getGripperMotion_impl(const InputData& input_data);

            OutputData solvedByRandomSampling(const InputData& input_data);

            OutputData solvedByNomad(const InputData& input_data);

            OutputData solvedByGradientDescentProjectionViaGurobiMethod(const InputData& input_data);

            /////////////////////////////////////////////////////////////////////////////////////////
            // Helper functions
            /////////////////////////////////////////////////////////////////////////////////////////

            kinematics::Vector6d singleGripperPoseDeltaSampler(const double max_delta);

            AllGrippersSinglePoseDelta allGripperPoseDeltaSampler(const ssize_t num_grippers, const double max_delta);

            kinematics::Vector6d getConstraintAwareGripperDeltaSample(
                    const Eigen::Isometry3d& gripper_pose,
                    const CollisionData& collision_data,
                    const double max_delta,
                    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& stretching_correction_data);

            kinematics::Vector6d getFeasibleGripperDeltaGurobi(
                    const Eigen::Isometry3d& gripper_pose,
                    const CollisionData& collision_data,
                    const double max_delta,
                    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& stretching_correction_data) const;

            double errorOfControlByPrediction(const ObjectPointSet predicted_object_p_dot,
                                              const Eigen::VectorXd &desired_object_p_dot,
                                              const Eigen::VectorXd &desired_p_dot_weight) const;

            void visualize_stretching_vector(const ObjectPointSet& object_configuration);

            void visualize_rope_stretching_vector(const ObjectPointSet& object_configuration);

            void visualize_cloth_stretching_vector(const ObjectPointSet& object_configuration);

            void visualize_gripper_motion(
                    const AllGrippersSinglePose& current_gripper_pose,
                    const AllGrippersSinglePoseDelta& gripper_motion);

            /////////////////////////////////////////////////////////////////////////////////////////
            // Collision constraint related function
            /////////////////////////////////////////////////////////////////////////////////////////

            double gripperCollisionCheckHelper(
                    const AllGrippersSinglePose& current_gripper_pose,
                    const AllGrippersSinglePoseDelta& test_gripper_motion) const;

            bool gripperCollisionCheckResult(
                    const AllGrippersSinglePose& current_gripper_pose,
                    const AllGrippersSinglePoseDelta &test_gripper_motion) const;

            /////////////////////////////////////////////////////////////////////////////////////////
            // Stretching constraint related function
            /////////////////////////////////////////////////////////////////////////////////////////

            bool stretchingDetection(
                    const InputData& input_data,
                    const AllGrippersSinglePoseDelta& test_gripper_motion);

            double stretchingFunctionEvaluation(
                    const InputData& input_data,
                    const AllGrippersSinglePoseDelta& test_gripper_motion);

            double ropeTwoGripperStretchingHelper(
                    const InputData& input_data,
                    const AllGrippersSinglePoseDelta& test_gripper_motion);

            bool ropeTwoGrippersStretchingDetection(
                    const InputData& input_data,
                    const AllGrippersSinglePoseDelta& test_gripper_motion);

            double clothTwoGripperStretchingHelper(
                    const InputData& input_data,
                    const AllGrippersSinglePoseDelta& test_gripper_motion);

            bool clothTwoGrippersStretchingDetection(
                    const InputData& input_data,
                    const AllGrippersSinglePoseDelta& test_gripper_motion);

            double evaluateStretchingConstraint(
                    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& stretching_constraint_data,
                    const kinematics::Vector6d& gripper_delta) const;

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

        private:	    
            GripperCollisionChecker gripper_collision_checker_;
            const std::vector<GripperData> grippers_data_;

            const sdf_tools::SignedDistanceField::ConstPtr environment_sdf_;
            const std::shared_ptr<std::mt19937_64> generator_;
            std::uniform_real_distribution<double> uniform_unit_distribution_;

            StretchingConstraintControllerSolverType gripper_controller_type_;
            const DeformableType deformable_type_;
            const TaskType task_type_;

            const Eigen::MatrixXd nominal_distance_;
            const Eigen::MatrixXd max_node_distance_;
            const Eigen::MatrixXd max_node_squared_distance_;
            const double distance_to_obstacle_threshold_;
            double stretching_cosine_threshold_;

            const int max_count_;
            int sample_count_;

            bool fix_step_;
            bool overstretch_;
            const std::string log_file_path_;
            Log::Log num_model_calls_;
    };
}

#endif // LEAST_SQUARES_CONTROLLER_RANDOM_SAMPLING_H
