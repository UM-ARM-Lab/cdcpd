#ifndef CONSTRAINT_JACOBIAN_MODEL_H
#define CONSTRAINT_JACOBIAN_MODEL_H

#include <sdf_tools/sdf.hpp>

#include <smmap_models/deformable_model.h>

namespace smmap
{
    class ConstraintJacobianModel final : public DeformableModel
    {
        public:
            ////////////////////////////////////////////////////////////////////
            // Constructors and Destructor
            ////////////////////////////////////////////////////////////////////

            ConstraintJacobianModel(
                    std::shared_ptr<ros::NodeHandle> nh,
                    const double translation_dir_deformability,
                    const double translation_dis_deformability,
                    const double rotation_deformability,
                    const sdf_tools::SignedDistanceField::ConstPtr& environment_sdf);

            virtual ObjectPointSet getObjectDelta_impl(
                    const WorldState& world_state,
                    const AllGrippersSinglePoseDelta& grippers_pose_delta) const final override;

            ////////////////////////////////////////////////////////////////////
            // Static functions to set data for all models
            ////////////////////////////////////////////////////////////////////

            static void SetInitialObjectConfiguration(
                    const ObjectPointSet& object_initial_configuration);

        private:

            ////////////////////////////////////////////////////////////////////
            // Virtual function overrides
            ////////////////////////////////////////////////////////////////////

            virtual void updateModel_impl(
                    const WorldState& previous,
                    const WorldState& next) final override;

            ////////////////////////////////////////////////////////////////////
            // Static helpers
            ////////////////////////////////////////////////////////////////////

            static void ComputeObjectNodeDistanceMatrix();

            ////////////////////////////////////////////////////////////////////
            // Static members
            ////////////////////////////////////////////////////////////////////

            static std::atomic<bool> static_data_initialized_;
            static Eigen::MatrixXd object_initial_node_distance_;

            /// Indexed first by gripper, second by node i.e. (gripper_ind, node_ind)
            static Eigen::MatrixXd gripper_influence_per_node_;
            static ssize_t num_nodes_;

            ////////////////////////////////////////////////////////////////////
            // Model function parameters
            ////////////////////////////////////////////////////////////////////

            const double translation_dir_deformability_;
            const double translation_dis_deformability_;

            // Analog of the original version
            const double rotation_deformability_;

            ////////////////////////////////////////////////////////////////////
            // Obstacle information from sdf tool
            ////////////////////////////////////////////////////////////////////
            const sdf_tools::SignedDistanceField::ConstPtr sdf_;
            const double obstacle_threshold_;

            ////////////////////////////////////////////////////////////////////
            // Function to adjust rigidity actually
            ////////////////////////////////////////////////////////////////////

            double dirPropotionalModel(
                    const Eigen::Vector3d node_to_gripper,
                    const Eigen::Vector3d gripper_translation,
                    const double dist_rest) const;

            double disLinearModel(
                    const double dist_to_gripper,
                    const double dist_rest) const;

            ////////////////////////////////////////////////////////////////////
            // Jacobian and Mask matrix computation
            ////////////////////////////////////////////////////////////////////

            Eigen::MatrixXd computeGrippersToDeformableObjectJacobian(
                    const WorldState& world_state,
                    const AllGrippersSinglePoseDelta& grippers_pose_delta) const;
    };
}

#endif // CONSTRAINT_JACOBIAN_MODEL_H
