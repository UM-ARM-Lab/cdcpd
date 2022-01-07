#ifndef DIMINISHING_RIGIDITY_MODEL_H
#define DIMINISHING_RIGIDITY_MODEL_H

#include <smmap_models/jacobian_model.h>

namespace smmap
{
    class DiminishingRigidityModel final : public JacobianModel
    {
        public:
            ////////////////////////////////////////////////////////////////////
            // Constructors and Destructor
            ////////////////////////////////////////////////////////////////////

            DiminishingRigidityModel(
                    std::shared_ptr<ros::NodeHandle> nh,
                    const double deformability);

            DiminishingRigidityModel(
                    std::shared_ptr<ros::NodeHandle> nh,
                    const double translation_deformability,
                    const double rotation_deformability);

            ////////////////////////////////////////////////////////////////////
            // Static functions to set data for all models
            ////////////////////////////////////////////////////////////////////

            static void SetInitialObjectConfiguration(
                    const ObjectPointSet& object_initial_configuration);

        private:

            ////////////////////////////////////////////////////////////////////
            // Static helpers
            ////////////////////////////////////////////////////////////////////

            static void ComputeObjectNodeDistanceMatrix();

            ////////////////////////////////////////////////////////////////////
            // Virtual function overrides
            ////////////////////////////////////////////////////////////////////

            virtual void updateModel_impl(
                    const WorldState& previous,
                    const WorldState& next) final override;

            virtual Eigen::MatrixXd computeGrippersToDeformableObjectJacobian_impl(
                    const WorldState& world_state) const override final;

            ////////////////////////////////////////////////////////////////////
            // Static members
            ////////////////////////////////////////////////////////////////////

            static std::atomic_bool static_data_initialized_;
            static Eigen::MatrixXd object_initial_node_distance_;
            static ssize_t num_nodes_;

            ////////////////////////////////////////////////////////////////////
            // Private members
            ////////////////////////////////////////////////////////////////////

            const double translation_deformability_;
            const double rotation_deformability_;
    };
}

#endif // DIMINISHING_RIGIDITY_MODEL_H
