#include <smmap_models/jacobian_model.h>
#include <arc_utilities/eigen_helpers.hpp>

using namespace smmap;
using namespace Eigen;
using namespace EigenHelpers;

////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructor
////////////////////////////////////////////////////////////////////////////////

JacobianModel::JacobianModel(std::shared_ptr<ros::NodeHandle> nh)
    : DeformableModel(nh)
{}

Eigen::MatrixXd JacobianModel::computeGrippersToDeformableObjectJacobian(
        const WorldState& world_state) const
{
    return computeGrippersToDeformableObjectJacobian_impl(world_state);
}

////////////////////////////////////////////////////////////////////////////////
// Virtual function overrides
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief JacobianModel::getObjectDelta_impl
 * @param input_data
 * @param grippers_pose_delta
 * @return
 */
ObjectPointSet JacobianModel::getObjectDelta_impl(
        const WorldState& world_state,
        const AllGrippersSinglePoseDelta& grippers_pose_delta) const
{
    const MatrixXd J = computeGrippersToDeformableObjectJacobian_impl(world_state);

    const Eigen::VectorXd grippers_delta =
            EigenHelpers::VectorEigenVectorToEigenVectorX(grippers_pose_delta);

    // Move the object based on the movement of each gripper
    MatrixXd object_delta = J * grippers_delta;

    object_delta.resizeLike(world_state.object_configuration_);
    return object_delta;
}
