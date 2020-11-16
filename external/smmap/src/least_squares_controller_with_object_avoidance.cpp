#include "smmap/least_squares_controller_with_object_avoidance.h"
#include <arc_utilities/eigen_helpers.hpp>
#include <smmap_utilities/gurobi_solvers.h>

using namespace smmap;
using namespace Eigen;
using namespace EigenHelpers;

#pragma message "Magic number - damping threshold and damping coefficient"
#define LEAST_SQUARES_DAMPING_THRESHOLD (1e-4)
#define LEAST_SQUARES_DAMPING_VALUE     (1e-3)

LeastSquaresControllerWithObjectAvoidance::LeastSquaresControllerWithObjectAvoidance(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        RobotInterface::Ptr robot,
        Visualizer::Ptr vis,
        const JacobianModel::ConstPtr& model,
        const double obstacle_avoidance_scale,
        const bool optimize)
    : DeformableController(nh, ph, robot, vis, model)
    , obstacle_avoidance_scale_(obstacle_avoidance_scale)
    , optimize_(optimize)
{}

DeformableController::OutputData LeastSquaresControllerWithObjectAvoidance::getGripperMotion_impl(
        const InputData& input_data)
{
    const auto& grippers_data = DeformableModel::GetGrippersData();

    const size_t num_grippers = grippers_data.size();
    const ssize_t num_nodes = input_data.world_current_state_.object_configuration_.cols();
    const ssize_t num_robot_dof = input_data.robot_jacobian_valid_ ? input_data.robot_jacobian_.cols() : 0;

    ////////////////////////////////////////////////////////////////////////
    // Find the velocities of each part of the algorithm
    ////////////////////////////////////////////////////////////////////////

    // Retrieve the desired object velocity (p_dot)
    const ObjectDeltaAndWeight& desired_object_motion = input_data.handle_overstretch_ ?
                input_data.desired_object_motion_.combined_correction_ :
                input_data.desired_object_motion_.error_correction_;

    // Recalculate the jacobian at each timestep, because of rotations being non-linear
    // We can use a static pointer cast here because we check the input on construction
    const auto jacobian_based_model = std::static_pointer_cast<const JacobianModel>(model_);
    const MatrixXd grippers_poses_to_object_jacobian =
            jacobian_based_model->computeGrippersToDeformableObjectJacobian(input_data.world_current_state_);

    // Zero initialize the output
    OutputData suggested_robot_motion(num_grippers, num_nodes, num_robot_dof);
    // Remapped data array
    Map<VectorXd> object_delta_as_vector(
                suggested_robot_motion.object_motion_.data(), suggested_robot_motion.object_motion_.size());

    if (input_data.robot_jacobian_valid_)
    {
        const double max_robot_dof_step_size = input_data.max_robot_dof_step_size_;
        const double max_grippers_step_size = input_data.max_grippers_step_size_;

        // Build the robot DOF to deformable object jacobian
        const MatrixXd& robot_dof_to_grippers_poses_jacobian = input_data.robot_jacobian_;
        const MatrixXd robot_dof_to_deformable_object_jacobian =
                grippers_poses_to_object_jacobian * robot_dof_to_grippers_poses_jacobian;

        const size_t num_poi = input_data.full_robot_poi_collision_data_.size();
        std::vector<RowVectorXd> linear_constraint_linear_terms(num_poi);
        std::vector<double> linear_constraint_affine_terms(num_poi);
        for (size_t poi_ind = 0; poi_ind < num_poi; ++poi_ind)
        {
            const CollisionData& collision_data = input_data.full_robot_poi_collision_data_[poi_ind].first;
            const MatrixXd& poi_jacobian = input_data.full_robot_poi_collision_data_[poi_ind].second;
            linear_constraint_linear_terms[poi_ind] =
                    -collision_data.obstacle_surface_normal_.transpose() * poi_jacobian;

            linear_constraint_affine_terms[poi_ind] =
                    collision_data.distance_to_obstacle_ - robot_->min_controller_distance_to_obstacles_;

//            std::cout << "Poi ind: " << poi_ind << " Dist to obstacle: " << collision_data.distance_to_obstacle_ << " Min dist: " << robot_->min_controller_distance_to_obstacles_ << std::endl
//                      << "Jacobian:\n" << poi_jacobian << std::endl;
        }

        const VectorXd& joint_weights = input_data.robot_->getJointWeights();
        std::vector<MatrixXd> quadratic_constraint_quadratic_terms;
        std::vector<RowVectorXd> quadratic_constraint_linear_terms;
        std::vector<double> quadratic_constraint_affine_terms;
        if (joint_weights.size() == 16)
        {
            ROS_WARN_THROTTLE(4.0, "Assuming that we are using Val, and each arm is to be treated independently, with the last 2 DOF for the torso");

            MatrixXd arm1_weights = joint_weights.asDiagonal();
            arm1_weights.block<7, 7>(7, 7).setZero();
            quadratic_constraint_quadratic_terms.push_back(arm1_weights);

            MatrixXd arm2_weights = joint_weights.asDiagonal();
            arm2_weights.block<7, 7>(0, 0).setZero();
            quadratic_constraint_quadratic_terms.push_back(arm2_weights);

            quadratic_constraint_linear_terms.resize(2, RowVectorXd::Zero(joint_weights.size()));
            quadratic_constraint_affine_terms.resize(2, max_robot_dof_step_size * max_robot_dof_step_size);
        }
        else
        {
            ROS_WARN_THROTTLE(4.0, "Assuming that we are using Victor, and each arm is to be treated independently");

            MatrixXd arm1_weights = joint_weights.asDiagonal();
            arm1_weights.block<7, 7>(7, 7).setZero();
            quadratic_constraint_quadratic_terms.push_back(arm1_weights);

            MatrixXd arm2_weights = joint_weights.asDiagonal();
            arm2_weights.block<7, 7>(0, 0).setZero();
            quadratic_constraint_quadratic_terms.push_back(arm2_weights);

            quadratic_constraint_linear_terms.resize(2, RowVectorXd::Zero(joint_weights.size()));
            quadratic_constraint_affine_terms.resize(2, max_robot_dof_step_size * max_robot_dof_step_size);
        }


        const VectorXd min_joint_delta = input_data.robot_->getJointLowerLimits() - input_data.world_current_state_.robot_configuration_;
        const VectorXd max_joint_delta = input_data.robot_->getJointUpperLimits() - input_data.world_current_state_.robot_configuration_;

        suggested_robot_motion.robot_dof_motion_ = minSquaredNorm_SE3VelocityConstraints_QuadraticConstraints(
                    robot_dof_to_deformable_object_jacobian,
                    desired_object_motion.delta,
                    desired_object_motion.weight,
                    robot_dof_to_grippers_poses_jacobian,
                    max_grippers_step_size,
                    linear_constraint_linear_terms,
                    linear_constraint_affine_terms,
                    quadratic_constraint_quadratic_terms,
                    quadratic_constraint_linear_terms,
                    quadratic_constraint_affine_terms,
                    min_joint_delta,
                    max_joint_delta);

        // Assemble the output
        object_delta_as_vector = robot_dof_to_deformable_object_jacobian * suggested_robot_motion.robot_dof_motion_;

        const VectorXd grippers_motion = robot_dof_to_grippers_poses_jacobian * suggested_robot_motion.robot_dof_motion_;
        suggested_robot_motion.grippers_motion_ = EigenVectorXToVectorEigenVector<double, 6>(grippers_motion);







//        const VectorXd grippers_delta_achieve_goal =
//                minSquaredNormSE3VelocityConstraints(
//                    grippers_poses_to_object_jacobian,
//                    desired_object_motion.delta,
//                    max_grippers_step_size,
//                    desired_object_motion.weight);


//        std::cout << "Desired_cloth_motion = [" << desired_object_motion.delta.transpose() << "]';\n";
//        std::cout << "Weights = [" << desired_object_motion.weight.transpose() << "]';\n";
//        std::cout << "Cloth_jacobian = [\n" << grippers_poses_to_object_jacobian << "];\n";
//        std::cout << "Robot_jacobian = [\n" << robot_dof_to_grippers_poses_jacobian << "];\n";
//        std::cout << "Full_jacobian = [\n" << robot_dof_to_deformable_object_jacobian << "];\n";








//        std::cout << "max_se3_norm = " << max_grippers_step_size << ";\n";

//        std::cout << "J_matrix = [\n" << J << "];\n";


//        std::cout << "A_matrix = [\n" << A << "];\n";
//        std::cout << "b_vector = [" << b.transpose() << "]';\n";
//        std::cout << "Weights  = [" << weights.transpose() << "]';\n";



//        std::cout << "Linear_constraint_linear_terms = [\n";
//        for (size_t idx = 0; idx < linear_constraint_linear_terms.size(); ++idx)
//        {
//            std::cout << linear_constraint_linear_terms[idx] << std::endl;
//        }
//        std::cout << "];\n";

//        std::cout << "Linear_constraint_affine_terms = [";
//        for (size_t idx = 0; idx < linear_constraint_affine_terms.size(); ++idx)
//        {
//            std::cout << linear_constraint_affine_terms[idx] << " ";
//        }
//        std::cout << "]';\n";



//        std::cout << "Quadratic_constraint_quadratic_terms = [\n";
//        for (size_t idx = 0; idx < quadratic_constraint_quadratic_terms.size(); ++idx)
//        {
//            std::cout << quadratic_constraint_quadratic_terms[idx] << std::endl;
//        }
//        std::cout << "];\n";

//        std::cout << "Quadratic_constraint_linear_terms = [\n";
//        for (size_t idx = 0; idx < quadratic_constraint_linear_terms.size(); ++idx)
//        {
//            std::cout << quadratic_constraint_linear_terms[idx] << std::endl;
//        }
//        std::cout << "];\n";

//        std::cout << "Quadratic_constraint_affine_terms = [";
//        for (size_t idx = 0; idx < quadratic_constraint_affine_terms.size(); ++idx)
//        {
//            std::cout << quadratic_constraint_affine_terms[idx] << " ";
//        }
//        std::cout << "]';\n";



//        std::cout << "delta_lower_bound = [" << min_joint_delta.transpose() << "]';\n";
//        std::cout << "delta_upper_bound = [" << max_joint_delta.transpose() << "]';\n";



//        std::cout << "full_robot_dof_motion = [" << suggested_robot_motion.robot_dof_motion_.transpose() << "]';\n";


//        std::cout << std::endl;
//        assert(false);





//        std::cout << "Pure gripper optimization: " << grippers_delta_achieve_goal.head<6>().normalized().transpose() << "    " << grippers_delta_achieve_goal.tail<6>().normalized().transpose() << std::endl;
//        std::cout << "Full robot   optimization: " << grippers_motion.head<6>().normalized().transpose() << "    " << grippers_motion.tail<6>().normalized().transpose() << std::endl;
//        std::cout << "Difference:                "
//                  << (grippers_delta_achieve_goal.head<6>().normalized() - grippers_motion.head<6>().normalized()).transpose() << "    "
//                  << (grippers_delta_achieve_goal.tail<6>().normalized() - grippers_motion.tail<6>().normalized()).transpose() << std::endl;

    }
    else
    {
        const double max_grippers_step_size = input_data.max_grippers_step_size_;

        // Find the least-squares fitting to the desired object velocity
        VectorXd grippers_delta_achieve_goal;
        if (optimize_)
        {
            grippers_delta_achieve_goal =
                    minSquaredNorm_SE3VelocityConstraints(
                        grippers_poses_to_object_jacobian,
                        desired_object_motion.delta,
                        desired_object_motion.weight,
                        max_grippers_step_size);
        }
        else
        {
            grippers_delta_achieve_goal =
                    ClampGripperPoseDeltas(
                        WeightedLeastSquaresSolver(
                            grippers_poses_to_object_jacobian,
                            desired_object_motion.delta,
                            desired_object_motion.weight,
                            LEAST_SQUARES_DAMPING_THRESHOLD,
                            LEAST_SQUARES_DAMPING_VALUE),
                        max_grippers_step_size);
        }

        // Find the collision avoidance data that we'll need
        const std::vector<CollisionAvoidanceResult> grippers_collision_avoidance_result =
                ComputeGripperObjectAvoidance(
                    input_data.world_current_state_.gripper_collision_data_,
                    input_data.world_current_state_.all_grippers_single_pose_,
                    max_grippers_step_size);

        ////////////////////////////////////////////////////////////////////////
        // Combine the velocities into a single command velocity
        ////////////////////////////////////////////////////////////////////////

        for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++)
        {
            suggested_robot_motion.grippers_motion_[gripper_ind] =
                    CombineDesiredAndObjectAvoidance(
                        grippers_delta_achieve_goal.segment<6>((ssize_t)gripper_ind * 6),
                        grippers_collision_avoidance_result[gripper_ind],
                    obstacle_avoidance_scale_);

            object_delta_as_vector +=
                    grippers_poses_to_object_jacobian.block(0, 6 * (ssize_t)gripper_ind, num_nodes * 3, 6) *
                    suggested_robot_motion.grippers_motion_[gripper_ind];
        }
    }

    return suggested_robot_motion;
}
