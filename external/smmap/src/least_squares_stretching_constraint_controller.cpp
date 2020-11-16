#include <arc_utilities/eigen_helpers.hpp>
#include <smmap_utilities/gurobi_solvers.h>

#include "smmap/least_squares_stretching_constraint_controller.h"
#include "smmap/ros_communication_helpers.h"
#include "smmap/jacobian_model.h"

using namespace smmap;
using namespace Eigen;
using namespace EigenHelpers;



LeastSquaresControllerWithStretchingConstraint::LeastSquaresControllerWithStretchingConstraint(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        RobotInterface::Ptr robot,
        Visualizer::Ptr vis,
        const JacobianModel::ConstPtr& model)
    : DeformableController(nh, ph, robot, vis, model)
    , deformable_type_(GetDeformableType(*nh_))
    , grippers_data_(robot->getGrippersData())
    , nominal_distance_(CalculateDistanceMatrix(GetObjectInitialConfiguration(*nh_)))
    , max_node_distance_(GetMaxStretchFactor(*ph_) * nominal_distance_)
    , max_node_squared_distance_(max_node_distance_.cwiseProduct(max_node_distance_))
    , distance_to_obstacle_threshold_(GetRobotGripperRadius())
    , stretching_cosine_threshold_(GetStretchingCosineThreshold(*ph_))
    , visualize_overstretch_cones_(GetVisualizeOverstretchCones(*ph_))
{}

DeformableController::OutputData LeastSquaresControllerWithStretchingConstraint::getGripperMotion_impl(
        const InputData& input_data)
{
    // Unpack the input data into its constituent parts
    const auto& world_state = input_data.world_current_state_;
    const auto& object_config = world_state.object_configuration_;
    const auto& grippers_poses = world_state.all_grippers_single_pose_;
    const ssize_t num_grippers = (ssize_t)(grippers_poses.size());
    const ssize_t num_nodes = input_data.world_current_state_.object_configuration_.cols();
    const ssize_t num_robot_dof = input_data.robot_jacobian_valid_ ? input_data.robot_jacobian_.cols() : 0;
    assert(num_grippers == 2 && "This function is only intended to be used with 2 grippers");

    const VectorXd& desired_object_p_dot = input_data.desired_object_motion_.error_correction_.delta;
    const VectorXd& desired_p_dot_weight = input_data.desired_object_motion_.error_correction_.weight;

    // Check object current stretching status
    bool overstretch = false;
    if (input_data.handle_overstretch_)
    {
        const MatrixXd node_squared_distance = CalculateSquaredDistanceMatrix(object_config);
        overstretch = ((max_node_squared_distance_ - node_squared_distance).array() < 0.0).any();
    }

    // Only needed if overstretch has happened, put here to keep code in one place
    // Note that the returned vectors and points are in gripper frame
    // stretching_constraint_data[].first is the direction that we want to move the point
    // stretching_constraint_data[].second is the point that we are constrainting the motion of
    const auto stretching_constraint_data = stretchingCorrectionVectorsAndPoints(input_data);
    std::vector<VectorVector3d> pyramid_plane_normals(num_grippers);
    if (input_data.handle_overstretch_)
    {
        for (ssize_t gripper_idx = 0; gripper_idx < num_grippers; ++gripper_idx)
        {
            pyramid_plane_normals[gripper_idx] =
                    ConvertConeToPyramid(stretching_constraint_data[gripper_idx].first, stretching_cosine_threshold_);

            // Visualization
            if (visualize_overstretch_cones_)
            {
                if (overstretch)
                {
                    const Vector3d stretching_start =
                            grippers_poses[gripper_idx] * stretching_constraint_data[gripper_idx].second;
                    const Vector3d stretching_end =
                            grippers_poses[gripper_idx] * (stretching_constraint_data[gripper_idx].second + 0.1 * stretching_constraint_data[gripper_idx].first);

                    vis_->visualizeLines("stretching_correction_vector_" + std::to_string(gripper_idx), {stretching_start}, {stretching_end}, Visualizer::Red(), (int32_t)gripper_idx + 1);
                    vis_->visualizePoints("stretching_correction_vector_" + std::to_string(gripper_idx), VectorVector3d{stretching_start, stretching_end}, {Visualizer::Red(), Visualizer::Blue()}, (int32_t)(gripper_idx + num_grippers) + 1);
                    visualizeCone(stretching_constraint_data[gripper_idx].first, stretching_cosine_threshold_, grippers_poses[gripper_idx], (int32_t)gripper_idx + 1);
                }
                else
                {
                    vis_->deleteObjects("stretching_correction_vector", 1, 10);
                    vis_->deleteObjects("stretching_cone", 1, 10);
                }
            }
        }
    }


    // If the object is overstretched, then the math used is the following.
    // Note that 'r' is stretching_constraint_data[0].second.
    //           'n' is derirved from stretching_constraint_data[0].first and stretching_cosine_threshold_
    //           'v' and 'w' are the translational and rotational component of the gripper velocity

    // We want the dot product of the motion of the point to be in the same general direction as the vector
    // that point towards the inside of the cone. I.e. a positive dot product. Note that below 'n' points
    // outward, hence we want a negative dot product

    // The motion of the point when accounting for the se(3) motion of the gripper is then
    //
    // w cross r + v
    //
    // Thus we want
    //
    // dot(n, w cross r + v) <= 0
    //
    // or equivalently
    //
    // dot(n, v - r cross w) <= 0
    // dot(n, v - skew(r) * w) <= 0





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
        assert(false && "Not implemented");
    }
    else
    {
        const auto& collision_data = world_state.gripper_collision_data_;

        // Build the linear version of the constraints
        std::vector<RowVectorXd> linear_constraint_linear_terms;
        std::vector<double> linear_constraint_affine_terms;

        // Collision constraints, in the form Cx <= d
        {
            const auto J_collision_g0 = ComputeGripperMotionToPointMotionJacobian(collision_data[0].nearest_point_to_obstacle_, grippers_poses[0]);
            const auto J_distance_g0 = collision_data[0].obstacle_surface_normal_.transpose() * J_collision_g0;

            RowVectorXd full_constraint_vec_g0(num_grippers * 6);
            full_constraint_vec_g0 << (-1.0 * J_distance_g0), RowVectorXd::Zero(6);

            linear_constraint_linear_terms.push_back(full_constraint_vec_g0);
            linear_constraint_affine_terms.push_back(collision_data[0].distance_to_obstacle_ - robot_->min_controller_distance_to_obstacles_);

            const auto J_collision_g1 = ComputeGripperMotionToPointMotionJacobian(collision_data[1].nearest_point_to_obstacle_, grippers_poses[1]);
            const auto J_distance_g1 = collision_data[0].obstacle_surface_normal_.transpose() * J_collision_g1;

//            std::cout << "J_collision_g1:\n" << J_collision_g1 << std::endl;
//            std::cout << "J_distance_g1:\n" << J_distance_g1 << std::endl;

            RowVectorXd full_constraint_vec_g1(num_grippers * 6);
            full_constraint_vec_g1 << RowVectorXd::Zero(6), (-1.0 * J_distance_g1);

            linear_constraint_linear_terms.push_back(full_constraint_vec_g1);
            linear_constraint_affine_terms.push_back(collision_data[1].distance_to_obstacle_ - robot_->min_controller_distance_to_obstacles_);
//            std::cout << "max movement towards wall: " << linear_constraint_affine_terms.back() << std::endl;
        }

        // Stretching constraints
        if (overstretch)
        {
            assert(input_data.handle_overstretch_);

            Matrix<double, 3, 6> J_stretching_g0;
            J_stretching_g0.leftCols<3>() = Matrix3d::Identity();
            J_stretching_g0.rightCols<3>() = -kinematics::skew(stretching_constraint_data[0].second);

            for (size_t ind = 0; ind < pyramid_plane_normals[0].size(); ++ind)
            {
                RowVectorXd full_constraint_vec(num_grippers * 6);
                full_constraint_vec << (pyramid_plane_normals[0][ind].transpose() * J_stretching_g0), RowVectorXd::Zero(6);
                linear_constraint_linear_terms.push_back(full_constraint_vec);
                linear_constraint_affine_terms.push_back(0.0);
            }

            Matrix<double, 3, 6> J_stretching_g1;
            J_stretching_g1.leftCols<3>() = Matrix3d::Identity();
            J_stretching_g1.rightCols<3>() = -kinematics::skew(stretching_constraint_data[1].second);

            for (size_t ind = 0; ind < pyramid_plane_normals[1].size(); ++ind)
            {
                RowVectorXd full_constraint_vec(num_grippers * 6);
                full_constraint_vec << RowVectorXd::Zero(6), (pyramid_plane_normals[1][ind].transpose() * J_stretching_g1);
                linear_constraint_linear_terms.push_back(full_constraint_vec);
                linear_constraint_affine_terms.push_back(0.0);
            }
        }

        const VectorXd grippers_motion =
                minSquaredNorm_SE3VelocityConstraints_LinearConstraints(
                    grippers_poses_to_object_jacobian,
                    desired_object_p_dot,
                    desired_p_dot_weight,
                    input_data.max_grippers_step_size_,
                    linear_constraint_linear_terms,
                    linear_constraint_affine_terms);

        object_delta_as_vector = grippers_poses_to_object_jacobian * grippers_motion;

//        std::cout << linear_constraint_linear_terms[1] * grippers_motion << "   shall be <=    " << linear_constraint_affine_terms[1] << std::endl;

        suggested_robot_motion.grippers_motion_ =
                EigenVectorXToVectorEigenVector<double, 6>(grippers_motion);
    }

    return suggested_robot_motion;
}


// Note that the returned vectors and points are in gripper frame
// result.second is the point that we are constrainting the motion of
// result.first is the direction that we want to move the point
std::vector<std::pair<Vector3d, Vector3d>> LeastSquaresControllerWithStretchingConstraint::stretchingCorrectionVectorsAndPoints(const InputData& input_data) const
{
    switch (deformable_type_)
    {
        case ROPE:
        {
            return ropeTwoGrippersStretchingCorrectionVectorsAndPoints(input_data);
        }
        case CLOTH:
        {
            return clothTwoGrippersStretchingCorrectionVectorsAndPoints(input_data);
        }
        default:
        {
            assert(false && "deformable_type is neither rope nor cloth");
        }
    }
}

std::vector<std::pair<Vector3d, Vector3d>> LeastSquaresControllerWithStretchingConstraint::ropeTwoGrippersStretchingCorrectionVectorsAndPoints(const InputData& input_data) const
{
    assert(grippers_data_.size() == 2 || "grippers size is not 2, stretching vectors not defined");

    const ObjectPointSet& object_config = input_data.world_current_state_.object_configuration_;
    const Isometry3d& first_gripper_pose = input_data.world_current_state_.all_grippers_single_pose_[0];
    const Isometry3d& second_gripper_pose = input_data.world_current_state_.all_grippers_single_pose_[1];

    const ssize_t num_nodes = object_config.cols();
    const ssize_t start_node = 0;
    const ssize_t end_node = num_nodes - 1;

    const auto first_correction_vector = object_config.col(start_node + 1) - object_config.col(start_node);
    const auto second_correction_vector = object_config.col(end_node - 1) - object_config.col(end_node);

    // Zero initialize the vectors, we will update momentarily
    std::vector<std::pair<Vector3d, Vector3d>> result(2, {Vector3d::Zero(), Vector3d::Zero()});

    // Normalize the vectors to get direction only; will be zero if the input norm is 0.
    // Store the resulting vector in the return value
    const auto first_vector_norm = first_correction_vector.norm();
    if (first_vector_norm > 1e-6)
    {
        result[0].first = first_correction_vector / first_vector_norm;
    }

    const auto second_vector_norm = second_correction_vector.norm();
    if (second_vector_norm > 1e-6)
    {
        result[1].first = second_correction_vector / second_vector_norm;
    }

    // Rotate the vectors into gripper frame
    result[0].first = first_gripper_pose.rotation().transpose() * result[0].first;
    result[1].first = second_gripper_pose.rotation().transpose() * result[1].first;

    return result;
}

std::vector<std::pair<Vector3d, Vector3d>> LeastSquaresControllerWithStretchingConstraint::clothTwoGrippersStretchingCorrectionVectorsAndPoints(const InputData& input_data) const
{
    // Assume knowing there are two grippers.
    assert(grippers_data_.size() == 2 || "grippers size is not 2, stretching vectors not defined");

    const ObjectPointSet& object_config = input_data.world_current_state_.object_configuration_;
    const Isometry3d& first_gripper_pose = input_data.world_current_state_.all_grippers_single_pose_[0];
    const Isometry3d& second_gripper_pose = input_data.world_current_state_.all_grippers_single_pose_[1];

    const StretchingVectorInfo& first_stretching_vector_info = grippers_data_[0].stretching_vector_info_;
    const std::vector<long>& first_from_nodes = first_stretching_vector_info.from_nodes_;
    const std::vector<long>& first_to_nodes = first_stretching_vector_info.to_nodes_;
    const std::vector<double>& first_contribution = first_stretching_vector_info.node_contribution_;

    const StretchingVectorInfo& second_stretching_vector_info = grippers_data_[1].stretching_vector_info_;
    const std::vector<long>& second_from_nodes = second_stretching_vector_info.from_nodes_;
    const std::vector<long>& second_to_nodes = second_stretching_vector_info.to_nodes_;
    const std::vector<double>& second_contribution = second_stretching_vector_info.node_contribution_;

    Vector3d point_on_first_gripper_before_motion = MatrixXd::Zero(3,1);
    Vector3d point_on_second_gripper_before_motion = MatrixXd::Zero(3,1);
    Vector3d first_correction_vector = MatrixXd::Zero(3,1);
    Vector3d second_correction_vector = MatrixXd::Zero(3,1);

    for (size_t stretching_ind = 0; stretching_ind < first_from_nodes.size(); stretching_ind++)
    {
        const auto from_node = object_config.col(first_from_nodes[stretching_ind]);
        const auto to_node = object_config.col(first_to_nodes[stretching_ind]);
        const auto node_delta = to_node - from_node;
        first_correction_vector += first_contribution[stretching_ind] * node_delta;
        point_on_first_gripper_before_motion += first_contribution[stretching_ind] * from_node;
    }

    for (size_t stretching_ind = 0; stretching_ind < second_from_nodes.size(); stretching_ind++)
    {
        const auto from_node = object_config.col(second_from_nodes[stretching_ind]);
        const auto to_node = object_config.col(second_to_nodes[stretching_ind]);
        const auto node_delta = to_node - from_node;
        second_correction_vector += second_contribution[stretching_ind] * node_delta;
        point_on_second_gripper_before_motion += second_contribution[stretching_ind] * from_node;
    }

    // Zero initialize the vectors, we will update momentarily
    std::vector<std::pair<Vector3d, Vector3d>> result(2, {Vector3d::Zero(), Vector3d::Zero()});

    // Normalize the vectors to get direction only; will be zero if the input norm is 0.
    // Store the resulting vector in the return value
    const auto first_vector_norm = first_correction_vector.norm();
    if (first_vector_norm > 1e-6)
    {
        result[0].first = first_correction_vector / first_vector_norm;
    }
    result[0].second = point_on_first_gripper_before_motion;

    const auto second_vector_norm = second_correction_vector.norm();
    if (second_vector_norm > 1e-6)
    {
        result[1].first = second_correction_vector / second_vector_norm;
    }
    result[1].second = point_on_second_gripper_before_motion;

    // Rotate the vectors into gripper frame
    result[0].first = first_gripper_pose.rotation().transpose() * result[0].first;
    result[1].first = second_gripper_pose.rotation().transpose() * result[1].first;

    // Transform the points into the gripper frame
    result[0].second = first_gripper_pose.inverse() * result[0].second;
    result[1].second = second_gripper_pose.inverse() * result[1].second;

    return result;
}


void LeastSquaresControllerWithStretchingConstraint::visualizeCone(
        const Vector3d& cone_direction,
        const double min_normalized_dot_product,
        const Isometry3d& pose,
        const int marker_id)
{
    // Build a vector that is garunteed to be perpendicular to cone_direction, and non-zero
    auto tmp = VectorRejection(cone_direction, Vector3d::UnitX());
    tmp += VectorRejection(cone_direction, Vector3d::UnitY());
    tmp += VectorRejection(cone_direction, Vector3d::UnitZ());

    assert(tmp.norm() > 1e-6);
    tmp.normalize();

//    std::cerr << "pose_" << marker_id << " = [\n" << pose.matrix() << "];\n";
//    std::cerr << "cone_direction_" << marker_id << " = [" << cone_direction.transpose() << " 0.0]';\n";
//    std::cerr << "perp_direction_" << marker_id << " = [" << tmp.transpose() << " 0.0]';\n";
//    std::cerr << std::endl;

    const Vector3d p1 = tmp;
    const Vector3d p2 = cone_direction.cross(p1).normalized();
    const Vector3d p3 = -p1;
    const Vector3d p4 = -p2;

    const double theta_max = std::acos(min_normalized_dot_product);
    const double dist = std::tan(theta_max);

    const Vector3d ray1 = cone_direction + dist * p1;
    const Vector3d ray2 = cone_direction + dist * p2;
    const Vector3d ray3 = cone_direction + dist * p3;
    const Vector3d ray4 = cone_direction + dist * p4;

    const VectorVector3d starts(4, pose.translation());
    const VectorVector3d ends =
    {
        pose * (ray1.normalized() / 10.0),
        pose * (ray2.normalized() / 10.0),
        pose * (ray3.normalized() / 10.0),
        pose * (ray4.normalized() / 10.0)
    };

    vis_->visualizeLines("stretching_cone", starts, ends, Visualizer::Magenta(), marker_id);
}

