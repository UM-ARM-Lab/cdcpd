#include <deformable_manipulation_experiment_params/ros_params.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <smmap_utilities/nomad_solvers.h>
#include <smmap_utilities/gurobi_solvers.h>
#include <kinematics_toolbox/kinematics.h>
#include <omp.h>

#include "smmap/stretching_constraint_controller.h"
#include "smmap/ros_communication_helpers.h"

// Needed due to rounding problems
#define GRIPPER_COLLISION_REPULSION_MARGIN 0.000001

#define ERROR_CONVERGENCE_LIMIT 1e-6
#define BARRIER_INNER_LOOP_CONVERGENCE_LIMIT 1e-6
#define BARRIER_OUTER_LOOP_CONVERGENCE_LIMIT 1e-6
#define BARRIER_UPDATE_RATE 10.0
#define BARRIER_VIOLATED_LARGE_COST 1e3

using namespace smmap;
using namespace Eigen;
using namespace EigenHelpers;


StretchingConstraintController::StretchingConstraintController(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        RobotInterface::Ptr robot,
        Visualizer::Ptr vis,
        const DeformableModel::ConstPtr& model,
        const sdf_tools::SignedDistanceField::ConstPtr sdf,
        const std::shared_ptr<std::mt19937_64>& generator,
        const StretchingConstraintControllerSolverType gripper_controller_type,
        const int max_count)
    : DeformableController(nh, ph, robot, vis, model)
    , gripper_collision_checker_(*nh_)
    , grippers_data_(robot->getGrippersData())
    , environment_sdf_(sdf)
    , generator_(generator)
    , uniform_unit_distribution_(0.0, 1.0)
    , gripper_controller_type_(gripper_controller_type)
    , deformable_type_(GetDeformableType(*nh_))
    , task_type_(GetTaskType(*nh_))
    , nominal_distance_(CalculateDistanceMatrix(GetObjectInitialConfiguration(*nh_)))
    , max_node_distance_(GetMaxStretchFactor(*ph_) * nominal_distance_)
    , max_node_squared_distance_(max_node_distance_.cwiseProduct(max_node_distance_))
    , distance_to_obstacle_threshold_(GetRobotGripperRadius())
    , stretching_cosine_threshold_(GetStretchingCosineThreshold(*ph_))
    , max_count_(max_count)
    , sample_count_(-1)
    , fix_step_(GetUseFixedGripperDeltaSize(*ph_))
    , overstretch_(false)
    , log_file_path_(GetLogFolder(*nh_))
    , num_model_calls_(log_file_path_ + "num_model_calls.txt", false)
{}

////////////////////////////////////////////////////////////////////////////////
// Functions that are used to initialize function pointers in the
// constructor. These all require that task_type_ and
// deformable_type_ have been set already
////////////////////////////////////////////////////////////////////////////////

DeformableController::OutputData StretchingConstraintController::getGripperMotion_impl(const InputData& input_data)
{
    switch (gripper_controller_type_)
    {
        case StretchingConstraintControllerSolverType::RANDOM_SAMPLING:
            assert(false && "Not used or tested anymore");
            return solvedByRandomSampling(input_data);

        case StretchingConstraintControllerSolverType::NOMAD_OPTIMIZATION:
            assert(false && "Not used or tested anymore");
            return solvedByNomad(input_data);

        case StretchingConstraintControllerSolverType::GRADIENT_DESCENT:
            return solvedByGradientDescentProjectionViaGurobiMethod(input_data);

        default:
            assert(false && "This code should be un-reachable");
    };
}

/////////////////////////////////////////////////////////////////////////////////
// Private optimization function
/////////////////////////////////////////////////////////////////////////////////

//#define USE_MULTITHREADED_EVALUATION_FOR_SAMPLING_CONTROLLER 1

DeformableController::OutputData StretchingConstraintController::solvedByRandomSampling(const InputData& input_data)
{
    assert(false && "Not updated to use new constraints etc. Verify that this whole function is doing what we want, for both simulation and live robot");

    const WorldState& current_world_state = input_data.world_current_state_;
    const ssize_t num_grippers = (ssize_t)(current_world_state.all_grippers_single_pose_.size());
//    const ssize_t num_nodes = current_world_state.object_configuration_.cols();
    assert(num_grippers == 2 && "This function is only intended to be used with 2 grippers");

    const VectorXd& desired_object_p_dot = input_data.desired_object_motion_.error_correction_.delta;
    const VectorXd& desired_p_dot_weight = input_data.desired_object_motion_.error_correction_.weight;

    const MatrixXd node_squared_distance =
            CalculateSquaredDistanceMatrix(current_world_state.object_configuration_);

    // Check object current stretching status
    // Checking the stretching status for current object configuration for once
    overstretch_ = ((max_node_squared_distance_ - node_squared_distance).array() < 0.0).any();

    if (input_data.robot_jacobian_valid_)
    {
        assert(false && "Not implemented");
    }
    else
    {
        const double max_step_size = input_data.max_grippers_step_size_;

        std::vector<std::pair<AllGrippersSinglePoseDelta, double>> per_thread_optimal_command(
        #ifdef USE_MULTITHREADED_EVALUATION_FOR_SAMPLING_CONTROLLER
                    arc_helpers::GetNumOMPThreads(),
        #else
                    1,
        #endif
                    std::make_pair(AllGrippersSinglePoseDelta(), std::numeric_limits<double>::infinity()));

        // Check object current stretching status
        // Checking the stretching status for current object configuration for once
        overstretch_ = ((max_node_squared_distance_ - node_squared_distance).array() < 0.0).any();

        #ifdef USE_MULTITHREADED_EVALUATION_FOR_SAMPLING_CONTROLLER
            #pragma omp parallel for
        #endif
        for (int64_t ind_count = 0; ind_count < max_count_; ind_count++)
        {
            AllGrippersSinglePoseDelta grippers_motion_sample = allGripperPoseDeltaSampler(num_grippers, max_step_size);

            #ifdef USE_MULTITHREADED_EVALUATION_FOR_SAMPLING_CONTROLLER
                const size_t thread_num = (size_t)omp_get_thread_num();
            #else
                const size_t thread_num = 0;
            #endif

            // Use constraint_violation checker for gripper collosion
            // Constraint violation checking here
            const bool collision_violation = gripperCollisionCheckResult(
                        current_world_state.all_grippers_single_pose_,
                        grippers_motion_sample);

            const bool stretching_violation = stretchingDetection(
                        input_data,
                        grippers_motion_sample);

            // If no constraint violation
            if (!collision_violation && !stretching_violation)
            {
                std::pair<AllGrippersSinglePoseDelta, double>& current_thread_optimal = per_thread_optimal_command[thread_num];

                // get predicted object motion
                ObjectPointSet predicted_object_p_dot = model_->getObjectDelta(
                            input_data.world_current_state_,
                            grippers_motion_sample);

                const double sample_error = errorOfControlByPrediction(predicted_object_p_dot,
                                                                 desired_object_p_dot,
                                                                 desired_p_dot_weight);

                // Compare if the sample grippers motion is better than the best to now
                if (sample_error < current_thread_optimal.second)
                {
                    current_thread_optimal.first = grippers_motion_sample;
                    current_thread_optimal.second = sample_error;
                }
            }
        }

        // Aggreate the results from each thread into a single best command
        double best_error = std::numeric_limits<double>::infinity();
        AllGrippersSinglePoseDelta optimal_gripper_motion;
        for (size_t thread_idx = 0; thread_idx < per_thread_optimal_command.size(); thread_idx++)
        {
            if (per_thread_optimal_command[thread_idx].second < best_error)
            {
                optimal_gripper_motion = per_thread_optimal_command[thread_idx].first;
                best_error = per_thread_optimal_command[thread_idx].second;
            }
        }

        if (sample_count_ >= 0)
        {
            sample_count_++;
            if (sample_count_ >= num_grippers)
            {
                sample_count_ = 0;
            }
        }
        if ((optimal_gripper_motion.size() == 0))
        {
            ROS_WARN("No valid samples generated, setting motion to zero.");
            const kinematics::Vector6d no_movement = kinematics::Vector6d::Zero();
            optimal_gripper_motion = AllGrippersSinglePoseDelta(num_grippers, no_movement);
        }

        const ObjectPointSet object_motion = model_->getObjectDelta(
                    input_data.world_current_state_,
                    optimal_gripper_motion);

        const OutputData suggested_robot_command(
                    optimal_gripper_motion,
                    object_motion,
                    VectorXd());

        return suggested_robot_command;
    }
	DeformableController::OutputData pass;
	return pass;
}

//#undef USE_MULTITHREADED_EVALUATION_FOR_SAMPLING_CONTROLLER

DeformableController::OutputData StretchingConstraintController::solvedByNomad(const InputData& input_data)
{
    assert(false && "Not updated to use new constraints etc. Verify that this whole function is doing what we want, for both simulation and live robot");

    const WorldState& current_world_state = input_data.world_current_state_;
    const ssize_t num_grippers = (ssize_t)(current_world_state.all_grippers_single_pose_.size());
    assert(num_grippers == 2 && "This function is only intended to be used with 2 grippers");

    const VectorXd& desired_object_p_dot = input_data.desired_object_motion_.error_correction_.delta;
    const VectorXd& desired_p_dot_weight = input_data.desired_object_motion_.error_correction_.weight;

    const MatrixXd node_squared_distance =
            CalculateSquaredDistanceMatrix(current_world_state.object_configuration_);

    // Check object current stretching status
    // Checking the stretching status for current object configuration for once
    overstretch_ = ((max_node_squared_distance_ - node_squared_distance).array() < 0.0).any();

    if (input_data.robot_jacobian_valid_)
    {
        std::cerr << "Using direct robot joint velocity version of optimization" << std::endl;

        const ssize_t num_dof = input_data.world_current_state_.robot_configuration_.size();

        // Determine the search space for NOMAD, at least in terms of the decision variables only
        const double max_step_size = robot_->max_dof_velocity_norm_ * robot_->dt_;
        const VectorXd distance_to_lower_joint_limits =
                input_data.robot_->getJointLowerLimits() - input_data.world_current_state_.robot_configuration_;
        const VectorXd min_joint_delta =
                distance_to_lower_joint_limits.unaryExpr([&max_step_size] (const double x) {return std::max(x, -max_step_size);});

        const VectorXd distance_to_upper_joint_limits =
                input_data.robot_->getJointUpperLimits() - input_data.world_current_state_.robot_configuration_;
        const VectorXd max_joint_delta =
                distance_to_upper_joint_limits.unaryExpr([&max_step_size] (const double x) {return std::min(x, max_step_size);});

        // Return value of objective function, cost = norm(p_dot_desired - p_dot_test)
        const std::function<double(const VectorXd&)> eval_error_cost_fn = [&] (
                const VectorXd& test_robot_motion)
        {
            const VectorXd grippers_motion_as_single_vector =
                    input_data.robot_jacobian_ * test_robot_motion;

            if (grippers_motion_as_single_vector.size() != num_grippers * 6)
            {
                assert(false && "num of grippers not match");
            }

            AllGrippersSinglePoseDelta test_gripper_motion(num_grippers);
            for (ssize_t ind = 0; ind < num_grippers; ++ind)
            {
                test_gripper_motion[ind] = grippers_motion_as_single_vector.segment<6>(ind * 6);
            }

            const ObjectPointSet predicted_object_p_dot = model_->getObjectDelta(
                        input_data.world_current_state_,
                        test_gripper_motion);

            return errorOfControlByPrediction(predicted_object_p_dot,
                                              desired_object_p_dot,
                                              desired_p_dot_weight);
        };

        // Note that NOMAD wants all constraints in the form c(x) <= 0
        // Return the min distance of the points of interest to the obstacles, minus the required clearance
        // It is assumed that the robot's internal state matches that that is passed to us, so we do not need to set active dof values
        const std::vector<std::pair<CollisionData, Matrix3Xd>> poi_collision_data = robot_->getPointsOfInterestCollisionData();
        const double required_obstacle_clearance = input_data.robot_->min_controller_distance_to_obstacles_;
        const std::function<double(const VectorXd&)> collision_constraint_fn = [&] (
                const VectorXd& test_robot_motion)
        {
            double min_poi_distance = std::numeric_limits<double>::max();
            for (size_t poi_ind = 0; poi_ind < poi_collision_data.size(); ++poi_ind)
            {
                const CollisionData& collision_data = poi_collision_data[poi_ind].first;
                const Matrix3Xd& poi_jacobian = poi_collision_data[poi_ind].second;
                const double initial_distance = poi_collision_data[poi_ind].first.distance_to_obstacle_;

                const double current_distance = initial_distance +
                        collision_data.obstacle_surface_normal_.transpose() * poi_jacobian * test_robot_motion;

                min_poi_distance = std::min(min_poi_distance, current_distance);
            }

            return required_obstacle_clearance - min_poi_distance;
        };

        // Note that NOMAD wants all constraints in the form c(x) <= 0
        // Return the sum of cos (an indicator of direction) gripper motion to stretching vector
        const std::function<double(const VectorXd&)> stretching_constraint_fn = [&] (
                const VectorXd& test_robot_motion)
        {
            const VectorXd grippers_motion_as_single_vector =
                    input_data.robot_jacobian_ * test_robot_motion;

            if (grippers_motion_as_single_vector.size() != num_grippers * 6)
            {
                assert(false && "num of grippers not match");
            }

            AllGrippersSinglePoseDelta test_gripper_motion(num_grippers);
            for (ssize_t ind = 0; ind < num_grippers; ++ind)
            {
                test_gripper_motion[ind] = grippers_motion_as_single_vector.segment<6>(ind * 6);
            }
            return stretchingFunctionEvaluation(input_data, test_gripper_motion);
        };

        // Prevents the robot from moving too quickly
        const std::function<double(const VectorXd&)> robot_motion_constraint_fn = [&] (
                const VectorXd& test_robot_motion)
        {
            return test_robot_motion.norm() - max_step_size;
        };

        std::cerr << "Invoking NOMAD wrapper" << std::endl;

        const VectorXd optimal_robot_motion =
                minFunctionPointerDirectRobotDOF(
                    log_file_path_,
                    fix_step_,
                    max_count_,
                    num_dof,
                    min_joint_delta,
                    max_joint_delta,
                    *generator_,
                    uniform_unit_distribution_,
                    eval_error_cost_fn,
                    collision_constraint_fn,
                    stretching_constraint_fn,
                    robot_motion_constraint_fn);

        const VectorXd grippers_motion_as_single_vector =
                input_data.robot_jacobian_ * optimal_robot_motion;

        if (grippers_motion_as_single_vector.size() != num_grippers * 6)
        {
            assert(false && "num of grippers not match");
        }

        AllGrippersSinglePoseDelta optimal_gripper_motion(num_grippers);
        for (ssize_t ind = 0; ind < num_grippers; ++ind)
        {
            optimal_gripper_motion[ind] = grippers_motion_as_single_vector.segment<6>(ind * 6);
        }

        const ObjectPointSet object_motion = model_->getObjectDelta(
                    input_data.world_current_state_,
                    optimal_gripper_motion);

        const OutputData suggested_robot_command(
                    optimal_gripper_motion,
                    object_motion,
                    optimal_robot_motion);

        return suggested_robot_command;
    }
    else
    {
        std::cerr << "Using pure tan(SE3) velocity version of optimization" << std::endl;

        const double max_step_size = input_data.max_grippers_step_size_;

        // Return value of objective function, cost = norm(p_dot_desired - p_dot_test)
        const std::function<double(const AllGrippersSinglePoseDelta&)> eval_error_cost_fn = [&] (
                const AllGrippersSinglePoseDelta& test_gripper_motion)
        {
            const ObjectPointSet predicted_object_p_dot = model_->getObjectDelta(
                        input_data.world_current_state_,
                        test_gripper_motion);

            return errorOfControlByPrediction(predicted_object_p_dot,
                                              desired_object_p_dot,
                                              desired_p_dot_weight);
        };

        // Return the min distance of gripper to obstacle, minus the gripper radius
        const double required_obstacle_clearance = GetRobotGripperRadius();
        const std::function<double(const AllGrippersSinglePoseDelta&)> collision_constraint_fn = [&] (
                const AllGrippersSinglePoseDelta& test_gripper_motion)
        {
            const double min_dis_to_obstacle = gripperCollisionCheckHelper(
                        current_world_state.all_grippers_single_pose_,
                        test_gripper_motion);

            return required_obstacle_clearance - min_dis_to_obstacle;
        };

        // Return the sum of cos (an indicator of direction) gripper motion to stretching vector
        const std::function<double(const AllGrippersSinglePoseDelta&)> stretching_constraint_fn = [&] (
                const AllGrippersSinglePoseDelta& test_gripper_motion)
        {
            return stretchingFunctionEvaluation(input_data, test_gripper_motion);
        };

        // Prevents the grippers from moving too quickly
        const std::function<double(const AllGrippersSinglePoseDelta&)> gripper_motion_constraint_fn = [&] (
                const AllGrippersSinglePoseDelta& test_gripper_motion)
        {
            double max_value = 0.0;
            for (size_t gripper_ind = 0; gripper_ind < test_gripper_motion.size(); gripper_ind += 6)
            {
                const double velocity_norm = GripperVelocity6dNorm(test_gripper_motion[gripper_ind]);
                if (velocity_norm > max_value)
                {
                    max_value = velocity_norm;
                }
            }
            return max_value - max_step_size;
        };

        std::cerr << "Invoking NOMAD wrapper" << std::endl;

        const AllGrippersSinglePoseDelta optimal_gripper_motion =
                minFunctionPointerSE3Delta(
                    log_file_path_,
                    fix_step_,
                    max_count_,
                    num_grippers,
                    max_step_size,
                    *generator_,
                    uniform_unit_distribution_,
                    eval_error_cost_fn,
                    collision_constraint_fn,
                    stretching_constraint_fn,
                    gripper_motion_constraint_fn);

        const ObjectPointSet object_motion = model_->getObjectDelta(
                    input_data.world_current_state_,
                    optimal_gripper_motion);

        const OutputData suggested_robot_command(
                    optimal_gripper_motion,
                    object_motion,
                    VectorXd());

        return suggested_robot_command;
    }
}








inline VectorXd projectToMaxDeltaConstraints(
        const VectorXd& delta,
        const double max_step_size,
        const VectorXd& min_joint_delta,
        const VectorXd& max_joint_delta)
{
    auto max_norm_constrainted = delta;

    // Project to max norm
    const double starting_norm = delta.norm();
    if (starting_norm > max_step_size)
    {
        max_norm_constrainted *= (max_step_size / starting_norm);
    }

    // Project to joint limits
    auto min_delta_constrained = max_norm_constrainted.cwiseMax(min_joint_delta);
    auto max_delta_constrained = min_delta_constrained.cwiseMin(max_joint_delta);

    return max_delta_constrained;
}



// Calculates left side of: dmin - d - normal' * J * dq <= 0
//   In the form of:               b -           M * dq <= 0
inline double collisionConstraintFunction(
        const double b,
        const RowVectorXd& M,
        const VectorXd& dq)
{
    return b - M * dq;
}



inline double barrier(const double u)
{
    return -std::log(-u);
}

inline double barrier(const double t, const double u)
{
    return -(1.0/t) * std::log(-u);
}

VectorXd evaluateJointLimitsMaxDeltaConstraintsBarrier(
        const VectorXd& dq,
        const VectorXd& dq_min,
        const VectorXd& dq_max,
        const double max_step_size,
        const double t)
{
    assert(dq.size() == dq_min.size());
    assert(dq.size() == dq_max.size());
    const auto u_min = dq_min - dq;
    const auto u_max = dq - dq_max;

    VectorXd results(dq.size() * 2 + 1);
    for (ssize_t ind = 0; ind < dq.size(); ++ind)
    {
        results(ind) = barrier(t, u_min(ind));
        results(dq.size() + ind) = barrier(t, u_max(ind));
    }
    results(dq.size() * 2) = barrier(t, dq.squaredNorm() - max_step_size * max_step_size);
    return results;
}

VectorXd evaluateCollisionConstraintsBarrier(
        const std::vector<double>& b_offsets,
        const std::vector<RowVectorXd>& M_matrices,
        const VectorXd& dq,
        const double t)
{
    assert(b_offsets.size() == M_matrices.size());
    VectorXd results(b_offsets.size());
    for (size_t ind = 0; ind < b_offsets.size(); ++ind)
    {
        const double u = collisionConstraintFunction(b_offsets[ind], M_matrices[ind], dq);
        results[ind] = barrier(t, u);
    }
    return results;
}

template <typename T>
VectorXd evaluateStretchingConstraintsBarrier(
        const AllGrippersSinglePoseDelta& gripper_motion,
        const T& stretching_evaluation_fn,
        const double t)
{
    VectorXd result(1);
    const double stretching_constraint_val = stretching_evaluation_fn(gripper_motion);
    std::cout << "stretching_constraint_val: " << stretching_constraint_val << std::endl;
    result(0) = barrier(t, stretching_constraint_val);
    return result;
}




inline AllGrippersSinglePoseDelta stepInDirection(
        const AllGrippersSinglePoseDelta& start,
        const AllGrippersSinglePoseDelta& direction,
        const double step_size)
{
    assert(start.size() == direction.size());

    AllGrippersSinglePoseDelta result(start.size());
    for (size_t ind = 0; ind < start.size(); ++ind)
    {
        result[ind] = start[ind] + step_size * direction[ind];
    }
    return result;
}

inline AllGrippersSinglePoseDelta stepInDirection(
        const AllGrippersSinglePoseDelta& start,
        const VectorXd& direction,
        const double step_size)
{
    AllGrippersSinglePoseDelta dir_as_eigen = EigenVectorXToVectorEigenVector<double, 6>(direction);
    return stepInDirection(start, dir_as_eigen, step_size);
}

inline AllGrippersSinglePoseDelta stepInDirection(
        const AllGrippersSinglePoseDelta& start,
        const std::vector<double>& direction,
        const double step_size)
{
    AllGrippersSinglePoseDelta dir_as_eigen = StdVectorXToVectorEigenVector<double, 6>(direction);
    return stepInDirection(start, dir_as_eigen, step_size);
}

// Note that this assumes that the projectino is needed, and is already on the boundary of the collision constraint
inline kinematics::Vector6d projectToCornerOfCollisionAndMaxDeltaConstraints(
        const kinematics::Vector6d & delta,
        const Matrix<double, 3, 6>& collision_jacobian,
        const double max_step_size)
{
    const Matrix<double, 6, 3> collision_jacobian_inv = Pinv(collision_jacobian, SuggestedRcond());
    const auto nullspace_projector = Matrix<double, 6, 6>::Identity() - collision_jacobian_inv * collision_jacobian;

    const auto unnormalized_direction_along_line = nullspace_projector * delta;
    const double direction_norm = GripperVelocity6dNorm(unnormalized_direction_along_line);
    assert(direction_norm > 0.0);
    const auto direction_along_line = unnormalized_direction_along_line / direction_norm;
    const auto& point_on_line = delta;

    // Math taken from here: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    // They have an error in the simplification, but definition of a, b, and c is correct
//    const double a = 1.0;
    const auto b = 2.0 * GripperVelocityDotProduct(direction_along_line, point_on_line);
    const auto c = GripperVelocity6dSquaredNorm(point_on_line) - max_step_size * max_step_size;
    const auto discriminant = std::max(0.0, b * b - 4 * c);

//    std::cout << "b: " << b << std::endl;
//    std::cout << "c: " << c << std::endl;
//    std::cout << "d: " << discriminant << std::endl;

    const auto distance_along_line_plus = (-b + std::sqrt(discriminant)) / 2.0;
    const auto distance_along_line_minus = (-b - std::sqrt(discriminant)) / 2.0;

//    std::cout << "minus: " << distance_along_line_minus << std::endl;
//    std::cout << "plus: " << distance_along_line_plus << std::endl;
//    std::cout << "b*b - 4*c: " << b * b - 4 * c << std::endl;

    // Accept the motion that is smaller (i.e. closer to the start point)
    if (std::abs(distance_along_line_plus) < std::abs(distance_along_line_minus))
    {
        return delta + distance_along_line_plus * direction_along_line;
    }
    else
    {
        return delta + distance_along_line_minus * direction_along_line;
    }
}

inline kinematics::Vector6d projectToCollisionAndMaxDeltaConstraints(
        const Isometry3d& starting_pose,
        const kinematics::Vector6d& delta,
        const CollisionData& collision_data,
        const double min_distance,
        const double max_step_size)
{
    const Matrix<double, 3, 6> J_collision =
            ComputeGripperMotionToPointMotionJacobian(
                collision_data.nearest_point_to_obstacle_, starting_pose);

    // Gripper DOF direction to increase distance from the obstacle
    const auto J_distance = collision_data.obstacle_surface_normal_.transpose() * J_collision;
    const auto J_distance_pinv = Pinv(J_distance, SuggestedRcond());

    // First check if the intersection of valid motion is empty; if it is empty;
    // take the least violating motion (max speed, directly away from collision) as the result of projection
    {
        // If the distance is already far enough away, then we are already satisfying the constraint with zero movement,
        // thus the valid motion region cannot be empty
        const double current_distance = collision_data.distance_to_obstacle_;
        const double current_distance_change_needed = std::max(0.0, min_distance + GRIPPER_COLLISION_REPULSION_MARGIN - current_distance);
        const auto min_gripper_delta_to_satisfy_collision_constraint = J_distance_pinv * current_distance_change_needed;
        const double norm = GripperVelocity6dNorm(min_gripper_delta_to_satisfy_collision_constraint);
        if (norm > max_step_size)
        {
            return min_gripper_delta_to_satisfy_collision_constraint * (max_step_size / norm);
        }
    }

    // Second option, project to the delta ball first.
    // If, after projecting, we are violating the collision constraint,
    // then we need to find the "corner" between the circle and the half plane
    {
        const double delta_norm = GripperVelocity6dNorm(delta);
        if (delta_norm > max_step_size)
        {
            const auto max_motion_constrained_delta = delta * (max_step_size / delta_norm);
            const double displacement_towards_obstacle =  J_distance * max_motion_constrained_delta;
            const double current_distance = collision_data.distance_to_obstacle_ + displacement_towards_obstacle;
            if (current_distance >= min_distance)
            {
                return max_motion_constrained_delta;
            }
            else
            {
                const double current_distance_change_needed = min_distance + GRIPPER_COLLISION_REPULSION_MARGIN - current_distance;
                const auto collision_constrained_delta_update = J_distance_pinv * current_distance_change_needed;
                const auto collision_constrained_delta = max_motion_constrained_delta + collision_constrained_delta_update;
                const auto collision_constrained_displacement = J_distance * collision_constrained_delta;

                assert(collision_constrained_displacement.size() == 1);
                assert(collision_data.distance_to_obstacle_ + collision_constrained_displacement(0) >= min_distance);

                return projectToCornerOfCollisionAndMaxDeltaConstraints(
                            collision_constrained_delta,
                            J_collision,
                            max_step_size);
            }
        }
    }

    // We did not need to project to the delta ball, so project to the collision constraint.
    // If, after projecting, we are violating the max delta constraint
    // then we need to find the "corner" between the circle and the half plane
    {
        const double displacement_towards_obstacle =  J_distance * delta;
        const double current_distance = collision_data.distance_to_obstacle_ + displacement_towards_obstacle;
        if (current_distance < min_distance)
        {
            const double current_distance_change_needed = min_distance + GRIPPER_COLLISION_REPULSION_MARGIN - current_distance;
            const auto collision_constrained_delta = J_distance_pinv * current_distance_change_needed + delta;
            const auto collision_constrained_displacement = J_distance * collision_constrained_delta;

            assert(collision_constrained_displacement.size() == 1);
            assert(collision_data.distance_to_obstacle_ + collision_constrained_displacement(0) >= min_distance);

            const double delta_norm = GripperVelocity6dNorm(collision_constrained_delta);
            if (delta_norm <= max_step_size)
            {
                return collision_constrained_delta;
            }
            else
            {
                return projectToCornerOfCollisionAndMaxDeltaConstraints(
                            collision_constrained_delta,
                            J_collision,
                            max_step_size);
            }
        }
    }

    // If no constraints are violated, then return the original value unchanged
    return delta;
}

inline AllGrippersSinglePoseDelta projectToCollisionAndMaxDeltaConstraints(
        const AllGrippersSinglePose& starting_poses,
        const AllGrippersSinglePoseDelta& delta,
        const std::vector<CollisionData>& collision_data,
        const double min_distance,
        const double max_step_size)
{
    assert(delta.size() == starting_poses.size());
    assert(delta.size() == collision_data.size());

    AllGrippersSinglePoseDelta result(delta.size());
    for (size_t ind = 0; ind < delta.size(); ++ind)
    {
        result[ind] = projectToCollisionAndMaxDeltaConstraints(starting_poses[ind], delta[ind], collision_data[ind], min_distance, max_step_size);
    }
    return result;
}







DeformableController::OutputData StretchingConstraintController::solvedByGradientDescentProjectionViaGurobiMethod(const InputData& input_data)
{
    // Unpack the input data into its constituent parts
    const auto& world_state = input_data.world_current_state_;
    const auto& object_config = world_state.object_configuration_;
    const auto& grippers_poses = world_state.all_grippers_single_pose_;
    const ssize_t num_grippers = (ssize_t)(grippers_poses.size());
    assert(num_grippers == 2 && "This function is only intended to be used with 2 grippers");

    const VectorXd& desired_object_p_dot = input_data.desired_object_motion_.error_correction_.delta;
    const VectorXd& desired_p_dot_weight = input_data.desired_object_motion_.error_correction_.weight;

    // Check object current stretching status
    overstretch_ = false;
    if (input_data.handle_overstretch_)
    {
        const MatrixXd node_squared_distance = CalculateSquaredDistanceMatrix(object_config);
        overstretch_ = ((max_node_squared_distance_ - node_squared_distance).array() < 0.0).any();
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
            if (true)
            {
                if (overstretch_)
                {
                    Vector3d stretching_start = grippers_poses[gripper_idx] * stretching_constraint_data[gripper_idx].second;
                    Vector3d stretching_end = grippers_poses[gripper_idx] * (stretching_constraint_data[gripper_idx].second + 0.1 * stretching_constraint_data[gripper_idx].first);

    //                std::cerr << "stretching_data_first_" << gripper_idx + 1 << " = [" << stretching_constraint_data[gripper_idx].first.transpose() << " 0.0]';\n";
    //                std::cerr << "stretching_data_second_" << gripper_idx + 1 << " = [" << stretching_constraint_data[gripper_idx].second.transpose() << " 1.0]';\n";

    //                std::cout << "stretching_start_" << gripper_idx + 1 << " = [" << stretching_start.transpose() << "];\n";
    //                std::cout << "stretching_end_" << gripper_idx + 1 << " = [" << stretching_end.transpose() << "];\n";

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

    if (input_data.robot_jacobian_valid_)
    {
        int num_model_calls = 0;
        const ssize_t num_dof = input_data.world_current_state_.robot_configuration_.size();

        // Build the linear version of the constraints
        std::vector<RowVectorXd> linear_constraint_linear_terms;
        std::vector<double> linear_constraint_affine_terms;

        // Collision constraints
        {
            const size_t num_poi = input_data.full_robot_poi_collision_data_.size();

            linear_constraint_linear_terms.reserve(linear_constraint_linear_terms.size() + num_poi);
            linear_constraint_affine_terms.reserve(linear_constraint_affine_terms.size() + num_poi);

            for (size_t poi_ind = 0; poi_ind < num_poi; ++poi_ind)
            {
                const auto& collision_data = input_data.full_robot_poi_collision_data_[poi_ind];

                const MatrixXd& poi_jacobian = collision_data.second;
                linear_constraint_linear_terms.push_back(
                        -collision_data.first.obstacle_surface_normal_.transpose() * poi_jacobian);

                linear_constraint_affine_terms.push_back(
                        collision_data.first.distance_to_obstacle_ - robot_->min_controller_distance_to_obstacles_);

//                std::cout << "Poi ind: " << poi_ind << " Dist to obstacle: " << collision_data.first.distance_to_obstacle_ << " Min dist: " << robot_->min_controller_distance_to_obstacles_ << std::endl
//                          << "Jacobian:\n" << poi_jacobian << std::endl;
            }
        }

        // Stretching constraints:
        if (overstretch_)
        {
            assert(input_data.handle_overstretch_);

            for (ssize_t gripper_idx = 0; gripper_idx < num_grippers; ++gripper_idx)
            {
                Matrix<double, 3, 6> J_point_to_gripper;
                J_point_to_gripper.leftCols<3>() = Matrix3d::Identity();
                J_point_to_gripper.rightCols<3>() = -kinematics::skew(stretching_constraint_data[gripper_idx].second);
                const auto J_stretching = J_point_to_gripper * input_data.robot_jacobian_.middleRows<6>(6 * gripper_idx);

                for (size_t idx = 0; idx < pyramid_plane_normals[gripper_idx].size(); ++idx)
                {
                    linear_constraint_linear_terms.push_back(pyramid_plane_normals[gripper_idx][idx].transpose() * J_stretching);
                    linear_constraint_affine_terms.push_back(0.0);
                }
            }
        }

        // Speed constraints
        const double max_robot_dof_step_size = input_data.max_robot_dof_step_size_;
        const double max_grippers_step_size = input_data.max_grippers_step_size_;
        const VectorXd& joint_weights = input_data.robot_->getJointWeights();
        const MatrixXd jacobian = input_data.robot_->getGrippersJacobian();

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

        // Joint limits
        const VectorXd min_joint_delta = input_data.robot_->getJointLowerLimits() - input_data.world_current_state_.robot_configuration_;
        const VectorXd max_joint_delta = input_data.robot_->getJointUpperLimits() - input_data.world_current_state_.robot_configuration_;

        // Gradient descent params
        const double differencing_step_size = max_robot_dof_step_size / 10.0 / 4.0;
        const double initial_gradient_step_size = max_robot_dof_step_size / 2.0 / 4.0;

        bool converged = false;
        VectorXd robot_motion = VectorXd::Zero(num_dof);
        ObjectPointSet object_delta = model_->getObjectDelta(world_state, RobotMotionToGripperMotion(jacobian, robot_motion));
        double error = errorOfControlByPrediction(object_delta, desired_object_p_dot, desired_p_dot_weight);

        while (!converged)
        {
            VectorXd error_numerical_gradient(num_dof);
            VectorXd error_plus_h(num_dof);
            VectorXd error_minus_h(num_dof);

            for (ssize_t dof_ind = 0; dof_ind < num_dof; ++dof_ind)
            {
                VectorXd local_test_motion_plus_h = robot_motion;
                local_test_motion_plus_h(dof_ind) += differencing_step_size;
                const auto test_object_delta_plus_h = model_->getObjectDelta(world_state, RobotMotionToGripperMotion(jacobian, local_test_motion_plus_h));
                const auto test_error_plus_h = errorOfControlByPrediction(test_object_delta_plus_h, desired_object_p_dot, desired_p_dot_weight);

                VectorXd local_test_motion_minus_h = robot_motion;
                local_test_motion_minus_h(dof_ind) -= differencing_step_size;
                const auto test_object_delta_minus_h = model_->getObjectDelta(world_state, RobotMotionToGripperMotion(jacobian, local_test_motion_minus_h));
                const auto test_error_minus_h = errorOfControlByPrediction(test_object_delta_minus_h, desired_object_p_dot, desired_p_dot_weight);

                // Don't bother normalizing here, as we will do so at the end of the loop
                error_numerical_gradient(dof_ind) = test_error_plus_h - test_error_minus_h;
                error_plus_h(dof_ind) = test_error_plus_h;
                error_minus_h(dof_ind) = test_error_minus_h;
            }

            num_model_calls += (int)num_dof * 2;


            // Normalize the gradient as it is just giving us a direction to move, not a distance to move
            if (error_numerical_gradient.norm() > 1e-6)
            {
                error_numerical_gradient.normalize();
            }
            else
            {
                converged = true;
                continue;
            }

            // Take a step downhill, doing a line search to find a reasonable motion
            auto next_robot_motion = robot_motion;
            auto next_object_delta = model_->getObjectDelta(world_state, RobotMotionToGripperMotion(jacobian, next_robot_motion));
            double next_error = error;

            int downhill_attempt_ind = 0;
            double error_gradient_step_size = -initial_gradient_step_size;
            do
            {
                const auto potential_next_motion = robot_motion + error_gradient_step_size * error_numerical_gradient;
                next_robot_motion = findClosestValidPoint(
                            potential_next_motion,
                            jacobian,
                            max_grippers_step_size,
                            linear_constraint_linear_terms,
                            linear_constraint_affine_terms,
                            quadratic_constraint_quadratic_terms,
                            quadratic_constraint_linear_terms,
                            quadratic_constraint_affine_terms,
                            min_joint_delta,
                            max_joint_delta);

                next_object_delta = model_->getObjectDelta(world_state, RobotMotionToGripperMotion(jacobian, next_robot_motion));
                next_error = errorOfControlByPrediction(next_object_delta, desired_object_p_dot, desired_p_dot_weight);

                num_model_calls++;

                ++downhill_attempt_ind;
                error_gradient_step_size *= 0.7;
            }
            while (next_error > error && downhill_attempt_ind < 12);

            // If we could not make progress, then return whatever the last valid movement we had was
            if (next_error > error)
            {
                converged = true;
            }
            // Otherwise, accept the update
            else
            {
                converged = (error - next_error) < std::abs(error) * ERROR_CONVERGENCE_LIMIT;

                robot_motion = next_robot_motion;
                object_delta = next_object_delta;
                error = next_error;
            }
        }

        ARC_LOG(num_model_calls_, num_model_calls);

        return OutputData(RobotMotionToGripperMotion(jacobian, robot_motion), object_delta, robot_motion);
    }
    else
    {
        const auto& collision_data = world_state.gripper_collision_data_;

        int num_model_calls = 0;

        // Build the linear version of the constraints
        std::vector<RowVectorXd> linear_constraint_linear_terms_g0;
        std::vector<RowVectorXd> linear_constraint_linear_terms_g1;
        std::vector<double> linear_constraint_affine_terms_g0;
        std::vector<double> linear_constraint_affine_terms_g1;

        // Collision constraints
        {
            const auto J_collision_g0 = ComputeGripperMotionToPointMotionJacobian(collision_data[0].nearest_point_to_obstacle_, grippers_poses[0]);
            const auto J_distance_g0 = collision_data[0].obstacle_surface_normal_.transpose() * J_collision_g0;

            linear_constraint_linear_terms_g0.push_back(-1.0 * J_distance_g0);
            linear_constraint_affine_terms_g0.push_back(robot_->min_controller_distance_to_obstacles_ - collision_data[0].distance_to_obstacle_);

            const auto J_collision_g1 = ComputeGripperMotionToPointMotionJacobian(collision_data[1].nearest_point_to_obstacle_, grippers_poses[1]);
            const auto J_distance_g1 = collision_data[0].obstacle_surface_normal_.transpose() * J_collision_g1;

            linear_constraint_linear_terms_g1.push_back(-1.0 * J_distance_g1);
            linear_constraint_affine_terms_g1.push_back(robot_->min_controller_distance_to_obstacles_ - collision_data[1].distance_to_obstacle_);
        }

        // Stretching constraints
        if (overstretch_)
        {
            Matrix<double, 3, 6> J_stretching_g0;
            J_stretching_g0.leftCols<3>() = Matrix3d::Identity();
            J_stretching_g0.rightCols<3>() = -kinematics::skew(stretching_constraint_data[0].second);

            for (size_t ind = 0; ind < pyramid_plane_normals[0].size(); ++ind)
            {
                linear_constraint_linear_terms_g0.push_back(pyramid_plane_normals[0][ind].transpose() * J_stretching_g0);
                linear_constraint_affine_terms_g0.push_back(0.0);
            }

            Matrix<double, 3, 6> J_stretching_g1;
            J_stretching_g1.leftCols<3>() = Matrix3d::Identity();
            J_stretching_g1.rightCols<3>() = -kinematics::skew(stretching_constraint_data[1].second);

            for (size_t ind = 0; ind < pyramid_plane_normals[1].size(); ++ind)
            {
                linear_constraint_linear_terms_g1.push_back(pyramid_plane_normals[1][ind].transpose() * J_stretching_g1);
                linear_constraint_affine_terms_g1.push_back(0.0);
            }
        }

        const double max_individual_gripper_step_size = input_data.max_grippers_step_size_;
        const double differencing_step_size = max_individual_gripper_step_size / 10.0;
        const double initial_gradient_step_size = max_individual_gripper_step_size / 2.0;

        bool converged = false;
        AllGrippersSinglePoseDelta gripper_motion(num_grippers, kinematics::Vector6d::Zero());
        ObjectPointSet object_delta = model_->getObjectDelta(world_state, gripper_motion);
        double error = errorOfControlByPrediction(object_delta, desired_object_p_dot, desired_p_dot_weight);

        while (!converged)
        {
//            std::cout << "Error start of loop:  " << error << std::endl;

            VectorXd error_numerical_gradient(num_grippers * 6);
            VectorXd error_plus_h(num_grippers * 6);
            VectorXd error_minus_h(num_grippers * 6);
            for (ssize_t gripper_ind = 0; gripper_ind < num_grippers; ++gripper_ind)
            {
                for (size_t single_gripper_dir_ind = 0; single_gripper_dir_ind < 6; ++single_gripper_dir_ind)
                {
                    const auto test_input_ind = gripper_ind * 6 + single_gripper_dir_ind;

                    AllGrippersSinglePoseDelta local_test_motion_plus_h = gripper_motion;
                    local_test_motion_plus_h[gripper_ind](single_gripper_dir_ind) += differencing_step_size;
                    const auto test_object_delta_plus_h = model_->getObjectDelta(world_state, local_test_motion_plus_h);
                    const auto test_error_plus_h = errorOfControlByPrediction(test_object_delta_plus_h, desired_object_p_dot, desired_p_dot_weight);


                    AllGrippersSinglePoseDelta local_test_motion_minus_h = gripper_motion;
                    local_test_motion_minus_h[gripper_ind](single_gripper_dir_ind) -= differencing_step_size;
                    const auto test_object_delta_minus_h = model_->getObjectDelta(world_state, local_test_motion_minus_h);
                    const auto test_error_minus_h = errorOfControlByPrediction(test_object_delta_minus_h, desired_object_p_dot, desired_p_dot_weight);

                    // Don't bother normalizing here, as we will do so at the end of the loop
                    error_numerical_gradient(test_input_ind) = test_error_plus_h - test_error_minus_h;
                    error_plus_h(test_input_ind) = test_error_plus_h;
                    error_minus_h(test_input_ind) = test_error_minus_h;

//                    if (test_input_ind == 6)
//                    {
//                        vis_->visualizeObjectDelta("weirdness_gripper2_plus_x", object_config, object_config + 20.0 * test_object_delta_plus_h, Visualizer::Cyan(), 1);
//                    }


//                    const auto test_object_delta = model_->getObjectDelta(world_state, local_test_motion);
//                    const double test_error = errorOfControlByPrediction(test_object_delta, desired_object_p_dot, desired_p_dot_weight);
//                    error_numerical_gradient(test_input_ind) = test_error - error;
                }
            }



            num_model_calls += (int)num_grippers * 6 * 2;



            // Normalize the gradient as it is just giving us a direction to move, not a distance to move
            if (error_numerical_gradient.norm() > 1e-6)
            {
                error_numerical_gradient.normalize();
            }
            else
            {
//                std::cout << "\tObjective gradient is effectively flat, exiting loop" << std::endl;
                converged = true;
                continue;
            }

//            std::cout << "Error gradient:                     0th: " << error_numerical_gradient.head<6>().transpose() << "           1st: " << error_numerical_gradient.tail<6>().transpose() << std::endl;

            // Take a step downhill, doing a line search to find a reasonable motion
            auto next_gripper_motion = gripper_motion;
            auto next_object_delta = model_->getObjectDelta(world_state, next_gripper_motion);
            double next_error = error;

            int downhill_attempt_ind = 0;
            double error_gradient_step_size = -initial_gradient_step_size;
            do
            {
                const auto potential_next_motion = stepInDirection(gripper_motion, error_numerical_gradient, error_gradient_step_size);

//                const auto potential_next_object_delta = model_->getObjectDelta(world_state, potential_next_motion);
//                const auto potential_next_error = errorOfControlByPrediction(potential_next_object_delta, desired_object_p_dot, desired_p_dot_weight);

                auto projected_g0_motion = findClosestValidPoint(potential_next_motion[0], linear_constraint_linear_terms_g0, linear_constraint_affine_terms_g0, max_individual_gripper_step_size);
                auto projected_g1_motion = findClosestValidPoint(potential_next_motion[1], linear_constraint_linear_terms_g1, linear_constraint_affine_terms_g1, max_individual_gripper_step_size);

                if (projected_g0_motion.size() != 0)
                {
                    next_gripper_motion[0] = projected_g0_motion;
                }
                if (projected_g1_motion.size() != 0)
                {
                    next_gripper_motion[1] = projected_g1_motion;
                }

                next_object_delta = model_->getObjectDelta(world_state, next_gripper_motion);
                next_error = errorOfControlByPrediction(next_object_delta, desired_object_p_dot, desired_p_dot_weight);

                num_model_calls++;




                {
//                VectorXd manually_projected_g0_motion = potential_next_motion[0];
//                if (potential_next_motion[0].norm() > max_individual_gripper_step_size)
//                {
//                    manually_projected_g0_motion = potential_next_motion[0].normalized() * max_individual_gripper_step_size;
//                    std::cout << "Manual projection max delta constraint triggered\n";
//                }

//                VectorXd manually_projected_g1_motion = potential_next_motion[1];
//                if (potential_next_motion[1].norm() > max_individual_gripper_step_size)
//                {
//                    manually_projected_g1_motion = potential_next_motion[1].normalized() * max_individual_gripper_step_size;
//                    std::cout << "Manual projection max delta constraint triggered\n";
//                }
//                auto manual_proj_object_delta = model_->getObjectDelta(world_state, {manually_projected_g0_motion, manually_projected_g1_motion});
//                auto manual_proj_next_error = errorOfControlByPrediction(manual_proj_object_delta, desired_object_p_dot, desired_p_dot_weight);



//                if (projected_g0_motion.size() == 0)
//                {
//                    projected_g0_motion = kinematics::Vector6d::Zero();
//                }

//                if (projected_g1_motion.size() == 0)
//                {
//                    projected_g1_motion = kinematics::Vector6d::Zero();
//                }



//                auto collision_cnt_g0 = linear_constraint_linear_terms_g0[0] * potential_next_motion[0] + linear_constraint_affine_terms_g0[0];
//                auto collision_cnt_g1 = linear_constraint_linear_terms_g1[0] * potential_next_motion[1] + linear_constraint_affine_terms_g1[0];

//                std::cout << "Collision constraint G0 before: " << collision_cnt_g0
//                          << " Collision constraint G0 after:  " << linear_constraint_linear_terms_g0[0] * projected_g0_motion + linear_constraint_affine_terms_g0[0] << std::endl;

//                std::cout << "Collision constraint G1 before: " << collision_cnt_g1
//                          << " Collision constraint G0 after:  " << linear_constraint_linear_terms_g1[0] * projected_g1_motion + linear_constraint_affine_terms_g1[0] << std::endl;

//                if (collision_cnt_g0 >= 0.0)
//                {
//                    Vector3d collision_start = grippers_poses[0].translation();
//                    Vector3d collision_end = collision_start + grippers_poses[0].rotation() * (J_distance_g0.head<3>().transpose());
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 1);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 1);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 1);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 1);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 1);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 1);
//                }
//                else
//                {
//                    vis_->deleteObjects("collision_avoidance_vector", 1, 2);
//                    vis_->deleteObjects("collision_avoidance_vector", 1, 2);
//                    vis_->deleteObjects("collision_avoidance_vector", 1, 2);
//                    vis_->deleteObjects("collision_avoidance_vector", 1, 2);
//                    vis_->deleteObjects("collision_avoidance_vector", 1, 2);
//                    vis_->deleteObjects("collision_avoidance_vector", 1, 2);
//                }

//                if (collision_cnt_g1 >= 0.0)
//                {
//                    Vector3d collision_start = grippers_poses[1].translation();
//                    Vector3d collision_end = collision_start + grippers_poses[1].rotation() * (J_distance_g1.head<3>().transpose());
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 2);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 2);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 2);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 2);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 2);
//                    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 2);
//                }
//                else
//                {
//                    vis_->deleteObjects("collision_avoidance_vector", 2, 3);
//                    vis_->deleteObjects("collision_avoidance_vector", 2, 3);
//                    vis_->deleteObjects("collision_avoidance_vector", 2, 3);
//                    vis_->deleteObjects("collision_avoidance_vector", 2, 3);
//                    vis_->deleteObjects("collision_avoidance_vector", 2, 3);
//                    vis_->deleteObjects("collision_avoidance_vector", 2, 3);
//                }

//                std::cout << "Num constraints: " << linear_constraint_linear_terms_g0.size() << std::endl;
//                if (linear_constraint_linear_terms_g0.size() > 1)
//                {
//                    std::cout << "Stretching constraints must be active" << std::endl;
//                    std::cout << std::endl;
//                    std::cout << std::endl;
//                }

//                std::cout << "Attempt ind: " << downhill_attempt_ind << "    Gradient step size: " << error_gradient_step_size << std::endl;

//                std::cout << "Error plus h change                 0th: " << error_plus_h.head<6>().transpose() - kinematics::Vector6d::Ones().transpose() * error                     << "     1st: " << error_plus_h.tail<6>().transpose() - kinematics::Vector6d::Ones().transpose() * error << std::endl;
//                std::cout << "Error minus h change                0th: " << error_minus_h.head<6>().transpose() - kinematics::Vector6d::Ones().transpose() * error                    << "     1st: " << error_minus_h.tail<6>().transpose() - kinematics::Vector6d::Ones().transpose() * error << std::endl;
//                std::cout << "Error gradient:                     0th: " << error_numerical_gradient.head<6>().transpose()           << "           1st: " << error_numerical_gradient.tail<6>().transpose() << std::endl;
//                std::cout << "attempted delta:                    0th: " << delta.head<6>().transpose()                                    << "     1st: " << delta.tail<6>().transpose() << std::endl;
//                std::cout << "Gurobi based delta:                 0th: " << (projected_g0_motion - gripper_motion[0]).transpose()          << "     1st: " << (projected_g1_motion - gripper_motion[1]).transpose() << std::endl;
//                std::cout << "manual constraint based delta:      0th: " << (manually_projected_g0_motion - gripper_motion[0]).transpose() << "     1st: " << (manually_projected_g1_motion - gripper_motion[1]).transpose() << std::endl;


//                std::cout << "Starting motion:  " << print(gripper_motion) << std::endl;
//                std::cout << "Potential motion: " << print(potential_next_motion) << std::endl;
//                std::cout << "Gurobi proj    :  " << print(next_gripper_motion) << std::endl;
//                std::cout << "Manual proj    :  " << print({manually_projected_g0_motion, manually_projected_g1_motion}) << std::endl;

//                std::cout << "Starting norms:  " << GripperVelocity6dNorm(gripper_motion[0]) << " " << GripperVelocity6dNorm(gripper_motion[0]) << std::endl;
//                std::cout << "Potential norms: " << GripperVelocity6dNorm(potential_next_motion[0]) << " " << GripperVelocity6dNorm(potential_next_motion[0]) << std::endl;
//                std::cout << "Gurobi   norms:  " << GripperVelocity6dNorm(next_gripper_motion[0]) << " " << GripperVelocity6dNorm(next_gripper_motion[0]) << std::endl;
//                std::cout << "manual   norms:  " << GripperVelocity6dNorm(manually_projected_g0_motion) << " " << GripperVelocity6dNorm(manually_projected_g0_motion) << std::endl;

//                std::cout << "Starting error:       " << error << std::endl;
//                std::cout << "Potential next error: " << potential_next_error << std::endl;
//                std::cout << "Gurobi proj    error: " << next_error << std::endl;
//                std::cout << "Manual proj    error: " << manual_proj_next_error << std::endl;
//                std::cout << std::endl;


//                vis_->visualizeRope("gurobi_prediction", object_config + next_object_delta, Visualizer::Cyan(), 1);
//                vis_->visualizeRope("gurobi_prediction", object_config + next_object_delta, Visualizer::Cyan(), 1);
//                vis_->visualizeRope("gurobi_prediction", object_config + next_object_delta, Visualizer::Cyan(), 1);


//                const VectorVector3d potential_gripper_translation_starts =
//                {
//                    grippers_poses[0].translation(),
//                    grippers_poses[1].translation()
//                };
//                const VectorVector3d potential_gripper_translation_ends =
//                {
//                    grippers_poses[0].translation() + 100.0 * potential_next_motion[0].head<3>(),
//                    grippers_poses[1].translation() + 100.0 * potential_next_motion[1].head<3>()
//                };
//                vis_->visualizeLines("pre_projection_gripper_translation", potential_gripper_translation_starts, potential_gripper_translation_ends, Visualizer::Red(), 1);
//                vis_->visualizeLines("pre_projection_gripper_translation", potential_gripper_translation_starts, potential_gripper_translation_ends, Visualizer::Red(), 1);
//                vis_->visualizeLines("pre_projection_gripper_translation", potential_gripper_translation_starts, potential_gripper_translation_ends, Visualizer::Red(), 1);


//                const VectorVector3d gurobi_translation_starts =
//                {
//                    grippers_poses[0].translation(),
//                    grippers_poses[1].translation()
//                };
//                const VectorVector3d gubrobi_translation_ends =
//                {
//                    grippers_poses[0].translation() + 100.0 * next_gripper_motion[0].head<3>(),
//                    grippers_poses[1].translation() + 100.0 * next_gripper_motion[1].head<3>()
//                };
//                vis_->visualizeLines("gurobi_gripper_translation", gurobi_translation_starts, gubrobi_translation_ends, Visualizer::Blue(), 1);
//                vis_->visualizeLines("gurobi_gripper_translation", gurobi_translation_starts, gubrobi_translation_ends, Visualizer::Blue(), 1);
//                vis_->visualizeLines("gurobi_gripper_translation", gurobi_translation_starts, gubrobi_translation_ends, Visualizer::Blue(), 1);
                }


                ++downhill_attempt_ind;
                error_gradient_step_size *= 0.7;
            }
            while (next_error > error && downhill_attempt_ind < 12);


            // If we could not make progress, then return whatever the last valid movement we had was
            if (next_error > error)
            {
//                std::cout << "\tUnable to find downhill step" << std::endl;
                converged = true;
            }
            // Otherwise, accept the update
            else
            {
                converged = (error - next_error) < std::abs(error) * ERROR_CONVERGENCE_LIMIT;

                gripper_motion = next_gripper_motion;
                object_delta = next_object_delta;
                error = next_error;
//                std::cout << "\tAccepting motion update: " << print(gripper_motion) << std::endl;
            }

//            if (converged)
//            {
//                std::cout << "\tConverged, exiting loop" << std::endl;
//            }

//            std::cout << "Error end of loop:    " << error << std::endl;
        }

        ARC_LOG(num_model_calls_, num_model_calls);

        return OutputData(gripper_motion, object_delta, VectorXd());
    }
}

//////////////////////////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////////////////////////

kinematics::Vector6d StretchingConstraintController::singleGripperPoseDeltaSampler(
        const double max_delta)
{
    const double x_trans = Interpolate(-max_delta, max_delta, uniform_unit_distribution_(*generator_));
    const double y_trans = Interpolate(-max_delta, max_delta, uniform_unit_distribution_(*generator_));
    const double z_trans = Interpolate(-max_delta, max_delta, uniform_unit_distribution_(*generator_));

//    const double x_rot = Interpolate(-max_delta, max_delta, uniform_unit_distribution_(*generator_));
//    const double y_rot = Interpolate(-max_delta, max_delta, uniform_unit_distribution_(*generator_));
//    const double z_rot = Interpolate(-max_delta, max_delta, uniform_unit_distribution_(*generator_));

    kinematics::Vector6d random_sample = kinematics::Vector6d::Zero();

    double raw_norm = std::sqrt(std::pow(x_trans, 2) + std::pow(y_trans, 2) + std::pow(z_trans, 2));

    if (raw_norm > 0.000001 && (fix_step_ || raw_norm > max_delta))
    {
        random_sample(0) = x_trans / raw_norm * max_delta;
        random_sample(1) = y_trans / raw_norm * max_delta;
        random_sample(2) = z_trans / raw_norm * max_delta;
    }
    else
    {
        random_sample(0) = x_trans;
        random_sample(1) = y_trans;
        random_sample(2) = z_trans;
    }

//    random_sample(3) = x_rot;
//    random_sample(4) = y_rot;
//    random_sample(5) = z_rot;

    return ClampGripperPoseDeltas(random_sample, max_delta);
}

AllGrippersSinglePoseDelta StretchingConstraintController::allGripperPoseDeltaSampler(
        const ssize_t num_grippers,
        const double max_delta)
{
    AllGrippersSinglePoseDelta grippers_motion_sample(num_grippers, kinematics::Vector6d::Zero());

    // if sample_count_ < 0, return all-sampled motion, otherwise, return one-for-each-time sample
    if (sample_count_ < 0)
    {
        for (ssize_t ind = 0; ind < num_grippers; ind++)
        {
            grippers_motion_sample[ind] = singleGripperPoseDeltaSampler(max_delta);
        }
    }
    else if (sample_count_ < num_grippers)
    {
        grippers_motion_sample[sample_count_] = singleGripperPoseDeltaSampler(max_delta);
    }
    else
    {
        assert(false && "This code should not be reachable");
    }
    return grippers_motion_sample;
}


kinematics::Vector6d StretchingConstraintController::getConstraintAwareGripperDeltaSample(
                    const Isometry3d& gripper_pose,
                    const CollisionData& collision_data,
                    const double max_delta,
                    const std::pair<Vector3d, Vector3d>& stretching_correction_data)
{
    const auto J_collision = ComputeGripperMotionToPointMotionJacobian(collision_data.nearest_point_to_obstacle_, gripper_pose);
    const auto J_distance = collision_data.obstacle_surface_normal_.transpose() * J_collision;
    // Returns true if the constraint is satisfied
    const auto collision_constraint_fn = [&] (const kinematics::Vector6d& gripper_delta)
    {
        return robot_->min_controller_distance_to_obstacles_
                - collision_data.distance_to_obstacle_ - J_distance * gripper_delta;
    };

//    vis_->visualizeGripper("sampling_gripper", gripper_pose, Visualizer::Blue(), 1);

//    std::cout << "J_coll:\n" << J_collision << std::endl;
//    std::cout << "J_dist:\n" << J_distance << std::endl;

//    Vector3d stretching_start = gripper_pose * stretching_correction_data.second;
//    Vector3d stretching_end = gripper_pose * (stretching_correction_data.second + stretching_correction_data.first);
//    vis_->visualizeLines("stretching_correction_vector", {stretching_start}, {stretching_end}, Visualizer::Red(), 1);

//    Vector3d collision_start = gripper_pose.translation();
//    Vector3d collision_end = collision_start + gripper_pose.rotation() *(J_distance.head<3>().transpose());
//    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 1);

    kinematics::Vector6d sample = kinematics::Vector6d::Zero();
    bool collision_satisfied = (collision_constraint_fn(sample) < 0.0);
    bool stretching_satisified = (evaluateStretchingConstraint(stretching_correction_data, sample) < 0.0);
    bool valid_sample =  collision_satisfied && stretching_satisified;
    while (!valid_sample)
    {
        sample = singleGripperPoseDeltaSampler(max_delta * 0.8);

//        Vector3d sample_start = gripper_pose.translation();
//        Vector3d sample_end = sample_start + gripper_pose.rotation() * sample.head<3>() * 10.0;
//        vis_->visualizeLines("gripper_delta_sample", {sample_start}, {sample_end}, Visualizer::Magenta(), 1);

        collision_satisfied = (collision_constraint_fn(sample) < 0.0);
        stretching_satisified = (evaluateStretchingConstraint(stretching_correction_data, sample) < 0.0);

//        std::cout << "Collision constraint:  " << collision_constraint_fn(sample) << std::endl;
//        std::cout << "Stretching constraint: " << evaluateStretchingConstraint(stretching_correction_data, sample) << std::endl;

        valid_sample = collision_satisfied && stretching_satisified;
    }

    return sample;
}

kinematics::Vector6d StretchingConstraintController::getFeasibleGripperDeltaGurobi(
        const Isometry3d& gripper_pose,
        const CollisionData& collision_data,
        const double max_delta,
        const std::pair<Vector3d, Vector3d>& stretching_correction_data) const
{
    std::vector<RowVectorXd> linear_constraint_linear_terms;
    std::vector<double> linear_constraint_affine_terms;

    // Add the collision constraint: Ax <= b
    const auto J_collision = ComputeGripperMotionToPointMotionJacobian(collision_data.nearest_point_to_obstacle_, gripper_pose);
    const auto J_distance = collision_data.obstacle_surface_normal_.transpose() * J_collision;
    linear_constraint_linear_terms.push_back(-1.0 * J_distance);
    linear_constraint_affine_terms.push_back(collision_data.distance_to_obstacle_ - robot_->min_controller_distance_to_obstacles_ - GRIPPER_COLLISION_REPULSION_MARGIN);

    vis_->visualizeGripper("sampling_gripper", gripper_pose, Visualizer::Blue(), 1);

//    std::cout << "J_coll:\n" << J_collision << std::endl;
    std::cout << "J_dist:\n" << J_distance << std::endl;

    Vector3d stretching_start = gripper_pose * stretching_correction_data.second;
    Vector3d stretching_end = gripper_pose * (stretching_correction_data.second + stretching_correction_data.first);
    vis_->visualizeLines("stretching_correction_vector", {stretching_start}, {stretching_end}, Visualizer::Red(), 1);

    Vector3d collision_start = gripper_pose.translation();
    Vector3d collision_end = collision_start + gripper_pose.rotation() *(J_distance.head<3>().transpose());
    vis_->visualizeLines("collision_avoidance_vector", {collision_start}, {collision_end}, Visualizer::Green(), 1);

    // Add the stretching constraint (if active)
    if (overstretch_)
    {
        const auto pyramid_plane_normals = ConvertConeToPyramid(stretching_correction_data.first, stretching_cosine_threshold_);
        Matrix<double, 3, 6> J_stretching;
        J_stretching.leftCols<3>() = Matrix3d::Identity();
        J_stretching.rightCols<3>() = kinematics::skew(stretching_correction_data.second);

        for (size_t ind = 0; ind < pyramid_plane_normals.size(); ++ind)
        {
            std::cout << "Testing plane normal, should be negative: " << pyramid_plane_normals[ind].transpose() * stretching_correction_data.first << std::endl;

            linear_constraint_linear_terms.push_back(pyramid_plane_normals[ind].transpose() * J_stretching);
            linear_constraint_affine_terms.push_back(0.0);
        }
    }

    const std::pair<VectorXd, double> potential_feasible_point =
            minimizeConstraintViolations(6, linear_constraint_linear_terms, linear_constraint_affine_terms, max_delta * 0.99);

    std::cout << "Collision constraint val:  " << robot_->min_controller_distance_to_obstacles_ - collision_data.distance_to_obstacle_ - J_distance * potential_feasible_point.first << std::endl;
    std::cout << "Stretching constraint val: " << evaluateStretchingConstraint(stretching_correction_data, potential_feasible_point.first) << std::endl;

    Vector3d sample_start = gripper_pose.translation();
    Vector3d sample_end = sample_start + gripper_pose.rotation() * potential_feasible_point.first.head<3>() * 10.0;
    vis_->visualizeLines("gripper_delta_sample", {sample_start}, {sample_end}, Visualizer::Magenta(), 1);

    if (potential_feasible_point.second >= 0.0)
    {
//        std::cout <<
//        auto tmp = Pinv(J_distance, SuggestedRcond()) * (robot_->min_controller_distance_to_obstacles_ - collision_data.distance_to_obstacle_);
    }

    assert(potential_feasible_point.second < 0.0);
    return potential_feasible_point.first;
}






double StretchingConstraintController::errorOfControlByPrediction(
        const ObjectPointSet predicted_object_p_dot,
        const VectorXd& desired_object_p_dot,
        const VectorXd& desired_p_dot_weight) const
{
    const Map<const VectorXd> prediction_as_vector(predicted_object_p_dot.data(), desired_object_p_dot.size());
    const auto individual_error = (prediction_as_vector- desired_object_p_dot).cwiseAbs2();
    return std::sqrt(individual_error.dot(desired_p_dot_weight));
}


void StretchingConstraintController::visualize_stretching_vector(
        const ObjectPointSet& object_configuration)
{
    switch (deformable_type_)
    {
        case ROPE:
        {
            visualize_rope_stretching_vector(object_configuration);
            break;
        }
        case CLOTH:
        {
            visualize_cloth_stretching_vector(object_configuration);
            break;
        }
        default:
        {
            assert(false && "visualize stretching vector of neither cloth nor rope");
            break;
        }
    }
}

void StretchingConstraintController::visualize_rope_stretching_vector(
        const ObjectPointSet& object_configuration)
{
    const ssize_t num_nodes = object_configuration.cols();
    const ssize_t start_node = 1;
    const ssize_t end_node = num_nodes - 2;

    Vector3d first_correction_vector =
            (object_configuration.block<3, 1>(0, start_node + 1)
                - object_configuration.block<3, 1>(0, start_node));
    first_correction_vector = first_correction_vector/first_correction_vector.norm();

    Vector3d second_correction_vector =
            (object_configuration.block<3, 1>(0, end_node - 1)
                - object_configuration.block<3, 1>(0, end_node));
    second_correction_vector = second_correction_vector/second_correction_vector.norm();

    VectorVector3d line_starts;
    VectorVector3d line_ends;
    line_starts.push_back(object_configuration.block<3,1>(0, 0));
    line_starts.push_back(object_configuration.block<3,1>(0, num_nodes-1));
    line_ends.push_back(line_starts[0] + 0.5 * first_correction_vector);
    line_ends.push_back(line_starts[1] + 0.5 * second_correction_vector);

    vis_->visualizeLines("gripper overstretch motion",
                        line_starts,
                        line_ends,
                        Visualizer::Orange());
}

void StretchingConstraintController::visualize_cloth_stretching_vector(
        const ObjectPointSet& object_configuration)
{
    // Assume knowing there are two grippers.
    assert(grippers_data_.size()==2 || "grippers size is not 2, stretching vector visualization not developed");

    const StretchingVectorInfo& first_stretching_vector_info = grippers_data_[0].stretching_vector_info_;
    const std::vector<long>& first_from_nodes = first_stretching_vector_info.from_nodes_;
    const std::vector<long>& first_to_nodes = first_stretching_vector_info.to_nodes_;
    const std::vector<double>& first_contribution = first_stretching_vector_info.node_contribution_;

    const StretchingVectorInfo& second_stretching_vector_info = grippers_data_[1].stretching_vector_info_;
    const std::vector<long>& second_from_nodes = second_stretching_vector_info.from_nodes_;
    const std::vector<long>& second_to_nodes = second_stretching_vector_info.to_nodes_;
    const std::vector<double>& second_contribution = second_stretching_vector_info.node_contribution_;

    Vector3d first_correction_vector = MatrixXd::Zero(3,1);
    for (size_t stretching_ind = 0; stretching_ind < first_from_nodes.size(); stretching_ind++)
    {
        first_correction_vector +=
                first_contribution[stretching_ind] *
                (object_configuration.block<3, 1>(0, first_to_nodes[stretching_ind])
                 - object_configuration.block<3, 1>(0, first_from_nodes[stretching_ind]));
    }
    Vector3d second_correction_vector = MatrixXd::Zero(3,1);
    for (size_t stretching_ind = 0; stretching_ind < second_from_nodes.size(); stretching_ind++)
    {
        second_correction_vector +=
                second_contribution[stretching_ind] *
                (object_configuration.block<3, 1>(0, second_to_nodes[stretching_ind])
                 - object_configuration.block<3, 1>(0, second_from_nodes[stretching_ind]));
    }

    VectorVector3d line_starts;
    VectorVector3d line_ends;
    line_starts.push_back(object_configuration.block<3,1>(0, first_from_nodes[0]));
    line_starts.push_back(object_configuration.block<3,1>(0, second_from_nodes[0]));
    line_ends.push_back(line_starts[0] + 10 * first_correction_vector);
    line_ends.push_back(line_starts[1] + 10 * second_correction_vector);

    vis_->visualizeLines("gripper overstretch motion",
                        line_starts,
                        line_ends,
                        Visualizer::Orange());
}

void StretchingConstraintController::visualize_gripper_motion(
        const AllGrippersSinglePose& current_gripper_pose,
        const AllGrippersSinglePoseDelta& gripper_motion)
{
    const auto grippers_test_poses = kinematics::applyTwist(current_gripper_pose, gripper_motion);
    VectorVector3d line_starts;
    VectorVector3d line_ends;

    for (size_t gripper_ind = 0; gripper_ind < current_gripper_pose.size(); gripper_ind++)
    {
        line_starts.push_back(current_gripper_pose[gripper_ind].translation());
        line_ends.push_back(current_gripper_pose[gripper_ind].translation() + 100 * (grippers_test_poses[gripper_ind].translation() - current_gripper_pose[gripper_ind].translation()));
    }

    vis_->visualizeLines("gripper motion",
                         line_starts,
                         line_ends,
                         Visualizer::Olive());
}

double StretchingConstraintController::gripperCollisionCheckHelper(
        const AllGrippersSinglePose& current_gripper_pose,
        const AllGrippersSinglePoseDelta& test_gripper_motion) const
{
    assert(false && "This function is not used for the gradient descent method");

    const auto grippers_test_poses = kinematics::applyTwist(current_gripper_pose, test_gripper_motion);

    double min_collision_distance = std::numeric_limits<double>::infinity();

    for (size_t gripper_idx = 0; gripper_idx < grippers_test_poses.size(); ++gripper_idx)
    {
        const auto gripper_pos = grippers_test_poses[gripper_idx].translation();
        // Changed from legacy to new projection here
        const auto collision_result = environment_sdf_->EstimateDistance3d(gripper_pos);
        if (collision_result.first < min_collision_distance)
        {
            min_collision_distance = collision_result.first;
        }
    }

    return min_collision_distance;
}

bool StretchingConstraintController::gripperCollisionCheckResult(
        const AllGrippersSinglePose& current_gripper_pose,
        const AllGrippersSinglePoseDelta& test_gripper_motion) const
{
    const double min_dis_to_obstacle = gripperCollisionCheckHelper(current_gripper_pose, test_gripper_motion);
    const bool collision_violation = (min_dis_to_obstacle < distance_to_obstacle_threshold_);
    return collision_violation;
}


// Returns true if the constraint is violated
bool StretchingConstraintController::stretchingDetection(
        const InputData& input_data,
        const AllGrippersSinglePoseDelta& test_gripper_motion)
{
    return (stretchingFunctionEvaluation(input_data, test_gripper_motion) > 0.0);
}

// Note that NOMAD wants all constraints in the form c(x) <= 0
// If the calc'd value is larger than the threshold, then the gripper motion is sufficently pointed
// in the direction needed to reduce/not cause more stretching
double StretchingConstraintController::stretchingFunctionEvaluation(
        const InputData& input_data,
        const AllGrippersSinglePoseDelta& test_gripper_motion)
{
//    assert(false && "This function is not used for the gradient descent method");
    if (test_gripper_motion.size() != 2)
    {
        assert(false && "num of grippers not match");
    }

    switch (deformable_type_)
    {
        case ROPE:
        {
            return stretching_cosine_threshold_ - ropeTwoGripperStretchingHelper(input_data, test_gripper_motion);
        }
        case CLOTH:
        {
            return stretching_cosine_threshold_ - clothTwoGripperStretchingHelper(input_data, test_gripper_motion);
        }
        default:
        {
            assert(false && "deformable_type is neither rope nor cloth");
            return 0.0;
        }
    }
}

double StretchingConstraintController::ropeTwoGripperStretchingHelper(
        const InputData& input_data,
        const AllGrippersSinglePoseDelta& test_gripper_motion)
{
//    assert(false && "This function is not used for the gradient descent method" && "Verify the math in this implementation");

    double stretching_sum = 0.0;
    double stretching_cos = 1.0; // return a value > stretching_cos_threshold

    if (overstretch_)
    {
        const ObjectPointSet& object_configuration = input_data.world_current_state_.object_configuration_;
        const AllGrippersSinglePose& current_gripper_pose = input_data.world_current_state_.all_grippers_single_pose_;
        const ssize_t num_nodes = object_configuration.cols();

        const ssize_t start_node = 0;
        const ssize_t end_node = num_nodes - 1;

        Vector3d first_correction_vector =
                (object_configuration.block<3, 1>(0, start_node + 1)
                    - object_configuration.block<3, 1>(0, start_node));
        first_correction_vector = first_correction_vector/first_correction_vector.norm();

        Vector3d second_correction_vector =
                (object_configuration.block<3, 1>(0, end_node - 1)
                    - object_configuration.block<3, 1>(0, end_node));
        second_correction_vector = second_correction_vector/second_correction_vector.norm();

        VectorVector3d stretching_correction_vector;
        stretching_correction_vector.push_back(first_correction_vector);
        stretching_correction_vector.push_back(second_correction_vector);

        const auto grippers_test_poses = kinematics::applyTwist(current_gripper_pose, test_gripper_motion);
        double sum_resulting_motion_norm = 0.0;

        switch (gripper_controller_type_)
        {
            case StretchingConstraintControllerSolverType::RANDOM_SAMPLING:
            {
                if (sample_count_ > -1)
                {
                    for (size_t gripper_ind = 0; gripper_ind < current_gripper_pose.size(); gripper_ind++)
                    {
                        Vector3d resulting_gripper_motion = grippers_test_poses[gripper_ind].translation()
                                - current_gripper_pose[gripper_ind].translation();
                       stretching_sum += resulting_gripper_motion.dot(stretching_correction_vector[gripper_ind]);
                        sum_resulting_motion_norm += resulting_gripper_motion.norm();
                    }
                    if (sum_resulting_motion_norm != 0.0)
                    {
                        stretching_cos = stretching_sum / sum_resulting_motion_norm;
                    }
                }
                else
                {
                    for (size_t gripper_ind = 0; gripper_ind < current_gripper_pose.size(); gripper_ind++)
                    {
                        Vector3d resulting_gripper_motion = grippers_test_poses[gripper_ind].translation()
                                - current_gripper_pose[gripper_ind].translation();
                        stretching_sum += resulting_gripper_motion.dot(stretching_correction_vector[gripper_ind])
                                / resulting_gripper_motion.norm();
                    }
                    stretching_cos = stretching_sum / (double)(current_gripper_pose.size());
               }
                break;
            }
//            case StretchingAvoidanceControllerSolverType::NOMAD_OPTIMIZATION:
            default:
            {
                for (size_t gripper_ind = 0; gripper_ind < current_gripper_pose.size(); gripper_ind++)
                {
                    Vector3d resulting_gripper_motion = grippers_test_poses[gripper_ind].translation()
                            - current_gripper_pose[gripper_ind].translation();
                    stretching_sum += resulting_gripper_motion.dot(stretching_correction_vector[gripper_ind])
                            / resulting_gripper_motion.norm();
                }
                stretching_cos = stretching_sum / (double)(current_gripper_pose.size());
                break;
            }
//            default:
//            {
//                assert(false && "not valid controller solving type");
//                break;
//            }
        }

    }
    return stretching_cos;

}

double StretchingConstraintController::clothTwoGripperStretchingHelper(
        const InputData& input_data,
        const AllGrippersSinglePoseDelta& test_gripper_motion)
{
//    assert(false && "This function is not used for the gradient descent method");

    // Assume knowing there are two grippers.
    assert(grippers_data_.size() == 2 || "grippers size is not 2, stretching vector visualization not developed");

    std::cout << print(test_gripper_motion) << std::endl;

    const ObjectPointSet& object_configuration = input_data.world_current_state_.object_configuration_;
    const AllGrippersSinglePose& current_grippers_pose = input_data.world_current_state_.all_grippers_single_pose_;
    const AllGrippersSinglePose grippers_test_poses = kinematics::applyTwist(current_grippers_pose, test_gripper_motion);
    const Isometry3d& gripper0_current_pose = current_grippers_pose[0];
    const Isometry3d& gripper1_current_pose = current_grippers_pose[1];
    const Isometry3d& gripper0_test_pose = grippers_test_poses[0];
    const Isometry3d& gripper1_test_pose = grippers_test_poses[1];

    // If the object is not overstretched already, then the constraint is not active, indicited by a value of 1.0
    // 1.0 indicates the least stretching possible, with both grippers moving in the direction needed to reduce stretching
    if (!overstretch_)
    {
        return 1.0;
    }

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
        const auto from_node = object_configuration.block<3, 1>(0, first_from_nodes[stretching_ind]);
        const auto to_node = object_configuration.block<3, 1>(0, first_to_nodes[stretching_ind]);
        const auto node_delta = to_node - from_node;
        first_correction_vector += first_contribution[stretching_ind] * node_delta;
        point_on_first_gripper_before_motion += first_contribution[stretching_ind] * from_node;
    }

    for (size_t stretching_ind = 0; stretching_ind < second_from_nodes.size(); stretching_ind++)
    {
        const auto from_node = object_configuration.block<3, 1>(0, second_from_nodes[stretching_ind]);
        const auto to_node = object_configuration.block<3, 1>(0, second_to_nodes[stretching_ind]);
        const auto node_delta = to_node - from_node;
        second_correction_vector += second_contribution[stretching_ind] * node_delta;
        point_on_second_gripper_before_motion += second_contribution[stretching_ind] * from_node;

    }

    // Normalize the vectors to get direction only; will be zero if the input norm is 0.
    VectorVector3d per_gripper_stretching_correction_vector(2, Vector3d::Zero());
    {
        const auto first_vector_norm = first_correction_vector.norm();
        if (first_vector_norm > 1e-6)
        {
            per_gripper_stretching_correction_vector[0] = first_correction_vector / first_vector_norm;
        }

        const auto second_vector_norm = second_correction_vector.norm();
        if (second_vector_norm > 1e-6)
        {
            per_gripper_stretching_correction_vector[1] = second_correction_vector / second_vector_norm;
        }
    }

    // Transform the input points based on the motion of each gripper
    const Vector3d first_point_in_first_gripper_frame   = gripper0_current_pose.inverse() * point_on_first_gripper_before_motion;
    const Vector3d second_point_in_second_gripper_frame = gripper1_current_pose.inverse() * point_on_second_gripper_before_motion;
    const Vector3d point_on_first_gripper_after_motion  = gripper0_test_pose * first_point_in_first_gripper_frame;
    const Vector3d point_on_second_gripper_after_motion = gripper1_test_pose * second_point_in_second_gripper_frame;

    std::vector<Vector3d> point_motion_vector(2, Vector3d::Zero());
    {
        const auto first_point_motion = point_on_first_gripper_after_motion - point_on_first_gripper_before_motion;
        const auto first_vector_norm = first_point_motion.norm();
        if (first_vector_norm > 1e-6)
        {
            point_motion_vector[0] = first_point_motion / first_vector_norm;
        }

        const auto second_point_motion = point_on_second_gripper_after_motion - point_on_second_gripper_before_motion;
        const auto second_vector_norm = second_point_motion.norm();
        if (second_vector_norm > 1e-6)
        {
            point_motion_vector[1] = second_point_motion / second_vector_norm;
        }
    }

    double stretching_cos = 0.0;
    // sample_count_ > -1 means only sample one gripper each time
    if ((sample_count_ > -1) && (gripper_controller_type_ == StretchingConstraintControllerSolverType::RANDOM_SAMPLING))
    {
        assert(sample_count_ < (int)(per_gripper_stretching_correction_vector.size()));
        const Vector3d& point_movement_direction = point_motion_vector[sample_count_];
        stretching_cos = point_movement_direction.dot(per_gripper_stretching_correction_vector[sample_count_]);
    }
    else
    {
        stretching_cos = 0.0;
        for (size_t gripper_ind = 0; gripper_ind < current_grippers_pose.size(); gripper_ind++)
        {
            std::cout << "point direction: " << point_motion_vector[gripper_ind].transpose() << "    stretching_correction_vec: " << per_gripper_stretching_correction_vector[gripper_ind].transpose() << std::endl;

            const Vector3d& point_movement_direction = point_motion_vector[gripper_ind];
            stretching_cos += point_movement_direction.dot(per_gripper_stretching_correction_vector[gripper_ind]);
        }
        stretching_cos /= (double)(current_grippers_pose.size());

        std::cout << "result: " << stretching_cos << std::endl;
    }

    if (std::isnan(stretching_cos))
    {
        std::cout << "Test gripper motion: " << print(test_gripper_motion) << std::endl;
        std::cout << std::endl;

        std::cout << "Point on first gripper:        " << point_on_first_gripper_before_motion.transpose() << std::endl;
        std::cout << "Point on second gripper:       " << point_on_second_gripper_before_motion.transpose() << std::endl;
        std::cout << "First correction vector:       " << first_correction_vector.normalized().transpose() << std::endl;
        std::cout << "Second correction vector:      " << second_correction_vector.normalized().transpose() << std::endl;
        std::cout << "Point on first gripper after:  " << point_on_first_gripper_after_motion.transpose() << std::endl;
        std::cout << "Point on second gripper after: " << point_on_second_gripper_after_motion.transpose() << std::endl;

        std::cout << "trans0: " << (grippers_test_poses[0].translation() - current_grippers_pose[0].translation()).transpose() << std::endl;
        std::cout << "trans1: " << (grippers_test_poses[1].translation() - current_grippers_pose[1].translation()).transpose() << std::endl;

        assert(!std::isnan(stretching_cos));
    }

    return stretching_cos;
}



double StretchingConstraintController::evaluateStretchingConstraint(const std::pair<Vector3d, Vector3d>& stretching_constraint_data, const kinematics::Vector6d& gripper_delta) const
{
    const auto& stretching_reduction_vector              = stretching_constraint_data.first;
    const auto& vector_from_gripper_to_translation_point = stretching_constraint_data.second;

    if (!overstretch_)
    {
        return -1000.0;
    }
    else
    {
        const Vector3d r_dot = gripper_delta.head<3>() + gripper_delta.tail<3>().cross(vector_from_gripper_to_translation_point);
        const double r_dot_norm = r_dot.norm();
        if (r_dot_norm < 1e-6)
        {
            return stretching_cosine_threshold_ - 1.0;
        }
        else
        {
            const double cos_angle = stretching_reduction_vector.dot(r_dot) / r_dot_norm;
            return stretching_cosine_threshold_ - cos_angle;
        }
    }
}

// Note that the returned vectors and points are in gripper frame
// result.first is the direction that we want to move the point
// result.second is the point that we are constrainting the motion of
std::vector<std::pair<Vector3d, Vector3d>> StretchingConstraintController::stretchingCorrectionVectorsAndPoints(const InputData& input_data) const
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
	std::vector<std::pair<Vector3d, Vector3d>> pass;
	return pass;
}

std::vector<std::pair<Vector3d, Vector3d>> StretchingConstraintController::ropeTwoGrippersStretchingCorrectionVectorsAndPoints(const InputData& input_data) const
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

std::vector<std::pair<Vector3d, Vector3d>> StretchingConstraintController::clothTwoGrippersStretchingCorrectionVectorsAndPoints(const InputData& input_data) const
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

void StretchingConstraintController::visualizeCone(const Vector3d& cone_direction, const double min_normalized_dot_product, const Isometry3d& pose, const int marker_id)
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
