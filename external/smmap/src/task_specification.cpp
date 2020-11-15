#include <thread>
#include <numeric>
#include <arc_utilities/arc_exceptions.hpp>
#include <arc_utilities/log.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/timing.hpp>
#include <deformable_manipulation_msgs/messages.h>
#include <deformable_manipulation_experiment_params/ros_params.hpp>
#include <deformable_manipulation_experiment_params/utility.hpp>

#include "smmap/task_specification.h"
#include "smmap/task_specification_implementions.h"

using namespace smmap;
using namespace arc_utilities;

#pragma message "Magic number - Stretching weight multiplication factor here"
static constexpr double STRETCHING_WEIGHT_MULTIPLICATION_FACTOR = 2000.0;
#pragma message "Magic number - Step size and min progress for forward projection of dijkstras field following"
static constexpr int VECTOR_FIELD_FOLLOWING_NUM_MICROSTEPS      = 10;
static constexpr double VECTOR_FIELD_FOLLOWING_MIN_PROGRESS     = 1e-6;

//#define ENABLE_PROJECTION

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Task Specification /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Static builder function
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TaskSpecification::Ptr TaskSpecification::MakeTaskSpecification(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
{
    const TaskType task_type = GetTaskType(*nh);

    switch (task_type)
    {
        case TaskType::ROPE_TABLE_LINEAR_MOTION:
        case TaskType::CLOTH_TABLE_LINEAR_MOTION:
        case TaskType::ROPE_TABLE_PENTRATION:
        case TaskType::CLOTH_PLACEMAT_LINEAR_MOTION:
            return std::make_shared<ModelAccuracyTestTask>(nh, ph, vis);

        case TaskType::CLOTH_COLAB_FOLDING:
            return std::make_shared<ClothColabFolding>(nh, ph, vis);

        case TaskType::ROPE_SIMPLE_COVERAGE_TWO_GRIPPERS:
        case TaskType::ROPE_CYLINDER_COVERAGE:
        case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
            return std::make_shared<RopeDirectCoverage>(nh, ph, vis);

        case TaskType::CLOTH_TABLE_COVERAGE:
            return std::make_shared<ClothDirectCoverage>(nh, ph, vis);

        case TaskType::ROPE_ENGINE_ASSEMBLY_LIVE:
        case TaskType::ROPE_GENERIC_DIJKSTRAS_COVERAGE:
            return std::make_shared<RopeDistanceBasedCorrespondences>(nh, ph, vis);

        case TaskType::ROPE_MAZE:
        case TaskType::ROPE_ZIG_MATCH:
        case TaskType::ROPE_HOOKS:
        case TaskType::ROPE_HOOKS_SIMPLE:
        case TaskType::ROPE_HOOKS_MULTI:
        case TaskType::ROPE_GENERIC_FIXED_COVERAGE:
        case TaskType::ROPE_ENGINE_ASSEMBLY:
            return std::make_shared<RopeFixedCorrespondences>(nh, ph, vis);

        case TaskType::CLOTH_PLACEMAT:
        case TaskType::CLOTH_MFLAG:
        case TaskType::CLOTH_CYLINDER_COVERAGE:
        case TaskType::CLOTH_WAFR:
        case TaskType::CLOTH_WALL:
        case TaskType::CLOTH_SINGLE_POLE:
        case TaskType::CLOTH_DOUBLE_SLIT:
        case TaskType::CLOTH_HOOKS_SIMPLE:
        case TaskType::CLOTH_HOOKS_COMPLEX:
        case TaskType::CLOTH_GENERIC_DIJKSTRAS_COVERAGE:
            return std::make_shared<ClothDistanceBasedCorrespondences>(nh, ph, vis);

        case TaskType::CLOTH_GENERIC_FIXED_COVERAGE:
            return std::make_shared<ClothFixedCorrespondences>(nh, ph, vis);

        default:
            throw_arc_exception(std::invalid_argument, "Invalid task type in MakeTaskSpecification(), this should not be possible");
            return nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor to initialize objects that all TaskSpecifications share
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TaskSpecification::TaskSpecification(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis,
        const bool is_dijkstras_type_task)
    : first_step_calculated_(false)
    , first_step_last_simtime_calced_(NAN)

    , current_error_calculated_(false)
    , current_error_last_simtime_calced_(NAN)
    , current_error_(NAN)

    , desired_motion_scaling_factor_(GetDesiredMotionScalingFactor(*ph))

    , deformable_type_(GetDeformableType(*nh))
    , task_type_(GetTaskType(*nh))
    , is_dijkstras_type_task_(is_dijkstras_type_task)

    , nh_(nh)
    , ph_(ph)
    , vis_(vis)

    , grippers_data_(GetGrippersData(*nh_))
    , object_initial_node_distance_(EigenHelpers::CalculateDistanceMatrix(GetObjectInitialConfiguration(*nh_)))
    , num_nodes_(object_initial_node_distance_.cols())
    , default_deformability_(GetDefaultDeformability(*ph_))
    , collision_scaling_factor_(GetCollisionScalingFactor(*ph_))
    , max_stretch_factor_(GetMaxStretchFactor(*ph_))
    , max_band_length_(GetMaxBandLength(*ph_))
    , max_time_(GetMaxTime(*ph_))
{}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Virtual function wrappers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<Visualizer::NamespaceId> TaskSpecification::visualizeDeformableObject(
        const std::string& marker_name,
        const ObjectPointSet& object_configuration,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    return visualizeDeformableObject_impl(marker_name, object_configuration, color, id);
}

std::vector<Visualizer::NamespaceId> TaskSpecification::visualizeDeformableObject(
        const std::string& marker_name,
        const ObjectPointSet& object_configuration,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id) const
{
    return visualizeDeformableObject_impl(marker_name, object_configuration, colors, id);
}

double TaskSpecification::calculateError(const WorldState& world_state)
{
    if (current_error_last_simtime_calced_ != world_state.sim_time_)
    {
        current_error_calculated_.store(false);
    }

    if (current_error_calculated_.load())
    {
        return current_error_;
    }
    else
    {
        std::lock_guard<std::mutex> lock(current_error_mtx_);
        if (current_error_calculated_.load())
        {
            return current_error_;
        }
        else
        {
            GlobalStopwatch(RESET);
            current_error_ = calculateError_impl(world_state);
            ROS_INFO_STREAM_NAMED("task_specification", "Calculated error in                  " << GlobalStopwatch(READ) << " seconds");

            current_error_last_simtime_calced_ = world_state.sim_time_;
            current_error_calculated_.store(true);
            return current_error_;
        }
    }
}

ObjectDeltaAndWeight TaskSpecification::calculateObjectErrorCorrectionDelta(
        const WorldState& world_state)
{
    return calculateObjectErrorCorrectionDelta_impl(world_state);
}

bool TaskSpecification::taskDone(
        const WorldState& world_state)
{
    return taskDone_impl(world_state);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double TaskSpecification::defaultDeformability() const
{
    return default_deformability_;
}

double TaskSpecification::collisionScalingFactor() const
{
    return collision_scaling_factor_;
}

double TaskSpecification::maxStretchFactor() const
{
    return max_stretch_factor_;
}

double TaskSpecification::maxBandLength() const
{
    return max_band_length_;
}

double TaskSpecification::maxTime() const
{
    return max_time_;
}


inline void addStrechingCorrectionVector(
        ObjectDeltaAndWeight& stretching_correction,
        const ObjectPointSet& object_configuration,
        const ssize_t first_node,
        const ssize_t second_node,
        const double node_distance_delta)
{
    // The correction vector points from the first node to the second node,
    // and is half the length of the "extra" distance
    const Eigen::Vector3d correction_vector = 0.5 * node_distance_delta
            * (object_configuration.col(second_node)
                - object_configuration.col(first_node));

    stretching_correction.delta.segment<3>(3 * first_node) += correction_vector;
    stretching_correction.delta.segment<3>(3 * second_node) -= correction_vector;

    // Set the weight to be the stretch distance of the worst offender
    const double first_node_max_stretch = std::max(stretching_correction.weight(3 * first_node), STRETCHING_WEIGHT_MULTIPLICATION_FACTOR * node_distance_delta);
    stretching_correction.weight(3 * first_node)     = first_node_max_stretch;
    stretching_correction.weight(3 * first_node + 1) = first_node_max_stretch;
    stretching_correction.weight(3 * first_node + 2) = first_node_max_stretch;

    // Set the weight to be the stretch distance of the worst offender
    const double second_node_max_stretch = std::max(stretching_correction.weight(3 * second_node), STRETCHING_WEIGHT_MULTIPLICATION_FACTOR * node_distance_delta);
    stretching_correction.weight(3 * second_node)     = second_node_max_stretch;
    stretching_correction.weight(3 * second_node + 1) = second_node_max_stretch;
    stretching_correction.weight(3 * second_node + 2) = second_node_max_stretch;
}

bool TaskSpecification::stretchingConstraintViolated(
        const ssize_t first_node_ind,
        const Eigen::Vector3d& first_node,
        const ssize_t second_node_ind,
        const Eigen::Vector3d& second_node) const
{
    const double dist = (first_node - second_node).norm();
    const double max_dist = object_initial_node_distance_(first_node_ind, second_node_ind) * maxStretchFactor();
    return dist > max_dist;
}


/**
 * @brief TaskSpecification::calculateStretchingCorrectionDeltaFullyConnected
 * @param object_configuration
 * @param visualize
 * @return
 */
ObjectDeltaAndWeight TaskSpecification::calculateStretchingCorrectionDeltaFullyConnected(
        const ObjectPointSet& object_configuration,
        bool visualize) const
{
    ObjectDeltaAndWeight stretching_correction(num_nodes_ * 3);
    const double max_stretch_factor = maxStretchFactor();
    const Eigen::MatrixXd object_current_node_distance = EigenHelpers::CalculateDistanceMatrix(object_configuration);

    EigenHelpers::VectorVector3d vis_start_points;
    EigenHelpers::VectorVector3d vis_end_points;

    for (ssize_t first_node = 0; first_node < num_nodes_; ++first_node)
    {
        for (ssize_t second_node = first_node + 1; second_node < num_nodes_; ++second_node)
        {
            const double max_dist = object_initial_node_distance_(first_node, second_node) * max_stretch_factor;
            const double dist = object_current_node_distance(first_node, second_node);
            if (max_dist < dist)
            {
                addStrechingCorrectionVector(stretching_correction,
                                             object_configuration,
                                             first_node,
                                             second_node,
                                             dist - object_initial_node_distance_(first_node, second_node));

                if (visualize)
                {
                    vis_start_points.push_back(object_configuration.col(first_node));
                    vis_end_points.push_back(object_configuration.col(second_node));
                }
            }
        }
    }

    if (visualize)
    {
        vis_->visualizeLines("stretching_lines", vis_start_points, vis_end_points, Visualizer::Blue());
    }

    return stretching_correction;
}

/**
 * @brief TaskSpecification::calculateStretchingCorrectionDeltaPairwise
 * @param object_configuration
 * @param visualize
 * @return
 */
ObjectDeltaAndWeight TaskSpecification::calculateStretchingCorrectionDeltaPairwise(
        const ObjectPointSet& object_configuration,
        bool visualize) const
{
    ObjectDeltaAndWeight stretching_correction(num_nodes_ * 3);
    const double max_stretch_factor = maxStretchFactor();

    EigenHelpers::VectorVector3d vis_start_points;
    EigenHelpers::VectorVector3d vis_end_points;

    for (ssize_t first_node = 0; first_node < num_nodes_; ++first_node)
    {
        for (ssize_t second_node : getNodeNeighbours(first_node))
        {
            // Only calculate corrections for nodes beyond the first so as to avoid duplicating work
            if (first_node < second_node)
            {
                const double max_dist = object_initial_node_distance_(first_node, second_node) * max_stretch_factor;
                const double dist = (object_configuration.col(second_node) - object_configuration.col(first_node)).norm();
                if (max_dist < dist)
                {
                    addStrechingCorrectionVector(
                                stretching_correction,
                                object_configuration,
                                first_node,
                                second_node,
                                dist - object_initial_node_distance_(first_node, second_node));

                    if (visualize)
                    {
                        vis_start_points.push_back(object_configuration.col(first_node));
                        vis_end_points.push_back(object_configuration.col(second_node));
                    }
                }
            }
        }
    }

    if (visualize)
    {
        vis_->visualizeLines("stretching_lines", vis_start_points, vis_end_points, Visualizer::Blue());
    }

    return stretching_correction;
}

/**
 * @brief TaskSpecification::calculateStretchingCorrectionDelta
 * @param world_state
 * @param visualize
 * @return
 */
ObjectDeltaAndWeight TaskSpecification::calculateStretchingCorrectionDelta(
        const WorldState& world_state,
        bool visualize) const
{
    return calculateStretchingCorrectionDeltaPairwise(world_state.object_configuration_, visualize);
}


/**
 * @brief TaskSpecification::combineErrorCorrectionAndStretchingCorrection
 * @param error_correction
 * @param stretching_correction
 * @return
 */
ObjectDeltaAndWeight TaskSpecification::combineErrorCorrectionAndStretchingCorrection(
        const ObjectDeltaAndWeight& error_correction,
        const ObjectDeltaAndWeight& stretching_correction) const
{
    ObjectDeltaAndWeight combined(num_nodes_ * 3);

    for (ssize_t node_ind = 0; node_ind < num_nodes_ * 3; node_ind += 3)
    {
        const Eigen::Vector3d error_correction_perpendicular =
                EigenHelpers::VectorRejection(stretching_correction.delta.segment<3>(node_ind),
                                               error_correction.delta.segment<3>(node_ind) );

        combined.delta.segment<3>(node_ind) = stretching_correction.delta.segment<3>(node_ind) + error_correction_perpendicular;
        combined.weight.segment<3>(node_ind) = stretching_correction.weight.segment<3>(node_ind) + error_correction.weight.segment<3>(node_ind);
    }

    // Normalize the weights for later use
    const double combined_normalizer = combined.weight.maxCoeff();
    if (combined_normalizer > 0)
    {
        combined.weight /= combined_normalizer;
    }

    return combined;
}


DesiredDirection TaskSpecification::calculateDesiredDirection(const WorldState& world_state)
{
    if (first_step_last_simtime_calced_ != world_state.sim_time_)
    {
        first_step_calculated_.store(false);
    }

    if (first_step_calculated_.load())
    {
        return first_step_desired_motion_;
    }
    else
    {
        std::lock_guard<std::mutex> lock(first_step_mtx_);
        if (first_step_calculated_.load())
        {
            return first_step_desired_motion_;
        }
        else
        {
            GlobalStopwatch(RESET);
            first_step_desired_motion_.error_correction_ = calculateObjectErrorCorrectionDelta(world_state);
            ROS_INFO_STREAM_NAMED("task_specification", "Found best error correction delta in " << GlobalStopwatch(READ) << " seconds");

            GlobalStopwatch(RESET);
            const bool visualize_stretching_lines = false;
            first_step_desired_motion_.stretching_correction_ = calculateStretchingCorrectionDelta(world_state, visualize_stretching_lines);
            ROS_INFO_STREAM_NAMED("task_specification", "Found stretching correction delta in " << GlobalStopwatch(READ) << " seconds");

            GlobalStopwatch(RESET);
            first_step_desired_motion_.combined_correction_ = combineErrorCorrectionAndStretchingCorrection(
                        first_step_desired_motion_.error_correction_, first_step_desired_motion_.stretching_correction_);
            ROS_INFO_STREAM_NAMED("task_specification", "Combined deltas in                   " << GlobalStopwatch(READ) << " seconds");

            first_step_last_simtime_calced_ = world_state.sim_time_;
            first_step_calculated_.store(true);

            // Scale down the motion so that we are not asking for movements well beyond anything reasonable
            first_step_desired_motion_.error_correction_.delta /= desired_motion_scaling_factor_;
            first_step_desired_motion_.stretching_correction_.delta /= desired_motion_scaling_factor_;
            first_step_desired_motion_.combined_correction_.delta /= desired_motion_scaling_factor_;

            return first_step_desired_motion_;
        }
    }
}

std::vector<ssize_t> TaskSpecification::getNodeNeighbours(const ssize_t node) const
{
    assert(node < num_nodes_);
    return getNodeNeighbours_impl(node);
}

const std::vector<long>& TaskSpecification::getGripperAttachedNodesIndices(const size_t gripper_idx) const
{
    assert(gripper_idx < grippers_data_.size());
    return grippers_data_[gripper_idx].node_indices_;
}

std::vector<Visualizer::NamespaceId> TaskSpecification::visualizeDeformableObject_impl(
        const std::string& marker_name,
        const ObjectPointSet& object_configuration,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    switch (deformable_type_)
    {
        case ROPE:
            return vis_->visualizeRope(marker_name, object_configuration, color, id);

        case CLOTH:
            return vis_->visualizeCloth(marker_name, object_configuration, color, id);

        default:
            assert(false && "Imposibru!");
    }
	std::vector<Visualizer::NamespaceId> pass;
	return pass;
}

std::vector<Visualizer::NamespaceId> TaskSpecification::visualizeDeformableObject_impl(
        const std::string& marker_name,
        const ObjectPointSet& object_configuration,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id) const
{
    switch (deformable_type_)
    {
        case ROPE:
            return vis_->visualizeRope(marker_name, object_configuration, colors, id);

        case CLOTH:
            return vis_->visualizeCloth(marker_name, object_configuration, colors, id);

        default:
            assert(false && "Imposibru!");
    }
	std::vector<Visualizer::NamespaceId> pass;
	return pass;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Model Accuracy Test Task ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ModelAccuracyTestTask::ModelAccuracyTestTask(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : TaskSpecification(nh, ph, vis, false)
{}

double ModelAccuracyTestTask::calculateError_impl(
        const WorldState& world_state)
{
    (void)world_state;
    return 0.0;
}

ObjectDeltaAndWeight ModelAccuracyTestTask::calculateObjectErrorCorrectionDelta_impl(
        const WorldState& world_state)
{
    (void)world_state;
    return ObjectDeltaAndWeight(num_nodes_ * 3);
}

std::vector<ssize_t> ModelAccuracyTestTask::getNodeNeighbours_impl(const ssize_t node) const
{
    (void)node;
    return std::vector<ssize_t>(0);
}

bool ModelAccuracyTestTask::taskDone_impl(
        const WorldState& world_state)
{
    (void)world_state;
    return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Coverage Task //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CoverageTask::CoverageTask(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis,
        const bool is_dijkstras_type_task)
    : TaskSpecification(nh, ph, vis, is_dijkstras_type_task)
    , sdf_(GetEnvironmentSDF(*nh_))
    , work_space_grid_(sdf_->GetOriginTransform(),
                       sdf_->GetFrame(),
                       GetWorldXStep(*nh_),
                       GetWorldYStep(*nh_),
                       GetWorldZStep(*nh_),
                       GetWorldXNumSteps(*nh_),
                       GetWorldYNumSteps(*nh_),
                       GetWorldZNumSteps(*nh_))
    , cover_points_(GetCoverPoints(*nh_))
    , cover_point_normals_(GetCoverPointNormals(*nh_))
    , num_cover_points_(cover_points_.cols())
    , error_threshold_along_normal_(GetErrorThresholdAlongNormal(*ph_))
    , error_threshold_distance_to_normal_(GetErrorThresholdDistanceToNormal(*ph_))
    , error_threshold_task_done_(GetErrorThresholdTaskDone(*ph_))
{
    assert(sdf_->GetFrame() == GetWorldFrameName());
    assert(work_space_grid_.getFrame() == GetWorldFrameName());
}

bool CoverageTask::pointIsCovered(const ssize_t cover_idx, const Eigen::Vector3d& test_point) const
{
    const auto cover_point          = cover_points_.col(cover_idx);
    const auto cover_point_normal   = cover_point_normals_.col(cover_idx);
    const std::pair<double, double> distances = EigenHelpers::DistanceToLine(cover_point, cover_point_normal, test_point);
    return (distances.first < error_threshold_distance_to_normal_) && (std::abs(distances.second) < error_threshold_along_normal_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Direct Coverage Task ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DirectCoverageTask::DirectCoverageTask(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : CoverageTask(nh, ph, vis, false)
{}

double DirectCoverageTask::calculateError_impl(const WorldState& world_state)
{
    Eigen::VectorXd error(num_cover_points_);

    // for every cover point, find the nearest deformable object point
    #pragma omp parallel for
    for (ssize_t cover_idx = 0; cover_idx < num_cover_points_; ++cover_idx)
    {
        // find the closest deformable object point
        ssize_t min_idx;
        double min_dist;
        bool covered;
        std::tie(min_idx, min_dist, covered) = findNearestObjectPoint(world_state, cover_idx);

        // Only count the distance if the point is not considered covered
        error(cover_idx) = covered ? 0.0 : min_dist;
    }

    return error.sum();
}

ObjectDeltaAndWeight DirectCoverageTask::calculateObjectErrorCorrectionDelta_impl(
        const WorldState& world_state)
{
    const ObjectPointSet& object_configuration = world_state.object_configuration_;
    ObjectDeltaAndWeight desired_object_delta(num_nodes_ * 3);

    // for every target point, find the nearest deformable object point
    for (ssize_t cover_idx = 0; cover_idx < num_cover_points_; ++cover_idx)
    {
        // Find the closest deformable object point
        ssize_t min_idx;
        double min_dist;
        bool covered;
        std::tie(min_idx, min_dist, covered) = findNearestObjectPoint(world_state, cover_idx);

        // Only count the distance if the point is not considered covered
        if (!covered)
        {
            desired_object_delta.delta.segment<3>(min_idx * 3) =
                    desired_object_delta.delta.segment<3>(min_idx * 3)
                    + (cover_points_.col(cover_idx) - object_configuration.col(min_idx));

            const double weight = std::max(desired_object_delta.weight(min_idx * 3), min_dist);
            desired_object_delta.weight(min_idx * 3) = weight;
            desired_object_delta.weight(min_idx * 3 + 1) = weight;
            desired_object_delta.weight(min_idx * 3 + 2) = weight;
        }
    }

    return desired_object_delta;
}

/**
 * @brief DirectCoverageTask::findNearestObjectPoint
 * @param object_configuration
 * @param cover_ind
 * @return index in object_configuration of the closest point
 *         the matching distance,
 *         if the point is consisered 'covered'
 */
std::tuple<ssize_t, double, bool> DirectCoverageTask::findNearestObjectPoint(
        const WorldState& world_state,
        const ssize_t cover_idx) const
{
    const Eigen::Vector3d& cover_point = cover_points_.col(cover_idx);

    // find the closest deformable object point
    bool covered = false;
    ssize_t min_idx = -1;
    double min_dist_squared = std::numeric_limits<double>::infinity();
    for (ssize_t deformable_idx = 0; deformable_idx < num_nodes_; ++deformable_idx)
    {
        const Eigen::Vector3d& deformable_point = world_state.object_configuration_.col(deformable_idx);
        const double new_dist_squared = (cover_point - deformable_point).squaredNorm();
        if (new_dist_squared < min_dist_squared)
        {
            min_dist_squared = new_dist_squared;
            min_idx = deformable_idx;
        }

        covered |= pointIsCovered(cover_idx, deformable_point);
    }

    return std::make_tuple(min_idx, std::sqrt(min_dist_squared), covered);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Dijkstra's Coverage Task ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DijkstrasCoverageTask::DijkstrasCoverageTask(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : CoverageTask(nh, ph, vis, true)
    , current_correspondences_calculated_(false)
    , current_correspondences_last_simtime_calced_(std::numeric_limits<double>::quiet_NaN())
    , current_correspondences_(num_nodes_)
    , visualize_correspondences_(GetVisualizeCorrespondences(*ph_))
{
    GetFreeSpaceGraph(*nh_, free_space_graph_, cover_ind_to_free_space_graph_ind_);
    assert(cover_ind_to_free_space_graph_ind_.size() == (size_t)num_cover_points_);
    if (vis_->visualizationsEnabled() && GetVisualizeFreeSpaceGraph(*ph_))
    {
        visualizeFreeSpaceGraph();
    }

    const bool need_to_run_dijkstras = !loadDijkstrasResults();
    if (need_to_run_dijkstras)
    {
        ROS_INFO_STREAM_NAMED("coverage_task", "Generating " << num_cover_points_ << " Dijkstra's solutions");
        GlobalStopwatch(RESET);
        dijkstras_results_.resize((size_t)num_cover_points_);
        #pragma omp parallel for schedule(guided)
        for (size_t cover_ind = 0; cover_ind < (size_t)num_cover_points_; ++cover_ind)
        {
            const int64_t free_space_graph_ind = cover_ind_to_free_space_graph_ind_[cover_ind];
            const auto result = arc_dijkstras::SimpleDijkstrasAlgorithm<Eigen::Vector3d>::PerformDijkstrasAlgorithm(free_space_graph_, free_space_graph_ind);
            dijkstras_results_[cover_ind] = result.second;

//            std::cout << "Cover idx: " << cover_ind << " Dijktsras size: " << dijkstras_results_[cover_ind].first.size() << std::endl;
//            visualizeIndividualDijkstrasResult(cover_ind, Eigen::Vector3d(-50.0, -50.0, -50.0));
        }
        ROS_INFO_NAMED("coverage_task", "Writing solutions to file");

        saveDijkstrasResults();
        ROS_INFO_STREAM_NAMED("coverage_task", "Found solutions in " << GlobalStopwatch(READ) << " seconds");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Dijkstras Coverage Task - Virtual function wrappers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const DijkstrasCoverageTask::Correspondences& DijkstrasCoverageTask::getCoverPointCorrespondences(
        const WorldState& world_state)
{
    if (current_correspondences_last_simtime_calced_ != world_state.sim_time_)
    {
        current_correspondences_calculated_.store(false);
    }

    if (current_correspondences_calculated_.load())
    {
        return current_correspondences_;
    }
    else
    {
        std::lock_guard<std::mutex> lock(current_correspondences_mtx_);
        if (current_correspondences_calculated_.load())
        {
            return current_correspondences_;
        }
        else
        {
            GlobalStopwatch(RESET);
            current_correspondences_ = getCoverPointCorrespondences_impl(world_state);

            assert(current_correspondences_.uncovered_target_points_idxs_.size()
                   == current_correspondences_.uncovered_target_points_distances_.size());

            assert((ssize_t)current_correspondences_.correspondences_.size() == num_nodes_);
            assert((ssize_t)current_correspondences_.correspondences_next_step_.size() == num_nodes_);
            assert((ssize_t)current_correspondences_.correspondences_distances_.size() == num_nodes_);
            assert((ssize_t)current_correspondences_.correspondences_is_covered_.size() == num_nodes_);

            size_t total_correspondences = 0;
            for (size_t deform_idx = 0; (ssize_t)deform_idx < num_nodes_; ++deform_idx)
            {
                const size_t current_num_correspondences = current_correspondences_.correspondences_[deform_idx].size();

                assert(current_correspondences_.correspondences_next_step_[deform_idx].size() == current_num_correspondences);
                assert(current_correspondences_.correspondences_distances_[deform_idx].size() == current_num_correspondences);
                assert(current_correspondences_.correspondences_is_covered_[deform_idx].size() == current_num_correspondences);

                total_correspondences += current_num_correspondences;
            }
            assert((ssize_t)total_correspondences == num_cover_points_);

            ROS_INFO_STREAM_NAMED("task_specification", "Calculated correspondences in        " << GlobalStopwatch(READ) << " seconds");

            if (visualize_correspondences_)
            {
                EigenHelpers::VectorVector3d start_points;
                EigenHelpers::VectorVector3d end_points;
                for (size_t deform_idx = 0; (ssize_t)deform_idx < num_nodes_; ++deform_idx)
                {
                    const Eigen::Vector3d& current_node_pos = world_state.object_configuration_.col((ssize_t)deform_idx);
                    const auto& current_deform_idx_correspondences = current_correspondences_.correspondences_[deform_idx];
                    for (size_t correspondence_idx = 0; correspondence_idx < current_deform_idx_correspondences.size(); ++correspondence_idx )
                    {
                        const ssize_t cover_point_idx = current_deform_idx_correspondences[correspondence_idx];

                        start_points.push_back(current_node_pos);
                        end_points.push_back(cover_points_.col(cover_point_idx));
                    }
                }
                vis_->visualizeLines("correspondences", start_points, end_points, Visualizer::Yellow(), 1, 0.001);
            }

            current_correspondences_last_simtime_calced_ = world_state.sim_time_;
            current_correspondences_calculated_.store(true);
            return current_correspondences_;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Dijkstras Coverage Task - Interface functions used externally
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief DijkstrasCoverageTask::findPathFromObjectToTarget
 * @param world_state
 * @param max_steps
 * @return
 */
std::vector<EigenHelpers::VectorVector3d> DijkstrasCoverageTask::findPathFromObjectToTarget(
        const WorldState& world_state,
        const size_t max_steps)
{
    const Correspondences& correspondences = getCoverPointCorrespondences(world_state);
    const ObjectPointSet& object_configuration = world_state.object_configuration_;

    std::vector<EigenHelpers::VectorVector3d> dijkstras_paths(num_nodes_);
    // Next, for each deformable point, follow the (combined) Dijkstras field
    for (size_t deformable_ind = 0; (ssize_t)deformable_ind < num_nodes_; ++deformable_ind)
    {
        dijkstras_paths[deformable_ind] =
                followCoverPointAssignments(
                    object_configuration.col(deformable_ind),
                    correspondences.correspondences_[deformable_ind],
                    max_steps);
    }

    return dijkstras_paths;
}

/**
 * @brief DijkstrasCoverageTask::calculateErrorCorrectionDeltaFixedCorrespondences
 * @param world_state
 * @param correspondences
 * @return
 */
ObjectDeltaAndWeight DijkstrasCoverageTask::calculateErrorCorrectionDeltaFixedCorrespondences(
        const WorldState& world_state,
        const std::vector<std::vector<ssize_t>>& correspondences)
{
    ObjectDeltaAndWeight desired_object_delta(num_nodes_ * 3);

    for (ssize_t deform_idx = 0; deform_idx < num_nodes_; ++deform_idx)
    {
        const std::vector<ssize_t>& current_correspondences     = correspondences[deform_idx];
        #ifdef ENABLE_PROJECTION
        const Eigen::Vector3d& deformable_point                 = sdf_->ProjectOutOfCollision3d(world_state.object_configuration_.col(deform_idx));
        #else
        const Eigen::Vector3d& deformable_point                 = world_state.object_configuration_.col(deform_idx);
        #endif
        const ssize_t deformable_point_idx_in_free_space_graph  = work_space_grid_.worldPosToGridIndexClamped(deformable_point);

        for (size_t correspondence_idx = 0; correspondence_idx < current_correspondences.size(); ++correspondence_idx)
        {
            const ssize_t cover_idx = current_correspondences[correspondence_idx];
            const Eigen::Vector3d& cover_point = cover_points_.col(cover_idx);
            const double straight_line_distance = (deformable_point - cover_point).norm();

            Eigen::Vector3d target_point;
            double distance_to_cover_point;
            // If we are within 1 (rounded) grid cell, go straight towards the cover point
            if (straight_line_distance <= work_space_grid_.minStepDimension() * std::sqrt(2.0))
            {
                target_point = cover_point;
                distance_to_cover_point = straight_line_distance;
            }
            // Otherwise use the Dijkstra's result
            else
            {
                // Collect the Dijkstras next step and distance
                const auto& dijkstras_individual_result         = dijkstras_results_[(size_t)cover_idx];
                const ssize_t target_idx_in_free_space_graph    = dijkstras_individual_result.first[deformable_point_idx_in_free_space_graph];
                const double dijkstras_distance                 = dijkstras_individual_result.second[deformable_point_idx_in_free_space_graph];

                target_point = free_space_graph_.getNode(target_idx_in_free_space_graph).getValue();
                distance_to_cover_point = dijkstras_distance;
            }

            desired_object_delta.delta.segment<3>(deform_idx * 3) += (target_point - deformable_point);

            const double weight = std::max(desired_object_delta.weight(deform_idx * 3), distance_to_cover_point);
            desired_object_delta.weight(deform_idx * 3) = weight;
            desired_object_delta.weight(deform_idx * 3 + 1) = weight;
            desired_object_delta.weight(deform_idx * 3 + 2) = weight;
        }
    }

    return desired_object_delta;
}

std::vector<double> DijkstrasCoverageTask::averageDijkstrasDistanceBetweenGrippersAndClusters(
        const Eigen::Isometry3d& gripper_pose,
        const std::vector<ssize_t>& cover_indices,
        const std::vector<uint32_t>& cluster_labels,
        const uint32_t num_clusters) const
{
    assert(cover_indices.size() == cluster_labels.size());
    std::vector<std::vector<double>> distances(num_clusters);
    const ssize_t gripper_idx_in_free_space_graph = work_space_grid_.worldPosToGridIndexClamped(gripper_pose.translation());

    // Itterate through all of the cluster labels, looking up the dijkstras distance to the gripper for each
    for (size_t label_idx = 0; label_idx < cluster_labels.size(); ++label_idx)
    {
        const uint32_t cluster_idx = cluster_labels[label_idx];
        const ssize_t cover_idx = cover_indices[label_idx];

        const double dijkstras_distance = dijkstras_results_[cover_idx].second[(size_t)gripper_idx_in_free_space_graph];
        distances[cluster_idx].push_back(dijkstras_distance);
    }

    std::vector<double> average_distances(num_clusters, 0.0);
    for (uint32_t cluster_idx = 0; cluster_idx < num_clusters; ++cluster_idx)
    {
        average_distances[cluster_idx] = EigenHelpers::AverageStdVectorDouble(distances[cluster_idx]);
    }
    return average_distances;
}

void DijkstrasCoverageTask::visualizeFreeSpaceGraph() const
{
    EigenHelpers::VectorVector3d node_centers;
    EigenHelpers::VectorVector3d start_points;
    EigenHelpers::VectorVector3d end_points;

    const auto& nodes = free_space_graph_.getNodes();
    for (const auto& node : nodes)
    {
        const auto& node_center = node.getValue();
        node_centers.push_back(node_center);

        const auto& out_edges = node.getOutEdges();
        for (const auto& edge : out_edges)
        {
            start_points.push_back(nodes[edge.getFromIndex()].getValue()); // Should be the same as start_points, but not verified
            end_points.push_back(nodes[edge.getToIndex()].getValue());
        }
    }

//    vis_->visualizeSpheres("free_space_graph_nodes", node_centers, Visualizer::Cyan(), 1, 0.002);
    vis_->visualizeLines("free_space_graph_edges", start_points, end_points, Visualizer::Cyan(), 1, 0.0002);
    vis_->forcePublishNow(0.2);
}

void DijkstrasCoverageTask::visualizeIndividualDijkstrasResult(
        const size_t cover_idx,
        const Eigen::Vector3d& querry_loc) const
{
    const std::vector<int64_t>& dijkstras_next_target = dijkstras_results_[(size_t)cover_idx].first;

    EigenHelpers::VectorVector3d start_points;
    EigenHelpers::VectorVector3d end_points;

    for (ssize_t x_idx = 0; x_idx < work_space_grid_.getXNumSteps(); ++x_idx)
    {
        for (ssize_t y_idx = 0; y_idx < work_space_grid_.getYNumSteps(); ++y_idx)
        {
            for (ssize_t z_idx = 0; z_idx < work_space_grid_.getZNumSteps(); ++z_idx)
            {
                const ssize_t point_idx_in_free_space_graph = work_space_grid_.xyzIndexToGridIndex(x_idx, y_idx, z_idx);
                assert(point_idx_in_free_space_graph >= 0 && point_idx_in_free_space_graph < (ssize_t)(dijkstras_next_target.size() - num_cover_points_));
                const Eigen::Vector3d point_in_world = work_space_grid_.xyzIndexToWorldPosition(x_idx, y_idx, z_idx);
                const ssize_t next_idx_in_free_space_graph = dijkstras_next_target[(size_t)point_idx_in_free_space_graph];
                if (next_idx_in_free_space_graph >= 0)
                {
                    const Eigen::Vector3d next_in_world = free_space_graph_.getNode(next_idx_in_free_space_graph).getValue();
                    start_points.push_back(point_in_world);
                    end_points.push_back(next_in_world);
                }
            }
        }
    }

    vis_->visualizeSpheres("dijkstras_navigation_field_querry", {querry_loc}, Visualizer::Orange(), 1, 0.0002);
    vis_->visualizeArrows("dijkstras_navigation_field", start_points, end_points, Visualizer::Orange(), work_space_grid_.minStepDimension() * 0.05, work_space_grid_.minStepDimension() * 0.1);
    vis_->forcePublishNow(0.2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Virtual functions that we implement
////////////////////////////////////////////////////////////////////////////////////////////////////////////

double DijkstrasCoverageTask::calculateError_impl(
        const WorldState& world_state)
{
    const Correspondences& correspondences = getCoverPointCorrespondences(world_state);
    const std::vector<double>& distances = correspondences.uncovered_target_points_distances_;
    const double sum = std::accumulate(distances.begin(), distances.end(), 0.0);
    return sum;
}

ObjectDeltaAndWeight DijkstrasCoverageTask::calculateObjectErrorCorrectionDelta_impl(
        const WorldState& world_state)
{
    const Correspondences& correspondences = getCoverPointCorrespondences(world_state);
    const ObjectPointSet& object_configuration = world_state.object_configuration_;
    ObjectDeltaAndWeight desired_object_delta(num_nodes_ * 3);

    for (ssize_t deform_idx = 0; deform_idx < num_nodes_; ++deform_idx)
    {
        // Extract the correct part of each data structure
        const Eigen::Vector3d& deformable_point = object_configuration.col(deform_idx);
        const std::vector<ssize_t>& current_correspondences                     = correspondences.correspondences_[deform_idx];
        const std::vector<bool>& current_correspondences_is_covered             = correspondences.correspondences_is_covered_[deform_idx];
        const std::vector<double>& current_correspondences_distances            = correspondences.correspondences_distances_[deform_idx];
        const EigenHelpers::VectorVector3d& current_correspondences_next_step   = correspondences.correspondences_next_step_[deform_idx];

        for (size_t correspondence_idx = 0; correspondence_idx < current_correspondences.size(); ++correspondence_idx)
        {
            // If the current cover point is not covered (by this or another object point), then create a pull on the cloth/rope
            if (!current_correspondences_is_covered[correspondence_idx])
            {
                const Eigen::Vector3d& target_point = current_correspondences_next_step[correspondence_idx];
                desired_object_delta.delta.segment<3>(deform_idx * 3) += (target_point - deformable_point);

                const double weight = std::max(desired_object_delta.weight(deform_idx * 3), current_correspondences_distances[correspondence_idx]);
                desired_object_delta.weight(deform_idx * 3) = weight;
                desired_object_delta.weight(deform_idx * 3 + 1) = weight;
                desired_object_delta.weight(deform_idx * 3 + 2) = weight;
            }
        }
    }

    return desired_object_delta;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Dijkstras Coverage Task - Private helpers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool DijkstrasCoverageTask::saveDijkstrasResults()
{
    try
    {
        std::vector<uint8_t> buffer;
        // First serialize the graph that created the results
        ROS_INFO_NAMED("coverage_task", "Serializing the data");
        free_space_graph_.serializeSelf(buffer, &arc_utilities::SerializeEigen<double, 3, 1>);

        // Next serialize the results themselves
        const auto first_serializer = [] (const std::vector<int64_t>& vec_to_serialize, std::vector<uint8_t>& buf)
        {
            return arc_utilities::SerializeVector<int64_t>(vec_to_serialize, buf, &arc_utilities::SerializeFixedSizePOD<int64_t>);
        };
        const auto second_serializer = [] (const std::vector<double>& vec_to_serialize, std::vector<uint8_t>& buf)
        {
            return arc_utilities::SerializeVector<double>(vec_to_serialize, buf, &arc_utilities::SerializeFixedSizePOD<double>);
        };
        const auto pair_serializer = [&first_serializer, &second_serializer] (const std::pair<std::vector<int64_t>, std::vector<double>>& pair_to_serialize, std::vector<uint8_t>& buf)
        {
            return arc_utilities::SerializePair<std::vector<int64_t>, std::vector<double>>(pair_to_serialize, buf, first_serializer, second_serializer);
        };
        arc_utilities::SerializeVector<std::pair<std::vector<int64_t>, std::vector<double>>>(dijkstras_results_, buffer, pair_serializer);

        // Compress and save to file
        const std::string dijkstras_file_path = GetDijkstrasStorageLocation(*nh_);
        ROS_INFO_STREAM_NAMED("coverage_task", "Compressing and saving to file " << dijkstras_file_path);
        ZlibHelpers::CompressAndWriteToFile(buffer, dijkstras_file_path);
        return true;
    }
    catch (const std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED("coverage_task", "Saving Dijkstras results to file failed: " << ex.what());
        return false;
    }
}

bool DijkstrasCoverageTask::loadDijkstrasResults()
{
    try
    {
        Stopwatch stopwatch;
        ROS_INFO_NAMED("coverage_task", "Checking if Dijkstra's solution already exists");
        const std::string dijkstras_file_path = GetDijkstrasStorageLocation(*nh_);
        const auto decompressed_dijkstras_results = ZlibHelpers::LoadFromFileAndDecompress(dijkstras_file_path);

        // First check that the graph we have matches the graph that is stored
        std::vector<uint8_t> temp_buffer;
        const uint64_t serialzed_graph_size = free_space_graph_.serializeSelf(temp_buffer, &arc_utilities::SerializeEigen<double, 3, 1>);
        const auto mismatch_results = std::mismatch(temp_buffer.begin(), temp_buffer.end(), decompressed_dijkstras_results.begin());
        if (mismatch_results.first != temp_buffer.end())
        {
            throw_arc_exception(std::runtime_error, "Mismatch in serialzed graphs, need to regenerate Dijkstras results");
        }

        // Next deserialze the Dijkstras results
        const auto first_deserializer = [] (const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return arc_utilities::DeserializeVector<int64_t>(buffer, current, &arc_utilities::DeserializeFixedSizePOD<int64_t>);
        };
        const auto second_deserializer = [] (const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return arc_utilities::DeserializeVector<double>(buffer, current, &arc_utilities::DeserializeFixedSizePOD<double>);
        };
        const auto pair_deserializer = [&first_deserializer, &second_deserializer] (const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return arc_utilities::DeserializePair<std::vector<int64_t>, std::vector<double>>(buffer, current, first_deserializer, second_deserializer);
        };

        uint64_t current_position = serialzed_graph_size;
        const auto deserialized_result = arc_utilities::DeserializeVector<std::pair<std::vector<int64_t>, std::vector<double>>>(decompressed_dijkstras_results, current_position, pair_deserializer);
        dijkstras_results_ = deserialized_result.first;
        current_position += deserialized_result.second;
        if (current_position != decompressed_dijkstras_results.size())
        {
            throw_arc_exception(std::runtime_error, "Invalid data size found");
        }

        ROS_INFO_STREAM_NAMED("coverage_task", "Read solutions in " << stopwatch(READ) << " seconds");
        return true;
    }
    catch (...)
    {
        ROS_ERROR_NAMED("coverage_task", "Loading Dijkstras results from file failed");
        return false;
    }
}

Eigen::Vector3d DijkstrasCoverageTask::sumVectorFields(
        const std::vector<ssize_t>& cover_point_assignments,
        const Eigen::Vector3d& querry_loc) const
{
    Eigen::Vector3d summed_dijkstras_deltas(0, 0, 0);

    const ssize_t graph_aligned_querry_ind = work_space_grid_.worldPosToGridIndexClamped(querry_loc);
    const Eigen::Vector3d& graph_aligned_querry_point = free_space_graph_.getNode(graph_aligned_querry_ind).getValue();

    // Combine the vector fields from each assignment
    for (size_t assignment_ind = 0; assignment_ind < cover_point_assignments.size(); ++assignment_ind)
    {
        // Each entry in dijktras_results_[cover_ind] is a (next_node, distance to goal) pair
        const ssize_t cover_ind = cover_point_assignments[assignment_ind];
        const auto& vector_field = dijkstras_results_[(size_t)cover_ind].first;
        const ssize_t target_ind_in_work_space_graph = vector_field[(size_t)graph_aligned_querry_ind];

        const Eigen::Vector3d& target_point = free_space_graph_.getNode(target_ind_in_work_space_graph).getValue();
        summed_dijkstras_deltas += (target_point - graph_aligned_querry_point);
    }

    return summed_dijkstras_deltas;
}

// Note that this is only used for predicting if the local controller will get stuck
EigenHelpers::VectorVector3d DijkstrasCoverageTask::followCoverPointAssignments(
        const Eigen::Vector3d& starting_pos,
        const std::vector<ssize_t>& cover_point_assignments,
        const size_t maximum_iterations) const
{
    static const double min_outer_progress_squared = std::pow(work_space_grid_.minStepDimension() / 2.0, 2);

    #ifdef ENABLE_PROJECTION
    Eigen::Vector3d current_pos = sdf_->ProjectOutOfCollision3d(starting_pos);
    #else
    Eigen::Vector3d current_pos = starting_pos;
    #endif
    EigenHelpers::VectorVector3d trajectory(1, current_pos);

    bool outer_progress = cover_point_assignments.size() > 0;
    for (size_t itr = 0; outer_progress && itr < maximum_iterations; ++itr)
    {
        Eigen::Vector3d updated_pos = current_pos;

        // Split the delta up into smaller steps to simulate "pulling" the cloth along with constant obstacle collision resolution
        bool inner_progress = true;
        for (int i = 0; inner_progress && i  < VECTOR_FIELD_FOLLOWING_NUM_MICROSTEPS; ++i)
        {
            const Eigen::Vector3d summed_dijkstras_deltas = sumVectorFields(cover_point_assignments, updated_pos);

            // If the combined vector field has minimal movement, then stop.
            if (summed_dijkstras_deltas.squaredNorm() <= min_outer_progress_squared)
            {
                break;
            }

            // Scale the delta to the size of the grid to normalize for number of correspondences
            const Eigen::Vector3d combined_delta = summed_dijkstras_deltas.normalized() * work_space_grid_.minStepDimension();

            const Eigen::Vector3d micro_delta = combined_delta/ (double)VECTOR_FIELD_FOLLOWING_NUM_MICROSTEPS;
            const Eigen::Vector3d projected_pos = sdf_->ProjectOutOfCollision3d(updated_pos + micro_delta);

            inner_progress = (projected_pos - updated_pos).squaredNorm() > VECTOR_FIELD_FOLLOWING_MIN_PROGRESS;
            if (inner_progress)
            {
                updated_pos = projected_pos;
            }
        }

        outer_progress = (current_pos - updated_pos).squaredNorm() > min_outer_progress_squared;
        if (outer_progress)
        {
            current_pos = updated_pos;
            trajectory.push_back(current_pos);
        }
    }

    return trajectory;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Distance Based Correspondences Task ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DistanceBasedCorrespondencesTask::DistanceBasedCorrespondencesTask(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : DijkstrasCoverageTask(nh, ph, vis)
{}

DijkstrasCoverageTask::Correspondences DistanceBasedCorrespondencesTask::getCoverPointCorrespondences_impl(
        const WorldState& world_state) const
{
    Correspondences correspondences(num_nodes_);

    // For every cover point, figure out the correspondence
    for (ssize_t cover_idx = 0; cover_idx < num_cover_points_; ++cover_idx)
    {
        // Find the closest deformable object point
        ssize_t closest_deformable_idx;
        double min_dist_to_deformable_object;
        ssize_t best_target_idx_in_free_space_graph;
        bool covered;
        std::tie(closest_deformable_idx, min_dist_to_deformable_object, best_target_idx_in_free_space_graph, covered) =
                findNearestObjectPoint(world_state, cover_idx);

        if (closest_deformable_idx == -1)
        {
            visualizeIndividualDijkstrasResult(cover_idx, cover_points_.col(cover_idx));
            ROS_ERROR_NAMED("task_specification", "Weirdness in correspondences calculation");
            PressAnyKeyToContinue("Weirdness in correspondences calculation ");
        }

        // Record the results in the data structure
        correspondences.correspondences_[closest_deformable_idx].push_back(cover_idx);
        correspondences.correspondences_next_step_[closest_deformable_idx].push_back(
                    free_space_graph_.getNode(best_target_idx_in_free_space_graph).getValue());
        correspondences.correspondences_distances_[closest_deformable_idx].push_back(min_dist_to_deformable_object);

        // Record the "coveredness" of this correspondence
        correspondences.correspondences_is_covered_[closest_deformable_idx].push_back(covered);
        if (!covered)
        {
            correspondences.uncovered_target_points_idxs_.push_back(cover_idx);
            correspondences.uncovered_target_points_distances_.push_back(min_dist_to_deformable_object);
        }
    }

    return correspondences;
}

/**
 * @brief DistanceBasedCorrespondencesTask::findNearestObjectPoint
 * @param object_configuration
 * @param cover_ind
 * @return index in object_configuration of the closest point as defined by Dijkstras,
 *         the matching distance,
 *         the index of the target position in the free space graph,
 *         if the point is consisered 'covered'
 */
std::tuple<ssize_t, double, ssize_t, bool> DistanceBasedCorrespondencesTask::findNearestObjectPoint(
        const WorldState& world_state,
        const ssize_t cover_idx) const
{
    const Eigen::Vector3d& cover_point = cover_points_.col(cover_idx);
    const auto& dijkstras_individual_result = dijkstras_results_[(size_t)cover_idx];

    ssize_t closest_deformable_idx = -1;
    double min_dist = std::numeric_limits<double>::infinity();
    ssize_t best_target_idx_in_free_space_graph = -1;
    bool covered = false;

    for (ssize_t deformable_idx = 0; deformable_idx < num_nodes_; ++deformable_idx)
    {
        const Eigen::Vector3d& deformable_point = world_state.object_configuration_.col((size_t)deformable_idx);
        const double straight_line_distance = (cover_point - deformable_point).norm();

        // First calculate the distance as defined by Dijkstras - code duplicated in FixedCorrespondencesTask::calculateObjectErrorCorrectionDelta_impl
        double graph_dist;
        size_t target_idx_in_free_space_graph;
        {
            // If we are within 1 (rounded) grid cell, go straight towards the cover point
            if (straight_line_distance <= work_space_grid_.minStepDimension() * std::sqrt(2.0))
            {
                target_idx_in_free_space_graph = cover_ind_to_free_space_graph_ind_[(size_t)cover_idx];
                graph_dist = straight_line_distance;
            }
            // Otherwise use the Dijkstra's result
            else
            {
                const ssize_t deformable_point_idx_in_free_space_graph = work_space_grid_.worldPosToGridIndexClamped(deformable_point);
                target_idx_in_free_space_graph = dijkstras_individual_result.first[(size_t)deformable_point_idx_in_free_space_graph];
                graph_dist                     = dijkstras_individual_result.second[(size_t)deformable_point_idx_in_free_space_graph];

            }
        }

        // Next, if we've found something closer than our record, update our record of the closest point on the deformable object
        if (graph_dist < min_dist)
        {
            min_dist = graph_dist;
            closest_deformable_idx = deformable_idx;
            best_target_idx_in_free_space_graph = target_idx_in_free_space_graph;
        }

        // Last, record if this point counts as covering the target (multiple deformable points may cover a single target point)
        covered |= pointIsCovered(cover_idx, deformable_point);
    }

    return std::make_tuple(closest_deformable_idx, min_dist, best_target_idx_in_free_space_graph, covered);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Fixed Correspondences Task /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FixedCorrespondencesTask::FixedCorrespondencesTask(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : DijkstrasCoverageTask(nh, ph, vis)
{}

DijkstrasCoverageTask::Correspondences FixedCorrespondencesTask::getCoverPointCorrespondences_impl(
        const WorldState& world_state) const
{
    Correspondences correspondences_external(num_nodes_);
    correspondences_external.correspondences_ = correspondences_internal_fixed_;

    // For each node on the object, record the distance to the corresponding target points
    for (size_t deform_idx = 0; (ssize_t)deform_idx < num_nodes_; ++deform_idx)
    {
        // Ensure that regardless of the sensed point, we are working with a point that is in the valid volume
        const Eigen::Vector3d& deformable_point         = world_state.object_configuration_.col(deform_idx);
        #ifdef ENABLE_PROJECTION
        const Eigen::Vector3d point_in_free_space       = sdf_->ProjectOutOfCollision3d(deformable_point);
        #else
        const Eigen::Vector3d point_in_free_space = deformable_point;
        #endif

        const ssize_t nearest_idx_in_free_space_graph   = work_space_grid_.worldPosToGridIndexClamped(point_in_free_space);

        // Extract the correct part of each data structure - each deformable point can correspond to multiple target points
        const std::vector<ssize_t>& current_correspondences             = correspondences_external.correspondences_[deform_idx];
        EigenHelpers::VectorVector3d& current_correspondences_next_step = correspondences_external.correspondences_next_step_[deform_idx];
        std::vector<double>& current_correspondences_distances          = correspondences_external.correspondences_distances_[deform_idx];
        std::vector<bool>& current_correspondences_is_covered           = correspondences_external.correspondences_is_covered_[deform_idx];

//        std::cerr << "Deformable point:    " << deformable_point.transpose() << std::endl
//                  << "Point in free space: " << point_in_free_space.transpose() << std::endl
//                  << "  Nearest idx in free space graph: " << nearest_idx_in_free_space_graph << std::endl
//                  << "  num correspondences: " << current_correspondences.size() << std::endl;

//        std::cerr << "  SDF dist pre: "
//                  << " In bounds:" << environment_sdf_->CheckInBounds3d(deformable_point)
//                  << " Dist: " << environment_sdf_->Get3d(deformable_point)
//                  << std::endl;

//        std::cerr << "  SDF dist post: "
//                  << " In bounds:" << environment_sdf_->CheckInBounds3d(point_in_free_space)
//                  << " Dist: " << environment_sdf_->Get3d(point_in_free_space)
//                  << std::endl;

        // Iterate through the correspondences, recording all the data needed for other parts of the system
        current_correspondences_next_step.reserve(current_correspondences.size());
        current_correspondences_distances.reserve(current_correspondences.size());
        current_correspondences_is_covered.reserve(current_correspondences.size());
        for (size_t correspondence_idx = 0; correspondence_idx < current_correspondences.size(); ++correspondence_idx)
        {
            // Get the straight line distance
            const ssize_t cover_idx = current_correspondences[correspondence_idx];
            const Eigen::Vector3d& cover_point = cover_points_.col(cover_idx);
            const double straight_line_distance = (deformable_point - cover_point).norm();

            // If we are within 1 (rounded) grid cell, go straight towards the cover point
            if (straight_line_distance <= work_space_grid_.minStepDimension() * std::sqrt(2.0))
            {
                current_correspondences_next_step.push_back(cover_point);
                current_correspondences_distances.push_back(straight_line_distance);
            }
            // Otherwise use the Dijkstra's result
            else
            {
                // Collect the Dijkstras next step and distance
                const auto& dijkstras_individual_result         = dijkstras_results_[(size_t)cover_idx];
                const ssize_t target_ind_in_free_space_graph    = dijkstras_individual_result.first[(size_t)nearest_idx_in_free_space_graph];
                const double dijkstras_distance                 = dijkstras_individual_result.second[(size_t)nearest_idx_in_free_space_graph];

//                std::cerr << "Correspondece idx: " << correspondence_idx << std::endl;
//                std::cerr << "Target ind in free space graph: " << target_ind_in_free_space_graph << std::endl;
//                std::cerr << "Dijkstras_distance: " << dijkstras_distance << std::endl;

                current_correspondences_next_step.push_back(free_space_graph_.getNode(target_ind_in_free_space_graph).getValue());
                current_correspondences_distances.push_back(dijkstras_distance);
            }
            const double final_distance = current_correspondences_distances.back();

            // Record the "coveredness" of this correspondence
            const bool covered = pointIsCovered(cover_idx, deformable_point);
            current_correspondences_is_covered.push_back(covered);
            if (!covered)
            {
                correspondences_external.uncovered_target_points_idxs_.push_back(cover_idx);
                correspondences_external.uncovered_target_points_distances_.push_back(final_distance);
            }
        }
    }

    return correspondences_external;
}
