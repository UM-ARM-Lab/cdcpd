#include "smmap/task_framework.h"

#include <future>
#include <assert.h>
#include <numeric>
#include <memory>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/log.hpp>
#include <arc_utilities/first_order_deformation.h>
#include <arc_utilities/simple_kmeans_clustering.hpp>
#include <arc_utilities/get_neighbours.hpp>
#include <arc_utilities/path_utils.hpp>
#include <arc_utilities/timing.hpp>
#include <arc_utilities/filesystem.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/serialization_ros.hpp>
#include <deformable_manipulation_experiment_params/utility.hpp>
#include <deformable_manipulation_experiment_params/ros_params.hpp>

#include "smmap/conversions.h"

#include <smmap_models/diminishing_rigidity_model.h>
#include <smmap_models/adaptive_jacobian_model.h>
#include <smmap_models/least_squares_jacobian_model.h>
#include <smmap_models/constraint_jacobian_model.h>

#include "smmap/least_squares_controller_with_object_avoidance.h"
#include "smmap/least_squares_stretching_constraint_controller.h"
#include "smmap/stretching_constraint_controller.h"
#include "smmap/straight_line_controller.h"

using namespace smmap;
using namespace arc_utilities;
using namespace Eigen;
using namespace EigenHelpers;
using namespace EigenHelpersConversions;
using ColorBuilder = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>;
namespace dmm = deformable_manipulation_msgs;

const static std_msgs::ColorRGBA PREDICTION_GRIPPER_COLOR = ColorBuilder::MakeFromFloatColors(0.0f, 0.0f, 0.6f, 0.5f);
const static std_msgs::ColorRGBA PREDICTION_RUBBER_BAND_SAFE_COLOR = ColorBuilder::MakeFromFloatColors(0.0f, 0.0f, 0.0f, 1.0f);
const static std_msgs::ColorRGBA PREDICTION_RUBBER_BAND_VIOLATION_COLOR = ColorBuilder::MakeFromFloatColors(0.0f, 1.0f, 1.0f, 1.0f);

//#define ENABLE_SEND_NEXT_COMMAND_LOAD_SAVE 1
#define ENABLE_SEND_NEXT_COMMAND_LOAD_SAVE 0
//#define ENABLE_TRANSITION_LEARNING 1
#define ENABLE_TRANSITION_LEARNING 0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal helpers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T, typename Alloc = std::allocator<T>>
static size_t sizeOfLargestVector(const std::vector<T, Alloc>& vectors)
{
    size_t largest_vector = 0;

    for (size_t idx = 0; idx < vectors.size(); ++idx)
    {
        largest_vector = std::max(largest_vector, vectors[idx].size());
    }

    return largest_vector;
}

static std::vector<uint32_t> numberOfPointsInEachCluster(
        const std::vector<uint32_t>& cluster_labels,
        const uint32_t num_clusters,
        const std::vector<long>& grapsed_points,
        const DijkstrasCoverageTask::Correspondences& correspondences)
{
    std::vector<uint32_t> counts(num_clusters, 0);
    const auto& uncovered_target_point_idxs = correspondences.uncovered_target_points_idxs_;

    for (size_t grasped_point_idx = 0; grasped_point_idx < grapsed_points.size(); ++grasped_point_idx)
    {
        const long deform_idx = grapsed_points[grasped_point_idx];

        const std::vector<ssize_t>& correspondences_for_current_deform_idx            = correspondences.correspondences_[deform_idx];
        const std::vector<bool>&    correspondences_is_covered_for_current_deform_idx = correspondences.correspondences_is_covered_[deform_idx];

        for (size_t correspondence_idx = 0; correspondence_idx < correspondences_for_current_deform_idx.size(); ++correspondence_idx)
        {
            const ssize_t cover_idx = correspondences_for_current_deform_idx[correspondence_idx];
            const bool is_covered   = correspondences_is_covered_for_current_deform_idx[correspondence_idx];

            // If the current correspondece is not covered, lookup its position in cluster_labels
            if (!is_covered)
            {
                const auto found_itr = std::find(uncovered_target_point_idxs.begin(), uncovered_target_point_idxs.end(), cover_idx);
                assert(found_itr != uncovered_target_point_idxs.end()); // The point is not covered, so it should exist in the vector
                const ssize_t found_idx = std::distance(uncovered_target_point_idxs.begin(), found_itr);
                assert(found_idx >= 0);
                assert(found_idx < (ssize_t)cluster_labels.size());
                const uint32_t cluster_label = cluster_labels[found_idx];
                counts[cluster_label]++;
            }
        }
    }

    return counts;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor and model list builder
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Planner::Planner
 * @param robot
 * @param vis
 * @param task_specification
 */
TaskFramework::TaskFramework(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        RobotInterface::Ptr robot,
        Visualizer::Ptr vis,
        TaskSpecification::Ptr task_specification)
    // Robot and task parameters
    : nh_(nh)
    , ph_(ph)
    , seed_(GetPlannerSeed(*ph_))
    , generator_(std::make_shared<std::mt19937_64>(seed_))
    , test_id_(GetTestId(*nh_))
    , robot_(robot)
    , task_specification_(task_specification)
    , dijkstras_task_(std::dynamic_pointer_cast<DijkstrasCoverageTask>(task_specification_)) // If possible, this will be done, if not, it will be NULL (nullptr?)
    // Multi-model and regret based model selection parameters
    , collect_results_for_all_models_(GetCollectResultsForAllModels(*ph_))
    , mab_algorithm_(GetMABAlgorithm(*ph_))
    , reward_std_dev_scale_factor_(GetRewardScaleFactorStart(*ph_))
    , process_noise_factor_(GetProcessNoiseFactor(*ph_))
    , observation_noise_factor_(GetObservationNoiseFactor(*ph_))
    , correlation_strength_factor_(GetCorrelationStrengthFactor(*ph_))
    // 'Stuck' detection and RRT prameters
    , enable_stuck_detection_(GetEnableStuckDetection(*ph_))
    , max_lookahead_steps_(GetNumLookaheadSteps(*ph_))
    , max_grippers_pose_history_length_(GetMaxGrippersPoseHistoryLength(*ph_))
    , plan_triggered_once_(false)
    , executing_global_trajectory_(false)
    , num_times_planner_invoked_(0)
    , band_rrt_(nullptr)
    , policy_current_idx_(-1)
    , policy_segment_next_idx_(-1)
    , transition_estimator_(nullptr)
    // Used to generate some log data by some controllers
    , object_initial_node_distance_(CalculateDistanceMatrix(GetObjectInitialConfiguration(*nh_)))
    , initial_grippers_distance_(robot_->getGrippersInitialDistance())
    // Logging and visualization parameters
    , bandits_logging_enabled_(GetBanditsLoggingEnabled(*ph_))
    , controller_logging_enabled_(GetControllerLoggingEnabled(*ph_))
    , vis_(vis)
    , visualize_desired_motion_(vis_->visualizationsEnabled() && GetVisualizeObjectDesiredMotion(*ph_))
    , visualize_gripper_motion_(vis_->visualizationsEnabled() && GetVisualizeGripperMotion(*ph_))
    , visualize_predicted_motion_(vis_->visualizationsEnabled() && GetVisualizeObjectPredictedMotion(*ph_))
{
    ROS_INFO_STREAM_NAMED("task_framework", "Using seed " << IntToHex(seed_));
    std::srand((unsigned int)seed_);
    initializeBanditsLogging();
    initializeControllerLogging();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The one function that gets invoked externally
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskFramework::execute()
{
    WorldState world_state = robot_->start();
    double start_time = world_state.sim_time_;
    initializeModelAndControllerSet(world_state);

    if (false)
    {
        std::vector<std_msgs::ColorRGBA> colors(world_state.object_configuration_.cols());
        for (ssize_t idx = 0; idx < world_state.object_configuration_.cols(); ++idx)
        {
            colors[idx] = ColorBuilder::InterpolateHotToCold((double)(idx)/(double)(world_state.object_configuration_.cols() - 1));
        }
        vis_->visualizePoints("object_config_test", world_state.object_configuration_, colors);
        vis_->visualizePoint("gripper0grasped_point", world_state.object_configuration_.col(robot_->getGrippersData().at(0).node_indices_[0]), Visualizer::Blue());
        vis_->visualizePoint("gripper1grasped_point", world_state.object_configuration_.col(robot_->getGrippersData().at(1).node_indices_[0]), Visualizer::Cyan());
        PressAnyKeyToContinue("Waiting for object config ordering confirmation");
//        arc_helpers::Sleep(0.5);
    }

    if (enable_stuck_detection_)
    {
        assert(dijkstras_task_ != nullptr);

        // TODO: Assumptions in the implementation that need be addressed later
        assert(robot_->getGrippersData().size() == 2);
        assert(model_list_.size() == 1);

        if (ENABLE_SEND_NEXT_COMMAND_LOAD_SAVE && useStoredWorldState())
        {
            const auto world_state_and_band = loadStoredWorldState();
            world_state = world_state_and_band.first;
            vis_->visualizeCloth("controller_input_deformable_object", world_state.object_configuration_, Visualizer::Green(0.5), 1);
        }
        initializeBand(world_state);

        // Initialize the MDP transition learner
        transition_estimator_ = std::make_shared<TransitionEstimation>(
                    nh_, ph_, generator_, dijkstras_task_->sdf_, dijkstras_task_->work_space_grid_, vis_, *rubber_band_);
        // Initialize the BandRRT structure
        initializeBandRRT(world_state.robot_configuration_valid_);
    }

    ROS_INFO_STREAM_NAMED("task_framework", "-------------- Robot and frameowrk    initialized with start_time: " << start_time << " --------------");

    while (robot_->ok())
    {
        static const bool first_iteration_always_requires_plan =
                ROSHelpers::GetParamRequired<bool>(*ph_, "task/first_control_loop_triggers_plan", __func__);
        if (first_iteration_always_requires_plan && !plan_triggered_once_)
        {
            // Evaluate the performance of the planner at the initial state
            const RRTNode start_config = [&]
            {
                const RRTGrippersRepresentation gripper_config(
                            world_state.all_grippers_single_pose_[0],
                            world_state.all_grippers_single_pose_[1]);

                RRTRobotRepresentation robot_config;
                if (world_state.robot_configuration_valid_)
                {
                    robot_config = world_state.robot_configuration_;
                }
                else
                {
                    robot_config.resize(6);
                    robot_config.head<3>() = gripper_config.first.translation();
                    robot_config.tail<3>() = gripper_config.second.translation();
                }

                return RRTNode(gripper_config, robot_config, rubber_band_);
            }();
            const RRTGrippersRepresentation target_grippers_poses = ToGripperPosePair(getGripperTargets(world_state));

            const auto parallel = ROSHelpers::GetParam<bool>(*ph_, "test_planning_performance_parallel", true);
            testPlanningPerformance(world_state, seed_, start_config, target_grippers_poses, parallel);

            ROS_WARN_NAMED("task_framework", "Triggering global planner regardless of local controller on first iteration");
            ROS_WARN_STREAM_NAMED("task_framework", "   Planner/Task sim time at planner evaluation " << world_state.sim_time_);

            // Plan a trajectory actually generate data to learn from
            planGlobalGripperTrajectory(world_state);
            plan_triggered_once_ = true;
        }

        const auto feedback = sendNextCommand(world_state);
        const auto& world_feedback = feedback.first;
        const auto force_restart = feedback.second;
        const auto time_ellapsed = world_feedback.sim_time_ - start_time;

        if (!force_restart &&
            (time_ellapsed < task_specification_->maxTime()) &&
            !task_specification_->taskDone(world_feedback))
        {
            world_state = world_feedback;
        }
        else
        {
            ROS_INFO_NAMED("task_framework", "------------------------------- End of Task -------------------------------------------");
            const double current_error = task_specification_->calculateError(world_feedback);
            ROS_INFO_STREAM_NAMED("task_framework", "   Planner/Task sim time " << world_feedback.sim_time_ << "\t Error: " << current_error);

            vis_->purgeAndPublishDeleteAllAction();

            ROS_INFO_COND(force_restart, "Terminating task as a restart has been forced");
            ROS_INFO_COND(time_ellapsed >= task_specification_->maxTime(), "Terminating task as time has run out");
            ROS_INFO_COND(task_specification_->taskDone(world_feedback), "Terminating task as the task has been completed");

            // If rrt/num_trials is larger than 1, then all the seeds are reset in planGlobalGripperTrajectory,
            // so reseting and running another full system trial makes no sense
            if (ROSHelpers::GetParam<bool>(*ph_, "rerun_forever", false))
            {
                ROS_INFO_NAMED("task_framework", "------------------------------- RESETING RESETING -------------------------------------");
                if (enable_stuck_detection_)
                {
                    ROS_INFO_NAMED("task_framework", "Adding experience to classifier if possible");
                    transition_estimator_->addExperienceToClassifier(rrt_executed_path_);
                }

                robot_->reset();
                world_state = robot_->start();
                start_time = world_state.sim_time_;
                if (enable_stuck_detection_)
                {
                    initializeBand(world_state);
                    initializeBandRRT(world_state.robot_configuration_valid_);

                    plan_triggered_once_ = false;
                    executing_global_trajectory_ = false;
                    rrt_executed_path_.clear();
                    num_times_planner_invoked_ = 0;
                    policy_current_idx_ = -1;
                    policy_segment_next_idx_ = -1;

                    grippers_pose_history_.clear();
                    error_history_.clear();
                    microstep_history_buffer_.clear();

                    vis_->purgeAndPublishDeleteAllAction();
                }
                ROS_INFO_STREAM_NAMED("task_framework", "-------------- Robot and frameowrk re-initialized with start_time: " << start_time << " --------------");
            }
            else
            {
                ROS_INFO_NAMED("task_framework", "------------------------------- Pausing for 5 seconds for recording purposes ----------");
                arc_helpers::Sleep(5.0);
                robot_->shutdown();
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Gripper movement functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<WorldState, bool> TaskFramework::sendNextCommand(
        const WorldState& world_state)
{
#if ENABLE_SEND_NEXT_COMMAND_LOAD_SAVE
    if (useStoredWorldState())
    {
        const auto world_state_and_band = loadStoredWorldState();
        world_state = world_state_and_band.first;
        rubber_band_between_grippers_ = world_state_and_band.second;
        vis_->visualizeCloth("controller_input_deformable_object", world_state.object_configuration_, Visualizer::Green(0.5), 1);
    }
    else
    {
        storeWorldState(world_state, rubber_band_);
    }
#endif

    ROS_INFO_NAMED("task_framework", "---------------------------- Start of Loop -----------------------------------------");
    const double current_error = task_specification_->calculateError(world_state);
    ROS_INFO_STREAM_NAMED("task_framework", "Planner/Task sim time " << world_state.sim_time_ << "\t Error: " << current_error);

    if (enable_stuck_detection_)
    {
        bool planning_needed = false;

        // Check if the global plan has 'hooked' the deformable object on something
        if (executing_global_trajectory_)
        {
            Stopwatch stopwatch;
            const bool global_plan_will_overstretch = predictStuckForGlobalPlannerResults();
            ROS_INFO_STREAM_NAMED("task_framework", "Determined if global planner needed in          " << stopwatch(READ) << " seconds");

            if (global_plan_will_overstretch)
            {
                arc_helpers::Sleep(1.0);

                vis_->purgeAndPublishDeleteAllAction();
                vis_->clearVisualizationsBullet();
                band_rrt_->visualizePolicy(rrt_planned_policy_);
                vis_->forcePublishNow();

//                vis_->deleteObjects(BandRRT::RRT_SOLUTION_RUBBER_BAND_NS, 1, 40);
//                vis_->deleteObjects(PROJECTED_BAND_NS, 2, 11);
//                vis_->deleteObjects(PROJECTED_GRIPPER_NS, 2, 21);
//                vis_->forcePublishNow(0.05);

                planning_needed = true;
#if ENABLE_TRANSITION_LEARNING
                ROS_WARN_NAMED("task_framework", "Determining most recent bad transition");
                const auto last_bad_transition = transition_estimator_->findMostRecentBadTransition(rrt_executed_path_);
                if (last_bad_transition.Valid())
                {
                    arc_helpers::Sleep(1.0);
//                    vis_->deleteObjects("TRANSITION_LEARNING_LOOKING_BACK_PLANNED", 1, (int32_t)(rrt_executed_path_.size()));
//                    vis_->forcePublishNow(0.01);
//                    vis_->deleteObjects("TRANSITION_LEARNING_LOOKING_BACK_EXECUTED", 1, (int32_t)(rrt_executed_path_.size()));
//                    vis_->forcePublishNow(0.01);
                    transition_estimator_->learnTransition(last_bad_transition.GetImmutable());
//                    transition_estimator_->visualizeTransition(last_bad_transition.GetImmutable());
                    vis_->forcePublishNow(0.5);
                    PressAnyKeyToContinue("Waiting for input after visualizing new memorized transition");
                }
                else
                {
                    ROS_WARN_NAMED("task_framework", "Global plan failed but unable to learn new transition");
                }
#endif

                ROS_WARN_NAMED("task_framework", "Invoking global planner as the current plan will overstretch the deformable object");
                ROS_INFO_NAMED("task_framework", "----------------------------------------------------------------------------");
            }
        }
        // Check if the local controller will be stuck
        else
        {
            // TODO: HACK: force a restart of the scenario after learning rather than continuing
            ROS_INFO_NAMED("task_framework", "Local controller taking over in main loop; stopping task and resetting");
            return {world_state, true};



            Stopwatch stopwatch;
            arc_helpers::DoNotOptimize(world_state);
            const bool global_planner_needed_due_to_overstretch = globalPlannerNeededDueToOverstretch(world_state);
            const bool global_planner_needed_due_to_lack_of_progress = globalPlannerNeededDueToLackOfProgress();
            arc_helpers::DoNotOptimize(global_planner_needed_due_to_lack_of_progress);
            ROS_INFO_STREAM_NAMED("task_framework", "Determined if global planner needed in " << stopwatch(READ) << " seconds");

            if (global_planner_needed_due_to_overstretch || global_planner_needed_due_to_lack_of_progress)
            {
                planning_needed = true;

                ROS_WARN_COND_NAMED(global_planner_needed_due_to_overstretch, "task_framework", "Invoking global planner as the controller will overstretch the deformable object");
                ROS_WARN_COND_NAMED(global_planner_needed_due_to_lack_of_progress, "task_framework", "Invoking global planner due to lack of progress");
                ROS_WARN_NAMED("task_framework", "Adding current rubber band to blacklist");
                ROS_INFO_NAMED("task_framework", "----------------------------------------------------------------------------");
                band_rrt_->addBandToBlacklist(*rubber_band_);

                if (vis_->visualizationsEnabled())
                {
                    vis_->forcePublishNow(2.0);
                }
            }
        }

        // If we need to (re)plan due to the local controller getting stuck, or the gobal plan failing, then do so
        if (planning_needed)
        {
            // TODO: HACK: force a restart of the scenario after learning rather than continuing
            ROS_INFO_NAMED("task_framework", "Global planning triggered in main loop; stopping task and resetting");
            return {world_state, true};

//            PressAnyKeyToContinue("pausing before planning ...");
            vis_->purgeAndPublishDeleteAllAction();
            planGlobalGripperTrajectory(world_state);

//            if (task_specification_->task_type_ == TaskType::ROPE_ENGINE_ASSEMBLY_LIVE)
//            {
//                PressAnyKeyToContinue("Pausing before executing plan ... ");
//            }
        }

        // Execute a single step in the global plan, or use the local controller if we have no plan to follow
        WorldState world_feedback;
        if (executing_global_trajectory_)
        {
            world_feedback = sendNextCommandUsingGlobalPlannerResults(world_state);
            // Band is updated internally in sendNextCommandUsingGlobalPlannerResults
        }
        else
        {
            world_feedback = sendNextCommandUsingLocalController(world_state);
            updateBand(world_feedback);
        }

        // Keep the last N grippers positions recorded to detect if the grippers are stuck
        grippers_pose_history_.push_back(world_feedback.all_grippers_single_pose_);
        error_history_.push_back(dijkstras_task_->calculateError(world_feedback));
        assert(grippers_pose_history_.size() == error_history_.size());
        if (grippers_pose_history_.size() > max_grippers_pose_history_length_)
        {
            grippers_pose_history_.erase(grippers_pose_history_.begin());
            error_history_.erase(error_history_.begin());
        }

        return {world_feedback, false};
    }
    else
    {
        ROS_WARN_ONCE_NAMED("task_framework", "Future constraint violation detection disabled");
        return {sendNextCommandUsingLocalController(world_state), false};
    }
}

void TaskFramework::updateBand(const WorldState& world_feedback)
{
    RubberBand prev_band = *rubber_band_;
    if (!rubber_band_->resetBand(world_feedback))
    {
//        PressAnyKeyToContinue("Error resetting the band after moving the grippers with the local controller. Aborting.");
//        assert(false);

        ROS_WARN_NAMED("task_framework", "Error resetting the band after moving the grippers with the local controller, skipping this reset step and propagating instead ");
        *rubber_band_ = prev_band;
        rubber_band_->forwardPropagate(ToGripperPositions(world_feedback.all_grippers_single_pose_), false);
    }
}

/**
 * @brief Planner::sendNextCommandUsingLocalController
 * @param world_state
 * @return
 */
WorldState TaskFramework::sendNextCommandUsingLocalController(
        const WorldState& current_world_state)
{
    Stopwatch stopwatch;
    Stopwatch function_wide_stopwatch;

    // Temporaries needed here bercause model_input_data takes things by reference
    const DesiredDirection desired_object_manipulation_direction = task_specification_->calculateDesiredDirection(current_world_state);
    // It is assumed that the robot's internal state matches that that is passed to us, so we do not need to set active dof values
    const MatrixXd robot_dof_to_grippers_poses_jacobian = robot_->getGrippersJacobian();
    // Build the constraints for the gippers and other points of interest on the robot - includes the grippers
    const std::vector<std::pair<CollisionData, Matrix3Xd>> full_robot_poi_collision_data_ = robot_->getPointsOfInterestCollisionData();

    const bool handle_overstretch = true;
    const DeformableController::InputData model_input_data(
                current_world_state,
                desired_object_manipulation_direction,
                robot_,
                robot_dof_to_grippers_poses_jacobian,
                current_world_state.robot_configuration_valid_,
                full_robot_poi_collision_data_,
                robot_->max_gripper_velocity_norm_ * robot_->dt_,
                robot_->max_dof_velocity_norm_ * robot_->dt_,
                handle_overstretch);

    if (visualize_desired_motion_)
    {
        visualizeDesiredMotion(current_world_state, model_input_data.desired_object_motion_.error_correction_);
//        std::this_thread::sleep_for(std::chrono::duration<double>(2.0));
    }

    // Pick an arm to use
    const ssize_t model_to_use = model_utility_bandit_->selectArmToPull(*generator_);

    const bool get_action_for_all_models = model_utility_bandit_->generateAllModelActions();
    ROS_INFO_STREAM_COND_NAMED(num_models_ > 1, "task_framework", "Using model index " << model_to_use);

    // Querry each model for it's best gripper delta
    ROS_INFO_STREAM_NAMED("task_framework", "Generating model suggestions");
    stopwatch(RESET);
    std::vector<DeformableController::OutputData> suggested_robot_commands(num_models_);
    std::vector<double> controller_computation_time(num_models_, 0.0);
    ROS_WARN_COND_NAMED(num_models_ > 1, "task_framework", "Parallel compute next command per model disabled");
//    #pragma omp parallel for
    for (size_t model_ind = 0; model_ind < (size_t)num_models_; model_ind++)
    {
        if (collect_results_for_all_models_ || get_action_for_all_models || (ssize_t)model_ind == model_to_use)
        {
            Stopwatch individual_controller_stopwatch;

            suggested_robot_commands[model_ind] =
                controller_list_[model_ind]->getGripperMotion(
                        model_input_data);

            controller_computation_time[model_ind] = individual_controller_stopwatch(READ);

            // Measure the time it took to pick a model
            ROS_DEBUG_STREAM_NAMED("task_framework", model_ind << "th Controller get suggested motion in          " << controller_computation_time[model_ind] << " seconds");
        }
    }
    // Measure the time it took to pick a model
    const DeformableController::OutputData& selected_command = suggested_robot_commands[(size_t)model_to_use];
    ROS_INFO_STREAM_NAMED("task_framework", "Calculated model suggestions and picked one in  " << stopwatch(READ) << " seconds");
    if (current_world_state.robot_configuration_valid_)
    {
        ROS_INFO_STREAM_NAMED("task_framework", "Robot DOF motion: " << selected_command.robot_dof_motion_.transpose());
    }
    for (size_t ind = 0; ind < selected_command.grippers_motion_.size(); ++ind)
    {
        ROS_INFO_STREAM_NAMED("task_framework", "Gripper " << ind << " motion: " << selected_command.grippers_motion_[ind].transpose());
    }
    ROS_INFO_STREAM_NAMED("task_framework", "Selected command gripper action norm:  " << MultipleGrippersVelocity6dNorm(selected_command.grippers_motion_));

    // Collect feedback data for logging purposes
    std::vector<WorldState> individual_model_results(num_models_);
    if (collect_results_for_all_models_)
    {
        stopwatch(RESET);
        // Build a feedback function to log data for each model that we are testing
        const auto test_feedback_fn = [&] (const size_t model_ind, const WorldState& resulting_world_state)
        {
            individual_model_results[model_ind] = resulting_world_state;
        };
        std::vector<AllGrippersSinglePose> poses_to_test(num_models_);
        std::vector<VectorXd> configurations_to_test(num_models_);
        for (size_t model_ind = 0; model_ind < (size_t)num_models_; model_ind++)
        {
            poses_to_test[model_ind] = kinematics::applyTwist(
                        current_world_state.all_grippers_single_pose_, suggested_robot_commands[model_ind].grippers_motion_);

            configurations_to_test[model_ind] =
                    current_world_state.robot_configuration_ + suggested_robot_commands[model_ind].robot_dof_motion_;
        }
        robot_->testRobotMotion(poses_to_test, configurations_to_test, current_world_state.robot_configuration_valid_, test_feedback_fn);

        ROS_INFO_STREAM_NAMED("task_framework", "Collected data to calculate regret in " << stopwatch(READ) << " seconds");
    }

    if (visualize_gripper_motion_)
    {
//        ROS_WARN_THROTTLE_NAMED(1.0, "task_framework", "Asked to visualize grippper motion but this is disabled. Manually enable the type of visualization you want.");

//        for (ssize_t model_ind = 0; model_ind < num_models_; ++model_ind)
//        {
//            ssize_t model_ind = 0;
//            visualizeGripperMotion(world_state.all_grippers_single_pose_,
//                                   suggested_robot_commands[(size_t)model_ind].grippers_motion_,
//                                   model_ind);
//        }
        std::cout << "min dist: " << robot_->min_controller_distance_to_obstacles_ << std::endl;
        const auto all_grippers_single_pose = kinematics::applyTwist(current_world_state.all_grippers_single_pose_, selected_command.grippers_motion_);
        for (size_t gripper_idx = 0; gripper_idx < all_grippers_single_pose.size(); ++gripper_idx)
        {
            vis_->visualizeGripper("target_gripper_positions", all_grippers_single_pose[gripper_idx], Visualizer::Yellow(), (int)gripper_idx + 1);
        }

        const auto updated_collision_data = robot_->checkGripperCollision(all_grippers_single_pose);
        for (size_t idx = 1; idx < current_world_state.gripper_collision_data_.size(); ++idx)
        {
            {
                const auto& data = current_world_state.gripper_collision_data_[idx];

                vis_->visualizeSpheres("collision_poi",
                                       {data.nearest_point_to_obstacle_},
                                       Visualizer::Red(0.5),
                                       static_cast<int32_t>(idx + 1),
                                       {data.distance_to_obstacle_});

                std::cout << "Start:        " << data.nearest_point_to_obstacle_.transpose() << "    dist: " << data.distance_to_obstacle_ << std::endl;
            }
            {
                const auto& data = updated_collision_data[idx];

                vis_->visualizeSpheres("collision_poi_updated",
                                       {data.nearest_point_to_obstacle_},
                                       Visualizer::Yellow(0.5),
                                       static_cast<int32_t>(idx + 1),
                                       {data.distance_to_obstacle_});

                std::cout << "After move:   " << data.nearest_point_to_obstacle_.transpose() << "    dist: " << data.distance_to_obstacle_ << std::endl;
            }
        }
        vis_->forcePublishNow();
        PressAnyKeyToContinue();

//        const size_t num_grippers = world_feedback.all_grippers_single_pose_.size();
//        for (size_t gripper_idx = 0; gripper_idx < num_grippers; ++gripper_idx)
//        {
//            std::cerr << "Desired delta: " << selected_command.grippers_motion_[gripper_idx].head<3>().transpose() << std::endl;
//            std::cerr << "Actual delta:  " << kinematics::calculateError(world_state.all_grippers_single_pose_[gripper_idx], world_feedback.all_grippers_single_pose_[gripper_idx]).head<3>().transpose() << std::endl;
//        }
    }

    // Execute the command
    ROS_INFO_STREAM_NAMED("task_framework", "Sending command to robot");
    const auto all_grippers_single_pose = kinematics::applyTwist(current_world_state.all_grippers_single_pose_, selected_command.grippers_motion_);
    const auto robot_configuration = current_world_state.robot_configuration_ + selected_command.robot_dof_motion_;
    // Measure execution time
    stopwatch(RESET);
    arc_helpers::DoNotOptimize(all_grippers_single_pose);
    const WorldState world_feedback = robot_->commandRobotMotion(
                all_grippers_single_pose,
                robot_configuration,
                current_world_state.robot_configuration_valid_).first;
    arc_helpers::DoNotOptimize(world_feedback);
    const double robot_execution_time = stopwatch(READ);

    const double predicted_delta_scale_factor = 25.0;
    if (visualize_predicted_motion_)
    {
//        ROS_WARN_THROTTLE_NAMED(1.0, "task_framework", "Asked to visualize predicted motion but this is disabled. Manually enable the type of visualization you want.");

        const ObjectPointSet true_object_delta = world_feedback.object_configuration_ - current_world_state.object_configuration_;
        vis_->visualizeObjectDelta(
                    "true_object_delta",
                    current_world_state.object_configuration_,
                    current_world_state.object_configuration_ + predicted_delta_scale_factor * true_object_delta,
                    Visualizer::Green());

//        task_specification_->visualizeDeformableObject(
//                PREDICTED_DELTA_NS,
//                world_state.object_configuration_ + object_delta,
//                Visualizer::Blue());
    }

    std::vector<double> model_prediction_errors_weighted(model_list_.size(), 0.0);
    std::vector<double> model_prediction_errors_unweighted(model_list_.size(), 0.0);
    if (collect_results_for_all_models_)
    {
        ROS_INFO_NAMED("task_framework", "Calculating model predictions based on real motion taken");

        const ObjectPointSet true_object_delta = world_feedback.object_configuration_ - current_world_state.object_configuration_;
        const AllGrippersSinglePoseDelta true_robot_delta = CalculateGrippersPoseDelta(current_world_state.all_grippers_single_pose_, world_feedback.all_grippers_single_pose_);

        for (size_t model_ind = 0; model_ind < (size_t)num_models_; model_ind++)
        {
            const ObjectPointSet predicted_delta = model_list_[model_ind]->getObjectDelta(current_world_state, true_robot_delta);
            const ObjectPointSet prediction_error_sq = (predicted_delta - true_object_delta).cwiseAbs2();

            const Map<const VectorXd> error_sq_as_vector(prediction_error_sq.data(), prediction_error_sq.size());
            model_prediction_errors_weighted[model_ind] = error_sq_as_vector.dot(desired_object_manipulation_direction.error_correction_.weight);
            model_prediction_errors_unweighted[model_ind] = prediction_error_sq.sum();

            if (visualize_predicted_motion_)
            {
                if (task_specification_->task_type_ != CLOTH_PLACEMAT_LINEAR_MOTION)
                {
                    ROS_WARN_NAMED("task_framework", "this visualization is only desgined for one task");
                }
                if (model_ind == 0)
                {
                    vis_->visualizeObjectDelta(
                                "constraint_model_prediction",
                                current_world_state.object_configuration_,
                                current_world_state.object_configuration_ + predicted_delta_scale_factor * predicted_delta,
                                Visualizer::Cyan());
                }

                else if (model_ind == 1)
                {
                    vis_->visualizeObjectDelta(
                                "diminishing_model_prediction",
                                current_world_state.object_configuration_,
                                current_world_state.object_configuration_ + predicted_delta_scale_factor * predicted_delta,
                                Visualizer::Red(0.3f));
                }
            }
        }
    }

    ROS_INFO_NAMED("task_framework", "Updating models");
    updateModels(current_world_state, model_input_data.desired_object_motion_.error_correction_, suggested_robot_commands, model_to_use, world_feedback);

    const double controller_time = function_wide_stopwatch(READ) - robot_execution_time;
    ROS_INFO_STREAM_NAMED("task_framework", "Total local controller time                     " << controller_time << " seconds");

    ROS_INFO_NAMED("task_framework", "Logging data");
    logBanditsData(current_world_state, world_feedback, individual_model_results, model_utility_bandit_->getMean(), model_utility_bandit_->getSecondStat(), model_to_use);
    controllerLogData(current_world_state, world_feedback, individual_model_results, model_input_data, controller_computation_time, model_prediction_errors_weighted, model_prediction_errors_unweighted);

    return world_feedback;
}

/**
 * @brief Planner::sendNextCommandUsingGlobalGripperPlannerResults
 * @param current_world_state
 * @return
 */
WorldState TaskFramework::sendNextCommandUsingGlobalPlannerResults(
        const WorldState& current_world_state)
{
    assert(executing_global_trajectory_);
    assert(policy_current_idx_ < rrt_planned_policy_.size());
    const RRTPath& current_segment = rrt_planned_policy_[policy_current_idx_].first;
    assert(policy_segment_next_idx_ < current_segment.size());

    // Check if we need to interpolate the command - we only advance the traj waypoint
    // pointer if we started within one step of the current target
    bool next_waypoint_targetted = true;
    Eigen::VectorXd next_dof_target(0);
    AllGrippersSinglePose next_grippers_target(0);
    if (current_world_state.robot_configuration_valid_)
    {
        const auto clamp_result = robot_->clampFullRobotMovement(
                    current_world_state.robot_configuration_,
                    current_segment[policy_segment_next_idx_].robotConfiguration());
        next_dof_target = clamp_result.first;
        next_waypoint_targetted = clamp_result.second;
        if (next_waypoint_targetted)
        {
            next_grippers_target = ToGripperPoseVector(current_segment[policy_segment_next_idx_].grippers());
        }
        else
        {
            robot_->lockEnvironment();
            robot_->setActiveDOFValues(next_dof_target);
            next_grippers_target = robot_->getGrippersPosesFunctionPointer();
            robot_->unlockEnvironment();
        }
    }
    else
    {
        const auto clamp_result = robot_->clampGrippersMovement(
                    current_world_state.all_grippers_single_pose_,
                    ToGripperPoseVector(current_segment[policy_segment_next_idx_].grippers()));
        next_grippers_target = clamp_result.first;
        next_waypoint_targetted = clamp_result.second;
    }

    const std::pair<WorldState, std::vector<WorldState>> command_result =
            robot_->commandRobotMotion(next_grippers_target,
                                       next_dof_target,
                                       current_world_state.robot_configuration_valid_);
    const WorldState& world_feedback = command_result.first;
    microstep_history_buffer_.insert(microstep_history_buffer_.end(), command_result.second.begin(), command_result.second.end());
    // Update the band with the new position of the deformable object
    updateBand(world_feedback);

    // If we targetted the last node of the current path segment, then we need to handle
    // recording data differently and determine which path segment to follow next
    const bool last_waypoint_targetted = next_waypoint_targetted
            && (policy_segment_next_idx_ + 1 == current_segment.size());

    const bool split_in_policy = last_waypoint_targetted
            && (rrt_planned_policy_[policy_current_idx_].second.size() != 0);

    // If we are directly targetting the waypoint itself (i.e., no interpolation)
    // then update the waypoint index, and record the resulting configuration in
    // the "path actually taken" list
    if (next_waypoint_targetted && !split_in_policy)
    {
        const TransitionEstimation::State tes =
        {
            world_feedback.object_configuration_,
            std::make_shared<RubberBand>(*rubber_band_),
            std::make_shared<RubberBand>(*current_segment[policy_segment_next_idx_].band()),
            world_feedback.rope_node_transforms_
        };
        rrt_executed_path_.push_back({tes, microstep_history_buffer_});
        ++policy_segment_next_idx_;
        microstep_history_buffer_.clear();
    }
    else if (next_waypoint_targetted && split_in_policy)
    {
        std::cerr << "Unhandled edge condition!!!!" << std::endl;
        assert(false);
    }

    if (last_waypoint_targetted && split_in_policy)
    {
        policy_current_idx_ = findBestBandMatchAtEndOfSegment();
        policy_segment_next_idx_ = 1;
    }
    else if (last_waypoint_targetted && !split_in_policy)
    {
        ROS_INFO_NAMED("task_framework", "Global plan finished, resetting grippers pose history and error history");

        executing_global_trajectory_ = false;
        policy_current_idx_ = -1;
        policy_segment_next_idx_ = -1;
        grippers_pose_history_.clear();
        error_history_.clear();
        assert(microstep_history_buffer_.size() == 0);

        vis_->purgeAndPublishDeleteAllAction();
    }

    const std::vector<WorldState> fake_all_models_results(num_models_, world_feedback);
    logBanditsData(current_world_state, world_feedback, fake_all_models_results, model_utility_bandit_->getMean(), model_utility_bandit_->getSecondStat(), -1);

    return world_feedback;
}

size_t TaskFramework::findBestBandMatchAtEndOfSegment() const
{
    assert(policy_current_idx_ < rrt_planned_policy_.size());
    const std::vector<size_t>& child_paths = rrt_planned_policy_[policy_current_idx_].second;
    assert(child_paths.size() > 0);

    size_t best_idx = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t child_idx = 0; child_idx < child_paths.size(); ++child_idx)
    {
        const size_t path_idx = child_paths[child_idx];
        const RRTPath& child_path = rrt_planned_policy_[path_idx].first;
        const RRTNode& first_node = child_path[0];

        ROS_WARN_THROTTLE_NAMED(4.0, "task_framework", "Band distance check to determine which branch of the policy to follow is not using homotopy");
        const double dist = RubberBand::Distance(*rubber_band_, *first_node.band());
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = path_idx;
        }
    }
    assert(best_idx < rrt_planned_policy_.size());
    return best_idx;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constraint violation detection
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskFramework::visualizeProjectedPaths(
        const std::vector<VectorVector3d>& projected_paths,
        const bool visualization_enabled)
{
    if (visualization_enabled)
    {
        EigenHelpers::VectorVector3d points;
        EigenHelpers::VectorVector3d lines_start_points;
        EigenHelpers::VectorVector3d lines_end_points;

        for (ssize_t node_ind = 0; node_ind < (ssize_t)projected_paths.size(); ++node_ind)
        {
            const auto& current_points = projected_paths[node_ind];
            if (current_points.size() > 1)
            {
                points.insert(points.end(), current_points.begin(), current_points.end());
                for (size_t point_idx = 1; point_idx < current_points.size(); ++point_idx)
                {
                    lines_start_points.push_back(current_points[point_idx - 1]);
                    lines_end_points.push_back(current_points[point_idx]);
                }
            }
        }
        vis_->visualizePoints(PROJECTED_POINT_PATH_NS, points, Visualizer::Magenta(), 1);
        vis_->visualizeLines(PROJECTED_POINT_PATH_LINES_NS, lines_start_points, lines_end_points, Visualizer::Magenta(), 1);
    }
}

std::pair<std::vector<VectorVector3d>, std::vector<RubberBand>> TaskFramework::projectFutureSystemState(
        const WorldState& starting_world_state,
        const bool visualization_enabled)
{
    Stopwatch stopwatch;
    Stopwatch function_wide_stopwatch;

    assert(task_specification_->is_dijkstras_type_task_ && starting_world_state.all_grippers_single_pose_.size() == 2);
    std::pair<std::vector<VectorVector3d>, std::vector<RubberBand>> projected_deformable_point_paths_and_projected_virtual_rubber_bands;

    const bool band_verbose = false;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constraint violation Version 1 - Purely cloth overstretch
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    stopwatch(RESET);
    const std::vector<VectorVector3d> projected_deformable_point_paths = dijkstras_task_->findPathFromObjectToTarget(starting_world_state, max_lookahead_steps_);

    const size_t actual_lookahead_steps = sizeOfLargestVector(projected_deformable_point_paths) - 1;
    // sizeOfLargest(...) should be at least 2, so this assert should always be true
    assert(actual_lookahead_steps <= max_lookahead_steps_);

    ROS_INFO_STREAM_NAMED("task_framework", "Calculated projected cloth paths                 - Version 1 - in " << stopwatch(READ) << " seconds");
    ROS_INFO_STREAM_NAMED("task_framework", "Max lookahead steps: " << max_lookahead_steps_ << " Actual steps: " << actual_lookahead_steps);

    visualizeProjectedPaths(projected_deformable_point_paths, visualization_enabled);
    projected_deformable_point_paths_and_projected_virtual_rubber_bands.first = projected_deformable_point_paths;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constraint violation Version 2a - Vector field forward "simulation" - rubber band
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO_STREAM_NAMED("task_framework", "Starting future constraint violation detection   - Version 2a - Total steps taken " << actual_lookahead_steps);
    assert(num_models_ == 1 && starting_world_state.all_grippers_single_pose_.size() == 2);
    const auto& correspondences = dijkstras_task_->getCoverPointCorrespondences(starting_world_state);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    WorldState world_state_copy = starting_world_state;
    RubberBand rubber_band_copy = *rubber_band_;
    rubber_band_copy.visualize(PROJECTED_BAND_NS, PREDICTION_RUBBER_BAND_SAFE_COLOR, PREDICTION_RUBBER_BAND_VIOLATION_COLOR, 1, visualization_enabled);

    projected_deformable_point_paths_and_projected_virtual_rubber_bands.second.reserve(actual_lookahead_steps);
    for (size_t t = 0; t < actual_lookahead_steps; ++t)
    {
        // We only want the error correction part of the movement
        DesiredDirection desired_object_manipulation_direction;
        desired_object_manipulation_direction.error_correction_ =
                dijkstras_task_->calculateErrorCorrectionDeltaFixedCorrespondences(world_state_copy, correspondences.correspondences_);
        desired_object_manipulation_direction.stretching_correction_ = ObjectDeltaAndWeight(world_state_copy.object_configuration_.size());
        desired_object_manipulation_direction.combined_correction_ = desired_object_manipulation_direction.error_correction_;

        // It is assumed that the robot's internal state matches what is passed to us, so we do not need to set active dof values
        const MatrixXd robot_dof_to_grippers_poses_jacobian = robot_->getGrippersJacobian();
        const std::vector<std::pair<CollisionData, Matrix3Xd>> full_robot_poi_collision_data_ = robot_->getPointsOfInterestCollisionData();

        const double normal_motion_grippers_max_step = robot_->max_gripper_velocity_norm_ * robot_->dt_;
        const double forward_prediction_grippers_max_step = dijkstras_task_->work_space_grid_.minStepDimension() * 1.1;
        const double velocity_scale_factor = forward_prediction_grippers_max_step / normal_motion_grippers_max_step;
        const double normal_motion_robot_dof_max_step = robot_->max_dof_velocity_norm_ * robot_->dt_;
        const double forward_prediction_robot_dof_max_step = velocity_scale_factor * normal_motion_robot_dof_max_step * 2.0;

        const bool handle_overstretch = false;
        const DeformableController::InputData input_data(
                    world_state_copy,
                    desired_object_manipulation_direction,
                    robot_,
                    robot_dof_to_grippers_poses_jacobian,
                    world_state_copy.robot_configuration_valid_,
                    full_robot_poi_collision_data_,
                    forward_prediction_grippers_max_step,
                    forward_prediction_robot_dof_max_step,
                    handle_overstretch);

        const DeformableController::OutputData robot_command = controller_list_[0]->getGripperMotion(input_data);

        /**
           Things to be updated in world_state_copy after "executing" a robot commad
                ObjectPointSet object_configuration_;
                AllGrippersSinglePose all_grippers_single_pose_;
                VectorXd robot_configuration_;
                bool robot_configuration_valid_;
                std::vector<CollisionData> gripper_collision_data_;
                double sim_time_;
         */

        // Move the grippers forward
        world_state_copy.all_grippers_single_pose_
                = kinematics::applyTwist(world_state_copy.all_grippers_single_pose_, robot_command.grippers_motion_);
        for (auto& pose : world_state_copy.all_grippers_single_pose_)
        {
            pose.translation() = dijkstras_task_->sdf_->ProjectOutOfCollisionToMinimumDistance3d(pose.translation(), GetRobotGripperRadius());
        }

        // Update the gripper collision data
        auto collision_check_future = std::async(std::launch::async, &RobotInterface::checkGripperCollision, robot_, world_state_copy.all_grippers_single_pose_);

        // Move the robot DOF forward
        world_state_copy.robot_configuration_ += robot_command.robot_dof_motion_;
        robot_->setActiveDOFValues(world_state_copy.robot_configuration_);

        // Move the cloth forward - copy the "projected" state of the cloth into the world_state_copy
        world_state_copy.sim_time_ += robot_->dt_;
        for (ssize_t node_ind = 0; node_ind < world_state_copy.object_configuration_.cols(); ++node_ind)
        {
            if (projected_deformable_point_paths[node_ind].size() > t + 1)
            {
                world_state_copy.object_configuration_.col(node_ind) = projected_deformable_point_paths[node_ind][t + 1];
            }
        }

        // Move the virtual rubber band to follow the grippers, projecting out of collision as needed
        rubber_band_copy.forwardPropagate(
                    ToGripperPositions(world_state_copy.all_grippers_single_pose_),
                    band_verbose);
        projected_deformable_point_paths_and_projected_virtual_rubber_bands.second.push_back(rubber_band_copy);

        // Visualize
        if (visualization_enabled)
        {
            rubber_band_copy.visualize(PROJECTED_BAND_NS, PREDICTION_RUBBER_BAND_SAFE_COLOR, PREDICTION_RUBBER_BAND_VIOLATION_COLOR, (int32_t)t + 2, visualization_enabled);
            vis_->visualizeGrippers(PROJECTED_GRIPPER_NS, world_state_copy.all_grippers_single_pose_, PREDICTION_GRIPPER_COLOR, (int32_t)(2 * t) + 2);
        }

        // Finish collecting the gripper collision data
        world_state_copy.gripper_collision_data_ = collision_check_future.get();
    }
    ROS_INFO_STREAM_NAMED("task_framework", "Calculated future constraint violation detection - Version 2a - in " << function_wide_stopwatch(READ) << " seconds");

    // Add duplicates of the last state to clear out any old visualizations
    if (visualization_enabled)
    {
        for (size_t t = actual_lookahead_steps; t < max_lookahead_steps_; ++t)
        {
            rubber_band_copy.visualize(PROJECTED_BAND_NS, PREDICTION_RUBBER_BAND_SAFE_COLOR, PREDICTION_RUBBER_BAND_VIOLATION_COLOR, (int32_t)t + 2, visualization_enabled);
            vis_->visualizeGrippers(PROJECTED_GRIPPER_NS, world_state_copy.all_grippers_single_pose_, PREDICTION_GRIPPER_COLOR, (int32_t)(2 * t) + 2);
        }
        vis_->forcePublishNow();
    }

    // Revert the robot state back to what it was before this was called
    robot_->setActiveDOFValues(starting_world_state.robot_configuration_);

    return projected_deformable_point_paths_and_projected_virtual_rubber_bands;
}

bool TaskFramework::globalPlannerNeededDueToOverstretch(
        const WorldState& current_world_state)
{
    static double annealing_factor = GetRubberBandOverstretchPredictionAnnealingFactor(*ph_);

    const bool visualization_enabled = true;
    const auto projection_results = projectFutureSystemState(current_world_state, visualization_enabled);
    const auto& projected_rubber_bands = projection_results.second;

    if (projected_rubber_bands.size() == 0)
    {
        return false;
    }

    double filtered_band_length = projected_rubber_bands.front().totalLength();

    for (size_t t = 0; t < projected_rubber_bands.size(); ++t)
    {
        const RubberBand& band = projected_rubber_bands[t];
        const double band_length = band.totalLength();
        const Pair3dPositions endpoints = band.getEndpoints();
        const double distance_between_endpoints = (endpoints.first - endpoints.second).norm();

        // Apply a low pass filter to the band length to try and remove "blips" in the estimate
        filtered_band_length = annealing_factor * filtered_band_length + (1.0 - annealing_factor) * band_length;

        // If the band is currently overstretched, and not in free space, then predict future problems
        if (filtered_band_length > band.maxSafeLength() && !CloseEnough(band_length, distance_between_endpoints, 1e-3))
        {
            return true;
        }
    }

    return false;
}

bool TaskFramework::globalPlannerNeededDueToLackOfProgress()
{
    static double error_delta_threshold_for_progress = GetErrorDeltaThresholdForProgress(*ph_);
    static double grippers_distance_delta_threshold_for_progress = GetGrippersDistanceDeltaThresholdForProgress(*ph_);

    // If we have not yet collected enough data, then assume we are not stuck
    if (grippers_pose_history_.size() < max_grippers_pose_history_length_)
    {
        return false;
    }

    assert(grippers_pose_history_.size() == max_grippers_pose_history_length_);

    // Calculate distances from the first gripper config to the last
    const AllGrippersSinglePose& start_config = grippers_pose_history_[0];
    const double start_error = error_history_[0];
    std::vector<double> grippers_distance_deltas(max_grippers_pose_history_length_ - 1);
    std::vector<double> error_deltas(max_grippers_pose_history_length_ - 1);
    for (size_t time_idx = 1; time_idx < max_grippers_pose_history_length_; ++time_idx)
    {
        const AllGrippersSinglePoseDelta grippers_delta = CalculateGrippersPoseDelta(start_config, grippers_pose_history_[time_idx]);
        const double distance = MultipleGrippersVelocity6dNorm(grippers_delta);
        grippers_distance_deltas[time_idx - 1] = distance;
        error_deltas[time_idx - 1] = error_history_[time_idx] - start_error;
    }

    if (bandits_logging_enabled_)
    {
        // Determine if there is a general positive slope on the distances
        // - we should be moving away from the start config if we are not stuck
        ARC_LOG(loggers_.at("grippers_distance_delta_history"), PrettyPrint::PrettyPrint(grippers_distance_deltas, false, ", "));

        ARC_LOG(loggers_.at("error_delta_history"), PrettyPrint::PrettyPrint(error_deltas, false, ", "));
    }

    // If error has not decreased sufficiently, then we may not be making progress
    const double error_improvemnt = start_error - error_history_.back();
    if (error_improvemnt < error_delta_threshold_for_progress)
    {
        // If error has not decreased sufficiently, and the grippers have not moved much, then we are not making progress
        const double grippers_distance_delta = grippers_distance_deltas.back();
        if (grippers_distance_delta < grippers_distance_delta_threshold_for_progress)
        {
            return true;
        }
    }

    return false;
}

bool TaskFramework::predictStuckForGlobalPlannerResults(const bool visualization_enabled)
{
    static double annealing_factor = GetRubberBandOverstretchPredictionAnnealingFactor(*ph_);

    ROS_WARN_THROTTLE_NAMED(4.0, "task_framework", "Only predicting stuck using the current segment, and not using transition learning in the process");

    const RRTPath& current_segment = rrt_planned_policy_[policy_current_idx_].first;
    assert(policy_segment_next_idx_ < current_segment.size());

    constexpr bool band_verbose = false;

    RubberBand band = *rubber_band_;

    bool overstretch_predicted = false;
    double filtered_band_length = band.totalLength();

    for (size_t t = 0; t < max_lookahead_steps_; ++t)
    {
        // Always predict the full number of steps, duplicating the last point in the path as needed
        const size_t next_idx = std::min(current_segment.size() - 1, policy_segment_next_idx_+ t);

        // Forward project the band and check for overstretch
        const auto& grippers_poses = current_segment[next_idx].grippers();
        band.forwardPropagate(
                    ToGripperPositions(grippers_poses),
                    band_verbose);
        const double band_length = band.totalLength();
        const Pair3dPositions endpoints = band.getEndpoints();
        const double distance_between_endpoints = (endpoints.first - endpoints.second).norm();

        filtered_band_length = annealing_factor * filtered_band_length + (1.0 - annealing_factor) * band_length;
        if (filtered_band_length > band.maxSafeLength() && !CloseEnough(band_length, distance_between_endpoints, 1e-3))
        {
            overstretch_predicted = true;
        }

        // Visualize
        band.visualize(PROJECTED_BAND_NS, PREDICTION_RUBBER_BAND_SAFE_COLOR, PREDICTION_RUBBER_BAND_VIOLATION_COLOR, (int32_t)t + 2, visualization_enabled);
        vis_->visualizeGrippers(PROJECTED_GRIPPER_NS, VectorIsometry3d{grippers_poses.first, grippers_poses.second}, PREDICTION_GRIPPER_COLOR, (int32_t)(2 * t) + 2);
    }

    vis_->forcePublishNow();

    return overstretch_predicted;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global gripper planner functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskFramework::initializeBand(const WorldState& world_state)
{
    assert(world_state.all_grippers_single_pose_.size() == 2);
    // Extract the maximum distance between the grippers
    // This assumes that the starting position of the grippers is at the maximum "unstretched" distance
    const auto& grippers_starting_poses = world_state.all_grippers_single_pose_;
    const double max_calced_band_length =
            (grippers_starting_poses[0].translation() - grippers_starting_poses[1].translation()).norm()
            * dijkstras_task_->maxStretchFactor();
    ROS_ERROR_STREAM_COND_NAMED(!CloseEnough(max_calced_band_length, dijkstras_task_->maxBandLength(), 1e-3),
                                "task_framework",
                                "Calc'd max band distance is: " << max_calced_band_length <<
                                " but the ros param saved distance is " << dijkstras_task_->maxBandLength() <<
                                ". Double check the stored value in the roslaunch file.");

    // Find the shortest path through the object, between the grippers, while follow nodes of the object.
    // Used to determine the starting position of the rubber band at each timestep
    const auto neighbour_fn = [&] (const ssize_t& node)
    {
        return dijkstras_task_->getNodeNeighbours(node);
    };

    // Create the initial rubber band
    const double resampled_band_max_pointwise_dist = dijkstras_task_->work_space_grid_.minStepDimension() / 2.0;
    const size_t upsampled_band_num_points = GetRRTBandMaxPoints(*ph_);

    rubber_band_ = std::make_shared<RubberBand>(
                nh_,
                ph_,
                vis_,
                dijkstras_task_->sdf_,
                dijkstras_task_->work_space_grid_,
                neighbour_fn,
                world_state,
                resampled_band_max_pointwise_dist,
                upsampled_band_num_points,
                dijkstras_task_->maxBandLength());
}

void TaskFramework::initializeBandRRT(const bool planning_for_whole_robot)
{
    assert(rubber_band_ != nullptr);

    // "World" params used by planning
    world_params_ = std::make_shared<const BandRRT::WorldParams>(BandRRT::WorldParams
    {
        robot_,
        planning_for_whole_robot,
        dijkstras_task_->sdf_,
        dijkstras_task_->work_space_grid_,
        transition_estimator_,
        generator_
    });

    // Algorithm parameters
    const auto use_cbirrt_style_projection      = GetUseCBiRRTStyleProjection(*ph_);
    const auto forward_tree_extend_iterations   = GetRRTForwardTreeExtendIterations(*ph_);
    const auto backward_tree_extend_iterations  = GetRRTBackwardTreeExtendIterations(*ph_);
    const auto kd_tree_grow_threshold           = GetRRTKdTreeGrowThreshold(*ph_);
    const auto use_brute_force_nn               = GetRRTUseBruteForceNN(*ph_);
    const auto goal_bias                        = GetRRTGoalBias(*ph_);
    const auto best_near_radius                 = GetRRTBestNearRadius(*ph_);
    const auto feasibility_dist_scale_factor    = GetRRTFeasibilityDistanceScaleFactor(*ph_);
    assert(!use_cbirrt_style_projection && "CBiRRT style projection is no longer supported");
    planning_params_ =
    {
        forward_tree_extend_iterations,
        backward_tree_extend_iterations,
        use_brute_force_nn,
        kd_tree_grow_threshold,
        best_near_radius * best_near_radius,
        goal_bias,
        feasibility_dist_scale_factor
    };

    // Smoothing parameters
    const auto max_shortcut_index_distance      = GetRRTMaxShortcutIndexDistance(*ph_);
    const auto max_smoothing_iterations         = GetRRTMaxSmoothingIterations(*ph_);
    const auto smoothing_band_dist_threshold    = GetRRTSmoothingBandDistThreshold(*ph_);
    smoothing_params_ =
    {
        max_shortcut_index_distance,
        max_smoothing_iterations,
        smoothing_band_dist_threshold
    };

    // Task defined parameters
    const auto task_aligned_frame = robot_->getWorldToTaskFrameTf();
    const auto task_frame_lower_limits = Vector3d(
                GetRRTPlanningXMinBulletFrame(*ph_),
                GetRRTPlanningYMinBulletFrame(*ph_),
                GetRRTPlanningZMinBulletFrame(*ph_));
    const auto task_frame_upper_limits = Vector3d(
                GetRRTPlanningXMaxBulletFrame(*ph_),
                GetRRTPlanningYMaxBulletFrame(*ph_),
                GetRRTPlanningZMaxBulletFrame(*ph_));
    const auto max_gripper_step_size                = dijkstras_task_->work_space_grid_.minStepDimension();
    const auto max_robot_step_size                  = GetRRTMaxRobotDOFStepSize(*ph_);
    const auto min_robot_step_size                  = GetRRTMinRobotDOFStepSize(*ph_);
    const auto max_gripper_rotation                 = GetRRTMaxGripperRotation(*ph_); // only matters for real robot
    const auto goal_reached_radius                  = dijkstras_task_->work_space_grid_.minStepDimension();
    const auto min_gripper_distance_to_obstacles    = GetRRTMinGripperDistanceToObstacles(*ph_); // only matters for simulation
    const auto band_distance2_scaling_factor        = GetRRTBandDistance2ScalingFactor(*ph_);
    const auto upsampled_band_num_points            = GetRRTBandMaxPoints(*ph_);
    task_params_ =
    {
        task_aligned_frame,
        task_frame_lower_limits,
        task_frame_upper_limits,
        max_gripper_step_size,
        max_robot_step_size,
        min_robot_step_size,
        max_gripper_rotation,
        goal_reached_radius,
        min_gripper_distance_to_obstacles,
        band_distance2_scaling_factor,
        upsampled_band_num_points
    };

    // Visualization
    const auto enable_rrt_visualizations = GetVisualizeRRT(*ph_);

    // Pass in all the config values that the RRT needs; for example goal bias, step size, etc.
    band_rrt_ = std::make_shared<BandRRT>(
                nh_,
                ph_,
                *world_params_,
                planning_params_,
                smoothing_params_,
                task_params_,
                rubber_band_,
                vis_,
                enable_rrt_visualizations);
}

AllGrippersSinglePose TaskFramework::getGripperTargets(const WorldState& world_state)
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////// Determine the cluster centers /////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    const auto& correspondences = dijkstras_task_->getCoverPointCorrespondences(world_state);
    const auto& cover_point_indices_= correspondences.uncovered_target_points_idxs_;

    // Only cluster the points that are not covered
    VectorVector3d cluster_targets;
    cluster_targets.reserve(cover_point_indices_.size());
    for (size_t idx = 0; idx < cover_point_indices_.size(); ++idx)
    {
        const ssize_t cover_idx = cover_point_indices_[idx];
        cluster_targets.push_back(dijkstras_task_->cover_points_.col(cover_idx));
    }

//    vis_->visualizePoints(CLUSTERING_TARGETS_NS, cluster_targets, Visualizer::Blue(), 1);

    const ObjectPointSet cluster_targets_as_matrix = VectorEigenVector3dToEigenMatrix3Xd(cluster_targets);
    const MatrixXd distance_matrix = CalculateSquaredDistanceMatrix(cluster_targets_as_matrix);

    // Get the 2 most disparate points to initialize the clustering
    ssize_t row, col;
    distance_matrix.maxCoeff(&row, &col);
    assert(row != col);
    const VectorVector3d starting_cluster_centers = {cluster_targets[row], cluster_targets[col]};

    // Cluster the target points using K-means, then extract the cluster centers
    const std::function<double(const Vector3d&, const Vector3d&)> distance_fn = [] (const Vector3d& v1, const Vector3d& v2)
    {
        return (v1 - v2).norm();
    };
    const std::function<Vector3d(const VectorVector3d&)> average_fn = [] (const VectorVector3d& data)
    {
        return AverageEigenVector3d(data);
    };
    const auto cluster_results = simple_kmeans_clustering::SimpleKMeansClustering::Cluster(cluster_targets, distance_fn, average_fn, starting_cluster_centers);
    const std::vector<uint32_t>& cluster_labels = cluster_results.first;
    const VectorVector3d cluster_centers = cluster_results.second;
    const uint32_t num_clusters = (uint32_t)cluster_centers.size();
    assert(num_clusters == 2);

//    vis_->visualizeCubes(CLUSTERING_RESULTS_PRE_PROJECT_NS, cluster_centers, Vector3d::Ones() * dijkstras_task_->work_space_grid_.minStepDimension(), Visualizer::Red(), 1);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////// Determine which gripper gets assigned which cluster center ////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Get the orientations for each gripper based on their starting orientation
    AllGrippersSinglePose target_gripper_poses = world_state.all_grippers_single_pose_;

    // Decide which gripper goes to which cluster
    {
        const auto& gripper0_grapsed_points = dijkstras_task_->getGripperAttachedNodesIndices(0);
        const auto& gripper1_grapsed_points = dijkstras_task_->getGripperAttachedNodesIndices(1);

        const auto gripper0_cluster_counts = numberOfPointsInEachCluster(cluster_labels, num_clusters, gripper0_grapsed_points, correspondences);
        const auto gripper1_cluster_counts = numberOfPointsInEachCluster(cluster_labels, num_clusters, gripper1_grapsed_points, correspondences);

        // Set some values so that the logic used in the if-else chain makes sense to read
        const bool gripper0_no_match_to_cluster0    = gripper0_cluster_counts[0] == 0;
        const bool gripper0_no_match_to_cluster1    = gripper0_cluster_counts[1] == 0;
        const bool gripper0_no_match_to_any_cluster = gripper0_no_match_to_cluster0 && gripper0_no_match_to_cluster1;
        const bool gripper0_best_match_to_cluster0  = gripper0_cluster_counts[0] > gripper1_cluster_counts[0]; // Note that this requires at least 1 correspondence for gripper0 to cluster0
        const bool gripper0_best_match_to_cluster1  = gripper0_cluster_counts[1] > gripper1_cluster_counts[1]; // Note that this requires at least 1 correspondence for gripper0 to cluster1

        const bool gripper1_no_match_to_cluster0    = gripper1_cluster_counts[0] == 0;
        const bool gripper1_no_match_to_cluster1    = gripper1_cluster_counts[1] == 0;
        const bool gripper1_no_match_to_any_cluster = gripper1_no_match_to_cluster0 && gripper1_no_match_to_cluster1;
        const bool gripper1_best_match_to_cluster0  = gripper1_cluster_counts[0] > gripper0_cluster_counts[0]; // Note that this requires at least 1 correspondence for gripper1 to cluster0
        const bool gripper1_best_match_to_cluster1  = gripper1_cluster_counts[1] > gripper0_cluster_counts[1]; // Note that this requires at least 1 correspondence for gripper1 to cluster1

        const bool equal_match_to_cluster0 = (!gripper0_no_match_to_cluster0) && (!gripper1_no_match_to_cluster0) && (gripper0_cluster_counts[0] == gripper1_cluster_counts[0]);
        const bool equal_match_to_cluster1 = (!gripper0_no_match_to_cluster1) && (!gripper1_no_match_to_cluster1) && (gripper0_cluster_counts[1] == gripper1_cluster_counts[1]);

        const bool gripper0_best_match_to_both = gripper0_best_match_to_cluster0 && gripper0_best_match_to_cluster1;
        const bool gripper1_best_match_to_both = gripper1_best_match_to_cluster0 && gripper1_best_match_to_cluster1;


        // If each gripper has a unique best pull direction, use it
        if (gripper0_best_match_to_cluster0 && gripper1_best_match_to_cluster1)
        {
            target_gripper_poses[0].translation() = cluster_centers[0];
            target_gripper_poses[1].translation() = cluster_centers[1];
        }
        else if (gripper0_best_match_to_cluster1 && gripper1_best_match_to_cluster0)
        {
            target_gripper_poses[0].translation() = cluster_centers[1];
            target_gripper_poses[1].translation() = cluster_centers[0];
        }
        // If a single gripper has the best pull to both, then that gripper dominates the choice
        else if (gripper0_best_match_to_both)
        {
            if (gripper0_cluster_counts[0] > gripper0_cluster_counts[1])
            {
                target_gripper_poses[0].translation() = cluster_centers[0];
                target_gripper_poses[1].translation() = cluster_centers[1];
            }
            else if (gripper0_cluster_counts[0] < gripper0_cluster_counts[1])
            {
                target_gripper_poses[0].translation() = cluster_centers[1];
                target_gripper_poses[1].translation() = cluster_centers[0];
            }
            // If gripper0 has no unique best target, then allow gripper1 to make the choice
            else
            {
                if (gripper1_cluster_counts[0] > gripper1_cluster_counts[1])
                {
                    target_gripper_poses[0].translation() = cluster_centers[1];
                    target_gripper_poses[1].translation() = cluster_centers[0];
                }
                else if (gripper1_cluster_counts[0] < gripper1_cluster_counts[1])
                {
                    target_gripper_poses[0].translation() = cluster_centers[0];
                    target_gripper_poses[1].translation() = cluster_centers[1];
                }
                // If everything is all tied up, decide what to do later
                else
                {
                    assert(false && "Setting gripper targets needs more logic");
                }
            }
        }
        else if (gripper1_best_match_to_both)
        {
            if (gripper1_cluster_counts[0] > gripper1_cluster_counts[1])
            {
                target_gripper_poses[0].translation() = cluster_centers[1];
                target_gripper_poses[1].translation() = cluster_centers[0];
            }
            else if (gripper1_cluster_counts[0] < gripper1_cluster_counts[1])
            {
                target_gripper_poses[0].translation() = cluster_centers[0];
                target_gripper_poses[1].translation() = cluster_centers[1];
            }
            // If gripper1 has no unique best target, then allow gripper0 to make the choice
            else
            {
                if (gripper0_cluster_counts[0] > gripper0_cluster_counts[1])
                {
                    target_gripper_poses[0].translation() = cluster_centers[0];
                    target_gripper_poses[1].translation() = cluster_centers[1];
                }
                else if (gripper0_cluster_counts[0] < gripper0_cluster_counts[1])
                {
                    target_gripper_poses[0].translation() = cluster_centers[1];
                    target_gripper_poses[1].translation() = cluster_centers[0];
                }
                // If everything is all tied up, decide what to do later
                else
                {
                    assert(false && "Setting gripper targets needs more logic");
                }
            }
        }
        // If there is only a pull on a single gripper, then that gripper dominates the choice
        else if (!gripper0_no_match_to_any_cluster &&  gripper1_no_match_to_any_cluster)
        {
            // Double check the logic that got us here; lets me simplify the resulting logic
            // Gripper1 has no pulls on it, and gripper0 has pulls from only 1 cluster, otherwise
            // one of the other caes would have triggered before this one
            assert(!gripper0_best_match_to_both);
            if (gripper0_best_match_to_cluster0)
            {
                assert(gripper0_no_match_to_cluster1);
                target_gripper_poses[0].translation() = cluster_centers[0];
                target_gripper_poses[1].translation() = cluster_centers[1];
            }
            else if (gripper0_best_match_to_cluster1)
            {
                assert(gripper0_no_match_to_cluster0);
                target_gripper_poses[0].translation() = cluster_centers[1];
                target_gripper_poses[1].translation() = cluster_centers[0];
            }
            else
            {
                assert(false && "Logic error in set gripper targets");
            }

        }
        else if ( gripper0_no_match_to_any_cluster && !gripper1_no_match_to_any_cluster)
        {
            // Double check the logic that got us here; lets me simplify the resulting logic
            // Gripper0 has no pulls on it, and gripper1 has pulls from only 1 cluster, otherwise
            // one of the other caes would have triggered before this one
            assert(!gripper1_best_match_to_both);
            if (gripper1_best_match_to_cluster0)
            {
                assert(gripper1_no_match_to_cluster1);
                target_gripper_poses[0].translation() = cluster_centers[1];
                target_gripper_poses[1].translation() = cluster_centers[0];
            }
            else if (gripper1_best_match_to_cluster1)
            {
                assert(gripper1_no_match_to_cluster0);
                target_gripper_poses[0].translation() = cluster_centers[0];
                target_gripper_poses[1].translation() = cluster_centers[1];
            }
            else
            {
                assert(false && "Logic error in set gripper targets");
            }
        }
        // If neither gripper has a pull on it, or both grippers have equal pull, then use some other metric
        else if ((gripper0_no_match_to_any_cluster  && gripper1_no_match_to_any_cluster) ||
                 (equal_match_to_cluster0           && equal_match_to_cluster1))
        {
            const std::vector<double> gripper0_distances_to_clusters =
                    dijkstras_task_->averageDijkstrasDistanceBetweenGrippersAndClusters(world_state.all_grippers_single_pose_[0], correspondences.uncovered_target_points_idxs_, cluster_labels, num_clusters);
            const std::vector<double> gripper1_distances_to_clusters =
                    dijkstras_task_->averageDijkstrasDistanceBetweenGrippersAndClusters(world_state.all_grippers_single_pose_[1], correspondences.uncovered_target_points_idxs_, cluster_labels, num_clusters);

            const bool gripper0_is_closest_to_cluster0 = gripper0_distances_to_clusters[0] <= gripper1_distances_to_clusters[0];
            const bool gripper0_is_closest_to_cluster1 = gripper0_distances_to_clusters[1] <= gripper1_distances_to_clusters[1];

            // If there is a unique best match, then use it
            if (gripper0_is_closest_to_cluster0 && !gripper0_is_closest_to_cluster1)
            {
                target_gripper_poses[0].translation() = cluster_centers[0];
                target_gripper_poses[1].translation() = cluster_centers[1];
            }
            else if (!gripper0_is_closest_to_cluster0 && gripper0_is_closest_to_cluster1)
            {
                target_gripper_poses[0].translation() = cluster_centers[1];
                target_gripper_poses[1].translation() = cluster_centers[0];
            }
            // Otherwise, pick the combination that minimizes the total distance
            else
            {
                const double dist_version0 = gripper0_distances_to_clusters[0] + gripper1_distances_to_clusters[1];
                const double dist_version1 = gripper0_distances_to_clusters[1] + gripper1_distances_to_clusters[0];

                if (dist_version0 <= dist_version1)
                {
                    target_gripper_poses[0].translation() = cluster_centers[0];
                    target_gripper_poses[1].translation() = cluster_centers[1];
                }
                else
                {
                    target_gripper_poses[0].translation() = cluster_centers[1];
                    target_gripper_poses[1].translation() = cluster_centers[0];
                }
            }
        }
        // If none of the above are true, than there is a logic error
        else
        {
            assert(false && "Unhandled edge case in get gripper targets");
        }

//        vis_->visualizeCubes(CLUSTERING_RESULTS_ASSIGNED_CENTERS_NS, {world_state.all_grippers_single_pose_[0].translation(), target_gripper_poses[0].translation()}, Vector3d::Ones() * dijkstras_task_->work_space_grid_.minStepDimension(), Visualizer::Magenta(), 1);
//        vis_->visualizeCubes(CLUSTERING_RESULTS_ASSIGNED_CENTERS_NS, {world_state.all_grippers_single_pose_[1].translation(), target_gripper_poses[1].translation()}, Vector3d::Ones() * dijkstras_task_->work_space_grid_.minStepDimension(), Visualizer::Cyan(), 5);
    }

    // Project the targets out of collision
    const double min_dist_to_obstacles = std::max(GetControllerMinDistanceToObstacles(*ph_), GetRRTMinGripperDistanceToObstacles(*ph_)) * GetRRTTargetMinDistanceScaleFactor(*ph_);
    const auto gripper_positions_pre_project = ToGripperPositions(target_gripper_poses);
    target_gripper_poses[0].translation() = dijkstras_task_->sdf_->ProjectOutOfCollisionToMinimumDistance3d(gripper_positions_pre_project.first, min_dist_to_obstacles);
    target_gripper_poses[1].translation() = dijkstras_task_->sdf_->ProjectOutOfCollisionToMinimumDistance3d(gripper_positions_pre_project.second, min_dist_to_obstacles);

    // Visualization
    {
        vis_->visualizeCubes(CLUSTERING_RESULTS_POST_PROJECT_NS, {target_gripper_poses[0].translation()}, Vector3d::Ones() * dijkstras_task_->work_space_grid_.minStepDimension(), band_rrt_->gripper_a_forward_tree_color_, 1);
        vis_->visualizeCubes(CLUSTERING_RESULTS_POST_PROJECT_NS, {target_gripper_poses[1].translation()}, Vector3d::Ones() * dijkstras_task_->work_space_grid_.minStepDimension(), band_rrt_->gripper_b_forward_tree_color_, 5);

        std::vector<std_msgs::ColorRGBA> colors;
        for (size_t idx = 0; idx < cluster_targets.size(); ++idx)
        {
            colors.push_back(arc_helpers::GenerateUniqueColor<std_msgs::ColorRGBA>(cluster_labels[idx] + 2, 0.5));
        }
        const std::vector<double> radiuses(cluster_targets.size(), dijkstras_task_->work_space_grid_.minStepDimension());
        vis_->visualizeSpheres(CLUSTERING_TARGETS_NS, cluster_targets, colors, 10, radiuses);
        vis_->forcePublishNow();
    }


//    std::cout << "cover_points = [\n" << dijkstras_task_->cover_points_ << "];\n";
//    std::cout << "cluster_targets = [\n" << PrettyPrint::PrettyPrint(cluster_targets, false, "\n") << "];\n";
//    std::cout << "cluster_labels = [" << PrettyPrint::PrettyPrint(cluster_labels, false, " ") << "];\n";
    return target_gripper_poses;
}

void TaskFramework::planGlobalGripperTrajectory(const WorldState& world_state)
{
    num_times_planner_invoked_++;
    ROS_INFO_STREAM_NAMED("task_framework", "!!!!!!!!!!!!!!!!!! Planner Invoked " << num_times_planner_invoked_ << " times!!!!!!!!!!!");
    vis_->purgeAndPublishDeleteAllAction();

    const RRTNode start_config = [&]
    {
        const RRTGrippersRepresentation gripper_config(
                    world_state.all_grippers_single_pose_[0],
                    world_state.all_grippers_single_pose_[1]);

        RRTRobotRepresentation robot_config;
        if (world_state.robot_configuration_valid_)
        {
            robot_config = world_state.robot_configuration_;
        }
        else
        {
            robot_config.resize(6);
            robot_config.head<3>() = gripper_config.first.translation();
            robot_config.tail<3>() = gripper_config.second.translation();
        }

        return RRTNode(gripper_config, robot_config, rubber_band_);
    }();
    const RRTGrippersRepresentation target_grippers_poses = ToGripperPosePair(getGripperTargets(world_state));

    rrt_planned_policy_.clear();
    if (GetRRTReuseOldResults(*ph_))
    {
        // Deserialization
        ROS_INFO_NAMED("rrt_planner_results", "Checking if RRT solution already exists");
        const std::string file_path =
                GetLogFolder(*nh_) +
                "rrt_cache_step." +
                PrettyPrint::PrettyPrint(num_times_planner_invoked_);
        rrt_planned_policy_ = band_rrt_->loadPolicy(file_path);

        if (world_state.robot_configuration_valid_)
        {
            std::cerr << "!!!!!!!!!!!!! About to execute a stored plan on a physical robot\n";
            std::cerr << "Are you sure about this? ";
            std::string input;
            std::cin >> input;
            if (input != "yes")
            {
                std::cerr << "Ignoring path loaded from file, replanning.\n";
                rrt_planned_policy_.clear();
            }
        }
    }

    // Planning if we did not load a plan from file
    if (rrt_planned_policy_.size() == 0)
    {
        const std::chrono::duration<double> time_limit(GetRRTTimeout(*ph_));

        rrt_planned_policy_.clear();
        do
        {
            rrt_planned_policy_ = band_rrt_->plan(
                        start_config,
                        target_grippers_poses,
                        time_limit);
        }
        while (rrt_planned_policy_.size() == 0);

        if (vis_->visualizationsEnabled())
        {
            vis_->deleteObjects(BandRRT::RRT_BLACKLISTED_GOAL_BANDS_NS, 1, 2);
            band_rrt_->visualizePolicy(rrt_planned_policy_);
            vis_->forcePublishNow(0.5);
        }

        // Serialization
        if (GetRRTStoreNewResults(*ph_))
        {
            ROS_INFO_NAMED("rrt_planner_results", "Compressing and saving RRT results to file");
            const std::string file_path =
                    GetLogFolder(*nh_) +
                    "rrt_cache_step." +
                    PrettyPrint::PrettyPrint(num_times_planner_invoked_);
            band_rrt_->storePolicy(rrt_planned_policy_, file_path);
        }
    }

    // We set the next index to "1" because the policy starts with the current configuration
    policy_current_idx_ = 0;
    policy_segment_next_idx_ = 1;
    executing_global_trajectory_ = true;
    microstep_history_buffer_.clear();

    rrt_executed_path_.clear();
    const TransitionEstimation::State tes =
    {
        world_state.object_configuration_,
        std::make_shared<RubberBand>(*rubber_band_),
        std::make_shared<RubberBand>(*rubber_band_),
        world_state.rope_node_transforms_
    };
    // The first state does not get any microstep history because this history is
    // how we got to the state, which does not make sense to have for the first (zero'th) state
    rrt_executed_path_.push_back({tes, std::vector<WorldState>(0)});
}

void TaskFramework::testPlanningPerformance(
        const WorldState& world_state,
        const unsigned long base_seed,
        const RRTNode& start_config,
        const RRTGrippersRepresentation& target_grippers_poses,
        const bool parallel_planning)
{
    static int num_batch_tests = 0;
    ++num_batch_tests;

    ROS_INFO_NAMED("task_framework", "Testing planning performance");

    const auto data_folder = GetDataFolder(*nh_);
    arc_utilities::CreateDirectory(data_folder);
    const std::string batch_name =
            "seed_" + IntToHex(base_seed) +
            "__batch_test_" + std::to_string(num_batch_tests) +
            "__" + arc_helpers::GetCurrentTimeAsString();
    storeWorldState(world_state, rubber_band_, data_folder + batch_name + "__starting_world_state.compressed");

    const auto enable_rrt_visualizations = !parallel_planning && GetVisualizeRRT(*ph_);
    if (enable_rrt_visualizations)
    {
        vis_->purgeAndPublishDeleteAllAction();
    }

    const std::chrono::duration<double> time_limit(GetRRTTimeout(*ph_));
    const size_t num_trials = GetRRTNumTrials(*ph_);
    const bool test_paths_in_bullet = GetRRTTestPathsInBullet(*ph_);

    std::vector<std::string> file_basenames(num_trials);
    std::vector<RRTPath> planned_paths(num_trials);
    // Stores the planning and smoothing stats for all trials;
    // each trial may have more than one planning attempt, so we need a vector of stats for each trial
    std::vector<std::vector<BandRRT::PlanningSmoothingStatistics>> statistics(num_trials);

    const auto omp_default_threads = arc_helpers::GetNumOMPThreads();
    const int omp_planning_threads = parallel_planning ? omp_default_threads : 1;
    #pragma omp parallel for num_threads(omp_planning_threads) schedule(guided)
    for (size_t trial_idx = 0; trial_idx < num_trials; ++trial_idx)
    {
        ROS_INFO_STREAM_NAMED("task_framework", "Planning performance trial idx: " << trial_idx);
        const auto basename = data_folder + batch_name + "__trial_idx_" + ToStrFill0(num_trials, trial_idx);
        const auto rrt_path_file = basename + "__rrt_path.compressed";
        file_basenames[trial_idx] = basename;

        // Update the seed for this particular trial
        // When done in parallel, resetting the robot makes no sense, so omit it
        if (!parallel_planning)
        {
            robot_->resetRandomSeeds(base_seed, trial_idx * 0xFFFF);
        }
        BandRRT::WorldParams world_params = *world_params_;
        world_params.generator_ = std::make_shared<std::mt19937_64>(*generator_);
        world_params.generator_->seed(base_seed);
        world_params.generator_->discard(trial_idx * 0xFFFF);
        world_params.transition_estimator_ =
                std::make_shared<TransitionEstimation>(
                    nh_, ph_, world_params.generator_, dijkstras_task_->sdf_, dijkstras_task_->work_space_grid_, vis_, *rubber_band_);

        // Pass in all the config values that the RRT needs; for example goal bias, step size, etc.
        auto band_rrt = BandRRT(nh_,
                                ph_,
                                world_params,
                                planning_params_,
                                smoothing_params_,
                                task_params_,
                                rubber_band_,
                                vis_,
                                enable_rrt_visualizations);
        // Update the blacklist of the RRT copy to match the old
        for (const auto& band : band_rrt_->getBlacklist())
        {
            band_rrt.addBandToBlacklist(*band);
        }

        const RRTPolicy policy = band_rrt.plan(start_config, target_grippers_poses, time_limit);
        statistics[trial_idx].push_back(band_rrt.getStatistics());

        if (enable_rrt_visualizations)
        {
            vis_->deleteObjects(BandRRT::RRT_BLACKLISTED_GOAL_BANDS_NS, 1, 2);
            band_rrt_->visualizePolicy(rrt_planned_policy_);
            vis_->forcePublishNow(0.5);
        }

        if (policy.size() == 0)
        {
            ROS_INFO_STREAM_NAMED("task_framework", "Planning failure, saving empty path to prefix: " << basename);
            BandRRT::SavePath({}, rrt_path_file);
        }
        else if (policy.size() == 1)
        {
            ROS_INFO_STREAM_NAMED("task_framework", "Saving path to prefix: " << basename);
            planned_paths[trial_idx] = policy[0].first;
            BandRRT::SavePath(policy[0].first, rrt_path_file);
        }
        else
        {
            ROS_FATAL_NAMED("task_framework", "This ought to not be possible");
            assert(policy.size() <= 1);
        }
    }
    Log::Log planning_tests_log(GetLogFolder(*nh_) + batch_name + "__planning_statistics.log", true);
    LogPlanningPerformanceData(planning_tests_log, statistics);

    if (test_paths_in_bullet)
    {
        // Execute the paths in bullet
        {
            std::vector<AllGrippersPoseTrajectory> test_paths;
            std::vector<std::string> test_filenames;
            for (size_t idx = 0; idx < num_trials; ++idx)
            {
                // It is assumed that the robot starts where the path is at idx 0, so trim that element from the planned path
                const auto& planned_path = planned_paths[idx];
                if (planned_path.size() > 0)
                {
                    const RRTPath commanded_path(planned_path.begin() + 1, planned_path.end());
                    assert(commanded_path.size() > 0 && "If this is false, it probably means that plan_start == plan_goal");
                    const auto robot_path = RRTPathToGrippersPoseTrajectory(commanded_path);
                    const auto interp_result = robot_->interpolateGrippersTrajectory(robot_path);
                    test_paths.push_back(interp_result.first);
                    test_filenames.push_back(file_basenames[idx] + "__path_test_results.compressed");
                }
            }
            robot_->testRobotPaths(test_paths, test_filenames, nullptr, false, false);
        }

        // Parse the results
        {
            std::atomic<int> num_planning_failures = 0;
            std::atomic<int> num_succesful_paths = 0;
            std::atomic<int> num_unsuccesful_paths = 0;
            const auto omp_evaluation_threads = (task_specification_->deformable_type_ == ROPE) ? omp_default_threads : 1;
            #pragma omp parallel for num_threads(omp_evaluation_threads)
            for (size_t idx = 0; idx < num_trials; ++idx)
            {
                const auto& basename = file_basenames[idx];
                const auto rrt_path_file = basename + "__rrt_path.compressed";
                const auto test_result_file = basename + "__path_test_results.compressed";
                const auto trajectory_file = basename + "__trajectory.compressed";

                try
                {
                    const auto rrt_path = BandRRT::LoadPath(rrt_path_file, *rubber_band_);
                    if (rrt_path.empty())
                    {
                        ++num_planning_failures;
                        continue;
                    }

                    const auto test_result = arc_utilities::RosMessageDeserializationWrapper<dmm::TestRobotPathsFeedback>(
                                ZlibHelpers::LoadFromFileAndDecompress(test_result_file), 0).first.test_result;
                    // This redoes work in the above for loop, but oh well.
                    const auto test_waypoint_indices = [&] ()
                    {
                        // It is assumed that the robot starts where the path is at idx 0, so trim that element from the planned path
                        RRTPath const commanded_path(rrt_path.begin() + 1, rrt_path.end());
                        assert(commanded_path.size() > 0 && "If this is false, it probably means that plan_start == plan_goal");
                        auto const robot_path = RRTPathToGrippersPoseTrajectory(commanded_path);
                        auto const interp_result = robot_->interpolateGrippersTrajectory(robot_path);
                        return interp_result.second;
                    }();

                    const auto traj_gen_result = ToTrajectory(world_state, rrt_path, test_result, test_waypoint_indices);
                    const auto& trajectory = traj_gen_result.first;
                    const auto parsed_cleanly = traj_gen_result.second;
                    transition_estimator_->saveTrajectory(trajectory, trajectory_file);
                    if (parsed_cleanly)
                    {
                        ++num_succesful_paths;
                    }
                    else
                    {
                        ++num_unsuccesful_paths;
                    }
                }
                catch (const std::runtime_error& ex)
                {
                    ROS_ERROR_STREAM_NAMED("task_framework", "Error evalutating trajectory: " << basename << ": " << ex.what());
                    ++num_unsuccesful_paths;
                }
            }

            std::stringstream classifier_success_rate;
            classifier_success_rate << transition_estimator_->classifierName() << "  "
                                    << transition_estimator_->featuresUsed() << "  "
                                    << "Total successful paths: " << num_succesful_paths << "  "
                                    << "Total unsuccessful paths: " << num_unsuccesful_paths << "  "
                                    << "Total planning failures: " << num_planning_failures;
            ROS_INFO_STREAM_NAMED("task_framework", classifier_success_rate.str());
            ARC_LOG_STREAM(planning_tests_log, classifier_success_rate.str());

//            ROS_INFO_NAMED("task_framwork", "Terminating.");
//            throw_arc_exception(std::runtime_error, "Bullet tests done, terminating.");
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Model list management
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskFramework::initializeModelAndControllerSet(const WorldState& initial_world_state)
{
    // Initialze each model type with the shared data
    DeformableModel::SetGrippersData(robot_->getGrippersData());
    DeformableModel::SetCallbackFunctions(std::bind(&RobotInterface::checkGripperCollision, robot_, std::placeholders::_1));
    DiminishingRigidityModel::SetInitialObjectConfiguration(GetObjectInitialConfiguration(*nh_));
    ConstraintJacobianModel::SetInitialObjectConfiguration(GetObjectInitialConfiguration(*nh_));

    const bool optimization_enabled = GetJacobianControllerOptimizationEnabled(*ph_);
    const TrialType trial_type = GetTrialType(*ph_);

    switch (trial_type)
    {
        case DIMINISHING_RIGIDITY_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_AVOIDANCE_CONTROLLER:
        {
            double translational_deformability, rotational_deformability;
            if (ph_->getParam("translational_deformability", translational_deformability) &&
                ph_->getParam("rotational_deformability", rotational_deformability))
            {
                ROS_INFO_STREAM_NAMED("task_framework", "Overriding deformability values to "
                                       << translational_deformability << " "
                                       << rotational_deformability);
            }
            else
            {
                translational_deformability = task_specification_->defaultDeformability();
                rotational_deformability = task_specification_->defaultDeformability();
                ROS_INFO_STREAM_NAMED("task_framework", "Using default deformability value of "
                                       << task_specification_->defaultDeformability());
            }

            const auto model = std::make_shared<DiminishingRigidityModel>(
                        nh_,
                        translational_deformability,
                        rotational_deformability);
            model_list_.push_back(model);
            controller_list_.push_back(std::make_shared<LeastSquaresControllerWithObjectAvoidance>(
                                           nh_, ph_, robot_, vis_, model,
                                           task_specification_->collisionScalingFactor(),
                                           optimization_enabled));
            break;
        }
        case ADAPTIVE_JACOBIAN_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_AVOIDANCE_CONTROLLER:
        {
            const auto tmp_model = DiminishingRigidityModel(nh_, task_specification_->defaultDeformability(), false);
            const auto starting_jacobian = tmp_model.computeGrippersToDeformableObjectJacobian(initial_world_state);

            const auto model = std::make_shared<AdaptiveJacobianModel>(
                        nh_,
                        starting_jacobian,
                        GetAdaptiveModelLearningRate(*ph_));
            model_list_.push_back(model);
            controller_list_.push_back(std::make_shared<LeastSquaresControllerWithObjectAvoidance>(
                                           nh_, ph_, robot_, vis_, model,
                                           task_specification_->collisionScalingFactor(),
                                           optimization_enabled));
            break;
        }
        case DIMINISHING_RIGIDITY_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_CONSTRAINT_CONTROLLER:
        {
            double translational_deformability, rotational_deformability;
            if (ph_->getParam("translational_deformability", translational_deformability) &&
                ph_->getParam("rotational_deformability", rotational_deformability))
            {
                ROS_INFO_STREAM_NAMED("task_framework", "Overriding deformability values to "
                                       << translational_deformability << " "
                                       << rotational_deformability);
            }
            else
            {
                translational_deformability = task_specification_->defaultDeformability();
                rotational_deformability = task_specification_->defaultDeformability();
                ROS_INFO_STREAM_NAMED("task_framework", "Using default deformability value of "
                                       << task_specification_->defaultDeformability());
            }

            const auto model = std::make_shared<DiminishingRigidityModel>(
                        nh_,
                        translational_deformability,
                        rotational_deformability);
            model_list_.push_back(model);
            controller_list_.push_back(std::make_shared<LeastSquaresControllerWithStretchingConstraint>(
                                           nh_, ph_, robot_, vis_, model));
            break;
        }
        case CONSTRAINT_SINGLE_MODEL_CONSTRAINT_CONTROLLER:
        {
            ROS_INFO_NAMED("task_framework", "Using constraint model and random sampling controller");

            const double translation_dir_deformability = GetConstraintTranslationalDir(*ph_);
            const double translation_dis_deformability = GetConstraintTranslationalDis(*ph_);
            const double rotation_deformability = GetConstraintRotational(*ph_);

            const auto sdf = GetEnvironmentSDF(*nh_);

            const auto model = std::make_shared<ConstraintJacobianModel>(
                        nh_,
                        translation_dir_deformability,
                        translation_dis_deformability,
                        rotation_deformability,
                        sdf);
            model_list_.push_back(model);
            controller_list_.push_back(std::make_shared<StretchingConstraintController>(
                                           nh_, ph_, robot_, vis_, model,
                                           sdf,
                                           generator_,
                                           GetStretchingConstraintControllerSolverType(*ph_),
                                           GetMaxSamplingCounts(*ph_)));
            break;
        }
        case DIMINISHING_RIGIDITY_SINGLE_MODEL_CONSTRAINT_CONTROLLER:
        {
            ROS_INFO_NAMED("task_framework", "Using dminishing model and random sampling controller");

            double translational_deformability, rotational_deformability;
            const auto sdf = GetEnvironmentSDF(*nh_);

            if (ph_->getParam("translational_deformability", translational_deformability) &&
                ph_->getParam("rotational_deformability", rotational_deformability))
            {
                ROS_INFO_STREAM_NAMED("task_framework", "Overriding deformability values to "
                                       << translational_deformability << " "
                                       << rotational_deformability);
            }
            else
            {
                translational_deformability = task_specification_->defaultDeformability();
                rotational_deformability = task_specification_->defaultDeformability();
                ROS_INFO_STREAM_NAMED("task_framework", "Using default deformability value of "
                                       << task_specification_->defaultDeformability());
            }

            const auto model = std::make_shared<DiminishingRigidityModel>(
                        nh_,
                        translational_deformability,
                        rotational_deformability);
            model_list_.push_back(model);
            controller_list_.push_back(std::make_shared<StretchingConstraintController>(
                                           nh_, ph_, robot_, vis_, model,
                                           sdf,
                                           generator_,
                                           GetStretchingConstraintControllerSolverType(*ph_),
                                           GetMaxSamplingCounts(*ph_)));
            break;
        }
        case MULTI_MODEL_BANDIT_TEST:
        {
            bool use_stretching_constraint_controller = false;
            if (initial_world_state.all_grippers_single_pose_.size() == 2)
            {
                use_stretching_constraint_controller = true;
                ROS_INFO_NAMED("task_framework", "Using least squares controllers with stretching constraint");
            }
            else
            {
                use_stretching_constraint_controller = false;
                ROS_INFO_NAMED("task_framework", "Using least squares controllers with stretching avoidance");
            }

            ////////////////////////////////////////////////////////////////////////
            // Diminishing rigidity models
            ////////////////////////////////////////////////////////////////////////
            {
                const double deform_min = GetDeformabilityRangeMin(*ph_);
                const double deform_max = GetDeformabilityRangeMax(*ph_);
                const double deform_step = GetDeformabilityRangeStep(*ph_);

                for (double trans_deform = deform_min; trans_deform < deform_max; trans_deform += deform_step)
                {
                    for (double rot_deform = deform_min; rot_deform < deform_max; rot_deform += deform_step)
                    {
                        const auto model = std::make_shared<DiminishingRigidityModel>(
                                    nh_,
                                    trans_deform,
                                    rot_deform);
                        model_list_.push_back(model);
                        if (use_stretching_constraint_controller)
                        {
                            controller_list_.push_back(std::make_shared<LeastSquaresControllerWithStretchingConstraint>(
                                                           nh_, ph_, robot_, vis_, model));
                        }
                        else
                        {
                            controller_list_.push_back(std::make_shared<LeastSquaresControllerWithObjectAvoidance>(
                                                           nh_, ph_, robot_, vis_, model,
                                                           task_specification_->collisionScalingFactor(),
                                                           optimization_enabled));
                        }
                    }
                }
                ROS_INFO_STREAM_NAMED("task_framework", "Num diminishing rigidity models: " << model_list_.size());
            }

            ////////////////////////////////////////////////////////////////////////
            // Adaptive jacobian models
            ////////////////////////////////////////////////////////////////////////
            {
                const double learning_rate_min = GetAdaptiveLearningRateRangeMin(*ph_);
                const double learning_rate_max = GetAdaptiveLearningRateRangeMax(*ph_);
                const double learning_rate_step = GetAdaptiveLearningRateRangeStep(*ph_);

                const auto tmp_model = DiminishingRigidityModel(nh_, task_specification_->defaultDeformability(), false);
                const auto starting_jacobian = tmp_model.computeGrippersToDeformableObjectJacobian(initial_world_state);

                for (double learning_rate = learning_rate_min; learning_rate < learning_rate_max; learning_rate *= learning_rate_step)
                {
                    const auto model = std::make_shared<AdaptiveJacobianModel>(
                                nh_,
                                starting_jacobian,
                                learning_rate);
                    model_list_.push_back(model);

                    if (use_stretching_constraint_controller)
                    {
                        controller_list_.push_back(std::make_shared<LeastSquaresControllerWithStretchingConstraint>(
                                                       nh_, ph_, robot_, vis_, model));
                    }
                    else
                    {
                        controller_list_.push_back(std::make_shared<LeastSquaresControllerWithObjectAvoidance>(
                                                       nh_, ph_, robot_, vis_, model,
                                                       task_specification_->collisionScalingFactor(),
                                                       optimization_enabled));
                    }
                }
                ROS_INFO_STREAM_NAMED("task_framework", "Num adaptive Jacobian models: "
                                       << std::floor(std::log(learning_rate_max / learning_rate_min) / std::log(learning_rate_step)));
            }
            break;
        }
        case MULTI_MODEL_CONTROLLER_TEST:
        {
            ROS_INFO_NAMED("task_framework", "Using multiple model-controller sets");

            // Constraint Model with New Controller. (MM)
            {
                const auto sdf = GetEnvironmentSDF(*nh_);

                const double translation_dir_deformability = GetConstraintTranslationalDir(*ph_);
                const double translation_dis_deformability = GetConstraintTranslationalDis(*ph_);
                const double rotation_deformability = GetConstraintRotational(*ph_);

                const auto model = std::make_shared<ConstraintJacobianModel>(
                            nh_,
                            translation_dir_deformability,
                            translation_dis_deformability,
                            rotation_deformability,
                            sdf);
                model_list_.push_back(model);
                controller_list_.push_back(std::make_shared<StretchingConstraintController>(
                                               nh_, ph_, robot_, vis_, model,
                                               sdf,
                                               generator_,
                                               GetStretchingConstraintControllerSolverType(*ph_),
                                               GetMaxSamplingCounts(*ph_)));
            }

            // Dminishing Model with Old Controller. (DD)
            {
                double translational_deformability, rotational_deformability;
                if (ph_->getParam("translational_deformability", translational_deformability) &&
                    ph_->getParam("rotational_deformability", rotational_deformability))
                {
                    ROS_INFO_STREAM_NAMED("task_framework", "Overriding deformability values to "
                                           << translational_deformability << " "
                                           << rotational_deformability);
                }
                else
                {
                    translational_deformability = task_specification_->defaultDeformability();
                    rotational_deformability = task_specification_->defaultDeformability();
                    ROS_INFO_STREAM_NAMED("task_framework", "Using default deformability value of "
                                           << task_specification_->defaultDeformability());
                }

                const auto model = std::make_shared<DiminishingRigidityModel>(
                            nh_,
                            translational_deformability,
                            rotational_deformability);
                model_list_.push_back(model);

                controller_list_.push_back(std::make_shared<LeastSquaresControllerWithObjectAvoidance>(
                                               nh_, ph_, robot_, vis_, model,
                                               task_specification_->collisionScalingFactor(),
                                               optimization_enabled));
            }
            break;
        }
        case MULTI_MODEL_ACCURACY_TEST:
        {
            // Constraint Model
            {
                const auto sdf = GetEnvironmentSDF(*nh_);

                const double translation_dir_deformability = GetConstraintTranslationalDir(*ph_);
                const double translation_dis_deformability = GetConstraintTranslationalDis(*ph_);
                const double rotation_deformability = GetConstraintRotational(*ph_);

                const auto model = std::make_shared<ConstraintJacobianModel>(
                            nh_,
                            translation_dir_deformability,
                            translation_dis_deformability,
                            rotation_deformability,
                            sdf);
                model_list_.push_back(model);
                controller_list_.push_back(std::make_shared<StraightLineController>(
                                               nh_, ph_, robot_, vis_, model));
            }
            // Dminishing Rigidity Model
            {
                double translational_deformability, rotational_deformability;
                if (ph_->getParam("translational_deformability", translational_deformability) &&
                    ph_->getParam("rotational_deformability", rotational_deformability))
                {
                    ROS_INFO_STREAM_NAMED("task_framework", "Overriding deformability values to "
                                           << translational_deformability << " "
                                           << rotational_deformability);
                }
                else
                {
                    translational_deformability = task_specification_->defaultDeformability();
                    rotational_deformability = task_specification_->defaultDeformability();
                    ROS_INFO_STREAM_NAMED("task_framework", "Using default deformability value of "
                                           << task_specification_->defaultDeformability());
                }

                const auto model = std::make_shared<DiminishingRigidityModel>(
                            nh_,
                            translational_deformability,
                            rotational_deformability);
                model_list_.push_back(model);
                controller_list_.push_back(std::make_shared<StraightLineController>(
                                               nh_, ph_, robot_, vis_, model));
            }
            break;
        }
        default:
        {
            ROS_FATAL_NAMED("task_framework", "Invalid trial type, this should not be possible.");
            assert(false && "Invalid trial type, this should not be possible.");
        }
    }

    assert(controller_list_.size() == model_list_.size());

    createBandits();
}

void TaskFramework::createBandits()
{
    num_models_ = (ssize_t)model_list_.size();
    ROS_INFO_STREAM_NAMED("task_framework", "Generating bandits for " << num_models_ << " bandits");

    switch (mab_algorithm_)
    {
        case UCB1Normal:
        {
            model_utility_bandit_ = std::make_shared<UCB1NormalBandit>((size_t)num_models_);
            break;
        }

        case KFMANB:
        {
            model_utility_bandit_ = std::make_shared<KalmanFilterMANB>(
                    VectorXd::Zero(num_models_), VectorXd::Ones(num_models_) * 1e6);
            break;
        }

        case KFMANDB:
        {
            model_utility_bandit_ = std::make_shared<KalmanFilterMANDB>(
                    VectorXd::Zero(num_models_), MatrixXd::Identity(num_models_, num_models_) * 1e6);
            break;
        }

        default:
            assert(false && "Impossibu!");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Model utility functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Planner::updateModels
 * @param suggested_trajectories
 * @param model_used
 * @param world_feedback
 */
void TaskFramework::updateModels(
        const WorldState& starting_world_state,
        const ObjectDeltaAndWeight& task_desired_motion,
        const std::vector<DeformableController::OutputData>& suggested_commands,
        const ssize_t model_used,
        const WorldState& world_feedback)
{
    const static double reward_annealing_factor = GetRewardScaleAnnealingFactor(*ph_);

    // First we update the bandit algorithm
    const double starting_error = task_specification_->calculateError(starting_world_state);
    const double true_error_reduction = starting_error - task_specification_->calculateError(world_feedback);
    reward_std_dev_scale_factor_ = std::max(1e-10, reward_annealing_factor * reward_std_dev_scale_factor_ + (1.0 - reward_annealing_factor) * std::abs(true_error_reduction));
    const double process_noise_scaling_factor = process_noise_factor_ * std::pow(reward_std_dev_scale_factor_, 2);
    const double observation_noise_scaling_factor = observation_noise_factor_ * std::pow(reward_std_dev_scale_factor_, 2);

    switch (mab_algorithm_)
    {
        case UCB1Normal:
        {
            (void)task_desired_motion;
            (void)suggested_commands;
            (void)process_noise_scaling_factor;
            (void)observation_noise_scaling_factor;
            model_utility_bandit_->updateArms(model_used, true_error_reduction);
            break;
        }

        case KFMANB:
        {
            (void)task_desired_motion;
            (void)suggested_commands;
            model_utility_bandit_->updateArms(
                        process_noise_scaling_factor * VectorXd::Ones(num_models_),
                        model_used,
                        true_error_reduction,
                        observation_noise_scaling_factor * 1.0);
            break;
        }

        case KFMANDB:
        {
            (void)task_desired_motion;

            const MatrixXd process_noise = calculateProcessNoise(suggested_commands);
            MatrixXd observation_matrix = RowVectorXd::Zero(num_models_);
            observation_matrix(0, model_used) = 1.0;
            const VectorXd observed_reward = VectorXd::Ones(1) * true_error_reduction;
            const MatrixXd observation_noise = MatrixXd::Ones(1, 1);

            model_utility_bandit_->updateArms(
                        process_noise_scaling_factor * process_noise,
                        observation_matrix,
                        observed_reward,
                        observation_noise_scaling_factor * observation_noise);
            break;
        }

        default:
        {
            assert(false && "Impossibu!");
        }
    }

    // Then we allow each model to update itself based on the new data
    #pragma omp parallel for
    for (size_t model_ind = 0; model_ind < (size_t)num_models_; model_ind++)
    {
        model_list_[model_ind]->updateModel(starting_world_state, world_feedback);
    }
}

/**
 * @brief Planner::calculateProcessNoise
 * @param suggested_commands
 * @return
 */
MatrixXd TaskFramework::calculateProcessNoise(
        const std::vector<DeformableController::OutputData>& suggested_commands) const
{
    std::vector<double> grippers_velocity_norms((size_t)num_models_);

    for (size_t model_ind = 0; model_ind < (size_t)num_models_; model_ind++)
    {
        grippers_velocity_norms[model_ind] = MultipleGrippersVelocity6dNorm(suggested_commands[model_ind].grippers_motion_);
    }

    MatrixXd process_noise = MatrixXd::Identity(num_models_, num_models_);
    for (ssize_t i = 0; i < num_models_; i++)
    {
        for (ssize_t j = i+1; j < num_models_; j++)
        {
            if (grippers_velocity_norms[(size_t)i] != 0 &&
                grippers_velocity_norms[(size_t)j] != 0)
            {
                process_noise(i, j) =
                        MultipleGrippersVelocityDotProduct(
                            suggested_commands[(size_t)i].grippers_motion_,
                            suggested_commands[(size_t)j].grippers_motion_)
                        / (grippers_velocity_norms[(size_t)i] * grippers_velocity_norms[(size_t)j]);
            }
            else if (grippers_velocity_norms[(size_t)i] == 0 &&
                     grippers_velocity_norms[(size_t)j] == 0)
            {
                process_noise(i, j) = 1;
            }
            else
            {
                process_noise(i, j) = 0;
            }

            process_noise(j, i) = process_noise(i, j);
        }
    }

    return correlation_strength_factor_ * process_noise + (1.0 - correlation_strength_factor_) * MatrixXd::Identity(num_models_, num_models_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Logging and visualization functionality
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskFramework::visualizeDesiredMotion(
        const WorldState& current_world_state,
        const ObjectDeltaAndWeight& desired_motion,
        const bool visualization_enabled) const
{
    if (visualization_enabled)
    {
        ssize_t num_nodes = current_world_state.object_configuration_.cols();
        std::vector<std_msgs::ColorRGBA> colors((size_t)num_nodes);
        for (size_t node_ind = 0; node_ind < (size_t)num_nodes; node_ind++)
        {
            colors[node_ind].r = (float)desired_motion.weight((ssize_t)node_ind * 3);
            colors[node_ind].g = 0.0f;
            colors[node_ind].b = 0.0f;
            colors[node_ind].a = desired_motion.weight((ssize_t)node_ind * 3) > 0 ? 1.0f : 0.0f;
        }
        task_specification_->visualizeDeformableObject(
                DESIRED_DELTA_NS,
                AddObjectDelta(current_world_state.object_configuration_, desired_motion.delta),
                colors);

//        if (task_specification_->deformable_type_ == DeformableType::CLOTH)
        {
            vis_->visualizeObjectDelta(
                        DESIRED_DELTA_NS,
                        current_world_state.object_configuration_,
                        AddObjectDelta(current_world_state.object_configuration_, desired_motion.delta),
                        Visualizer::Green(),
                        10);
        }
    }
}

void TaskFramework::visualizeGripperMotion(
        const AllGrippersSinglePose& current_gripper_pose,
        const AllGrippersSinglePoseDelta& gripper_motion,
        const ssize_t model_ind) const
{
    const auto grippers_test_poses = kinematics::applyTwist(current_gripper_pose, gripper_motion);
    EigenHelpers::VectorVector3d line_starts;
    EigenHelpers::VectorVector3d line_ends;

    for (size_t gripper_ind = 0; gripper_ind < current_gripper_pose.size(); gripper_ind++)
    {
        line_starts.push_back(current_gripper_pose[gripper_ind].translation());
        line_ends.push_back(current_gripper_pose[gripper_ind].translation() + 100.0 * (grippers_test_poses[gripper_ind].translation() - current_gripper_pose[gripper_ind].translation()));
    }

    switch (model_ind)
    {
        case 0:
        {
            vis_->visualizeLines("MM_grippers_motion", line_starts, line_ends, Visualizer::Silver());
            vis_->visualizeLines("MM_grippers_motion", line_starts, line_ends, Visualizer::Silver());
            vis_->visualizeLines("MM_grippers_motion", line_starts, line_ends, Visualizer::Silver());
            vis_->visualizeLines("MM_grippers_motion", line_starts, line_ends, Visualizer::Silver());
            break;
        }
        case 1:
        {
            vis_->visualizeLines("DD_grippers_motion", line_starts, line_ends, Visualizer::Yellow());
            vis_->visualizeLines("DD_grippers_motion", line_starts, line_ends, Visualizer::Yellow());
            vis_->visualizeLines("DD_grippers_motion", line_starts, line_ends, Visualizer::Yellow());
            vis_->visualizeLines("DD_grippers_motion", line_starts, line_ends, Visualizer::Yellow());
            break;
        }
        default:
        {
            assert(false && "grippers_motion color not assigned for this index");
            break;
        }
    }
}

void TaskFramework::initializeBanditsLogging()
{
    if (bandits_logging_enabled_)
    {
        const std::string log_folder = GetLogFolder(*nh_);
        ROS_INFO_STREAM_NAMED("task_framework", "Logging to " << log_folder);

        Log::Log seed_log(log_folder + "seed.txt", false);
        ARC_LOG_STREAM(seed_log, std::hex << seed_);

        loggers_.insert(std::make_pair<std::string, Log::Log>(
                            "time",
                            Log::Log(log_folder + "time.txt", false)));

        loggers_.insert(std::make_pair<std::string, Log::Log>(
                            "error",
                            Log::Log(log_folder + "error.txt", false)));

        loggers_.insert(std::make_pair<std::string, Log::Log>(
                            "utility_mean",
                            Log::Log(log_folder + "utility_mean.txt", false)));

        loggers_.insert(std::make_pair<std::string, Log::Log>(
                            "utility_covariance",
                            Log::Log(log_folder + "utility_covariance.txt", false)));

        loggers_.insert(std::make_pair<std::string, Log::Log>(
                            "model_chosen",
                            Log::Log(log_folder + "model_chosen.txt", false)));

        loggers_.insert(std::make_pair<std::string, Log::Log>(
                            "rewards_for_all_models",
                            Log::Log(log_folder + "rewards_for_all_models.txt", false)));

        loggers_.insert(std::make_pair<std::string, Log::Log>(
                            "grippers_distance_delta_history",
                            Log::Log(log_folder + "grippers_distance_delta_history.txt", false)));

        loggers_.insert(std::make_pair<std::string, Log::Log>(
                            "error_delta_history",
                            Log::Log(log_folder + "error_delta_history.txt", false)));
    }
}

void TaskFramework::initializeControllerLogging()
{
    if(controller_logging_enabled_)
    {
        const std::string log_folder = GetLogFolder(*nh_);
        ROS_INFO_STREAM_NAMED("task_framework", "Logging to " << log_folder);

        Log::Log seed_log(log_folder + "seed.txt", false);
        ARC_LOG_STREAM(seed_log, std::hex << seed_);

        controller_loggers_.insert(std::make_pair<std::string, Log::Log>(
                                       "control_time",
                                       Log::Log(log_folder + "control_time.txt", false)));

        controller_loggers_.insert(std::make_pair<std::string, Log::Log>(
                                       "control_error_realtime",
                                       Log::Log(log_folder + "control_error_realtime.txt", false)));

        controller_loggers_.insert(std::make_pair<std::string, Log::Log>(
                                       "realtime_stretching_factor",
                                       Log::Log(log_folder + "realtime_stretching_factor.txt", false)));

        controller_loggers_.insert(std::make_pair<std::string, Log::Log>(
                                       "individual_computation_times",
                                       Log::Log(log_folder + "individual_computation_times.txt", false)));

        controller_loggers_.insert(std::make_pair<std::string, Log::Log>(
                                       "model_prediction_error_weighted",
                                       Log::Log(log_folder + "model_prediction_error_weighted.txt", false)));

        controller_loggers_.insert(std::make_pair<std::string, Log::Log>(
                                       "model_prediction_error_unweighted",
                                       Log::Log(log_folder + "model_prediction_error_unweighted.txt", false)));
    }
}

// Note that resulting_world_state may not be exactly indentical to individual_model_rewards[model_used]
// because of the way forking works (and doesn't) in Bullet. They should be very close however.
void TaskFramework::logBanditsData(
        const WorldState& initial_world_state,
        const WorldState& resulting_world_state,
        const std::vector<WorldState>& individual_model_results,
        const VectorXd& model_utility_mean,
        const MatrixXd& model_utility_second_stat,
        const ssize_t model_used)
{
    if (bandits_logging_enabled_)
    {
        std::vector<double> rewards_for_all_models(num_models_, std::numeric_limits<double>::quiet_NaN());
        if (collect_results_for_all_models_)
        {
            const double prev_error = task_specification_->calculateError(initial_world_state);
            for (ssize_t model_ind = 0; model_ind < num_models_; ++model_ind)
            {
                const double current_error = task_specification_->calculateError(individual_model_results[(size_t)model_ind]);
                rewards_for_all_models[(size_t)model_ind] = prev_error - current_error;
            }
        }

        const static IOFormat single_line(
                    StreamPrecision,
                    DontAlignCols,
                    " ", " ", "", "");

        ARC_LOG(loggers_.at("time"),
             resulting_world_state.sim_time_);

        ARC_LOG(loggers_.at("error"),
             task_specification_->calculateError(resulting_world_state));

        ARC_LOG(loggers_.at("utility_mean"),
             model_utility_mean.format(single_line));

        ARC_LOG(loggers_.at("utility_covariance"),
             model_utility_second_stat.format(single_line));

        ARC_LOG(loggers_.at("model_chosen"),
             model_used);

        ARC_LOG(loggers_.at("rewards_for_all_models"),
            PrettyPrint::PrettyPrint(rewards_for_all_models, false, " "));
    }
}

// Note that resulting_world_state may not be exactly indentical to individual_model_rewards[model_used]
// because of the way forking works (and doesn't) in Bullet. They should be very close however.
void TaskFramework::controllerLogData(
        const WorldState& initial_world_state,
        const WorldState& resulting_world_state,
        const std::vector<WorldState>& individual_model_results,
        const DeformableController::InputData& input_data,
        const std::vector<double>& individual_computation_times,
        const std::vector<double>& model_prediction_errors_weighted,
        const std::vector<double>& model_prediction_errors_unweighted)
{
    if (controller_logging_enabled_)
    {
        // This function only works properly if we've collected all the data for each model,
        // so make sure the code crashes at a known point if that's the case
        assert(collect_results_for_all_models_);

        // Split out data used for computation for each model
        const ObjectDeltaAndWeight& task_desired_error_correction = input_data.desired_object_motion_.error_correction_;
        const VectorXd& desired_p_dot = task_desired_error_correction.delta;
        const VectorXd& desired_p_dot_weight = task_desired_error_correction.weight;
        const ssize_t num_grippers = initial_world_state.all_grippers_single_pose_.size();
        const ssize_t num_nodes = initial_world_state.object_configuration_.cols();

        // Data used by the function, per model
        std::vector<double> avg_control_error(num_models_, 0.0);
        std::vector<double> current_stretching_factor(num_models_, 0.0);

        for (size_t model_ind = 0; (ssize_t)model_ind < num_models_; ++model_ind)
        {
            // Get control errors for different model-controller sets.
            const ObjectPointSet real_p_dot =
                    individual_model_results[model_ind].object_configuration_
                    - initial_world_state.object_configuration_;

            int point_count = 0;
            double max_stretching = 0.0;
            double desired_p_dot_avg_norm = 0.0;
            double desired_p_dot_max = 0.0;

            // Calculate stretching factor
            const MatrixXd node_distance =
                    CalculateDistanceMatrix(individual_model_results[model_ind].object_configuration_);

            for (ssize_t node_ind = 0; node_ind < num_nodes; node_ind++)
            {
                const double point_weight = desired_p_dot_weight(node_ind * 3);
                if (point_weight > 0.0)
                {
                    //  Calculate p_dot error
                    const Vector3d& point_real_p_dot = real_p_dot.col(node_ind);
                    const Vector3d& point_desired_p_dot = desired_p_dot.segment<3>(node_ind * 3);
                    avg_control_error[model_ind] += (point_real_p_dot - point_desired_p_dot).norm();

                    desired_p_dot_avg_norm += point_desired_p_dot.norm();
                    if (point_desired_p_dot.norm() > desired_p_dot_max)
                    {
                        desired_p_dot_max = point_desired_p_dot.norm();
                    }

                    point_count++;
                }

                const ssize_t first_node = node_ind;
                for (ssize_t second_node = first_node + 1; second_node < num_nodes; ++second_node)
                {
                    const double this_stretching_factor = node_distance(first_node, second_node)
                            / object_initial_node_distance_(first_node, second_node);
                    max_stretching = std::max(max_stretching, this_stretching_factor);
                }
            }

            if (point_count > 0)
            {
                avg_control_error[model_ind] = avg_control_error[model_ind] / point_count;
                desired_p_dot_avg_norm /= point_count;
            }

            // Catch cases where the grippers and the nodes don't align, this should still be flagged as large stretch
            if (num_grippers == 2)
            {
                const auto gripper_delta =
                        individual_model_results[model_ind].all_grippers_single_pose_.at(0).translation()
                        - individual_model_results[model_ind].all_grippers_single_pose_.at(1).translation();
                const double this_stretching_factor = gripper_delta.norm() / initial_grippers_distance_;
                max_stretching = std::max(max_stretching, this_stretching_factor);
            }
            current_stretching_factor[model_ind] = max_stretching;
            ROS_INFO_STREAM_NAMED("task_framework", "average desired p dot is       " << desired_p_dot_avg_norm);
            ROS_INFO_STREAM_NAMED("task_framework", "max pointwise desired p dot is " << desired_p_dot_max);
        }

        // Do the actual logging itself
        const static IOFormat single_line(
                    StreamPrecision,
                    DontAlignCols,
                    " ", " ", "", "");

        ARC_LOG(controller_loggers_.at("control_time"),
            resulting_world_state.sim_time_);

        ARC_LOG(controller_loggers_.at("control_error_realtime"),
            PrettyPrint::PrettyPrint(avg_control_error, false, " "));

        ARC_LOG(controller_loggers_.at("realtime_stretching_factor"),
            PrettyPrint::PrettyPrint(current_stretching_factor, false, " "));

        ARC_LOG(controller_loggers_.at("individual_computation_times"),
            PrettyPrint::PrettyPrint(individual_computation_times, false, " "));

        ARC_LOG(controller_loggers_.at("model_prediction_error_weighted"),
            PrettyPrint::PrettyPrint(model_prediction_errors_weighted, false, " "));

        ARC_LOG(controller_loggers_.at("model_prediction_error_unweighted"),
            PrettyPrint::PrettyPrint(model_prediction_errors_unweighted, false, " "));
    }
}

void TaskFramework::LogPlanningPerformanceData(
        Log::Log& log,
        const std::vector<std::vector<BandRRT::PlanningSmoothingStatistics>>& statistics)
{
    for (size_t trial_idx = 0; trial_idx < statistics.size(); ++trial_idx)
    {
        const auto& trial_stats = statistics[trial_idx];
        ARC_LOG_STREAM(log, "Trial idx " << trial_idx << " planning iterations: " << trial_stats.size());
        for (size_t idx = 0; idx < trial_stats.size(); ++idx)
        {
            ARC_LOG(log, PrettyPrint::PrettyPrint(trial_stats[idx].first, false, "\n"));
            ARC_LOG(log, PrettyPrint::PrettyPrint(trial_stats[idx].second, false, "\n"));
        }
        ARC_LOG(log, "\n");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debugging
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskFramework::storeWorldState(const WorldState& world_state, const RubberBand::ConstPtr band, std::string filename)
{
    try
    {
        if (filename.empty())
        {
            const auto log_folder = GetLogFolder(*nh_);
            arc_utilities::CreateDirectory(log_folder);
            const auto file_name_prefix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "world_state/file_name_prefix", __func__);
            const std::string file_name_suffix = arc_helpers::GetCurrentTimeAsStringWithMilliseconds();
            const std::string file_name = file_name_prefix + "__" + file_name_suffix + ".compressed";
            filename = log_folder + file_name;
        }
        ROS_INFO_STREAM_NAMED("task_framework", "Saving world_state to " << filename);

        std::vector<uint8_t> buffer;
        world_state.serializeSelf(buffer);
        band->serialize(buffer);
        arc_utilities::CreateDirectory(boost::filesystem::path(filename).parent_path());
        ZlibHelpers::CompressAndWriteToFile(buffer, filename);

        const auto deserialized_results = WorldState::Deserialize(buffer, 0);
        assert(deserialized_results.first == world_state);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM_NAMED("task_framework", "Failed to store world_state to file: "  <<  e.what());
    }
}

std::pair<WorldState, RubberBand::Ptr> TaskFramework::loadStoredWorldState(std::string filename)
{
    std::pair<WorldState, RubberBand::Ptr> deserialized_result;

    try
    {
        if (filename.empty())
        {
            const auto log_folder = GetLogFolder(*nh_);
            const auto file_name_prefix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "world_state/file_name_prefix", __func__);
            const auto file_name_suffix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "world_state/file_name_suffix_to_load", __func__);
            const std::string file_name = file_name_prefix + "__" + file_name_suffix + ".compressed";
            filename = log_folder + file_name;
        }
        ROS_INFO_STREAM_NAMED("task_framework", "Loading world state from " << filename);

        const auto buffer = ZlibHelpers::LoadFromFileAndDecompress(filename);
        const auto deserialized_world_state = WorldState::Deserialize(buffer, 0);
        deserialized_result.first = deserialized_world_state.first;

        deserialized_result.second = std::make_shared<RubberBand>(*rubber_band_);
        deserialized_result.second->deserialize(buffer, deserialized_world_state.second);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM_NAMED("task_framework", "Failed to load stored world_state: "  <<  e.what());
    }

    return deserialized_result;
}

bool TaskFramework::useStoredWorldState() const
{
    return ROSHelpers::GetParamDebugLog<bool>(*ph_, "world_state/use_stored_world_state", false);
}
