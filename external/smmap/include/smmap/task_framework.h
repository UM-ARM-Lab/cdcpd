#ifndef SMMAP_PLANNER_H
#define SMMAP_PLANNER_H

#include <arc_utilities/log.hpp>
#include <smmap_utilities/multiarm_bandits.h>
#include <smmap_utilities/visualization_tools.h>

#include <smmap_utilities/task_function_pointer_types.h>
#include <smmap/task_specification.h>
#include "smmap/robot_interface.h"
#include <smmap_models/deformable_model.h>
#include "smmap/deformable_controller.h"
#include "smmap/quinlan_rubber_band.h"
#include "smmap/band_rrt.h"
#include "smmap/transition_estimation.h"

namespace smmap
{
    class TaskFramework
    {
        public:
            ////////////////////////////////////////////////////////////////////
            // Constructor and the one function that gets called externally
            ////////////////////////////////////////////////////////////////////

            TaskFramework(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    RobotInterface::Ptr robot,
                    Visualizer::Ptr vis,
                    TaskSpecification::Ptr task_specification);

            void execute();

        private:
            ////////////////////////////////////////////////////////////////////
            // Multipurpose
            ////////////////////////////////////////////////////////////////////

            const std::shared_ptr<ros::NodeHandle> nh_;
            const std::shared_ptr<ros::NodeHandle> ph_;
            const unsigned long seed_;
            const std::shared_ptr<std::mt19937_64> generator_;

            const std::string test_id_;
            const RobotInterface::Ptr robot_;
            const TaskSpecification::Ptr task_specification_;
            const DijkstrasCoverageTask::Ptr dijkstras_task_;

            ////////////////////////////////////////////////////////////////////
            // Sending gripper commands
            ////////////////////////////////////////////////////////////////////

            // TODO: figure out a good way to pass by const reference (Eigen consistency)
            // TODO: HACK: Second element indicates that we should trigger a reset rather than continuing
            std::pair<WorldState, bool> sendNextCommand(
                    const WorldState& current_world_state);

            void updateBand(
                    const WorldState& world_feedback);

            WorldState sendNextCommandUsingLocalController(
                    const WorldState& current_world_state);

            WorldState sendNextCommandUsingGlobalPlannerResults(
                    const WorldState& current_world_state);

            size_t findBestBandMatchAtEndOfSegment() const;

            ////////////////////////////////////////////////////////////////////
            // Constraint violation detection
            ////////////////////////////////////////////////////////////////////

            void visualizeProjectedPaths(
                    const std::vector<EigenHelpers::VectorVector3d>& projected_paths,
                    const bool visualization_enabled);

            std::pair<std::vector<EigenHelpers::VectorVector3d>, std::vector<RubberBand>> projectFutureSystemState(
                    const WorldState& current_world_state,
                    const bool visualization_enabled);

            bool globalPlannerNeededDueToOverstretch(const WorldState& current_world_state);

            bool globalPlannerNeededDueToLackOfProgress();

            bool predictStuckForGlobalPlannerResults(const bool visualization_enabled = true);

            ////////////////////////////////////////////////////////////////////
            // Global gripper planner functions
            ////////////////////////////////////////////////////////////////////

            void initializeBand(const WorldState& world_state);

            void initializeBandRRT(const bool planning_for_whole_robot);

            AllGrippersSinglePose getGripperTargets(
                    const WorldState& world_state);

            void planGlobalGripperTrajectory(
                    const WorldState& world_state);

            BandRRT::Ptr bandRRTCopy(const unsigned long seed);

            void testPlanningPerformance(
                    const WorldState& world_state,
                    const unsigned long base_seed,
                    const RRTNode& start_config,
                    const RRTGrippersRepresentation& target_grippers_poses,
                    const bool parallel_planning);

            ////////////////////////////////////////////////////////////////////
            // Model list management
            ////////////////////////////////////////////////////////////////////

            void initializeModelAndControllerSet(const WorldState& initial_world_state);
            void addModel(DeformableModel::Ptr model);
            void createBandits();

            const bool collect_results_for_all_models_;
            ssize_t num_models_;
            std::vector<DeformableModel::Ptr> model_list_;
            std::vector<DeformableController::Ptr> controller_list_;

            const MABAlgorithm mab_algorithm_;
            std::shared_ptr<MABBase> model_utility_bandit_;
            double reward_std_dev_scale_factor_;
            const double process_noise_factor_;
            const double observation_noise_factor_;
            const double correlation_strength_factor_;

            ////////////////////////////////////////////////////////////////////
            // Model utility functions
            ////////////////////////////////////////////////////////////////////

            void updateModels(
                    const WorldState& starting_world_state,
                    const ObjectDeltaAndWeight& task_desired_motion,
                    const std::vector<DeformableController::OutputData>& suggested_commands,
                    const ssize_t model_used,
                    const WorldState& world_feedback);

            Eigen::MatrixXd calculateProcessNoise(
                    const std::vector<DeformableController::OutputData>& suggested_commands) const;

            ////////////////////////////////////////////////////////////////////
            // Constraint violation and global planner data
            ////////////////////////////////////////////////////////////////////

            // TODO: make it so that paramters that are not used are not looked up
            // if stuck detection is not enbaled
            const bool enable_stuck_detection_;
            RubberBand::Ptr rubber_band_;
            const size_t max_lookahead_steps_;
            const size_t max_grippers_pose_history_length_;
            AllGrippersPoseTrajectory grippers_pose_history_;
            std::vector<double> error_history_;

            bool plan_triggered_once_;
            bool executing_global_trajectory_;
            size_t num_times_planner_invoked_;

            std::shared_ptr<const BandRRT::WorldParams> world_params_;
            BandRRT::PlanningParams planning_params_;
            BandRRT::SmoothingParams smoothing_params_;
            BandRRT::TaskParams task_params_;
            BandRRT::Ptr band_rrt_;
            RRTPolicy rrt_planned_policy_;
            size_t policy_current_idx_;
            size_t policy_segment_next_idx_;            
            // Stores the microstep history of the deformable object trajectory
            // between waypoints
            std::vector<WorldState> microstep_history_buffer_;

            // Stores the transition estimator version of state,
            // and the trajectory of world states in microsteps
            TransitionEstimation::StateTrajectory rrt_executed_path_;
            TransitionEstimation::Ptr transition_estimator_;

            // These are both intended only for logging purposes, the individual
            // controllers may (or may not) have their own copies for their own purposes
            const Eigen::MatrixXd object_initial_node_distance_;
            // The way this is used assumes that the grippers start at a
            // "max distance but not with object stretched" distance from each other
            const double initial_grippers_distance_;

            ////////////////////////////////////////////////////////////////////
            // Logging and visualization functionality
            ////////////////////////////////////////////////////////////////////

            void visualizeDesiredMotion(
                    const WorldState& current_world_state,
                    const ObjectDeltaAndWeight& desired_motion,
                    const bool visualization_enabled = true) const;

            void visualizeGripperMotion(
                    const AllGrippersSinglePose& current_gripper_pose,
                    const AllGrippersSinglePoseDelta& gripper_motion,
                    const ssize_t model_ind) const;

            void initializeBanditsLogging();

            void initializeControllerLogging();

            void logBanditsData(
                    const WorldState& initial_world_state,
                    const WorldState& resulting_world_state,
                    const std::vector<WorldState>& individual_model_results,
                    const Eigen::VectorXd& model_utility_mean,
                    const Eigen::MatrixXd& model_utility_second_stat,
                    const ssize_t model_used);

            void controllerLogData(
                    const WorldState& initial_world_state,
                    const WorldState& resulting_world_state,
                    const std::vector<WorldState>& individual_model_results,
                    const DeformableController::InputData& controller_input_data,
                    const std::vector<double>& individual_computation_times,
                    const std::vector<double>& model_prediction_errors_weighted,
                    const std::vector<double>& model_prediction_errors_unweighted);

            static void LogPlanningPerformanceData(
                    Log::Log& log,
                    const std::vector<std::vector<BandRRT::PlanningSmoothingStatistics>>& statistics);

            void storeWorldState(const WorldState& world_state, const RubberBand::ConstPtr band, std::string filename = "");
            std::pair<WorldState, RubberBand::Ptr> loadStoredWorldState(std::string filename = "");
            bool useStoredWorldState() const;

            const bool bandits_logging_enabled_;
            const bool controller_logging_enabled_;
            std::unordered_map<std::string, Log::Log> loggers_;
            std::unordered_map<std::string, Log::Log> controller_loggers_;

            Visualizer::Ptr vis_;
            const bool visualize_desired_motion_;
            const bool visualize_gripper_motion_;
            const bool visualize_predicted_motion_;

        public:
            // Topic names used for publishing visualization data
            static constexpr auto DESIRED_DELTA_NS                         = "desired_delta";
            static constexpr auto PREDICTED_DELTA_NS                       = "predicted_delta";

            static constexpr auto PROJECTED_GRIPPER_NS                     = "projected_grippers";
            static constexpr auto PROJECTED_BAND_NS                        = "projected_band";
            static constexpr auto PROJECTED_POINT_PATH_NS                  = "projected_point_paths";
            static constexpr auto PROJECTED_POINT_PATH_LINES_NS            = "projected_point_path_lines";

            static constexpr auto CONSTRAINT_VIOLATION_VERSION1_NS         = "constraint_violation_version1";

            static constexpr auto CLUSTERING_TARGETS_NS                    = "clustering_targets";
            static constexpr auto CLUSTERING_RESULTS_PRE_PROJECT_NS        = "clustering_pre_project";
            static constexpr auto CLUSTERING_RESULTS_POST_PROJECT_NS       = "clustering_post_project";
            static constexpr auto CLUSTERING_RESULTS_ASSIGNED_CENTERS_NS   = "clustering_assigned_centers";
    };
}

#endif // SMMAP_PLANNER_H
