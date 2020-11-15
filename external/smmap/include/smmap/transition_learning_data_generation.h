#pragma once

#include <random>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <std_msgs/Int32.h>
#include <arc_utilities/arc_helpers.hpp>
#include <deformable_manipulation_msgs/TransitionTest.h>
#include <smmap_utilities/visualization_tools.h>
#include "smmap/transition_estimation.h"
#include "smmap/quinlan_rubber_band.h"
#include "smmap/robot_interface.h"
#include "smmap/band_rrt.h"

namespace smmap
{
    struct TransitionSimulationRecord
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TransitionEstimation::StateTransition template_;
        ObjectPointSet template_band_surface_;

        TransitionEstimation::StateTransition tested_;
        ObjectPointSet tested_band_surface_;

        TransitionEstimation::TransitionAdaptationResult adaptation_result_;

        uint64_t serializeSelf(std::vector<uint8_t>& buffer) const;

        static uint64_t Serialize(
                const TransitionSimulationRecord& test_results,
                std::vector<uint8_t>& buffer);

        static std::pair<TransitionSimulationRecord, uint64_t> Deserialize(
                const std::vector<uint8_t>& buffer,
                const uint64_t current,
                const RubberBand& template_band);

        bool operator==(const TransitionSimulationRecord& other) const;

        std::vector<Visualizer::NamespaceId> visualize(
                const std::string& basename,
                const Visualizer::Ptr& vis) const;
    };
    typedef Eigen::aligned_allocator<TransitionSimulationRecord> TransitionSimulationRecordAlloc;
    typedef std::map<std::string, TransitionSimulationRecord, std::less<std::string>, TransitionSimulationRecordAlloc> MapStringTransitionSimulationRecord;

    class TransitionTesting
    {
    private:
        const std::shared_ptr<ros::NodeHandle> nh_;
        const std::shared_ptr<ros::NodeHandle> ph_;
        const RobotInterface::Ptr robot_;
        const Visualizer::Ptr vis_;
        const bool disable_visualizations_;
        const bool visualize_gripper_motion_;

        const unsigned long seed_;
        const std::shared_ptr<std::mt19937_64> generator_;

        const DijkstrasCoverageTask::Ptr task_;
        // Note that work_space_grid_ and the environment_sdf_ are using different
        // resolutions due to the way the SDF is created in CustomScene
        const sdf_tools::SignedDistanceField::ConstPtr sdf_;
        const XYZGrid work_space_grid_;

        std::shared_ptr<const BandRRT::WorldParams> world_params_;
        BandRRT::PlanningParams planning_params_;
        BandRRT::SmoothingParams smoothing_params_;
        BandRRT::TaskParams task_params_;
        std::shared_ptr<const BandRRT> band_rrt_vis_;

        Eigen::Isometry3d gripper_a_starting_pose_;
        Eigen::Isometry3d gripper_b_starting_pose_;
        Eigen::Vector3d gripper_a_action_vector_;
        Eigen::Vector3d gripper_b_action_vector_;
        Eigen::Isometry3d experiment_center_of_rotation_;

        const DeformableType deformable_type_;
        const TaskType task_type_;

        const WorldState initial_world_state_;
        RubberBand::Ptr initial_band_;
        TransitionEstimation::Ptr transition_estimator_;

        const std::string data_folder_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TransitionTesting(
                std::shared_ptr<ros::NodeHandle> nh,
                std::shared_ptr<ros::NodeHandle> ph,
                RobotInterface::Ptr robot,
                Visualizer::Ptr vis);

    private:
        void initialize(const WorldState& world_state);
        void initializeBand(const WorldState& world_state);
        void initializeRRTParams();
        void clampGripperDeltas(
                Eigen::Ref<Eigen::Vector3d> a_delta,
                Eigen::Ref<Eigen::Vector3d> b_delta) const;
        // Generates file name stubs: "/path/to/file/experiment" without the "__type_of_data.compressed" suffix
        void getDataFileLists();
        std::vector<std::string> last_step_data_files_;
        std::vector<std::string> path_test_data_files_;

    public:
        void runTests(const bool generate_test_data,
                      const bool generate_last_step_transition_approximations,
                      const bool generate_trajectories,
                      const bool visualize_trajectories,
                      const bool generate_meaningful_mistake_examples,
                      const bool generate_features,
                      const bool test_classifiers);
        void generateLastStepTestData();
        void generateLastStepTransitionApproximations();
        void generateLastStepTrajectories();
        void visualizeLastStepDataTrajectories();
        void visualizePathTestTrajectories();
        void generateMeaningfulMistakeExamples();

        void generateFeatures(const std::string& parabola_slice_option);
        std::vector<std::string> extractFeatures(
                const TransitionEstimation::StateTransition& transition,
                const std::string& parabola_slice_option) const;

        void testClassifier();
        AllGrippersSinglePose getGripperTargets();

        std::pair<TransitionEstimation::StateTrajectory, bool> toTrajectory(
                const deformable_manipulation_msgs::TransitionTestResult& test,
                const RRTPath& path,
                const std::string& filename);

    private:

        //// Data saving and loading ///////////////////////////////////////////

        void saveTestResult(const deformable_manipulation_msgs::TransitionTestResult& test_result, const std::string& filename) const;
        deformable_manipulation_msgs::TransitionTestResult loadTestResult(const std::string& filename) const;

        //// Data Generation ///////////////////////////////////////////////////

        RRTPath loadOrGeneratePath(const std::string& filename,
                                   const AllGrippersSinglePose& gripper_target_poses,
                                   const unsigned long long num_discards);
        RRTPath generateTestPath(const AllGrippersSinglePose& gripper_target_poses,
                                 const unsigned long long num_discards);

        //// Data Visualization ////////////////////////////////////////////////

        // Maps filenames to ns+ids
        std::map<std::string, std::vector<Visualizer::NamespaceId>> visid_to_markers_;

        int next_vis_prefix_;
        ros::Subscriber next_vis_id_sub_;
        ros::ServiceServer remove_visualization_;

        // Transition Adaptation
        bool source_valid_;
        std::string source_file_;
        TransitionEstimation::StateTransition source_transition_;
        ObjectPointSet source_band_surface_;
        int source_num_foh_changes_;

        ros::ServiceServer set_transition_adaptation_source_;
        ros::ServiceServer add_transition_adaptation_visualization_;

        // Mistake Example
        const double mistake_dist_thresh_;
        ros::ServiceServer add_mistake_example_visualization_;

        // Classification Example
        Classifier::Ptr transition_mistake_classifier_;
        ros::ServiceServer add_classification_example_visualization_;

    public:
        void setNextVisId(const std_msgs::Int32& msg);
        bool removeVisualizationCallback(
                deformable_manipulation_msgs::TransitionTestingVisualizationRequest& req,
                deformable_manipulation_msgs::TransitionTestingVisualizationResponse& res);

        std::vector<Visualizer::NamespaceId> visualizePathAndTrajectory(
                const RRTPath& path,
                const TransitionEstimation::StateTrajectory& trajectory,
                const std::string ns_prefix) const;

        bool setTransitionAdaptationSourceCallback(
                deformable_manipulation_msgs::TransitionTestingVisualizationRequest& req,
                deformable_manipulation_msgs::TransitionTestingVisualizationResponse& res);
        bool addTransitionAdaptationVisualizationCallback(
                deformable_manipulation_msgs::TransitionTestingVisualizationRequest& req,
                deformable_manipulation_msgs::TransitionTestingVisualizationResponse& res);

        bool addMistakeExampleVisualizationCallback(
                deformable_manipulation_msgs::TransitionTestingVisualizationRequest& req,
                deformable_manipulation_msgs::TransitionTestingVisualizationResponse& res);

        bool addClassificationExampleVisualizationCallback(
                deformable_manipulation_msgs::TransitionTestingVisualizationRequest& req,
                deformable_manipulation_msgs::TransitionTestingVisualizationResponse& res);
    };
}
