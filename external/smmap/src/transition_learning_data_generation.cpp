#include "smmap/transition_learning_data_generation.h"

#include <iomanip>
#include <sstream>
#include <boost/filesystem.hpp>

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/timing.hpp>
#include <arc_utilities/serialization_ros.hpp>
#include <arc_utilities/filesystem.hpp>
#include <arc_utilities/log.hpp>
#include <arc_utilities/math_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/simple_kmeans_clustering.hpp>
#include <sdf_tools/collision_map.hpp>
#include <smmap_utilities/neighbours.h>
#include <deformable_manipulation_experiment_params/conversions.hpp>
#include <deformable_manipulation_experiment_params/utility.hpp>
#include <deformable_manipulation_experiment_params/ros_params.hpp>
#include <deformable_manipulation_msgs/GenerateTransitionDataAction.h>
#include <smmap_utilities/grippers.h>

#include "smmap/band_rrt.h"
#include "smmap/parabola.h"
#include "smmap/conversions.h"

using namespace arc_utilities;
using namespace arc_helpers;
using namespace Eigen;
using namespace EigenHelpers;
using namespace EigenHelpersConversions;
namespace dmm = deformable_manipulation_msgs;
namespace fs = boost::filesystem;
using ColorBuilder = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>;

enum Features
{
    GRIPPER_A_PRE_X,
    GRIPPER_A_PRE_Y,
    GRIPPER_A_PRE_Z,
    GRIPPER_B_PRE_X,
    GRIPPER_B_PRE_Y,
    GRIPPER_B_PRE_Z,
    GRIPPER_A_POST_X,
    GRIPPER_A_POST_Y,
    GRIPPER_A_POST_Z,
    GRIPPER_B_POST_X,
    GRIPPER_B_POST_Y,
    GRIPPER_B_POST_Z,

    GRIPPER_DELTA_LENGTH_PRE,
    GRIPPER_DELTA_LENGTH_POST,
    GRIPPER_DELTA_LENGTH_RATIO_PRE,
    GRIPPER_DELTA_LENGTH_RATIO_POST,

    MAX_BAND_LENGTH,
    BAND_LENGTH_PRE,
    BAND_LENGTH_POST,
    BAND_LENGTH_RATIO_PRE,
    BAND_LENGTH_RATIO_POST,

    SLICE_NUM_CONNECTED_COMPONENTS_PRE,
    SLICE_NUM_CONNECTED_COMPONENTS_POST,
    SLICE_NUM_CONNECTED_COMPONENTS_DELTA,
    SLICE_NUM_CONNECTED_COMPONENTS_DELTA_SIGN,

    SLICE_NUM_FREE_CONNECTED_COMPONENTS_PRE,
    SLICE_NUM_FREE_CONNECTED_COMPONENTS_POST,
    SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA,
    SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA_SIGN,

    SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_PRE,
    SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_POST,
    SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA,
    SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA_SIGN,

    FEATURES_DUMMY_ITEM
};

static const std::vector<std::string> FEATURE_NAMES =
{
    std::string("GRIPPER_A_PRE_X"),
    std::string("GRIPPER_A_PRE_Y"),
    std::string("GRIPPER_A_PRE_Z"),
    std::string("GRIPPER_B_PRE_X"),
    std::string("GRIPPER_B_PRE_Y"),
    std::string("GRIPPER_B_PRE_Z"),
    std::string("GRIPPER_A_POST_X"),
    std::string("GRIPPER_A_POST_Y"),
    std::string("GRIPPER_A_POST_Z"),
    std::string("GRIPPER_B_POST_X"),
    std::string("GRIPPER_B_POST_Y"),
    std::string("GRIPPER_B_POST_Z"),

    std::string("GRIPPER_DELTA_LENGTH_PRE"),
    std::string("GRIPPER_DELTA_LENGTH_POST"),
    std::string("GRIPPER_DELTA_LENGTH_RATIO_PRE"),
    std::string("GRIPPER_DELTA_LENGTH_RATIO_POST"),

    std::string("MAX_BAND_LENGTH"),
    std::string("BAND_LENGTH_PRE"),
    std::string("BAND_LENGTH_POST"),
    std::string("BAND_LENGTH_RATIO_PRE"),
    std::string("BAND_LENGTH_RATIO_POST"),

    std::string("SLICE_NUM_CONNECTED_COMPONENTS_PRE"),
    std::string("SLICE_NUM_CONNECTED_COMPONENTS_POST"),
    std::string("SLICE_NUM_CONNECTED_COMPONENTS_DELTA"),
    std::string("SLICE_NUM_CONNECTED_COMPONENTS_DELTA_SIGN"),

    std::string("SLICE_NUM_FREE_CONNECTED_COMPONENTS_PRE"),
    std::string("SLICE_NUM_FREE_CONNECTED_COMPONENTS_POST"),
    std::string("SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA"),
    std::string("SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA_SIGN"),

    std::string("SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_PRE"),
    std::string("SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_POST"),
    std::string("SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA"),
    std::string("SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA_SIGN")
};

////////////////////////////////////////////////////////////////////////////////
//          Conversions and Random Helpers
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    static std::string ToString(const Eigen::Vector3d& mat)
    {
        std::stringstream ss;
        ss << mat.x() << "_" << mat.y() << "_" << mat.z() ;
        return ss.str();
    }

    static TransitionEstimation::StateTransition ToStateTransition(
            const dmm::TransitionTestResult& test,
            const RRTPath& path)
    {
        const auto template_band = *path.back().band();

        const WorldState start = ConvertToEigenFeedback(test.start_after_following_path);
        const std::vector<WorldState> microsteps_all = ConvertToEigenFeedback(test.microsteps_all);
        const std::vector<WorldState> microsteps_last = ConvertToEigenFeedback(test.microsteps_last_action);
        const WorldState& end = microsteps_all.back();

        const auto start_state = TransitionEstimation::State
        {
            start.object_configuration_,
            RubberBand::BandFromWorldState(start, template_band),
            std::make_shared<RubberBand>(*path.back().band()),
            start.rope_node_transforms_
        };

        // Propagate the planned band the last step
        auto band = std::make_shared<RubberBand>(*path.back().band());
        band->forwardPropagate(ToGripperPositions(end.all_grippers_single_pose_), false);
        const auto end_state = TransitionEstimation::State
        {
            end.object_configuration_,
            RubberBand::BandFromWorldState(end, template_band),
            band,
            end.rope_node_transforms_
        };

        std::vector<RubberBand::Ptr> microsteps_last_bands;
        microsteps_last_bands.reserve(microsteps_last.size());
        for (size_t idx = 0; idx < microsteps_last.size(); ++idx)
        {
            microsteps_last_bands.push_back(std::make_shared<RubberBand>(template_band));
            if (!microsteps_last_bands.back()->resetBand(microsteps_last[idx]))
            {
                throw_arc_exception(std::runtime_error, "Unable to extract surface");
            }
        }

        return TransitionEstimation::StateTransition
        {
            start_state,
            end_state,
            start_state.planned_rubber_band_->getEndpoints(),
            end_state.planned_rubber_band_->getEndpoints(),
            microsteps_last,
            microsteps_last_bands
        };
    }

    // Returns the trajectory, and a flag indicating if everything was parsed cleanly
    // I.e.; {traj, true} means "no problem"
    std::pair<TransitionEstimation::StateTrajectory, bool> TransitionTesting::toTrajectory(
            const dmm::TransitionTestResult& test_result,
            const RRTPath& path,
            const std::string& filename)
    {
        const bool has_last_action = test_result.microsteps_last_action.size() > 0;

        // Designed to address different ways of generating data (with or without a last action)
        const AllGrippersSinglePose grippers_ending_poses = has_last_action
                ? VectorGeometryPoseToVectorIsometry3d(test_result.microsteps_last_action.back().gripper_poses)
                : ToGripperPoseVector(path.back().grippers());

        // Determine what to expect, under the assumption that the path was completely executed in the simulator
        const auto test = robot_->toRosTransitionTest(
                    initial_world_state_.rope_node_transforms_, // Doesn't matter what this is because we do not read anything calculated from this
                    ToGripperPoseVector(path.front().grippers()),
                    RRTPathToGrippersPoseTrajectory(path),
                    grippers_ending_poses);

        const auto traj = ToTrajectory(initial_world_state_, path, test, test_result);
        if (!traj.second)
        {
            ROS_WARN_STREAM_NAMED("to_traj", "Short path returned for " << filename);
        }
        return traj;
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Transition Simulation Record
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    uint64_t TransitionSimulationRecord::serializeSelf(std::vector<uint8_t>& buffer) const
    {
        const auto starting_bytes = buffer.size();
        uint64_t bytes_written = 0;
        bytes_written += template_.serialize(buffer);
        bytes_written += arc_utilities::SerializeEigen(template_band_surface_, buffer);
        bytes_written += tested_.serialize(buffer);
        bytes_written += arc_utilities::SerializeEigen(tested_band_surface_, buffer);
        bytes_written += adaptation_result_.serialize(buffer);

        const auto ending_bytes = buffer.size();
        assert(ending_bytes - starting_bytes == bytes_written);
        const auto deserialized = Deserialize(buffer, starting_bytes, *template_.starting_state_.rubber_band_);
        assert(bytes_written = deserialized.second);
        assert(*this == deserialized.first);
        return bytes_written;;
    }

    uint64_t TransitionSimulationRecord::Serialize(
            const TransitionSimulationRecord& test_results,
            std::vector<uint8_t>& buffer)
    {
        return test_results.serializeSelf(buffer);
    }

    std::pair<TransitionSimulationRecord, uint64_t> TransitionSimulationRecord::Deserialize(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const RubberBand& template_band)
    {
        uint64_t bytes_read = 0;

        const auto template_deserialized =
                TransitionEstimation::StateTransition::Deserialize(buffer, current + bytes_read, template_band);
        bytes_read += template_deserialized.second;

        const auto template_band_surface_deserialized =
                arc_utilities::DeserializeEigen<ObjectPointSet>(buffer, current + bytes_read);
        bytes_read += template_band_surface_deserialized.second;

        const auto tested_deserialized =
                TransitionEstimation::StateTransition::Deserialize(buffer, current + bytes_read, template_band);
        bytes_read += tested_deserialized.second;

        const auto tested_band_surface_deserialized =
                arc_utilities::DeserializeEigen<ObjectPointSet>(buffer, current + bytes_read);
        bytes_read += tested_band_surface_deserialized.second;

        const auto adaptation_result_deserialized =
                TransitionEstimation::TransitionAdaptationResult::Deserialize(buffer, current + bytes_read, template_band);
        bytes_read += adaptation_result_deserialized.second;

        TransitionSimulationRecord record =
        {
            template_deserialized.first,
            template_band_surface_deserialized.first,
            tested_deserialized.first,
            tested_band_surface_deserialized.first,
            adaptation_result_deserialized.first
        };
        return {record, bytes_read};
    }

    bool TransitionSimulationRecord::operator==(const TransitionSimulationRecord& other) const
    {
        if (template_ != template_)
        {
            return false;
        }
        if (template_band_surface_ != other.template_band_surface_)
        {
            return false;
        }
        if (tested_ != other.tested_)
        {
            return false;
        }
        if (tested_band_surface_ != other.tested_band_surface_)
        {
            return false;
        }
        if (adaptation_result_ != other.adaptation_result_)
        {
            return false;
        }
        return true;
    }

    std::vector<Visualizer::NamespaceId> TransitionSimulationRecord::visualize(
            const std::string& basename,
            const Visualizer::Ptr& vis) const
    {
        std::vector<Visualizer::NamespaceId> marker_ids;

        constexpr bool vis_template_starting_band = true;
        constexpr bool vis_template_ending_executed_band = true;
        constexpr bool vis_template_executed_band_surface = true;
        constexpr bool vis_test_start_planned_band = true;
        constexpr bool vis_test_executed_band = true;
        constexpr bool vis_test_executed_band_surface = true;
        constexpr bool vis_adapt_default_next_band = true;
        constexpr bool vis_adapt_target_band_and_action = true;
        constexpr bool vis_adapt_template_band_and_action = true;
        constexpr bool vis_adapt_template_aligned = true;
        constexpr bool vis_adapt_next_band_points_to_smooth = true;
        constexpr bool vis_adapt_transformed_band_surface_points = true;
        constexpr bool vis_adapt_retightend_band_surface = true;
        constexpr bool vis_adapt_final_result = true;

        // Template - starting planned band
        if (vis_template_starting_band)
        {
            const auto color = Visualizer::Green();
            const auto name = basename + "template__start";
            const auto new_ids = template_.starting_state_.planned_rubber_band_->visualize(name, color, color, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Template - ending executed band
        if (vis_template_ending_executed_band)
        {
            const auto color = Visualizer::Cyan();
            const auto name = basename + "template__executed";
            const auto new_ids = template_.ending_state_.rubber_band_->visualize(name, color, color, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Template - Executed band surface
        if (vis_template_executed_band_surface)
        {
            const auto start_color = Visualizer::Green();
            const auto end_color = Visualizer::Cyan();
            const auto name = basename + "template__band_surface";
            const auto new_ids = RubberBand::VisualizeBandSurface(vis, template_band_surface_, template_.microstep_band_history_.size(), start_color, end_color, name, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Test - start planned band
        if (vis_test_start_planned_band)
        {
            const auto color = Visualizer::Yellow();
            const auto name = basename + "tested__start";
            const auto new_ids = tested_.starting_state_.planned_rubber_band_->visualize(name, color, color, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Test - exectued band
        if (vis_test_executed_band)
        {
            const auto color = Visualizer::Orange();
            const auto name = basename + "tested__executed";
            const auto new_ids = tested_.ending_state_.rubber_band_->visualize(name, color, color, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Test - Executed band surface
        if (vis_test_executed_band_surface)
        {
            const auto start_color = Visualizer::Yellow();
            const auto end_color = Visualizer::Orange();
            const auto name = basename + "tested__band_surface";
            const auto new_ids = RubberBand::VisualizeBandSurface(vis, tested_band_surface_, tested_.microstep_band_history_.size(), start_color, end_color, name, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Adaptation process - default next band
        if (vis_adapt_default_next_band)
        {
            const auto color = Visualizer::Red();
            const auto name = basename + "adaptation__default_next_band";
            const auto new_ids = adaptation_result_.default_next_band_->visualize(name, color, color, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Adaptation process - target band and action
        if (vis_adapt_target_band_and_action)
        {
            const auto color = Visualizer::Yellow();
            const auto name = basename + "adaptation__target_points_to_match";
            std::vector<std_msgs::ColorRGBA> colors;
            const auto num_divs = (adaptation_result_.target_points_to_match_.cols() - 1);
            for (ssize_t idx = 0; idx <= num_divs; ++idx)
            {
                colors.push_back(InterpolateColor(color, Visualizer::Red(), (float)idx / (float)num_divs));
            }
            const auto new_ids = vis->visualizePoints(name, adaptation_result_.target_points_to_match_, colors, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Adaptation process - template band and action
        if (vis_adapt_template_band_and_action)
        {
            const auto color = Visualizer::Green();
            const auto name = basename + "adaptation__template_points_to_align";
            std::vector<std_msgs::ColorRGBA> colors;
            const auto num_divs = adaptation_result_.template_points_to_align_.cols() - 1;
            for (ssize_t idx = 0; idx <= num_divs; ++idx)
            {
                colors.push_back(InterpolateColor(color, Visualizer::Red(), (float)idx / (float)num_divs));
            }
            const auto new_ids = vis->visualizePoints(name, adaptation_result_.template_points_to_align_, colors, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Adaptation process - template aligned
        if (vis_adapt_template_aligned)
        {
            const auto color = Visualizer::Magenta();
            const auto name = basename + "adaptation__template_aligned_to_target";
            std::vector<std_msgs::ColorRGBA> colors;
            const auto num_divs = adaptation_result_.template_planned_band_aligned_to_target_.cols() - 1;
            for (ssize_t idx = 0; idx <= num_divs; ++idx)
            {
                colors.push_back(InterpolateColor(color, Visualizer::Red(), (float)idx / (float)num_divs));
            }
            const auto new_ids = vis->visualizePoints(name, adaptation_result_.template_planned_band_aligned_to_target_, colors, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Adaptation process - next_band_points_to_smooth_
        if (vis_adapt_next_band_points_to_smooth)
        {
            const auto color = Visualizer::Seafoam();
            const auto name = basename + "adaptation__next_band_points_to_smooth";
            const auto new_ids = vis->visualizePoints(name, adaptation_result_.next_band_points_to_smooth_, color, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Adaptation process - transformed_band_surface_points
        if (vis_adapt_transformed_band_surface_points)
        {
            const auto start_color = Visualizer::Blue();
            const auto end_color = Visualizer::Seafoam();
            const auto name = basename + "adaptation__transformed_band_surface_points";
            const auto new_ids = RubberBand::VisualizeBandSurface(vis, adaptation_result_.transformed_band_surface_points_, template_.microstep_band_history_.size(), start_color, end_color, name, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Adaptation process - re-tightened band surface
        if (vis_adapt_retightend_band_surface)
        {
            const auto start_color = Visualizer::Olive();
            const auto end_color = Visualizer::Coral();
            const auto name = basename + "adaptation__tightened_transformed_bands_surface";
            const auto new_ids = RubberBand::VisualizeBandSurface(vis, adaptation_result_.tightened_transformed_bands_surface_, template_.microstep_band_history_.size(), start_color, end_color, name, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }
        // Adaptation process - final result
        if (vis_adapt_final_result)
        {
            const auto color = Visualizer::Coral();
            const auto name = basename + "adaptation__result";
            const auto new_ids = adaptation_result_.result_->visualize(name, color, color, 1);
            marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
        }

        return marker_ids;
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Transition Testing - Initialization and main function
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    TransitionTesting::TransitionTesting(
            std::shared_ptr<ros::NodeHandle> nh,
            std::shared_ptr<ros::NodeHandle> ph,
            RobotInterface::Ptr robot,
            Visualizer::Ptr vis)
        : nh_(nh)
        , ph_(ph)
        , robot_(robot)
        , vis_(vis)
        , disable_visualizations_(!vis_->visualizationsEnabled())
        , visualize_gripper_motion_(!disable_visualizations_ && GetVisualizeGripperMotion(*ph_))

        , seed_(GetPlannerSeed(*ph_))
        , generator_(std::make_shared<std::mt19937_64>(seed_))

        , task_(std::dynamic_pointer_cast<DijkstrasCoverageTask>(TaskSpecification::MakeTaskSpecification(nh_, ph_, vis_)))
        , sdf_(GetEnvironmentSDF(*nh_))
        , work_space_grid_(sdf_->GetOriginTransform(),
                           sdf_->GetFrame(),
                           GetWorldXStep(*nh_),
                           GetWorldYStep(*nh_),
                           GetWorldZStep(*nh_),
                           GetWorldXNumSteps(*nh_),
                           GetWorldYNumSteps(*nh_),
                           GetWorldZNumSteps(*nh_))

        , deformable_type_(GetDeformableType(*nh_))
        , task_type_(GetTaskType(*nh_))
        , initial_world_state_(robot_->start())

        , data_folder_(ROSHelpers::GetParam<std::string>(*ph_, "data_folder", "/tmp/transition_learning_data_generation"))

        , next_vis_prefix_(0)
        , next_vis_id_sub_(nh_->subscribe("transition_vis/set_next_vis_id", 1, &TransitionTesting::setNextVisId, this))
        , remove_visualization_(nh_->advertiseService("transition_vis/remove_visualization", &TransitionTesting::removeVisualizationCallback, this))

        , source_valid_(false)
        , set_transition_adaptation_source_(nh_->advertiseService("transition_vis/set_transition_adaptation_source", &TransitionTesting::setTransitionAdaptationSourceCallback, this))
        , add_transition_adaptation_visualization_(nh_->advertiseService("transition_vis/add_transition_adaptation_visualization", &TransitionTesting::addTransitionAdaptationVisualizationCallback, this))

        , mistake_dist_thresh_(GetTransitionMistakeThreshold(*ph_))
        , add_mistake_example_visualization_(nh_->advertiseService("transition_vis/add_mistake_example_visualization", &TransitionTesting::addMistakeExampleVisualizationCallback, this))

        , transition_mistake_classifier_(Classifier::MakeClassifier(nh_, ph_))
        , add_classification_example_visualization_(nh_->advertiseService("transition_vis/add_classification_example_visualization", &TransitionTesting::addClassificationExampleVisualizationCallback, this))
    {
        assert(task_ != nullptr && "This class is only intended for DijkstrasCoverageTask based tasks");
        std::srand((unsigned int)seed_);
        initialize(initial_world_state_);

        // Used for generating data
        gripper_a_starting_pose_.linear() = initial_world_state_.all_grippers_single_pose_[0].linear();
        gripper_b_starting_pose_.linear() = initial_world_state_.all_grippers_single_pose_[1].linear();
        clampGripperDeltas(gripper_a_action_vector_, gripper_b_action_vector_);

        if (visualize_gripper_motion_)
        {
            vis_->visualizeAxes("center_of_rotation",   experiment_center_of_rotation_, 0.1, 0.005, 1);
            vis_->visualizeAxes("gripper_a_start",      gripper_a_starting_pose_,       0.1, 0.005, 1);
            vis_->visualizeAxes("gripper_b_start",      gripper_b_starting_pose_,       0.1, 0.005, 1);
            vis_->visualizeAxes("gripper_a_end",        Translation3d(gripper_a_action_vector_) * gripper_a_starting_pose_, 0.1, 0.005, 1);
            vis_->visualizeAxes("gripper_b_end",        Translation3d(gripper_b_action_vector_) * gripper_b_starting_pose_, 0.1, 0.005, 1);
        }
    }

    void TransitionTesting::initialize(const WorldState& world_state)
    {
        initializeBand(world_state);
        transition_estimator_ = std::make_shared<TransitionEstimation>(
                    nh_, ph_, generator_, sdf_, work_space_grid_, vis_, *initial_band_);
        initializeRRTParams();
    }

    void TransitionTesting::initializeBand(const WorldState& world_state)
    {
        // Extract the maximum distance between the grippers
        // This assumes that the starting position of the grippers is at the maximum "unstretched" distance
        const auto& grippers_starting_poses = world_state.all_grippers_single_pose_;
        const double max_calced_band_length =
                (grippers_starting_poses[0].translation() - grippers_starting_poses[1].translation()).norm()
                * GetMaxStretchFactor(*ph_);
        const auto max_band_length = GetMaxBandLength(*ph_);
        ROS_ERROR_STREAM_COND_NAMED(!CloseEnough(max_calced_band_length, max_band_length, 1e-3),
                                    "data_generation",
                                    "Calc'd max band distance is: " << max_calced_band_length <<
                                    " but the ros param saved distance is " << max_band_length <<
                                    ". Double check the stored value in the roslaunch file.");

        // Find the shortest path through the object, between the grippers, while following nodes of the object.
        // Used to determine the starting position of the rubber band at each timestep
        const auto num_nodes = world_state.object_configuration_.cols();
        std::function<std::vector<ssize_t>(const ssize_t node)> neighbour_fn;
        switch (deformable_type_)
        {
            case ROPE:
            {
                LineNeighbours neighbours_calc(num_nodes);
                neighbour_fn = [neighbours_calc] (const ssize_t node)
                {
                    return neighbours_calc.getNodeNeighbours(node);
                };
                break;
            }
            case CLOTH:
            {
                Grid4Neighbours neighbours_calc(num_nodes, GetClothNumControlPointsX(*nh_));
                neighbour_fn = [neighbours_calc] (const ssize_t node)
                {
                    return neighbours_calc.getNodeNeighbours(node);
                };
                break;
            }
            default:
                throw_arc_exception(std::invalid_argument, "Invalid deformable type; this should not be possible");
        }

        // Create the initial rubber band
        const double resampled_band_max_pointwise_dist = work_space_grid_.minStepDimension() / 2.0;
        const size_t upsampled_band_num_points = GetRRTBandMaxPoints(*ph_);

        initial_band_ = std::make_shared<RubberBand>(
                    nh_,
                    ph_,
                    vis_,
                    sdf_,
                    work_space_grid_,
                    neighbour_fn,
                    world_state,
                    resampled_band_max_pointwise_dist,
                    upsampled_band_num_points,
                    max_band_length);
    }

    void TransitionTesting::initializeRRTParams()
    {
        assert(initial_band_ != nullptr);

        // "World" params used by planning
        world_params_ = std::make_shared<const BandRRT::WorldParams>(BandRRT::WorldParams
        {
            robot_,
            false,
            sdf_,
            work_space_grid_,
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
        const auto max_gripper_step_size                = work_space_grid_.minStepDimension();
        const auto max_robot_step_size                  = GetRRTMaxRobotDOFStepSize(*ph_);
        const auto min_robot_step_size                  = GetRRTMinRobotDOFStepSize(*ph_);
        const auto max_gripper_rotation                 = GetRRTMaxGripperRotation(*ph_); // only matters for real robot
        const auto goal_reached_radius                  = work_space_grid_.minStepDimension();
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

        band_rrt_vis_ = std::make_shared<const BandRRT>(nh_,
                                                        ph_,
                                                        *world_params_,
                                                        planning_params_,
                                                        smoothing_params_,
                                                        task_params_,
                                                        initial_band_,
                                                        vis_,
                                                        false);
    }

    void TransitionTesting::clampGripperDeltas(Ref<Vector3d> a_delta, Ref<Vector3d> b_delta) const
    {
        const double distance = std::sqrt(a_delta.squaredNorm() + b_delta.squaredNorm());
        if (distance > task_params_.max_gripper_step_size_)
        {
            a_delta *= (task_params_.max_gripper_step_size_ / distance);
            b_delta *= (task_params_.max_gripper_step_size_ / distance);
        }
    }

    void TransitionTesting::getDataFileLists()
    {
        ROS_INFO_STREAM("Finding data files in folder: " << data_folder_);

        last_step_data_files_.clear();
        path_test_data_files_.clear();
        try
        {
            const fs::path p(data_folder_);
            const fs::recursive_directory_iterator start(p);
            const fs::recursive_directory_iterator end;
            for (auto itr = start; itr != end; ++itr)
            {
                if (fs::is_regular_file(itr->status()))
                {
                    const auto filename = itr->path().string();
                    // Only warn about file types that are not expected
                    if (filename.find("compressed") == std::string::npos &&
                        filename.find("failed") == std::string::npos &&
                        filename.find("classification_features") == std::string::npos &&
                        filename.find("npz") == std::string::npos &&
                        filename.find("csv") == std::string::npos)
                    {
                        ROS_WARN_STREAM("Ignoring file: " << filename);
                    }

                    const auto last_step_pos = filename.find("__last_step_test_results.compressed");
                    if (last_step_pos != std::string::npos)
                    {
                        // Strip off the extra string for simpler use later
                        last_step_data_files_.push_back(filename.substr(0, last_step_pos));
                    }

                    const auto path_test_pos = filename.find("__path_test_results.compressed");
                    if (path_test_pos != std::string::npos)
                    {
                        // Strip off the extra string for simpler use later
                        path_test_data_files_.push_back(filename.substr(0, path_test_pos));
                    }
                }
            }
            std::sort(last_step_data_files_.begin(), last_step_data_files_.end());
            std::sort(path_test_data_files_.begin(), path_test_data_files_.end());
            ROS_INFO_STREAM("Found " << last_step_data_files_.size() << " possible last step data files in " << data_folder_);
            ROS_INFO_STREAM("Found " << path_test_data_files_.size() << " possible path test data files in " << data_folder_);
        }
        catch (const fs::filesystem_error& ex)
        {
            ROS_WARN_STREAM("Error loading file list: " << ex.what());
        }
    }

    void TransitionTesting::runTests(const bool generate_test_data,
                                     const bool generate_last_step_transition_approximations,
                                     const bool generate_trajectories,
                                     const bool visualize_trajectories,
                                     const bool generate_meaningful_mistake_examples,
                                     const bool generate_features,
                                     const bool test_classifiers)
    {
        if (generate_test_data)
        {
            Stopwatch stopwatch;
            ROS_INFO("Generating test data via Bullet");
            generateLastStepTestData();
            ROS_INFO_STREAM("Data generation time taken: " << stopwatch(READ));
        }

        getDataFileLists();

        if (generate_last_step_transition_approximations)
        {
            ROS_INFO("Generating last step transition approximations");
            Stopwatch stopwatch;
            generateLastStepTransitionApproximations();
            ROS_INFO_STREAM("Last step transition approximations time taken: " << stopwatch(READ));
        }

        if (generate_trajectories)
        {
            ROS_INFO("Generating trajectories");
            Stopwatch stopwatch;
            generateLastStepTrajectories();
            ROS_INFO_STREAM("Generate trajectories time taken: " << stopwatch(READ));
        }

        if (visualize_trajectories)
        {
            if (vis_->visualizationsEnabled())
            {
                visualizeLastStepDataTrajectories();
                visualizePathTestTrajectories();
            }
            else
            {
                ROS_ERROR("Asked to visualize trajectories, but visualization is disabled");
            }
        }

        if (generate_meaningful_mistake_examples)
        {
            ROS_INFO("Generating meaningful mistake examples");
            Stopwatch stopwatch;
            generateMeaningfulMistakeExamples();
            ROS_INFO_STREAM("Finding meaningful mistake examples time taken: " << stopwatch(READ));
        }

        if (generate_features)
        {
            const std::vector<std::string> options =
            {
                "basic",
//                "in_plane_gravity_aligned",
//                "in_plane_gripper_aligned",
//                "extend_downwards_gravity_aligned",
//                "extend_downwards_gripper_aligned"
            };
            for (const auto& opt : options)
            {
                Stopwatch stopwatch;
                ROS_INFO_STREAM("Generating transition features for option " << opt);
                generateFeatures(opt);
                ROS_INFO_STREAM("Generate features time taken: " << stopwatch(READ));
            }
        }

        if (test_classifiers)
        {
            Stopwatch stopwatch;
            ROS_INFO("Testing classifier");
            testClassifier();
            ROS_INFO_STREAM("Classifier testing time taken: " << stopwatch(READ));
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Data Saving/Loading
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    void TransitionTesting::saveTestResult(const dmm::TransitionTestResult& test_result, const std::string& filename) const
    {
        std::vector<uint8_t> buffer;
        arc_utilities::RosMessageSerializationWrapper(test_result, buffer);
        ZlibHelpers::CompressAndWriteToFile(buffer, filename);
    }

    dmm::TransitionTestResult TransitionTesting::loadTestResult(const std::string& filename) const
    {
        const auto buffer = ZlibHelpers::LoadFromFileAndDecompress(filename);
        return arc_utilities::RosMessageDeserializationWrapper<dmm::GenerateTransitionDataFeedback>(buffer, 0).first.test_result;
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Data Generation
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    // Note: this includes "no perturbation" so that it can be combined "cartesian-product" style
    static VectorVector3d Vec3dPerturbations(const double max_magnitude, const int num_divisions)
    {
        VectorVector3d perturbations;
        perturbations.reserve((size_t)(std::pow(2 * num_divisions + 1, 3)));
        for (int x_idx = -num_divisions; x_idx <= num_divisions; ++x_idx)
        {
            const double x_delta = max_magnitude * x_idx / num_divisions;
            for (int y_idx = -num_divisions; y_idx <= num_divisions; ++y_idx)
            {
                const double y_delta = max_magnitude * y_idx / num_divisions;
                for (int z_idx = -num_divisions; z_idx <= num_divisions; ++z_idx)
                {
                    const double z_delta = max_magnitude * z_idx / num_divisions;
                    perturbations.push_back(Vector3d(x_delta, y_delta, z_delta));
                }
            }
        }
        return perturbations;
    }

    void TransitionTesting::generateLastStepTestData()
    {
        gripper_a_starting_pose_ = GetPoseFromParamServer(*ph_, "gripper_a_test_start", true);
        gripper_b_starting_pose_ = GetPoseFromParamServer(*ph_, "gripper_b_test_start", true);
        gripper_a_action_vector_ = GetVector3FromParamServer(*ph_, "gripper_a_action_vector");
        gripper_b_action_vector_ = GetVector3FromParamServer(*ph_, "gripper_b_action_vector");
        experiment_center_of_rotation_ = Isometry3d(Translation3d(GetVector3FromParamServer(*ph_, "experiment_cor")));

        const auto num_threads = GetNumOMPThreads();
        std::vector<dmm::TransitionTest> tests;
        std::vector<std::string> filenames;
        tests.reserve(num_threads);
        filenames.reserve(num_threads);

        //// Generate the canonical example ////////////////////////////////////
        {
            const std::string folder(data_folder_);
            const std::string test_id("/unmodified");
            const std::string test_results_filename = folder + test_id + "__last_step_test_results.compressed";
            const std::string path_to_start_filename = folder + test_id + "__path_to_start.compressed";
            arc_utilities::CreateDirectory(fs::path(test_results_filename).parent_path());

            if (!fs::is_regular_file(test_results_filename))
            {
                const auto trial_idx = 0;

                Isometry3d gripper_a_ending_pose_ = Translation3d(gripper_a_action_vector_) * gripper_a_starting_pose_;
                Isometry3d gripper_b_ending_pose_ = Translation3d(gripper_b_action_vector_) * gripper_b_starting_pose_;

                // Generate a path and convert the test to a ROS format (if needed)
                const RRTPath path_to_start_of_test = loadOrGeneratePath(
                            path_to_start_filename,
                            {gripper_a_starting_pose_, gripper_b_starting_pose_},
                            trial_idx);

                const auto canonical_test = robot_->toRosTransitionTest(
                            initial_world_state_.rope_node_transforms_,
                            initial_world_state_.all_grippers_single_pose_,
                            RRTPathToGrippersPoseTrajectory(path_to_start_of_test),
                            {gripper_a_ending_pose_, gripper_b_ending_pose_});

                // Add the test to the list waiting to be executed
                tests.push_back(canonical_test);
                filenames.push_back(test_results_filename);
            }
        }

        //// Generate versions with perturbed gripper start positions //////////
        {
            const auto max_magnitude = ROSHelpers::GetParamRequired<double>(*ph_, "perturbations/gripper_positions/max_magnitude", __func__);
            const auto num_divisions = ROSHelpers::GetParamRequired<int>(*ph_, "perturbations/gripper_positions/num_divisions", __func__);
            const auto perturbations = Vec3dPerturbations(max_magnitude, num_divisions);
            std::cerr << "Num position perturbations: " << perturbations.size() * perturbations.size()<< std::endl;
            #pragma omp parallel for
            for (size_t a_idx = 0; a_idx < perturbations.size(); ++a_idx)
            {
                const Isometry3d gripper_a_starting_pose = Translation3d(perturbations[a_idx]) * gripper_a_starting_pose_;
                const Isometry3d gripper_a_ending_pose = Translation3d(gripper_a_action_vector_) * gripper_a_starting_pose;

                const std::string folder(data_folder_ +
                                         "/perturbed_gripper_start_positions"
                                         "/gripper_a_" + ToString(perturbations[a_idx]));
                arc_utilities::CreateDirectory(folder);

                for (size_t b_idx = 0; b_idx < perturbations.size(); ++b_idx)
                {
                    const auto trial_idx = a_idx * perturbations.size() + b_idx + 1;

                    const Isometry3d gripper_b_starting_pose = Translation3d(perturbations[b_idx]) * gripper_b_starting_pose_;
                    const Isometry3d gripper_b_ending_pose = Translation3d(gripper_b_action_vector_) * gripper_b_starting_pose;

                    const std::string test_id("/gripper_b_" + ToString(perturbations[b_idx]));
                    const std::string test_results_filename = folder + test_id + "__last_step_test_results.compressed";
                    const std::string path_to_start_filename = folder + test_id + "__path_to_start.compressed";
                    const std::string failure_file = folder + test_id + "__path_to_start.failure";

                    // Check for the file flag that indicates that this test is not possible
                    if (fs::is_regular_file(failure_file))
                    {
                        continue;
                    }

                    try
                    {
                        if (!fs::is_regular_file(test_results_filename))
                        {
                            // Generate a path and convert the test to a ROS format (if needed)
                            const RRTPath path_to_start_of_test = loadOrGeneratePath(
                                        path_to_start_filename,
                                        {gripper_a_starting_pose, gripper_b_starting_pose},
                                        trial_idx * 0xFFFF);

                            const auto test = robot_->toRosTransitionTest(
                                        initial_world_state_.rope_node_transforms_,
                                        initial_world_state_.all_grippers_single_pose_,
                                        RRTPathToGrippersPoseTrajectory(path_to_start_of_test),
                                        {gripper_a_ending_pose, gripper_b_ending_pose});

                            #pragma omp critical
                            {
                                // Add the test to the list waiting to be executed
                                tests.push_back(test);
                                filenames.push_back(test_results_filename);

                                // Execute the tests if there are enough to run
                                if (tests.size() > 100)
                                {
                                    // Ignore the feedback as the action sever saves the results to file anyway
                                    robot_->generateTransitionData(tests, filenames, nullptr, false);
                                    tests.clear();
                                    filenames.clear();
                                }
                            }
                        }
                    }
                    catch (const std::runtime_error& ex)
                    {
                        Log::Log failure_logger(failure_file, true);
                        ARC_LOG_STREAM(failure_logger, "Unable to plan with perturbation"
                                   << " a: " << perturbations[a_idx].transpose()
                                   << " b: " << perturbations[b_idx].transpose()
                                   << " Message: " << ex.what());
                        ROS_ERROR_STREAM_NAMED("data_generation", "Unable to plan with perturbation"
                                               << " a: " << perturbations[a_idx].transpose()
                                               << " b: " << perturbations[b_idx].transpose()
                                               << " Message: " << ex.what());
                    }
                }
            }
        }

        //// Generate versions with perturbed action vectors ///////////////////
        {
            const auto max_magnitude = ROSHelpers::GetParamRequired<double>(*ph_, "perturbations/action_vectors/max_magnitude", __func__);
            const auto num_divisions = ROSHelpers::GetParamRequired<int>(*ph_, "perturbations/action_vectors/num_divisions", __func__);
            const auto perturbations = Vec3dPerturbations(max_magnitude, num_divisions);
            std::cerr << "Num action perturbations: " << perturbations.size() * perturbations.size()<< std::endl;
            #pragma omp parallel for
            for (size_t a_idx = 0; a_idx < perturbations.size(); ++a_idx)
            {
                const std::string folder(data_folder_ +
                                         "/perturbed_gripper_action_vectors"
                                         "/gripper_a_" + ToString(perturbations[a_idx]));
                arc_utilities::CreateDirectory(folder);

                const Vector3d gripper_a_action_vector = gripper_a_action_vector_ + perturbations[a_idx];
                for (size_t b_idx = 0; b_idx < perturbations.size(); ++b_idx)
                {
                    const auto trial_idx =
                            perturbations.size() * perturbations.size() +
                            a_idx * perturbations.size() + b_idx + 1;

                    const Vector3d gripper_b_action_vector = gripper_b_action_vector_ + perturbations[b_idx];
                    Vector3d gripper_a_action_vector_normalized = gripper_a_action_vector;
                    Vector3d gripper_b_action_vector_normalized = gripper_b_action_vector;
                    clampGripperDeltas(gripper_a_action_vector_normalized, gripper_b_action_vector_normalized);

                    const Isometry3d gripper_a_ending_pose = Translation3d(gripper_a_action_vector_normalized) * gripper_a_starting_pose_;
                    const Isometry3d gripper_b_ending_pose = Translation3d(gripper_b_action_vector_normalized) * gripper_b_starting_pose_;

                    const std::string test_id("/gripper_b_" + ToString(perturbations[b_idx]));
                    const std::string test_results_filename = folder + test_id + "__last_step_test_results.compressed";
                    const std::string path_to_start_filename = folder + test_id + "__path_to_start.compressed";
                    const std::string failure_file = folder + test_id + "__path_to_start.failure";

                    // Check for the file flag that indicates that this test is not possible
                    if (fs::is_regular_file(failure_file))
                    {
                        continue;
                    }

                    try
                    {
                        if (!fs::is_regular_file(test_results_filename))
                        {
                            // Generate a path and convert the test to a ROS format (if needed)
                            const RRTPath path_to_start_of_test = loadOrGeneratePath(
                                        path_to_start_filename,
                                        {gripper_a_starting_pose_, gripper_b_starting_pose_},
                                        trial_idx * 0xFFFF);

                            const auto test = robot_->toRosTransitionTest(
                                        initial_world_state_.rope_node_transforms_,
                                        initial_world_state_.all_grippers_single_pose_,
                                        RRTPathToGrippersPoseTrajectory(path_to_start_of_test),
                                        {gripper_a_ending_pose, gripper_b_ending_pose});

                            #pragma omp critical
                            {
                                // Add the test to the list waiting to be executed
                                tests.push_back(test);
                                filenames.push_back(test_results_filename);

                                // Execute the tests if tehre are enough to run
                                if (tests.size() > 100)
                                {
                                    // Ignore the feedback as the action sever saves the results to file anyway
                                    robot_->generateTransitionData(tests, filenames, nullptr, false);
                                    tests.clear();
                                    filenames.clear();
                                }
                            }

                        }
                    }
                    catch (const std::runtime_error& ex)
                    {
                        Log::Log failure_logger(failure_file, true);
                        ARC_LOG_STREAM(failure_logger, "Unable to plan with perturbation"
                                   << " a: " << perturbations[a_idx].transpose()
                                   << " b: " << perturbations[b_idx].transpose()
                                   << " Message: " << ex.what());
                        ROS_ERROR_STREAM_NAMED("data_generation", "Unable to plan with perturbation"
                                               << " a: " << perturbations[a_idx].transpose()
                                               << " b: " << perturbations[b_idx].transpose()
                                               << " Message: " << ex.what());
                    }
                }
            }
        }

        // Run an tests left over
        if (tests.size() != 0)
        {
            // Ignore the feedback as the action sever saves the results to file anyway
            robot_->generateTransitionData(tests, filenames, nullptr, false);
            tests.clear();
            filenames.clear();
        }
    }

    RRTPath TransitionTesting::loadOrGeneratePath(
            const std::string& filename,
            const AllGrippersSinglePose& gripper_target_poses,
            const unsigned long long num_discards)
    {
        if (fs::is_regular_file(filename))
        {
            return BandRRT::LoadPath(filename, *initial_band_);
        }
        else
        {
            const auto path = generateTestPath(gripper_target_poses, num_discards);
            BandRRT::SavePath(path, filename);
            return path;
        }
    }

    RRTPath TransitionTesting::generateTestPath(
            const AllGrippersSinglePose& gripper_target_poses,
            const unsigned long long num_discards)
    {
        // Update the seed for this particular trial
        BandRRT::WorldParams world_params = *world_params_;
        world_params.generator_ = std::make_shared<std::mt19937_64>(*world_params_->generator_);
        world_params.generator_->discard(num_discards);
        world_params.transition_estimator_ =
                std::make_shared<TransitionEstimation>(
                    nh_, ph_, world_params.generator_, sdf_, work_space_grid_, vis_, *initial_band_);

        // Pass in all the config values that the RRT needs; for example goal bias, step size, etc.
        auto band_rrt = BandRRT(nh_,
                                ph_,
                                world_params,
                                planning_params_,
                                smoothing_params_,
                                task_params_,
                                initial_band_,
                                vis_,
                                false);

        const auto gripper_config = RRTGrippersRepresentation(
                    initial_world_state_.all_grippers_single_pose_[0],
                    initial_world_state_.all_grippers_single_pose_[1]);

        RRTRobotRepresentation robot_config(6);
        robot_config.head<3>() = gripper_config.first.translation();
        robot_config.tail<3>() = gripper_config.second.translation();

        const auto rubber_band = RubberBand::BandFromWorldState(
                    initial_world_state_, *initial_band_);

        const RRTNode start_config(
                    gripper_config,
                    robot_config,
                    rubber_band);

        const std::chrono::duration<double> time_limit(GetRRTTimeout(*ph_));

        const auto policy = band_rrt.plan(start_config,
                                          {gripper_target_poses[0], gripper_target_poses[1]},
                                          time_limit);
        if (policy.size() == 0)
        {
            throw_arc_exception(std::runtime_error, "No path returned by RRT.");
        }
        else if (policy.size() > 1)
        {
            throw_arc_exception(std::runtime_error, "Multiple paths returned by RRT. Weird.");
        }
        return policy[0].first;
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Generate Last Step Approximations
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    void TransitionTesting::generateLastStepTransitionApproximations()
    {
        // Setup the transition data source to generate transition approximations from
        dmm::TransitionTestingVisualizationRequest req;
        req.data = "unmodified__last_step_test_results.compressed";
        dmm::TransitionTestingVisualizationResponse res;
        setTransitionAdaptationSourceCallback(req, res);
        assert(source_valid_);

        enum
        {
            FILENAME,
            ERROR_STRING,
            TEMPLATE_MISALIGNMENT_EUCLIDEAN,
            DEFAULT_VS_ADAPTATION_FOH,
            DEFAULT_VS_ADAPTATION_EUCLIDEAN,
            BAND_TIGHTEN_DELTA,
            SOURCE_NUM_FOH_CHANGES,
            RESULT_NUM_FOH_CHANGES,
            TRUE_VS_DEFAULT_FOH,
            TRUE_VS_DEFAULT_EUCLIDEAN,
            TRUE_VS_ADAPTATION_FOH,
            TRUE_VS_ADAPTATION_EUCLIDEAN,
            PLANNED_VS_ACTUAL_START_FOH,
            PLANNED_VS_ACTUAL_START_EUCLIDEAN,
            DUMMY_ITEM
        };
        Log::Log logger(data_folder_ + "/generate_last_step_transition_approximations.csv", false);
        ARC_LOG(logger, "FILENAME, "
                    "ERROR_STRING, "
                    "TEMPLATE_MISALIGNMENT_EUCLIDEAN, "
                    "DEFAULT_VS_ADAPTATION_FOH, "
                    "DEFAULT_VS_ADAPTATION_EUCLIDEAN, "
                    "BAND_TIGHTEN_DELTA, "
                    "SOURCE_NUM_FOH_CHANGES, "
                    "RESULT_NUM_FOH_CHANGES, "
                    "TRUE_VS_DEFAULT_FOH, "
                    "TRUE_VS_DEFAULT_EUCLIDEAN, "
                    "TRUE_VS_ADAPTATION_FOH, "
                    "TRUE_VS_ADAPTATION_EUCLIDEAN, "
                    "PLANNED_VS_ACTUAL_START_FOH, "
                    "PLANNED_VS_ACTUAL_START_EUCLIDEAN");
        #pragma omp parallel for
        for (size_t idx = 0; idx < last_step_data_files_.size(); ++idx)
        {
            const auto& experiment = last_step_data_files_[idx];
            const auto test_result_file =       experiment + "__last_step_test_results.compressed";
            const auto path_to_start_file =     experiment + "__path_to_start.compressed";
            const auto test_transition_file =   experiment + "__test_transition.compressed";
            const auto adaptation_result_file = experiment + "__adaptation_record.compressed";
            const auto failure_file =           experiment + "__adaptation_record.failed";

            std::vector<std::string> dists_etc(DUMMY_ITEM, "");
            dists_etc[FILENAME] = test_result_file.substr(data_folder_.length() + 1);
            try
            {
                // Check for the file flag that indicatest that this test is not possible
                if (fs::is_regular_file(failure_file))
                {
                    continue;
                }

                // Load the test result itself
                const dmm::TransitionTestResult test_result = loadTestResult(test_result_file);

                // Load the resulting transition, if needed generate it first
                const TransitionEstimation::StateTransition test_transition = [&]
                {
                    if (!fs::is_regular_file(test_transition_file))
                    {
                        // Load the path that generated the test
                        const RRTPath path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);
                        // Generate the transition at the end of the path
                        const auto transition = ToStateTransition(test_result, path_to_start);
                        transition_estimator_->saveStateTransition(transition, test_transition_file);
                        return transition;
                    }
                    else
                    {
                        return transition_estimator_->loadStateTransition(test_transition_file);
                    }
                }();

                // Load the adaptation record, if needed generate it first
                const TransitionEstimation::TransitionAdaptationResult adaptation_result = [&]
                {
                    if (!fs::is_regular_file(adaptation_result_file))
                    {
                        const auto ar = transition_estimator_->generateTransition(
                                    source_transition_,
                                    *test_transition.starting_state_.planned_rubber_band_,
                                    test_transition.ending_gripper_positions_);
                        transition_estimator_->saveAdaptationResult(ar, adaptation_result_file);
                        return ar;
                    }
                    else
                    {
                        return transition_estimator_->loadAdaptationResult(adaptation_result_file);
                    }
                }();

                const auto test_band_start = RubberBand::BandFromWorldState(ConvertToEigenFeedback(test_result.start_after_following_path), *initial_band_);
                if (test_band_start->isOverstretched())
                {
                    throw_arc_exception(std::runtime_error, "Starting configuration of test band is overstretched");
                }
                const auto test_band_end = RubberBand::BandFromWorldState(ConvertToEigenFeedback(test_result.microsteps_last_action.back()), *initial_band_);

                dists_etc[TEMPLATE_MISALIGNMENT_EUCLIDEAN] = std::to_string(adaptation_result.template_misalignment_dist_);
                dists_etc[DEFAULT_VS_ADAPTATION_FOH] = std::to_string(adaptation_result.default_band_foh_result_);
                dists_etc[DEFAULT_VS_ADAPTATION_EUCLIDEAN] = std::to_string(adaptation_result.default_band_dist_);

                dists_etc[BAND_TIGHTEN_DELTA] = std::to_string(adaptation_result.band_tighten_delta_);
                dists_etc[SOURCE_NUM_FOH_CHANGES] = std::to_string(source_num_foh_changes_);
                dists_etc[RESULT_NUM_FOH_CHANGES] = std::to_string(adaptation_result.num_foh_changes_);

                dists_etc[TRUE_VS_DEFAULT_FOH] = std::to_string(transition_estimator_->checkFirstOrderHomotopy(*adaptation_result.default_next_band_, *test_band_end));
                dists_etc[TRUE_VS_DEFAULT_EUCLIDEAN] = std::to_string(adaptation_result.default_next_band_->distance(*test_band_end));

                dists_etc[TRUE_VS_ADAPTATION_FOH] = std::to_string(transition_estimator_->checkFirstOrderHomotopy(*adaptation_result.result_, *test_band_end));
                dists_etc[TRUE_VS_ADAPTATION_EUCLIDEAN] = std::to_string(adaptation_result.result_->distance(*test_band_end));

                dists_etc[PLANNED_VS_ACTUAL_START_FOH] = std::to_string(transition_estimator_->checkFirstOrderHomotopy(*test_transition.starting_state_.planned_rubber_band_, *test_transition.starting_state_.rubber_band_));
                dists_etc[PLANNED_VS_ACTUAL_START_EUCLIDEAN] = std::to_string(test_transition.starting_state_.planned_rubber_band_->distance(*test_transition.starting_state_.rubber_band_));
            }
            catch (const std::exception& ex)
            {
                Log::Log failure_logger(failure_file, true);
                ARC_LOG_STREAM(failure_logger, "Error parsing idx: " << idx << " file: " << test_result_file << ": " << ex.what());
                ROS_ERROR_STREAM_NAMED("last_step_approximations", "Error parsing idx: " << idx << " file: " << test_result_file << ": " << ex.what());
                dists_etc[ERROR_STRING] = ex.what();
            }

            ARC_LOG(logger, PrettyPrint::PrettyPrint(dists_etc, false, ", "));
        }
    }

    bool TransitionTesting::setTransitionAdaptationSourceCallback(
            dmm::TransitionTestingVisualizationRequest& req,
            dmm::TransitionTestingVisualizationResponse& res)
    {
        (void)res;

        source_valid_ = false;

        const std::string delimiter = "__last_step_test_results.compressed";
        const auto pos = req.data.find(delimiter);
        const auto experiment = data_folder_ + "/" + req.data.substr(0, pos);

        const auto test_result_file =       experiment + "__last_step_test_results.compressed";
        const auto path_to_start_file =     experiment + "__path_to_start.compressed";
        const auto test_transition_file =   experiment + "__test_transition.compressed";

        // Load the resulting transition, if needed generate it first
        source_file_ = req.data;
        source_transition_ = [&]
        {
            if (!fs::is_regular_file(test_transition_file))
            {
                const RRTPath path_to_start =BandRRT::LoadPath(path_to_start_file, *initial_band_);
                const dmm::TransitionTestResult test_result = loadTestResult(test_result_file);

                const auto transition = ToStateTransition(test_result, path_to_start);
                transition_estimator_->saveStateTransition(transition, test_transition_file);
                return transition;
            }
            else
            {
                return transition_estimator_->loadStateTransition(test_transition_file);
            }
        }();
        source_band_surface_ = RubberBand::AggregateBandPoints(source_transition_.microstep_band_history_);

        std::vector<bool> foh_values;
        for (size_t idx = 0; idx < source_transition_.microstep_band_history_.size() - 1; ++idx)
        {
            RubberBand::Ptr b1 = source_transition_.microstep_band_history_[idx];
            RubberBand::Ptr b2 = source_transition_.microstep_band_history_[idx + 1];
            foh_values.push_back(transition_estimator_->checkFirstOrderHomotopy(*b1, *b2));
        }
        source_num_foh_changes_ = 0;
        for (size_t idx = 0; idx < foh_values.size() - 1; ++idx)
        {
            if (foh_values[idx] != foh_values[idx + 1])
            {
                ++source_num_foh_changes_;
            }
        }

        // Ensure all bands have been upsampled and resampled to avoid race conditions in multithreading later
        source_transition_.starting_state_.rubber_band_->upsampleBand();
        source_transition_.starting_state_.rubber_band_->resampleBand();
        source_transition_.starting_state_.planned_rubber_band_->upsampleBand();
        source_transition_.starting_state_.planned_rubber_band_->resampleBand();
        source_transition_.ending_state_.rubber_band_->upsampleBand();
        source_transition_.ending_state_.rubber_band_->resampleBand();
        source_transition_.ending_state_.planned_rubber_band_->upsampleBand();
        source_transition_.ending_state_.planned_rubber_band_->resampleBand();

        source_valid_ = true;
        ROS_INFO_STREAM("Source transition set to " << req.data);
        return true;
    }

    bool TransitionTesting::addTransitionAdaptationVisualizationCallback(
            dmm::TransitionTestingVisualizationRequest& req,
            dmm::TransitionTestingVisualizationResponse& res)
    {
        if (!source_valid_)
        {
            ROS_WARN_NAMED("data_visualization", "Visualization requested, but transition source is invalid");
            res.response = "Visualization requested, but transition source is invalid";
            return false;
        }

        const std::string delimiter = "__last_step_test_results.compressed";
        const auto pos = req.data.find(delimiter);
        const auto experiment = data_folder_ + "/" + req.data.substr(0, pos);

        const auto test_result_file =       experiment + "__last_step_test_results.compressed";
        const auto path_to_start_file =     experiment + "__path_to_start.compressed";
        const auto test_transition_file =   experiment + "__test_transition.compressed";

        // Load the test result itself
        const dmm::TransitionTestResult test_result = loadTestResult(test_result_file);

        // Load the resulting transition, if needed generate it first
        const TransitionEstimation::StateTransition test_transition = [&]
        {
            if (!fs::is_regular_file(test_transition_file))
            {
                // Load the path that generated the test
                const RRTPath path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);
                // Generate the transition at the end of the path
                const auto transition = ToStateTransition(test_result, path_to_start);
                transition_estimator_->saveStateTransition(transition, test_transition_file);
                return transition;
            }
            else
            {
                return transition_estimator_->loadStateTransition(test_transition_file);
            }
        }();

        // Don't use any saved files as we could be using a different source transition
        const TransitionEstimation::TransitionAdaptationResult adaptation_result =
                transition_estimator_->generateTransition(
                    source_transition_,
                    *test_transition.starting_state_.planned_rubber_band_,
                    test_transition.ending_gripper_positions_);

        const auto sim_record = TransitionSimulationRecord
        {
            source_transition_,
            RubberBand::AggregateBandPoints(source_transition_.microstep_band_history_),
            test_transition,
            RubberBand::AggregateBandPoints(test_transition.microstep_band_history_),
            adaptation_result
        };

        // Remove any existing visualization at this id (if there is one)
        {
            dmm::TransitionTestingVisualizationRequest dmmreq;
            dmmreq.data = std::to_string(next_vis_prefix_);
            dmm::TransitionTestingVisualizationResponse dmmres;
            removeVisualizationCallback(dmmreq, dmmres);
        }

        res.response = std::to_string(next_vis_prefix_);
        visid_to_markers_[res.response] = sim_record.visualize(std::to_string(next_vis_prefix_) + "__", vis_);
        ++next_vis_prefix_;

        const auto test_band_end = RubberBand::BandFromWorldState(ConvertToEigenFeedback(test_result.microsteps_last_action.back()), *initial_band_);
        ROS_INFO_STREAM("Added vis id: " << res.response << " for file " << req.data << std::endl
                        << "Template alignment dist:      " << adaptation_result.template_misalignment_dist_ << std::endl
                        << "Default band FOH:             " << adaptation_result.default_band_foh_result_ << std::endl
                        << "Default band dist:            " << adaptation_result.default_band_dist_ << std::endl
                        << "Band tighten delta:           " << adaptation_result.band_tighten_delta_ << std::endl
                        << "Source FOH changes:           " << source_num_foh_changes_ << std::endl
                        << "Adaptation FOH changes:       " << adaptation_result.num_foh_changes_ << std::endl
                        << "True vs default FOH:          " << transition_estimator_->checkFirstOrderHomotopy(*adaptation_result.default_next_band_, *test_band_end) << std::endl
                        << "True vs default dist:         " << adaptation_result.default_next_band_->distance(*test_band_end) << std::endl
                        << "True vs adaptation FOH:       " << transition_estimator_->checkFirstOrderHomotopy(*adaptation_result.result_, *test_band_end) << std::endl
                        << "True vs adaptation dist:      " << adaptation_result.result_->distance(*test_band_end) << std::endl
                        << "Planned vs actual start FOH:  " << transition_estimator_->checkFirstOrderHomotopy(*test_transition.starting_state_.planned_rubber_band_, *test_transition.starting_state_.rubber_band_) << std::endl
                        << "Planned vs actual start dist: " << test_transition.starting_state_.planned_rubber_band_->distance(*test_transition.starting_state_.rubber_band_) << std::endl);
        return true;
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Generate Trajectories from rrt::paths and dmm::results
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    void TransitionTesting::generateLastStepTrajectories()
    {
        std::atomic<int> num_succesful_paths = 0;
        std::atomic<int> num_unsuccesful_paths = 0;

        const auto omp_threads = (deformable_type_ == ROPE) ? arc_helpers::GetNumOMPThreads() : 2;
        #pragma omp parallel for num_threads(omp_threads)
        for (size_t idx = 0; idx < last_step_data_files_.size(); ++idx)
        {
            const auto& experiment = last_step_data_files_[idx];
            const auto test_result_file =       experiment + "__last_step_test_results.compressed";
            const auto path_to_start_file =     experiment + "__path_to_start.compressed";
            const auto trajectory_file =        experiment + "__trajectory.compressed";

            try
            {
//                if (!fs::is_regular_file(trajectory_file))
                {
                    const auto path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);
                    const auto test_result = loadTestResult(test_result_file);

                    const auto traj_gen_result = toTrajectory(test_result, path_to_start, test_result_file);
                    const auto& trajectory = traj_gen_result.first;
                    transition_estimator_->saveTrajectory(trajectory, trajectory_file);

                    if (traj_gen_result.second)
                    {
                        ++num_succesful_paths;
                    }
                    else
                    {
                        ++num_unsuccesful_paths;
                    }
                }
            }
            catch (const std::exception& ex)
            {
                ROS_ERROR_STREAM_NAMED("generate_trajectories", "Error parsing idx: " << idx << " file: " << test_result_file << ": " << ex.what());
                ++num_unsuccesful_paths;
            }
        }

        const int classifier_dim = ROSHelpers::GetParamRequiredDebugLog<int>(*ph_, "classifier/dim", __func__);
        const std::string classifier_slice_type = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "classifier/slice_type", __func__);
        const std::string classifier_type = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "classifier/type", __func__);

        ROS_INFO_STREAM(
                    classifier_dim << " " <<
                    classifier_slice_type << " " <<
                    classifier_type << " " <<
                    "Total successful paths: " << num_succesful_paths << "    Total unsuccessful paths: " << num_unsuccesful_paths);
    }

    void TransitionTesting::visualizeLastStepDataTrajectories()
    {
        const auto omp_threads = (deformable_type_ == ROPE) ? arc_helpers::GetNumOMPThreads() : 2;
        #pragma omp parallel for num_threads(omp_threads)
        for (size_t idx = 0; idx < last_step_data_files_.size(); ++idx)
        {
            const auto& experiment = last_step_data_files_[idx];
            const auto test_result_file =       experiment + "__last_step_test_results.compressed";
            const auto path_to_start_file =     experiment + "__path_to_start.compressed";

            try
            {
                const RRTPath path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);;
                const dmm::TransitionTestResult test_result = loadTestResult(test_result_file);
                const auto traj_gen_result = toTrajectory(test_result, path_to_start, experiment.substr(data_folder_.length() + 1));
                const auto& trajectory = traj_gen_result.first;
                const auto parsed_cleanly = traj_gen_result.second;

//                if (!parsed_cleanly)
                #pragma omp critical
                {
                    // Original Visualizations
                    {
                        // Visualize and then pause, waiting for the user to move to the next trajectory
                        const std::string ns_prefix = std::to_string(next_vis_prefix_) + "__";
                        const auto marker_ids = visualizePathAndTrajectory(path_to_start, trajectory, ns_prefix);
                        std::cout << "Experiment: " << experiment.substr(data_folder_.length() + 1) << "    ";
                        PressAnyKeyToContinue();

                        // Rmove any visualizations added to make a clean slate for the next set of visualizations
                        for (const auto& nsid : marker_ids)
                        {
                            vis_->deleteObject(nsid.first, nsid.second);
                        }
                    }

                    // Extra stuff for workshop figures
                    if (false)
                    {
                        std::vector<Visualizer::NamespaceId> marker_ids;
                        const std::string ns_prefix = std::to_string(next_vis_prefix_) + "__";

                        assert(trajectory.size() > 0);
                        for (size_t traj_step = 1; traj_step < trajectory.size(); ++traj_step)
                        {
                            const auto& start_state = trajectory[traj_step - 1].first;
                            const auto& end_state = trajectory[traj_step].first;
                            const bool start_foh = transition_estimator_->checkFirstOrderHomotopy(
                                        *start_state.planned_rubber_band_,
                                        *start_state.rubber_band_);
                            const bool end_foh = transition_estimator_->checkFirstOrderHomotopy(
                                        *end_state.planned_rubber_band_,
                                        *end_state.rubber_band_);
                            const auto start_dist = start_state.planned_rubber_band_->distance(*start_state.rubber_band_);
                            const auto end_dist = end_state.planned_rubber_band_->distance(*end_state.rubber_band_);
                            const auto start_close = start_foh && (start_dist <= mistake_dist_thresh_);
                            const auto end_close = end_foh && (end_dist <= mistake_dist_thresh_);
                            const bool mistake = start_close && !end_close;

                            const TransitionEstimation::StateTransition transition
                            {
                                start_state,
                                end_state,
                                start_state.planned_rubber_band_->getEndpoints(),
                                end_state.planned_rubber_band_->getEndpoints(),
                                trajectory[traj_step].second,
                                transition_estimator_->reduceMicrostepsToBands(trajectory[traj_step].second)
                            };
                            // Add the planned vs executed start and end bands on their own namespaces
                            {
                                const auto new_ids1 = transition.starting_state_.planned_rubber_band_->visualize(
                                            ns_prefix + "MISTAKE_START_PLANNED",
                                            Visualizer::Green(),
                                            Visualizer::Green(),
                                            1);
                                const auto new_ids2 = transition.starting_state_.rubber_band_->visualize(
                                            ns_prefix + "MISTAKE_START_EXECUTED",
                                            Visualizer::Red(),
                                            Visualizer::Red(),
                                            1);
                                const auto new_ids3 = transition.ending_state_.planned_rubber_band_->visualize(
                                            ns_prefix + "MISTAKE_END_PLANNED",
                                            Visualizer::Olive(),
                                            Visualizer::Olive(),
                                            1);
                                const auto new_ids4 = transition.ending_state_.rubber_band_->visualize(
                                            ns_prefix + "MISTAKE_END_EXECUTED",
                                            Visualizer::Orange(),
                                            Visualizer::Orange(),
                                            1);

                                const auto new_ids5 = vis_->visualizeGrippers(
                                            ns_prefix + "MISTAKE_START_GRIPPERS",
                                            transition.starting_gripper_positions_,
                                            Visualizer::Blue(),
                                            1);
                                const auto new_ids6 = vis_->visualizeGrippers(
                                            ns_prefix + "MISTAKE_END_GRIPPERS",
                                            transition.ending_gripper_positions_,
                                            Visualizer::Blue(),
                                            1);

                                marker_ids.insert(marker_ids.begin(), new_ids1.begin(), new_ids1.end());
                                marker_ids.insert(marker_ids.begin(), new_ids2.begin(), new_ids2.end());
                                marker_ids.insert(marker_ids.begin(), new_ids3.begin(), new_ids3.end());
                                marker_ids.insert(marker_ids.begin(), new_ids4.begin(), new_ids4.end());
                                marker_ids.insert(marker_ids.begin(), new_ids5.begin(), new_ids5.end());
                                marker_ids.insert(marker_ids.begin(), new_ids6.begin(), new_ids6.end());
                            }

                            static const auto parabola_slice_option = ROSHelpers::GetParam<std::string>(*ph_, "parabola_slice_option", "basic");
                            const auto features = extractFeatures(transition, parabola_slice_option);
                            assert(FEATURE_NAMES.size() == features.size());
                            if (features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA] != std::to_string(0) ||
                                features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA] != std::to_string(0))
                            {
                                assert(features.size() == FEATURE_NAMES.size());
                                ROS_INFO_STREAM_NAMED("features", test_result_file);
                                ROS_INFO_STREAM_NAMED("features", "  Transition at idx: " << traj_step << " with distance " << end_dist << " and mistake recorded: " << mistake);
                                for (size_t i = 0; i < FEATURE_NAMES.size(); ++i)
                                {
                                    ROS_INFO_STREAM_NAMED("features", "  " << /* std::left << */ std::setw(48) << FEATURE_NAMES[i] << ": " << features[i]);
                                }

                                if (mistake)
                                {
                                    PressAnyKeyToContinue();
                                }
                            }

                            // Rmove any visualizations added to make a clean slate for the next set of visualizations
                            for (const auto& nsid : marker_ids)
                            {
                                vis_->deleteObject(nsid.first, nsid.second);
                            }
                        }
                    }
                }
            }
            catch (const std::exception& ex)
            {
                ROS_ERROR_STREAM_NAMED("visualizeLastStepDataTrajectories", "Error parsing idx: " << idx << " file: " << test_result_file << ": " << ex.what());
            }
        }
    }

    void TransitionTesting::visualizePathTestTrajectories()
    {
        const auto omp_threads = (deformable_type_ == ROPE) ? arc_helpers::GetNumOMPThreads() : 2;
        #pragma omp parallel for num_threads(omp_threads)
        for (size_t idx = 0; idx < path_test_data_files_.size(); ++idx)
        {
            const auto& experiment = path_test_data_files_[idx];
            const auto rrt_path_file = experiment + "__rrt_path.compressed";
            const auto test_result_file = experiment + "__path_test_results.compressed";

            try
            {
                const WorldState starting_world_state = [&]
                {
                    // First look for a world state stored with this specific trial's name
                    std::string world_state_file = experiment + "__starting_world_state.compressed";
                    if (!fs::is_regular_file(world_state_file))
                    {
                        // Otherwise, look for an initial state trimming off the __trial_idx_### tag
                        const auto pos = experiment.find("__trial_idx_");
                        world_state_file = experiment.substr(0, pos) + "__starting_world_state.compressed";
                    }

                    std::pair<WorldState, RubberBand::Ptr> deserialized_result;
                    const auto buffer = ZlibHelpers::LoadFromFileAndDecompress(world_state_file);
                    const auto deserialized_world_state = WorldState::Deserialize(buffer, 0);
                    deserialized_result.first = deserialized_world_state.first;
                    // We don't need to deserialize the band state
//                        deserialized_result.second = std::make_shared<RubberBand>(*rubber_band_);
//                        deserialized_result.second->deserialize(buffer, deserialized_world_state.second);
                    return deserialized_world_state.first;
                }();

                const RRTPath rrt_path = BandRRT::LoadPath(rrt_path_file, *initial_band_);
                const auto test_waypoint_indices = [&]
                {
                    // It is assumed that the robot starts where the path is at idx 0, so trim that element from the planned path
                    const RRTPath commanded_path(rrt_path.begin() + 1, rrt_path.end());
                    assert(commanded_path.size() > 0 && "If this is false, it probably means that plan_start == plan_goal");
                    const auto robot_path = RRTPathToGrippersPoseTrajectory(commanded_path);
                    const auto interp_result = robot_->interpolateGrippersTrajectory(robot_path);
                    return interp_result.second;
                }();

                const auto test_result = [&]
                {
                    const auto buffer = ZlibHelpers::LoadFromFileAndDecompress(test_result_file);
                    return arc_utilities::RosMessageDeserializationWrapper<deformable_manipulation_msgs::TestRobotPathsFeedback>(buffer, 0).first.test_result;
                }();

                const auto traj_gen_result = ToTrajectory(starting_world_state, rrt_path, test_result, test_waypoint_indices);
                const auto& trajectory = traj_gen_result.first;
                const auto parsed_cleanly = traj_gen_result.second;

//                if (!parsed_cleanly)
                #pragma omp critical
                {
                    // Visualize and then pause, waiting for the user to move to the next trajectory
                    const std::string ns_prefix = std::to_string(next_vis_prefix_) + "__";
                    const auto marker_ids = visualizePathAndTrajectory(rrt_path, trajectory, ns_prefix);
                    std::cout << "Experiment: " << experiment.substr(data_folder_.length() + 1) << "    ";
                    PressAnyKeyToContinue();

                    // Rmove any visualizations added to make a clean slate for the next set of visualizations
                    for (const auto& nsid : marker_ids)
                    {
                        vis_->deleteObject(nsid.first, nsid.second);
                    }
                }
            }
            catch (const std::exception& ex)
            {
                ROS_ERROR_STREAM_NAMED("visualizePathTestTrajectories", "Error parsing idx: " << idx << " file: " << test_result_file << ": " << ex.what());
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Find Meaningful Mistakes
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    void TransitionTesting::generateMeaningfulMistakeExamples()
    {
        enum
        {
            FILENAME,
            ERROR_STRING,
            PLANNED_VS_EXECUTED_START_EUCLIDEAN,
            PLANNED_VS_EXECUTED_END_EUCLIDEAN,
            START_VS_END_EUCLIDEN_PLANNED,
            START_VS_END_EUCLIDEN_EXECUTED,
            FOH_RESULTS,
            NUM_FOH_CHANGES,
            LARGEST_FOH_CHANGE_DIST,
            DUMMY_ITEM
        };
        Log::Log logger(data_folder_ + "/generate_meaningful_mistake_examples.csv", false);
        ARC_LOG(logger, "FILENAME, "
                    "ERROR_STRING, "
                    "PLANNED_VS_EXECUTED_START_EUCLIDEAN, "
                    "PLANNED_VS_EXECUTED_END_EUCLIDEAN, "
                    "START_VS_END_EUCLIDEN_PLANNED, "
                    "START_VS_END_EUCLIDEN_EXECUTED, "
                    "FOH_RESULTS, "
                    "LARGEST_FOH_CHANGE_DIST, "
                    "NUM_FOH_CHANGES");

        #pragma omp parallel for
        for (size_t idx = 0; idx < last_step_data_files_.size(); ++idx)
        {
            const auto& experiment = last_step_data_files_[idx];
            const auto test_result_file =       experiment + "__last_step_test_results.compressed";
            const auto path_to_start_file =     experiment + "__path_to_start.compressed";
            const auto trajectory_file =        experiment + "__trajectory.compressed";
            const auto example_mistake_file =   experiment + "__example_mistake.compressed";
            const auto failure_file =           experiment + "__example_mistake.failed";

            std::vector<std::string> dists_etc(DUMMY_ITEM, "");
            dists_etc[FILENAME] = test_result_file.substr(data_folder_.length() + 1);
            try
            {
                // Check for the file flag that indicatest that this test is not possible
                if (fs::is_regular_file(failure_file))
                {
                    continue;
                }

                // Load the transition example if possible, otherwise generate it
                const TransitionEstimation::StateTransition transition = [&]
                {
                    if (!fs::is_regular_file(example_mistake_file))
                    {
                        // Load the trajectory if possible, otherwise generate it
                        const auto trajectory = [&]
                        {
                            if (!fs::is_regular_file(trajectory_file))
                            {
                                const RRTPath path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);
                                const dmm::TransitionTestResult test_result = loadTestResult(test_result_file);
                                const auto traj = toTrajectory(test_result, path_to_start, experiment.substr(data_folder_.length() + 1)).first;
                                transition_estimator_->saveTrajectory(traj, trajectory_file);
                                return traj;
                            }
                            else
                            {
                                return transition_estimator_->loadTrajectory(trajectory_file);
                            }
                        }();

                        const auto example = transition_estimator_->findMostRecentBadTransition(trajectory).Get();
                        transition_estimator_->saveStateTransition(example, example_mistake_file);
                        return example;
                    }
                    else
                    {
                        return transition_estimator_->loadStateTransition(example_mistake_file);
                    }
                }();

                std::vector<int> foh_values;
                for (size_t step_idx = 0; step_idx < transition.microstep_band_history_.size() - 1; ++step_idx)
                {
                    RubberBand::Ptr b1 = transition.microstep_band_history_[step_idx];
                    RubberBand::Ptr b2 = transition.microstep_band_history_[step_idx + 1];
                    foh_values.push_back(transition_estimator_->checkFirstOrderHomotopy(*b1, *b2));
                }
                int num_foh_changes = 0;
                double largest_foh_change_dist = 0.0;
                for (size_t foh_idx = 0; foh_idx < foh_values.size() - 1; ++foh_idx)
                {
                    if (foh_values[foh_idx] != foh_values[foh_idx + 1])
                    {
                        ++num_foh_changes;

                        RubberBand::Ptr b1 = transition.microstep_band_history_[foh_idx];
                        RubberBand::Ptr b2 = transition.microstep_band_history_[foh_idx + 1];
                        const double foh_change_dist = b1->distance(*b2);
                        largest_foh_change_dist = std::max(largest_foh_change_dist, foh_change_dist);
                    }
                }

                dists_etc[PLANNED_VS_EXECUTED_START_EUCLIDEAN] = std::to_string(transition.starting_state_.planned_rubber_band_->distance(*transition.starting_state_.rubber_band_));
                dists_etc[PLANNED_VS_EXECUTED_END_EUCLIDEAN] = std::to_string(transition.ending_state_.planned_rubber_band_->distance(*transition.ending_state_.rubber_band_));

                dists_etc[START_VS_END_EUCLIDEN_PLANNED] = std::to_string(transition.starting_state_.planned_rubber_band_->distance(*transition.ending_state_.planned_rubber_band_));
                dists_etc[START_VS_END_EUCLIDEN_EXECUTED] = std::to_string(transition.starting_state_.rubber_band_->distance(*transition.ending_state_.rubber_band_));

                dists_etc[FOH_RESULTS] = PrettyPrint::PrettyPrint(foh_values, false, "");
                dists_etc[NUM_FOH_CHANGES] = std::to_string(num_foh_changes);
                dists_etc[LARGEST_FOH_CHANGE_DIST] = std::to_string(largest_foh_change_dist);
            }
            catch (const std::exception& ex)
            {
                Log::Log failure_logger(failure_file, true);
                ARC_LOG_STREAM(failure_logger, "Error parsing idx: " << idx << " file: " << test_result_file << ": " << ex.what());
                ROS_ERROR_STREAM_NAMED("meaningful_mistake", "Error parsing idx: " << idx << " file: " << test_result_file << ": " << ex.what());
                dists_etc[ERROR_STRING] = ex.what();
            }

            ARC_LOG(logger, PrettyPrint::PrettyPrint(dists_etc, false, ", "));
        }
    }

    bool TransitionTesting::addMistakeExampleVisualizationCallback(
            dmm::TransitionTestingVisualizationRequest& req,
            dmm::TransitionTestingVisualizationResponse& res)
    {
        const std::string delimiter = "__last_step_test_results.compressed";
        const auto pos = req.data.find(delimiter);
        const auto experiment = data_folder_ + "/" + req.data.substr(0, pos);

        const auto test_result_file =       experiment + "__last_step_test_results.compressed";
        const auto path_to_start_file =     experiment + "__path_to_start.compressed";
        const auto example_mistake_file =   experiment + "__example_mistake.compressed";
        const auto trajectory_file =        experiment + "__trajectory.compressed";

        // Load the path that generated the test
        const RRTPath path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);

        // Load the trajectory if possible, otherwise generate it
        const auto trajectory = [&]
        {
            if (!fs::is_regular_file(trajectory_file))
            {
                const dmm::TransitionTestResult test_result = loadTestResult(test_result_file);
                const auto traj = toTrajectory(test_result, path_to_start, experiment.substr(data_folder_.length() + 1)).first;
                transition_estimator_->saveTrajectory(traj, trajectory_file);
                return traj;
            }
            else
            {
                return transition_estimator_->loadTrajectory(trajectory_file);
            }
        }();

        // Load the transition example if possible, otherwise generate it
        const TransitionEstimation::StateTransition transition = [&]
        {
            if (!fs::is_regular_file(example_mistake_file))
            {
                const auto example = transition_estimator_->findMostRecentBadTransition(trajectory).Get();
                transition_estimator_->saveStateTransition(example, example_mistake_file);
                return example;
            }
            else
            {
                return transition_estimator_->loadStateTransition(example_mistake_file);
            }
        }();

        // Determine the FOH and distance values along the band surface
        Matrix2Xd dist_and_foh_values(2, transition.microstep_band_history_.size() - 1);
        for (size_t step_idx = 0; step_idx < transition.microstep_band_history_.size() - 1; ++step_idx)
        {
            RubberBand::Ptr b1 = transition.microstep_band_history_[step_idx];
            RubberBand::Ptr b2 = transition.microstep_band_history_[step_idx + 1];
            dist_and_foh_values(0, step_idx) = b1->distance(*b2);
            dist_and_foh_values(1, step_idx) = transition_estimator_->checkFirstOrderHomotopy(*b1, *b2);
        }
        int num_foh_changes = 0;
        for (ssize_t step_idx = 0; step_idx < dist_and_foh_values.cols() - 1; ++step_idx)
        {
            if (dist_and_foh_values(1, step_idx) != dist_and_foh_values(1, step_idx + 1))
            {
                ++num_foh_changes;
            }
        }

        // Visualization
        {
            const std::string ns_prefix = std::to_string(next_vis_prefix_) + "__";

            // Remove any existing visualization at this id (if there is one)
            {
                dmm::TransitionTestingVisualizationRequest dmmreq;
                dmmreq.data = std::to_string(next_vis_prefix_);
                dmm::TransitionTestingVisualizationResponse dmmres;
                removeVisualizationCallback(dmmreq, dmmres);
            }

            // Planned path and actual trajectory taken
            auto marker_ids = visualizePathAndTrajectory(path_to_start, trajectory, ns_prefix);

            // Discovered mistake
            {
                // Add the first band surface band
                {
                    const bool foh = dist_and_foh_values(1, 0);
                    const auto color = foh ? Visualizer::Green() : Visualizer::Red();
                    const auto ns = foh ? ns_prefix + "MISTAKE_EXECUTED_BAND_SURFACE_FOH_SAME" : ns_prefix + "MISTAKE_EXECUTED_BAND_SURFACE_FOH_DIFF";
                    const auto new_ids = transition.microstep_band_history_.back()->visualize(ns, color, color, (int)(dist_and_foh_values.cols() + 1));
                    marker_ids.insert(marker_ids.begin(), new_ids.begin(), new_ids.end());
                }
                // Add the "middle" band surface bands
                for (size_t step_idx = 1; step_idx < transition.microstep_band_history_.size() - 1; ++step_idx)
                {
                    const auto ratio = (float)(step_idx) / (float)(transition.microstep_band_history_.size() - 1);
                    const bool foh = (bool)dist_and_foh_values(1, step_idx - 1) && (bool)dist_and_foh_values(1, step_idx);
                    const auto color = foh
                            ? InterpolateColor(Visualizer::Green(), Visualizer::Cyan(), ratio)
                            : InterpolateColor(Visualizer::Red(), Visualizer::Magenta(), ratio);
                    const auto ns = foh ? ns_prefix + "MISTAKE_EXECUTED_BAND_SURFACE_FOH_SAME" : ns_prefix + "MISTAKE_EXECUTED_BAND_SURFACE_FOH_DIFF";
                    const auto new_ids = transition.microstep_band_history_[step_idx]->visualize(ns, color, color, (int)(step_idx + 1));
                    marker_ids.insert(marker_ids.begin(), new_ids.begin(), new_ids.end());
                }
                // Add the last band surface band
                {
                    const bool foh = dist_and_foh_values(1, dist_and_foh_values.cols() - 1);
                    const auto color = foh ? Visualizer::Cyan() : Visualizer::Magenta();
                    const auto ns = foh ? ns_prefix + "MISTAKE_EXECUTED_BAND_SURFACE_FOH_SAME" : ns_prefix + "MISTAKE_EXECUTED_BAND_SURFACE_FOH_DIFF";
                    const auto new_ids = transition.microstep_band_history_.back()->visualize(ns, color, color, (int)(dist_and_foh_values.cols() + 1));
                    marker_ids.insert(marker_ids.begin(), new_ids.begin(), new_ids.end());
                }

                // Add the planned vs executed start and end bands on their own namespaces
                {
                    const auto new_ids1 = transition.starting_state_.planned_rubber_band_->visualize(
                                ns_prefix + "MISTAKE_START_PLANNED",
                                Visualizer::Green(),
                                Visualizer::Green(),
                                1);
                    const auto new_ids2 = transition.starting_state_.rubber_band_->visualize(
                                ns_prefix + "MISTAKE_START_EXECUTED",
                                Visualizer::Red(),
                                Visualizer::Red(),
                                1);
                    const auto new_ids3 = transition.ending_state_.planned_rubber_band_->visualize(
                                ns_prefix + "MISTAKE_END_PLANNED",
                                Visualizer::Olive(),
                                Visualizer::Olive(),
                                1);
                    const auto new_ids4 = transition.ending_state_.rubber_band_->visualize(
                                ns_prefix + "MISTAKE_END_EXECUTED",
                                Visualizer::Orange(),
                                Visualizer::Orange(),
                                1);

                    marker_ids.insert(marker_ids.begin(), new_ids1.begin(), new_ids1.end());
                    marker_ids.insert(marker_ids.begin(), new_ids2.begin(), new_ids2.end());
                    marker_ids.insert(marker_ids.begin(), new_ids3.begin(), new_ids3.end());
                    marker_ids.insert(marker_ids.begin(), new_ids4.begin(), new_ids4.end());
                }
            }

            res.response = std::to_string(next_vis_prefix_);
            visid_to_markers_[res.response] = marker_ids;
            ++next_vis_prefix_;
        }

        ROS_INFO_STREAM("Added vis id: " << res.response << " for file " << req.data << std::endl
                        << "Planned vs executed start FOH:      " << transition_estimator_->checkFirstOrderHomotopy(*transition.starting_state_.planned_rubber_band_, *transition.starting_state_.rubber_band_) << std::endl
                        << "Planned vs executed start dist:     " << transition.starting_state_.planned_rubber_band_->distance(*transition.starting_state_.rubber_band_) << std::endl
                        << "Planned vs executed end FOH:        " << transition_estimator_->checkFirstOrderHomotopy(*transition.ending_state_.planned_rubber_band_, *transition.ending_state_.rubber_band_) << std::endl
                        << "Planned vs executed end dist:       " << transition.ending_state_.planned_rubber_band_->distance(*transition.ending_state_.rubber_band_) << std::endl
                        << "Start vs end dist planned:          " << transition.starting_state_.planned_rubber_band_->distance(*transition.ending_state_.planned_rubber_band_) << std::endl
                        << "Start vs end dist executed:         " << transition.starting_state_.rubber_band_->distance(*transition.ending_state_.rubber_band_) << std::endl
                        << "Num FOH changes:                    " << num_foh_changes << std::endl
                        << "Distance and FOH values along band surface:\n" << dist_and_foh_values.transpose() << std::endl);

        return true;
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Generate data for offline learning testing of features etc
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    void TransitionTesting::generateFeatures(const std::string& parabola_slice_option)
    {
        int num_examples_delta_eq_0 = 0;
        int num_examples_delta_neq_0 = 0;

        // The following files all have length 1 trajectories due to overstretch
        // and band creation failures, and thus lead to no features being generated
//        "seed15bd52f1e32b1391__stamp2019-08-22__21-59-46"
//        "seed15bd52f1e32b1391__stamp2019-08-22__21-59-48"
//        "seed15bd52f1e32b1391__stamp2019-08-22__21-59-50"
//        "seed15bd52f1e32b1391__stamp2019-08-22__21-59-52"
//        "seed15bd52f1e32b1391__stamp2019-08-22__21-59-54"
//        "seed15bd52f1e32b1391__stamp2019-08-22__21-59-56"
//        "seed15bd52f1e32b1391__stamp2019-08-22__21-59-59"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-01"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-04"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-06"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-08"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-10"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-12"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-15"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-17"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-19"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-22"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-24"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-26"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-29"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-31"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-33"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-35"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-38"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-40"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-42"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-45"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-47"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-49"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-51"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-54"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-57"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-00-59"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-02"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-04"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-06"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-08"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-10"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-13"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-15"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-17"
//        "seed15bd52f1e32b1391__stamp2019-08-22__22-01-19"

        #pragma omp parallel for
        for (size_t file_idx = 0; file_idx < last_step_data_files_.size(); ++file_idx)
        {
            const auto& experiment = last_step_data_files_[file_idx];
            const auto test_result_file =            experiment + "__last_step_test_results.compressed";
            const auto path_to_start_file =          experiment + "__path_to_start.compressed";
            const auto trajectory_file =             experiment + "__trajectory.compressed";
            const auto features_file =               experiment + "__classification_features__" + parabola_slice_option + "__start_and_end_dists.csv";
            const auto features_complete_flag_file = experiment + "__classification_features__" + parabola_slice_option + "__start_and_end_dists.complete";

            try
            {
                if (!fs::is_regular_file(features_complete_flag_file))
                {
                    const RRTPath path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);

                    // Load the trajectory if possible, otherwise generate it
                    const auto trajectory = [&]
                    {
                        if (!fs::is_regular_file(trajectory_file))
                        {
                            const dmm::TransitionTestResult test_result = loadTestResult(test_result_file);
                            const auto traj = toTrajectory(test_result, path_to_start, experiment.substr(data_folder_.length() + 1)).first;
                            transition_estimator_->saveTrajectory(traj, trajectory_file);
                            return traj;
                        }
                        else
                        {
                            return transition_estimator_->loadTrajectory(trajectory_file);
                        }
                    }();

                    assert(trajectory.size() > 0);
                    if (trajectory.size() == 1)
                    {
                        ROS_WARN_STREAM_NAMED("features", experiment << " has only one state in the resulting trajectory, skipping this file");
                        continue;
                    }

                    // Step through the trajectory, looking for cases where the prediction goes
                    // from "close" to "close" with everything else labeled as a mistake
                    Log::Log logger(features_file, false);
                    for (size_t idx = 1; idx < trajectory.size(); ++idx)
                    {
                        const auto& start_state = trajectory[idx - 1].first;
                        const auto& end_state = trajectory[idx].first;
                        const bool start_foh = transition_estimator_->checkFirstOrderHomotopy(
                                    *start_state.planned_rubber_band_,
                                    *start_state.rubber_band_);
                        const bool end_foh = transition_estimator_->checkFirstOrderHomotopy(
                                    *end_state.planned_rubber_band_,
                                    *end_state.rubber_band_);
                        const auto dist_pre = start_state.planned_rubber_band_->distance(*start_state.rubber_band_);
                        const auto dist_post = end_state.planned_rubber_band_->distance(*end_state.rubber_band_);

                        // Only compute the microstep band history if we're going to actually use it
                        std::vector<RubberBand::Ptr> microstep_band_history;
                        if (!disable_visualizations_)
                        {
                            microstep_band_history = transition_estimator_->reduceMicrostepsToBands(trajectory[idx].second);
                        }

                        const TransitionEstimation::StateTransition transition
                        {
                            start_state,
                            end_state,
                            start_state.planned_rubber_band_->getEndpoints(),
                            end_state.planned_rubber_band_->getEndpoints(),
                            trajectory[idx].second,
                            microstep_band_history
                        };
                        const auto features = extractFeatures(transition, parabola_slice_option);
                        assert(FEATURE_NAMES.size() == features.size());
//                        const bool mistake =
//                                (start_foh && dist_pre <= mistake_dist_thresh_) &&
//                                start_foh &&
//                                !(end_foh && dist_post <= mistake_dist_thresh_);
                        ARC_LOG_STREAM(logger, std::to_string(start_foh) << ", " <<
                                               std::to_string(dist_pre) << ", " <<
                                               std::to_string(end_foh) << ", " <<
                                               std::to_string(dist_post) << ", " <<
                                               PrettyPrint::PrettyPrint(features, false, ", "));

                        if (!disable_visualizations_)
                        {
                            // Determine the FOH and distance values along the band surface
                            Matrix2Xd dist_and_foh_values(2, transition.microstep_band_history_.size() - 1);
                            for (size_t step_idx = 0; step_idx < transition.microstep_band_history_.size() - 1; ++step_idx)
                            {
                                RubberBand::Ptr b1 = transition.microstep_band_history_[step_idx];
                                RubberBand::Ptr b2 = transition.microstep_band_history_[step_idx + 1];
                                dist_and_foh_values(0, step_idx) = b1->distance(*b2);
                                dist_and_foh_values(1, step_idx) = transition_estimator_->checkFirstOrderHomotopy(*b1, *b2);
                            }
                            int num_foh_changes = 0;
                            for (ssize_t step_idx = 0; step_idx < dist_and_foh_values.cols() - 1; ++step_idx)
                            {
                                if (dist_and_foh_values(1, step_idx) != dist_and_foh_values(1, step_idx + 1))
                                {
                                    ++num_foh_changes;
                                }
                            }

                            std::vector<Visualizer::NamespaceId> marker_ids;
                            const std::string ns_prefix = std::to_string(next_vis_prefix_) + "__";

                            // Remove any existing visualization at this id (if there is one)
                            {
                                dmm::TransitionTestingVisualizationRequest dmmreq;
                                dmmreq.data = std::to_string(next_vis_prefix_);
                                dmm::TransitionTestingVisualizationResponse dmmres;
                                removeVisualizationCallback(dmmreq, dmmres);
                            }

                            // Planned Path
                            {
                                const auto draw_bands = true;
                                const auto path_ids = band_rrt_vis_->visualizePath(path_to_start, ns_prefix + "PLANNED_", 1, draw_bands);

                                const auto gripper_a_last_id = vis_->visualizeCubes(ns_prefix + "PLANNED_" + BandRRT::RRT_PATH_GRIPPER_A_NS, {trajectory.back().first.planned_rubber_band_->getEndpoints().first}, Vector3d(0.005, 0.005, 0.005), Visualizer::Magenta(), 2);
                                const auto gripper_b_last_id = vis_->visualizeCubes(ns_prefix + "PLANNED_" + BandRRT::RRT_PATH_GRIPPER_B_NS, {trajectory.back().first.planned_rubber_band_->getEndpoints().second}, Vector3d(0.005, 0.005, 0.005), Visualizer::Red(), 2);

                                marker_ids.insert(marker_ids.end(), path_ids.begin(), path_ids.end());
                                marker_ids.insert(marker_ids.end(), gripper_a_last_id.begin(), gripper_a_last_id.end());
                                marker_ids.insert(marker_ids.end(), gripper_b_last_id.begin(), gripper_b_last_id.end());
                            }

                            // Actual Path
                            {
                                for (size_t path_idx = 0; path_idx < trajectory.size(); ++path_idx)
                                {
                                    const auto& state = trajectory[path_idx].first;
                                    const auto new_ids = state.rubber_band_->visualize(ns_prefix + "EXECUTED_BAND", Visualizer::Yellow(), Visualizer::Yellow(), (int32_t)(path_idx + 1));
                                    marker_ids.insert(marker_ids.begin(), new_ids.begin(), new_ids.end());
                                }
                            }

                            // Transition under consideration
                            {
                                // Add the first band surface band
                                {
                                    const bool foh = dist_and_foh_values(1, 0);
                                    const auto color = foh ? Visualizer::Green() : Visualizer::Red();
                                    const auto ns = foh ? ns_prefix + "FEATURE_EXECUTED_BAND_SURFACE_FOH_SAME" : ns_prefix + "FEATURE_EXECUTED_BAND_SURFACE_FOH_DIFF";
                                    const auto new_ids = transition.microstep_band_history_.back()->visualize(ns, color, color, (int)(dist_and_foh_values.cols() + 1));
                                    marker_ids.insert(marker_ids.begin(), new_ids.begin(), new_ids.end());
                                }
                                // Add the "middle" band surface bands
                                for (size_t step_idx = 1; step_idx < transition.microstep_band_history_.size() - 1; ++step_idx)
                                {
                                    const auto ratio = (float)(step_idx) / (float)(transition.microstep_band_history_.size() - 1);
                                    const bool foh = (bool)dist_and_foh_values(1, step_idx - 1) && (bool)dist_and_foh_values(1, step_idx);
                                    const auto color = foh
                                            ? InterpolateColor(Visualizer::Green(), Visualizer::Cyan(), ratio)
                                            : InterpolateColor(Visualizer::Red(), Visualizer::Magenta(), ratio);
                                    const auto ns = foh ? ns_prefix + "FEATURE_EXECUTED_BAND_SURFACE_FOH_SAME" : ns_prefix + "FEATURE_EXECUTED_BAND_SURFACE_FOH_DIFF";
                                    const auto new_ids = transition.microstep_band_history_[step_idx]->visualize(ns, color, color, (int)(step_idx + 1));
                                    marker_ids.insert(marker_ids.begin(), new_ids.begin(), new_ids.end());
                                }
                                // Add the last band surface band
                                {
                                    const bool foh = dist_and_foh_values(1, dist_and_foh_values.cols() - 1);
                                    const auto color = foh ? Visualizer::Cyan() : Visualizer::Magenta();
                                    const auto ns = foh ? ns_prefix + "FEATURE_EXECUTED_BAND_SURFACE_FOH_SAME" : ns_prefix + "FEATURE_EXECUTED_BAND_SURFACE_FOH_DIFF";
                                    const auto new_ids = transition.microstep_band_history_.back()->visualize(ns, color, color, (int)(dist_and_foh_values.cols() + 1));
                                    marker_ids.insert(marker_ids.begin(), new_ids.begin(), new_ids.end());
                                }

                                // Add the planned vs executed start and end bands on their own namespaces
                                {
                                    const auto new_ids1 = transition.starting_state_.planned_rubber_band_->visualize(
                                                ns_prefix + "FEATURE_START_PLANNED",
                                                Visualizer::Green(),
                                                Visualizer::Green(),
                                                1);
                                    const auto new_ids2 = transition.starting_state_.rubber_band_->visualize(
                                                ns_prefix + "FEATURE_START_EXECUTED",
                                                Visualizer::Red(),
                                                Visualizer::Red(),
                                                1);
                                    const auto new_ids3 = transition.ending_state_.planned_rubber_band_->visualize(
                                                ns_prefix + "FEATURE_END_PLANNED",
                                                Visualizer::Olive(),
                                                Visualizer::Olive(),
                                                1);
                                    const auto new_ids4 = transition.ending_state_.rubber_band_->visualize(
                                                ns_prefix + "FEATURE_END_EXECUTED",
                                                Visualizer::Orange(),
                                                Visualizer::Orange(),
                                                1);

                                    marker_ids.insert(marker_ids.begin(), new_ids1.begin(), new_ids1.end());
                                    marker_ids.insert(marker_ids.begin(), new_ids2.begin(), new_ids2.end());
                                    marker_ids.insert(marker_ids.begin(), new_ids3.begin(), new_ids3.end());
                                    marker_ids.insert(marker_ids.begin(), new_ids4.begin(), new_ids4.end());
                                }
                            }

//                            res.response = std::to_string(next_vis_prefix_);
                            visid_to_markers_[std::to_string(next_vis_prefix_)] = marker_ids;
//                            ++next_vis_prefix_;

//                            ROS_INFO_STREAM("Added vis id: " << std::to_string(next_vis_prefix_) << " for file " << test_result_file << std::endl
//                                            << "Planned vs executed start FOH:      " << start_foh << std::endl
//                                            << "Planned vs executed start dist:     " << transition.starting_state_.planned_rubber_band_->distance(*transition.starting_state_.rubber_band_) << std::endl
//                                            << "Planned vs executed end FOH:        " << end_foh << std::endl
//                                            << "Planned vs executed end dist:       " << transition.ending_state_.planned_rubber_band_->distance(*transition.ending_state_.rubber_band_) << std::endl
//                                            << "Start vs end dist planned:          " << transition.starting_state_.planned_rubber_band_->distance(*transition.ending_state_.planned_rubber_band_) << std::endl
//                                            << "Start vs end dist executed:         " << transition.starting_state_.rubber_band_->distance(*transition.ending_state_.rubber_band_) << std::endl
//                                            << "Num FOH changes:                    " << num_foh_changes << std::endl
//                                            << "Distance and FOH values along band surface:\n" << dist_and_foh_values.transpose() << std::endl);

                            if (features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA] != std::to_string(0) ||
                                features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA] != std::to_string(0))
                            {
                                assert(features.size() == FEATURE_NAMES.size());
                                ++num_examples_delta_neq_0;
                                ROS_INFO_STREAM_NAMED("features", "Examples with equal number of components: " << num_examples_delta_eq_0 << "   Not equal components: " << num_examples_delta_neq_0);
                                ROS_INFO_STREAM_NAMED("features", test_result_file);
                                ROS_INFO_STREAM_NAMED("features", "  Transition at idx: " << idx << " with distance " << dist_post);
                                for (size_t i = 0; i < FEATURE_NAMES.size(); ++i)
                                {
                                    ROS_INFO_STREAM_NAMED("features", "  " << /* std::left << */ std::setw(48) << FEATURE_NAMES[i] << ": " << features[i]);
                                }
//                                    PressAnyKeyToContinue();
                            }
                            else
                            {
                                ++num_examples_delta_eq_0;
                            }
                        }
                    }

                    // Create the flag file indicating that this file has been parsed, including the day and time stamp
                    Log::Log flag_file(features_complete_flag_file, true);
                    (void)flag_file;
                }
            }
            catch (const std::exception& ex)
            {
                ROS_ERROR_STREAM("Error parsing file idx: " << file_idx << " file: " << test_result_file << ": " << ex.what());
            }
        }
    }

    std::vector<std::string> TransitionTesting::extractFeatures(
            const TransitionEstimation::StateTransition& transition,
            const std::string& parabola_slice_option) const
    {
//        static double time = 0.0;
//        static size_t calls = 0;
//        ++calls;

//        Stopwatch sw;

        const Vector3d mid_point_pre =
                (transition.starting_gripper_positions_.first +
                 transition.starting_gripper_positions_.second) / 2.0;

        const Vector3d mid_point_post =
                (transition.ending_gripper_positions_.first +
                 transition.ending_gripper_positions_.second) / 2.0;

        const Vector3d midpoint_translation =  mid_point_post - mid_point_pre;

        const Vector3d mid_to_gripper_b_pre =
                transition.starting_gripper_positions_.second -
                mid_point_pre;

//        const Vector3d mid_to_gripper_b_post =
//                (transition.ending_gripper_positions_.second -
//                 mid_point_post);

        // Transform the starting gipper positions to "neutral":
        //      Centered on the origin (midpoint of the starting gripper positions)
        //      World-z defines the transformed z-direction
        //      +'ve x pointing towards gripper b
        //          If the grippers are directly in line with the z-axis
        //          then use the (average) direction of motion to define the y-axis
        //              If the direction of motion is directly in line with the z-axis
        //              then use use world x and y to define the rotation
        const Isometry3d origin = [&]
        {
            Vector3d x_axis;
            Vector3d y_axis;
            const Vector3d z_axis = Vector3d::UnitZ();
            if (!mid_to_gripper_b_pre.normalized().isApprox(Vector3d::UnitZ()))
            {
                x_axis = Vector3d(mid_to_gripper_b_pre.x(), mid_to_gripper_b_pre.y(), 0.0).normalized();
                y_axis = z_axis.cross(x_axis).normalized();
            }
            else
            {
                if (!midpoint_translation.normalized().isApprox(Vector3d::UnitZ()))
                {
                    y_axis = Vector3d(midpoint_translation.x(), midpoint_translation.y(), 0.0);
                    x_axis = y_axis.cross(x_axis).normalized();
                }
                else
                {
                    x_axis = Vector3d::UnitX();
                    y_axis = Vector3d::UnitY();
                }
            }

            return Isometry3d((Matrix4d() << x_axis, y_axis, z_axis, mid_point_pre,
                                             0.0,    0.0,    0.0,    1.0).finished());
        }();
        const Isometry3d inv_origin = origin.inverse();

        const Vector3d gripper_a_pre = inv_origin * transition.starting_gripper_positions_.first;
        const Vector3d gripper_b_pre = inv_origin * transition.starting_gripper_positions_.second;
        const Vector3d gripper_a_post = inv_origin * transition.ending_gripper_positions_.first;
        const Vector3d gripper_b_post = inv_origin * transition.ending_gripper_positions_.second;

        const double band_length_pre = transition.starting_state_.planned_rubber_band_->totalLength();
        const double default_band_length_post = transition.ending_state_.planned_rubber_band_->totalLength();

        const double resolution = work_space_grid_.minStepDimension() / 2.0;
        const Visualizer::Ptr vis = disable_visualizations_ ? nullptr : vis_;
        auto collision_grid_pre = ExtractParabolaSlice(
                    *sdf_,
                    resolution,
                    transition.starting_gripper_positions_,
                    transition.starting_state_.planned_rubber_band_,
                    parabola_slice_option,
                    vis);
        auto collision_grid_post = ExtractParabolaSlice(
                    *sdf_,
                    resolution,
                    transition.ending_gripper_positions_,
                    transition.ending_state_.planned_rubber_band_,
                    parabola_slice_option,
                    vis);

        if (!disable_visualizations_)
        {
            auto collision_grid_marker_pre = collision_grid_pre.ExportForDisplay(Visualizer::Red(), Visualizer::Green(), Visualizer::Blue());
            auto collision_grid_marker_post = collision_grid_post.ExportForDisplay(Visualizer::Orange(), Visualizer::Seafoam(), Visualizer::Blue());
//            auto collision_grid_marker_pre = collision_grid_pre.ExportForDisplay(Visualizer::Red(0.2f), Visualizer::Green(0.2f), Visualizer::Blue(0.2f));
//            auto collision_grid_marker_post = collision_grid_post.ExportForDisplay(Visualizer::Orange(0.2f), Visualizer::Seafoam(0.2f), Visualizer::Blue(0.2f));
            collision_grid_marker_pre.ns = "collision_grid_pre";
            collision_grid_marker_post.ns = "collision_grid_post";
            vis_->publish(collision_grid_marker_pre);
            vis_->publish(collision_grid_marker_post);

            vis_->visualizePoints("pre_band_planned", transition.starting_state_.planned_rubber_band_->upsampleBand(), Visualizer::Cyan(0.4f), 1, 0.005);
            vis_->visualizePoints("post_band_planned", transition.ending_state_.planned_rubber_band_->upsampleBand(), Visualizer::Cyan(0.4f), 1, 0.005);
        }

        const auto components_pre = collision_grid_pre.ExtractConnectedComponents();
        const auto components_post = collision_grid_post.ExtractConnectedComponents();
        const int num_connected_components_pre = static_cast<int>(components_pre.size());
        const int num_connected_components_post = static_cast<int>(components_post.size());

        // Figure out how many "occupied" components there are, and how many "empty" components there are (pre)
        int num_free_components_pre = 0;
        int num_occupied_components_pre = 0;
        for (size_t idx = 0; idx < components_pre.size(); ++idx)
        {
            const auto grid_idx = components_pre[idx].at(0);
            const auto occupancy = collision_grid_pre.GetImmutable(grid_idx).first.occupancy;
            if (occupancy < 0.5)
            {
                ++num_free_components_pre;
            }
            else if (occupancy > 0.5)
            {
                ++num_occupied_components_pre;
            }
            else
            {
                assert(false && "Unknown cell in grid");
            }
        }

        // Figure out how many "occupied" components there are, and how many "empty" components there are (post)
        int num_free_components_post = 0;
        int num_occupied_components_post = 0;
        for (size_t idx = 0; idx < components_post.size(); ++idx)
        {
            const auto grid_idx = components_post[idx].at(0);
            const auto occupancy = collision_grid_post.GetImmutable(grid_idx).first.occupancy;
            if (occupancy < 0.5)
            {
                ++num_free_components_post;
            }
            else if (occupancy > 0.5)
            {
                ++num_occupied_components_post;
            }
            else
            {
                assert(false && "Unknown cell in grid");
            }
        }

        std::vector<std::string> features(FEATURES_DUMMY_ITEM, "");

        features[GRIPPER_A_PRE_X] = std::to_string(gripper_a_pre.x());
        features[GRIPPER_A_PRE_Y] = std::to_string(gripper_a_pre.y());
        features[GRIPPER_A_PRE_Z] = std::to_string(gripper_a_pre.z());
        features[GRIPPER_B_PRE_X] = std::to_string(gripper_b_pre.x());
        features[GRIPPER_B_PRE_Y] = std::to_string(gripper_b_pre.y());
        features[GRIPPER_B_PRE_Z] = std::to_string(gripper_b_pre.z());
        features[GRIPPER_A_POST_X] = std::to_string(gripper_a_post.x());
        features[GRIPPER_A_POST_Y] = std::to_string(gripper_a_post.y());
        features[GRIPPER_A_POST_Z] = std::to_string(gripper_a_post.z());
        features[GRIPPER_B_POST_X] = std::to_string(gripper_b_post.x());
        features[GRIPPER_B_POST_Y] = std::to_string(gripper_b_post.y());
        features[GRIPPER_B_POST_Z] = std::to_string(gripper_b_post.z());

        const double dmax = initial_band_->maxSafeLength();
        const double gripper_delta_norm_pre = (transition.starting_gripper_positions_.first - transition.starting_gripper_positions_.second).norm();
        const double gripper_delta_norm_post = (transition.ending_gripper_positions_.first - transition.ending_gripper_positions_.second).norm();
        features[GRIPPER_DELTA_LENGTH_PRE]          = std::to_string(gripper_delta_norm_pre);
        features[GRIPPER_DELTA_LENGTH_POST]         = std::to_string(gripper_delta_norm_post);
        features[GRIPPER_DELTA_LENGTH_RATIO_PRE]    = std::to_string(gripper_delta_norm_pre / dmax);
        features[GRIPPER_DELTA_LENGTH_RATIO_POST]   = std::to_string(gripper_delta_norm_post / dmax);

        features[MAX_BAND_LENGTH]           = std::to_string(dmax);
        features[BAND_LENGTH_PRE]           = std::to_string(band_length_pre);
        features[BAND_LENGTH_POST]          = std::to_string(default_band_length_post);
        features[BAND_LENGTH_RATIO_PRE]     = std::to_string(band_length_pre / dmax);
        features[BAND_LENGTH_RATIO_POST]    = std::to_string(default_band_length_post / dmax);

        const int num_connected_components_delta = num_connected_components_post - num_connected_components_pre;
        features[SLICE_NUM_CONNECTED_COMPONENTS_PRE]        = std::to_string(num_connected_components_pre);
        features[SLICE_NUM_CONNECTED_COMPONENTS_POST]       = std::to_string(num_connected_components_post);
        features[SLICE_NUM_CONNECTED_COMPONENTS_DELTA]      = std::to_string(num_connected_components_delta);
        features[SLICE_NUM_CONNECTED_COMPONENTS_DELTA_SIGN] = std::to_string(Sign(num_connected_components_delta));

        const int num_free_connected_components_delta = num_free_components_post - num_free_components_pre;
        features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_PRE]           = std::to_string(num_free_components_pre);
        features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_POST]          = std::to_string(num_free_components_post);
        features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA]         = std::to_string(num_free_connected_components_delta);
        features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA_SIGN]    = std::to_string(Sign(num_free_connected_components_delta));

        const int num_occupied_connected_components_delta = num_occupied_components_post - num_occupied_components_pre;
        features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_PRE]           = std::to_string(num_occupied_components_pre);
        features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_POST]          = std::to_string(num_occupied_components_post);
        features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA]         = std::to_string(num_occupied_connected_components_delta);
        features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA_SIGN]    = std::to_string(Sign(num_occupied_connected_components_delta));

//        time += sw(READ);
//        std::cerr << "Calls: " << calls
//                  << "    Time: " << time << std::endl;

        return features;
    }

    bool TransitionTesting::addClassificationExampleVisualizationCallback(
            dmm::TransitionTestingVisualizationRequest& req,
            dmm::TransitionTestingVisualizationResponse& res)
    {
        // Separate out the data source and the transition index. Assumed string format is
        // relative/path/to/file/test_identifier__classification_features.csv : 38
        const std::string delimiter = "__classification_features.csv : ";
        const auto pos = req.data.find(delimiter);
        const auto experiment = data_folder_ + "/" + req.data.substr(0, pos);
        const auto traj_idx = std::stoi(req.data.substr(pos + delimiter.size()));
        assert(traj_idx > 0);

        const auto test_result_file =   experiment + "__last_step_test_results.compressed";
        const auto path_to_start_file = experiment + "__path_to_start.compressed";
        const auto trajectory_file =    experiment + "__trajectory.compressed";

        const RRTPath path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);

        // Load the trajectory if possible, otherwise generate it
        const auto trajectory = [&]
        {
            if (!fs::is_regular_file(trajectory_file))
            {
                const dmm::TransitionTestResult test_result = loadTestResult(test_result_file);
                const auto traj = toTrajectory(test_result, path_to_start, experiment.substr(data_folder_.length() + 1)).first;
                transition_estimator_->saveTrajectory(traj, trajectory_file);
                return traj;
            }
            else
            {
                return transition_estimator_->loadTrajectory(trajectory_file);
            }
        }();

        std::vector<Visualizer::NamespaceId> marker_ids;
        const std::string ns_prefix = std::to_string(next_vis_prefix_) + "__";

        // Planned Path
        {
            const auto draw_bands = true;
            const auto path_ids = band_rrt_vis_->visualizePath(path_to_start, ns_prefix + "PLANNED_", 1, draw_bands);

            const auto gripper_a_last_id = vis_->visualizeCubes(ns_prefix + "PLANNED_" + BandRRT::RRT_PATH_GRIPPER_A_NS, {trajectory.back().first.planned_rubber_band_->getEndpoints().first}, Vector3d(0.005, 0.005, 0.005), Visualizer::Magenta(), 2);
            const auto gripper_b_last_id = vis_->visualizeCubes(ns_prefix + "PLANNED_" + BandRRT::RRT_PATH_GRIPPER_B_NS, {trajectory.back().first.planned_rubber_band_->getEndpoints().second}, Vector3d(0.005, 0.005, 0.005), Visualizer::Red(), 2);

            marker_ids.insert(marker_ids.end(), path_ids.begin(), path_ids.end());
            marker_ids.insert(marker_ids.end(), gripper_a_last_id.begin(), gripper_a_last_id.end());
            marker_ids.insert(marker_ids.end(), gripper_b_last_id.begin(), gripper_b_last_id.end());
        }

        // Actual Path
        {
            for (size_t path_idx = 0; path_idx < trajectory.size(); ++path_idx)
            {
                const auto& state = trajectory[path_idx].first;
                const auto new_ids = state.rubber_band_->visualize(ns_prefix + "EXECUTED_BAND", Visualizer::Yellow(), Visualizer::Yellow(), (int32_t)(path_idx + 1));
                marker_ids.insert(marker_ids.begin(), new_ids.begin(), new_ids.end());
            }
        }

        // Specific transition
        const auto& start_state = trajectory[traj_idx - 1].first;
        const auto& end_state = trajectory[traj_idx].first;

        const bool start_foh = transition_estimator_->checkFirstOrderHomotopy(
                    *start_state.planned_rubber_band_,
                    *end_state.rubber_band_);
        const bool end_foh = transition_estimator_->checkFirstOrderHomotopy(
                    *end_state.planned_rubber_band_,
                    *end_state.rubber_band_);
        const auto start_dist = start_state.planned_rubber_band_->distance(*start_state.rubber_band_);
        const auto end_dist = end_state.planned_rubber_band_->distance(*end_state.rubber_band_);

        std::vector<RubberBand::Ptr> microstep_band_history = transition_estimator_->reduceMicrostepsToBands(trajectory[traj_idx].second);

        const TransitionEstimation::StateTransition transition
        {
            start_state,
            end_state,
            start_state.planned_rubber_band_->getEndpoints(),
            end_state.planned_rubber_band_->getEndpoints(),
            trajectory[traj_idx].second,
            microstep_band_history
        };
        static const auto parabola_slice_option = ROSHelpers::GetParam<std::string>(*ph_, "parabola_slice_option", "basic");
        const auto features = extractFeatures(transition, parabola_slice_option);
        const bool mistake = (start_foh && !end_foh) && (end_dist > mistake_dist_thresh_);
        const bool predicted_mistake = transition_mistake_classifier_->predict(
                    transition_estimator_->transitionFeatures(
                        *start_state.planned_rubber_band_, *end_state.planned_rubber_band_, false));

        res.response = std::to_string(next_vis_prefix_);
        visid_to_markers_[res.response] = marker_ids;
        ++next_vis_prefix_;

        ROS_INFO_STREAM("Added vis id: " << res.response << " for file " << req.data << std::endl
                        << "Planned vs executed start FOH:              " << start_foh << std::endl
                        << "Planned vs executed start dist:             " << start_dist << std::endl
                        << "Planned vs executed end FOH:                " << end_foh << std::endl
                        << "Planned vs executed end dist:               " << end_dist << std::endl
                        << "Mistake:                                    " << std::boolalpha <<  mistake << std::endl
                        << "Predicted mistake:                          " << std::boolalpha <<  predicted_mistake << std::endl
//                        << "Gripper a transformed start position:       " << features[GRIPPER_A_PRE_X] << ", " << features[GRIPPER_A_PRE_Y] << ", " << features[GRIPPER_A_PRE_Z] << std::endl
//                        << "Gripper b transformed start position:       " << features[GRIPPER_B_PRE_X] << ", " << features[GRIPPER_B_PRE_Y] << ", " << features[GRIPPER_B_PRE_Z] << std::endl
//                        << "Gripper a transformed end position:         " << features[GRIPPER_A_POST_X] << ", " << features[GRIPPER_A_POST_Y] << ", " << features[GRIPPER_A_POST_Z] << std::endl
//                        << "Gripper b transformed end position:         " << features[GRIPPER_B_POST_X] << ", " << features[GRIPPER_B_POST_Y] << ", " << features[GRIPPER_B_POST_Z] << std::endl
                        << "Gripper seperation pre:                     " << features[GRIPPER_DELTA_LENGTH_PRE] << std::endl
                        << "Gripper seperation post:                    " << features[GRIPPER_DELTA_LENGTH_POST] << std::endl
                        << "Gripper seperation ratio pre:               " << features[GRIPPER_DELTA_LENGTH_RATIO_PRE] << std::endl
                        << "Gripper seperation ratio post:              " << features[GRIPPER_DELTA_LENGTH_RATIO_POST] << std::endl
//                        << "Band length max:                            " << features[MAX_BAND_LENGTH] << std::endl
                        << "Band length pre:                            " << features[BAND_LENGTH_PRE] << std::endl
                        << "Band length post:                           " << features[BAND_LENGTH_POST] << std::endl
                        << "Band length ratio pre:                      " << features[BAND_LENGTH_RATIO_PRE] << std::endl
                        << "Band length ratio post:                     " << features[BAND_LENGTH_RATIO_POST] << std::endl
                        << "Num connected components pre:               " << features[SLICE_NUM_CONNECTED_COMPONENTS_PRE] << std::endl
                        << "Num connected components post:              " << features[SLICE_NUM_CONNECTED_COMPONENTS_POST] << std::endl
                        << "Num connected components delta:             " << features[SLICE_NUM_CONNECTED_COMPONENTS_DELTA] << std::endl
                        << "Num connected components delta sign:        " << features[SLICE_NUM_CONNECTED_COMPONENTS_DELTA_SIGN] << std::endl
                        << "Num free connected components pre:          " << features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_PRE] << std::endl
                        << "Num free connected components post:         " << features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_POST] << std::endl
                        << "Num free connected components delta:        " << features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA] << std::endl
                        << "Num free connected components delta sign:   " << features[SLICE_NUM_FREE_CONNECTED_COMPONENTS_DELTA_SIGN] << std::endl
                        << "Num filled connected components pre:        " << features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_PRE] << std::endl
                        << "Num filled connected components post:       " << features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_POST] << std::endl
                        << "Num filled connected components delta:      " << features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA] << std::endl
                        << "Num filled connected components delta sign: " << features[SLICE_NUM_OCCUPIED_CONNECTED_COMPONENTS_DELTA_SIGN] << std::endl);

        return true;
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Testing Classifiers
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    void TransitionTesting::testClassifier()
    {
        const auto target_grippers_poses = getGripperTargets();

        const auto classifier_type = ROSHelpers::GetParamRequired<std::string>(*ph_, "classifier/type", __func__);
        const auto folder = data_folder_ + "/" + classifier_type + "_classifier_" + IntToHex(seed_) + "/";
        arc_utilities::CreateDirectory(folder);

        const auto num_threads = GetNumOMPThreads();
        std::vector<dmm::TransitionTest> tests;
        std::vector<std::string> test_result_filenames;
        tests.reserve(num_threads);
        test_result_filenames.reserve(num_threads);




        assert(false && "Replace this code with stuff from TaskFramework()");





        const auto num_trials = GetRRTNumTrials(*ph_);
        std::atomic<int> num_failed_plans = 0;
        const auto omp_planning_threads = std::max(1, GetNumOMPThreads() / 2);
        #pragma omp parallel for num_threads(omp_planning_threads)
        for (size_t trial_idx = 0; trial_idx < num_trials; ++trial_idx)
        {
            try
            {
                const auto path_to_start_file = folder + "trial_idx_" + ToStrFill0(num_trials, trial_idx) + "__path_to_start.compressed";
                const auto test_result_file = folder + "trial_idx_" + ToStrFill0(num_trials, trial_idx) + "__last_step_test_results.compressed";

                const auto rrt_path = generateTestPath(target_grippers_poses, trial_idx * 0xFFFF);
                BandRRT::SavePath(rrt_path, path_to_start_file);

                const auto test = robot_->toRosTransitionTest(
                            initial_world_state_.rope_node_transforms_,
                            initial_world_state_.all_grippers_single_pose_,
                            RRTPathToGrippersPoseTrajectory(rrt_path),
                            ToGripperPoseVector(rrt_path.back().grippers()));

                // Add the test to the list waiting to be executed
                #pragma omp critical
                {
                    tests.push_back(test);
                    test_result_filenames.push_back(test_result_file);

                    // Execute the tests if there are enough to run
                    if (tests.size() > 100)
                    {
                        assert(false && "Replace this call with a call to robot_->testRobotPaths(...)");
                        robot_->generateTransitionData(tests, test_result_filenames, nullptr, false);
                        tests.clear();
                        test_result_filenames.clear();
                    }
                }
            }
            catch (const std::runtime_error& ex)
            {
                ROS_WARN_STREAM("Planning failed for idx " << trial_idx << ": " << ex.what());
                ++num_failed_plans;
            }
        }

        // Run any last tests that are left over
        if (tests.size() != 0)
        {
            assert(false && "Replace this call with a call to robot_->testRobotPaths(...)");
            robot_->generateTransitionData(tests, test_result_filenames, nullptr, false);
            tests.clear();
            test_result_filenames.clear();
        }

        ROS_INFO("Parsing generated trajectories");

        // We can't rely on ROS messaging to get all feedback, so post-process everything instead
        std::atomic<int> num_succesful_paths = 0;
        std::atomic<int> num_unsuccesful_paths = 0;
        std::atomic<int> num_unparsable_paths = 0;
        const auto omp_parsing_threads = std::max(1, (deformable_type_ == ROPE) ? arc_helpers::GetNumOMPThreads() / 2 : 1);
        #pragma omp parallel for num_threads(omp_parsing_threads)
        for (size_t trial_idx = 0; trial_idx < num_trials; ++trial_idx)
        {
            try
            {
                const auto path_to_start_file = folder + "trial_idx_" + ToStrFill0(num_trials, trial_idx) + "__path_to_start.compressed";
                const auto test_result_file = folder + "trial_idx_" + ToStrFill0(num_trials, trial_idx) + "__last_step_test_results.compressed";
                const auto trajectory_file = folder + "trial_idx_" + ToStrFill0(num_trials, trial_idx) + "__trajectory.compressed";

                const auto path_to_start = BandRRT::LoadPath(path_to_start_file, *initial_band_);
                const auto test_result = loadTestResult(test_result_file);

                const auto traj_gen_result = toTrajectory(test_result, path_to_start, test_result_file);
                const auto& trajectory = traj_gen_result.first;
                transition_estimator_->saveTrajectory(trajectory, trajectory_file);
                if (traj_gen_result.second)
                {
                    ++num_succesful_paths;
                }
                else
                {
                    ++num_unsuccesful_paths;
                }
            }
            catch (const std::exception& ex)
            {
                ROS_WARN_STREAM("Unable to parse idx " << trial_idx << ": " << ex.what());
                ++num_unparsable_paths;
            }
        }

        const int classifier_dim = ROSHelpers::GetParamRequiredDebugLog<int>(*ph_, "classifier/dim", __func__);
        const std::string classifier_slice_type = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "classifier/slice_type", __func__);
        ROS_INFO_STREAM(classifier_dim << " " <<
                        classifier_slice_type << " " <<
                        classifier_type << " " <<
                        "Total successful paths: " << num_succesful_paths <<
                        "    Total unsuccessful paths: " << num_unsuccesful_paths <<
                        "    Total plan failures: " << num_failed_plans <<
                        "    Total unparsable paths: " << num_unparsable_paths);
    }

    // Duplicated from task_framework.cpp
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

    // Duplicated from task_framework.cpp and modified slightly
    AllGrippersSinglePose TransitionTesting::getGripperTargets()
    {
        const auto& correspondences = task_->getCoverPointCorrespondences(initial_world_state_);
        const auto& cover_point_indices_= correspondences.uncovered_target_points_idxs_;

        // Only cluster the points that are not covered
        VectorVector3d cluster_targets;
        cluster_targets.reserve(cover_point_indices_.size());
        for (size_t idx = 0; idx < cover_point_indices_.size(); ++idx)
        {
            const ssize_t cover_idx = cover_point_indices_[idx];
            cluster_targets.push_back(task_->cover_points_.col(cover_idx));
        }

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

        // Get the orientations for each gripper based on their starting orientation
        AllGrippersSinglePose target_gripper_poses = initial_world_state_.all_grippers_single_pose_;

        // Decide which gripper goes to which cluster
        {
            const auto& gripper0_grapsed_points = robot_->getGrippersData().at(0).node_indices_;
            const auto& gripper1_grapsed_points = robot_->getGrippersData().at(1).node_indices_;

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
                        task_->averageDijkstrasDistanceBetweenGrippersAndClusters(
                            initial_world_state_.all_grippers_single_pose_[0],
                            correspondences.uncovered_target_points_idxs_,
                            cluster_labels,
                            num_clusters);
                const std::vector<double> gripper1_distances_to_clusters =
                        task_->averageDijkstrasDistanceBetweenGrippersAndClusters(
                            initial_world_state_.all_grippers_single_pose_[1],
                            correspondences.uncovered_target_points_idxs_,
                            cluster_labels,
                            num_clusters);

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
        }

        // Project the targets out of collision
        const double min_dist_to_obstacles = std::max(GetControllerMinDistanceToObstacles(*ph_), GetRRTMinGripperDistanceToObstacles(*ph_)) * GetRRTTargetMinDistanceScaleFactor(*ph_);
        const auto gripper_positions_pre_project = ToGripperPositions(target_gripper_poses);
        target_gripper_poses[0].translation() = sdf_->ProjectOutOfCollisionToMinimumDistance3d(gripper_positions_pre_project.first, min_dist_to_obstacles);
        target_gripper_poses[1].translation() = sdf_->ProjectOutOfCollisionToMinimumDistance3d(gripper_positions_pre_project.second, min_dist_to_obstacles);

        return target_gripper_poses;
    }
}

////////////////////////////////////////////////////////////////////////////////
//          Generic Visualization
////////////////////////////////////////////////////////////////////////////////

namespace smmap
{
    void TransitionTesting::setNextVisId(const std_msgs::Int32& msg)
    {
        next_vis_prefix_ = msg.data;
        ROS_INFO_STREAM("Next vis id: " << next_vis_prefix_);
    }

    bool TransitionTesting::removeVisualizationCallback(
            dmm::TransitionTestingVisualizationRequest& req,
            dmm::TransitionTestingVisualizationResponse& res)
    {
        (void)res;
        try
        {
            const auto markers_nsid = visid_to_markers_.at(req.data);
            for (const auto& nsid : markers_nsid)
            {
                vis_->deleteObject(nsid.first, nsid.second);
            }
            visid_to_markers_.erase(req.data);
            ROS_DEBUG_STREAM("Removed vis id: " << req.data);
            return true;
        }
        catch (...)
        {
            res.response = "Invalid vis id";
            return false;
        }
    }

    std::vector<Visualizer::NamespaceId> TransitionTesting::visualizePathAndTrajectory(
            const RRTPath& path,
            const TransitionEstimation::StateTrajectory& trajectory,
            const std::string ns_prefix) const
    {
        std::vector<Visualizer::NamespaceId> marker_ids;

        // Planned Path
        {
            const auto draw_bands = true;
            const auto path_ids = band_rrt_vis_->visualizePath(path, ns_prefix + "PLANNED_", 1, draw_bands);

            const auto gripper_a_last_id = vis_->visualizeCubes(ns_prefix + "PLANNED_" + BandRRT::RRT_PATH_GRIPPER_A_NS, {trajectory.back().first.planned_rubber_band_->getEndpoints().first}, Vector3d(0.005, 0.005, 0.005), Visualizer::Magenta(), 2);
            const auto gripper_b_last_id = vis_->visualizeCubes(ns_prefix + "PLANNED_" + BandRRT::RRT_PATH_GRIPPER_B_NS, {trajectory.back().first.planned_rubber_band_->getEndpoints().second}, Vector3d(0.005, 0.005, 0.005), Visualizer::Red(), 2);

            marker_ids.insert(marker_ids.end(), path_ids.begin(), path_ids.end());
            marker_ids.insert(marker_ids.end(), gripper_a_last_id.begin(), gripper_a_last_id.end());
            marker_ids.insert(marker_ids.end(), gripper_b_last_id.begin(), gripper_b_last_id.end());
        }

        // Actual Path
        {
            for (size_t path_idx = 0; path_idx < trajectory.size(); ++path_idx)
            {
                const auto& state = trajectory[path_idx].first;
                const auto band_ids = state.rubber_band_->visualize(ns_prefix + "EXECUTED_BAND", Visualizer::Yellow(), Visualizer::Yellow(), (int32_t)(path_idx + 1));
                const auto deform_ids = task_->visualizeDeformableObject(ns_prefix + "EXECUTED_DEFORMABLE", state.deform_config_, Visualizer::Green(), (int32_t)(path_idx + 1));
                marker_ids.insert(marker_ids.begin(), band_ids.begin(), band_ids.end());
                marker_ids.insert(marker_ids.begin(), deform_ids.begin(), deform_ids.end());
            }
        }
        return marker_ids;
    }
}
