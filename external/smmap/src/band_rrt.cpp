#include "smmap/band_rrt.h"
#include "smmap/task_framework.h"

#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/timing.hpp>
#include <arc_utilities/filesystem.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/path_utils.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <deformable_manipulation_experiment_params/utility.hpp>

using namespace smmap;
using namespace arc_utilities;
using namespace arc_helpers;
using namespace Eigen;
using namespace EigenHelpers;

//#define SMMAP_RRT_VERBOSE true
#define SMMAP_RRT_VERBOSE false


RRTRobotRepresentation RRTDistance::joint_weights_;

constexpr char BandRRT::RRT_BLACKLISTED_GOAL_BANDS_NS[];
constexpr char BandRRT::RRT_GOAL_TESTING_NS[];

constexpr char BandRRT::RRT_FORWARD_TREE_GRIPPER_A_NS[];
constexpr char BandRRT::RRT_FORWARD_TREE_GRIPPER_B_NS[];
constexpr char BandRRT::RRT_BACKWARD_TREE_GRIPPER_A_NS[];
constexpr char BandRRT::RRT_BACKWARD_TREE_GRIPPER_B_NS[];
constexpr char BandRRT::RRT_TREE_BAND_NS[];

constexpr char BandRRT::RRT_SAMPLE_NS[];
constexpr char BandRRT::RRT_FORWARD_PROP_START_NS[];

constexpr char BandRRT::RRT_PATH_GRIPPER_A_NS[];
constexpr char BandRRT::RRT_PATH_GRIPPER_B_NS[];
constexpr char BandRRT::RRT_PATH_RUBBER_BAND_NS[];

constexpr char BandRRT::RRT_SMOOTHING_GRIPPER_A_NS[];
constexpr char BandRRT::RRT_SMOOTHING_GRIPPER_B_NS[];

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper function for assertion testing
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline bool gripperPositionsAreApproximatelyEqual(
        const RRTGrippersRepresentation& c1,
        const RRTGrippersRepresentation& c2)
{
    const auto& c1_first_gripper     = c1.first.translation();
    const auto& c1_second_gripper    = c1.second.translation();
    const auto& c2_first_gripper     = c2.first.translation();
    const auto& c2_second_gripper    = c2.second.translation();

    bool is_equal = true;
    is_equal &= c1_first_gripper.isApprox(c2_first_gripper, BandRRT::GRIPPER_TRANSLATION_IS_APPROX_DIST);
    is_equal &= c1_second_gripper.isApprox(c2_second_gripper, BandRRT::GRIPPER_TRANSLATION_IS_APPROX_DIST);
    return is_equal;
}

static inline bool robotConfigurationsAreApproximatelyEqual(
        const RRTRobotRepresentation& r1,
        const RRTRobotRepresentation& r2)
{
    return r1.isApprox(r2);
}

static inline bool bandEndpointsMatchGripperPositions(
        const RubberBand& band,
        const RRTGrippersRepresentation& grippers)
{
    RRTGrippersRepresentation test_representation = grippers;
    test_representation.first.translation() = band.getVectorRepresentation().front();
    test_representation.second.translation() = band.getVectorRepresentation().back();
    return gripperPositionsAreApproximatelyEqual(grippers, test_representation);
}

static inline bool maxGrippersDistanceViolated(
        const Vector3d& gripper_a_pos,
        const Vector3d& gripper_b_pos,
        const double max_dist)
{
    return (gripper_a_pos - gripper_b_pos).squaredNorm() > (max_dist * max_dist);
}

static inline bool maxGrippersDistanceViolated(
        const RRTGrippersRepresentation& grippers,
        const double max_dist)
{
    const auto& gripper_a_pos = grippers.first.translation();
    const auto& gripper_b_pos = grippers.second.translation();
    return (gripper_a_pos - gripper_b_pos).squaredNorm() > (max_dist * max_dist);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////           Conversion functions                   ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AllGrippersPoseTrajectory smmap::RRTPathToGrippersPoseTrajectory(const RRTPath& path)
{
    AllGrippersPoseTrajectory traj;
    traj.reserve(path.size());
    for (const auto& node : path)
    {
        const auto grippers = node.grippers();
        traj.push_back({grippers.first, grippers.second});
    }
    return traj;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////           RRTNode functions                      ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RRTNode::RRTNode()
    : initialized_(false)
{}

RRTNode::RRTNode(
        const RRTGrippersRepresentation& grippers_poses,
        const RRTRobotRepresentation& robot_configuration,
        const RubberBand::Ptr& band)
    : grippers_poses_(grippers_poses)
    , robot_configuration_(robot_configuration)
    , band_(band)
    , cost_to_come_(0.0)
    , p_reachability_(-0.0)
    , p_transition_(-0.0)
    , p_goal_reachable_(-0.0)
    , parent_index_(-1)
    , state_index_(0)
    , transition_index_(-1)
    , split_index_(-1)
    , child_indices_(0)
    , initialized_(false)
    , already_extended_towards_goal_set_(false)
    , blacklisted_from_nn_search_(false)
{}

RRTNode::RRTNode(
        const RRTGrippersRepresentation& grippers_poses,
        const RRTRobotRepresentation& robot_configuration,
        const RubberBand::Ptr& band,
        const double cost_to_come,
        const double p_reachability,
        const double p_transition,
        const int64_t parent_index,
        const int64_t state_index,
        const int64_t transition_index,
        const int64_t split_index)
    : grippers_poses_(grippers_poses)
    , robot_configuration_(robot_configuration)
    , band_(band)
    , cost_to_come_(cost_to_come)
    , p_reachability_(p_reachability)
    , p_transition_(p_transition)
    , p_goal_reachable_(0.0)
    , parent_index_(parent_index)
    , state_index_(state_index)
    , transition_index_(transition_index)
    , split_index_(split_index)
    , child_indices_(0)
    , initialized_(true)
    , already_extended_towards_goal_set_(false)
    , blacklisted_from_nn_search_(false)
{}

RRTNode::RRTNode(
        const RRTGrippersRepresentation& grippers_poses,
        const RRTRobotRepresentation& robot_configuration,
        const QuinlanRubberBand::Ptr& band,
        const double cost_to_come,
        const double p_reachability,
        const double p_transition,
        const double p_goal_reachable,
        const int64_t parent_index,
        const int64_t state_index,
        const int64_t transition_index,
        const int64_t split_index,
        const std::vector<int64_t>& child_indices,
        const bool initialized,
        const bool already_extended_towards_backwards_tree,
        const bool blacklisted_from_nn_search)
    : grippers_poses_(grippers_poses)
    , robot_configuration_(robot_configuration)
    , band_(band)
    , cost_to_come_(cost_to_come)
    , p_reachability_(p_reachability)
    , p_transition_(p_transition)
    , p_goal_reachable_(p_goal_reachable)
    , parent_index_(parent_index)
    , state_index_(state_index)
    , transition_index_(transition_index)
    , split_index_(split_index)
    , child_indices_(child_indices)
    , initialized_(initialized)
    , already_extended_towards_goal_set_(already_extended_towards_backwards_tree)
    , blacklisted_from_nn_search_(blacklisted_from_nn_search)
{}

bool RRTNode::initialized() const
{
    return initialized_;
}

const RRTGrippersRepresentation& RRTNode::grippers() const
{
    return grippers_poses_;
}

const RRTRobotRepresentation& RRTNode::robotConfiguration() const
{
    return robot_configuration_;
}

const RubberBand::Ptr& RRTNode::band() const
{
    return band_;
}

double RRTNode::costToCome() const
{
    return cost_to_come_;
}


double RRTNode::pTransition() const
{
    return p_transition_;
}

double RRTNode::pReachability() const
{
    return p_reachability_;
}

double RRTNode::getpGoalReachable() const
{
    return p_goal_reachable_;
}

void RRTNode::setpGoalReachable(const double p_goal_reachable)
{
    // Do some error checking to make sure things are still sane before recording the value
    if ((p_goal_reachable < 0.0) || (p_goal_reachable > 1.0))
    {
        throw_arc_exception(std::runtime_error, "p_goal_reachable out of range [0, 1]");
    }
    p_goal_reachable_ = p_goal_reachable;
}


int64_t RRTNode::parentIndex() const
{
    return parent_index_;
}

int64_t RRTNode::stateIndex() const
{
    return state_index_;
}

int64_t RRTNode::transitionIndex() const
{
    return transition_index_;
}

int64_t RRTNode::splitIndex() const
{
    return split_index_;
}


const std::vector<int64_t>& RRTNode::childIndices() const
{
    return child_indices_;
}

void RRTNode::clearChildIndices()
{
    child_indices_.clear();
}

void RRTNode::addChildIndex(const int64_t child_index)
{
    for (size_t idx = 0; idx < child_indices_.size(); idx++)
    {
        if (child_indices_[idx] == child_index)
        {
            return;
        }
    }
    child_indices_.push_back(child_index);
}

void RRTNode::removeChildIndex(const int64_t child_index)
{
    std::vector<int64_t> new_child_indices;
    for (size_t idx = 0; idx < child_indices_.size(); idx++)
    {
        if (child_indices_[idx] != child_index)
        {
            new_child_indices.push_back(child_indices_[idx]);
        }
    }
    child_indices_ = new_child_indices;
}


std::string RRTNode::print() const
{
    std::stringstream out;
    out << "Initialized:            " << initialized_ << std::endl
        << "Parent index:           " << parent_index_ << std::endl
        << "State index:            " << state_index_ << std::endl
        << "Transition index:       " << transition_index_ << std::endl
        << "Split index:            " << split_index_ << std::endl
        << "Child indices:          " << PrettyPrint::PrettyPrint(child_indices_, false, ", ") << std::endl
        << "Already extended:       " << already_extended_towards_goal_set_ << std::endl
        << "Blacklisted from NN:    " << blacklisted_from_nn_search_ << std::endl
        << "Robot configuration:    " << robot_configuration_.transpose() << std::endl
        << "  p_reachablity_:       " << p_reachability_ << std::endl
        << "  p_transition_:        " << p_transition_ << std::endl
        << "  p_goal_reachable_:    " << p_goal_reachable_ << std::endl
        << std::endl;
    return out.str();
}

/**
 * @brief RRTNode::operator ==
 * @param other
 * @return Uses approximate equality tests for the gripper config, robot config, and band config;
 *         exact equality for all other fields
 */
bool RRTNode::operator==(const RRTNode& other) const
{
    if (!gripperPositionsAreApproximatelyEqual(grippers_poses_, other.grippers_poses_))
    {
        return false;
    }

    if (!robotConfigurationsAreApproximatelyEqual(robot_configuration_, other.robot_configuration_))
    {
        return false;
    }

    const auto& this_band_as_vector = band_->getVectorRepresentation();
    const auto& other_band_as_vector = other.band_->getVectorRepresentation();
    if (this_band_as_vector.size() != other_band_as_vector.size())
    {
        return false;
    }

    for (size_t idx = 0; idx < this_band_as_vector.size(); ++idx)
    {
        if (!this_band_as_vector[idx].isApprox(other_band_as_vector[idx]))
        {
            return false;
        }
    }

    if (cost_to_come_ != other.cost_to_come_)
    {
        return false;
    }

    if (p_reachability_ != other.p_reachability_)
    {
        return false;
    }

    if (p_transition_ != other.p_transition_)
    {
        return false;
    }

    if (p_goal_reachable_ != other.p_goal_reachable_)
    {
        return false;
    }

    if (parent_index_ != other.parent_index_)
    {
        return false;
    }

    if (state_index_ != other.state_index_)
    {
        return false;
    }

    if (transition_index_ != other.transition_index_)
    {
        return false;
    }

    if (split_index_ != other.split_index_)
    {
        return false;
    }

    if (child_indices_ != other.child_indices_)
    {
        return false;
    }

    if (initialized_ != other.initialized_)
    {
        return false;
    }

    if (already_extended_towards_goal_set_ != other.already_extended_towards_goal_set_)
    {
        return false;
    }

    if (blacklisted_from_nn_search_ != other.blacklisted_from_nn_search_)
    {
        return false;
    }

    return true;
}

uint64_t RRTNode::serialize(std::vector<uint8_t>& buffer) const
{
    const uint64_t starting_bytes = buffer.size();

    arc_utilities::SerializePair<Isometry3d, Isometry3d>(grippers_poses_, buffer, &arc_utilities::SerializeEigen<Isometry3d>, &arc_utilities::SerializeEigen<Isometry3d>);
    arc_utilities::SerializeEigen<double, Dynamic, 1>(robot_configuration_, buffer);
    band_->serialize(buffer);
    arc_utilities::SerializeFixedSizePOD<double>(cost_to_come_, buffer);
    arc_utilities::SerializeFixedSizePOD<double>(p_reachability_, buffer);
    arc_utilities::SerializeFixedSizePOD<double>(p_transition_, buffer);
    arc_utilities::SerializeFixedSizePOD<double>(p_goal_reachable_, buffer);
    arc_utilities::SerializeFixedSizePOD<int64_t>(parent_index_, buffer);
    arc_utilities::SerializeFixedSizePOD<int64_t>(state_index_, buffer);
    arc_utilities::SerializeFixedSizePOD<int64_t>(transition_index_, buffer);
    arc_utilities::SerializeFixedSizePOD<int64_t>(split_index_, buffer);
    arc_utilities::SerializeVector<int64_t>(child_indices_, buffer, arc_utilities::SerializeFixedSizePOD<int64_t>);
    arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)initialized_, buffer);
    arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)already_extended_towards_goal_set_, buffer);
    arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)blacklisted_from_nn_search_, buffer);

    const uint64_t ending_bytes = buffer.size();

    // Verify the result
    const auto deserialized = Deserialize(buffer, starting_bytes, *band_);
    assert(ending_bytes - starting_bytes == deserialized.second);
    assert(*this == deserialized.first);

    return ending_bytes - starting_bytes;
}

uint64_t RRTNode::Serialize(const RRTNode& config, std::vector<uint8_t>& buffer)
{
    return config.serialize(buffer);
}

std::pair<RRTNode, uint64_t> RRTNode::Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current, const RubberBand& starting_band)
{
    assert(current < buffer.size());
    uint64_t current_position = current;

    // Deserialize the grippers poses
    const auto grippers_poses_deserialized = arc_utilities::DeserializePair<Isometry3d, Isometry3d>(
                buffer, current_position, &arc_utilities::DeserializeEigen<Isometry3d>, &arc_utilities::DeserializeEigen<Isometry3d>);
    current_position += grippers_poses_deserialized.second;

    // Deserialize the robot configuration
    const auto robot_configuration_deserialized = arc_utilities::DeserializeEigen<VectorXd>(buffer, current_position);
    current_position += robot_configuration_deserialized.second;

    // Deserialize the rubber band
    auto band = std::make_shared<RubberBand>(starting_band);
    current_position += band->deserialize(buffer, current_position);

    // Deserialize the cost to come
    const auto cost_to_come_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
    current_position += cost_to_come_deserialized.second;

    // Deserialize the probability of being reachable from the root
    const auto p_reachablity_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
    current_position += p_reachablity_deserialized.second;

    // Deserialize the transition probability
    const auto p_transition_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
    current_position += p_transition_deserialized.second;

    // Deserialize the probability that the goal is reachable from this node
    const auto p_goal_reachable_deserialized = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
    current_position += p_goal_reachable_deserialized.second;

    // Deserialize the parent index
    const auto parent_index_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
    current_position += parent_index_deserialized.second;

    // Deserialize the state index
    const auto state_index_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
    current_position += state_index_deserialized.second;

    // Deserialize the transition index
    const auto transition_index_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
    current_position += transition_index_deserialized.second;

    // Deserialize the split index
    const auto split_index_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
    current_position += split_index_deserialized.second;

    // Deserialize the child indices
    const auto child_indices_deserialized = arc_utilities::DeserializeVector<int64_t>(buffer, current_position, &arc_utilities::DeserializeFixedSizePOD<int64_t>);
    current_position += child_indices_deserialized.second;

    // Deserialize the initialized
    const auto initialized_deserialized = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer, current_position);
    current_position += initialized_deserialized.second;

    // Deserialize the already_extended_towards_goal_set
    const auto already_extended_towards_goal_set_deserialized = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer, current_position);
    current_position += already_extended_towards_goal_set_deserialized.second;

    // Deserialize the blacklisted_from_nn_search
    const auto blacklisted_from_nn_search_deserialized = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer, current_position);
    current_position += blacklisted_from_nn_search_deserialized.second;

    // Build the resulting node
    RRTNode deserialized(
        grippers_poses_deserialized.first,
        robot_configuration_deserialized.first,
        band,
        cost_to_come_deserialized.first,
        p_reachablity_deserialized.first,
        p_transition_deserialized.first,
        p_goal_reachable_deserialized.first,
        parent_index_deserialized.first,
        state_index_deserialized.first,
        transition_index_deserialized.first,
        split_index_deserialized.first,
        child_indices_deserialized.first,
        (bool)initialized_deserialized.first,
        (bool)already_extended_towards_goal_set_deserialized.first,
        (bool)blacklisted_from_nn_search_deserialized.first);

    const uint64_t bytes_read = current_position - current;
    return std::make_pair(deserialized, bytes_read);
}

std::ostream& smmap::operator<<(std::ostream& out, const RRTNode& node)
{
    return out << node.print();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////           RRTDistance functions                    /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const RRTRobotRepresentation& RRTDistance::GetJointWeights()
{
    return joint_weights_;
}

void RRTDistance::SetJointWeights(const RRTRobotRepresentation joint_weights)
{
    joint_weights_ = joint_weights;
}

// Note that this does only translational distance
double RRTDistance::DistanceSquared(const RRTGrippersRepresentation& c1, const RRTGrippersRepresentation& c2)
{
    const auto& c1_first_gripper     = c1.first.translation();
    const auto& c1_second_gripper    = c1.second.translation();
    const auto& c2_first_gripper     = c2.first.translation();
    const auto& c2_second_gripper    = c2.second.translation();
    return (c1_first_gripper - c2_first_gripper).squaredNorm() +
            (c1_second_gripper - c2_second_gripper).squaredNorm();
}

double RRTDistance::Distance(const RRTGrippersRepresentation& c1, const RRTGrippersRepresentation& c2)
{
    return std::sqrt(DistanceSquared(c1, c2));
}

double RRTDistance::DistanceSquared(const RRTRobotRepresentation& r1, const RRTRobotRepresentation& r2)
{
    return ((r1 - r2).cwiseProduct(joint_weights_)).squaredNorm();
}

double RRTDistance::Distance(const RRTRobotRepresentation& r1, const RRTRobotRepresentation& r2)
{
    return std::sqrt(DistanceSquared(r1, r2));
}

// Only calculates the distance travelled by the grippers, not the entire band
double RRTDistance::GrippersPathDistance(const RRTTree& path, const size_t start_index, const size_t end_index)
{
    assert(start_index < end_index);
    assert(end_index < path.size());
    double path_distance = 0;
    for (size_t idx = start_index; idx < end_index; ++idx)
    {
        path_distance += Distance(path[idx].grippers(), path[idx + 1].grippers());
    }
    return path_distance;
}

double RRTDistance::RobotPathDistance(const RRTTree& path, const size_t start_index, const size_t end_index)
{
    assert(start_index < end_index);
    assert(end_index < path.size());
    double path_distance = 0;
    for (size_t idx = start_index; idx < end_index; ++idx)
    {
        path_distance += Distance(path[idx].robotConfiguration(), path[idx + 1].robotConfiguration());
    }
    return path_distance;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////           RRTHelper functions                      /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BandRRT::BandRRT(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        const WorldParams& world_params,
        const PlanningParams& planning_params,
        const SmoothingParams& smoothing_params,
        const TaskParams& task_params,
        const RubberBand::ConstPtr& template_band,
        Visualizer::Ptr vis,
        const bool visualization_enabled)
    : nh_(nh)
    , ph_(std::make_shared<ros::NodeHandle>(ph->getNamespace() + "/rrt"))
    , robot_(world_params.robot_)
    , planning_for_whole_robot_(world_params.planning_for_whole_robot_)
    , sdf_(world_params.sdf_)
    , work_space_grid_(world_params.work_space_grid_)
    , transition_estimator_(world_params.transition_estimator_)
    , template_band_(*template_band)

    , generator_(world_params.generator_)
    , uniform_unit_distribution_(0.0, 1.0)

    , task_aligned_frame_transform_(task_params.task_aligned_frame_transform_)
    , task_aligned_frame_inverse_transform_(task_aligned_frame_transform_.inverse())
    , task_aligned_lower_limits_(task_params.task_aligned_lower_limits_)
    , task_aligned_upper_limits_(task_params.task_aligned_upper_limits_)

    , robot_joint_lower_limits_(robot_->getJointLowerLimits())
    , robot_joint_upper_limits_(robot_->getJointUpperLimits())
    , robot_joint_weights_(robot_->getJointWeights())
    , total_dof_(robot_joint_weights_.size())

    , max_gripper_step_size_(task_params.max_gripper_step_size_)
    , max_robot_dof_step_size_(task_params.max_robot_dof_step_size_)
    , min_robot_dof_step_size_(task_params.min_robot_dof_step_size_)
    , max_gripper_rotation_(task_params.max_gripper_rotation_)
    , goal_reach_radius_(task_params.goal_reach_radius_)
    , gripper_min_distance_to_obstacles_(task_params.gripper_min_distance_to_obstacles_)

    , band_distance2_scaling_factor_(task_params.band_distance2_scaling_factor_)
    , band_max_points_(task_params.band_max_points_)
    , band_max_dist2_(band_distance2_scaling_factor_ * (double)band_max_points_ * (task_aligned_upper_limits_ - task_aligned_lower_limits_).squaredNorm())

    , forward_tree_extend_iterations_(planning_params.forward_tree_extend_iterations_)
    , backward_tree_extend_iterations_(planning_params.backward_tree_extend_iterations_)
    , use_brute_force_nn_(planning_params.use_brute_force_nn_)
    , kd_tree_grow_threshold_(planning_params.kd_tree_grow_threshold_)
    , best_near_radius2_(planning_params.best_near_radius2_)
    , goal_bias_(planning_params.goal_bias_)

    , max_shortcut_index_distance_(smoothing_params.max_shortcut_index_distance_)
    , max_smoothing_iterations_(smoothing_params.max_smoothing_iterations_)
    , smoothing_band_dist_threshold_(smoothing_params.smoothing_band_dist_threshold_)
    , uniform_shortcut_smoothing_int_distribution_(1, 4)

    , forward_nn_data_idx_to_tree_idx_(0)
    , goal_set_nn_data_idx_to_tree_idx_(0)
    , forward_nn_raw_data_(0)
    , goal_set_nn_raw_data_(0)
    , forward_nn_index_(nullptr)
    , goal_set_nn_index_(nullptr)
    , forward_next_idx_to_add_to_nn_dataset_(0)
    , goal_set_next_idx_to_add_to_nn_dataset_(0)

    , total_sampling_time_(NAN)
    , total_nearest_neighbour_index_building_time_(NAN)
    , total_nearest_neighbour_index_searching_time_(NAN)
    , total_nearest_neighbour_linear_searching_time_(NAN)
    , total_nearest_neighbour_radius_searching_time_(NAN)
    , total_nearest_neighbour_best_searching_time_(NAN)
    , total_nearest_neighbour_time_(NAN)
    , total_forward_kinematics_time_(NAN)
    , total_projection_time_(NAN)
    , total_collision_check_time_(NAN)
    , total_band_forward_propogation_time_(NAN)
    , total_first_order_vis_propogation_time_(NAN)
    , total_everything_included_forward_propogation_time_(NAN)
    , forward_random_samples_useful_(0)
    , forward_random_samples_useless_(0)
    , backward_random_samples_useful_(0)
    , backward_random_samples_useless_(0)
    , forward_connection_attempts_useful_(0)
    , forward_connection_attempts_useless_(0)
    , forward_connections_made_(0)
    , backward_connection_attempts_useful_(0)
    , backward_connection_attempts_useless_(0)
    , backward_connections_made_(0)
//    , path_found_(false)
//    , goal_idx_in_forward_tree_(-1)

    , vis_(vis)
    , visualization_enabled_globally_(visualization_enabled)
    , gripper_a_forward_tree_color_(Visualizer::Magenta())
    , gripper_b_forward_tree_color_(Visualizer::Red())
    , gripper_a_backward_tree_color_(Visualizer::Yellow())
    , gripper_b_backward_tree_color_(Visualizer::Cyan())
    , band_tree_color_(Visualizer::Blue())
{
    assert(task_aligned_lower_limits_.x() <= task_aligned_upper_limits_.x());
    assert(task_aligned_lower_limits_.y() <= task_aligned_upper_limits_.y());
    assert(task_aligned_lower_limits_.z() <= task_aligned_upper_limits_.z());
    assert(max_gripper_step_size_ > 0.0);
    assert(goal_reach_radius_ > 0.0);
    assert(max_shortcut_index_distance_ > 0);
    assert(gripper_min_distance_to_obstacles_ > 0.0);

    RRTDistance::SetJointWeights(robot_joint_weights_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Externally accessible planning related functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Builds the helper functions needed by simple_rrt_planner and invokes the planner (and shortcut smoother)
 */
RRTPolicy BandRRT::plan(
        const RRTNode& start,
        const RRTGrippersRepresentation& grippers_goal_poses,
        const std::chrono::duration<double>& time_limit)
{
//    sample_history_buffer_.clear();
//    sample_history_buffer_.reserve(1e9);

    const auto estimated_tree_size = ROSHelpers::GetParam(*ph_, "estimated_tree_size", 100000);

    // Extract start information
    starting_band_ = std::make_shared<RubberBand>(*start.band());
    starting_robot_configuration_ = start.robotConfiguration();
    starting_grippers_poses_ = start.grippers();

    // Setup the forward tree
    const double cost_to_come = 0.0;
    const double p_reachability = 1.0;
    const double p_transition = 1.0;
    const int64_t parent_index = -1;
    const int64_t state_index = 0;
    const int64_t transition_index = -1;
    const int64_t split_index = -1;

    forward_tree_.clear();
    forward_tree_.reserve(estimated_tree_size);
    forward_tree_.push_back(
                RRTNode(
                    starting_grippers_poses_,
                    starting_robot_configuration_,
                    starting_band_,
                    cost_to_come,
                    p_reachability,
                    p_transition,
                    parent_index,
                    state_index,
                    transition_index,
                    split_index));

    // Reset the bookkeeping counters
    next_state_index_ = 1;
    next_transition_index_ = 0;
    next_split_index_ = 0;

    // Extract goal/termination information
    max_grippers_distance_ = starting_band_->maxSafeLength();
    time_limit_ = time_limit;

    // Setup the backward tree
    grippers_goal_set_.clear();
    grippers_goal_set_.reserve(estimated_tree_size);
    // If we're using the whole robot, we may need to tweak the goal config to be feasible
    if (planning_for_whole_robot_)
    {
        const auto goal_configurations = robot_->getCloseIkSolutions(
                    {grippers_goal_poses.first, grippers_goal_poses.second},
                    max_grippers_distance_);
        assert(goal_configurations.size() > 0);

        robot_->setActiveDOFValues(goal_configurations[0]);
        const auto grippers_goal_poses_updated_vec = robot_->getGrippersPosesFunctionPointer();
        grippers_goal_poses_.first = grippers_goal_poses_updated_vec.at(0);
        grippers_goal_poses_.second = grippers_goal_poses_updated_vec.at(1);

        for (size_t idx = 0; idx < goal_configurations.size(); ++idx)
        {
            grippers_goal_set_.push_back(
                        RRTNode(
                            grippers_goal_poses_,
                            goal_configurations[idx],
                            start.band()));
        }
    }
    else
    {
        grippers_goal_poses_ = grippers_goal_poses;
        RRTRobotRepresentation goal_configuration(6);
        goal_configuration << grippers_goal_poses_.first.translation(), grippers_goal_poses_.second.translation();
        grippers_goal_set_.push_back(RRTNode(grippers_goal_poses_, goal_configuration, start.band()));
    }

    // Double check that the input goal location isn't immediately impossible
    const double first_gripper_dist_to_env = sdf_->EstimateDistance3d(grippers_goal_poses_.first.translation()).first;
    const double second_gripper_dist_to_env = sdf_->EstimateDistance3d(grippers_goal_poses_.second.translation()).first;
    if (first_gripper_dist_to_env < gripper_min_distance_to_obstacles_ ||
        second_gripper_dist_to_env < gripper_min_distance_to_obstacles_ ||
        (maxGrippersDistanceViolated(grippers_goal_poses_, max_grippers_distance_) > max_grippers_distance_))
    {
        const double dist_between_grippers = (grippers_goal_poses_.first.translation() - grippers_goal_poses_.second.translation()).norm();
        std::cerr << "Unfeasible goal location: " << grippers_goal_poses_.first.translation().transpose() << "  :  " << grippers_goal_poses_.second.translation().transpose() << std::endl;
        std::cerr << "Min gripper collision distance: " << gripper_min_distance_to_obstacles_ << " Current Distances: " << first_gripper_dist_to_env << " " << second_gripper_dist_to_env << std::endl;
        std::cerr << "Max allowable distance: " << max_grippers_distance_ << " Distance beteween goal grippers: " << dist_between_grippers << std::endl;

        vis_->visualizeGrippers("weird_gripper_goals", VectorIsometry3d{grippers_goal_poses_.first, grippers_goal_poses_.second}, Visualizer::Red(), 1);
//        PressAnyKeyToContinue("Unfeasible goal location");
//        assert(false && "Unfeasible goal location");
//        throw_arc_exception(std::runtime_error, "Unfeasible goal location");
        return {};
    }

    // Clear the forward tree flann data
    forward_nn_data_idx_to_tree_idx_.clear();
    forward_nn_data_idx_to_tree_idx_.reserve(estimated_tree_size);
    forward_nn_raw_data_.clear();
    forward_nn_raw_data_.reserve(total_dof_ * estimated_tree_size);
    forward_nn_index_ = std::make_shared<NNIndexType>(flann::KDTreeSingleIndexParams(), flann::L2_weighted<float>(robot_->getJointWeights()));
    forward_next_idx_to_add_to_nn_dataset_ = 0;

    // Clear the backward tree flann data
    goal_set_nn_data_idx_to_tree_idx_.clear();
    goal_set_nn_data_idx_to_tree_idx_.reserve(estimated_tree_size);
    goal_set_nn_raw_data_.clear();
    goal_set_nn_raw_data_.reserve(total_dof_ * estimated_tree_size);
    goal_set_nn_index_ = std::make_shared<NNIndexType>(flann::KDTreeSingleIndexParams(), flann::L2_weighted<float>(robot_->getJointWeights()));
    goal_set_next_idx_to_add_to_nn_dataset_ = 0;

    // Force an initializion the backward tree flann data
//    const bool force_rebuild = true;
//    rebuildNNIndex(goal_set_nn_index_, goal_set_nn_raw_data_, goal_set_nn_data_idx_to_tree_idx_, grippers_goal_set_, goal_set_next_idx_to_add_to_nn_dataset_, force_rebuild);

    if (visualization_enabled_globally_)
    {
        tree_marker_id_ = 1;
        forward_tree_next_visualized_node_ = 0;
        backward_tree_next_visualized_node_ = 0;
        deleteTreeVisualizations();
        visualizeBlacklist();
        transition_estimator_->visualizeLearnedTransitions("all_");
        vis_->forcePublishNow(0.02);
        vis_->purgeMarkerList();
    }

    // Clear statistics
    transition_estimator_->resetStatistics();
    total_sampling_time_ = 0.0;
    total_nearest_neighbour_index_building_time_ = 0.0;
    total_nearest_neighbour_index_searching_time_ = 0.0;
    total_nearest_neighbour_linear_searching_time_ = 0.0;
    total_nearest_neighbour_radius_searching_time_ = 0.0;
    total_nearest_neighbour_best_searching_time_ = 0.0;
    total_nearest_neighbour_time_ = 0.0;
    total_forward_kinematics_time_ = 0.0;
    total_projection_time_ = 0.0;
    total_collision_check_time_ = 0.0;
    total_band_forward_propogation_time_ = 0.0;
    total_first_order_vis_propogation_time_ = 0.0;
    total_everything_included_forward_propogation_time_ = 0.0;

    forward_random_samples_useful_ = 0;
    forward_random_samples_useless_ = 0;
    backward_random_samples_useful_ = 0;
    backward_random_samples_useless_ = 0;
    forward_connection_attempts_useful_ = 0;
    forward_connection_attempts_useless_ = 0;
    forward_connections_made_ = 0;
    backward_connection_attempts_useful_ = 0;
    backward_connection_attempts_useless_ = 0;
    backward_connections_made_ = 0;

    ROS_INFO_NAMED("rrt", "Starting BandRRT");
    if (useStoredTree())
    {
        forward_tree_ = loadStoredTree();
    }
    else
    {
        robot_->lockEnvironment();
        start_time_ = std::chrono::steady_clock::now();
        ROS_INFO_NAMED("rrt", "Starting planning...");
        planningMainLoop();
        const std::chrono::time_point<std::chrono::steady_clock> end_time = std::chrono::steady_clock::now();
        const std::chrono::duration<double> planning_time(end_time - start_time_);
        robot_->unlockEnvironment();

        ROS_INFO_STREAM_NAMED("rrt", "Finished planning, for better or worse. Path found with probability " << forward_tree_[0].getpGoalReachable());

        planning_statistics_["planning_time0_sampling                                 "] = total_sampling_time_;
        planning_statistics_["planning_time1_1_nearest_neighbour_index_building       "] = total_nearest_neighbour_index_building_time_;
        planning_statistics_["planning_time1_2_nearest_neighbour_index_searching      "] = total_nearest_neighbour_index_searching_time_;
        planning_statistics_["planning_time1_3_nearest_neighbour_linear_searching     "] = total_nearest_neighbour_linear_searching_time_;
        planning_statistics_["planning_time1_4_nearest_neighbour_radius_searching     "] = total_nearest_neighbour_radius_searching_time_;
        planning_statistics_["planning_time1_5_nearest_neighbour_best_searching       "] = total_nearest_neighbour_best_searching_time_;
        planning_statistics_["planning_time1_nearest_neighbour                        "] = total_nearest_neighbour_time_;
        planning_statistics_["planning_time2_1_forward_propogation_fk                 "] = total_forward_kinematics_time_;
        planning_statistics_["planning_time2_2_forward_propogation_projection         "] = total_projection_time_;
        planning_statistics_["planning_time2_3_forward_propogation_collision_check    "] = total_collision_check_time_;
        planning_statistics_["planning_time2_4_forward_propogation_band               "] = total_band_forward_propogation_time_ - transition_estimator_->classifierTime();
        planning_statistics_["planning_time2_5_classifier                             "] = transition_estimator_->classifierTime();
        planning_statistics_["planning_time2_6_forward_propogation_first_order_vis    "] = total_first_order_vis_propogation_time_;
        planning_statistics_["planning_time2_forward_propogation_everything_included  "] = total_everything_included_forward_propogation_time_;
        planning_statistics_["planning_time3_total                                    "] = planning_time.count();

        planning_statistics_["planning_size00_forward_random_samples_useless          "] = (double)forward_random_samples_useless_;
        planning_statistics_["planning_size01_forward_random_samples_useful           "] = (double)forward_random_samples_useful_;
        planning_statistics_["planning_size02_forward_states                          "] = (double)forward_tree_.size();

        planning_statistics_["planning_size03_backward_random_samples_useless         "] = (double)backward_random_samples_useless_;
        planning_statistics_["planning_size04_backward_random_samples_useful          "] = (double)backward_random_samples_useful_;
        planning_statistics_["planning_size05_backward_states                         "] = (double)grippers_goal_set_.size();

        planning_statistics_["planning_size06_forward_connection_attempts_useless     "] = (double)forward_connection_attempts_useless_;
        planning_statistics_["planning_size07_forward_connection_attempts_useful      "] = (double)forward_connection_attempts_useful_;
        planning_statistics_["planning_size08_forward_connections_made                "] = (double)forward_connections_made_;

        planning_statistics_["planning_classifier_num0_band_wierdness                 "] = (double)transition_estimator_->numBandWeirdness();
        planning_statistics_["planning_classifier_num1_band_safe                      "] = (double)transition_estimator_->numBandSafe();
        planning_statistics_["planning_classifier_num2_band_overstretch               "] = (double)transition_estimator_->numBandOverstretch();
        planning_statistics_["planning_classifier_num3_band_no_mistake                "] = (double)transition_estimator_->numNoMistakes();
        planning_statistics_["planning_classifier_num4_band_mistakes                  "] = (double)transition_estimator_->numMistakes();
        planning_statistics_["planning_classifier_num5_band_accepted_mistakes         "] = (double)transition_estimator_->numAcceptedMistakes();

//        planning_statistics_["planning_size09_backward_connection_attempts_useless    "] = (double)backward_connection_attempts_useless_;
//        planning_statistics_["planning_size10_backward_connection_attempts_useful     "] = (double)backward_connection_attempts_useful_;
//        planning_statistics_["planning_size11_backward_connections_made               "] = (double)backward_connections_made_;

        ROS_INFO_STREAM_NAMED("rrt", "RRT Helper Planning Statistics:\n" << PrettyPrint::PrettyPrint(planning_statistics_, false, "\n") << std::endl);
//        storeTree(forward_tree_);

        if (visualization_enabled_globally_)
        {
            ROS_INFO_NAMED("rrt", "Visualizing tree.");
            visualizeBothTrees();
            visualizeBlacklist();
            vis_->forcePublishNow(0.5);
        }
    }

    RRTPolicy policy;
    if (forward_tree_[0].getpGoalReachable() > 0.0)
    {
        ROS_INFO_NAMED("rrt", "Extracting solution policy");
        policy = ExtractSolutionPolicy(forward_tree_);
    }

    // If we either retreived a path, or made a new one, visualize and do smoothing
    if (policy.size() != 0)
    {
        if (visualization_enabled_globally_)
        {
            deleteTreeVisualizations();
            visualizeBlacklist();
            transition_estimator_->visualizeLearnedTransitions("all_");
            visualizePolicy(policy);
            vis_->forcePublishNow(0.05);
        }

//        ROS_INFO_NAMED("rrt", "Playing back unsmoothed path in OpenRAVE");
//        robot_->testPathForCollision(ConvertRRTPathToRobotPath(path));

        ROS_INFO_NAMED("rrt", "Starting Shortcut Smoothing");
        robot_->lockEnvironment();
        const bool visualize_rrt_smoothing = visualization_enabled_globally_ && true;
        shortcutSmoothPolicy(policy, visualize_rrt_smoothing);
        robot_->unlockEnvironment();
        std::cout << "RRT Helper Smoothing Statistics:\n" << PrettyPrint::PrettyPrint(smoothing_statistics_, false, "\n") << std::endl << std::endl;

//        ROS_INFO_NAMED("rrt", "Playing back smoothed path in OpenRAVE");
//        robot_->testPathForCollision(ConvertRRTPathToRobotPath(smoothed_path));

        if (visualization_enabled_globally_)
        {
            deleteTreeVisualizations();
            visualizeBlacklist();
            transition_estimator_->visualizeLearnedTransitions("all_");
            visualizePolicy(policy);
            vis_->forcePublishNow(0.5);
        }
    }

    return policy;
}

const std::vector<RubberBand::ConstPtr>& BandRRT::getBlacklist() const
{
    return blacklisted_goal_rubber_bands_;
}

void BandRRT::addBandToBlacklist(const RubberBand& band)
{
    blacklisted_goal_rubber_bands_.push_back(std::make_shared<const RubberBand>(band));
    blacklisted_goal_rubber_bands_.back()->resampleBand();
    blacklisted_goal_rubber_bands_.back()->upsampleBand();
}

void BandRRT::clearBlacklist()
{
    blacklisted_goal_rubber_bands_.clear();
}

BandRRT::PlanningSmoothingStatistics BandRRT::getStatistics() const
{
    return {planning_statistics_, smoothing_statistics_};
}

//////////// Helpers used to check abritray trees, and extract policies from planning trees ////////////////////////////

/* Checks the planner tree to make sure the parent-child linkages are correct
 */
bool BandRRT::CheckTreeLinkage(const RRTTree& tree)
{
    // Step through each state in the tree. Make sure that the linkage to the parent and child states are correct
    for (size_t current_index = 0; current_index < tree.size(); current_index++)
    {
        // For every state, make sure all the parent<->child linkages are valid
        const auto& current_node = tree[current_index];
        if (!current_node.initialized())
        {
            std::cerr << "Tree contains uninitialized node(s) " << current_index << std::endl;
            return false;
        }
        // Check the linkage to the parent state
        const int64_t parent_index = current_node.parentIndex();
        if ((parent_index >= 0) && (parent_index < (int64_t)tree.size()))
        {
            if (parent_index != (int64_t)current_index)
            {
                const auto& parent_node = tree[parent_index];
                if (!parent_node.initialized())
                {
                    std::cerr << "Tree contains uninitialized node(s) " << parent_index << std::endl;
                    return false;
                }
                // Make sure the corresponding parent contains the current node in the list of child indices
                const std::vector<int64_t>& parent_child_indices = parent_node.childIndices();
                auto index_found = std::find(parent_child_indices.begin(), parent_child_indices.end(), (int64_t)current_index);
                if (index_found == parent_child_indices.end())
                {
                    std::cerr << "Parent state " << parent_index << " does not contain child index for current node " << current_index << std::endl;
                    return false;
                }
            }
            else
            {
                std::cerr << "Invalid parent index " << parent_index << " for state " << current_index << " [Indices can't be the same]" << std::endl;
                return false;
            }
        }
        else if (parent_index < -1)
        {
            std::cerr << "Invalid parent index " << parent_index << " for state " << current_index << std::endl;
            return false;
        }
        // Check the linkage to the child states
        const std::vector<int64_t>& current_child_indices = current_node.childIndices();
        for (size_t idx = 0; idx < current_child_indices.size(); idx++)
        {
            // Get the current child index
            const int64_t current_child_index = current_child_indices[idx];
            if ((current_child_index > 0) && (current_child_index < (int64_t)tree.size()))
            {
                if (current_child_index != (int64_t)current_index)
                {
                    const auto& child_state = tree[current_child_index];
                    if (!child_state.initialized())
                    {
                        std::cerr << "Tree contains uninitialized node(s) " << current_child_index << std::endl;
                        return false;
                    }
                    // Make sure the child node points to us as the parent index
                    const int64_t child_parent_index = child_state.parentIndex();
                    if (child_parent_index != (int64_t)current_index)
                    {
                        std::cerr << "Parent index " << child_parent_index << " for current child state " << current_child_index << " does not match index " << current_index << " for current node " << std::endl;
                        return false;
                    }
                }
                else
                {
                    std::cerr << "Invalid child index " << current_child_index << " for state " << current_index << " [Indices can't be the same]" << std::endl;
                    return false;
                }
            }
            else
            {
                std::cerr << "Invalid child index " << current_child_index << " for state " << current_index << std::endl;
                return false;
            }
        }
    }
    return true;
}

/* Checks the planner tree to make sure the parent-child linkages are correct, and that the object is a path
 */
bool BandRRT::CheckPathLinkage(const RRTPath& path)
{
    // Step through each state in the path. Make sure that the linkage to the parent and child states are correct
    for (size_t current_index = 0; current_index < path.size(); current_index++)
    {
        // For every state, make sure all the parent<->child linkages are valid
        const auto& current_node = path[current_index];
        if (!current_node.initialized())
        {
            std::cerr << "Path contains uninitialized node " << current_index << std::endl;
            return false;
        }

        // Check the linkage to the parent state
        const int64_t parent_index = current_node.parentIndex();
        if (parent_index != (int64_t)current_index - 1)
        {
            std::cerr << "Path contains a node whose parent is not the immediate prior node " << current_index << std::endl;
        }
        // Check the linkage to the child states
        const auto& current_child_indices = current_node.childIndices();
        // Check that the current node at most one child
        if (current_child_indices.size() > 1)
        {
            std::cerr << "Path contains a node with multiple children " << current_index << std::endl;
            return false;
        }
        // Check that if there are no children, then it is the last node
        if (current_child_indices.size() == 0 && current_index != path.size() - 1)
        {
            std::cerr << "Path contains a node with no children that is not the last node " << current_index << std::endl;
            return false;
        }
        // Check that if this is the last node, then it has no children
        if (current_index == path.size() - 1 && current_child_indices.size() != 0)
        {
            std::cerr << "Last node in path contains children " << current_index << std::endl;
            return false;
        }
        // Check that the child is the next index in the path
        if (current_child_indices[0] != (int64_t)current_index + 1)
        {
            std::cerr << "Path contains a node whose children are not the next node in the path " << current_index << std::endl;
            return false;
        }
    }
    return true;
}

/* Extracts the portion of the tree that can potentially reach the goal,
 * removing all "stranded" branches, as well as any dominated branches
 *
 * A path is dominated if there is some other action that has a higher probability of reaching the goal.
 * In the event of a tie, paths without splits are preferred (chosen greedily from the root).
 * If there is still a tie, visualize a bunch of stuff and revist this question later.
 *
 * Returned value is a serpies of paths, each with the potential next paths to take
 * depending on the resolution of a split.
 *         Stricktly speaking this duplicates the functionality of the RRTNode/RRTTree structure
 *         but the different implementation potentially makes it easier to understand which way
 *         to interpret the data, and avoid repeatedly scanning through and separating the data
 *         into the needed segments for smoothing
 *
 * The node indices of the returned paths will be rewritten assume that there is only one branch.
 * This is done to make later shortcut smoothing easier with the existing framework.
 *
 * Each path overlaps with the preceeding and the following paths in the robot configuration.
 * The final configuration of the planned band is not meaningful if there is a following path.
 */
RRTPolicy BandRRT::ExtractSolutionPolicy(const RRTTree& tree)
{
    assert(CheckTreeLinkage(tree));

    ROS_WARN_NAMED("rrt", "Extracting a solution policy from the tree");
    if (tree.empty())
    {
        ROS_ERROR_NAMED("rrt", "Asked to extract policy from empty tree; this doesn't make sense. Returning an empty policy");
        return {};
    }

    const RRTNode& root = tree[0];
    if (root.getpGoalReachable() <= 0.0)
    {
        ROS_ERROR_NAMED("rrt", "Asked to extract policy from a tree that cannot reach the goal. Returning an empty policy");
        return {};
    }

    // Roots of each path; we will add to this list as we find splits in the dominant path
    // Note that the fact that we have chosen a queue lets us choose a convininet way to keep track of the paths by index
    size_t next_path_idx = 0;
    std::queue<int64_t> path_roots;
    path_roots.push(0);
    ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Added root of tree to path roots queue");

    std::vector<std::pair<RRTPath, std::vector<size_t>>> policy;
    while (!path_roots.empty())
    {
        // Retrieve the next path_root to consider, updating the corresponding counter
        const RRTNode& path_root = tree[path_roots.front()];
        path_roots.pop();
        ++next_path_idx;
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Popped node with state idx: " << path_root.stateIndex() << " from path roots queue");

        // Travel down the tree, choosing the dominating path until the dominating path contains a split
        RRTPath path;
        RRTNode next_node = path_root;
        bool goal_reached = false;
        bool split_reached = false;
        do
        {
            const RRTNode& curr_node = next_node;
            path.push_back(curr_node);
            ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "    Added node with state idx: " << curr_node.stateIndex() << " to path");

            // Find the action with the highest probability of reaching the goal

            // First, extract the potential options
            const auto child_indices = curr_node.childIndices();
            std::map<int64_t, std::vector<int64_t>> effective_child_branches;
            for (size_t idx = 0; idx < child_indices.size(); ++idx)
            {
                const int64_t current_child_index = child_indices[idx];
                const int64_t child_transition_idx = tree[(size_t)current_child_index].transitionIndex();
                effective_child_branches[child_transition_idx].push_back(current_child_index);
            }
            ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "    " << effective_child_branches.size() << " transitions taken out of this node");

            // Now that we have the splits separated out, compute the goal probability of each transition, keeping only the largest
            double p_goal_reachable = 0.0;
            std::vector<int64_t> best_transition_idx; // should be only one element long, ideally. A vector is stored to catch edge conditions that we have not yet decided how to resolve.
            for (auto itr = effective_child_branches.begin(); itr != effective_child_branches.end(); ++itr)
            {
                const std::vector<int64_t>& current_transition_children = itr->second;
                double p_goal_reachable_current_transition = 0.0;
                for (size_t child_idx = 0; child_idx < current_transition_children.size(); ++child_idx)
                {
                    const RRTNode& child = tree[current_transition_children[child_idx]];
                    p_goal_reachable_current_transition += child.pTransition() * child.getpGoalReachable();
                }

                if (p_goal_reachable_current_transition > 0.0 &&
                    p_goal_reachable_current_transition > p_goal_reachable)
                {
                    best_transition_idx = {itr->first};
                    p_goal_reachable = p_goal_reachable_current_transition;
                }
                else if (p_goal_reachable_current_transition > 0.0 &&
                         p_goal_reachable_current_transition == p_goal_reachable)
                {
                    best_transition_idx.push_back(itr->first);
                }

                ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "    Transition idx " << itr->first << " p_goal_reachable_current_transition: " << p_goal_reachable_current_transition);
            }

            if (best_transition_idx.size() == 0)
            {
                // If there are no children, and we're at this point in the tree, then this had better satisfy the goal check.
                // TODO: make this function non static and explicitly use the goalCheck function
                if (curr_node.getpGoalReachable() != 1.0)
                {
                    std::cout << curr_node << std::endl;
                    PressAnyKeyToContinue("Weirdness in tree");
                    assert(false);
                }
                goal_reached = true;
                assert(path.size() > 1);
                std::vector<size_t> child_paths;
                policy.push_back({path, child_paths});
            }
            else if (best_transition_idx.size() == 1)
            {
                const std::vector<int64_t> next_child_indices = effective_child_branches[best_transition_idx[0]];
                ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "    Using transition idx " << best_transition_idx[0]);
                // We are not at the goal, so we should have some children
                assert(next_child_indices.size() > 0);

                // If the next child is a singleton (i.e., not a split), carry on.
                // If the next node is the child of a split, this will cause the loop to exit
                next_node = tree.at(next_child_indices[0]);

                // If reached the end of this path, record the next paths to consider and upate the data structures
                if (next_child_indices.size() > 1)
                {
                    split_reached = true;

                    std::vector<size_t> child_paths;
                    for (const int64_t next_child_idx : next_child_indices)
                    {
                        path_roots.push(next_child_idx);
                        child_paths.push_back(next_path_idx);
                        ++next_path_idx;
                    }

                    // Note that we are including the next_node in the current path to ensure overlap between the paths
                    path.push_back(next_node);
                    assert(child_paths.size() > 1);
                    assert(path.size() > 1);
                    policy.push_back({path, child_paths});
                }
            }
            else
            {
                std::cerr << "Unhandled edge case when considering 'best transition'.\n"
                          << "    p_goal_reachable = " << p_goal_reachable << "    Transitions: " << PrettyPrint::PrettyPrint(best_transition_idx, false, " ") << std::endl;
                assert(best_transition_idx.size() == 1);
            }
        }
        // Keep iterating so long as the next node is not the result of a split
        while(!goal_reached && !split_reached);
    }

    // Post process the paths to clean them up and make the indices independent of the given tree
    for (auto& partial_result : policy)
    {
        RRTPath& path = partial_result.first;
        for (int64_t path_idx = 0; path_idx < (int64_t)path.size(); ++path_idx)
        {
            RRTNode& node = path[path_idx];
            node.state_index_ = path_idx;
            node.parent_index_ = path_idx - 1;
            node.clearChildIndices();
            node.addChildIndex(node.stateIndex() + 1);
        }
        path.back().clearChildIndices();
        assert(CheckPathLinkage(path));
    }

    return policy;
}

std::vector<VectorXd> BandRRT::ConvertRRTPathToRobotPath(const RRTTree& path)
{
    assert(false && "Redo this function - a path is a policy now");
    std::vector<VectorXd> robot_config_path(path.size());
    for (size_t ind = 0; ind < path.size(); ++ind)
    {
        robot_config_path[ind] =  path[ind].robotConfiguration();
    }
    return robot_config_path;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Visualization and other debugging tools
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Draws lines connecting all nodes in the tree, from start_idx through to the end of the vector
std::vector<Visualizer::NamespaceId> BandRRT::visualizeTree(
        const RRTTree& tree,
        const size_t start_idx,
        const std::string ns_a,
        const std::string ns_b,
        const std::string ns_band,
        const int id_a,
        const int id_b,
        const int id_band,
        const std_msgs::ColorRGBA& color_a,
        const std_msgs::ColorRGBA& color_b,
        const std_msgs::ColorRGBA& color_band,
        const bool draw_band) const
{
    std::vector<Visualizer::NamespaceId> marker_ids;

    if (visualization_enabled_globally_)
    {
        assert(start_idx < tree.size());

        const std_msgs::ColorRGBA color_a_dull = Multiply(color_a, 0.5f);
        const std_msgs::ColorRGBA color_b_dull = Multiply(color_b, 0.5f);
        const std_msgs::ColorRGBA color_band_dull = Multiply(color_band, 0.5f);

        VectorVector3d band_line_start_points;
        VectorVector3d band_line_end_points;
        std::vector<std_msgs::ColorRGBA> band_line_colors;

        VectorVector3d gripper_a_tree_start_points;
        VectorVector3d gripper_a_tree_end_points;
        std::vector<std_msgs::ColorRGBA> gripper_a_line_colors;
        VectorVector3d gripper_b_tree_start_points;
        VectorVector3d gripper_b_tree_end_points;
        std::vector<std_msgs::ColorRGBA> gripper_b_line_colors;

        gripper_a_tree_start_points.reserve(tree.size() - start_idx);
        gripper_b_tree_start_points.reserve(tree.size() - start_idx);
        gripper_a_tree_end_points.reserve(tree.size() - start_idx);
        gripper_b_tree_end_points.reserve(tree.size() - start_idx);

        for (size_t idx = start_idx; idx < tree.size(); ++idx)
        {
            const RRTNode& curr = tree[idx];
            if (!curr.initialized() && tree == forward_tree_)
            {
                std::cout << "Node idx: " << idx << std::endl
                          << curr.print() << std::endl;
            }

            if (draw_band)
            {
                const VectorVector3d& band_vec = curr.band()->getVectorRepresentation();
                for (size_t band_idx = 0; band_idx + 1 < band_vec.size(); ++band_idx)
                {
                    band_line_start_points.push_back(band_vec[band_idx]);
                    band_line_end_points.push_back(band_vec[band_idx + 1]);
                    if (curr.blacklisted_from_nn_search_)
                    {
                        band_line_colors.push_back(color_band_dull);
                    }
                    else
                    {
                        band_line_colors.push_back(color_band);
                    }
                }
            }

            if (curr.parentIndex() >= 0)
            {
                const RRTNode& parent = tree[curr.parentIndex()];

                // Add edges from the parent to the current node
                gripper_a_tree_start_points.push_back(parent.grippers().first.translation());
                gripper_b_tree_start_points.push_back(parent.grippers().second.translation());

                gripper_a_tree_end_points.push_back(curr.grippers().first.translation());
                gripper_b_tree_end_points.push_back(curr.grippers().second.translation());

                // Add colors to match the blacklisted status
                if (curr.blacklisted_from_nn_search_)
                {
                    gripper_a_line_colors.push_back(color_a_dull);
                    gripper_b_line_colors.push_back(color_b_dull);
                }
                else
                {
                    gripper_a_line_colors.push_back(color_a);
                    gripper_b_line_colors.push_back(color_b);
                }
            }
        }

        const auto gripper_a_ids = vis_->visualizeLines(ns_a, gripper_a_tree_start_points, gripper_a_tree_end_points, gripper_a_line_colors, id_a);
        marker_ids.insert(marker_ids.end(), gripper_a_ids.begin(), gripper_a_ids.end());
        const auto gripper_b_ids = vis_->visualizeLines(ns_b, gripper_b_tree_start_points, gripper_b_tree_end_points, gripper_b_line_colors, id_b);
        marker_ids.insert(marker_ids.end(), gripper_b_ids.begin(), gripper_b_ids.end());
        if (draw_band)
        {
            const auto band_ids = vis_->visualizeLines(ns_band, band_line_start_points, band_line_end_points, band_line_colors, id_band);
            marker_ids.insert(marker_ids.end(), band_ids.begin(), band_ids.end());
        }
    }

    return marker_ids;
}

std::vector<Visualizer::NamespaceId> BandRRT::visualizeBothTrees() const
{
    std::vector<Visualizer::NamespaceId> marker_ids;

    deleteTreeVisualizations();

    const bool draw_band = false;

    const auto forward_tree_ids = visualizeTree(
                forward_tree_,
                0,
                RRT_FORWARD_TREE_GRIPPER_A_NS,
                RRT_FORWARD_TREE_GRIPPER_B_NS,
                RRT_TREE_BAND_NS,
                1,
                1,
                1,
                gripper_a_forward_tree_color_,
                gripper_b_forward_tree_color_,
                band_tree_color_,
                draw_band);
    marker_ids.insert(marker_ids.end(),forward_tree_ids.begin(), forward_tree_ids.end());

    const auto backward_tree_ids = visualizeTree(
                grippers_goal_set_,
                0,
                RRT_BACKWARD_TREE_GRIPPER_A_NS,
                RRT_BACKWARD_TREE_GRIPPER_B_NS,
                RRT_TREE_BAND_NS,
                2,
                2,
                2,
                gripper_a_backward_tree_color_,
                gripper_b_backward_tree_color_,
                band_tree_color_,
                draw_band);
    marker_ids.insert(marker_ids.end(), backward_tree_ids.begin(),backward_tree_ids.end());


    vis_->forcePublishNow();

    return marker_ids;
}

void BandRRT::deleteTreeVisualizations() const
{
    vis_->purgeMarkerList();
    visualization_msgs::Marker marker;
    marker.ns = "delete_markers";
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.header.frame_id = "world_origin";
    marker.header.stamp = ros::Time::now();
    vis_->publish(marker);
    vis_->forcePublishNow(0.01);
    vis_->purgeMarkerList();

    vis_->visualizeCubes(TaskFramework::CLUSTERING_RESULTS_POST_PROJECT_NS, {grippers_goal_poses_.first.translation()}, Vector3d::Ones() * work_space_grid_.minStepDimension(), gripper_a_forward_tree_color_, 1);
    vis_->visualizeCubes(TaskFramework::CLUSTERING_RESULTS_POST_PROJECT_NS, {grippers_goal_poses_.second.translation()}, Vector3d::Ones() * work_space_grid_.minStepDimension(), gripper_b_forward_tree_color_, 5);
}

// Assumes the path that is passed is sequential
std::vector<Visualizer::NamespaceId> BandRRT::visualizePath(const RRTPath& path, const std::string& ns_prefix, const int32_t id, const bool draw_band) const
{
    std::vector<Visualizer::NamespaceId> marker_ids;

    VectorVector3d gripper_a_cubes;
    VectorVector3d gripper_b_cubes;
    gripper_a_cubes.reserve(path.size());
    gripper_b_cubes.reserve(path.size());

    VectorVector3d line_start_points;
    VectorVector3d line_end_points;

    for (int32_t ind = 0; ind < (int32_t)path.size(); ++ind)
    {
        const RRTNode& config = path[ind];
        assert(config.childIndices().size() <= 1);
        const RRTGrippersRepresentation& gripper_positions = config.grippers();
        const RubberBand::Ptr& rubber_band = config.band();

        gripper_a_cubes.push_back(gripper_positions.first.translation());
        gripper_b_cubes.push_back(gripper_positions.second.translation());

        const VectorVector3d band_vec = rubber_band->getVectorRepresentation();
        for (size_t band_idx = 0; band_idx + 1 < band_vec.size(); ++band_idx)
        {
            line_start_points.push_back(band_vec[band_idx]);
            line_end_points.push_back(band_vec[band_idx + 1]);
        }
    }

    const auto gripper_a_ids = vis_->visualizeCubes(ns_prefix + RRT_PATH_GRIPPER_A_NS, gripper_a_cubes, Vector3d(0.005, 0.005, 0.005), gripper_a_forward_tree_color_, id);
    marker_ids.insert(marker_ids.end(), gripper_a_ids.begin(), gripper_a_ids.end());
    const auto gripper_b_ids = vis_->visualizeCubes(ns_prefix + RRT_PATH_GRIPPER_B_NS, gripper_b_cubes, Vector3d(0.005, 0.005, 0.005), gripper_b_forward_tree_color_, id);
    marker_ids.insert(marker_ids.end(), gripper_b_ids.begin(), gripper_b_ids.end());
    if (draw_band)
    {
        const auto band_ids = vis_->visualizeLines(ns_prefix + RRT_PATH_RUBBER_BAND_NS, line_start_points, line_end_points, Visualizer::Blue(), id);
        marker_ids.insert(marker_ids.end(), band_ids.begin(), band_ids.end());
    }

    return marker_ids;
}

std::vector<Visualizer::NamespaceId> BandRRT::visualizePolicy(const RRTPolicy& policy, const bool draw_band) const
{
    std::vector<Visualizer::NamespaceId> marker_ids;
    for (size_t path_idx = 0; path_idx < policy.size(); ++path_idx)
    {
        const RRTPath& path = policy[path_idx].first;
        const auto new_ids = visualizePath(path, "", (int32_t)path_idx + 1, draw_band);
        marker_ids.insert(marker_ids.end(), new_ids.begin(), new_ids.end());
    }
    return marker_ids;
}

std::vector<Visualizer::NamespaceId> BandRRT::visualizeBlacklist() const
{
    std::vector<Visualizer::NamespaceId> ids;
    for (size_t idx = 0; idx < blacklisted_goal_rubber_bands_.size(); ++idx)
    {
        const auto& band = blacklisted_goal_rubber_bands_[idx];
        const auto new_ids = band->visualize(RRT_BLACKLISTED_GOAL_BANDS_NS, Visualizer::Red(), Visualizer::Red(), (int32_t)(idx + 1));
        ids.insert(ids.end(), new_ids.begin(), new_ids.end());
    }
    return ids;
}

void BandRRT::storeTree(const RRTTree& tree, std::string file_path) const
{
    try
    {
        if (file_path.empty())
        {
            const auto log_folder = ROSHelpers::GetParamRequiredDebugLog<std::string>(*nh_, "log_folder", __func__);
            const auto file_name_prefix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "path_file_name_prefix", __func__);
            const std::string file_name_suffix = arc_helpers::GetCurrentTimeAsString();
            const std::string file_name = file_name_prefix + "__" + file_name_suffix + ".compressed";
            file_path = log_folder + "/" + file_name;
        }
        ROS_DEBUG_STREAM_NAMED("rrt", "Saving path to " << file_path);

        std::vector<uint8_t> buffer;
        arc_utilities::SerializeVector<RRTNode, RRTAllocator>(tree, buffer, &RRTNode::Serialize);
        arc_utilities::CreateDirectory(boost::filesystem::path(file_path).parent_path());
        ZlibHelpers::CompressAndWriteToFile(buffer, file_path);
        // Verify no mistakes were made
        {
            const auto deserializer = [&] (const std::vector<uint8_t>& buf, const uint64_t cur)
            {
                return RRTNode::Deserialize(buf, cur, template_band_);
            };

            const RRTTree retrieved_path =
                    arc_utilities::DeserializeVector<RRTNode, RRTAllocator>(buffer, 0, deserializer).first;

            assert(retrieved_path == tree);
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM_NAMED("rrt", "Failed to store path: "  <<  e.what());
    }
}

RRTTree BandRRT::loadStoredTree(std::string file_path) const
{
    try
    {
        if (file_path.empty())
        {
            const auto log_folder = ROSHelpers::GetParamRequired<std::string>(*nh_, "log_folder", __func__);
            const auto file_name_prefix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "path_file_name_prefix", __func__);
            const auto file_name_suffix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "path_file_name_suffix_to_load", __func__);
            const std::string file_name = file_name_prefix + "__" + file_name_suffix + ".compressed";
            file_path = log_folder + "/" + file_name;
        }
        ROS_INFO_STREAM_NAMED("rrt", "Loading path from " << file_path);

        const auto deserializer = [&] (const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return RRTNode::Deserialize(buffer, current, template_band_);
        };

        const auto buffer = ZlibHelpers::LoadFromFileAndDecompress(file_path);
        const auto path_deserialized = arc_utilities::DeserializeVector<RRTNode, RRTAllocator>(buffer, 0, deserializer);
        return path_deserialized.first;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM_NAMED("rrt", "Failed to load stored path: "  <<  e.what());
        return RRTTree();
    }
}

bool BandRRT::useStoredTree() const
{
    return ROSHelpers::GetParamRequired<bool>(*ph_, "use_stored_path", __func__);
}

void BandRRT::SavePath(const RRTPath& path, const std::string& filename)
{
    std::vector<uint8_t> buffer;
    SerializeVector<RRTNode>(path, buffer, &RRTNode::Serialize);
    ZlibHelpers::CompressAndWriteToFile(buffer, filename);
}

RRTPath BandRRT::LoadPath(const std::string& filename, const RubberBand& template_band)
{
    const auto buffer = ZlibHelpers::LoadFromFileAndDecompress(filename);
    const auto deserializer = [&] (const std::vector<uint8_t>& buf, const uint64_t cur)
    {
        return RRTNode::Deserialize(buf, cur, template_band);
    };
    const auto path_deserialized = DeserializeVector<RRTNode, Eigen::aligned_allocator<RRTNode>>(buffer, 0, deserializer);
    return path_deserialized.first;
}

void BandRRT::storePolicy(const RRTPolicy& policy, const std::string& file_path) const
{
    (void)policy;
    (void)file_path;
    ROS_ERROR_NAMED("rrt", "storePolicy not implemented.");
}

RRTPolicy BandRRT::loadPolicy(const std::string& file_path) const
{
    (void)file_path;
    ROS_ERROR_NAMED("rrt", "loadStoredPolicy not implemented.");
    return RRTPolicy();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions and data for internal rrt planning algorithm
//  - Order is roughly the order that they are used internally
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BandRRT::planningMainLoop()
{
    // Make sure we've been given a start and goal state
    assert(forward_tree_.size() > 0);
    assert(grippers_goal_set_.size() > 0);

    // Plan
    ROS_INFO_NAMED("rrt", "Using single directional tree");
    std::chrono::duration<double> time_ellapsed = std::chrono::steady_clock::now() - start_time_;

    // Check if we have a path already
    bool path_found = false;
    if (forward_tree_.size() > 0 && forward_tree_[0].getpGoalReachable() == 1.0)
    {
        path_found = true;
        ROS_INFO_NAMED("rrt", "Goal found with probability 1.0 before the main loop started");
    }

    size_t main_loop_itr = 0;
    while (!path_found && time_ellapsed < time_limit_)
    {
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Starting forward iteration. Tree size: " << forward_tree_.size());

        //////////////// Extend (connect) the first tree towards a random target ////////////////
        const bool sample_band = true;
        const RRTNode random_target = configSampling(sample_band);
        const int64_t forward_tree_start_idx = nearestNeighbour(true, random_target);
        const size_t num_random_nodes_created = connectForwardTree(forward_tree_start_idx, random_target, true);
        if (forward_tree_.back().splitIndex() >= 0)
        {
            ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Split happened during connect to random operation");
            transition_estimator_->clearVisualizations();
        }
        checkNewStatesForGoal(num_random_nodes_created);
        if (forward_tree_[0].getpGoalReachable() == 1.0)
        {
            path_found = true;
            ROS_INFO_NAMED("rrt", "Goal found with probability 1.0 via random exploration");
        }
        else
        {
            //////////////// Attempt to connect to the backward tree ////////////////////////////////
            const int64_t last_node_idx_in_forward_tree_branch = num_random_nodes_created > 0 ?
                        (int64_t)forward_tree_.size() - 1 : forward_tree_start_idx;
            RRTNode& last_node = forward_tree_[last_node_idx_in_forward_tree_branch];

            const bool sample_goal = uniform_unit_distribution_(*generator_) < goal_bias_;
            if (!last_node.already_extended_towards_goal_set_
                && sample_goal)
            {
                last_node.already_extended_towards_goal_set_ = true;
                const size_t num_goal_directed_nodes_created =
                        connectTreeToGrippersGoalSet(last_node_idx_in_forward_tree_branch);
                if (forward_tree_.back().splitIndex() >= 0)
                {
                    ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Split happened during connect to backwards tree");
                }
                checkNewStatesForGoal(num_goal_directed_nodes_created);
            }

            if (forward_tree_[0].getpGoalReachable() == 1.0)
            {
                path_found = true;
                ROS_INFO_NAMED("rrt", "Goal found with probability 1.0 via targetting goal set");
            }
        }

        time_ellapsed = std::chrono::steady_clock::now() - start_time_;
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Ending forward iteration. Tree size: " << forward_tree_.size());

        ++main_loop_itr;
        if (main_loop_itr % 10000 == 0)
        {
            const std::chrono::time_point<std::chrono::steady_clock> end_time = std::chrono::steady_clock::now();
            const std::chrono::duration<double> planning_time(end_time - start_time_);

            std::map<std::string, double> partial_stats;

            partial_stats["planning_partial_time0_sampling                                 "] = total_sampling_time_;
            partial_stats["planning_partial_time1_1_nearest_neighbour_index_building       "] = total_nearest_neighbour_index_building_time_;
            partial_stats["planning_partial_time1_2_nearest_neighbour_index_searching      "] = total_nearest_neighbour_index_searching_time_;
            partial_stats["planning_partial_time1_3_nearest_neighbour_linear_searching     "] = total_nearest_neighbour_linear_searching_time_;
            partial_stats["planning_partial_time1_4_nearest_neighbour_radius_searching     "] = total_nearest_neighbour_radius_searching_time_;
            partial_stats["planning_partial_time1_5_nearest_neighbour_best_searching       "] = total_nearest_neighbour_best_searching_time_;
            partial_stats["planning_partial_time1_nearest_neighbour                        "] = total_nearest_neighbour_time_;
            partial_stats["planning_partial_time2_1_forward_propogation_fk                 "] = total_forward_kinematics_time_;
            partial_stats["planning_partial_time2_2_forward_propogation_projection         "] = total_projection_time_;
            partial_stats["planning_partial_time2_3_forward_propogation_collision_check    "] = total_collision_check_time_;
            partial_stats["planning_partial_time2_4_forward_propogation_band               "] = total_band_forward_propogation_time_ - transition_estimator_->classifierTime();
            partial_stats["planning_partial_time2_5_classifier                             "] = transition_estimator_->classifierTime();
            partial_stats["planning_partial_time2_6_forward_propogation_first_order_vis    "] = total_first_order_vis_propogation_time_;
            partial_stats["planning_partial_time2_forward_propogation_everything_included  "] = total_everything_included_forward_propogation_time_;
            partial_stats["planning_partial_time3_total                                    "] = planning_time.count();

            partial_stats["planning_partial_size00_forward_random_samples_useless          "] = (double)forward_random_samples_useless_;
            partial_stats["planning_partial_size01_forward_random_samples_useful           "] = (double)forward_random_samples_useful_;
            partial_stats["planning_partial_size02_forward_states                          "] = (double)forward_tree_.size();

            partial_stats["planning_partial_size03_backward_random_samples_useless         "] = (double)backward_random_samples_useless_;
            partial_stats["planning_partial_size04_backward_random_samples_useful          "] = (double)backward_random_samples_useful_;
            partial_stats["planning_partial_size05_backward_states                         "] = (double)grippers_goal_set_.size();

            partial_stats["planning_partial_size06_forward_connection_attempts_useless     "] = (double)forward_connection_attempts_useless_;
            partial_stats["planning_partial_size07_forward_connection_attempts_useful      "] = (double)forward_connection_attempts_useful_;
            partial_stats["planning_partial_size08_forward_connections_made                "] = (double)forward_connections_made_;

            partial_stats["planning_partial_classifier_num0_band_wierdness                 "] = (double)transition_estimator_->numBandWeirdness();
            partial_stats["planning_partial_classifier_num1_band_safe                      "] = (double)transition_estimator_->numBandSafe();
            partial_stats["planning_partial_classifier_num2_band_overstretch               "] = (double)transition_estimator_->numBandOverstretch();
            partial_stats["planning_partial_classifier_num3_band_no_mistake                "] = (double)transition_estimator_->numNoMistakes();
            partial_stats["planning_partial_classifier_num4_band_mistakes                  "] = (double)transition_estimator_->numMistakes();
            partial_stats["planning_partial_classifier_num5_band_accepted_mistakes         "] = (double)transition_estimator_->numAcceptedMistakes();

    //        partial_stats["planning_partial_size09_backward_connection_attempts_useless    "] = (double)backward_connection_attempts_useless_;
    //        partial_stats["planning_partial_size10_backward_connection_attempts_useful     "] = (double)backward_connection_attempts_useful_;
    //        partial_stats["planning_partial_size11_backward_connections_made               "] = (double)backward_connections_made_;

            ROS_INFO_STREAM_NAMED("rrt", "Planning Statistics @ Main Loop Itr: " << main_loop_itr << "\n" << PrettyPrint::PrettyPrint(partial_stats, false, "\n") << std::endl);

            if (forward_tree_next_visualized_node_ < forward_tree_.size())
            {
                const bool draw_band = true;
                visualizeTree(
                            forward_tree_,
                            forward_tree_next_visualized_node_,
                            RRT_FORWARD_TREE_GRIPPER_A_NS,
                            RRT_FORWARD_TREE_GRIPPER_B_NS,
                            RRT_TREE_BAND_NS,
                            tree_marker_id_,
                            tree_marker_id_,
                            1,
                            gripper_a_forward_tree_color_,
                            gripper_b_forward_tree_color_,
                            band_tree_color_,
                            draw_band);
                ++tree_marker_id_;
                if (tree_marker_id_ % 10 == 0)
                {
                    vis_->forcePublishNow(0.02);
                    vis_->purgeMarkerList();
                }
                forward_tree_next_visualized_node_ = forward_tree_.size();
            }
        }
    }
}

//////// Sampling functions ////////////////////////////////////////////////////////////////////////////////////////////

RRTNode BandRRT::configSampling(const bool sample_band)
{
    Stopwatch stopwatch;
    arc_helpers::DoNotOptimize(generator_);
    RRTNode sample;

    if (!planning_for_whole_robot_)
    {
        const auto gripper_poses = posPairSampling_internal();
        RRTRobotRepresentation robot_config(6);
        robot_config << gripper_poses.first.translation(), gripper_poses.second.translation();

        sample = RRTNode(
                    gripper_poses,
                    robot_config,
                    std::make_shared<RubberBand>(template_band_));
    }
    else
    {
        sample = RRTNode(
                    starting_grippers_poses_,
                    robotConfigPairSampling_internal(),
                    std::make_shared<RubberBand>(template_band_));
    }

    ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.sampling", "Random robot config: " << sample.robotConfiguration().transpose());

    if (sample_band)
    {
//        ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.sampling", "Sampling random band");
        sample.band()->overridePoints(bandSampling_internal());
    }

    arc_helpers::DoNotOptimize(sample);
    const double sampling_time = stopwatch(READ);
    total_sampling_time_ += sampling_time;

//    sample.serialize(sample_history_buffer_);

    return sample;
}

RRTGrippersRepresentation BandRRT::posPairSampling_internal()
{
    Vector3d gripper_a_pos, gripper_b_pos;

    do
    {
        const double x1 = Interpolate(task_aligned_lower_limits_.x(), task_aligned_upper_limits_.x(), uniform_unit_distribution_(*generator_));
        const double y1 = Interpolate(task_aligned_lower_limits_.y(), task_aligned_upper_limits_.y(), uniform_unit_distribution_(*generator_));
        const double z1 = Interpolate(task_aligned_lower_limits_.z(), task_aligned_upper_limits_.z(), uniform_unit_distribution_(*generator_));
        gripper_a_pos = Vector3d(x1, y1, z1);
    }
    while (sdf_->EstimateDistance3d(task_aligned_frame_transform_ * gripper_a_pos).first < gripper_min_distance_to_obstacles_);

    // We want to only sample within a radius max_grippers_distance_, and within the world extents; to do so
    // uniformly, we sample from an axis aligned box limited by R and the world extents, rejecting samples that lie
    // outside a radius max_grippers_distance_
    const double x2_min = std::max(task_aligned_lower_limits_.x(), gripper_a_pos.x() - max_grippers_distance_);
    const double x2_max = std::min(task_aligned_upper_limits_.x(), gripper_a_pos.x() + max_grippers_distance_);
    const double y2_min = std::max(task_aligned_lower_limits_.y(), gripper_a_pos.y() - max_grippers_distance_);
    const double y2_max = std::min(task_aligned_upper_limits_.y(), gripper_a_pos.y() + max_grippers_distance_);
    const double z2_min = std::max(task_aligned_lower_limits_.z(), gripper_a_pos.z() - max_grippers_distance_);
    const double z2_max = std::min(task_aligned_upper_limits_.z(), gripper_a_pos.z() + max_grippers_distance_);

    bool valid = false;
    do
    {
        const double x2 = Interpolate(x2_min, x2_max, uniform_unit_distribution_(*generator_));
        const double y2 = Interpolate(y2_min, y2_max, uniform_unit_distribution_(*generator_));
        const double z2 = Interpolate(z2_min, z2_max, uniform_unit_distribution_(*generator_));
        gripper_b_pos = Vector3d(x2, y2, z2);
        valid = !maxGrippersDistanceViolated(gripper_a_pos, gripper_b_pos, max_grippers_distance_);
    }
    while (!valid || sdf_->EstimateDistance3d(task_aligned_frame_transform_ * gripper_b_pos).first < gripper_min_distance_to_obstacles_);

    RRTGrippersRepresentation rand_sample = grippers_goal_poses_;
    rand_sample.first.translation() = task_aligned_frame_transform_ * gripper_a_pos;
    rand_sample.second.translation() = task_aligned_frame_transform_ * gripper_b_pos;

    return rand_sample;
}

RRTRobotRepresentation BandRRT::robotConfigPairSampling_internal()
{
    RRTRobotRepresentation rand_sample(total_dof_);
    for (ssize_t idx = 0; idx < total_dof_; ++idx)
    {
        rand_sample(idx) = Interpolate(robot_joint_lower_limits_(idx), robot_joint_upper_limits_(idx), uniform_unit_distribution_(*generator_));
    }
    return rand_sample;
}

VectorVector3d BandRRT::bandSampling_internal()
{
    VectorVector3d band_points;
    band_points.reserve(band_max_points_);

    for (size_t idx = 0; idx < band_max_points_; ++idx)
    {
        const double x = Interpolate(task_aligned_lower_limits_.x(), task_aligned_upper_limits_.x(), uniform_unit_distribution_(*generator_));
        const double y = Interpolate(task_aligned_lower_limits_.y(), task_aligned_upper_limits_.y(), uniform_unit_distribution_(*generator_));
        const double z = Interpolate(task_aligned_lower_limits_.z(), task_aligned_upper_limits_.z(), uniform_unit_distribution_(*generator_));
        band_points.push_back(task_aligned_frame_transform_ * Vector3d(x, y, z));
    }

    return band_points;
}

//////// Internal helpers used only by NN functionality ////////////////////////////////////////////////////////////////

static std::pair<int64_t, double> getNearest(
        const RRTRobotRepresentation& robot_config,
        const NNIndexType& index,
        const std::vector<size_t>& nn_data_idx_to_tree_idx)
{
    std::pair<int64_t, double> nearest(-1, std::numeric_limits<double>::infinity());

    VectorXf robot_config_float = robot_config.cast<float>();
    const flann::Matrix<float> query(robot_config_float.data(), 1, robot_config.size());

    const size_t knn = 1;
    std::vector<std::vector<size_t>> indices(query.rows, std::vector<size_t>(knn, -1));
    std::vector<std::vector<float>> dists(query.rows, std::vector<float>(knn, INFINITY));

    const float eps = 0.0;
    flann::SearchParams params(flann::flann_checks_t::FLANN_CHECKS_UNLIMITED, eps);
    index.knnSearch(query, indices, dists, knn, params);
    nearest.first = nn_data_idx_to_tree_idx.at(indices[0][0]);
    nearest.second = dists[0][0];

    return nearest;
}

static std::pair<int64_t, double> getNearest(
        const RRTRobotRepresentation& robot_config,
        const RRTTree& tree,
        const size_t start_idx)
{
    std::pair<int64_t, double> nearest(-1, std::numeric_limits<double>::infinity());

    for (size_t idx = start_idx; idx < tree.size(); idx++)
    {
        const RRTNode& test_node = tree[idx];
        if (!test_node.blacklisted_from_nn_search_)
        {
            const double distance2 = RRTDistance::DistanceSquared(test_node.robotConfiguration(), robot_config);
            if (nearest.second > distance2)
            {
                nearest.first = (int64_t)idx;
                nearest.second = distance2;
            }
        }
    }

    return nearest;
}

static std::vector<std::pair<int64_t, double>> radiusSearch(
        const RRTRobotRepresentation& robot_config,
        const NNIndexType& index,
        const std::vector<size_t>& nn_data_idx_to_tree_idx,
        const double radius2)
{
    std::vector<std::pair<int64_t, double>> near;

    VectorXf robot_config_float = robot_config.cast<float>();
    flann::Matrix<float> query(robot_config_float.data(), 1, robot_config.size());

    std::vector<std::vector<size_t>> indices(query.rows);
    std::vector<std::vector<float>> dists(query.rows);

    const float eps = 0.0;
    flann::SearchParams params(flann::flann_checks_t::FLANN_CHECKS_UNLIMITED, eps);
    index.radiusSearch(query, indices, dists, (float)radius2, params);

    assert(indices[0].size() == dists[0].size());
    near.reserve(indices[0].size());
    for (size_t idx = 0; idx < indices[0].size(); ++idx)
    {
        near.push_back({nn_data_idx_to_tree_idx.at(indices[0][idx]), dists[0][idx]});
    }

    return near;
}

static std::vector<std::pair<int64_t, double>> radiusSearch(
        const RRTRobotRepresentation& robot_config,
        const RRTTree& tree,
        const size_t start_idx,
        const double radius2)
{
    std::vector<std::pair<int64_t, double>> near;

    for (size_t idx = start_idx; idx < tree.size(); idx++)
    {
        const RRTNode& test_node = tree[idx];
        if (!test_node.blacklisted_from_nn_search_)
        {
            const double distance2 = RRTDistance::DistanceSquared(test_node.robotConfiguration(), robot_config);
            if (distance2 <= radius2)
            {
                near.push_back({idx, distance2});
            }
        }
    }

    return near;
}

static std::pair<int64_t, double> getNearestFullConfig(
        const RRTNode& config,
        const RRTTree& tree,
        const double band_distance2_scaling_factor_,
        const std::vector<std::pair<int64_t, double>>& radius_search_set_1,
        const std::vector<std::pair<int64_t, double>>& radius_search_set_2)
{
    assert(radius_search_set_1.size() + radius_search_set_2.size() > 0);

    std::pair<int64_t, double> nearest(-1, std::numeric_limits<double>::infinity());

    // Search through the first set of potential nearest nodes
    for (const auto& item : radius_search_set_1)
    {
        const auto& test_band = tree[item.first].band();
        const double band_distance2 = config.band()->distanceSq(*test_band);
        const double total_distance2 = item.second + band_distance2_scaling_factor_ * band_distance2;
        if (total_distance2 < nearest.second)
        {
            nearest.first = item.first;
            nearest.second = total_distance2;
        }
    }

    // Search through the second set of potential nearest nodes
    for (const auto& item : radius_search_set_2)
    {
        const auto& test_band = tree[item.first].band();
        const double band_distance2 = config.band()->distanceSq(*test_band);
        const double total_distance2 = item.second + band_distance2_scaling_factor_ * band_distance2;
        if (total_distance2 < nearest.second)
        {
            nearest.first = item.first;
            nearest.second = total_distance2;
        }
    }

    return nearest;
}

static std::pair<int64_t, double> getBestFullConfig(
        const RRTTree& tree,
        const std::vector<std::pair<int64_t, double>>& radius_search_set_1,
        const std::vector<std::pair<int64_t, double>>& radius_search_set_2)
{
    assert(radius_search_set_1.size() + radius_search_set_2.size() > 0);

    double min_cost = std::numeric_limits<double>::infinity();
    int64_t min_idx = -1;

    // Search through the first set of potential best nodes
    for (const auto& item : radius_search_set_1)
    {
        const auto test_cost = tree[item.first].costToCome();
        if (test_cost < min_cost)
        {
            min_idx = item.first;
            min_cost = test_cost;
        }
    }

    // Search through the second set of potential nearest nodes
    for (const auto& item : radius_search_set_2)
    {
        const auto test_cost = tree[item.first].costToCome();
        if (test_cost < min_cost)
        {
            min_idx = item.first;
            min_cost = test_cost;
        }
    }

    assert(min_idx >= 0);
    return {min_idx, min_cost};
}

//////// Nearest neighbour functions ///////////////////////////////////////////////////////////////////////////////////

int64_t BandRRT::nearestNeighbour(
        const bool use_forward_tree,
        const RRTNode& config)
{
    Stopwatch stopwatch;

    arc_helpers::DoNotOptimize(config);
    int64_t nn_idx = -1;
    if (use_forward_tree)
    {
        nn_idx = nearestBestNeighbourFullSpace(config);
    }
    else
    {
        nn_idx = nearestNeighbourRobotSpace(use_forward_tree, config).first;
    }
    arc_helpers::DoNotOptimize(nn_idx);

    const double nn_time = stopwatch(READ);
    total_nearest_neighbour_time_ += nn_time;

    assert(nn_idx >= 0);
    return nn_idx;
}

std::pair<int64_t, double> BandRRT::nearestNeighbourRobotSpace(
        const bool use_forward_tree,
        const RRTNode& config)
{
    RRTTree* tree = nullptr;
    std::shared_ptr<NNIndexType> nn_index = nullptr;
    std::vector<size_t>* nn_data_idx_to_tree_idx = nullptr;
    std::vector<float>* nn_raw_data = nullptr;
    size_t* manual_search_start_idx = nullptr;

    if (use_forward_tree)
    {
//        ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "Using forward tree");
        tree = &forward_tree_;
        nn_index = forward_nn_index_;
        nn_data_idx_to_tree_idx = &forward_nn_data_idx_to_tree_idx_,
        nn_raw_data = &forward_nn_raw_data_;
        manual_search_start_idx = &forward_next_idx_to_add_to_nn_dataset_;
    }
    else
    {
//        ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "Using backward tree");
        tree = &grippers_goal_set_;
        nn_index = goal_set_nn_index_;
        nn_data_idx_to_tree_idx = &goal_set_nn_data_idx_to_tree_idx_,
        nn_raw_data = &goal_set_nn_raw_data_;
        manual_search_start_idx = &goal_set_next_idx_to_add_to_nn_dataset_;
    }

//    ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "Querry:                    " << config.robotConfiguration().transpose());

    // Check if we should rebuild the NN Index
    if (!use_brute_force_nn_ &&
        *manual_search_start_idx + kd_tree_grow_threshold_ <= tree->size())
    {
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "Rebuilding FLANN index; forward tree? " << use_forward_tree);
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "RRT tree size: " << tree->size());
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "Initial manual search start idx: " << manual_search_start_idx);
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "Initial FLANN index size: " << nn_index->size());

        Stopwatch stopwatch;
        arc_helpers::DoNotOptimize(*manual_search_start_idx);
        const bool force_rebuild = false;
        rebuildNNIndex(nn_index, *nn_raw_data, *nn_data_idx_to_tree_idx, *tree, *manual_search_start_idx, force_rebuild);
        arc_helpers::DoNotOptimize(*manual_search_start_idx);
        const double index_building_time = stopwatch(READ);
        total_nearest_neighbour_index_building_time_ += index_building_time;

        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "Final manual search start idx: " << manual_search_start_idx);
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.nn", "Final FLANN index size: " << nn_index->size());
    }

    // If we have a FLANN index to search
    std::pair<int64_t, double> nearest(-1, std::numeric_limits<double>::infinity());
    if (!use_brute_force_nn_ && *manual_search_start_idx > 0)
    {
        Stopwatch stopwatch;
        arc_helpers::DoNotOptimize(config);
        nearest = getNearest(config.robotConfiguration(), *nn_index, *nn_data_idx_to_tree_idx);
        arc_helpers::DoNotOptimize(nearest);
        const double index_searching_time = stopwatch(READ);
        total_nearest_neighbour_index_searching_time_ += index_searching_time;
    }

    // If we have data that isn't in the FLANN index
    if (*manual_search_start_idx < tree->size())
    {
        Stopwatch stopwatch;
        arc_helpers::DoNotOptimize(*manual_search_start_idx);

        const std::pair<int64_t, double> linear_nearest = getNearest(config.robotConfiguration(), *tree, *manual_search_start_idx);
        if (linear_nearest.second < nearest.second)
        {
            nearest = linear_nearest;
        }
        arc_helpers::DoNotOptimize(nearest);
        const double linear_searching_time = stopwatch(READ);
        total_nearest_neighbour_linear_searching_time_ += linear_searching_time;
    }

    assert(nearest.first >= 0);
    return nearest;
}

int64_t BandRRT::nearestBestNeighbourFullSpace(
        const RRTNode &config)
{
    const std::pair<int64_t, double> nearest_robot_space = nearestNeighbourRobotSpace(true, config);

    Stopwatch stopwatch;
    arc_helpers::DoNotOptimize(nearest_robot_space);

    // If we have a FLANN index to search
    std::vector<std::pair<int64_t, double>> flann_radius_result;
    if (!use_brute_force_nn_ && forward_next_idx_to_add_to_nn_dataset_ > 0)
    {
        flann_radius_result = radiusSearch(
                    config.robotConfiguration(),
                    *forward_nn_index_,
                    forward_nn_data_idx_to_tree_idx_,
                    nearest_robot_space.second + band_max_dist2_);
    }

    // If we have data that isn't in the FLANN index
    std::vector<std::pair<int64_t, double>> linear_radius_result;
    if (forward_next_idx_to_add_to_nn_dataset_ < forward_tree_.size())
    {
        linear_radius_result = radiusSearch(config.robotConfiguration(), forward_tree_, forward_next_idx_to_add_to_nn_dataset_, nearest_robot_space.second + band_max_dist2_);
    }

    // Search both sets of results for the nearest neighbour in the
    // full configuration space, including the band
    const std::pair<int64_t, double> nearest_full_space =
            getNearestFullConfig(config, forward_tree_, band_distance2_scaling_factor_, flann_radius_result, linear_radius_result);

    arc_helpers::DoNotOptimize(nearest_full_space);
    const double radius_searching_time = stopwatch(READ);
    total_nearest_neighbour_radius_searching_time_ += radius_searching_time;

    // Perform a "best" subsearch if needed
    if (nearest_full_space.second <= best_near_radius2_ + band_max_dist2_)
    {
        auto flann_best_near_radius_result = flann_radius_result;
        auto linear_best_near_radius_result = linear_radius_result;

        // If the radius search that we already did is too small, then do a new search
        //   Note that due to the dual layer buisness, we are bloating the radius by a small amount (max band distance).
        arc_helpers::DoNotOptimize(nearest_robot_space);
        stopwatch(RESET);
        if (nearest_robot_space.second < best_near_radius2_)
        {
            if (!use_brute_force_nn_ && forward_next_idx_to_add_to_nn_dataset_ > 0)
            {
                flann_best_near_radius_result = radiusSearch(
                            config.robotConfiguration(),
                            *forward_nn_index_,
                            forward_nn_data_idx_to_tree_idx_,
                            best_near_radius2_);
            }
            if (forward_next_idx_to_add_to_nn_dataset_ < forward_tree_.size())
            {
                linear_best_near_radius_result = radiusSearch(config.robotConfiguration(), forward_tree_, forward_next_idx_to_add_to_nn_dataset_, best_near_radius2_);
            }
        }

        const std::pair<int64_t, double> best_full_space = getBestFullConfig(forward_tree_, flann_best_near_radius_result, linear_best_near_radius_result);
        arc_helpers::DoNotOptimize(best_full_space);

        assert(best_full_space.first >= 0);
        return best_full_space.first;
    }
    else
    {
        assert(nearest_full_space.first >= 0);
        return nearest_full_space.first;
    }
}

void BandRRT::rebuildNNIndex(
        std::shared_ptr<NNIndexType> index,
        std::vector<float>& nn_raw_data,
        std::vector<size_t>& nn_data_idx_to_tree_idx,
        const RRTTree& tree,
        size_t& new_data_start_idx,
        const bool force_rebuild)
{
    if (force_rebuild)
    {
        // If we are forcing a rebuild (for example, if we've just blacklisted part of the tree)
        // then new_data_start_idx is meaningless. We'll reset it to zero here, and clear the
        // vector that tracks the mapping between flann indices and tree indices
        new_data_start_idx = 0;
        nn_raw_data.clear();
        nn_data_idx_to_tree_idx.clear();
    }

    // These pointers are used to check if we need to rebuild the whole tree because the data moved,
    // or if we can just add points
    const float* initial_data_pointer = nn_raw_data.data();
    nn_raw_data.resize(total_dof_ * tree.size());
    const float* final_data_pointer = nn_raw_data.data();
    nn_data_idx_to_tree_idx.reserve(tree.size());

    size_t num_new_values = 0;
    for (size_t idx = new_data_start_idx; idx < tree.size(); ++idx)
    {
        if (!tree[idx].blacklisted_from_nn_search_)
        {
            size_t next_flann_idx = total_dof_ * nn_data_idx_to_tree_idx.size();

            const RRTRobotRepresentation& robot_config = tree[idx].robotConfiguration();
            const VectorXf robot_config_float = robot_config.cast<float>();
            memcpy(&nn_raw_data[next_flann_idx], robot_config_float.data(), total_dof_ * sizeof(float));
            nn_data_idx_to_tree_idx.push_back(idx);
            ++num_new_values;
        }
    }

    // TODO: clean up this potential re-allocation (reserve + push_back?)
    // Only some of the raw data was used, so resize down as needed
    nn_raw_data.resize(total_dof_ * nn_data_idx_to_tree_idx.size());

    // If the tree has already been initialized, and the raw data did not move in memory,
    // then we can just add the new points
    if ((new_data_start_idx == 0) || (initial_data_pointer != final_data_pointer))
    {
        const auto num_values = nn_raw_data.size() / total_dof_;
        flann::Matrix<float> data(nn_raw_data.data(), num_values, total_dof_);
        index->buildIndex(data);
    }
    else
    {
        flann::Matrix<float> data(&nn_raw_data[total_dof_ * new_data_start_idx], num_new_values, total_dof_);
        index->addPoints(data);
    }

    new_data_start_idx = tree.size();
}

//////// Tree extension functions //////////////////////////////////////////////////////////////////////////////////////

size_t BandRRT::connectForwardTree(const int64_t forward_tree_start_idx, const RRTNode& target, const bool is_random)
{
    constexpr bool allow_mistakes = true;
    constexpr bool fwd_prop_local_visualization_enabled = true;

    // Forward propagate towards the sampled target
    const size_t num_random_nodes_created =
            forwardPropogationFunction(
                forward_tree_,
                forward_tree_start_idx,
                target,
                allow_mistakes,
                fwd_prop_local_visualization_enabled);

    ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Nodes created: " << num_random_nodes_created << " Tree size: " << forward_tree_.size());

    if (SMMAP_RRT_VERBOSE)
    {
        for (size_t idx = forward_tree_.size() - num_random_nodes_created; idx < forward_tree_.size(); ++idx)
        {
            const RRTNode& node = forward_tree_[idx];
            ROS_INFO_STREAM_NAMED("rrt", "Node idx: " << idx << " Parent: " << node.parentIndex() << " Config: " << node.robotConfiguration().transpose());
        }
    }

    // Record statistics for the randomly sampled extensions
    if (num_random_nodes_created != 0)
    {
        if (is_random)
        {
            ++forward_random_samples_useful_;
        }
        else
        {
            ++forward_connection_attempts_useful_;
        }
    }
    else
    {
        if (is_random)
        {
            ++forward_random_samples_useless_;
        }
        else
        {
            ++forward_connection_attempts_useless_;
        }
    }

    return num_random_nodes_created;
}

size_t BandRRT::connectTreeToGrippersGoalSet(const int64_t last_node_idx_in_forward_tree_branch)
{
    constexpr bool nn_forward_tree = false;
    constexpr bool allow_mistakes = true;
    constexpr bool fwd_prop_local_visualization_enabled = true;

    const int64_t backward_tree_nearest_neighbour_idx = nearestNeighbour(
                nn_forward_tree, forward_tree_[last_node_idx_in_forward_tree_branch]);
    const RRTNode& target_in_backward_tree = grippers_goal_set_[backward_tree_nearest_neighbour_idx];

    const size_t num_goal_directed_nodes_created =
            forwardPropogationFunction(
                forward_tree_,
                last_node_idx_in_forward_tree_branch,
                target_in_backward_tree,
                allow_mistakes,
                fwd_prop_local_visualization_enabled);

    // Record statistics for the goal biased extension
    if (num_goal_directed_nodes_created != 0)
    {
        ++forward_connection_attempts_useful_;
    }
    else
    {
        ++forward_connection_attempts_useless_;
    }

    return num_goal_directed_nodes_created;
}

size_t BandRRT::forwardPropogationFunction(
        RRTTree& tree_to_extend,
        const int64_t& nearest_neighbor_idx,
        const RRTNode& target,
        const bool allow_mistakes,
        const bool visualization_enabled_locally)
{
    arc_helpers::DoNotOptimize(target.parentIndex());
    Stopwatch function_wide_stopwatch;
    Stopwatch stopwatch;

    const size_t nodes_at_start_of_propogation = tree_to_extend.size();
    const RRTNode nearest_neighbour = tree_to_extend[nearest_neighbor_idx];

    if (false && visualization_enabled_globally_ && visualization_enabled_locally)
    {
        vis_->visualizeCubes(
                    RRT_FORWARD_PROP_START_NS,
                    {nearest_neighbour.grippers().first.translation()},
                    Vector3d(0.01, 0.01, 0.01),
                    gripper_a_forward_tree_color_,
                    1);
        vis_->visualizeCubes(
                    RRT_FORWARD_PROP_START_NS,
                    {nearest_neighbour.grippers().second.translation()},
                    Vector3d(0.01, 0.01, 0.01),
                    gripper_b_forward_tree_color_,
                    5);

        nearest_neighbour.band()->visualize(
                    RRT_FORWARD_PROP_START_NS,
                    Visualizer::Green(),
                    Visualizer::Green(),
                    10,
                    true);

        vis_->visualizeCubes(
                    RRT_SAMPLE_NS,
                    {target.grippers().first.translation()},
                    Vector3d(0.01, 0.01, 0.01),
                    gripper_a_forward_tree_color_,
                    1);
        vis_->visualizeCubes(
                    RRT_SAMPLE_NS,
                    {target.grippers().second.translation()},
                    Vector3d(0.01, 0.01, 0.01),
                    gripper_b_forward_tree_color_,
                    5);
    }

    const size_t visualization_period = 100;
    const size_t force_vis_publish_period = 10; // Every this many markers, force a publish

    const RRTGrippersRepresentation& starting_grippers_poses = nearest_neighbour.grippers();
    const RRTRobotRepresentation& starting_robot_config = nearest_neighbour.robotConfiguration();

    // Extract the target robot configuration
    // Note that if planning only for the grippers, the target gripper positions
    //       are also stored in the robot configuration
    const RRTRobotRepresentation target_robot_config = target.robotConfiguration();
    if (!planning_for_whole_robot_)
    {
        const RRTGrippersRepresentation target_grippers_config = target.grippers();
        assert(target_grippers_config.first.translation() == target_robot_config.head<3>());
        assert(target_grippers_config.second.translation() == target_robot_config.tail<3>());
    }

    // Allocate space for potential children
    const double total_distance = RRTDistance::Distance(nearest_neighbour.robotConfiguration(), target_robot_config);
    const double max_step_size = planning_for_whole_robot_ ? max_robot_dof_step_size_ : max_gripper_step_size_;
    const uint32_t max_total_steps = (uint32_t)ceil(total_distance / max_step_size);
    tree_to_extend.reserve(tree_to_extend.size() + max_total_steps);

    int64_t parent_idx = nearest_neighbor_idx;
    uint32_t step_index = 0;
    bool split_happened = false;
    while (!split_happened && step_index < max_total_steps)
    {
        // We could be updating the child indices of this node later, so take the value by non-const reference
        RRTNode& prev_node = tree_to_extend[parent_idx];
        const RubberBand::Ptr& prev_band = prev_node.band();

        // Interpolate in joint space to find the next configuration of the robot
        const double ratio = std::min(1.0, (double)(step_index + 1) * max_step_size / total_distance);
        const RRTRobotRepresentation next_robot_config = Interpolate(starting_robot_config, target_robot_config, ratio);

        // Set the next grippers poses depending on needing to do FK or not
        // Also handle collision checking here as it does it differently for floating grippers vs a physical robot
        RRTGrippersRepresentation next_grippers_poses;
        if (planning_for_whole_robot_)
        {
            stopwatch(RESET);
            arc_helpers::DoNotOptimize(next_robot_config);
            robot_->setActiveDOFValues(next_robot_config);
            const AllGrippersSinglePose next_grippers_poses_vector = robot_->getGrippersPosesFunctionPointer();
            next_grippers_poses = {next_grippers_poses_vector[0], next_grippers_poses_vector[1]};
            arc_helpers::DoNotOptimize(next_grippers_poses);
            const double forward_kinematics_time = stopwatch(READ);
            total_forward_kinematics_time_ += forward_kinematics_time;

            // Check gripper position and rotation constraints
            {
                // Check if we rotated the grippers too much
                {
                    const double gripper_a_rotation_dist = EigenHelpers::Distance(starting_grippers_poses_.first.rotation(), next_grippers_poses.first.rotation());
                    const double gripper_b_rotation_dist = EigenHelpers::Distance(starting_grippers_poses_.second.rotation(), next_grippers_poses.second.rotation());
                    if (gripper_a_rotation_dist > max_gripper_rotation_ || gripper_b_rotation_dist > max_gripper_rotation_)
                    {
                        ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.prop", "Stopped due to excess gripper rotation");
                        break;
                    }
                }

                // If the grippers move outside of the planning arena then stop
                {
                    const RRTGrippersRepresentation task_frame_next_grippers_poses(
                                task_aligned_frame_inverse_transform_ * next_grippers_poses.first,
                                task_aligned_frame_inverse_transform_ * next_grippers_poses.second);

                    if ((task_frame_next_grippers_poses.first.translation().array() > task_aligned_upper_limits_.array()).any() ||
                        (task_frame_next_grippers_poses.first.translation().array() < task_aligned_lower_limits_.array()).any() ||
                        (task_frame_next_grippers_poses.second.translation().array() > task_aligned_upper_limits_.array()).any() ||
                        (task_frame_next_grippers_poses.second.translation().array() < task_aligned_lower_limits_.array()).any())
                    {
                        ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.prop", "Stopped due to moving outside of planning arena");
                        break;
                    }
                }
            }

            // Collision checking
            {
                stopwatch(RESET);
                arc_helpers::DoNotOptimize(next_grippers_poses);
                const bool in_collision_sdf =
                        (sdf_->EstimateDistance3d(next_grippers_poses.first.translation()).first < gripper_min_distance_to_obstacles_) ||
                        (sdf_->EstimateDistance3d(next_grippers_poses.second.translation()).first < gripper_min_distance_to_obstacles_) ||
                        (sdf_->DistanceToBoundary3d(next_grippers_poses.first.translation()).first < gripper_min_distance_to_obstacles_) ||
                        (sdf_->DistanceToBoundary3d(next_grippers_poses.second.translation()).first < gripper_min_distance_to_obstacles_);
                const bool in_collision_openrave = robot_->checkRobotCollision();
                const bool in_collision = in_collision_sdf || in_collision_openrave;
                arc_helpers::DoNotOptimize(in_collision);
                const double collision_check_time = stopwatch(READ);
                total_collision_check_time_ += collision_check_time;
                if (in_collision)
                {
                    ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.prop", "Stopped due to collision");
                    break;
                }
            }
        }
        else
        {
            next_grippers_poses = starting_grippers_poses;
            next_grippers_poses.first.translation() = next_robot_config.head<3>();
            next_grippers_poses.second.translation() = next_robot_config.tail<3>();

            // Collision checking
            {
                // If the grippers collide with each other, then return however far we are able to get
                {
                    stopwatch(RESET);
                    arc_helpers::DoNotOptimize(next_grippers_poses);
                    const bool in_collision = (next_grippers_poses.first.translation() - next_grippers_poses.second.translation()).norm() < gripper_min_distance_to_obstacles_;
                    arc_helpers::DoNotOptimize(in_collision);
                    const double collision_check_time_pt1 = stopwatch(READ);
                    total_collision_check_time_ += collision_check_time_pt1;
                    if (in_collision)
                    {
                        break;
                    }
                }

                // If the grippers enter collision with the environment, then return however far we were able to get
                {
                    stopwatch(RESET);
                    arc_helpers::DoNotOptimize(next_grippers_poses);
                    const bool in_collision =
                            (sdf_->EstimateDistance3d(next_grippers_poses.first.translation()).first < gripper_min_distance_to_obstacles_) ||
                            (sdf_->EstimateDistance3d(next_grippers_poses.second.translation()).first < gripper_min_distance_to_obstacles_) ||
                            (sdf_->DistanceToBoundary3d(next_grippers_poses.first.translation()).first < gripper_min_distance_to_obstacles_) ||
                            (sdf_->DistanceToBoundary3d(next_grippers_poses.second.translation()).first < gripper_min_distance_to_obstacles_);
                    arc_helpers::DoNotOptimize(in_collision);
                    const double collision_check_time_pt2 = stopwatch(READ);
                    total_collision_check_time_ += collision_check_time_pt2;
                    if (in_collision)
                    {
                        break;
                    }
                }
            }
        }

        // Generate next possible bands (no clustering)
        const auto next_bands = forwardPropogateBand(prev_band, next_grippers_poses, allow_mistakes);
        if (next_bands.size() == 0)
        {
            break;
        }
        split_happened = (next_bands.size() > 1);
        ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE && split_happened, "rrt.prop", "Split happend during forward propagation, loop should terminate");

        // The new configuation is valid, add it to the tree
        const RRTRobotRepresentation& prev_robot_config = prev_node.robotConfiguration();
        const double additional_cost = RRTDistance::Distance(prev_robot_config, next_robot_config);
        const double next_cost_to_come = prev_node.costToCome() + additional_cost;

        // Determine the total transion weight we have, used to determine probabilties in the next step
        double total_transition_weight = 0.0;
        for (size_t idx = 0; idx < next_bands.size(); ++idx)
        {
            total_transition_weight += next_bands[idx].second;
        }

        // Add each potential child to the tree
        for (const std::pair<RubberBand::Ptr, double>& child_band : next_bands)
        {
            auto next_band = child_band.first;

            const double p_transition = child_band.second / total_transition_weight;
            const RRTNode next_node(
                        next_grippers_poses,
                        next_robot_config,
                        next_band,
                        next_cost_to_come,
                        prev_node.pReachability() * p_transition,
                        p_transition,
                        parent_idx,
                        next_state_index_,
                        next_transition_index_,
                        split_happened ? next_split_index_ : -1);
            tree_to_extend.push_back(next_node);
            prev_node.addChildIndex(next_state_index_);

            ++next_state_index_;
        }

        // Note that parent_idx and step_index are only relevant at this point if there was no split
        parent_idx = (int64_t)tree_to_extend.size() - 1;
        ++step_index;
        ++next_transition_index_;
        if (split_happened)
        {
            ++next_split_index_;
        }
    }
    ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE && split_happened, "rrt.prop", "Split happend during forward propagation, this is immediately after the loop exit");

    const size_t nodes_at_end_of_propogation = tree_to_extend.size();
    const size_t nodes_created = nodes_at_end_of_propogation - nodes_at_start_of_propogation;

    // Decide if we should draw any new parts of the tree
    bool visualize = false;
    if (visualization_enabled_globally_ &&
        visualization_enabled_locally)
    {
        if (split_happened)
        {
            visualize = true;
        }
        else if (nodes_created > 0)
        {
            if (&tree_to_extend == &forward_tree_)
            {
                if (tree_to_extend.size() - forward_tree_next_visualized_node_ >= visualization_period)
                {
                    visualize = true;
                }
            }
            else
            {
                if (tree_to_extend.size() - backward_tree_next_visualized_node_ >= visualization_period)
                {
                    visualize = true;
                }
            }
        }
    }

    if (visualize)
    {
        const auto starting_idx = (&tree_to_extend == &forward_tree_)
                ? forward_tree_next_visualized_node_
                : backward_tree_next_visualized_node_;

        const auto& tree_a_color = (&tree_to_extend == &forward_tree_)
                ? gripper_a_forward_tree_color_
                : gripper_a_backward_tree_color_;

        const auto& tree_b_color = (&tree_to_extend == &forward_tree_)
                ? gripper_b_forward_tree_color_
                : gripper_b_backward_tree_color_;

        const auto& tree_a_ns = (&tree_to_extend == &forward_tree_)
                ? RRT_FORWARD_TREE_GRIPPER_A_NS
                : RRT_BACKWARD_TREE_GRIPPER_A_NS;

        const auto& tree_b_ns = (&tree_to_extend == &forward_tree_)
                ? RRT_FORWARD_TREE_GRIPPER_B_NS
                : RRT_BACKWARD_TREE_GRIPPER_B_NS;

        const bool draw_band = true;
        visualizeTree(
                    tree_to_extend,
                    starting_idx,
                    tree_a_ns,
                    tree_b_ns,
                    RRT_TREE_BAND_NS,
                    tree_marker_id_,
                    tree_marker_id_,
                    1,
                    tree_a_color,
                    tree_b_color,
                    band_tree_color_,
                    draw_band);
        ++tree_marker_id_;

        if (&tree_to_extend == &forward_tree_)
        {
            forward_tree_next_visualized_node_ = tree_to_extend.size();
        }
        else
        {
            backward_tree_next_visualized_node_ = tree_to_extend.size();
        }

        if (tree_marker_id_ % force_vis_publish_period == 0)
        {
            vis_->forcePublishNow(0.02);
            vis_->purgeMarkerList();
        }
    }

    arc_helpers::DoNotOptimize(nodes_created);
    const double everything_included_forward_propogation_time = function_wide_stopwatch(READ);
    total_everything_included_forward_propogation_time_ += everything_included_forward_propogation_time;

    return nodes_created;
}

std::vector<std::pair<RubberBand::Ptr, double>> BandRRT::forwardPropogateBand(
        const RubberBand::ConstPtr& starting_band,
        const RRTGrippersRepresentation& next_grippers_poses,
        const bool allow_mistakes)
{
    Stopwatch stopwatch;
    arc_helpers::DoNotOptimize(next_grippers_poses);

    const PairGripperPositions test_gripper_positions = ToGripperPositions(next_grippers_poses);
    const auto transitions = transition_estimator_->estimateTransitions(*starting_band, test_gripper_positions, allow_mistakes);
    ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.prop", transitions.size() << " transitions possible");
    if (SMMAP_RRT_VERBOSE)
    {
        for (size_t idx = 0; idx < transitions.size(); ++idx)
        {
            ROS_INFO_STREAM_NAMED("rrt.prop", "    Learned transition " << idx << " confidence: " << transitions[idx].second);
        }
    }

    arc_helpers::DoNotOptimize(transitions);
    const double band_forward_propogation_time = stopwatch(READ);
    total_band_forward_propogation_time_ += band_forward_propogation_time;

    return transitions;
}

//////// Goal check and node blacklist management functions ////////////////////////////////////////////////////////////

void BandRRT::checkNewStatesForGoal(const ssize_t num_nodes)
{
    bool force_nn_rebuild = false;
    // Check if any of the new nodes reached the goal
    for (size_t idx = forward_tree_.size() - num_nodes; idx < forward_tree_.size(); ++idx)
    {
        if (goalReached(forward_tree_[idx]))
        {
            force_nn_rebuild = true;
            forward_tree_[idx].setpGoalReachable(1.0);
            goalReachedCallback(idx);
            if (forward_tree_[idx].pReachability() < 1.0)
            {
                ROS_INFO_STREAM_NAMED("rrt", "Goal reached idx[" << idx << "]" <<", pReachability:         " << forward_tree_[idx].pReachability());
                ROS_INFO_STREAM_NAMED("rrt", "                       , pGoalReachable[root]:  " << forward_tree_[0].getpGoalReachable());
                visualizeBothTrees();
                visualizeBlacklist();
            }
//            PressKeyToContinue();
        }
    }
    if (force_nn_rebuild)
    {
//        if ((double)sample_history_buffer_.size() < 4e9)
//        {
//            ROS_INFO_STREAM_NAMED("rrt", "Sample history buffer size: " << sample_history_buffer_.size() << ". Saving to file.");
//            ZlibHelpers::CompressAndWriteToFile(sample_history_buffer_, GetLogFolder(*nh_) + "/sample_history_buffer.compressed");
//        }
//        else
//        {
//            ROS_ERROR_STREAM_NAMED("rrt", "Sample history buffer size: " << sample_history_buffer_.size() << ". Unable to save to file.");
//        }
        rebuildNNIndex(forward_nn_index_,
                       forward_nn_raw_data_,
                       forward_nn_data_idx_to_tree_idx_,
                       forward_tree_,
                       forward_next_idx_to_add_to_nn_dataset_,
                       force_nn_rebuild);
    }
}

bool BandRRT::goalReached(const RRTNode& node)
{
    // Check if the grippers are close enough to the goal position
    if (RRTDistance::Distance(node.grippers(), grippers_goal_poses_) > goal_reach_radius_)
    {
        return false;
    }

    // Check if the grippers have been rotated too far
    if (planning_for_whole_robot_)
    {
        const double gripper_a_rotation_dist = EigenHelpers::Distance(starting_grippers_poses_.first.rotation(), node.grippers().first.rotation());
        const double gripper_b_rotation_dist = EigenHelpers::Distance(starting_grippers_poses_.second.rotation(), node.grippers().second.rotation());
        if (gripper_a_rotation_dist > max_gripper_rotation_ || gripper_b_rotation_dist > max_gripper_rotation_)
        {
            return false;
        }

        if (visualization_enabled_globally_)
        {
            vis_->visualizeLineStrip(RRT_GOAL_TESTING_NS, node.band()->getVectorRepresentation(), Visualizer::White(), 1, 0.01);
        }
    }

    // Only accept paths that are different from those on the blacklist
    if (isBandFirstOrderVisibileToBlacklist(*node.band()))
    {
        return false;
    }

    return true;
}

bool BandRRT::isBandFirstOrderVisibileToBlacklist(const RubberBand& test_band)
{
    Stopwatch stopwatch;
    const bool is_first_order_visible = isBandFirstOrderVisibileToBlacklist_impl(test_band);
    const double first_order_vis_time = stopwatch(READ);
    total_first_order_vis_propogation_time_ += first_order_vis_time;

    return is_first_order_visible;
}

bool BandRRT::isBandFirstOrderVisibileToBlacklist_impl(const RubberBand& test_band) const
{
    for (size_t idx = 0; idx < blacklisted_goal_rubber_bands_.size(); idx++)
    {
        const RubberBand::ConstPtr& blacklisted_band = blacklisted_goal_rubber_bands_[idx];
        if (transition_estimator_->checkFirstOrderHomotopy(*blacklisted_band, test_band))
        {
            return true;
        }
    }

    return false;
}

void BandRRT::goalReachedCallback(const int64_t node_idx)
{
    assert(CheckTreeLinkage(forward_tree_));

    // Backtrack through the tree until we reach the root of the current "goal branch"
    // - i.e.; find the first relevant split
    int64_t current_index = node_idx;
    while (!isRootOfGoalBranch(current_index))
    {
        current_index = forward_tree_[current_index].parentIndex();
    }
    assert(current_index >= 0 && current_index < (int64_t)forward_tree_.size());
    const int64_t root_index = current_index;



//    if (root_index != 0)
//    {
//        const auto log_folder = ROSHelpers::GetParamRequiredDebugLog<std::string>(*nh_, "log_folder", __func__);
//        if (!log_folder.Valid())
//        {
//            throw_arc_exception(std::invalid_argument, "Unable to load log_folder from parameter server");
//        }
//        arc_utilities::CreateDirectory(log_folder.GetImmutable());
//        const auto file_name_prefix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "path_file_name_prefix", __func__);
//        if (!file_name_prefix.Valid())
//        {
//            throw_arc_exception(std::invalid_argument, "Unable to load path_file_name_prefix from parameter server");
//        }

//        const std::string file_name_suffix = arc_helpers::GetCurrentTimeAsString();
//        const std::string file_name = file_name_prefix.GetImmutable() + "__before_blacklisting_root_idx_" + std::to_string(root_index) + "__" + file_name_suffix + ".compressed";
//        std::string file_path = log_folder.GetImmutable() + file_name;
//        storeTree(forward_tree_, file_path);
//    }



    // First, blacklist the root and all children thereof
    blacklistGoalBranch(root_index);

    // Next, update p_goal_reachable_ for every node from the current node to the root of the entire forward_tree_
    ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Node " << node_idx << " reached goal, updating pGoalReachable from this node to root.");
    updatePGoalReachable(node_idx);

    ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Root p_goal_reachable: " << forward_tree_[0].getpGoalReachable());
}

bool BandRRT::isRootOfGoalBranch(const int64_t node_idx) const
{
    const RRTNode& node = forward_tree_[node_idx];

    // Two possible ways a state can be the root
    // 1) The node itself is the root of the tree
    const bool is_root_of_tree = (node.stateIndex() == 0);

    // 2) The transition leading to the state is the result of an unresolved split
    //    An unresolved split is one where there is at least one other child of this same
    //    split that has not yet reached the goal
    const bool is_child_of_split = (node.splitIndex() > 0) ? true : false;
    bool is_unresolved_split = false;
    if (is_child_of_split)
    {
        const RRTNode& parent = forward_tree_[node.parentIndex()];
        const std::vector<int64_t> parents_children = parent.childIndices();
        bool other_children_resolved = true;
        for (size_t idx = 0; idx < parents_children.size(); ++idx)
        {
            const RRTNode& other_child = forward_tree_[parents_children[idx]];
            if (other_child.splitIndex() == node.splitIndex() && !other_child.blacklisted_from_nn_search_)
            {
                other_children_resolved = false;
                break;
            }
        }

        if (other_children_resolved)
        {
            is_unresolved_split = false;
        }
        else
        {
            is_unresolved_split = true;
        }
    }
    return is_root_of_tree || is_unresolved_split;
}

void BandRRT::blacklistGoalBranch(const int64_t root_idx)
{
    if (root_idx < 0)
    {
        ROS_ERROR_STREAM_NAMED("rrt", "Asked to blacklist a negative root_idx; this should not be possible: " << root_idx);
        return;
    }
    else if (root_idx == 0)
    {
        ROS_WARN_NAMED("rrt", "Asked to blacklist the start node, blacklisting all children but not the root itself");
        const auto children = forward_tree_[0].childIndices();
        for (size_t idx = 0; idx < children.size(); ++idx)
        {
            blacklistGoalBranch(children[idx]);
        }
    }
    else
    {
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Blacklist branch starting with idx: " << root_idx);
        // Get the current node and blacklist it
        RRTNode& current_node = forward_tree_[(size_t)root_idx];
        current_node.blacklisted_from_nn_search_ = true;
        // Recursively blacklist each child
        const std::vector<int64_t>& child_indices = current_node.childIndices();
        for (size_t idx = 0; idx < child_indices.size(); ++idx)
        {
            blacklistGoalBranch(child_indices[idx]);
        }
    }
}

void BandRRT::updatePGoalReachable(const int64_t node_idx)
{
    RRTNode& new_goal = forward_tree_[node_idx];
    // Make sure something hasn't gone wrong
    if (new_goal.getpGoalReachable() != 1.0)
    {
        throw_arc_exception(std::runtime_error, "new_goal should have p_goal_reachable == 1.0 as it itself passes the goal check");
    }

    // Backtrack up the tree, updating states as we go
    int64_t current_idx = new_goal.parentIndex();
    while (current_idx >= 0)
    {
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Updating pGoalReachable for node " << current_idx);

        RRTNode& current_node = forward_tree_[current_idx];
        const auto& child_indices = current_node.childIndices();

        // Check all the children of the current node, and update the node's goal reached probability accordingly
        //
        // Naively, the goal reached probability of a node is the maximum of the child goal reached probabilities;
        // intuitively, the probability of reaching the goal is that of reaching the goal if we follow the best child.
        //
        // HOWEVER - the existence of "split" child states, where multiple states result from a single control input,
        // makes this more compilcated. For split child states, the goal reached probability of the split is the sum
        // over every split option of (split goal probability * probability of split)
        //
        // We can identify split nodes as children which share a transition id
        // First, we go through the children and separate them based on transition id (this puts all the children of a
        // transition together in one place)
        std::map<int64_t, std::vector<int64_t>> effective_child_branches;
        for (size_t idx = 0; idx < child_indices.size(); ++idx)
        {
            const int64_t current_child_index = child_indices[idx];
            const int64_t child_transition_idx = forward_tree_[(size_t)current_child_index].transitionIndex();
            effective_child_branches[child_transition_idx].push_back(current_child_index);
        }

        // Now that we have the splits separated out, compute the goal probability of each transition,
        // keeping only the largest
        double p_goal_reachable = 0.0;
        for (auto itr = effective_child_branches.begin(); itr != effective_child_branches.end(); ++itr)
        {
            const std::vector<int64_t>& current_transition_children = itr->second;
            double p_goal_reachable_current_transition = 0.0;
            for (size_t child_idx = 0; child_idx < current_transition_children.size(); ++child_idx)
            {
                const RRTNode& child = forward_tree_[current_transition_children[child_idx]];
                p_goal_reachable_current_transition += child.pTransition() * child.getpGoalReachable();
            }
            ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Transition pGoalReachable " << p_goal_reachable_current_transition);
            p_goal_reachable = std::max(p_goal_reachable, p_goal_reachable_current_transition);
        }

        // Check for numerical errors and store the final value
        if ((p_goal_reachable >= 0.999) && (p_goal_reachable <= 1.001) && (p_goal_reachable != 1.0))
        {
            ROS_WARN_STREAM_NAMED("rrt", "Total P(goal reached) = " << p_goal_reachable << ". Probably a numerical error, rounding to 1.0");
            p_goal_reachable = 1.0;
        }
        current_node.setpGoalReachable(p_goal_reachable);
        ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt", "Setting  pGoalReachable for node " << current_idx << " to " << p_goal_reachable);

        // Move on to the parent of this node
        current_idx = current_node.parentIndex();
    }
}

//////// Shortcut smoothing functions //////////////////////////////////////////////////////////////////////////////////

static VectorVector3d findFirstGripperWaypoints(
        const RRTTree& path,
        const size_t start_index,
        const size_t end_index)
{
    assert(start_index < end_index);
    assert(end_index < path.size());

    // The start of the path is clearly the first 'kink'
    VectorVector3d gripper_path_kinks(1, path[start_index].grippers().first.translation());

    size_t last_kink = start_index;
    Vector3d last_kink_gripper_position = path[last_kink].grippers().first.translation();
    double path_distance = 0.0;

    // We don't include the last index because it is clearly the last 'kink'
    for (size_t idx = start_index; idx < end_index - 1; ++idx)
    {
        const Vector3d& current_gripper_position = path[idx].grippers().first.translation();
        const Vector3d& next_gripper_position    = path[idx + 1].grippers().first.translation();
        path_distance += (next_gripper_position - current_gripper_position).norm();
        const double straight_line_distance = (next_gripper_position - last_kink_gripper_position).norm();

        // If the straight line distance between the start and the next gripper does not match the path distance, then the current node is a kink
        if (!IsApprox(straight_line_distance, path_distance, 1e-6))
        {
            last_kink = idx;
            last_kink_gripper_position = path[last_kink].grippers().first.translation();
            path_distance = (next_gripper_position - current_gripper_position).norm();
            gripper_path_kinks.push_back(last_kink_gripper_position);
        }
    }
    gripper_path_kinks.push_back(path[end_index].grippers().first.translation());

    return gripper_path_kinks;
}

static VectorVector3d findSecondGripperWaypoints(
        const RRTTree& path,
        const size_t start_index,
        const size_t end_index)
{
    assert(start_index < end_index);
    assert(end_index < path.size());

    // The start of the path is clearly the first 'kink'
    VectorVector3d gripper_path_kinks(1, path[start_index].grippers().second.translation());

    size_t last_kink = start_index;
    Vector3d last_kink_gripper_position = path[last_kink].grippers().second.translation();
    double path_distance = 0;

    // We don't include the last index because it is clearly the last 'kink'
    for (size_t idx = start_index; idx < end_index - 1; ++idx)
    {
        const Vector3d& current_gripper_position = path[idx].grippers().second.translation();
        const Vector3d& next_gripper_position    = path[idx + 1].grippers().second.translation();
        path_distance += (next_gripper_position - current_gripper_position).norm();
        const double straight_line_distance = (next_gripper_position - last_kink_gripper_position).norm();

        // If the straight line distance between the start and the next gripper does not match the path distance, then the current node is a kink
        if (!IsApprox(straight_line_distance, path_distance, 1e-6))
        {
            last_kink = idx;
            last_kink_gripper_position = current_gripper_position;
            path_distance = (next_gripper_position - current_gripper_position).norm();
            gripper_path_kinks.push_back(last_kink_gripper_position);
        }
    }
    gripper_path_kinks.push_back(path[end_index].grippers().second.translation());

    return gripper_path_kinks;
}

static VectorVector3d createOtherGripperWaypoints(
        const VectorVector3d& given_gripper_waypoints,
        const Vector3d& start_point,
        const Vector3d& end_point)
{
    const size_t num_waypoints = given_gripper_waypoints.size();
    assert(num_waypoints >= 2);

    VectorVector3d other_gripper_waypoints;
    other_gripper_waypoints.reserve(num_waypoints);
    other_gripper_waypoints.push_back(start_point);

    // We will need to "space out" the distance between start_point and end_point to match those of the given waypoints
    // Note that we've already inserted the first waypoint, and we'll insert the last manually as well
    const std::vector<double> cummulative_distances = CalculateCumulativeDistances(given_gripper_waypoints);
    for (size_t idx = 1; idx < num_waypoints - 1; ++idx)
    {
        const double ratio = cummulative_distances[idx] / cummulative_distances.back();
        const auto next_waypoint = Interpolate(start_point, end_point, ratio);
        other_gripper_waypoints.push_back(next_waypoint);
    }
    other_gripper_waypoints.push_back(end_point);

    assert(other_gripper_waypoints.size() == num_waypoints);
    return other_gripper_waypoints;
}

// To smooth a policy, we smooth each segment independently,
// making sure that the transitions between segments are still valid
void BandRRT::shortcutSmoothPolicy(
        RRTPolicy& policy,
        const bool visualization_enabled_locally)
{
    Stopwatch function_wide_stopwatch;

    transition_estimator_->resetStatistics();
    total_forward_kinematics_time_ = 0.0;
    total_projection_time_ = 0.0;
    total_collision_check_time_ = 0.0;
    total_band_forward_propogation_time_ = 0.0;
    total_first_order_vis_propogation_time_ = 0.0;
    total_everything_included_forward_propogation_time_ = 0.0;

    for (size_t segment_idx = 0; segment_idx < policy.size(); ++segment_idx)
    {
        RRTPath& segment = policy[segment_idx].first;
        const std::vector<size_t>& child_segment_indices = policy[segment_idx].second;

        // Segments that have no children must end at the goal, so we need to maintain that when smoothing
        const bool maintain_goal_reach_invariant = (child_segment_indices.size() == 0);

        shortcutSmoothPath(
                    segment,
                    maintain_goal_reach_invariant,
                    visualization_enabled_locally,
                    (int32_t)segment_idx + 1);
    }

    // Record the statistics and return the result
    const double smoothing_time = function_wide_stopwatch(READ);

//    smoothing_statistics_["smoothing0_failed_iterations                            "] = (double)failed_iterations;
//    smoothing_statistics_["smoothing1_iterations                                   "] = (double)num_iterations;
    smoothing_statistics_["smoothing2_forward_propogation_fk_time                  "] = total_forward_kinematics_time_;
    smoothing_statistics_["smoothing3_forward_propogation_crrt_projection_time     "] = total_projection_time_;
    smoothing_statistics_["smoothing4_forward_propogation_collision_check_time     "] = total_collision_check_time_;
    smoothing_statistics_["smoothing5_forward_propogation_band_sim_time            "] = total_band_forward_propogation_time_ - transition_estimator_->classifierTime();
    smoothing_statistics_["smoothing6_classifier                                   "] = transition_estimator_->classifierTime();
    smoothing_statistics_["smoothing7_forward_propogation_first_order_vis_time     "] = total_first_order_vis_propogation_time_;
    smoothing_statistics_["smoothing8_forward_propogation_everything_included_time "] = total_everything_included_forward_propogation_time_;
    smoothing_statistics_["smoothing9_total_time                                   "] = smoothing_time;

    smoothing_statistics_["smoothing_classifier_num0_band_wierdness                "] = (double)transition_estimator_->numBandWeirdness();
    smoothing_statistics_["smoothing_classifier_num1_band_safe                     "] = (double)transition_estimator_->numBandSafe();
    smoothing_statistics_["smoothing_classifier_num2_band_overstretch              "] = (double)transition_estimator_->numBandOverstretch();
    smoothing_statistics_["smoothing_classifier_num3_band_no_mistake               "] = (double)transition_estimator_->numNoMistakes();
    smoothing_statistics_["smoothing_classifier_num4_band_mistakes                 "] = (double)transition_estimator_->numMistakes();
    smoothing_statistics_["smoothing_classifier_num5_band_accepted_mistakes        "] = (double)transition_estimator_->numAcceptedMistakes();
}

void BandRRT::shortcutSmoothPath(
        RRTPath& path,
        const bool maintain_goal_reach_invariant,
        const bool visualization_enabled_locally,
        const int32_t visualization_idx)
{
    if (path.size() < 2)
    {
        return;
    }

    // We should either be maintaining the goal reach invariant,
    // or the transition targets, not both, so make sure that's true
//    assert(maintain_goal_reach_invariant ^ (transition_targets.size() == 0));
    // For the time being, just use the band immediately before the last
    // transition as the invariant if we are not using the "goal reach" invariant
    const RRTNode second_last_original_node = *(path.end() - 2);

    uint32_t num_iterations = 0;

    // The main smoothing loop
    while (path.size() > 2 &&
           num_iterations < max_smoothing_iterations_)
    {
        ++num_iterations;

        ///////////////////// Determine which nodes to try to shortcut between /////////////////////////////////////////

        const int64_t base_index = (int64_t)std::uniform_int_distribution<size_t>(0, path.size() - 1)(*generator_);

        // Compute the offset index
        // We want to sample the start and goal slightly more frequently, so allow "overshoots" of endpoints for the offset
        const int64_t min_delta = std::max(-base_index - max_shortcut_index_distance_ / 10, -max_shortcut_index_distance_);
        const int64_t max_delta = std::min((int64_t)path.size() - base_index - 1 + max_shortcut_index_distance_ / 10, max_shortcut_index_distance_);
        const int64_t offset_delta = std::uniform_int_distribution<int64_t>(min_delta, max_delta)(*generator_);
        // Clamp to the boundaries of the current path
        const int64_t second_index = arc_helpers::ClampValue(base_index + offset_delta, (int64_t)0, (int64_t)path.size() - 1);

        // Get start and end indices to establish the direction of the shortcut
        const size_t smoothing_start_index = (size_t)std::min(base_index, second_index);
        const size_t smoothing_end_index = (size_t)std::max(base_index, second_index);

        const auto& smoothing_start_config = path[smoothing_start_index];
        const auto& smoothing_end_config = path[smoothing_end_index];

        ///////////////////// Determine if a shortcut is even possible /////////////////////////////////////////////////

        // We know start_index <= end_index, this essentially checks if start == end or start + 1 == end
        if (smoothing_start_index + 1 >= smoothing_end_index)
        {
            continue;
        }
        // Distance checks are determined once we know what type of smoothing we are attempting, first, second, or both

        ///////////////////// Attempte a shortcut //////////////////////////////////////////////////////////////////////

        // Create structures to hold the results which will get filled by each part of the if/else chain
        RRTPath smoothed_segment;
        smoothed_segment.reserve(256);
        std::pair<bool, RRTTree> end_of_smoothing_to_goal_results;

        constexpr bool allow_mistakes = false;
        constexpr bool fwd_prop_local_visualization_enabled = false;

        if (planning_for_whole_robot_)
        {
            ROS_ERROR_NAMED("rrt", "smoothing for whole robot is not policy aware");
//            assert(false && "Not updated based on using policies");
            // Check if the edge possibly can be smoothed
            const double minimum_distance = RRTDistance::Distance(smoothing_start_config.robotConfiguration(), smoothing_end_config.robotConfiguration());
            const double path_distance = RRTDistance::RobotPathDistance(path, smoothing_start_index, smoothing_end_index);
            // Essentially this checks if there is a kink in the path
            if (IsApprox(path_distance, minimum_distance, 1e-6))
            {
//                std::cout << "No smoothing possible, continuing\n";
                continue;
            }

            // Forward simulate the rubber band along the straight line between gripper/robot positions
            const int64_t start_idx = 0;
            smoothed_segment.push_back(smoothing_start_config);
            forwardPropogationFunction(smoothed_segment, start_idx, smoothing_end_config, allow_mistakes, fwd_prop_local_visualization_enabled);

            // Check if the rubber band gets overstretched while propogating the grippers/robot on the new path
            const auto& target_robot_configuration = smoothing_end_config.robotConfiguration();
            const auto& last_robot_configuration = smoothed_segment.back().robotConfiguration();

            if (!robotConfigurationsAreApproximatelyEqual(last_robot_configuration, target_robot_configuration))
            {
//                std::cout << "Shortcut failed, continuing"
//                          << "   Robot configuration equal? " << robotConfigurationsAreApproximatelyEqual(last_robot_configuration, target_robot_configuration)
//                          << "\n";
                continue;
            }

            // We still need to check that the rubber band can still reach the goal correctly from this state,
            // so we'll forward propogate along the rest of the trajectory to check feasibility
            end_of_smoothing_to_goal_results = forwardSimulateGrippersPath(path, smoothing_end_index, *smoothed_segment.back().band());
        }
        else
        {
            // First determine which type of smoothing we are doing, both grippers, or a single gripper
            // On a 1 or a 2, smooth both grippers,
            // On a 3 smooth the first gripper only,
            // On a 4 smooth the second gripper only
            const int smoothing_type = uniform_shortcut_smoothing_int_distribution_(*generator_);

            if (smoothing_type == 1 || smoothing_type == 2)
            {
                // Check if the edge possibly can be smoothed
                const double minimum_distance = RRTDistance::Distance(smoothing_start_config.grippers(), smoothing_end_config.grippers());
                const double path_distance = RRTDistance::GrippersPathDistance(path, smoothing_start_index, smoothing_end_index);
                // Essentially this checks if there is a kink in the path
                if (IsApprox(path_distance, minimum_distance, 1e-6))
                {
                    ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.smoothing", "Skipping smoothing attempt as there is very little kink in sampled sub path");
                    continue;
                }

                if (visualization_enabled_locally)
                {
                    VectorVector3d gripper_a_cubes;
                    VectorVector3d gripper_b_cubes;

                    gripper_a_cubes.push_back(smoothing_start_config.grippers().first.translation());
                    gripper_a_cubes.push_back(smoothing_end_config.grippers().first.translation());

                    gripper_b_cubes.push_back(smoothing_start_config.grippers().second.translation());
                    gripper_b_cubes.push_back(smoothing_end_config.grippers().second.translation());

                    vis_->visualizeCubes(RRT_SMOOTHING_GRIPPER_A_NS, gripper_a_cubes, Vector3d(0.01, 0.01, 0.01), gripper_a_forward_tree_color_, 1);
                    vis_->visualizeCubes(RRT_SMOOTHING_GRIPPER_B_NS, gripper_b_cubes, Vector3d(0.01, 0.01, 0.01), gripper_b_forward_tree_color_, 2);
                }

                // Forward simulate the rubber band along the straight line between gripper positions
                const int64_t start_idx = 0;
                smoothed_segment.push_back(smoothing_start_config);
                forwardPropogationFunction(smoothed_segment, start_idx, smoothing_end_config, allow_mistakes, fwd_prop_local_visualization_enabled);
            }
            else if (smoothing_type == 3 || smoothing_type == 4)
            {
                // Once we know the fixed waypoints, then we do smoothing between these waypoints
                const VectorVector3d current_waypoints_first_gripper =
                        findFirstGripperWaypoints(path, smoothing_start_index, smoothing_end_index);
                const VectorVector3d current_waypoints_second_gripper =
                        findSecondGripperWaypoints(path, smoothing_start_index, smoothing_end_index);

                VectorVector3d target_waypoints_first_gripper;
                VectorVector3d target_waypoints_second_gripper;
                // Smooth the first gripper
                if (smoothing_type == 3)
                {
                    const auto& first_gripper_start_pos = smoothing_start_config.grippers().first.translation();
                    const auto& first_gripper_end_pos = smoothing_end_config.grippers().first.translation();

                    // Check if there is room for improvement for the first gripper
                    const double minimum_distance = (first_gripper_end_pos - first_gripper_start_pos).norm();
                    const double path_distance = CalculateTotalDistance(current_waypoints_first_gripper);
                    // Essentially this checks if there is a kink in the path
                    if (IsApprox(path_distance, minimum_distance, 1e-6))
                    {
                        ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.smoothing", "Skipping smoothing attempt as there is very little kink in sampled sub path");
                        continue;
                    }

                    // Follow the waypoints of the second gripper
                    target_waypoints_second_gripper = current_waypoints_second_gripper;
                    // Create new waypoints in a stright line for the first gripper
                    target_waypoints_first_gripper = createOtherGripperWaypoints(
                                target_waypoints_second_gripper,
                                first_gripper_start_pos,
                                first_gripper_end_pos);
                }
                // Smooth the second gripper
                else
                {
                    const auto& second_gripper_start_pos = smoothing_start_config.grippers().second.translation();
                    const auto& second_gripper_end_pos = smoothing_end_config.grippers().second.translation();

                    // Check if there is room for improvement for the second gripper
                    const double minimum_distance = (second_gripper_end_pos - second_gripper_start_pos).norm();
                    const double path_distance = CalculateTotalDistance(current_waypoints_first_gripper);
                    // Essentially this checks if there is a kink in the path
                    if (IsApprox(path_distance, minimum_distance, 1e-6))
                    {
                        ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.smoothing", "Skipping smoothing attempt as there is very little kink in sampled sub path");
                        continue;
                    }

                    // Follow the waypoints of the first gripper
                    target_waypoints_first_gripper = current_waypoints_first_gripper;
                    // Create new waypoints in a stright line for the second gripper
                    target_waypoints_second_gripper = createOtherGripperWaypoints(
                                target_waypoints_first_gripper,
                                second_gripper_start_pos,
                                second_gripper_end_pos);
                }

                assert(target_waypoints_first_gripper.size() == target_waypoints_second_gripper.size());
                const size_t num_waypoints = target_waypoints_first_gripper.size();

                if (visualization_enabled_locally)
                {
                    VectorVector3d gripper_a_cubes = {smoothing_start_config.grippers().first.translation()};
                    VectorVector3d gripper_b_cubes = {smoothing_start_config.grippers().second.translation()};

                    gripper_a_cubes.insert(gripper_a_cubes.end(), target_waypoints_first_gripper.begin(), target_waypoints_first_gripper.end());
                    gripper_b_cubes.insert(gripper_b_cubes.end(), target_waypoints_second_gripper.begin(), target_waypoints_second_gripper.end());

                    vis_->visualizeCubes(RRT_SMOOTHING_GRIPPER_A_NS, gripper_a_cubes, Vector3d(0.01, 0.01, 0.01), gripper_a_forward_tree_color_, 1);
                    vis_->visualizeCubes(RRT_SMOOTHING_GRIPPER_B_NS, gripper_b_cubes, Vector3d(0.01, 0.01, 0.01), gripper_b_forward_tree_color_, 2);
                }

                // Now that we have the waypoints, start building the smoothed path
                smoothed_segment.push_back(smoothing_start_config);
                for (size_t waypoint_idx = 1; waypoint_idx < num_waypoints; ++waypoint_idx)
                {
                    RRTGrippersRepresentation target_poses = path.front().grippers();
                    target_poses.first.translation() = target_waypoints_first_gripper[waypoint_idx];
                    target_poses.second.translation() = target_waypoints_second_gripper[waypoint_idx];
                    RRTRobotRepresentation target_config(6);
                    target_config << target_waypoints_first_gripper[waypoint_idx], target_waypoints_second_gripper[waypoint_idx];
                    const RRTNode forward_prop_target_config(
                                target_poses,
                                target_config,
                                path.front().band());

                    const int64_t start_idx = (int64_t)smoothed_segment.size() - 1;
                    forwardPropogationFunction(smoothed_segment, start_idx, forward_prop_target_config, allow_mistakes, fwd_prop_local_visualization_enabled);
                    // Exit early if we hit any of "failure" conditions
                    // Check if the we hit a split
                    if (smoothed_segment.back().splitIndex() >= 0)
                    {
                        ROS_WARN_NAMED("rrt.smoothing", "Split in path while smoothing single gripper, rejecting smoothing attempt");
                        break;
                    }

                    // Check if the rubber band gets overstretched while propogating the grippers on the new path
                    const auto& target_gripper_position = forward_prop_target_config.grippers();
                    const auto& last_gripper_position = smoothed_segment.back().grippers();
                    if (!gripperPositionsAreApproximatelyEqual(last_gripper_position, target_gripper_position))
                    {
                        break;
                    }
                }
            }
            else
            {
                assert(false && "Smoothing type was something other than [1, 4], this ougth to be impossible");
            }

            // Check if the we hit a split
            if (smoothed_segment.back().splitIndex() >= 0)
            {
                ROS_WARN_NAMED("rrt.smoothing", "Shortcut failed, split happened during smoothing");
                continue;
            }

            // Check if the rubber band gets overstretched while propogating the grippers on the new path
            const auto& target_gripper_position = smoothing_end_config.grippers();
            const auto& last_gripper_position = smoothed_segment.back().grippers();
            if (!gripperPositionsAreApproximatelyEqual(last_gripper_position, target_gripper_position))
            {
                ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.smoothing", "Shortcut failed, gripper positions don't match targets, thus the band got overstretched");
                continue;
            }

            // We still need to check that the rubber band can still reach the goal correctly from this state,
            // so we'll forward propogate along the rest of the trajectory to check feasibility
            end_of_smoothing_to_goal_results = forwardSimulateGrippersPath(path, smoothing_end_index, *smoothed_segment.back().band());
        }

        const bool final_band_at_goal_success = end_of_smoothing_to_goal_results.first;
        const auto& end_of_smoothing_to_goal_path = end_of_smoothing_to_goal_results.second;

        // Check if the rubber band gets overstretched or ends up in a blacklisted first order
        // homotopy class while following the tail of the starting trajectory
        if (maintain_goal_reach_invariant)
        {
            const auto& final_node_of_smoothing = end_of_smoothing_to_goal_path.back();
            const bool final_band_visible_to_blacklist = isBandFirstOrderVisibileToBlacklist(*final_node_of_smoothing.band());
            if (!final_band_at_goal_success || final_band_visible_to_blacklist)
            {
                ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.smoothing", "Shortcut failed"
                                                                     << "    Band at goal? " << final_band_at_goal_success
                                                                     << "    Band visible? " << final_band_visible_to_blacklist);
                continue;
            }
        }
        // Here we check that the 2nd last band state still matches close enough
        // to approximately ensure that all previously applied transitions are still valid
        else
        {
            RRTNode second_last_smoothed_node;
            if (end_of_smoothing_to_goal_path.size() >= 2)
            {
                second_last_smoothed_node = *(end_of_smoothing_to_goal_path.end() - 2);
            }
            else
            {
                assert(smoothed_segment.size() >= 2);
                second_last_smoothed_node = *(smoothed_segment.end() - 2);
            }

            const double band_dist = second_last_original_node.band()->distance(*second_last_smoothed_node.band());
            if (band_dist > smoothing_band_dist_threshold_)
            {
                ROS_INFO_STREAM_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.smoothing", "Shortcut failed"
                                                                     << "    Distance between original 2nd last band and new band: " << band_dist);
                continue;
            }
        }

        ///////////////////// Smoothing success - Create the new smoothed path /////////////////////////////////////////
        {
            ROS_INFO_COND_NAMED(SMMAP_RRT_VERBOSE, "rrt.smoothing", "Shortcut valid, accepting");

            // Allocate space for the total smoothed path
            RRTPath smoothed_path;
            smoothed_path.reserve((smoothing_start_index  + 1) + (smoothed_segment.size() - 1) + (end_of_smoothing_to_goal_path.size() - 1));

            // Insert the starting unchanged part of the path
            smoothed_path.insert(smoothed_path.end(), path.begin(), path.begin() + smoothing_start_index + 1);

            // Insert the smoothed portion (note that we are removing the overlap as we go here)
            smoothed_path.insert(smoothed_path.end(), smoothed_segment.begin() + 1, smoothed_segment.end());

            // Insert the changed end of the path with the new rubber band - gripper/robot positions are identical (note that we are removing the overlap as we go here)
            smoothed_path.insert(smoothed_path.end(), end_of_smoothing_to_goal_path.begin() + 1, end_of_smoothing_to_goal_path.end());

            // Record the change and re-visualize
            path = smoothed_path;
        }

        if (visualization_enabled_globally_ && visualization_enabled_locally)
        {
            visualizePath(path, "", visualization_idx, false);
            vis_->forcePublishNow(0.01);
        }
    }
}

/**
 * @brief RRTHelper::forwardSimulateGrippersPath
 *   Forward simulates the rubber band starting the grippers at position path[start_index]
 *   and ending at the end of the path. Used by rrtShortcutSmooth.
 * @param rubber_band
 * @param path
 * @param start_index
 * @param end_index
 * @return A vector of RRTConfig of at most (end_index - start_index) elements; includes path[start_index].
 */
std::pair<bool, RRTTree> BandRRT::forwardSimulateGrippersPath(
        const RRTTree& path,
        const size_t start_index,
        RubberBand rubber_band)
{
    Stopwatch function_wide_stopwatch;
    Stopwatch stopwatch;

    assert(start_index < path.size());

    // Verify that the endpoints of the rubber band match the start of the grippers path
    if (!bandEndpointsMatchGripperPositions(rubber_band, path[start_index].grippers()))
    {
        std::cerr << "Inside forwardSimulateGrippersPath\n";
        std::cerr << "initial rubber band endpoints:\n"
                  << PrettyPrint::PrettyPrint(rubber_band.getEndpoints()) << std::endl;

        std::cerr << "path gripper positions:\n"
                  << PrettyPrint::PrettyPrint(path[start_index].grippers()) << std::endl;

        assert(false && "Band endpoints do not match recorded gripper positions");
    }

    // Collect the results for use by the shortcutSmoothPath function
    RRTTree resulting_path;
    // Put the start position on the path
    {
        resulting_path.reserve(path.size() - start_index);
        resulting_path.push_back(
                    RRTNode(path[start_index].grippers(),
                            path[start_index].robotConfiguration(),
                            std::make_shared<RubberBand>(rubber_band)));
    }

    // Advance the grippers, simulating the rubber band until we reach the end of the path, or the band is overstretched
    bool band_is_overstretched = rubber_band.isOverstretched();
    bool band_got_stuck = false;
    size_t path_idx = start_index + 1;
    const bool rubber_band_verbose = false && visualization_enabled_globally_;
    while (!band_is_overstretched && !band_got_stuck && path_idx < path.size())
    {
        // Forward simulate the band
        stopwatch(RESET);
        const auto& ending_grippers_poses = path[path_idx].grippers();
        rubber_band.forwardPropagate(
                    ToGripperPositions(ending_grippers_poses),
                    rubber_band_verbose);
        const double forward_propogation_time = stopwatch(READ);
        total_band_forward_propogation_time_ += forward_propogation_time;

        // Store the band in the results
        resulting_path.push_back(RRTNode(
                                     path[path_idx].grippers(),
                                     path[path_idx].robotConfiguration(),
                                     std::make_shared<RubberBand>(rubber_band)));

        // Record if the band is overstretched
        band_is_overstretched = rubber_band.isOverstretched();
        band_got_stuck = !bandEndpointsMatchGripperPositions(rubber_band, path[path_idx].grippers());

        ++path_idx;
    }

    // If we the band is not overstretched, and the band did not get stuck,
    // then we reached the end of the path succesfully
    const bool success = !band_is_overstretched && !band_got_stuck;

    const double everything_included_forward_propogation_time = function_wide_stopwatch(READ);
    total_everything_included_forward_propogation_time_ += everything_included_forward_propogation_time;
    return std::make_pair(success, resulting_path);
}
