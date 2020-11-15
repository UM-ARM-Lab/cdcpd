#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <arc_utilities/eigen_typedefs.hpp>
#include <kinematics_toolbox/kinematics.h>
#include <deformable_manipulation_msgs/messages.h>
#include <smmap_utilities/grippers.h>

namespace smmap
{
    typedef std::vector<ObjectPointSet> ObjectTrajectory;
    typedef std::vector<ObjectTrajectory> VectorObjectTrajectory;

    struct ObjectDeltaAndWeight
    {
    public:
        ObjectDeltaAndWeight()
        {}

        ObjectDeltaAndWeight(ssize_t num_elems)
            : delta(Eigen::VectorXd::Zero(num_elems))
            , weight(Eigen::VectorXd::Zero(num_elems))
        {}

        Eigen::VectorXd delta;
        Eigen::VectorXd weight;
    };

    struct DesiredDirection
    {
    public:
        ObjectDeltaAndWeight error_correction_;
        ObjectDeltaAndWeight stretching_correction_;
        ObjectDeltaAndWeight combined_correction_;
    };

    /// World state structure for a single time step
    struct WorldState
    {
    public:
        ObjectPointSet object_configuration_;
        EigenHelpers::VectorIsometry3d rope_node_transforms_;
        AllGrippersSinglePose all_grippers_single_pose_;
        Eigen::VectorXd robot_configuration_;
        bool robot_configuration_valid_;
        std::vector<CollisionData> gripper_collision_data_;
        double sim_time_;

        uint64_t serializeSelf(std::vector<uint8_t>& buffer) const;

        static uint64_t Serialize(const WorldState& state, std::vector<uint8_t>& buffer);

        static std::pair<WorldState, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current);

        bool operator==(const WorldState& other) const;
    };

    /**
     * @brief ConvertToEigenFeedback
     * @param feedback_ros
     * @return
     */
    WorldState ConvertToEigenFeedback(
            const deformable_manipulation_msgs::WorldState& feedback_ros);

    std::vector<WorldState> ConvertToEigenFeedback(
            const std::vector<deformable_manipulation_msgs::WorldState>& feedback_ros);

    /**
     * @brief getGripperTrajectories
     * @param feedback
     * @return
     */
    AllGrippersPoseTrajectory GetGripperTrajectories(
            const std::vector<WorldState>& feedback);

    Eigen::VectorXd CalculateObjectDeltaAsVector(
            const ObjectPointSet& start,
            const ObjectPointSet& end);

    ObjectPointSet AddObjectDelta(
            ObjectPointSet start,
            const Eigen::VectorXd& delta);
}

#endif // TRAJECTORY_HPP
