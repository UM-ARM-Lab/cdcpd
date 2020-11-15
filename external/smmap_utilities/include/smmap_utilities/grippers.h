#ifndef GRIPPERS_H
#define GRIPPERS_H

#include <limits>
#include <memory>
#include <assert.h>
#include <ostream>

#include <arc_utilities/eigen_typedefs.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/serialization.hpp>
#include <arc_utilities/serialization_eigen.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <kinematics_toolbox/kinematics.h>
#include <deformable_manipulation_experiment_params/ros_params.hpp>
#include <deformable_manipulation_msgs/messages.h>

namespace smmap
{
    typedef Eigen::Matrix3Xd ObjectPointSet;

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////               Kinematics Related functionality                     ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    typedef std::pair<Eigen::Isometry3d, Eigen::Isometry3d> Pair3dPoses;
    typedef std::pair<Eigen::Isometry3d, Eigen::Isometry3d> PairGripperPoses;

    typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pair3dPositions;
    typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> PairGripperPositions;

    typedef EigenHelpers::VectorIsometry3d AllGrippersSinglePose;
    typedef std::vector<AllGrippersSinglePose> AllGrippersPoseTrajectory;

    // VectorVector6d is a 6xn vector of 6d vectors, n cols
    typedef kinematics::VectorVector6d AllGrippersSinglePoseDelta;
    typedef std::vector<AllGrippersSinglePoseDelta> AllGrippersPoseDeltaTrajectory;

    // Stretching tracking vector information
    struct StretchingVectorInfo
    {
        StretchingVectorInfo()
        {}

        StretchingVectorInfo(
                const std::string& to_gripper_name,
                const std::vector<long>& from_nodes,
                const std::vector<long>& to_nodes,
                const std::vector<double>& node_contribution)
            : to_gripper_name_(to_gripper_name)
            , from_nodes_(from_nodes)
            , to_nodes_(to_nodes)
            , node_contribution_(node_contribution)
        {}

        std::string to_gripper_name_;
        std::vector<long> from_nodes_;
        std::vector<long> to_nodes_;
        std::vector<double> node_contribution_;
    };

    struct GripperData
    {
        GripperData(const std::string& name, const std::vector<long>& node_indices)
            : name_(name)
            , node_indices_(node_indices)
        {}

        GripperData(const std::string& name,
                    const std::vector<long>& node_indices,
                    const std::string& to_gripper_name,
                    const std::vector<long>& from_nodes,
                    const std::vector<long>& to_nodes,
                    const std::vector<double>& node_contribution)
            : name_(name)
            , node_indices_(node_indices)
            , stretching_vector_info_(to_gripper_name,
                                      from_nodes,
                                      to_nodes,
                                      node_contribution)
        {}

        /// The name associated with this gripper
        std::string name_;

        /// Vector of the indices of the nodes that are grasped by the gripper
        std::vector<long> node_indices_;

        /// Stretching tracking vector information. Note that this data member is only valid for cloth experiments
        StretchingVectorInfo stretching_vector_info_;

        /**
         * @brief operator <<
         * @param out The stream to output the data too
         * @param data The gripper data to output
         * @return
         */
        friend std::ostream& operator<< (std::ostream& out, const GripperData& data)
        {
            out << data.name_ << " Node Indices: " << PrettyPrint::PrettyPrint(data.node_indices_);
            return out;
        }
    };

    template<typename T>
    inline std::vector<long> VectorAnytypeToVectorLong(
            const std::vector<T>& vector_anytype)
    {
        std::vector<long> vector_signed(vector_anytype.size());
        for (size_t ind = 0; ind < vector_anytype.size(); ind++)
        {
            vector_signed[ind] = (long)(vector_anytype[ind]);
        }
        return vector_signed;
    }

    std::vector<std::string> ExtractGripperNames(
            const std::vector<GripperData>& grippers_data);

    /**
     * @brief getMinimumDistanceToGripper
     * @param gripper_indices The indices of the nodes that the gripper is in contact with
     * @param node_index The index of the node that we want to get the distance to
     * @param object_initial_node_distance The matrix of distances between nodes
     * @return The the distance between given node, and the closest node grasped by the gripper
     */
    double GetMinimumDistanceToGripper(
            const std::vector<long>& gripper_indices,
            const long node_index,
            const Eigen::MatrixXd& object_initial_node_distance);

    // Return both min distance and grasped node index
    std::pair<double, long> GetMinimumDistanceIndexToGripper(
            const std::vector<long>& gripper_indices,
            const long node_index,
            const Eigen::MatrixXd& object_initial_node_distance);

    ////////////////////////////////////////////////////////////////////////////
    // Basic Math
    ////////////////////////////////////////////////////////////////////////////

    AllGrippersPoseDeltaTrajectory AddGripperPoseDeltas(
            AllGrippersPoseDeltaTrajectory lhs, const Eigen::VectorXd& rhs);

    AllGrippersSinglePoseDelta RobotMotionToGripperMotion(
            const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& robot_motion);

    ////////////////////////////////////////////////////////////////////////////
    // Distances
    ////////////////////////////////////////////////////////////////////////////

    double Distance(const AllGrippersSinglePose& c1,
                    const AllGrippersSinglePose& c2,
                    const bool include_rotation = false,
                    const double lambda = 1.0);

    ////////////////////////////////////////////////////////////////////////////
    // Dot products
    ////////////////////////////////////////////////////////////////////////////

    double GripperVelocityDotProduct(
            const kinematics::Vector6d& vel1,
            const kinematics::Vector6d& vel2);

    double MultipleGrippersVelocityDotProduct(
            const AllGrippersSinglePoseDelta& vel1,
            const AllGrippersSinglePoseDelta& vel2);

    double MultipleGrippersVelocityTrajectoryDotProduct(
            const AllGrippersPoseDeltaTrajectory& traj1,
            const AllGrippersPoseDeltaTrajectory& traj2);

    ////////////////////////////////////////////////////////////////////////////
    // Conversion functions
    ////////////////////////////////////////////////////////////////////////////

    PairGripperPositions ToGripperPositions(
            const EigenHelpers::VectorIsometry3d& poses);

    PairGripperPositions ToGripperPositions(
            const PairGripperPoses& poses);

    PairGripperPoses ToGripperPosePair(
            const AllGrippersSinglePose& poses_vector);

    AllGrippersSinglePose ToGripperPoseVector(
            const PairGripperPoses& poses_pair);

    // TODO:
    // # warning "Re-evaluate now this gripper pose delta math is done - remember Ruikun's example where the gripper went the wrong way"
    AllGrippersSinglePoseDelta CalculateGrippersPoseDelta(
            const AllGrippersSinglePose& prev,
            const AllGrippersSinglePose& next);

    AllGrippersPoseDeltaTrajectory CalculateGrippersPoseDeltas(
            const AllGrippersPoseTrajectory& grippers_trajectory);

    AllGrippersPoseTrajectory CalculateGrippersTrajectory(
            const AllGrippersSinglePose& grippers_initial_pose,
            const AllGrippersPoseDeltaTrajectory& grippers_pose_deltas);

    AllGrippersPoseTrajectory CalculateGrippersTrajectory(
            const AllGrippersSinglePose &grippers_initial_pose,
            const Eigen::VectorXd &grippers_pose_deltas);

    ////////////////////////////////////////////////////////////////////////////
    // Norms induced by said dot products
    ////////////////////////////////////////////////////////////////////////////

    double GripperVelocity6dSquaredNorm(const kinematics::Vector6d& gripper_velocity);

    double GripperVelocity6dNorm(const kinematics::Vector6d& gripper_velocity);

    double MultipleGrippersVelocity6dSquaredNorm(const AllGrippersSinglePoseDelta& grippers_velocity);

    double MultipleGrippersVelocity6dSquaredNorm(const Eigen::VectorXd& grippers_velocity);

    double MultipleGrippersVelocity6dNorm(const AllGrippersSinglePoseDelta& grippers_velocity);

    double MultipleGrippersVelocity6dNorm(const Eigen::VectorXd& grippers_velocity);

    double MultipleGrippersVelocityTrajectory6dSquaredNorm(const AllGrippersPoseDeltaTrajectory& grippers_trajectory);

    double MultipleGrippersVelocityTrajectory6dNorm(const AllGrippersPoseDeltaTrajectory& grippers_trajectory);

    Eigen::VectorXd ClampGripperPoseDeltas(Eigen::VectorXd velocities, const double max_pose_delta);

    AllGrippersSinglePoseDelta ClampGripperPoseDeltas(AllGrippersSinglePoseDelta pose_deltas, const double max_delta);

    AllGrippersPoseDeltaTrajectory ClampGripperPoseDeltas(AllGrippersPoseDeltaTrajectory pose_deltas, const double max_delta);

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                Collision related functionality                     ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    /// Stores the result of a collision check from bullet (or similar)
    struct CollisionData
    {
        public:
            CollisionData(const Eigen::Vector3d& nearest_point_to_obstacle,
                          const Eigen::Vector3d& obstacle_surface_normal,
                          const double distance_to_obstacle);

            CollisionData(Eigen::Vector3d&& nearest_point_to_obstacle,
                          Eigen::Vector3d&& obstacle_surface_normal,
                          const double distance_to_obstacle);

            Eigen::Vector3d nearest_point_to_obstacle_;
            Eigen::Vector3d obstacle_surface_normal_;
            double distance_to_obstacle_;

            static uint64_t SerializedSize();

            uint64_t serialize(std::vector<uint8_t>& buffer) const;

            static std::pair<CollisionData, uint64_t> Deserialize(
                    const std::vector<uint8_t>& buffer, const uint64_t current);

            bool operator==(const CollisionData& other) const;
    };

    std::ostream& operator<<(std::ostream& os, const CollisionData& data);

    /// Stores the result of a collision avoidance calculation for a single gripper
    struct CollisionAvoidanceResult
    {
        public:
            CollisionAvoidanceResult()
                : nullspace_projector(kinematics::Matrix6d::Identity())
                , velocity(kinematics::Vector6d::Zero())
                , distance(std::numeric_limits<double>::infinity())
            {}

            kinematics::Matrix6d nullspace_projector;
            kinematics::Vector6d velocity;
            double distance;
    };

    class GripperCollisionChecker
    {
        public:
            GripperCollisionChecker(ros::NodeHandle& nh);

            std::vector<CollisionData> gripperCollisionCheck(
                    const AllGrippersSinglePose& gripper_poses);

        private:
            const std::string world_frame_name_;
            ros::ServiceClient collision_checker_client_;
    };

    /**
     * @brief ComputeGripperMotionToPointMotionJacobian
     * @param point_on_gripper
     * @param gripper_pose
     * @return Jacobian between gripper motion defined in the body frame of the gripper, and the world coordinates of the point in question
     *         world_frame_p_dot = J * body_frame_twist
     */
    Eigen::Matrix<double, 3, 6> ComputeGripperMotionToPointMotionJacobian(
            const Eigen::Vector3d& point_on_gripper,
            const Eigen::Isometry3d& gripper_pose);

    /**
     * @brief ComputeGripperObjectAvoidance
     * @param collision_data
     * @param gripper_pose
     * @param max_step_size
     * @return
     */
    CollisionAvoidanceResult ComputeGripperObjectAvoidance(
            const CollisionData& collision_data,
            const Eigen::Isometry3d& gripper_pose,
            double max_step_size);

    std::vector<CollisionAvoidanceResult> ComputeGripperObjectAvoidance(
            const std::vector<CollisionData>& collision_data,
            const EigenHelpers::VectorIsometry3d& gripper_pose,
            double max_step_size);

    kinematics::Vector6d CombineDesiredAndObjectAvoidance(
            const kinematics::Vector6d& desired_motion,
            const CollisionAvoidanceResult& collision_result,
            const double obstacle_avoidance_scale);

    std::vector<kinematics::Vector6d> CombineDesiredAndObjectAvoidance(
            const std::vector<kinematics::Vector6d>& desired_motion,
            const std::vector<CollisionAvoidanceResult>& collision_result,
            const double obstacle_avoidance_scale);

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                Serializing and Deserializing                       ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    uint64_t DeserializePair3dPositions(
            const Pair3dPositions& positions,
            std::vector<uint8_t>& buffer);

    std::pair<PairGripperPositions, uint64_t> DeserializePair3dPositions(
            const std::vector<uint8_t>& buffer,
            const uint64_t current);

    uint64_t SerializePair3dPoses(
            const Pair3dPoses& poses,
            std::vector<uint8_t>& buffer);

    std::pair<Pair3dPoses, uint64_t> DeserializePair3dPoses(
            const std::vector<uint8_t>& buffer,
            const uint64_t current);

    uint64_t SerializeAllGrippersSinglePose(
            const AllGrippersSinglePose& poses,
            std::vector<uint8_t>& buffer);

    std::pair<AllGrippersSinglePose, uint64_t> DeserializeAllGrippersSinglePose(
            const std::vector<uint8_t>& buffer,
            const uint64_t current);

    uint64_t SerializeAllGrippersPoseTrajectory(
            const AllGrippersPoseTrajectory& traj,
            std::vector<uint8_t>& buffer);

    std::pair<AllGrippersPoseTrajectory, uint64_t> DeserializeAllGrippersPoseTrajectory(
            const std::vector<uint8_t>& buffer,
            const uint64_t current);

    uint64_t SerializeCollisionDataVector(
            const std::vector<CollisionData>& data,
            std::vector<uint8_t>& buffer);

    std::pair<std::vector<CollisionData>, uint64_t> DeserializeCollisionDataVector(
            const std::vector<uint8_t>& buffer,
            const uint64_t current);

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                Printing to screen                                  ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    std::string print(const AllGrippersSinglePoseDelta& delta);
}

#endif // GRIPPERS_H
