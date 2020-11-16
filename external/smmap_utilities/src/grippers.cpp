#include "smmap_utilities/grippers.h"
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/arc_exceptions.hpp>

namespace smmap
{
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////               Kinematics Related functionality                     ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    std::vector<std::string> ExtractGripperNames(
            const std::vector<GripperData>& grippers_data)
    {
        std::vector<std::string> names(grippers_data.size());

        for (size_t gripper_ind = 0; gripper_ind < grippers_data.size(); gripper_ind++)
        {
            names[gripper_ind] = grippers_data[gripper_ind].name_;
        }

        return names;
    }

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
            const Eigen::MatrixXd& object_initial_node_distance)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        for (long ind: gripper_indices)
        {
            min_dist = std::min(min_dist, object_initial_node_distance(ind, node_index));
        }

        return min_dist;
    }


    // Return both min distance and grasped node index
    std::pair<double, long> GetMinimumDistanceIndexToGripper(
            const std::vector<long>& gripper_indices,
            const long node_index,
            const Eigen::MatrixXd& object_initial_node_distance)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        long min_ind = gripper_indices.front();
        for (long ind: gripper_indices)
        {
            if (min_dist > object_initial_node_distance(ind, node_index))
            {
                min_dist = object_initial_node_distance(ind, node_index);
                min_ind = ind;
            }
        }
        return {min_dist, min_ind};
    }

    ////////////////////////////////////////////////////////////////////////////
    // Basic Math
    ////////////////////////////////////////////////////////////////////////////

    AllGrippersPoseDeltaTrajectory AddGripperPoseDeltas(
            AllGrippersPoseDeltaTrajectory lhs, const Eigen::VectorXd& rhs)
    {
        const size_t num_timesteps = lhs.size();
        assert(num_timesteps > 0);
        const size_t num_grippers = lhs[0].size();
        assert(num_grippers > 0);
        assert(num_timesteps * num_grippers * 6 == (size_t)rhs.rows());

        for (size_t time_ind = 0; time_ind < num_timesteps; time_ind++)
        {
            for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++)
            {
                lhs[time_ind][gripper_ind] += rhs.segment<6>((ssize_t)(time_ind * num_grippers * 6 + gripper_ind * 6));
            }
        }

        return lhs;
    }

    AllGrippersSinglePoseDelta RobotMotionToGripperMotion(
            const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& robot_motion)
    {
        const auto gripper_motion = jacobian * robot_motion;
        return EigenHelpers::EigenVectorXToVectorEigenVector<double, 6>(gripper_motion);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Distances
    ////////////////////////////////////////////////////////////////////////////

    // TODO: resolve duplicate code here and in the RRT codebase (and eigen_helpers.hpp codebase)
    double Distance(const AllGrippersSinglePose& c1,
                    const AllGrippersSinglePose& c2,
                    const bool include_rotation,
                    const double lambda)
    {
        assert(c1.size() == c2.size());
        double dist = 0.0;
        for (size_t idx = 0; idx < c1.size(); ++idx)
        {
            dist += (c2[idx].translation() - c1[idx].translation()).norm();
            if (include_rotation)
            {
                dist += lambda * EigenHelpers::Distance(c1[idx].rotation(), c2[idx].rotation());
            }
        }
        return dist;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Dot products
    ////////////////////////////////////////////////////////////////////////////

    double GripperVelocityDotProduct(
            const kinematics::Vector6d& vel1,
            const kinematics::Vector6d& vel2)
    {
        #pragma message "SE(3) velocity weight hard coded here"
        static constexpr double GRIPPER_VELOCITY_ROTATION_WEIGHT = 1.0/20.0;

        kinematics::Vector6d weight = kinematics::Vector6d::Ones();
        weight(3) = GRIPPER_VELOCITY_ROTATION_WEIGHT;
        weight(4) = GRIPPER_VELOCITY_ROTATION_WEIGHT;
        weight(5) = GRIPPER_VELOCITY_ROTATION_WEIGHT;
        weight.array() = weight.array().square();

        return EigenHelpers::WeightedDotProduct(vel1, vel2, weight);
    }

    double MultipleGrippersVelocityDotProduct(
            const AllGrippersSinglePoseDelta& vel1,
            const AllGrippersSinglePoseDelta& vel2)
    {
        assert(vel1.size() == vel2.size());

        double dot_product = 0;
        for (size_t vel_ind = 0; vel_ind < vel1.size(); vel_ind++)
        {
            dot_product += GripperVelocityDotProduct(vel1[vel_ind], vel2[vel_ind]);
        }

        return dot_product;
    }

    double MultipleGrippersVelocityTrajectoryDotProduct(
            const AllGrippersPoseDeltaTrajectory& traj1,
            const AllGrippersPoseDeltaTrajectory& traj2)
    {
        assert(traj1.size() == traj2.size());

        double dot_product = 0;
        for (size_t time_ind = 0; time_ind < traj1.size(); time_ind++)
        {
            dot_product += MultipleGrippersVelocityDotProduct(traj1[time_ind], traj2[time_ind]);
        }

        return dot_product;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Conversion functions
    ////////////////////////////////////////////////////////////////////////////

    PairGripperPositions ToGripperPositions(
            const EigenHelpers::VectorIsometry3d& poses)
    {
        assert(poses.size() == 2);
        return {poses[0].translation(), poses[1].translation()};
    }

    PairGripperPositions ToGripperPositions(
            const PairGripperPoses &poses)
    {
        return {poses.first.translation(), poses.second.translation()};
    }

    PairGripperPoses ToGripperPosePair(
            const AllGrippersSinglePose& poses_vector)
    {
        assert(poses_vector.size() == 2);
        return {poses_vector[0], poses_vector[1]};
    }

    AllGrippersSinglePose ToGripperPoseVector(
            const PairGripperPoses& poses_pair)
    {
        return {poses_pair.first, poses_pair.second};
    }

    // TODO:
    // # warning "Re-evaluate now this gripper pose delta math is done - remember Ruikun's example where the gripper went the wrong way"
    AllGrippersSinglePoseDelta CalculateGrippersPoseDelta(
            const AllGrippersSinglePose& prev,
            const AllGrippersSinglePose& next)
    {
        const size_t num_grippers = prev.size();
        assert(num_grippers > 0);
        assert(num_grippers == next.size());
        AllGrippersSinglePoseDelta grippers_pose_delta(num_grippers);
        for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++)
        {
            grippers_pose_delta[gripper_ind] =
                    kinematics::calculateError(
                        prev[gripper_ind],
                        next[gripper_ind]);
        }
        return grippers_pose_delta;
    }

    AllGrippersPoseDeltaTrajectory CalculateGrippersPoseDeltas(
            const AllGrippersPoseTrajectory& grippers_trajectory)
    {
        assert(grippers_trajectory.size() > 1);
        const size_t num_grippers = grippers_trajectory[0].size();

        AllGrippersPoseDeltaTrajectory grippers_pose_delta_traj(
                    grippers_trajectory.size() - 1,
                    AllGrippersSinglePoseDelta(num_grippers));

        for (size_t time_ind = 0; time_ind < grippers_pose_delta_traj.size(); time_ind++)
        {
            grippers_pose_delta_traj[time_ind] =
                    CalculateGrippersPoseDelta(
                        grippers_trajectory[time_ind],
                        grippers_trajectory[time_ind + 1]);
        }

        return grippers_pose_delta_traj;
    }

    AllGrippersPoseTrajectory CalculateGrippersTrajectory(
            const AllGrippersSinglePose& grippers_initial_pose,
            const AllGrippersPoseDeltaTrajectory& grippers_pose_deltas)
    {
        const size_t num_grippers = grippers_initial_pose.size();
        const size_t num_timesteps = grippers_pose_deltas.size();

        AllGrippersPoseTrajectory grippers_pose_trajectory(
                    num_timesteps + 1,
                    AllGrippersSinglePose(num_grippers));

        grippers_pose_trajectory[0] = grippers_initial_pose;

        for (size_t time_ind = 0; time_ind < num_timesteps; time_ind++)
        {
            for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind ++)
            {
                grippers_pose_trajectory[time_ind+1][gripper_ind] =
                        grippers_pose_trajectory[time_ind][gripper_ind] *
                        kinematics::expTwistIsometry3d(grippers_pose_deltas[time_ind][gripper_ind], 1);
            }
        }

        return grippers_pose_trajectory;
    }

    AllGrippersPoseTrajectory CalculateGrippersTrajectory(
            const AllGrippersSinglePose &grippers_initial_pose,
            const Eigen::VectorXd &grippers_pose_deltas)
    {
        const size_t num_grippers = grippers_initial_pose.size();
        const size_t num_timesteps = (size_t)grippers_pose_deltas.size() / (num_grippers * 6);
        assert((size_t)grippers_pose_deltas.size() % num_timesteps == 0);

        AllGrippersPoseTrajectory grippers_pose_trajectory(
                    num_timesteps + 1,
                    AllGrippersSinglePose(num_grippers));

        grippers_pose_trajectory[0] = grippers_initial_pose;

        for (size_t time_ind = 0; time_ind < num_timesteps; time_ind++)
        {
            for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind ++)
            {
                const kinematics::Vector6d gripper_delta =
                        grippers_pose_deltas.segment<6>(
                            (ssize_t)(time_ind * num_grippers * 6 + gripper_ind * 6));

                grippers_pose_trajectory[time_ind+1][gripper_ind] =
                        grippers_pose_trajectory[time_ind][gripper_ind] *
                        kinematics::expTwistIsometry3d(gripper_delta, 1);
            }
        }

        return grippers_pose_trajectory;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Norms induced by said dot products
    ////////////////////////////////////////////////////////////////////////////

    double GripperVelocity6dSquaredNorm(const kinematics::Vector6d& gripper_velocity)
    {
        return GripperVelocityDotProduct(gripper_velocity, gripper_velocity);
    }

    double GripperVelocity6dNorm(const kinematics::Vector6d& gripper_velocity)
    {
        return std::sqrt(GripperVelocity6dSquaredNorm(gripper_velocity));
    }

    double MultipleGrippersVelocity6dSquaredNorm(const AllGrippersSinglePoseDelta& grippers_velocity)
    {
        double squared_norm = 0;
        for (size_t gripper_ind = 0; gripper_ind < grippers_velocity.size(); gripper_ind++)
        {
            squared_norm += GripperVelocity6dSquaredNorm(grippers_velocity[gripper_ind]);
        }
        return squared_norm;
    }

    double MultipleGrippersVelocity6dSquaredNorm(const Eigen::VectorXd& grippers_velocity)
    {
        assert(grippers_velocity.size() % 6 == 0);

        double squared_norm = 0;
        for (long gripper_ind = 0; gripper_ind < grippers_velocity.size(); gripper_ind += 6)
        {
            squared_norm += GripperVelocity6dSquaredNorm(grippers_velocity.segment<6>(gripper_ind));
        }
        return squared_norm;
    }

    double MultipleGrippersVelocity6dNorm(const AllGrippersSinglePoseDelta& grippers_velocity)
    {
        return std::sqrt(MultipleGrippersVelocity6dSquaredNorm(grippers_velocity));
    }

    double MultipleGrippersVelocity6dNorm(const Eigen::VectorXd& grippers_velocity)
    {
        return std::sqrt(MultipleGrippersVelocity6dSquaredNorm(grippers_velocity));
    }

    double MultipleGrippersVelocityTrajectory6dSquaredNorm(const AllGrippersPoseDeltaTrajectory& grippers_trajectory)
    {
        double squared_norm = 0;
        for (size_t time_ind = 0; time_ind < grippers_trajectory.size(); time_ind++)
        {
            squared_norm += MultipleGrippersVelocity6dSquaredNorm(grippers_trajectory[time_ind]);
        }

        return squared_norm;
    }

    double MultipleGrippersVelocityTrajectory6dNorm(const AllGrippersPoseDeltaTrajectory& grippers_trajectory)
    {
        return std::sqrt(MultipleGrippersVelocityTrajectory6dSquaredNorm(grippers_trajectory));
    }

    Eigen::VectorXd ClampGripperPoseDeltas(Eigen::VectorXd velocities, const double max_pose_delta)
    {
        assert(velocities.size() % 6 == 0);
        for (ssize_t vel_ind = 0; vel_ind < velocities.size(); vel_ind += 6)
        {
            const double velocity_norm = GripperVelocity6dNorm(velocities.segment<6>(vel_ind));
            if (velocity_norm > max_pose_delta)
            {
                velocities.segment<6>(vel_ind) *= max_pose_delta / velocity_norm;
            }
        }
        return velocities;
    }

    AllGrippersSinglePoseDelta ClampGripperPoseDeltas(AllGrippersSinglePoseDelta pose_deltas, const double max_delta)
    {
        for (size_t gripper_ind = 0; gripper_ind < pose_deltas.size(); gripper_ind++)
        {
            pose_deltas[gripper_ind] = ClampGripperPoseDeltas(pose_deltas[gripper_ind], max_delta);
        }
        return pose_deltas;
    }

    AllGrippersPoseDeltaTrajectory ClampGripperPoseDeltas(AllGrippersPoseDeltaTrajectory pose_deltas, const double max_delta)
    {
        for (size_t time_ind = 0; time_ind < pose_deltas.size(); time_ind++)
        {
            pose_deltas[time_ind] = ClampGripperPoseDeltas(pose_deltas[time_ind], max_delta);
        }
        return pose_deltas;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                Collision related functionality                     ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    CollisionData::CollisionData(const Eigen::Vector3d& nearest_point_to_obstacle,
                  const Eigen::Vector3d& obstacle_surface_normal,
                  const double distance_to_obstacle)
        : nearest_point_to_obstacle_(nearest_point_to_obstacle)
        , obstacle_surface_normal_(obstacle_surface_normal)
        , distance_to_obstacle_(distance_to_obstacle)
    {}

    CollisionData::CollisionData(Eigen::Vector3d&& nearest_point_to_obstacle,
                  Eigen::Vector3d&& obstacle_surface_normal,
                  const double distance_to_obstacle)
        : nearest_point_to_obstacle_(nearest_point_to_obstacle)
        , obstacle_surface_normal_(obstacle_surface_normal)
        , distance_to_obstacle_(distance_to_obstacle)
    {}


    uint64_t CollisionData::SerializedSize()
    {
        return 2 * arc_utilities::SerializedSizeEigen(Eigen::Vector3d()) + sizeof(distance_to_obstacle_);
    }

    uint64_t CollisionData::serialize(std::vector<uint8_t>& buffer) const
    {
        const size_t starting_bytes = buffer.size();
        arc_utilities::SerializeEigen(nearest_point_to_obstacle_, buffer);
        arc_utilities::SerializeEigen(obstacle_surface_normal_, buffer);
        arc_utilities::SerializeFixedSizePOD(distance_to_obstacle_, buffer);
        const size_t ending_bytes = buffer.size();
        return ending_bytes - starting_bytes;
    }

    std::pair<CollisionData, uint64_t> CollisionData::Deserialize(
            const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        // Make sure there is enough data left
        assert(current + SerializedSize() <= buffer.size());

        // Do the deserialization itself
        uint64_t bytes_read = 0;
        const auto nearest_deserialized = arc_utilities::DeserializeEigen<Eigen::Vector3d>(buffer, current + bytes_read);
        bytes_read += nearest_deserialized.second;
        const auto normal_deserialized = arc_utilities::DeserializeEigen<Eigen::Vector3d>(buffer, current + bytes_read);
        bytes_read += normal_deserialized.second;
        const auto distance_deserailzied = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current + bytes_read);
        bytes_read += distance_deserailzied.second;

        // Error checking
        assert(bytes_read == SerializedSize());

        // Build and return the result
        return std::make_pair(CollisionData(nearest_deserialized.first,
                                            normal_deserialized.first,
                                            distance_deserailzied.first),
                              bytes_read);
    }

    bool CollisionData::operator==(const CollisionData& other) const
    {
        if (nearest_point_to_obstacle_.cwiseNotEqual(nearest_point_to_obstacle_).any())
        {
            return false;
        }

        if (obstacle_surface_normal_.cwiseNotEqual(other.obstacle_surface_normal_).any())
        {
            return false;
        }

        if (distance_to_obstacle_ != other.distance_to_obstacle_)
        {
            return false;
        }

        return true;
    }

    std::ostream& operator<<(std::ostream& os, const CollisionData& data)
    {
        os << "Dist: " << data.distance_to_obstacle_ << " Nearest: " << data.nearest_point_to_obstacle_.transpose() << " SNormal: " << data.obstacle_surface_normal_.transpose();
        return os;
    }

    ////////////////////////////////////////////////////////////////////////////

    GripperCollisionChecker::GripperCollisionChecker(ros::NodeHandle& nh)
        : world_frame_name_(smmap::GetWorldFrameName())
    {
        collision_checker_client_ =
            nh.serviceClient<deformable_manipulation_msgs::GetGripperCollisionReport>(
                    smmap::GetGripperCollisionCheckTopic(nh));
        collision_checker_client_.waitForExistence();
    }

    std::vector<CollisionData> GripperCollisionChecker::gripperCollisionCheck(
            const AllGrippersSinglePose& gripper_poses)
    {
        deformable_manipulation_msgs::GetGripperCollisionReport collision_report_ros;
        collision_report_ros.request.header.frame_id = world_frame_name_;
        collision_report_ros.request.pose =
                EigenHelpersConversions::VectorIsometry3dToVectorGeometryPose(gripper_poses);

        if (!collision_checker_client_.call(collision_report_ros))
        {
            ROS_FATAL_NAMED("gripper_collision_check", "Unable to retrieve gripper collision report.");
        }

        // TODO: Can we find a reasonable way to use CHECK_FRAME_NAME from ros_comms_helpers.hpp?
        if (collision_report_ros.response.header.frame_id != world_frame_name_)
        {
            ROS_FATAL_STREAM_NAMED("robot_interface", "gripperCollisionCheck() response data in incorrect frame. Expecting '"
                                   << world_frame_name_ << "', got '" << collision_report_ros.response.header.frame_id << "'.");
            throw_arc_exception(std::invalid_argument, "Invalid frame name");
        }

        std::vector<CollisionData> collision_report_eigen;
        collision_report_eigen.reserve(gripper_poses.size());

        for (size_t gripper_ind = 0; gripper_ind < gripper_poses.size(); gripper_ind++)
        {
            collision_report_eigen.push_back(
                        CollisionData(
                            EigenHelpersConversions::GeometryPointToEigenVector3d(
                                collision_report_ros.response.gripper_nearest_point_to_obstacle[gripper_ind]),
                            EigenHelpersConversions::GeometryVector3ToEigenVector3d(
                                collision_report_ros.response.obstacle_surface_normal[gripper_ind]),
                            collision_report_ros.response.gripper_distance_to_obstacle[gripper_ind]));
        }

        return collision_report_eigen;
    }

    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief ComputeGripperMotionToPointMotionJacobian
     * @param point_on_gripper
     * @param gripper_pose
     * @return Jacobian between gripper motion defined in the body frame of the gripper, and the world coordinates of the point in question
     *         world_frame_p_dot = J * body_frame_twist
     */
    Eigen::Matrix<double, 3, 6> ComputeGripperMotionToPointMotionJacobian(
            const Eigen::Vector3d& point_on_gripper,
            const Eigen::Isometry3d& gripper_pose)
    {

        Eigen::Matrix<double, 3, 6> J_collision;
        const Eigen::Matrix3d gripper_rot = gripper_pose.rotation();

        // Translation - if I move the gripper along its x/y/z-axis, what happens to the given point?
        J_collision.block<3, 3>(0, 0) = gripper_rot;

        const Eigen::Vector3d gripper_to_point_in_collision =
                point_on_gripper - gripper_pose.translation();

        // If I rotate the gripper about its x/y/z-axis, what happens to the point in question?
        J_collision.block<3, 1>(0, 3) = gripper_rot.block<3, 1>(0, 0).cross(gripper_to_point_in_collision);
        J_collision.block<3, 1>(0, 4) = gripper_rot.block<3, 1>(0, 1).cross(gripper_to_point_in_collision);
        J_collision.block<3, 1>(0, 5) = gripper_rot.block<3, 1>(0, 2).cross(gripper_to_point_in_collision);

        return J_collision;
    }

    CollisionAvoidanceResult ComputeGripperObjectAvoidance(
            const CollisionData& collision_data,
            const Eigen::Isometry3d& gripper_pose,
            double max_step_size)
    {
        CollisionAvoidanceResult collision_avoidance_result;

        collision_avoidance_result.distance = collision_data.distance_to_obstacle_;

        // If we have a collision to avoid, then find the vector
        if (!std::isinf(collision_data.distance_to_obstacle_))
        {
            // Create the collision Jacobian
            const Eigen::Matrix<double, 3, 6> J_collision =
                    ComputeGripperMotionToPointMotionJacobian(
                        collision_data.nearest_point_to_obstacle_, gripper_pose);
            const Eigen::Matrix<double, 6, 3> J_collision_inv =
                    EigenHelpers::Pinv(J_collision, EigenHelpers::SuggestedRcond());

            // Create the collision avoidance vector to follow
            const Eigen::Vector3d& avoid_collision_delta = collision_data.obstacle_surface_normal_;

            collision_avoidance_result.velocity =  J_collision_inv * avoid_collision_delta;
            collision_avoidance_result.velocity *=
                    max_step_size / GripperVelocity6dNorm(collision_avoidance_result.velocity);

            collision_avoidance_result.nullspace_projector =
                    Eigen::Matrix<double, 6, 6>::Identity() - J_collision_inv * J_collision;
        }
        // Otherwise, leave the collision result as the default "no collision" state
        else {}

        return collision_avoidance_result;
    }

    std::vector<CollisionAvoidanceResult> ComputeGripperObjectAvoidance(
            const std::vector<CollisionData>& collision_data,
            const EigenHelpers::VectorIsometry3d& gripper_pose,
            double max_step_size)
    {
        std::vector<CollisionAvoidanceResult> collision_avoidance_results;
        collision_avoidance_results.reserve(collision_data.size());

        for (size_t gripper_ind = 0; gripper_ind < gripper_pose.size(); gripper_ind++)
        {
            collision_avoidance_results.push_back(
                        ComputeGripperObjectAvoidance(
                                    collision_data[gripper_ind],
                                    gripper_pose[gripper_ind],
                                    max_step_size));
        }

        return collision_avoidance_results;
    }

    kinematics::Vector6d CombineDesiredAndObjectAvoidance(
            const kinematics::Vector6d& desired_motion,
            const CollisionAvoidanceResult& collision_result,
            const double obstacle_avoidance_scale)
    {
        if (!std::isinf(collision_result.distance))
        {
             const double collision_severity = std::min(1.0, std::exp(-obstacle_avoidance_scale * (collision_result.distance - smmap::GetRobotMinGripperDistanceToObstacles())));
             return collision_severity * (collision_result.velocity + collision_result.nullspace_projector * desired_motion) + (1.0 - collision_severity) * desired_motion;
        }
        // Otherwise use our desired velocity directly
        else
        {
             return desired_motion;
        }
    }

    std::vector<kinematics::Vector6d> CombineDesiredAndObjectAvoidance(
            const std::vector<kinematics::Vector6d>& desired_motion,
            const std::vector<CollisionAvoidanceResult>& collision_result,
            const double obstacle_avoidance_scale)
    {
        assert(desired_motion.size() == collision_result.size());
        std::vector<kinematics::Vector6d> result(desired_motion.size());
        for (size_t gripper_ind = 0; gripper_ind < desired_motion.size(); gripper_ind++)
        {
            result[gripper_ind] = CombineDesiredAndObjectAvoidance(
                        desired_motion[gripper_ind],
                        collision_result[gripper_ind],
                        obstacle_avoidance_scale);
        }
        return result;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                Serializing and Deserializing                       ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    uint64_t DeserializePair3dPositions(
            const Pair3dPositions& positions,
            std::vector<uint8_t>& buffer)
    {
        // TODO: determine why I need this lambda in order to avoid a linking error
        const auto serializer = [] (const Pair3dPositions::first_type& position, std::vector<uint8_t>& buf)
        {
            return arc_utilities::SerializeEigen(position, buf);
        };
        return arc_utilities::SerializePair<Eigen::Vector3d, Eigen::Vector3d>(
                    positions,
                    buffer,
                    serializer,
                    serializer);
    }

    std::pair<PairGripperPositions, uint64_t> DeserializePair3dPositions(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        return arc_utilities::DeserializePair<Pair3dPositions::first_type, Pair3dPositions::second_type>(
                    buffer,
                    current,
                    arc_utilities::DeserializeEigen<Pair3dPositions::first_type>,
                    arc_utilities::DeserializeEigen<Pair3dPositions::second_type>);
    }

    uint64_t SerializePair3dPoses(
            const Pair3dPoses& poses,
            std::vector<uint8_t>& buffer)
    {
        return arc_utilities::SerializePair<Pair3dPoses::first_type, Pair3dPoses::second_type>(
                    poses,
                    buffer,
                    arc_utilities::SerializeEigen<Pair3dPoses::first_type>,
                    arc_utilities::SerializeEigen<Pair3dPoses::second_type>);
    }

    std::pair<Pair3dPoses, uint64_t> DeserializePair3dPoses(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        return arc_utilities::DeserializePair<Pair3dPoses::first_type, Pair3dPoses::second_type>(
                    buffer,
                    current,
                    arc_utilities::DeserializeEigen<Pair3dPoses::first_type>,
                    arc_utilities::DeserializeEigen<Pair3dPoses::second_type>);
    }

    uint64_t SerializeAllGrippersSinglePose(
            const AllGrippersSinglePose& poses,
            std::vector<uint8_t>& buffer)
    {
        return arc_utilities::SerializeVector(poses, buffer, arc_utilities::SerializeEigen<Eigen::Isometry3d>);
    }

    std::pair<AllGrippersSinglePose, uint64_t> DeserializeAllGrippersSinglePose(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        return arc_utilities::DeserializeVector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>(
                    buffer,
                    current,
                    arc_utilities::DeserializeEigen<Eigen::Isometry3d>);
    }

    uint64_t SerializeAllGrippersPoseTrajectory(
            const AllGrippersPoseTrajectory& traj,
            std::vector<uint8_t>& buffer)
    {
        const std::function<uint64_t(const AllGrippersSinglePose&, std::vector<uint8_t>&)> item_serializer = []
                (const AllGrippersSinglePose& grippers_pose, std::vector<uint8_t>& buffer)
        {
            return SerializeAllGrippersSinglePose(grippers_pose, buffer);
        };
        return arc_utilities::SerializeVector(traj, buffer, item_serializer);
    }

    std::pair<AllGrippersPoseTrajectory, uint64_t> DeserializeAllGrippersPoseTrajectory(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        const std::function<std::pair<AllGrippersSinglePose, uint64_t>(const std::vector<uint8_t>&, const uint64_t)> item_deserializer = []
                (const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return DeserializeAllGrippersSinglePose(buffer, current);
        };
        return arc_utilities::DeserializeVector(buffer, current, item_deserializer);
    }

    uint64_t SerializeCollisionDataVector(
            const std::vector<CollisionData>& data,
            std::vector<uint8_t>& buffer)
    {
        const std::function<uint64_t(const CollisionData&, std::vector<uint8_t>&)> item_serializer = []
                (const CollisionData& data, std::vector<uint8_t>& buffer)
        {
            return data.serialize(buffer);
        };
        return arc_utilities::SerializeVector(data, buffer, item_serializer);
    }

    std::pair<std::vector<CollisionData>, uint64_t> DeserializeCollisionDataVector(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        const std::function<std::pair<CollisionData, uint64_t>(const std::vector<uint8_t>&, const uint64_t)> item_deserializer = []
                (const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return CollisionData::Deserialize(buffer, current);
        };
        return arc_utilities::DeserializeVector(buffer, current, item_deserializer);
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                Printing to screen                                  ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    std::string print(const AllGrippersSinglePoseDelta& delta)
    {
        assert(delta.size() == 2);

        std::ostringstream strm;
        strm << "0th: " << delta[0].transpose() << "     1st: " << delta[1].transpose();

        return strm.str();
    }
}
