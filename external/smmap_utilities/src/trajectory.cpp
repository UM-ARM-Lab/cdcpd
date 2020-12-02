#include <smmap_utilities/trajectory.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/serialization_eigen.hpp>
#include <smmap_utilities/ros_communication_helpers.h>

namespace smmap
{
    uint64_t WorldState::serializeSelf(std::vector<uint8_t>& buffer) const
    {
        const size_t starting_bytes = buffer.size();
        arc_utilities::SerializeEigen(object_configuration_, buffer);
        arc_utilities::SerializeVector(rope_node_transforms_, buffer, arc_utilities::SerializeEigen<Eigen::Isometry3d>);
        SerializeAllGrippersSinglePose(all_grippers_single_pose_, buffer);
        arc_utilities::SerializeEigen(robot_configuration_, buffer);
        arc_utilities::SerializeFixedSizePOD(robot_configuration_valid_, buffer);
        SerializeCollisionDataVector(gripper_collision_data_, buffer);
        arc_utilities::SerializeFixedSizePOD(sim_time_, buffer);
        const size_t bytes_written = buffer.size() - starting_bytes;

        // Test the result
        {
            const auto deserialized = Deserialize(buffer, starting_bytes);
            assert(deserialized.second == bytes_written);
            assert(deserialized.first == *this);
        }
        return bytes_written;
    }

    uint64_t WorldState::Serialize(const WorldState& state, std::vector<uint8_t>& buffer)
    {
        return state.serializeSelf(buffer);
    }

    std::pair<WorldState, uint64_t> WorldState::Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        uint64_t bytes_read = 0;

        const auto deserialized_object = arc_utilities::DeserializeEigen<ObjectPointSet>(buffer, current + bytes_read);
        bytes_read += deserialized_object.second;

        const auto deserialized_rope_node_transforms = arc_utilities::DeserializeVector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>(buffer, current + bytes_read, arc_utilities::DeserializeEigen<Eigen::Isometry3d>);
        bytes_read += deserialized_rope_node_transforms.second;

        const auto deserialized_grippers = DeserializeAllGrippersSinglePose(buffer, current + bytes_read);
        bytes_read += deserialized_grippers.second;

        const auto deserialized_robot_config = arc_utilities::DeserializeEigen<Eigen::VectorXd>(buffer, current + bytes_read);
        bytes_read += deserialized_robot_config.second;

        const auto deserialized_robot_config_valid = arc_utilities::DeserializeFixedSizePOD<bool>(buffer, current + bytes_read);
        bytes_read += deserialized_robot_config_valid.second;

        const auto deserialized_collision_data = DeserializeCollisionDataVector(buffer, current + bytes_read);
        bytes_read += deserialized_collision_data.second;

        const auto deserialized_sim_time = arc_utilities::DeserializeFixedSizePOD<double>(buffer, current + bytes_read);
        bytes_read += deserialized_sim_time.second;

        const WorldState result =
        {
            deserialized_object.first,
            deserialized_rope_node_transforms.first,
            deserialized_grippers.first,
            deserialized_robot_config.first,
            deserialized_robot_config_valid.first,
            deserialized_collision_data.first,
            deserialized_sim_time.first
        };
        return {result, bytes_read};
    }

    bool WorldState::operator==(const WorldState& other) const
    {
        if (object_configuration_.cwiseNotEqual(other.object_configuration_).any())
        {
            return false;
        }

        if (rope_node_transforms_.size() != other.rope_node_transforms_.size())
        {
            return false;
        }
        for (size_t idx = 0; idx < rope_node_transforms_.size(); ++idx)
        {
            if (rope_node_transforms_[idx].matrix().cwiseNotEqual(other.rope_node_transforms_[idx].matrix()).any())
            {
                return false;
            }
        }

        if (all_grippers_single_pose_.size() != other.all_grippers_single_pose_.size())
        {
            return false;
        }

        if (all_grippers_single_pose_.size() != other.all_grippers_single_pose_.size())
        {
            return false;
        }
        for (size_t idx = 0; idx < all_grippers_single_pose_.size(); ++ idx)
        {
            if (all_grippers_single_pose_[idx].matrix().cwiseNotEqual(other.all_grippers_single_pose_[idx].matrix()).any())
            {
                return false;
            }
        }

        if (robot_configuration_.cwiseNotEqual(other.robot_configuration_).any())
        {
            return false;
        }

        if (robot_configuration_valid_ != other.robot_configuration_valid_)
        {
            return false;
        }

        if (gripper_collision_data_.size() != other.gripper_collision_data_.size())
        {
            return false;
        }

        if (gripper_collision_data_ != other.gripper_collision_data_)
        {
            return false;
        }

        if (sim_time_ != other.sim_time_)
        {
            return false;
        }

        return true;
    }

    /**
     * @brief ConvertToEigenFeedback
     * @param feedback_ros
     * @return
     */
    WorldState ConvertToEigenFeedback(
            const deformable_manipulation_msgs::WorldState& feedback_ros)
    {
        WorldState feedback_eigen;

        feedback_eigen.object_configuration_ =
                SensorPointCloud2ToEigenMatrix3Xd(feedback_ros.object_configuration);

        feedback_eigen.rope_node_transforms_ =
                EigenHelpersConversions::VectorGeometryPoseToVectorIsometry3d(feedback_ros.rope_node_transforms);

        feedback_eigen.all_grippers_single_pose_ =
                EigenHelpersConversions::VectorGeometryPoseToVectorIsometry3d(feedback_ros.gripper_poses);

        feedback_eigen.robot_configuration_ =
                EigenHelpers::StdVectorToEigenVectorX(feedback_ros.robot_configuration);
        feedback_eigen.robot_configuration_valid_ = feedback_ros.robot_configuration_valid;

        size_t num_grippers = feedback_ros.gripper_poses.size();
        feedback_eigen.gripper_collision_data_.reserve(num_grippers);
        for (size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++)
        {
            feedback_eigen.gripper_collision_data_.push_back(
                        CollisionData(
                            EigenHelpersConversions::GeometryPointToEigenVector3d(
                                feedback_ros.gripper_nearest_point_to_obstacle[gripper_ind]),
                            EigenHelpersConversions::GeometryVector3ToEigenVector3d(
                                feedback_ros.obstacle_surface_normal[gripper_ind]),
                            feedback_ros.gripper_distance_to_obstacle[gripper_ind]));
        }

        feedback_eigen.sim_time_ = feedback_ros.sim_time;

        return feedback_eigen;
    }

    std::vector<WorldState> ConvertToEigenFeedback(
            const std::vector<deformable_manipulation_msgs::WorldState>& feedback_ros)
    {
        std::vector<WorldState> feedback_eigen;
        feedback_eigen.reserve(feedback_ros.size());
        for (size_t idx = 0; idx < feedback_ros.size(); ++idx)
        {
            feedback_eigen.push_back(ConvertToEigenFeedback(feedback_ros[idx]));
        }
        return feedback_eigen;
    }

    /**
     * @brief getGripperTrajectories
     * @param feedback
     * @return
     */
    AllGrippersPoseTrajectory GetGripperTrajectories(
            const std::vector<WorldState>& feedback)
    {
        AllGrippersPoseTrajectory grippers_trajectories(feedback.size());

        for (size_t time_ind = 0; time_ind < feedback.size(); time_ind++)
        {
            grippers_trajectories[time_ind] =
                    feedback[time_ind].all_grippers_single_pose_;
        }

        return grippers_trajectories;
    }

    Eigen::VectorXd CalculateObjectDeltaAsVector(
            const ObjectPointSet& start,
            const ObjectPointSet& end)
    {
        Eigen::MatrixXd diff = end - start;
        diff.resize(diff.rows() * diff.cols(), 1);
        return diff;
    }

    ObjectPointSet AddObjectDelta(
            ObjectPointSet start,
            const Eigen::VectorXd& delta)
    {
        assert(delta.rows() == start.cols() * 3);

        for (ssize_t point_ind = 0; point_ind < start.cols(); point_ind++)
        {
            start.col(point_ind) = start.col(point_ind) + delta.segment<3>(point_ind * 3);
        }
        return start;
    }
}
