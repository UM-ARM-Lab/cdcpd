#include <smmap_utilities/ros_communication_helpers.h>

#include <mutex>

#include <ros/ros.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <deformable_manipulation_experiment_params/task_enums.h>
#include <deformable_manipulation_msgs/messages.h>

namespace smmap
{
    std::vector<GripperData> GetGrippersData(ros::NodeHandle& nh)
    {
        ROS_INFO_NAMED("ros_comms_helpers" , "Getting grippers data");
        std::vector<GripperData> grippers_data;

        // Service client to get the names of each gripper
        ros::ServiceClient gripper_names_client =
            nh.serviceClient<deformable_manipulation_msgs::GetGripperNames>(GetGripperNamesTopic(nh));

        gripper_names_client.waitForExistence();

        deformable_manipulation_msgs::GetGripperNames names_srv_data;
        if (!gripper_names_client.call(names_srv_data))
        {
            ROS_FATAL_NAMED("ros_comms_helpers", "Unabled to retrieve gripper names.");
        }
        std::vector<std::string> gripper_names = names_srv_data.response.names;

        // Service client to get the attached nodes and transform for each gripper
        ros::ServiceClient gripper_node_indices_client =
            nh.serviceClient<deformable_manipulation_msgs::GetGripperAttachedNodeIndices>(GetGripperAttachedNodeIndicesTopic(nh));
        gripper_node_indices_client.waitForExistence();

        grippers_data.reserve(gripper_names.size());
        for (size_t gripper_ind = 0; gripper_ind < gripper_names.size(); gripper_ind++)
        {
            deformable_manipulation_msgs::GetGripperAttachedNodeIndices node_indices_srv_data;
            node_indices_srv_data.request.name = gripper_names[gripper_ind];

            if (gripper_node_indices_client.call(node_indices_srv_data))
            {
                // All types of deforable objects get the basic gripper info
                grippers_data.push_back(GripperData(gripper_names[gripper_ind],
                                                    VectorAnytypeToVectorLong(node_indices_srv_data.response.indices)));

                // Cloth also gets Geo info used by the StretchingAvoidanceController
                if (GetDeformableType(nh) == DeformableType::CLOTH)
                {
                    ros::ServiceClient gripper_stretching_vector_info_client =
                            nh.serviceClient<deformable_manipulation_msgs::GetGripperStretchingVectorInfo>(
                                GetGripperStretchingVectorInfoTopic(nh));
                    gripper_stretching_vector_info_client.waitForExistence();

                    deformable_manipulation_msgs::GetGripperStretchingVectorInfo stretching_vector_info_srv_data;
                    stretching_vector_info_srv_data.request.name = gripper_names[gripper_ind];

                    if (gripper_stretching_vector_info_client.call(stretching_vector_info_srv_data))
                    {
                        grippers_data.back().stretching_vector_info_ = StretchingVectorInfo(
                                    stretching_vector_info_srv_data.response.to_gripper_name,
                                    VectorAnytypeToVectorLong(stretching_vector_info_srv_data.response.attatched_indices),
                                    VectorAnytypeToVectorLong(stretching_vector_info_srv_data.response.neighbor_indices),
                                    stretching_vector_info_srv_data.response.contributions);
                    }
                    else
                    {
                        ROS_FATAL_STREAM_NAMED("ros_comms_helpers",
                                               "Unable to retrieve stretching vector info for gripper: "
                                               << gripper_names[gripper_ind]);
                        assert(false);
                    }
                }
            }
            else
            {
                ROS_ERROR_STREAM_NAMED("ros_comms_helpers", "Unable to retrieve node indices for gripper: " << gripper_names[gripper_ind]);
                assert(false);
            }
        }

        return grippers_data;
    }

    // Warning!! This code assumes that the data is laid out in (x,y,z) format, and contains floats
    Eigen::Matrix3Xd SensorPointCloud2ToEigenMatrix3Xd(const sensor_msgs::PointCloud2& ros)
    {
        assert(ros.fields.size() == 3);
        assert(ros.fields[0].name == "x");
        assert(ros.fields[1].name == "y");
        assert(ros.fields[2].name == "z");
        assert(ros.fields[0].datatype == sensor_msgs::PointField::FLOAT32);
        assert(ros.fields[1].datatype == sensor_msgs::PointField::FLOAT32);
        assert(ros.fields[2].datatype == sensor_msgs::PointField::FLOAT32);

        const auto num_points = ros.width * ros.height;
        const Eigen::Map<const Eigen::Matrix3Xf> eigen((const float *)ros.data.data(), 3, num_points);
        return eigen.cast<double>();
    }

    ObjectPointSet GetObjectInitialConfiguration(ros::NodeHandle& nh)
    {
        ROS_INFO_NAMED("ros_comms_helpers" , "Getting object initial configuration");

        // Get the initial configuration of the object
        ros::ServiceClient client =
            nh.serviceClient<deformable_manipulation_msgs::GetPointSet>(GetObjectInitialConfigurationTopic(nh));
        client.waitForExistence();
        deformable_manipulation_msgs::GetPointSet srv_data;
        client.call(srv_data);

        ROS_INFO_NAMED("ros_comms_helpers" , "Number of points on object: %zu", srv_data.response.points.width * srv_data.response.points.height);
        CHECK_FRAME_NAME("ros_comms_helpers", GetWorldFrameName(), srv_data.response.points.header.frame_id);

        return SensorPointCloud2ToEigenMatrix3Xd(srv_data.response.points);
    }

    std::vector<geometry_msgs::Pose> GetRopeNodeTransforms(ros::NodeHandle& nh)
    {
        ros::ServiceClient client =
            nh.serviceClient<deformable_manipulation_msgs::GetPoseSet>(GetRopeCurrentNodeTransformsTopic(nh));
        client.waitForExistence();
        deformable_manipulation_msgs::GetPoseSet srv_data;
        client.call(srv_data);

        ROS_INFO_NAMED("ros_comms_helpers" , "Number of poses on object: %zu", srv_data.response.poses.size());
        CHECK_FRAME_NAME("ros_comms_helpers", GetWorldFrameName(), srv_data.response.header.frame_id);

        return srv_data.response.poses;
    }

    ObjectPointSet GetCoverPoints(ros::NodeHandle& nh)
    {
        ROS_INFO_NAMED("ros_comms_helpers" , "Getting cover points");

        // Get the initial configuration of the object
        ros::ServiceClient cover_points_client =
            nh.serviceClient<deformable_manipulation_msgs::GetPointSet>(GetCoverPointsTopic(nh));

        cover_points_client.waitForExistence();

        deformable_manipulation_msgs::GetPointSet srv_data;
        cover_points_client.call(srv_data);

        ROS_INFO_NAMED("ros_comms_helpers" , "Number of cover points: %zu", srv_data.response.points.width * srv_data.response.points.height);
        CHECK_FRAME_NAME("ros_comms_helpers", GetWorldFrameName(), srv_data.response.points.header.frame_id);

        return SensorPointCloud2ToEigenMatrix3Xd(srv_data.response.points);;
    }

    ObjectPointSet GetCoverPointNormals(ros::NodeHandle& nh)
    {
        ROS_INFO_NAMED("ros_comms_helpers" , "Getting cover point normal vectors");

        // Get the initial configuration of the object
        ros::ServiceClient cover_point_normal_vectors_client =
            nh.serviceClient<deformable_manipulation_msgs::GetVector3Set>(GetCoverPointNormalsTopic(nh));

        cover_point_normal_vectors_client.waitForExistence();

        deformable_manipulation_msgs::GetVector3Set srv_data;
        cover_point_normal_vectors_client.call(srv_data);
        ObjectPointSet cover_point_normals = EigenHelpersConversions::VectorGeometryVector3ToEigenMatrix3Xd(srv_data.response.vectors);
        for (ssize_t col_ind = 0; col_ind < cover_point_normals.cols(); ++col_ind)
        {
            cover_point_normals.col(col_ind).normalize();
        }

        ROS_INFO_NAMED("ros_comms_helpers" , "Number of cover point normals: %zu", srv_data.response.vectors.size());
        CHECK_FRAME_NAME("ros_comms_helpers", GetWorldFrameName(), srv_data.response.header.frame_id);

        return cover_point_normals;
    }

    // TODO: replace these out params with something else
    void GetFreeSpaceGraph(
            ros::NodeHandle& nh,
            arc_dijkstras::Graph<Eigen::Vector3d>& free_space_graph,
            std::vector<int64_t>& cover_ind_to_free_space_graph_ind)
    {
        ROS_INFO_NAMED("ros_comms_helpers", "Getting free space graph");

        // First we collect the data in serialzed form
        ros::ServiceClient free_space_graph_client =
            nh.serviceClient<deformable_manipulation_msgs::GetFreeSpaceGraph>(GetFreeSpaceGraphTopic(nh));

        free_space_graph_client.waitForExistence();

        deformable_manipulation_msgs::GetFreeSpaceGraph srv_data;
        free_space_graph_client.call(srv_data);
        CHECK_FRAME_NAME("ros_comms_helpers", GetWorldFrameName(), srv_data.response.header.frame_id);

        // Next we deserialize the graph itself
        {
            // Define the graph value deserialization function
            const auto value_deserializer_fn = [] (const std::vector<uint8_t>& buffer, const uint64_t current)
            {
                uint64_t current_position = current;

                // Deserialze 3 floats (btScalar type), converting into doubles afterwards
                std::pair<float, uint64_t> x = arc_utilities::DeserializeFixedSizePOD<float>(buffer, current_position);
                current_position += x.second;
                std::pair<float, uint64_t> y = arc_utilities::DeserializeFixedSizePOD<float>(buffer, current_position);
                current_position += y.second;
                std::pair<float, uint64_t> z = arc_utilities::DeserializeFixedSizePOD<float>(buffer, current_position);
                current_position += z.second;

                const Eigen::Vector3d deserialized(x.first, y.first, z.first);

                // Figure out how many bytes were read
                const uint64_t bytes_read = current_position - current;
                return std::make_pair(deserialized, bytes_read);
            };

            uint64_t current_position = 0;
            current_position += free_space_graph.deserializeSelf(srv_data.response.graph_data_buffer, current_position, value_deserializer_fn);
            assert(current_position == srv_data.response.graph_data_buffer.size());
            assert(free_space_graph.checkGraphLinkage());
            ROS_INFO_STREAM_NAMED("ros_comms_helpers", "Recieved " << free_space_graph.getNodes().size() << " graph nodes");
        }

        // Last we copy the map between cover point indices and graph indices
        cover_ind_to_free_space_graph_ind = srv_data.response.cover_point_ind_to_graph_ind;
    }

    sdf_tools::SignedDistanceField::ConstPtr GetEnvironmentSDF(ros::NodeHandle& nh)
    {
        static sdf_tools::SignedDistanceField::Ptr sdf(nullptr);
        static std::mutex mtx;

        std::lock_guard<std::mutex> lock(mtx);
        if (sdf == nullptr)
        {
            ROS_INFO_NAMED("ros_comms_helpers", "Getting environment sdf");

            // First we collect the data in serialzed form
            ros::ServiceClient sdf_client =
                nh.serviceClient<deformable_manipulation_msgs::GetSignedDistanceField>(GetSignedDistanceFieldTopic(nh));

            sdf_client.waitForExistence();

            deformable_manipulation_msgs::GetSignedDistanceField srv_data;
            sdf_client.call(srv_data);

            // Then parse the message and return the result
            const sdf_tools::SignedDistanceField sdf_raw =
                sdf_tools::SignedDistanceField::LoadFromMessageRepresentation(srv_data.response.sdf);
            sdf = std::make_shared<sdf_tools::SignedDistanceField>(sdf_raw);
            CHECK_FRAME_NAME("ros_comms_helpers", GetWorldFrameName(), sdf->GetFrame());
        }

        return sdf;
    }
}
