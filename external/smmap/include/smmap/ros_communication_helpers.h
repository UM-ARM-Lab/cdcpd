#ifndef ROS_COMMUNICATION_HELPERS_H
#define ROS_COMMUNICATION_HELPERS_H

#include <ros/ros.h>
#include <arc_utilities/arc_exceptions.hpp>
#include <arc_utilities/dijkstras.hpp>
#include <sdf_tools/sdf.hpp>
#include <smmap_utilities/grippers.h>

#define CHECK_FRAME_NAME(logger, expected, given)                                                       \
    if ((given) != (expected))                                                                          \
    {                                                                                                   \
        ROS_FATAL_STREAM_NAMED((logger), __func__ << " response data in incorrect frame. Expecting '"   \
                               << (expected) << "', got '" << (given) << "'.");                         \
        throw_arc_exception(std::invalid_argument, "Invalid frame name");                               \
    }

namespace smmap
{
    std::vector<GripperData> GetGrippersData(ros::NodeHandle& nh);

    // Warning!! This code assumes that the data is laid out in (x,y,z) format, and contains floats
    Eigen::Matrix3Xd SensorPointCloud2ToEigenMatrix3Xd(const sensor_msgs::PointCloud2& ros);

    ObjectPointSet GetObjectInitialConfiguration(ros::NodeHandle& nh);

    std::vector<geometry_msgs::Pose> GetRopeNodeTransforms(ros::NodeHandle& nh);

    ObjectPointSet GetCoverPoints(ros::NodeHandle& nh);

    ObjectPointSet GetCoverPointNormals(ros::NodeHandle& nh);

    // TODO: replace these out params with something else
    void GetFreeSpaceGraph(
            ros::NodeHandle& nh,
            arc_dijkstras::Graph<Eigen::Vector3d>& free_space_graph,
            std::vector<int64_t>& cover_ind_to_free_space_graph_ind);

    sdf_tools::SignedDistanceField::ConstPtr GetEnvironmentSDF(ros::NodeHandle& nh);
}

#endif // ROS_COMMUNICATION_HELPERS_H
