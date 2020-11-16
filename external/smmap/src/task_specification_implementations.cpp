#include "smmap/task_specification_implementions.h"
#include <arc_utilities/arc_exceptions.hpp>
#include <arc_utilities/eigen_helpers.hpp>

using namespace smmap;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Colab folding
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ClothColabFolding::ClothColabFolding(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : TaskSpecification(nh, ph, vis)
    , neighbours_(num_nodes_, GetClothNumControlPointsX(*nh_))
    , point_reflector_(CreatePointReflector(*nh_))
    , mirror_map_(CreateMirrorMap(*nh_, point_reflector_))
{}

double ClothColabFolding::calculateError_impl(
        const WorldState& world_state)
{
    const ObjectPointSet& current_configuration = world_state.object_configuration_;

    double error = 0;
    for (std::map<long, long>::const_iterator ittr = mirror_map_.begin(); ittr != mirror_map_.end(); ++ittr)
    {
        error += (current_configuration.col(ittr->second) -
                   point_reflector_.reflect(current_configuration.col(ittr->first))).norm();
    }

    return error;
}

ObjectDeltaAndWeight ClothColabFolding::calculateObjectErrorCorrectionDelta_impl(
        const WorldState& world_state)
{
    ROS_INFO_NAMED("cloth_colab_folding" , "Finding 'best' cloth delta");
    const ObjectPointSet& object_configuration = world_state.object_configuration_;

    ObjectDeltaAndWeight desired_cloth_delta(object_configuration.cols() * 3);

    long robot_cloth_points_ind = 0;
    for (std::map<long, long>::const_iterator ittr = mirror_map_.begin();
          ittr != mirror_map_.end(); ++ittr, ++robot_cloth_points_ind)
    {
        desired_cloth_delta.delta.segment<3>(ittr->second * 3) =
                point_reflector_.reflect(object_configuration.block<3, 1>(0, ittr->first))
                - object_configuration.block<3, 1>(0, ittr->second);

        const double weight = desired_cloth_delta.delta.segment<3>(ittr->second * 3).norm();
        desired_cloth_delta.weight(ittr->second * 3) = weight;
        desired_cloth_delta.weight(ittr->second * 3 + 1) = weight;
        desired_cloth_delta.weight(ittr->second * 3 + 2) = weight;
    }

    return desired_cloth_delta;
}

std::vector<ssize_t> ClothColabFolding::getNodeNeighbours_impl(const ssize_t node) const
{
    return neighbours_.getNodeNeighbours(node);
}

PointReflector ClothColabFolding::CreatePointReflector(ros::NodeHandle& nh)
{
    ROS_INFO_NAMED("cloth_colab_folding" , "Getting mirror line");

    // Get the initial configuration of the object
    ros::ServiceClient mirror_line_client =
        nh.serviceClient<deformable_manipulation_msgs::GetMirrorLine>(GetMirrorLineTopic(nh));

    mirror_line_client.waitForExistence();

    deformable_manipulation_msgs::GetMirrorLine mirror_line_data;
    mirror_line_client.call(mirror_line_data);
    CHECK_FRAME_NAME("cloth_colab_folding", GetWorldFrameName(), mirror_line_data.response.header.frame_id);

    return PointReflector(mirror_line_data.response.mid_x,
                          mirror_line_data.response.min_y,
                          mirror_line_data.response.max_y);
}

std::map<long, long> ClothColabFolding::CreateMirrorMap(ros::NodeHandle& nh, const PointReflector& point_reflector)
{
    ROS_INFO_NAMED("cloth_colab_folding", "Finding point correspondences");

    const ObjectPointSet object_initial_configuration = GetObjectInitialConfiguration(nh);

    std::map<long, long> mirror_map;
    for (long node_ind = 0; node_ind < object_initial_configuration.cols(); node_ind++)
    {
        // for every node on one side of the mirror line, find the closest match on the other side
        // Note that nodes that have an x value > than mid_x are on the manual gripper side
        if (object_initial_configuration(0, node_ind) > point_reflector.get_mid_x())
        {
            const long mirror_ind = EigenHelpers::ClosestPointInSet(object_initial_configuration,
                    point_reflector.reflect(object_initial_configuration.col(node_ind)));

            mirror_map[node_ind] = mirror_ind;
        }
    }

    return mirror_map;
}

bool ClothColabFolding::taskDone_impl(
        const WorldState& world_state)
{
    (void)world_state;
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Rope cylinder coverage
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RopeDirectCoverage::RopeDirectCoverage(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : DirectCoverageTask(nh, ph, vis)
    , neighbours_(num_nodes_)
{}

std::vector<ssize_t> RopeDirectCoverage::getNodeNeighbours_impl(const ssize_t node) const
{
    return neighbours_.getNodeNeighbours(node);
}

bool RopeDirectCoverage::taskDone_impl(
        const WorldState& world_state)
{
    return calculateError(world_state) < error_threshold_task_done_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cloth table coverage
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ClothDirectCoverage::ClothDirectCoverage(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : DirectCoverageTask(nh, ph, vis)
    , neighbours_(num_nodes_, GetClothNumControlPointsX(*nh_))
{}

std::vector<ssize_t> ClothDirectCoverage::getNodeNeighbours_impl(const ssize_t node) const
{
    return neighbours_.getNodeNeighbours(node);
}

bool ClothDirectCoverage::taskDone_impl(
        const WorldState& world_state)
{
    return calculateError(world_state) < error_threshold_task_done_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Rope - Distance Based Correspondences - Using Dijkstras
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RopeDistanceBasedCorrespondences::RopeDistanceBasedCorrespondences(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : DistanceBasedCorrespondencesTask(nh, ph, vis)
    , neighbours_(num_nodes_)
{}

std::vector<ssize_t> RopeDistanceBasedCorrespondences::getNodeNeighbours_impl(const ssize_t node) const
{
    return neighbours_.getNodeNeighbours(node);
}

bool RopeDistanceBasedCorrespondences::taskDone_impl(
        const WorldState& world_state)
{
    return calculateError(world_state) < error_threshold_task_done_;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cloth - Distance Based Correspondences - Using Dijkstras
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ClothDistanceBasedCorrespondences::ClothDistanceBasedCorrespondences(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : DistanceBasedCorrespondencesTask(nh, ph, vis)
    , neighbours_(num_nodes_, GetClothNumControlPointsX(*nh_))
{
    if (task_type_ == CLOTH_WALL)
    {
        throw_arc_exception(std::invalid_argument, "Cloth Wall experiment needs revisions before it is used");
    }
    else if (task_type_ == CLOTH_WAFR)
    {
        ROS_WARN_NAMED("task_specification_implementation", "Cloth WAFR task needs to be revised");
    }
    else if (task_type_ == CLOTH_CYLINDER_COVERAGE)
    {
        throw_arc_exception(std::invalid_argument, "Cloth Cylinder experiment has not been updated in over 2 years");
    }
}

std::vector<ssize_t> ClothDistanceBasedCorrespondences::getNodeNeighbours_impl(const ssize_t node) const
{
    return neighbours_.getNodeNeighbours(node);
}

bool ClothDistanceBasedCorrespondences::taskDone_impl(
        const WorldState& world_state)
{
    return calculateError(world_state) < error_threshold_task_done_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Rope - Fixed Correspondences - Using Dijkstras
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RopeFixedCorrespondences::RopeFixedCorrespondences(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : FixedCorrespondencesTask(nh, ph, vis)
    , neighbours_(num_nodes_)
{
    assert(num_nodes_ == num_cover_points_);

    correspondences_internal_fixed_.clear();
    correspondences_internal_fixed_.reserve(num_nodes_);

    for (ssize_t idx = 0; idx < num_nodes_; ++idx)
    {
        correspondences_internal_fixed_.push_back({idx});
    }
}

std::vector<ssize_t> RopeFixedCorrespondences::getNodeNeighbours_impl(const ssize_t node) const
{
    return neighbours_.getNodeNeighbours(node);
}

bool RopeFixedCorrespondences::taskDone_impl(
        const WorldState& world_state)
{
    return calculateError(world_state) < error_threshold_task_done_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cloth - Fixed Correspondences - Using Dijkstras
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ClothFixedCorrespondences::ClothFixedCorrespondences(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        Visualizer::Ptr vis)
    : FixedCorrespondencesTask(nh, ph, vis)
    , neighbours_(num_nodes_, GetClothNumControlPointsX(*nh_))
{
    assert(num_nodes_ == num_cover_points_);

    correspondences_internal_fixed_.clear();
    correspondences_internal_fixed_.reserve(num_nodes_);

    for (ssize_t idx = 0; idx < num_nodes_; ++idx)
    {
        correspondences_internal_fixed_.push_back({idx});
    }
}

std::vector<ssize_t> ClothFixedCorrespondences::getNodeNeighbours_impl(const ssize_t node) const
{
    return neighbours_.getNodeNeighbours(node);
}

bool ClothFixedCorrespondences::taskDone_impl(
        const WorldState& world_state)
{
    return calculateError(world_state) < error_threshold_task_done_;
}
