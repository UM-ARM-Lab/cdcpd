#include <smmap_models/deformable_model.h>

#include <arc_utilities/arc_exceptions.hpp>
#include <arc_utilities/timing.hpp>

using namespace smmap;

////////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////////

DeformableModel::DeformableModel(
        std::shared_ptr<ros::NodeHandle> nh)
//    : computation_time_log_(std::make_shared<Log::Log>(GetLogFolder(*nh) + "/model_prediction_time.txt"))
{
    (void)nh;
    if (!grippers_data_initialized_.load())
    {
        throw_arc_exception(std::runtime_error,
                            "You must call SetGrippersData before constructing a DeformableObjectModel");
    }

    if (!function_pointers_initialized_.load())
    {
        throw_arc_exception(std::runtime_error,
                            "You must call SetCallbackFunctions before constructing a DeformableObjectModel");
    }
}

////////////////////////////////////////////////////////////////////////////////
// Static member initialization
////////////////////////////////////////////////////////////////////////////////

std::atomic_bool DeformableModel::grippers_data_initialized_(false);
std::vector<GripperData> DeformableModel::grippers_data_;

void DeformableModel::SetGrippersData(
        const std::vector<GripperData>& grippers_data)
{
    grippers_data_ = grippers_data;
    grippers_data_initialized_.store(true);
}

const std::vector<GripperData>& DeformableModel::GetGrippersData()
{
    assert(grippers_data_initialized_);
    return grippers_data_;
}

std::atomic_bool DeformableModel::function_pointers_initialized_(false);
GripperCollisionCheckFunction DeformableModel::gripper_collision_check_fn_;

void DeformableModel::SetCallbackFunctions(
        const GripperCollisionCheckFunction& gripper_collision_check_fn)
{
    gripper_collision_check_fn_ = gripper_collision_check_fn;
    function_pointers_initialized_.store(true);
}

////////////////////////////////////////////////////////////////////////////////
// Interface wrappers
////////////////////////////////////////////////////////////////////////////////

void DeformableModel::updateModel(
        const WorldState& previous,
        const WorldState& next)
{
    return updateModel_impl(previous, next);
}

ObjectPointSet DeformableModel::getObjectDelta(
        const WorldState& world_state,
        const AllGrippersSinglePoseDelta& grippers_pose_delta) const
{
//    arc_utilities::Stopwatch stopwatch;
    const auto retval = getObjectDelta_impl(world_state, grippers_pose_delta);
//    const double time = stopwatch(arc_utilities::READ);
//    LOG(*computation_time_log_, time);
    return retval;
}
