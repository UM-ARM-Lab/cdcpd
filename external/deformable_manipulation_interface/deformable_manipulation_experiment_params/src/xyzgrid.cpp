#include <arc_utilities/arc_helpers.hpp>
#include "deformable_manipulation_experiment_params/xyzgrid.h"

using namespace smmap;

XYZGrid::XYZGrid(const Eigen::Isometry3d& origin_transform,
                 const std::string& frame,
                 const double world_x_step,
                 const double world_y_step,
                 const double world_z_step,
                 const ssize_t world_x_num_steps,
                 const ssize_t world_y_num_steps,
                 const ssize_t world_z_num_steps)
    : frame_(frame)
    , transform_from_world_to_index0_(origin_transform)
    , transform_from_index0_to_world_(origin_transform.inverse())
    , world_x_step_(world_x_step)
    , world_y_step_(world_y_step)
    , world_z_step_(world_z_step)
    , inv_world_x_step_(1.0 / world_x_step)
    , inv_world_y_step_(1.0 / world_y_step)
    , inv_world_z_step_(1.0 / world_z_step)
    , world_x_num_steps_(world_x_num_steps)
    , world_y_num_steps_(world_y_num_steps)
    , world_z_num_steps_(world_z_num_steps)
{}

ssize_t XYZGrid::xyzIndexToGridIndex(const ssize_t x_ind, const ssize_t y_ind, const ssize_t z_ind) const
{
    // If the point is in the grid, return the index
    if ((0 <= x_ind && x_ind < world_x_num_steps_) &&
        (0 <= y_ind && y_ind < world_y_num_steps_) &&
        (0 <= z_ind && z_ind < world_z_num_steps_))
    {
        return (x_ind * world_y_num_steps_ + y_ind) * world_z_num_steps_ + z_ind;
    }
    // Otherwise return -1
    else
    {
        return -1;
    }
}

Eigen::Vector3d XYZGrid::xyzIndexToWorldPosition(const ssize_t x_ind, const ssize_t y_ind, const ssize_t z_ind) const
{
    Eigen::Vector3d grid_aligned_pos;
    grid_aligned_pos.x() = (double)x_ind * world_x_step_;
    grid_aligned_pos.y() = (double)y_ind * world_y_step_;
    grid_aligned_pos.z() = (double)z_ind * world_z_step_;

    return transform_from_world_to_index0_ * grid_aligned_pos;
}

ssize_t XYZGrid::worldPosToGridIndex(const double x, const double y, const double z) const
{
    return worldPosToGridIndex(Eigen::Vector3d(x, y, z));
}

ssize_t XYZGrid::worldPosToGridIndex(const Eigen::Vector3d& pos) const
{
    const Eigen::Vector3d grid_aligned_pos = transform_from_index0_to_world_ * pos;

    const ssize_t x_ind = std::lround(grid_aligned_pos.x() * inv_world_x_step_);
    const ssize_t y_ind = std::lround(grid_aligned_pos.y() * inv_world_y_step_);
    const ssize_t z_ind = std::lround(grid_aligned_pos.z() * inv_world_z_step_);

    return xyzIndexToGridIndex(x_ind, y_ind, z_ind);
}

ssize_t XYZGrid::worldPosToGridIndexClamped(const double x, const double y, const double z) const
{
    return worldPosToGridIndexClamped(Eigen::Vector3d(x, y, z));
}

ssize_t XYZGrid::worldPosToGridIndexClamped(const Eigen::Vector3d& pos) const
{
    const Eigen::Vector3d grid_aligned_pos = transform_from_index0_to_world_ * pos;

    const ssize_t x_ind = std::lround(grid_aligned_pos.x() * inv_world_x_step_);
    const ssize_t y_ind = std::lround(grid_aligned_pos.y() * inv_world_y_step_);
    const ssize_t z_ind = std::lround(grid_aligned_pos.z() * inv_world_z_step_);

    return xyzIndexToGridIndex(
                arc_helpers::ClampValue(x_ind, 0L, world_x_num_steps_ - 1),
                arc_helpers::ClampValue(y_ind, 0L, world_y_num_steps_ - 1),
                arc_helpers::ClampValue(z_ind, 0L, world_z_num_steps_ - 1));
}

Eigen::Vector3d XYZGrid::roundToGrid(const Eigen::Vector3d& pos) const
{
    const Eigen::Vector3d grid_aligned_pos = transform_from_index0_to_world_ * pos;

    const ssize_t x_ind = std::lround(grid_aligned_pos.x() * inv_world_x_step_);
    const ssize_t y_ind = std::lround(grid_aligned_pos.y() * inv_world_y_step_);
    const ssize_t z_ind = std::lround(grid_aligned_pos.z() * inv_world_z_step_);

    return xyzIndexToWorldPosition(x_ind, y_ind, z_ind);
}


bool XYZGrid::operator==(const XYZGrid& other) const
{
    if (frame_ != other.frame_)
    {
        return false;
    }
    if ((transform_from_world_to_index0_.matrix().array() !=
         other.transform_from_world_to_index0_.matrix().array()).any())
    {
        return false;
    }
    if ((transform_from_index0_to_world_.matrix().array() !=
         other.transform_from_index0_to_world_.matrix().array()).any())
    {
        return false;
    }
    if (world_x_step_ != other.world_x_step_)
    {
        return false;
    }
    if (world_y_step_ != other.world_y_step_)
    {
        return false;
    }
    if (world_z_step_ != other.world_z_step_)
    {
        return false;
    }
    if (inv_world_x_step_ != other.inv_world_x_step_)
    {
        return false;
    }
    if (inv_world_y_step_ != other.inv_world_y_step_)
    {
        return false;
    }
    if (inv_world_z_step_ != other.inv_world_z_step_)
    {
        return false;
    }
    if (world_x_num_steps_ != other.world_x_num_steps_)
    {
        return false;
    }
    if (world_y_num_steps_ != other.world_y_num_steps_)
    {
        return false;
    }
    if (world_z_num_steps_ != other.world_z_num_steps_)
    {
        return false;
    }
    return true;
}

bool XYZGrid::operator!=(const XYZGrid& other) const
{
    return !(*this == other);
}
