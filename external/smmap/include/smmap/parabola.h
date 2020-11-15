#ifndef PARABOLA_H
#define PARABOLA_H

#include <sdf_tools/collision_map.hpp>
#include <smmap_utilities/grippers.h>
#include <smmap_utilities/visualization_tools.h>
#include "smmap/quinlan_rubber_band.h"

namespace smmap
{
    sdf_tools::CollisionMapGrid ExtractParabolaSliceBasic(
            const sdf_tools::SignedDistanceField& sdf,
            const double resolution,
            const PairGripperPositions& gripper_positions,
            const double parabola_length,
            const Visualizer::Ptr vis = nullptr);

    sdf_tools::CollisionMapGrid ExtractParabolaSliceInPlane(
            const sdf_tools::SignedDistanceField& sdf,
            const double resolution,
            const PairGripperPositions& gripper_positions,
            const Eigen::Vector3d& band_midpoint,
            const double parabola_length,
            const bool gravity_aligned, // toggles choice of parabola extension
            const Visualizer::Ptr vis = nullptr);

    sdf_tools::CollisionMapGrid ExtractParabolaSliceExtendDownwards(
            const sdf_tools::SignedDistanceField& sdf,
            const double resolution,
            const PairGripperPositions& gripper_positions,
            const Eigen::Vector3d& band_midpoint,
            const double parabola_length,
            const bool gravity_aligned,
            const Visualizer::Ptr vis = nullptr);

    // Does work to call one of the above (or subtle variants thereof)
    sdf_tools::CollisionMapGrid ExtractParabolaSlice(
            const sdf_tools::SignedDistanceField& sdf,
            const double resolution,
            const PairGripperPositions& gripper_positions,
            const RubberBand::ConstPtr& band,
            const std::string& parabola_slice_option,
            const Visualizer::Ptr vis = nullptr);
}

#endif // PARABOLA_H
