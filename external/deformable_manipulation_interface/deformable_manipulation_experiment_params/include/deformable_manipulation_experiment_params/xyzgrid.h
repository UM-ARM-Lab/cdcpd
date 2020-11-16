#ifndef XYZGRID_H
#define XYZGRID_H

#include <Eigen/Dense>

namespace smmap
{
    class XYZGrid
    {
        public:
            XYZGrid(const Eigen::Isometry3d& origin_transform,
                    const std::string& frame,
                    const double world_x_step,
                    const double world_y_step,
                    const double world_z_step,
                    const ssize_t world_x_num_steps,
                    const ssize_t world_y_num_steps,
                    const ssize_t world_z_num_steps);

            const std::string& getFrame() const
            {
                return frame_;
            }

            const Eigen::Isometry3d& getOriginTransform() const
            {
                return transform_from_world_to_index0_;
            }

            const Eigen::Isometry3d& getInverseOriginTransform() const
            {
                return transform_from_index0_to_world_;
            }

            double minStepDimension() const
            {
                return std::min({world_x_step_, world_y_step_, world_z_step_});
            }

            int64_t getXNumSteps() const
            {
                return world_x_num_steps_;
            }

            int64_t getYNumSteps() const
            {
                return world_y_num_steps_;
            }

            int64_t getZNumSteps() const
            {
                return world_z_num_steps_;
            }

            int64_t getNumCells() const
            {
                return world_x_num_steps_ * world_y_num_steps_ * world_z_num_steps_;
            }

            ssize_t xyzIndexToGridIndex(const ssize_t x_ind, const ssize_t y_ind, const ssize_t z_ind) const;
            Eigen::Vector3d xyzIndexToWorldPosition(const ssize_t x_ind, const ssize_t y_ind, const ssize_t z_ind) const;
            ssize_t worldPosToGridIndex(const double x, const double y, const double z) const;
            ssize_t worldPosToGridIndex(const Eigen::Vector3d& pos) const;
            ssize_t worldPosToGridIndexClamped(const double x, const double y, const double z) const;
            ssize_t worldPosToGridIndexClamped(const Eigen::Vector3d& pos) const;

            Eigen::Vector3d roundToGrid(const Eigen::Vector3d& pos) const;

            bool operator==(const XYZGrid& other) const;
            bool operator!=(const XYZGrid& other) const;

        private:
            /// Variables describing the frame of the grid
            const std::string frame_;
            const Eigen::Isometry3d transform_from_world_to_index0_; // transform from the origin of frame_ to the grid cell at index (0, 0, 0)
            const Eigen::Isometry3d transform_from_index0_to_world_; // transform from the grid cell at index (0, 0, 0) to the origin of frame_

            /// Variables describing the extents of the graph
            const double world_x_step_;
            const double world_y_step_;
            const double world_z_step_;
            const double inv_world_x_step_;
            const double inv_world_y_step_;
            const double inv_world_z_step_;
            const ssize_t world_x_num_steps_;
            const ssize_t world_y_num_steps_;
            const ssize_t world_z_num_steps_;
    };
}

#endif // XYZGRID_H
