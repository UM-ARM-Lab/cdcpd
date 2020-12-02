#include <thread>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/filesystem.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/serialization_eigen.hpp>
#include <arc_utilities/path_utils.hpp>
#include <arc_utilities/simple_astar_planner.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include <smmap_utilities/ros_communication_helpers.h>
#include "smmap/quinlan_rubber_band.h"
#include <smmap_utilities/trajectory.hpp>

#define ENABLE_BAND_LOAD_SAVE 0
//#define ENABLE_BAND_LOAD_SAVE 1

using ColorBuilder = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>;

namespace smmap
{
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                Path between the grippers                           ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief GetShortestPathBetweenGrippersThroughObject
     * @param grippers_data
     * @param object
     * @param neighour_fn
     * @return The index of the nodes between the grippers, following the shortest path through the object
     */
    static std::vector<ssize_t> GetShortestPathBetweenGrippersThroughObject(
            const std::vector<GripperData>& grippers_data,
            const ObjectPointSet& object,
            const std::function<std::vector<ssize_t>(const ssize_t& node)>& neighbour_fn)
    {
        assert(grippers_data.size() == 2);
        assert(grippers_data[0].node_indices_.size() > 0);
        assert(grippers_data[1].node_indices_.size() > 0);

        const auto start = grippers_data[0].node_indices_[0];
        const auto goal = grippers_data[1].node_indices_[0];
        const auto distance_fn = [&] (const ssize_t& first_node, const ssize_t& second_node)
        {
            return (object.col(first_node) - object.col(second_node)).norm();
        };
        const auto heuristic_fn = [&] (const ssize_t& node)
        {
            return distance_fn(node, goal);
        };
        const auto goal_reached_fn = [&] (const ssize_t& test_node)
        {
            return test_node == goal;
        };
        const auto astar_results = simple_astar_planner::SimpleAStarPlanner<ssize_t>::Plan(
                    start, neighbour_fn, distance_fn, heuristic_fn, goal_reached_fn);

        const auto plan = astar_results.first;
        assert(plan.size() > 0);
        return plan;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////                QuinlanRubberBand                                   ////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    std::atomic<bool> QuinlanRubberBand::ENABLE_BAND_DEBUGGING_(false);
    std::atomic<bool> QuinlanRubberBand::ENABLE_INTERPOLATE_DEBUGGING_(false);
    std::atomic<bool> QuinlanRubberBand::ENABLE_REMOVE_DEBUGGING_(false);
    std::atomic<bool> QuinlanRubberBand::ENABLE_SMOOTHING_DEBUGGING_(false);

    ObjectPointSet QuinlanRubberBand::AggregateBandPoints(
            const std::vector<QuinlanRubberBand>& bands)
    {
        if (bands.size() == 0)
        {
            return ObjectPointSet();
        }

        const size_t points_per_band = bands[0].upsampleBand().size();
        ObjectPointSet points(3, points_per_band * bands.size());
        for (size_t band_idx = 0; band_idx < bands.size(); ++band_idx)
        {
            const auto band_points = bands[band_idx].upsampleBand();
            for (size_t point_idx = 0; point_idx < band_points.size(); ++point_idx)
            {
                points.col(band_idx * points_per_band + point_idx) =
                        band_points[point_idx];
            }
        }
        return points;
    }

    ObjectPointSet QuinlanRubberBand::AggregateBandPoints(
            const std::vector<QuinlanRubberBand::Ptr>& bands)
    {
        if (bands.size() == 0)
        {
            return ObjectPointSet();
        }

        const size_t points_per_band = bands[0]->upsampleBand().size();
        ObjectPointSet points(3, points_per_band * bands.size());
        for (size_t band_idx = 0; band_idx < bands.size(); ++band_idx)
        {
            if (bands[band_idx] != nullptr)
            {
                const auto band_points = bands[band_idx]->upsampleBand();
                for (size_t point_idx = 0; point_idx < band_points.size(); ++point_idx)
                {
                    points.col(band_idx * points_per_band + point_idx) =
                            band_points[point_idx];
                }
            }
            else
            {
                ROS_WARN_NAMED("rubber_band", "Invalid band in aggregate band points");
            }
        }
        return points;
    }

    ObjectPointSet QuinlanRubberBand::AggregateBandPoints(
            const std::vector<QuinlanRubberBand::ConstPtr>& bands)
    {
        if (bands.size() == 0)
        {
            return ObjectPointSet();
        }

        const size_t points_per_band = bands[0]->upsampleBand().size();
        ObjectPointSet points(3, points_per_band * bands.size());
        for (size_t band_idx = 0; band_idx < bands.size(); ++band_idx)
        {
            if (bands[band_idx] != nullptr)
            {
                const auto band_points = bands[band_idx]->upsampleBand();
                for (size_t point_idx = 0; point_idx < band_points.size(); ++point_idx)
                {
                    points.col(band_idx * points_per_band + point_idx) =
                            band_points[point_idx];
                }

            }
            else
            {
                ROS_WARN_NAMED("rubber_band", "Invalid band in aggregate band points");
            }
        }
        return points;
    }

    ObjectPointSet QuinlanRubberBand::PointsFromBandPointsAndGripperTargets(
            const EigenHelpers::VectorVector3d& starting_points,
            const PairGripperPositions& grippers_targets,
            const size_t num_gripper_steps)
    {
        assert(num_gripper_steps > 0);
        ObjectPointSet result(3, starting_points.size() + 2 * num_gripper_steps);
        // Put the interpolated points between one end of the band and the first gripper into the result
        for (size_t idx = 0; idx < num_gripper_steps; ++idx)
        {
            const double ratio = num_gripper_steps == 1 ? 0.0 : (double)idx / (double)(num_gripper_steps - 1);
            result.col(idx) = EigenHelpers::Interpolate(grippers_targets.first, starting_points.front(), ratio);
        }
        // Put the band points themselves into the result
        for (size_t idx = 0; idx < starting_points.size(); ++idx)
        {
            result.col(num_gripper_steps + idx) = starting_points[idx];
        }
        // Put the interpolated points between the other end of the band and the second gripper into the result
        for (size_t idx = 0; idx < num_gripper_steps; ++idx)
        {
            const double ratio = num_gripper_steps == 1 ? 0.0 : (double)idx / (double)(num_gripper_steps - 1);
            result.col(num_gripper_steps + starting_points.size() + idx) =
                    EigenHelpers::Interpolate(grippers_targets.second, starting_points.back(), ratio);
        }
        return result;
    }

    ObjectPointSet QuinlanRubberBand::PointsFromBandAndGrippers(
            const QuinlanRubberBand& band,
            const PairGripperPositions& grippers_start,
            const PairGripperPositions& grippers_end,
            const size_t num_gripper_steps)
    {
        assert(num_gripper_steps > 0);
        const EigenHelpers::VectorVector3d& band_points = band.upsampleBand();
        ObjectPointSet result(3, band_points.size() + 2 * (num_gripper_steps + 1));
        // Put the interpolated points the first start and end point into the result
        {
            for (size_t idx = 0; idx < num_gripper_steps + 1; ++idx)
            {
                const double ratio = (double)idx / (double)(num_gripper_steps);
                result.col(idx) = EigenHelpers::Interpolate(grippers_end.first, grippers_start.first, ratio);
            }
        }
        // Put the band points themselves into the result
        {
            const auto offset = num_gripper_steps + 1;
            for (size_t band_idx = 0; band_idx < band_points.size(); ++band_idx)
            {
                result.col(offset + band_idx) = band_points[band_idx];
            }
        }
        // Put the interpolated points the second start and end point into the result
        {
            const auto offset = num_gripper_steps + 1 + band_points.size();
            for (size_t idx = 0; idx < num_gripper_steps + 1; ++idx)
            {
                const double ratio = (double)idx / (double)(num_gripper_steps);
                result.col(offset + idx) = EigenHelpers::Interpolate(grippers_end.second, grippers_start.second, ratio);
            }
        }
        return result;
    }

    QuinlanRubberBand::Ptr QuinlanRubberBand::BandFromNodeTransformsAndGrippers(
            const EigenHelpers::VectorIsometry3d& node_transforms,
            const PairGripperPositions& grippers_position,
            const QuinlanRubberBand& template_band)
    {
        ObjectPointSet pointset(3, node_transforms.size());
        for (size_t idx = 0; idx < node_transforms.size(); ++idx)
        {
            pointset.col(idx) = node_transforms[idx].translation();
        }

        auto band = std::make_shared<QuinlanRubberBand>(template_band);
        if (!band->resetBand(pointset, grippers_position))
        {
            throw_arc_exception(std::runtime_error, "Unable to build band");
        }
        return band;
    }

    QuinlanRubberBand::Ptr QuinlanRubberBand::BandFromWorldState(
            const WorldState& world_state,
            const QuinlanRubberBand& template_band)
    {
        auto band = std::make_shared<QuinlanRubberBand>(template_band);
        if (!band->resetBand(world_state))
        {
            throw_arc_exception(std::runtime_error, "Unable to build band");
        }
        return band;
    }

    QuinlanRubberBand::QuinlanRubberBand(
            std::shared_ptr<ros::NodeHandle> nh,
            std::shared_ptr<ros::NodeHandle> ph,
            const Visualizer::Ptr vis,
            const sdf_tools::SignedDistanceField::ConstPtr& sdf,
            const XYZGrid& work_space_grid,
            const std::function<std::vector<ssize_t>(const ssize_t node)>& node_neighbours_fn,
            const WorldState& world_state,
            const double resample_max_pointwise_dist,
            const size_t upsample_num_points,
            const double max_safe_band_length)
        : nh_(nh)
        , ph_(std::make_shared<ros::NodeHandle>(ph->getNamespace() + "/band"))
        , sdf_(sdf)
        , work_space_grid_(work_space_grid)
        , vis_(vis)

        , path_between_grippers_through_object_(GetShortestPathBetweenGrippersThroughObject(
                                                    GetGrippersData(*nh_),
                                                    GetObjectInitialConfiguration(*nh),
                                                    node_neighbours_fn))
        , band_()
        , resampled_band_()
        , resample_max_pointwise_dist_(resample_max_pointwise_dist)
        , upsampled_band_()
        , upsampled_band_single_vector_(upsample_num_points)
        , upsample_num_points_(upsample_num_points)

        , max_safe_band_length_(max_safe_band_length)
        , min_overlap_distance_(work_space_grid_.minStepDimension() * MIN_OVERLAP_SCALE_FACTOR)
        , min_distance_to_obstacle_(work_space_grid_.minStepDimension() * MIN_DIST_TO_OBSTACLES_SCALE_FACTOR)
        , node_removal_overlap_factor_(NODE_REMOVAL_OVERLAP_FACTOR)
        , backtrack_threshold_(BACKTRACKING_THRESHOLD)
        , smoothing_iterations_(SMOOTHING_ITERATIONS)
    {
        assert(resetBand(world_state));
        assert(bandIsValidWithVisualization());
    }

    QuinlanRubberBand& QuinlanRubberBand::operator=(const QuinlanRubberBand& other)
    {
        assert(sdf_.get() == other.sdf_.get());
        assert(work_space_grid_ == other.work_space_grid_);
        assert(vis_.get() == (other.vis_.get()));
        assert(path_between_grippers_through_object_ == other.path_between_grippers_through_object_);

        assert(max_safe_band_length_ == other.max_safe_band_length_);

        band_ = other.band_;
        resampled_band_ = other.resampled_band_;
        assert(resample_max_pointwise_dist_ == other.resample_max_pointwise_dist_);
        upsampled_band_ = other.upsampled_band_;
        assert(upsample_num_points_ == other.upsample_num_points_);
        upsampled_band_single_vector_ = other.upsampled_band_single_vector_;
        #if ENABLE_BAND_LOAD_SAVE
        if (useStoredBand())
        {
            loadStoredBand();
        }
        else
        {
            storeBand();
        }
        #endif

        assert(bandIsValidWithVisualization());
        return *this;
    }

    bool QuinlanRubberBand::setPointsWithoutSmoothing(const EigenHelpers::VectorVector3d& points)
    {
        band_ = points;
        resampled_band_.clear();
        upsampled_band_.clear();

        #if ENABLE_BAND_LOAD_SAVE
            if (useStoredBand())
            {
                loadStoredBand();
            }
            else
            {
                storeBand();
            }
        #endif

        for (auto& point: band_)
        {
            point = projectToValidBubble(point);
        }
        if (ENABLE_BAND_DEBUGGING_)
        {
            vis_->forcePublishNow();
        }
        if (!interpolateBandPoints())
        {
            return false;
        }
        else
        {
            const bool verbose = true;
            if (ENABLE_BAND_DEBUGGING_)
            {
                vis_->forcePublishNow();
            }
            removeExtraBandPoints(verbose);
            if (ENABLE_BAND_DEBUGGING_)
            {
                vis_->forcePublishNow();
                assert(bandIsValidWithVisualization());
            }
            return true;
        }
    }

    bool QuinlanRubberBand::setPointsWithoutSmoothing(const ObjectPointSet& points)
    {
        return setPointsWithoutSmoothing(EigenHelpersConversions::EigenMatrix3XdToVectorEigenVector3d(points));
    }

    bool QuinlanRubberBand::setPointsAndSmooth(const EigenHelpers::VectorVector3d& points)
    {
        if (!setPointsWithoutSmoothing(points))
        {
            return false;
        }
        if (!smoothBandPoints(true))
        {
            return false;
        }
        assert(bandIsValidWithVisualization());
        return true;
    }

    bool QuinlanRubberBand::setPointsAndSmooth(const ObjectPointSet& points)
    {
        return setPointsAndSmooth(EigenHelpersConversions::EigenMatrix3XdToVectorEigenVector3d(points));
    }

    bool QuinlanRubberBand::resetBand(const WorldState& world_state)
    {
        assert(world_state.all_grippers_single_pose_.size() == 2);
        return resetBand(world_state.object_configuration_,
                         ToGripperPositions(world_state.all_grippers_single_pose_));
    }

    bool QuinlanRubberBand::resetBand(
            const ObjectPointSet& object_config,
            const PairGripperPositions& gripper_positions)
    {
        EigenHelpers::VectorVector3d points;
        points.reserve(path_between_grippers_through_object_.size() + 2);
        points.push_back(gripper_positions.first);

        // Extract the correct points from the deformable object
        for (size_t path_idx = 0; path_idx < path_between_grippers_through_object_.size(); ++path_idx)
        {
            const ssize_t node_idx = path_between_grippers_through_object_[path_idx];
            points.push_back(object_config.col(node_idx));
        }

        points.push_back(gripper_positions.second);
        return setPointsAndSmooth(points);
    }

    void QuinlanRubberBand::overridePoints(const EigenHelpers::VectorVector3d& points)
    {
        band_ = points;
        resampled_band_.clear();
        upsampled_band_.clear();
    }

    /**
     * @brief QuinlanRubberBand::forwardSimulateVirtualRubberBandToEndpointTargets This function assumes that the endpoints
     * that are passed are collision free, and the path between them can be resampled without any issues. We may have to
     * project the resampled points out of collision, but it should be feasible to do so without changing the "homotopy" class
     * @param first_endpoint_target
     * @param second_endpoint_target
     * @param verbose
     * @return
     */
    const EigenHelpers::VectorVector3d& QuinlanRubberBand::forwardPropagate(
            const PairGripperPositions& gripper_positions,
            bool verbose)
    {
        assert(bandIsValidWithVisualization());

        // Ensure that the new points are both in bounds, and are at least min_distance_to_obstacle_ from anything
        // Add the new endpoints, then let the interpolate and smooth process handle the propogation
        band_.insert(band_.begin(), projectToValidBubble(gripper_positions.first));
        band_.push_back(projectToValidBubble(gripper_positions.second));
        resampled_band_.clear();
        upsampled_band_.clear();

        #if ENABLE_BAND_LOAD_SAVE
            if (useStoredBand())
            {
                loadStoredBand();
            }
            else
            {
                storeBand();
            }
        #endif

        if (!interpolateBandPoints())
        {
            throw_arc_exception(std::runtime_error, "Unable to forward propagate band");
        }
        removeExtraBandPoints(verbose);
        if (!smoothBandPoints(verbose))
        {
            throw_arc_exception(std::runtime_error, "Unable to forward propagate band");
        }
        assert(bandIsValidWithVisualization());
        return band_;
    }





    const EigenHelpers::VectorVector3d& QuinlanRubberBand::getVectorRepresentation() const
    {
        return band_;
    }

    const EigenHelpers::VectorVector3d& QuinlanRubberBand::resampleBand() const
    {
        // If our current resampled_band_ cache is invalid, recalculate it
        if (resampled_band_.size() == 0)
        {
            const auto distance_fn = [] (const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
            {
                return (v1 - v2).norm();
            };
            resampled_band_ = path_utils::ResamplePath(
                        band_, resample_max_pointwise_dist_, distance_fn, EigenHelpers::Interpolate<double, 3>);
        }

        return resampled_band_;
    }

    const EigenHelpers::VectorVector3d& QuinlanRubberBand::upsampleBand() const
    {
        assert(upsample_num_points_ >= band_.size());

        // If our current upsampled_band_ cache is invalid, recalculate it
        if (upsampled_band_.size() == 0)
        {
            if (band_.size() == upsample_num_points_)
            {
                upsampled_band_ = band_;
            }
            else
            {
                const auto distance_fn = [] (const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
                {
                    return (v1 - v2).norm();
                };
                upsampled_band_  = path_utils::UpsamplePath<Eigen::Vector3d>(
                            band_, upsample_num_points_, distance_fn, EigenHelpers::Interpolate<double, 3>);
            }
            // Also create the upsampled version here as a way to keep the code simple
            upsampled_band_single_vector_ =
                    EigenHelpers::VectorEigenVectorToEigenVectorX(upsampled_band_);
        }

        return upsampled_band_;
    }

    const Eigen::VectorXd& QuinlanRubberBand::upsampleBandSingleVector() const
    {
        // If the upsampled version is out of date, regenerate it
        // and the corresponding 'single_vector' version
        upsampleBand();
        return upsampled_band_single_vector_;
    }

    Pair3dPositions QuinlanRubberBand::getEndpoints() const
    {
        return {band_.front(), band_.back()};
    }

    double QuinlanRubberBand::maxSafeLength() const
    {
        return max_safe_band_length_;
    }

    double QuinlanRubberBand::totalLength() const
    {
        return EigenHelpers::CalculateTotalDistance(band_);
    }

    bool QuinlanRubberBand::isOverstretched() const
    {
        return totalLength() > max_safe_band_length_;
    }

    std::vector<Visualizer::NamespaceId> QuinlanRubberBand::visualize(
            const std::string& marker_name,
            const std_msgs::ColorRGBA& safe_color,
            const std_msgs::ColorRGBA& overstretched_color,
            const int32_t id,
            const bool visualization_enabled) const
    {
        return visualize(band_, marker_name, safe_color, overstretched_color, id, visualization_enabled);
    }

    std::vector<Visualizer::NamespaceId> QuinlanRubberBand::visualize(
            const EigenHelpers::VectorVector3d& test_band,
            const std::string& marker_name,
            const std_msgs::ColorRGBA& safe_color,
            const std_msgs::ColorRGBA& overstretched_color,
            const int32_t id,
            const bool visualization_enabled) const
    {
        if (!visualization_enabled)
        {
            return {};
        }

        if (isOverstretched())
        {
            return vis_->visualizeXYZTrajectory(marker_name, test_band, overstretched_color, id);
        }
        else
        {
            return vis_->visualizeXYZTrajectory(marker_name, test_band, safe_color, id);
        }
    }

    std::vector<Visualizer::NamespaceId> QuinlanRubberBand::visualizeWithBubbles(
            const std::string& marker_name,
            const std_msgs::ColorRGBA& safe_color,
            const std_msgs::ColorRGBA& overstretched_color,
            const int32_t id,
            const bool visualization_enabled) const
    {
        return visualizeWithBubbles(band_, marker_name, safe_color, overstretched_color, id, visualization_enabled);
    }

    std::vector<Visualizer::NamespaceId> QuinlanRubberBand::visualizeWithBubbles(
            const EigenHelpers::VectorVector3d& test_band,
            const std::string& marker_name,
            const std_msgs::ColorRGBA& safe_color,
            const std_msgs::ColorRGBA& overstretched_color,
            const int32_t id,
            const bool visualization_enabled) const
    {
        if (!visualization_enabled)
        {
            return {};
        }

        auto marker_ids = visualize(test_band, marker_name, safe_color, overstretched_color, id, visualization_enabled);

        if (ENABLE_BAND_DEBUGGING_)
        {
            vis_->forcePublishNow();
            // Delete all sphere, markers, probably from just this publisher, and then republish
            {
                vis_->deleteObjects(marker_name + "_bubbles", 1, 505);
                vis_->forcePublishNow();

                std::vector<double> bubble_sizes(test_band.size());
                std::vector<std_msgs::ColorRGBA> colors(test_band.size());
                for (size_t idx = 0; idx < test_band.size(); ++idx)
                {
                    bubble_sizes[idx] = getBubbleSize(test_band[idx]);
                    colors[idx] = ColorBuilder::MakeFromFloatColors(
                                (float)idx / (float)(test_band.size() - 1),
                                0.0f,
                                (float)(test_band.size() - 1 - idx) / (float)(test_band.size() - 1),
                                0.3f);
                }
                const auto new_ids = vis_->visualizeSpheres(marker_name + "_bubbles", test_band, colors, id, bubble_sizes);
                vis_->forcePublishNow();
                marker_ids.insert(marker_ids.end(),
                                  std::make_move_iterator(new_ids.begin()),
                                  std::make_move_iterator(new_ids.end()));
            }
        }

        return marker_ids;
    }

    std::vector<Visualizer::NamespaceId> QuinlanRubberBand::VisualizeBandSurface(
            const Visualizer::Ptr& vis,
            const ObjectPointSet& band_surface,
            const size_t num_bands,
            const std_msgs::ColorRGBA& start_color,
            const std_msgs::ColorRGBA& end_color,
            const std::string& ns,
            const int32_t id)
    {
        if (num_bands == 0)
        {
            return {};
        }
        assert(band_surface.cols() % num_bands == 0);
        const size_t points_per_band = band_surface.cols() / num_bands;
        std::vector<std_msgs::ColorRGBA> colors;
        for (size_t band_idx = 0; band_idx < num_bands; ++band_idx)
        {
            const float ratio = (float)(band_idx) / (float)(num_bands - 1);
            const auto color = arc_helpers::InterpolateColor(start_color, end_color, ratio);
            colors.insert(colors.end(), points_per_band, color);
        }
        return vis->visualizePoints(ns, band_surface, colors, id, 0.002);
    }

    std::vector<Visualizer::NamespaceId> QuinlanRubberBand::VisualizeBandSurface(
            const Visualizer::Ptr& vis,
            const std::vector<QuinlanRubberBand>& bands,
            const std_msgs::ColorRGBA& start_color,
            const std_msgs::ColorRGBA& end_color,
            const std::string& ns,
            const int32_t id)
    {
        if (bands.empty())
        {
            return {};
        }
        const auto points = RubberBand::AggregateBandPoints(bands);
        return VisualizeBandSurface(vis, points, bands.size(), start_color, end_color, ns, id);
    }

    std::vector<Visualizer::NamespaceId> QuinlanRubberBand::VisualizeBandSurface(
            const Visualizer::Ptr& vis,
            const std::vector<QuinlanRubberBand::Ptr>& bands,
            const std_msgs::ColorRGBA& start_color,
            const std_msgs::ColorRGBA& end_color,
            const std::string& ns,
            const int32_t id)
    {
        if (bands.empty())
        {
            return {};
        }
        const auto points = RubberBand::AggregateBandPoints(bands);
        return VisualizeBandSurface(vis, points, bands.size(), start_color, end_color, ns, id);
    }

    std::vector<Visualizer::NamespaceId> QuinlanRubberBand::VisualizeBandSurface(
            const Visualizer::Ptr& vis,
            const std::vector<QuinlanRubberBand::ConstPtr>& bands,
            const std_msgs::ColorRGBA& start_color,
            const std_msgs::ColorRGBA& end_color,
            const std::string& ns,
            const int32_t id)
    {
        if (bands.empty())
        {
            return {};
        }
        const auto points = RubberBand::AggregateBandPoints(bands);
        return VisualizeBandSurface(vis, points, bands.size(), start_color, end_color, ns, id);
    }





    Eigen::Vector3d QuinlanRubberBand::projectToValidBubble(const Eigen::Vector3d& location) const
    {
        if (getBubbleSize(location) >= min_distance_to_obstacle_)
        {
            return location;
        }

        if (ENABLE_BAND_DEBUGGING_)
        {
            std::cout << "Projecting out of collision" << std::endl;
            vis_->visualizePoint("___point_to_project", location, Visualizer::Yellow(0.3f), 1, 0.05);
            vis_->forcePublishNow();
        }
        const auto post_collision_project = sdf_->ProjectOutOfCollisionToMinimumDistance3d(location, min_distance_to_obstacle_);
        if (ENABLE_BAND_DEBUGGING_)
        {
            std::cout << "Projecting to valid volume" << std::endl;
        }
        const auto post_boundary_project = sdf_->ProjectIntoValidVolumeToMinimumDistance3d(post_collision_project, min_distance_to_obstacle_);

        if (ENABLE_BAND_DEBUGGING_)
        {
            std::cout << std::setprecision(20)
                      << "location:       " << location.transpose() << std::endl
                      << "post collision: " << post_collision_project.transpose() << std::endl
                      << "post boundary:  " << post_boundary_project.transpose() << std::endl
                      << "Resulting size:        " << getBubbleSize(post_boundary_project) << std::endl;


            if (getBubbleSize(post_boundary_project) < min_distance_to_obstacle_)
            {
                const auto distance_to_boundary = sdf_->DistanceToBoundary3d(post_boundary_project);
                const auto distance_to_obstacles = sdf_->EstimateDistance3d(post_boundary_project);

                if (distance_to_boundary.first < min_distance_to_obstacle_ ||
                    distance_to_obstacles.first < min_distance_to_obstacle_)
                {
                    constexpr int p = 20;

                    const auto starting_distance_to_obstacles = sdf_->EstimateDistance3d(location);
                    const auto starting_distance_to_boundary = sdf_->DistanceToBoundary3d(location);

                    std::cerr << std::setprecision(p) << "Starting dist to obstacle: " << PrettyPrint::PrettyPrint(starting_distance_to_obstacles, true, " ") << std::endl;
                    std::cerr << std::setprecision(p) << "Starting dist to boundary: " << PrettyPrint::PrettyPrint(starting_distance_to_boundary, true, " ") << std::endl;

                    std::cerr << std::setprecision(p) << "Final dist to obstacle:    " << PrettyPrint::PrettyPrint(distance_to_obstacles, true, " ") << std::endl;
                    std::cerr << std::setprecision(p) << "Final dist to boundary:    " << PrettyPrint::PrettyPrint(distance_to_boundary, true, " ") << std::endl;

                    std::cerr << std::setprecision(p) << "collision - location:      " << (post_collision_project - location).norm() << std::endl;
                    std::cerr << std::setprecision(p) << "boundary - collision:      " << (post_boundary_project - post_collision_project).norm() << std::endl;

                    const auto post_collision_project_test = sdf_->ProjectOutOfCollisionToMinimumDistance3d(location, min_distance_to_obstacle_);
                    const auto post_boundary_project_test = sdf_->ProjectIntoValidVolumeToMinimumDistance3d(post_collision_project_test, min_distance_to_obstacle_);
                    (void)post_boundary_project_test;
                }
            }
            assert(getBubbleSize(post_boundary_project) >= min_distance_to_obstacle_);
        }
        return post_boundary_project;
    }

    double QuinlanRubberBand::getBubbleSize(const Eigen::Vector3d& location) const
    {
        if (ENABLE_BAND_DEBUGGING_)
        {
            vis_->visualizePoint("get_bubble_size_test_location", location, Visualizer::Orange(), 1, 0.005);
            vis_->forcePublishNow();
        }

        return sdf_->EstimateDistance3d(location).first;
    }

    bool QuinlanRubberBand::sufficientOverlap(
            const double bubble_size_a,
            const double bubble_size_b,
            const double distance) const
    {
        return (bubble_size_a + bubble_size_b) >= (distance + min_overlap_distance_);
    }

    bool QuinlanRubberBand::bandIsValid() const
    {
        return bandIsValid(band_);
    }

    bool QuinlanRubberBand::bandIsValid(const EigenHelpers::VectorVector3d& test_band) const
    {
        if (ENABLE_BAND_DEBUGGING_)
        {
            if (test_band.size() < 2)
            {
                return false;
            }

            for (size_t node_idx = 0; node_idx < test_band.size() - 1; ++node_idx)
            {
                const auto& curr = test_band[node_idx];
                const auto& next = test_band[node_idx + 1];
                const double curr_bubble_size = getBubbleSize(curr);
                const double next_bubble_size = getBubbleSize(next);
                const double dist = (curr - next).norm();
                if (!sdf_->LocationInBounds3d(curr) ||
                    curr_bubble_size < min_distance_to_obstacle_ ||
                    !sufficientOverlap(curr_bubble_size, next_bubble_size, dist))
                {
                    std::cerr << "Problem between node " << node_idx << " and " << node_idx + 1 << std::endl
                              << "In bounds: " << sdf_->LocationInBounds3d(curr) << std::endl
                              << "Curr bubble size: " << curr_bubble_size << std::endl
                              << "Next bubble size: " << next_bubble_size << std::endl
                              << "Curr + next:      " << curr_bubble_size + next_bubble_size << std::endl
                              << "Dist + min:       " << dist + min_overlap_distance_ << std::endl;
                    return false;
                }
            }
            if (getBubbleSize(test_band.back()) < min_distance_to_obstacle_)
            {
                std::cerr << "Problem at last node: "
                          << "Bubble size:        " << getBubbleSize(test_band.back()) << std::endl;
                return false;
            }
        }
        return true;
    }

    bool QuinlanRubberBand::bandIsValidWithVisualization() const
    {
        return bandIsValidWithVisualization(band_);
    }

    bool QuinlanRubberBand::bandIsValidWithVisualization(const EigenHelpers::VectorVector3d& test_band) const
    {
        if (ENABLE_BAND_DEBUGGING_)
        {
            if (!bandIsValid(test_band))
            {
                visualizeWithBubbles(test_band, "quinlan_band_something_is_invalid", Visualizer::Black(), Visualizer::Cyan(), 1, true);
                printBandData(test_band);
                return bandIsValid(test_band);
            }
        }
        return true;
    }

    /**
      * Interpolates bewteen the end of point_buffer and target, but does not add target to the buffer
      */

    bool QuinlanRubberBand::interpolateBetweenPoints(
            EigenHelpers::VectorVector3d& point_buffer,
            const Eigen::Vector3d& target) const
    {
        if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
        {
            std::cout << PrettyPrint::PrettyPrint(point_buffer, true, "\n") << std::endl << "Start of interpolateBetweenPoints: " << std::flush;
            if (point_buffer.size() >= 2)
            {
                assert(bandIsValidWithVisualization(point_buffer));
            }
            if (point_buffer.size() > 1)
            {
                visualizeWithBubbles(point_buffer, "start_of_interpolateBetweenPoints", Visualizer::Blue(), Visualizer::Cyan(), 1, true);
            }
            vis_->visualizePoint("start_of_interpolateBetweenPoints_target_point", target, Visualizer::Orange(), 1, 0.01);
            vis_->forcePublishNow();
        }

        const double target_bubble_size = getBubbleSize(target);

        // TODO: verify that this cannot get stuck in an infinite loop
        int outer_iteration_counter = 0;

        // Check if the bubbles for 2 adjacent nodes overlap with some minimum distance to spare
        double curr_bubble_size = getBubbleSize(point_buffer.back());

        double distance_to_end = (target - point_buffer.back()).norm();
        while (!sufficientOverlap(curr_bubble_size, target_bubble_size, distance_to_end))
        {
            if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
            {
                std::cout << "Start of interpolateBetweenPoints outer loop" << std::endl;
            }

            const Eigen::Vector3d curr = point_buffer.back();

            // Find a position between point_buffer.back() and next_node with sufficient bubble overlap
            // TODO: verify that this cannot get stuck in an infinite loop
            int inner_iteration_counter = 0;
            double interpolation_ratio = 0.5;

            Eigen::Vector3d interpolated_point = EigenHelpers::Interpolate(curr, target, interpolation_ratio);
            Eigen::Vector3d test_point = projectToValidBubble(interpolated_point);
            double test_point_bubble_size = getBubbleSize(test_point);
            double distance_between_curr_and_test_point = (curr - test_point).norm();

            if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
            {
                assert(sdf_->LocationInBounds3d(curr));
                assert(sdf_->LocationInBounds3d(target));
                assert(sdf_->LocationInBounds3d(interpolated_point));
                assert(sdf_->LocationInBounds3d(test_point));
            }
            if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_ && outer_iteration_counter >= 20)
            {
                std::string tmp;
                std::cout << "\n\n\n\n\n\n\nCurr: ";
                getBubbleSize(curr);
                std::cin >> tmp;
                std::cout << "\nInterp: ";
                const double interpolated_point_bubble_size = getBubbleSize(interpolated_point);
                std::cin >> tmp;
                std::cout << "\nTest: ";
                getBubbleSize(test_point);
                std::cin >> tmp;
                std::cout << "\nTarget: ";
                getBubbleSize(target);
                std::cin >> tmp;
                std::cout << std::endl;

                std::cerr << std::setprecision(12)
                          << "Curr:   " << curr.transpose() << std::endl
                          << "Target: " << target.transpose() << std::endl
                          << "Interp: " << interpolated_point.transpose() << std::endl
                          << "Test:   " << test_point.transpose() << std::endl
                          << std::endl;
                vis_->visualizePoint(  "interpolate_outer_debugging_curr_point",     curr,                  Visualizer::Red(1.0f),     1, 0.005);
                vis_->visualizeSpheres("interpolate_outer_debugging_curr_sphere",   {curr},                 Visualizer::Red(0.2f),     2, curr_bubble_size);
                vis_->visualizePoint(  "interpolate_outer_debugging_interp_point",   interpolated_point,    Visualizer::Green(1.0f),   1, 0.005);
                vis_->visualizeSpheres("interpolate_outer_debugging_interp_sphere", {interpolated_point},   Visualizer::Green(0.2f),   2, interpolated_point_bubble_size);
                vis_->visualizePoint(  "interpolate_outer_debugging_test_point",     test_point,            Visualizer::Cyan(1.0f),    1, 0.005);
                vis_->visualizeSpheres("interpolate_outer_debugging_test_sphere",   {test_point},           Visualizer::Cyan(0.2f),    2, test_point_bubble_size);
                vis_->visualizePoint(  "interpolate_outer_debugging_target_point",   target,                Visualizer::Blue(1.0f),    1, 0.005);
                vis_->visualizeSpheres("interpolate_outer_debugging_target_sphere", {target},               Visualizer::Blue(0.2f),    2, target_bubble_size);

                // delay some, doesn't actually do anything as nothing is on the "null" namespace
                vis_->deleteObjects("null", 1, 10);
                std::cin >> tmp;
            }
            if (outer_iteration_counter == 20)
            {
//                ROS_WARN_STREAM_NAMED("rubber_band", "Rubber band interpolation outer loop counter at " << outer_iteration_counter << ", probably stuck in an infinite loop");
//                throw_arc_exception(std::runtime_error, "Interpolation stuck");
                return false;
            }

            while (!sufficientOverlap(curr_bubble_size, test_point_bubble_size, distance_between_curr_and_test_point))
            {
                if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
                {
                    std::cout << "Start of interpolateBetweenPoints inner loop" << std::endl;
                    if (inner_iteration_counter >= 5)
                    {
                        std::cout << "\n\n\n\n\n\n\nCurr:\n";
                        getBubbleSize(curr);
                        std::cout << "\nInterp:\n";
                        const double interpolated_point_bubble_size = getBubbleSize(interpolated_point);
                        std::cout << "\nTest:\n";
                        getBubbleSize(test_point);
                        std::cout << "\nTarget\n";
                        getBubbleSize(target);
                        std::cout << std::endl;

                        vis_->visualizePoint(  "interpolate_inner_debugging_curr_point",     curr,                 Visualizer::Red(1.0f),     1, 0.005);
                        vis_->visualizeSpheres("interpolate_inner_debugging_curr_sphere",   {curr},                Visualizer::Red(0.2f),     2, curr_bubble_size);
                        vis_->visualizePoint(  "interpolate_inner_debugging_interp_point",   interpolated_point,   Visualizer::Green(1.0f),   1, 0.005);
                        vis_->visualizeSpheres("interpolate_inner_debugging_interp_sphere", {interpolated_point},  Visualizer::Green(0.2f),   2, interpolated_point_bubble_size);
                        vis_->visualizePoint(  "interpolate_inner_debugging_test_point",     test_point,           Visualizer::Cyan(1.0f),    1, 0.005);
                        vis_->visualizeSpheres("interpolate_inner_debugging_test_sphere",   {test_point},          Visualizer::Cyan(0.2f),    2, test_point_bubble_size);
                        vis_->visualizePoint(  "interpolate_inner_debugging_target_point",   target,               Visualizer::Blue(1.0f),    1, 0.005);
                        vis_->visualizeSpheres("interpolate_inner_debugging_target_sphere", {target},              Visualizer::Blue(0.2f),    2, target_bubble_size);

                        std::cerr << std::setprecision(12)
                                  << "Curr:   " << curr.transpose() << std::endl
                                  << "Interp: " << interpolated_point.transpose() << std::endl
                                  << "Test:   " << test_point.transpose() << std::endl
                                  << "Target: " << target.transpose() << std::endl
                                  << "curr bubble size:   " << curr_bubble_size << std::endl
                                  << "interp bubble size: " << interpolated_point_bubble_size << std::endl
                                  << "test bubble size:   " << test_point_bubble_size << std::endl
                                  << "target bubble size: " << target_bubble_size << std::endl
                                  << "curr + test:        " << curr_bubble_size + test_point_bubble_size << std::endl
                                  << "dist + min:         " << distance_between_curr_and_test_point + min_overlap_distance_ << std::endl
                                  << std::endl;

                        // delay some, doesn't actually do anything as nothing is on the "null" namespace
                        vis_->deleteObjects("null", 1, 10);
                    }
                }
                if (inner_iteration_counter == 5)
                {
//                    ROS_WARN_STREAM_NAMED("rubber_band", "Rubber band interpolation inner loop counter at " << inner_iteration_counter << ", probably stuck in an infinite loop");
//                    throw_arc_exception(std::runtime_error, "Interpolation stuck");
                    return false;
                }

                interpolation_ratio *= 0.5;
                interpolated_point = EigenHelpers::Interpolate(curr, target, interpolation_ratio);
                test_point = projectToValidBubble(interpolated_point);
                test_point_bubble_size = getBubbleSize(test_point);
                distance_between_curr_and_test_point = (curr - test_point).norm();

                ++inner_iteration_counter;

                if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
                {
                    std::cout << "End of interpolateBetweenPoints outer loop" << std::endl;
                }
            }
            // The bubbles now overlap sufficiently, so accept this point and record the new values
            point_buffer.push_back(test_point);
            curr_bubble_size = test_point_bubble_size;
            distance_to_end = (target - test_point).norm();

            if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
            {
                assert(bandIsValidWithVisualization(point_buffer));
                std::cout << "End of interpolateBetweenPoints outer loop" << std::endl;
            }
            ++outer_iteration_counter;
        }

        if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
        {
            point_buffer.push_back(target);
            visualizeWithBubbles(point_buffer, "end_of_interpolate_between_points", Visualizer::Black(), Visualizer::Cyan(), 1, true);
            assert(bandIsValidWithVisualization(point_buffer));
            point_buffer.erase(point_buffer.end() - 1);
            std::cout << "End of interpolateBetweenPoints" << std::endl;
        }

        return true;
    }

    /**
     * @brief QuinlanRubberBand::interpolateBandPoints
     * Re-interpolates the entire band, not to be used for just 1 segment
     */
    bool QuinlanRubberBand::interpolateBandPoints()
    {
        if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
        {
            assert(band_.size() >= 2);
            for (size_t idx = 0; idx < band_.size(); ++idx)
            {
                if (getBubbleSize(band_[idx]) < min_distance_to_obstacle_)
                {
                    std::cerr << "idx: " << idx << " point: " << band_[idx].transpose() << " size: " << getBubbleSize(band_[idx]) << std::endl;
                }

                assert(getBubbleSize(band_[idx]) >= min_distance_to_obstacle_);
            }

            visualizeWithBubbles("start_of_interpolate_band_points", Visualizer::Blue(), Visualizer::Cyan(), 1, true);

            std::cout << PrettyPrint::PrettyPrint(band_, true, "\n") << std::endl << "Start of interpolateBandPoints: "  << std::flush;
        }

        EigenHelpers::VectorVector3d new_band(1, band_.front());
        for (size_t idx = 0; idx + 1 < band_.size(); ++idx)
        {
            if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
            {
                std::cout << "Start of interpolateBandPointsLoop idx " << idx << std::endl;
            }

            const auto& next_node = band_[idx + 1];
            if (!interpolateBetweenPoints(new_band, next_node))
            {
                return false;
            }
            new_band.push_back(next_node);
            if (ENABLE_BAND_DEBUGGING_ && ENABLE_INTERPOLATE_DEBUGGING_)
            {
                assert(bandIsValidWithVisualization(new_band));
                std::cout << "End of interpolateBandPointsLoop idx " << idx << std::endl;
            }
        }

        band_ = new_band;
        assert(bandIsValidWithVisualization());
        return true;
    }

    void QuinlanRubberBand::removeExtraBandPoints(const bool verbose)
    {
        assert(bandIsValidWithVisualization());

        if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
        {
            std::cout << "Start of removeExtraBandPoints\n";
            visualizeWithBubbles("quinlan_band_test", Visualizer::Black(), Visualizer::Cyan(), 1, true);
            printBandData(band_);
        }

        // The start can't be removed, so push that point on immediately
        EigenHelpers::VectorVector3d forward_pass;
        forward_pass.reserve(band_.size());
        forward_pass.push_back(band_.front());
        if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
        {
            vis_->visualizePoint(  "remove_extra_test_points_kept_points",   forward_pass.back(),  Visualizer::Cyan(1.0f), (int32_t)forward_pass.size(), 0.002);
            vis_->visualizeSpheres("remove_extra_test_points_kept_spheres", {forward_pass.back()}, Visualizer::Cyan(0.2f), (int32_t)forward_pass.size(), getBubbleSize(forward_pass.back()));
        }

        for (size_t curr_idx = 1; curr_idx + 1 < band_.size(); ++curr_idx)
        {
            if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
            {
                std::cout << "Start of removeExtraBandPoints loop, idx: " << curr_idx << std::endl;
            }

            const auto& prev = forward_pass.back();
            const auto& curr = band_[curr_idx];
            const auto& next = band_[curr_idx + 1];

            const double prev_bubble_size = getBubbleSize(prev);
            const double curr_bubble_size = getBubbleSize(curr);
            const double next_bubble_size = getBubbleSize(next);

            const double prev_curr_dist = (prev - curr).norm();
            const double curr_next_dist = (next - curr).norm();
            const double prev_next_dist = (next - prev).norm();

            const bool curr_bubble_is_wholey_contained_in_prev =
                    prev_bubble_size > prev_curr_dist + curr_bubble_size + min_overlap_distance_;

            const bool curr_bubble_is_wholey_contained_in_next =
                    next_bubble_size > curr_next_dist + curr_bubble_size + min_overlap_distance_;

            // Discard this point if it is containted in either neighbouring bubble
            if (curr_bubble_is_wholey_contained_in_prev || curr_bubble_is_wholey_contained_in_next)
            {
                if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
                {
                    std::cout << "Removing point as it is wholey contained, idx: " << curr_idx << std::endl;
                }
                continue;
            }

            // Discard this point if prev overlaps next by enough
            if (sufficientOverlap(prev_bubble_size, next_bubble_size, prev_next_dist * node_removal_overlap_factor_))
            {
                if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
                {
                    std::cout << "Removing point as it has enough overlap, idx: " << curr_idx << std::endl;
                }
                continue;
            }

            // Discard this point if it is too close to the previous, or too close to the next
            if (prev.isApprox(curr) || next.isApprox(curr))
            {
                if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
                {
                    std::cout << "Removing point as it is really really really close to and adjacent one, idx: " << curr_idx << std::endl;
                }
                continue;
            }

            // Only keep points if they do not backtrack
            const double angle_defined_by_points = EigenHelpers::AngleDefinedByPoints(prev, curr, next);
            assert(angle_defined_by_points >= 0.0);
            const bool band_backtracks = angle_defined_by_points < backtrack_threshold_;

            if (band_backtracks)
            {
                if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
                {
                    std::cout << "Removing point as it is a backtrack, idx: " << curr_idx << std::endl;
                }
                continue;
            }

            if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
            {
                vis_->visualizePoint(  "remove_extra_test_prev",  prev,  Visualizer::Red(1.0f),   1, 0.002);
                vis_->visualizeSpheres("remove_extra_test_prev", {prev}, Visualizer::Red(0.2f),   2, prev_bubble_size);
                vis_->visualizePoint(  "remove_extra_test_curr",  curr,  Visualizer::Green(1.0f), 1, 0.002);
                vis_->visualizeSpheres("remove_extra_test_curr", {curr}, Visualizer::Green(0.2f), 2, curr_bubble_size);
                vis_->visualizePoint(  "remove_extra_test_next",  next,  Visualizer::Blue(1.0f),  1, 0.002);
                vis_->visualizeSpheres("remove_extra_test_next", {next}, Visualizer::Blue(0.2f),  2, next_bubble_size);
                vis_->forcePublishNow();

                std::cout << "prev bubble size: " << prev_bubble_size << std::endl;
                std::cout << "curr bubble size: " << curr_bubble_size << std::endl;
                std::cout << "next bubble size: " << next_bubble_size << std::endl;
                std::cout << "prev-curr dist:   " << prev_curr_dist << std::endl;
                std::cout << "curr-next dist:   " << curr_next_dist << std::endl;
                std::cout << "prev-next dist:   " << prev_next_dist << std::endl;
            }
            // If no item said we should delete this item, then keep it
            forward_pass.push_back(curr);
            if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
            {
                vis_->visualizePoint(  "remove_extra_test_points_kept_points",   forward_pass.back(),  Visualizer::Cyan(1.0f), (int32_t)forward_pass.size(), 0.002);
                vis_->visualizeSpheres("remove_extra_test_points_kept_spheres", {forward_pass.back()}, Visualizer::Cyan(0.2f), (int32_t)forward_pass.size(), getBubbleSize(forward_pass.back()));
                std::cout << "End of removeExtraBandPoints loop"  << std::endl;
            }
        }
        forward_pass.push_back(band_.back());
        if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
        {
            vis_->visualizePoints( "remove_extra_test_points_kept_points",   forward_pass.back(), Visualizer::Cyan(1.0f), (int32_t)forward_pass.size(), 0.002);
            vis_->visualizeSpheres("remove_extra_test_points_kept_spheres", {forward_pass.back()}, Visualizer::Cyan(0.2f), (int32_t)forward_pass.size(), getBubbleSize(forward_pass.back()));
        }

        band_ = forward_pass;

        if (ENABLE_BAND_DEBUGGING_ && ENABLE_REMOVE_DEBUGGING_ && verbose)
        {
            std::cout << "End of removeExtraBandPoints\n";
            visualizeWithBubbles("quinlan_band_test", Visualizer::Black(), Visualizer::Cyan(), 1, true);
            printBandData(band_);
            assert(bandIsValidWithVisualization());
        }
    }

    bool QuinlanRubberBand::smoothBandPoints(const bool verbose)
    {
        if (ENABLE_BAND_DEBUGGING_ && ENABLE_SMOOTHING_DEBUGGING_ && verbose)
        {
            visualizeWithBubbles(band_, "StartOfSmoothingBand", Visualizer::Black(), Visualizer::Cyan(), 1, true);
            assert(bandIsValidWithVisualization());
            vis_->forcePublishNow();
        }

        for (size_t smoothing_iter = 0; smoothing_iter < smoothing_iterations_; ++smoothing_iter)
        {
            if (ENABLE_BAND_DEBUGGING_ && ENABLE_SMOOTHING_DEBUGGING_ && verbose)
            {
                visualizeWithBubbles("StartOfSmoothingOuterLoop", Visualizer::Black(), Visualizer::Cyan(), 1, true);
                vis_->forcePublishNow();
                std::cerr << "\n\nStart of outer loop loop smoothBandPoints, smoothing iter: " << smoothing_iter << "\n";
                printBandData(band_);
            }

            // The start doesn't move, so push that point on immediately
            EigenHelpers::VectorVector3d next_band;
            next_band.reserve(band_.size());
            next_band.push_back(band_.front());

            for (size_t curr_idx = 1; curr_idx + 1 < band_.size(); ++ curr_idx)
            {
                if (ENABLE_BAND_DEBUGGING_ && ENABLE_SMOOTHING_DEBUGGING_ && verbose)
                {
                    visualizeWithBubbles("StartOfSmoothingInnerLoop", Visualizer::Black(), Visualizer::Cyan(), 1, true);
                    std::cout << "Start of smoothBandPointsInnerLoop: band idx: " << curr_idx << std::endl;
                }

                const auto& prev = next_band.back();
                const auto& curr = band_[curr_idx];
                const auto& next = band_[curr_idx + 1];

                const double prev_bubble_size = getBubbleSize(prev);
                const double curr_bubble_size = getBubbleSize(curr);
                const double next_bubble_size = getBubbleSize(next);

                // Only allow movement that points directly between next and prev
                const Eigen::Vector3d rejected_movement_direction = (next - curr).normalized() - (prev - curr).normalized();
                // The optimal point is directly between prev and next, so move as far that way as our bubble allows
                const Eigen::Vector3d midpoint = prev + (next - prev) / 2.0;
                const Eigen::Vector3d delta_raw = midpoint - curr;
                const Eigen::Vector3d delta = EigenHelpers::VectorRejection(rejected_movement_direction, delta_raw);
                // Determine if the projection is within the bubble at the current point, and if not only move part way
                // Only step at most part way there in order to try and avoid oscillations - too large of a step size can be bad due to errors in sdf->EstiamteDistance
                const double max_delta_norm = curr_bubble_size  * MAX_DELTA_SCALE_FACTOR;
                const bool curr_plus_delta_inside_bubble = delta.norm() <= max_delta_norm;
                const Eigen::Vector3d prime =  curr_plus_delta_inside_bubble ? Eigen::Vector3d(curr + delta) : Eigen::Vector3d(curr + max_delta_norm * delta.normalized());
                // Ensure that the resulting point is not in collision even with numerical rounding (and weirdness in the SDF)
                const Eigen::Vector3d projected = projectToValidBubble(prime);
                const double projected_bubble_size = getBubbleSize(projected);

                if (ENABLE_BAND_DEBUGGING_ && ENABLE_SMOOTHING_DEBUGGING_ && verbose)
                {
                    if (projected_bubble_size < min_distance_to_obstacle_)
                    {
                        const Eigen::Vector3d projected_testing = projectToValidBubble(prime);
                        const double projected_bubble_size_test = getBubbleSize(projected_testing);
                        std::cout << std::setprecision(12)
                                  << "Projected: " << projected_testing.transpose() << std::endl
                                  << "Proj bubble size: " << projected_bubble_size_test << std::endl;
                    }

                    const double prime_bubble_size = getBubbleSize(prime);

                    vis_->forcePublishNow();
                    vis_->purgeMarkerList();
                    vis_->visualizePoint( "smoothing_test_prev",        prev,       Visualizer::Red(1.0f),     1, 0.002);
                    vis_->visualizeSpheres("smoothing_test_prev",      {prev},      Visualizer::Red(0.2f),     2, prev_bubble_size);
                    vis_->visualizePoint( "smoothing_test_curr",        curr,       Visualizer::Green(1.0f),   1, 0.002);
                    vis_->visualizeSpheres("smoothing_test_curr",      {curr},      Visualizer::Green(0.2f),   2, curr_bubble_size);
                    vis_->visualizePoint( "smoothing_test_prime",       prime,      Visualizer::Cyan(1.0f),    1, 0.002);
                    vis_->visualizeSpheres("smoothing_test_prime",     {prime},     Visualizer::Cyan(0.2f),    2, prime_bubble_size);
                    vis_->visualizePoint( "smoothing_test_projected",   projected,  Visualizer::Magenta(1.0f), 1, 0.002);
                    vis_->visualizeSpheres("smoothing_test_projected", {projected}, Visualizer::Magenta(0.2f), 2, projected_bubble_size);
                    vis_->visualizePoint( "smoothing_test_next",        next,       Visualizer::Blue(1.0f),    1, 0.002);
                    vis_->visualizeSpheres("smoothing_test_next",      {next},      Visualizer::Blue(0.2f),    2, next_bubble_size);
                    vis_->forcePublishNow();

                    std::cout << std::setprecision(12)
                              << "prev                      = [" << prev.transpose() << "]';\n"
                              << "next                      = [" << next.transpose() << "]';\n"
                              << std::endl
                              << "curr                      = [" << curr.transpose() << "]';\n"
                              << "midpoint                  = [" << midpoint.transpose() << "]';\n"
                              << "prime                     = [" << prime.transpose() << "]';\n"
                              << "projected                 = [" << projected.transpose() << "]';\n"
                              << std::endl
                              << "prev_minus_curr           = [" << (prev - curr).normalized().transpose() << "]';\n"
                              << "next_minus_curr           = [" << (next - curr).normalized().transpose() << "]';\n"
                              << "rejected_movement_dir     = [" << rejected_movement_direction.transpose() << "]';\n"
                              << std::endl
                              << "sdf_gradient =            = [" << EigenHelpers::StdVectorDoubleToEigenVector3d(sdf_->GetGradient3d(curr)).transpose() << "]';\n"
                              << "delta_raw                 = [" << delta_raw.transpose() << "]';\n"
                              << "delta_projected_to_plane  = [" << delta.transpose() << "]';\n"
                              << "delta_clipped_to_sphere   = [" << (prime - curr).transpose() << "]';\n"
                              << "rejected_dot_delta_norm   = " << delta.dot(rejected_movement_direction) << ";\n"
                              << std::endl
                              << "max_delta_norm                = " << max_delta_norm << ";\n"
                              << "delta_raw_norm                = " << delta_raw.norm() << ";\n"
                              << "delta_projected_to_plane_norm = " << delta.norm() << ";\n"
                              << "delta_clipped_to_sphere_norm  = " << (prime - curr).norm() << ";\n"
                              << std::endl
                              << "prev_bubble_size  = " << prev_bubble_size << ";\n"
                              << "curr_bubble_size  = " << curr_bubble_size << ";\n"
                              << "prime_bubble_size = " << prime_bubble_size << ";\n"
                              << "proj_bubble_size  = " << projected_bubble_size << ";\n"
                              << "next_bubble_size  = " << next_bubble_size << ";\n"
                              << std::endl;


                    if (!curr_plus_delta_inside_bubble)
                    {
                        std::cout << "!!!!!!!!!!!!!!!! bubble size is meaningfully impacting movement" << std::endl;
                        std::cout << "SDF Est at (curr):              " << sdf_->EstimateDistance3d(curr).first << std::endl;
                        std::cout << "(SDF Est at curr) + delta_norm: " << sdf_->EstimateDistance3d(curr).first + delta.norm() << std::endl;
                        std::cout << "(SDF Est at curr) - delta_norm: " << sdf_->EstimateDistance3d(curr).first - delta.norm() << std::endl;
                        std::cout << "SDF Est at (curr + delta):      " << sdf_->EstimateDistance3d(curr + delta).first << std::endl;

                        if (sdf_->EstimateDistance3d(curr + delta).first > sdf_->EstimateDistance3d(curr).first)
                        {
                            std::getchar();
                        }
                    }
                }

                // Check if the bubbles still overlap on each side
                const double prev_curr_dist = (prev - projected).norm();
                const double curr_next_dist = (next - projected).norm();
                const bool prev_bubble_overlaps_curr = sufficientOverlap(prev_bubble_size, projected_bubble_size, prev_curr_dist);
                const bool next_bubble_overlaps_curr = sufficientOverlap(next_bubble_size, projected_bubble_size, curr_next_dist);
                if (!prev_bubble_overlaps_curr)
                {
                    if (!interpolateBetweenPoints(next_band, projected))
                    {
                        return false;
                    }
                }
                next_band.push_back(projected);
                if (!next_bubble_overlaps_curr)
                {
                    if (!interpolateBetweenPoints(next_band, next))
                    {
                        return false;
                    }
                }

                if (ENABLE_BAND_DEBUGGING_ && ENABLE_SMOOTHING_DEBUGGING_ && verbose)
                {
                    std::cout << "End of smoothBandPointsInnerLoop: band idx: " << curr_idx << std::endl;
                }
            }

            // The end doesn't move, so push that point on at the end, then swap buffers
            next_band.push_back(band_.back());

            // Shortcut the process if there has been no meaningful change in the band
            if (EigenHelpers::CloseEnough(band_, next_band, SMOOTHING_CLOSE_ENGOUGH_DIST))
            {
                return true;
            }

            band_ = next_band;
            if (ENABLE_BAND_DEBUGGING_ && ENABLE_SMOOTHING_DEBUGGING_ && verbose)
            {
                std::cout << "\n\n\nEnd of smoothing loop, iteration: " << smoothing_iter << std::endl;
                visualizeWithBubbles("quinlan_band_test", Visualizer::Black(), Visualizer::Cyan(), 1, true);
                printBandData(band_);
                assert(bandIsValidWithVisualization());
                vis_->forcePublishNow();
            }

            removeExtraBandPoints(verbose);
        }

        if (ENABLE_BAND_DEBUGGING_ && ENABLE_SMOOTHING_DEBUGGING_ && verbose)
        {
            printBandData(band_);
            assert(bandIsValidWithVisualization());
        }

        return true;
    }


    void QuinlanRubberBand::printBandData(const EigenHelpers::VectorVector3d& test_band) const
    {
        if (ENABLE_BAND_DEBUGGING_)
        {
            const Eigen::Vector3d min = sdf_->GetOriginTransform().translation();
            const Eigen::Vector3d max = min + Eigen::Vector3d(sdf_->GetXSize(), sdf_->GetYSize(), sdf_->GetZSize());
            std::cout << "SDF limits: x, y, z\n"
                      << "Max:            " << max.transpose() << std::endl
                      << "Min:            " << min.transpose() << std::endl
                      << "Band limits: Min dist to obstacle: " << min_distance_to_obstacle_ << "   Min overlap distance: " << min_overlap_distance_ << std::endl;

            std::cout << "                         Point                    ,    bubble size   ,     overlap    ,   Angles:\n";

            Eigen::MatrixXd data = Eigen::MatrixXd::Zero(test_band.size(), 6) * NAN;

            for (size_t idx = 0; idx < test_band.size(); ++idx)
            {
                data.block<1, 3>(idx, 0) = test_band[idx].transpose();
                data(idx, 3) = getBubbleSize(test_band[idx]);
                if (idx > 0)
                {
                    const double prev_bubble_size = getBubbleSize(test_band[idx - 1]);
                    const double curr_bubble_size = getBubbleSize(test_band[idx]);
                    const double distance_between_prev_and_curr = (test_band[idx] - test_band[idx-1]).norm();
                    data(idx, 4) = (prev_bubble_size + curr_bubble_size) - distance_between_prev_and_curr;
                }
                if (idx > 0 && idx + 1 < test_band.size())
                {
                    data(idx, 5) = EigenHelpers::AngleDefinedByPoints(test_band[idx - 1], test_band[idx], test_band[idx + 1]);
                }
            }
            std::cout << std::setprecision(12) << data << std::endl;
            arc_helpers::Sleep(0.001);
        }
    }


    uint64_t QuinlanRubberBand::serialize(std::vector<uint8_t>& buffer) const
    {
        // Note that we only serialize band_ because everything else is const,
        // or is derived from const properties and band_
        const auto starting_bytes = buffer.size();
        const auto bytes_written =
                arc_utilities::SerializeVector(band_, buffer, arc_utilities::SerializeEigen<double, 3, 1>);
        // Verify no mistakes were made
        {
            auto tmp = *this;
            const auto bytes_read = tmp.deserialize(buffer, starting_bytes);
            assert(bytes_written == bytes_read);
            assert(*this == tmp);
        }
        return bytes_written;
    }

    uint64_t QuinlanRubberBand::deserialize(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        const auto deserialized_band_results =
                arc_utilities::DeserializeVector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>(
                    buffer, current, arc_utilities::DeserializeEigen<Eigen::Vector3d>);
        band_ = deserialized_band_results.first;
        const auto bytes_read = deserialized_band_results.second;

        // Reset internal versions of band_
        resampled_band_.clear();
        upsampled_band_.clear();
        resampleBand();
        upsampleBand();

        return bytes_read;
    }

    uint64_t QuinlanRubberBand::Serialize(const QuinlanRubberBand::ConstPtr& band, std::vector<uint8_t>& buffer)
    {
        return band->serialize(buffer);
    }

    std::pair<QuinlanRubberBand::Ptr, uint64_t> QuinlanRubberBand::Deserialize(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const QuinlanRubberBand& template_band)
    {
        auto band = std::make_shared<QuinlanRubberBand>(template_band);
        const auto bytes = band->deserialize(buffer, current);
        return {band, bytes};
    }

    void QuinlanRubberBand::storeBand() const
    {
        try
        {
            const auto log_folder = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "log_folder", __func__);
            const auto file_name_prefix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "file_name_prefix", __func__);

            const std::string file_name_suffix = arc_helpers::GetCurrentTimeAsString();
            const std::string file_name = file_name_prefix + "__" + file_name_suffix + ".compressed";
            const std::string full_path = log_folder + file_name;
            ROS_DEBUG_STREAM("Saving band to " << full_path);

            std::vector<uint8_t> buffer;
            serialize(buffer);
            arc_utilities::CreateDirectory(log_folder);
            ZlibHelpers::CompressAndWriteToFile(buffer, full_path);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("Failed to store band: "  <<  e.what());
        }
    }

    void QuinlanRubberBand::loadStoredBand()
    {
        try
        {
            const auto log_folder = ROSHelpers::GetParamRequired<std::string>(*nh_, "log_folder", __func__);
            const auto file_name_prefix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "file_name_prefix", __func__);
            const auto file_name_suffix = ROSHelpers::GetParamRequiredDebugLog<std::string>(*ph_, "file_name_suffix_to_load", __func__);

            const std::string file_name = file_name_prefix + "__" + file_name_suffix + ".compressed";
            const std::string full_path = log_folder + file_name;
            ROS_INFO_STREAM("Loading band from " << full_path);

            const auto buffer = ZlibHelpers::LoadFromFileAndDecompress(full_path);
            deserialize(buffer, 0);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("Failed to load stored band: "  <<  e.what());
        }

        visualizeWithBubbles("band_post_load", Visualizer::Blue(), Visualizer::Cyan(), 1, true);
    }

    bool QuinlanRubberBand::useStoredBand() const
    {
        return ROSHelpers::GetParamRequired<bool>(*ph_, "use_stored_band", __func__);
    }


    bool QuinlanRubberBand::operator==(const QuinlanRubberBand& other) const
    {
        if (sdf_.get() != other.sdf_.get())
        {
            return false;
        }
        if (work_space_grid_ != other.work_space_grid_)
        {
            return false;
        }
        if (path_between_grippers_through_object_ != other.path_between_grippers_through_object_)
        {
            return false;
        }
        if (band_ != other.band_)
        {
            return false;
        }
        if (max_safe_band_length_ != other.max_safe_band_length_)
        {
            return false;
        }
        if (min_overlap_distance_ != other.min_overlap_distance_)
        {
            return false;
        }
        if (min_distance_to_obstacle_ != other.min_distance_to_obstacle_)
        {
            return false;
        }
        if (node_removal_overlap_factor_ != other.node_removal_overlap_factor_)
        {
            return false;
        }
        if (backtrack_threshold_ != other.backtrack_threshold_)
        {
            return false;
        }
        if (smoothing_iterations_ != other.smoothing_iterations_)
        {
            return false;
        }
        return true;
    }

    bool QuinlanRubberBand::operator!=(const QuinlanRubberBand& other) const
    {
        return !(*this == other);
    }


    double QuinlanRubberBand::distanceSq(const EigenHelpers::VectorVector3d& other) const
    {
        const auto b1_points = upsampleBand();
        assert(other.size() == b1_points.size());
        double dist_sq = 0.0;
        for (size_t idx = 0; idx < b1_points.size(); ++idx)
        {
            dist_sq += (b1_points[idx] - other[idx]).squaredNorm();
        }
        return dist_sq;
    }

    double QuinlanRubberBand::distance(const EigenHelpers::VectorVector3d& other) const
    {
        return std::sqrt(distanceSq(other));
    }

    double QuinlanRubberBand::distanceSq(const QuinlanRubberBand& other) const
    {
        const auto b1_path_vec = upsampleBandSingleVector();
        const auto b2_path_vec = other.upsampleBandSingleVector();
        return (b1_path_vec - b2_path_vec).squaredNorm();
    }

    double QuinlanRubberBand::distance(const QuinlanRubberBand& other) const
    {
        const auto b1_path_vec = upsampleBandSingleVector();
        const auto b2_path_vec = other.upsampleBandSingleVector();
        return (b1_path_vec - b2_path_vec).norm();
    }

    double QuinlanRubberBand::DistanceSq(const QuinlanRubberBand& b1, const QuinlanRubberBand& b2)
    {
        return b1.distanceSq(b2);
    }

    double QuinlanRubberBand::Distance(const QuinlanRubberBand& b1, const QuinlanRubberBand& b2)
    {
        return b1.distance(b2);
    }
}
