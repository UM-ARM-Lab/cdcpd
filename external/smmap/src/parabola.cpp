#include "smmap/parabola.h"
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <deformable_manipulation_experiment_params/utility.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/project_inliers.h>

using namespace Eigen;

namespace smmap
{
//    using PointType = pcl::PointXYZ;
//    using CloudType = pcl::PointCloud<PointType>;

//    static CloudType::Ptr StdToPcl(const EigenHelpers::VectorVector3d& points)
//    {
//        assert(points.size() < std::numeric_limits<uint32_t>::max());
//        // create an unorganized (height = 1) cloud of the correct size
//        CloudType::Ptr cloud = boost::make_shared<CloudType>();
//        cloud->width = static_cast<uint32_t>(points.size());
//        cloud->height = 1;
//        cloud->points.reserve(points.size());
//        for (size_t idx = 0; idx < points.size(); ++idx)
//        {
//            const auto& point = points[idx];
//            cloud->points.push_back(pcl::PointXYZ(static_cast<float>(point.x()),
//                                                  static_cast<float>(point.y()),
//                                                  static_cast<float>(point.z())));
//        }
//        return cloud;
//    }

//    static pcl::ModelCoefficients::Ptr PlaneCoeffs(const CloudType::ConstPtr& cloud, const double inlier_threshold = 0.01)
//    {
//        pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
//        pcl::PointIndices inliers;
//        // Create the segmentation object
//        pcl::SACSegmentation<pcl::PointXYZ> seg;
//        // Optional
//        seg.setOptimizeCoefficients(true);
//        // Mandatory
//        seg.setModelType(pcl::SACMODEL_PLANE);
//        seg.setMethodType(pcl::SAC_RANSAC);
//        seg.setDistanceThreshold(inlier_threshold);
//        seg.setInputCloud(cloud);
//        seg.segment(inliers, *coefficients);

//        return coefficients;
//    }

//    static CloudType::Ptr ProjectToPlane(const CloudType::ConstPtr& cloud, const pcl::ModelCoefficients::ConstPtr& coeffs)
//    {
//        CloudType::Ptr filtered = boost::make_shared<CloudType>();
//        pcl::ProjectInliers<PointType> proj;
//        proj.setModelType(pcl::SACMODEL_PLANE); // Needed to know how to interpret coeffs
//        proj.setInputCloud(cloud);
//        proj.setModelCoefficients(coeffs);
//        proj.filter(*filtered);
//        return filtered;
//    }

    // Assumes that (x0, y0) is the origin, and thus c = 0. Returns (a, b)
    // such that y = a*x^2 + b*x + 0; such that the arc length from (0, 0) to
    // (x1, y1) = 'length', and the parabola is convex. Requires that x1 is positive.
    // https://stackoverflow.com/questions/48486254/determine-parabola-with-given-arc-length-between-two-known-points
    static std::pair<double, double> FindParabolaCoeffs(
            const double x1,
            const double y1,
            const double length,
            const bool verbose = false)
    {
        if (verbose || !std::isfinite(x1) || x1 <= 0.0)
        {
            std::cerr << "x1: " << x1 << "    y1: " << y1 << "    length: " << length << std::endl;
        }
        assert(std::isfinite(x1) && x1 > 0.0);

        // Precomputed as it is used multiple places
        const auto ratio = y1 / x1;

        // Simplified formula inside the integral for the arc length of a parabola
        const auto dIntegral = [] (const double t)
        {
            return std::sqrt(1.0 + t*t);
        };

        // The result of integrating dIntegral without applying bounds
        const auto Integral = [] (const double t)
        {
            const auto rt = std::sqrt(1.0 + t*t);
            return 0.5 * (t * rt + std::log(t + rt));
        };

        // The arclength of a parabola*based on the above
        const auto arc_len_fn = [&] (const double a)
        {
            const auto upper = ratio + a*x1;
            const auto lower = ratio - a*x1;
            return 0.5 * (Integral(upper) - Integral(lower)) / a;
        };

        const auto darc_len_fn = [&] (const double a)
        {
            const auto upper = ratio + a*x1;
            const auto lower = ratio - a*x1;
            return 0.5 * (a*x1 * (dIntegral(upper) + dIntegral(lower)) + Integral(lower) - Integral(upper)) / (a*a);
        };

        const auto N = 1000;
        const auto EPSILON = 1e-5;
        // Start with a guess that is guaranteed to be positive, and could be in vaguely the right place
        double guess = std::abs(ratio) + 1.0 / x1;
        if (verbose)
        {
            std::cerr << "0: " << guess << " : " << arc_len_fn(guess) << std::endl;
        }

        for (int n = 0; n < N; ++n)
        {
            const auto dguess = (arc_len_fn(guess) - length) / darc_len_fn(guess);
            // Ensure guess stays positive, allowed because the function is symmetric about the y-axis
            guess = std::abs(guess - dguess);
            if (verbose)
            {
                std::cerr << n+1 << ": " << guess << " : " << arc_len_fn(guess) << std::endl;
                if (!std::isfinite(guess) || guess <= 0.0)
                {
                    PressAnyKeyToContinue("Weirdness in FindCoeffs");
                    assert(false);
                }
            }
            if (!std::isfinite(guess) || guess <= 0.0)
            {
                std::cerr << "x1: " << x1 << "    y1: " << y1 << "    length: " << length << std::endl;
                return FindParabolaCoeffs(x1, y1, length, true);
            }
            if (std::abs(dguess) <= EPSILON)
            {
                break;
            }
        }

        const double a = guess;
        const double b = ratio - a*x1;

        if (!std::isfinite(a) || a < 0.0)
        {
            std::cerr << "x1: " << x1 << "    y1: " << y1 << "    length: " << length << std::endl;
            std::cerr << "guess: " << guess << std::endl;
            return FindParabolaCoeffs(x1, y1, length, true);
            assert(std::isfinite(a) && a >= 0.0);
        }

        return {a, b};
    }

    static sdf_tools::CollisionMapGrid ExtractSlice(
            const sdf_tools::SignedDistanceField& sdf,
            const Isometry3d& parabola_origin,
            const PairGripperPositions& gripper_positions,
            const double resolution,
            const double parabola_length,
            const Visualizer::Ptr vis)
    {
        // Precalculate these once for efficiency
        const double half_res = resolution / 2;
        const Isometry3d parabola_origin_inv = parabola_origin.inverse();
        const Vector3d second_point_parabola_frame = parabola_origin_inv * gripper_positions.second;

        // Special case the instance when the two grippers are (approximately)
        // in line in the local-y axis; anything that fits within 1 cell counts as
        // "approximately in line"
        if (second_point_parabola_frame.x() <= resolution)
        {
            const double delta_y = std::abs(second_point_parabola_frame.y());
            const double y_range = (parabola_length + delta_y) / 2;

            const int64_t x_cells = 3;
            const int64_t y_cells = (int64_t)std::ceil(y_range / resolution) + 2;
            const int64_t z_cells = 1;

            const Vector3d offset(second_point_parabola_frame.x() / 2 - 1.5 * resolution,
                                  std::max(0.0, second_point_parabola_frame.y()) - y_range - half_res,
                                  -half_res);

            sdf_tools::CollisionMapGrid grid(parabola_origin * Translation3d(offset),
                                             sdf.GetFrame(),
                                             resolution,
                                             x_cells,
                                             y_cells,
                                             z_cells,
                                             sdf_tools::COLLISION_CELL(1.0),
                                             sdf_tools::COLLISION_CELL(1.0));

            const int64_t x_idx = 1;
            const int64_t z_idx = 0;
            for (int64_t y_idx = 1; y_idx < y_cells - 1; ++y_idx)
            {
                const auto location_world_frame = grid.GridIndexToLocation(x_idx, y_idx, z_idx);
                const auto sdf_val = sdf.GetImmutable4d(location_world_frame);
                // If the sdf lookup is valid, then set the collision cell accordingly,
                // otherwise leave it at the default "filled" value
                if (sdf_val.second)
                {
                    const auto occupancy = sdf_val.first > 0.0 ? 0.0f : 1.0f;
                    grid.SetValue(x_idx, y_idx, z_idx, sdf_tools::COLLISION_CELL(occupancy));
                }
            }

            return grid;
        }

        const Vector2d line_normal_parabola_frame(-second_point_parabola_frame.y(), second_point_parabola_frame.x());
        // Find the coefficients for the parabola
        const auto coeffs = FindParabolaCoeffs(second_point_parabola_frame.x(), second_point_parabola_frame.y(), parabola_length);
        const double a = coeffs.first;
        const double b = coeffs.second;
        const auto parabola_eqn = [&] (const double x)
        {
            return a * x * x + b * x;
        };
        // Ensure that the parabola is convex:
        assert(std::isfinite(a) && a >= 0.0 && "Parabola must be convex");
        // If b is positive, then there is not enough slack to create a loop below the gripper
        const double x_min = 0.0;
        const double x_max = second_point_parabola_frame.x();
        const double x_lowest = arc_helpers::ClampValue(-b/(2*a), x_min, x_max);
        const double y_min = parabola_eqn(x_lowest);
        const double y_max = std::max(0.0, second_point_parabola_frame(1));
        const double x_range = x_max - x_min;
        const double y_range = y_max - y_min;
        assert(y_max >= y_min);
        const int64_t x_cells = (int64_t)std::ceil(x_range / resolution) + 2;
        const int64_t y_cells = (int64_t)std::ceil(y_range / resolution) + 2;
        const int64_t z_cells = 1;

        // Move the origin to center the parabola; will ensure a 1 cell boundary by construction
        const Vector3d grid_offset(x_min - 0.5 * (resolution * (double)x_cells - x_range),  // Center the valid region of the voxel grid between the grippers
                                   y_min - 0.5 * (resolution * (double)y_cells - y_range),  // Center the valid region of the voxel grid on the parabola
                                   -half_res);                                              // Shift half a cell to put the slice directly overtop of the grippers

        sdf_tools::CollisionMapGrid grid(parabola_origin * Translation3d(grid_offset),
                                         sdf.GetFrame(),
                                         resolution,
                                         x_cells,
                                         y_cells,
                                         z_cells,
                                         sdf_tools::COLLISION_CELL(1.0),
                                         sdf_tools::COLLISION_CELL(1.0));

        if (vis != nullptr)
        {
            const VectorXd x = VectorXd::LinSpaced(1000, 0.0, x_max);
            const VectorXd y = x.unaryExpr(parabola_eqn);
            const VectorXd z = VectorXd::Zero(x.rows());
            vis->visualizeLineStrip("parabola",
                                     parabola_origin * (ObjectPointSet(3, x.rows()) << x.transpose(),
                                                                                       y.transpose(),
                                                                                       z.transpose()).finished(),
                                     Visualizer::Magenta(), 1, 0.002);
            vis->visualizeLineStrip("parabola",
                                     (ObjectPointSet(3, 2) << gripper_positions.first, gripper_positions.second).finished(),
                                     Visualizer::Magenta(), 2, 0.002);
            vis->visualizeAxes("parabola_origin", parabola_origin, 0.05, 0.005, 1);
        }

        // Iterate through the grid, only considering cells inside the parabola
        for (int64_t x_idx = 1; x_idx < x_cells - 1; ++x_idx)
        {
            for (int64_t y_idx = 1; y_idx < y_cells - 1; ++y_idx)
            {
                const int64_t z_idx = 0;
                const auto location_grid_frame = grid.GridIndexToLocationGridFrame(x_idx, y_idx, z_idx);
                const auto location_world_frame = grid.GetOriginTransform() * location_grid_frame;

                const Vector2d point_parabola_frame = (parabola_origin_inv * location_world_frame).head<2>();
                const bool above_parabola = point_parabola_frame.y() >= parabola_eqn(point_parabola_frame.x());
                const bool below_line = line_normal_parabola_frame.dot(point_parabola_frame) <= 0.0;

                // If the location is inside the parabola, then check if it is filled,
                if (above_parabola && below_line)
                {
                    const auto sdf_val = sdf.GetImmutable4d(location_world_frame);
                    // If the sdf lookup is valid, then set the collision cell accordingly,
                    // otherwise leave it at the default "filled" value
                    if (sdf_val.second)
                    {
                        const auto occupancy = sdf_val.first > 0.0 ? 0.0f : 1.0f;
                        grid.SetValue(x_idx, y_idx, z_idx, sdf_tools::COLLISION_CELL(occupancy));
                    }
                }
                // Check if we're at an edge case - i.e. the line passes through the voxel,
                // but the voxel center is on the wrong side of something
                else
                {
                    const Vector2d top_right    = point_parabola_frame + Vector2d( half_res,  half_res);
                    const Vector2d bottom_right = point_parabola_frame + Vector2d( half_res, -half_res);
                    const Vector2d bottom_left  = point_parabola_frame + Vector2d(-half_res, -half_res);
                    const Vector2d top_left     = point_parabola_frame + Vector2d(-half_res,  half_res);

                    // Check if the line itself passes through the voxel
                    const bool line_bisects_voxel = [&]
                    {
                        const double top_right_dot_product      = line_normal_parabola_frame.dot(top_right);
                        const double bottom_right_dot_product   = line_normal_parabola_frame.dot(bottom_right);
                        const double bottom_left_dot_product    = line_normal_parabola_frame.dot(bottom_left);
                        const double top_left_dot_product       = line_normal_parabola_frame.dot(top_left);

                        const bool top_right_below_line     = top_right_dot_product       < 0.0;
                        const bool bottom_right_below_line  = bottom_right_dot_product    < 0.0;
                        const bool bottom_left_below_line   = bottom_left_dot_product     < 0.0;
                        const bool top_left_below_line      = top_left_dot_product        < 0.0;

                        // If the line bisects the voxel, then it passes through
                        return (top_right_below_line ^ bottom_left_below_line) ||
                               (top_left_below_line ^ bottom_right_below_line);
                    }();
                    // Check if the parabola passes through the voxel
                    const bool parabola_bisects_voxel = [&]
                    {
                        const double top_right_parabola_val     = parabola_eqn(top_right.x());
                        const double bottom_right_parabola_val  = parabola_eqn(bottom_right.x());
                        const double bottom_left_parabola_val   = parabola_eqn(bottom_left.x());
                        const double top_left_parabola_val      = parabola_eqn(top_left.x());

                        const bool top_right_above_parabola     = top_right.y()     > top_right_parabola_val;
                        const bool bottom_right_above_parabola  = bottom_right.y()  > bottom_right_parabola_val;
                        const bool bottom_left_above_parabola   = bottom_left.y()   > bottom_left_parabola_val;
                        const bool top_left_above_parabola      = top_left.y()      > top_left_parabola_val;

                        // If the parabola*bisects the voxel, then it passes through
                        return (top_right_above_parabola && !bottom_left_above_parabola) ||
                               (top_left_above_parabola && !bottom_right_above_parabola);
                    }();

                    if ((line_bisects_voxel && above_parabola) ||
                        (parabola_bisects_voxel && below_line) ||
                        (line_bisects_voxel && parabola_bisects_voxel))
                    {
                        const auto sdf_val = sdf.GetImmutable4d(location_world_frame);
                        // If the sdf lookup is valid, then set the collision cell accordingly,
                        // otherwise leave it at the default "filled" value
                        if (sdf_val.second)
                        {
                            const auto occupancy = sdf_val.first > 0.0 ? 0.0f : 1.0f;
                            grid.SetValue(x_idx, y_idx, z_idx, sdf_tools::COLLISION_CELL(occupancy));
                        }
                    }
                }
            }
        }

        return grid;
    }

    // Returns a slice defined by the position of the grippers, and the gravity vector
    sdf_tools::CollisionMapGrid ExtractParabolaSliceBasic(
            const sdf_tools::SignedDistanceField& sdf,
            const double resolution,
            const PairGripperPositions& gripper_positions,
            const double parabola_length,
            const Visualizer::Ptr vis)
    {
        const Vector3d gripper_delta = gripper_positions.second - gripper_positions.first;

        // For the purposes of calculating the parabola coefficients, rotate the
        // frame so that gripper_positions.first is at the origin, with the
        // gravity vector pointing along positive-y, and the second gripper is
        // at a +'ve x position
        const Isometry3d parabola_origin = [&]
        {
            // Special case the instance when the grippers are approximately in line
            // with the gravity vector; anything that fits within 1 cell counts as
            // "approximately in line"
            if (gripper_delta.head<2>().norm() <= resolution)
            {
                const Vector3d x_axis = Vector3d::UnitX();
                const Vector3d y_axis = Vector3d::UnitZ();
                const Vector3d z_axis = -Vector3d::UnitY();
                return Isometry3d((Matrix4d() << x_axis, y_axis, z_axis, gripper_positions.first,
                                                 0.0,    0.0,    0.0,    1.0).finished());
            }

            const Vector3d x_axis = Vector3d(gripper_delta.x(), gripper_delta.y(), 0.0).normalized();
            const Vector3d y_axis = Vector3d::UnitZ();
            const Vector3d z_axis = x_axis.cross(y_axis).normalized();

            return Isometry3d((Matrix4d() << x_axis, y_axis, z_axis, gripper_positions.first,
                                             0.0,    0.0,    0.0,    1.0).finished());
        }();

        return ExtractSlice(sdf, parabola_origin, gripper_positions, resolution, parabola_length, vis);
    }

    sdf_tools::CollisionMapGrid ExtractParabolaSliceInPlane(
            const sdf_tools::SignedDistanceField& sdf,
            const double resolution,
            const PairGripperPositions& gripper_positions,
            const Eigen::Vector3d& band_midpoint,
            const double parabola_length,
            const bool gravity_aligned,
            const Visualizer::Ptr vis)
    {
        // Special case the instance when the grippers are approximately in line
        // with the band midpoint; anything that fits within half a cell counts as
        // "approximately in line"
        const Vector3d gripper_delta = gripper_positions.second - gripper_positions.first;
        auto distances = EigenHelpers::DistanceToLine(gripper_positions.first, gripper_delta.normalized(), band_midpoint);
        const auto distance_to_line = distances.first;
//        const auto displacement_along_line = distances.second;
        if (distance_to_line < resolution)
        {
            return ExtractParabolaSliceBasic(sdf, resolution, gripper_positions, parabola_length, vis);
        }

        // Otherwise, create a transform that rotates the frame so that
        // gripper_a is at the origin, gripper_b is +'ve x +'ve y
        // and band_midpoint is on the plane somewhere
        const Isometry3d parabola_origin = [&]
        {
            const Eigen::Vector3d& plane_vector1 = gripper_delta;
            const Eigen::Vector3d plane_vector2 = gripper_positions.first - band_midpoint;
            const Vector3d z_axis = (plane_vector1.cross(plane_vector2)).normalized();

            Isometry3d origin;

            // Option 1: project the z-axis onto the plane to define the y-axis
            if (gravity_aligned)
            {
                const Vector3d y_axis_temp = EigenHelpers::VectorProjectionToPlane(plane_vector1, plane_vector2, Eigen::Vector3d::UnitZ());
                assert(y_axis_temp.norm() > 1e-3);
                const Vector3d y_axis = y_axis_temp.normalized();
                const Vector3d x_axis = (y_axis.cross(z_axis).normalized());

                origin.matrix() << x_axis, y_axis, z_axis, gripper_positions.first,
                                   0.0,    0.0,    0.0,    1.0;
            }
            // Option 2: use the line between the grippers to define the x-axis
            else
            {
                const Vector3d x_axis = gripper_delta.normalized();
                const Vector3d y_axis = z_axis.cross(x_axis).normalized();

                origin.matrix() << x_axis, y_axis, z_axis, gripper_positions.first,
                                   0.0,    0.0,    0.0,    1.0;
            }
            // If the second point is not in positive x (required for FindCoeffs)
            // then rotate 180 degrees around local y-axis
            const Vector3d second_point_parabola_frame = origin.inverse() * gripper_positions.second;
            if (second_point_parabola_frame.x() < 0)
            {
                origin.matrix().col(0) *= -1;
                origin.matrix().col(2) *= -1;
            }
            return origin;
        }();

        return ExtractSlice(sdf, parabola_origin, gripper_positions, resolution, parabola_length, vis);
    }

    sdf_tools::CollisionMapGrid ExtractParabolaSliceExtendDownwards(
            const sdf_tools::SignedDistanceField& sdf,
            const double resolution,
            const PairGripperPositions& gripper_positions,
            const Eigen::Vector3d& band_midpoint,
            const double parabola_length,
            const bool gravity_aligned,
            const Visualizer::Ptr vis)
    {
        // Find the lowest point below the midpoint that is within the band max length
        const Vector3d delta1 = gripper_positions.first - band_midpoint;
        const Vector3d delta2 = gripper_positions.second - band_midpoint;
        const double alpha = delta1.head<2>().squaredNorm();
        const double beta = delta2.head<2>().squaredNorm();
        const double a = gripper_positions.first.z();
        const double b = gripper_positions.second.z();
        const double l = parabola_length;
        const double l2 = l*l;
        const double a2 = a*a;
        const double b2 = b*b;
        const double l4 = l2*l2;

        // Per matlab/wolfram
        const double numer = a2*a - a2*b + alpha*a - a*b2 - a*beta - a*l2 + b2*b - alpha*b + b*beta - b*l2;
        const double denom = 2*((a-b)*(a-b) - l2);
        const double offset = a2*a2*l2 - 4*a2*a*b*l2 + 6*a2*b2*l2 - 2*a2*l4 + 2*a2*alpha*l2 + 2*a2*beta*l2 - 4*a*b2*b*l2 + 4*a*b*l4 - 4*a*alpha*b*l2 - 4*a*b*beta*l2 + b2*b2*l2 - 2*b2*l4 + 2*alpha*b2*l2 + 2*b2*beta*l2 + l2*l4 - 2*alpha*l4 - 2*beta*l4 + alpha*alpha*l2 - 2*alpha*beta*l2 + beta*beta*l2;
        const double z = (numer + std::sqrt(offset)) / denom;

        return ExtractParabolaSliceInPlane(sdf, resolution, gripper_positions, Vector3d(band_midpoint.x(), band_midpoint.y(), z), parabola_length, gravity_aligned, vis);
    }

    sdf_tools::CollisionMapGrid ExtractParabolaSlice(
            const sdf_tools::SignedDistanceField& sdf,
            const double resolution,
            const PairGripperPositions& gripper_positions,
            const RubberBand::ConstPtr& band,
            const std::string& parabola_slice_option,
            const Visualizer::Ptr vis)
    {
        if (parabola_slice_option == "basic")
        {
            return ExtractParabolaSliceBasic(
                        sdf,
                        resolution,
                        gripper_positions,
                        band->maxSafeLength(),
                        vis);
        }
        if (parabola_slice_option == "in_plane_gravity_aligned")
        {
            const auto& band_points = band->upsampleBand();
            const auto midpoint_idx = band_points.size() / 2;
            return ExtractParabolaSliceInPlane(
                        sdf,
                        resolution,
                        gripper_positions,
                        band_points[midpoint_idx],
                        band->maxSafeLength(),
                        true,
                        vis);
        }
        if (parabola_slice_option == "in_plane_gripper_aligned")
        {
            const auto& band_points = band->upsampleBand();
            const auto midpoint_idx = band_points.size() / 2;
            return ExtractParabolaSliceInPlane(
                        sdf,
                        resolution,
                        gripper_positions,
                        band_points[midpoint_idx],
                        band->maxSafeLength(),
                        false,
                        vis);
        }
        if (parabola_slice_option == "extend_downwards_gravity_aligned")
        {
            const auto& band_points = band->upsampleBand();
            const auto midpoint_idx = band_points.size() / 2;
            return ExtractParabolaSliceExtendDownwards(
                        sdf,
                        resolution,
                        gripper_positions,
                        band_points[midpoint_idx],
                        band->maxSafeLength(),
                        true,
                        vis);
        }
        if (parabola_slice_option == "extend_downwards_gripper_aligned")
        {
            const auto& band_points = band->upsampleBand();
            const auto midpoint_idx = band_points.size() / 2;
            return ExtractParabolaSliceExtendDownwards(
                        sdf,
                        resolution,
                        gripper_positions,
                        band_points[midpoint_idx],
                        band->maxSafeLength(),
                        false,
                        vis);
        }
        assert(false);
    }
}
