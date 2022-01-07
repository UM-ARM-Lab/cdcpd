#include "deformable_manipulation_experiment_params/ros_params.hpp"

#include <cmath>
#include <chrono>
#include <unordered_map>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/arc_exceptions.hpp>

namespace smmap
{
    float GetClothXSize(ros::NodeHandle &nh);

    ////////////////////////////////////////////////////////////////////////////
    // Visualization Settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetDisableSmmapVisualizations(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<bool>(nh, "disable_smmap_visualizations", __func__);
    }

    bool GetVisualizeObjectDesiredMotion(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_object_desired_motion", true);
    }

    bool GetVisualizeGripperMotion(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_gripper_motion", true);
    }

    bool GetVisualizeObjectPredictedMotion(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_object_predicted_motion", true);
    }

    bool GetVisualizeRRT(ros::NodeHandle& nh, const bool default_vis)
    {
        return ROSHelpers::GetParam(nh, "visualize_rrt", default_vis);
    }

    bool GetVisualizeFreeSpaceGraph(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_free_space_graph", true);
    }

    bool GetVisualizeCorrespondences(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_correspondences", true);
    }

    bool VisualizeStrainLines(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_strain_lines", false);
    }

    int GetViewerWidth(ros::NodeHandle& nh)      // Pixels
    {
        const auto val = ROSHelpers::GetParamDebugLog<int>(nh, "viewer_width", 800);
        assert(val > 0);
        return val;
    }

    int GetViewerHeight(ros::NodeHandle& nh)     // Pixels
    {
        const auto val = ROSHelpers::GetParamDebugLog<int>(nh, "viewer_height", 800);
        assert(val > 0);
        return val;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Task and Deformable Type parameters
    ////////////////////////////////////////////////////////////////////////////

    std::string GetTestId(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequiredDebugLog<std::string>(nh, "test_id", __func__);
    }

    DeformableType GetDeformableType(ros::NodeHandle& nh)
    {
        const static std::unordered_map<std::string, DeformableType> deformable_type_map
        {
            {"rope",    DeformableType::ROPE},
            {"cloth",   DeformableType::CLOTH},
        };

        const auto deformable_type = ROSHelpers::GetParamRequiredDebugLog<std::string>(nh, "deformable_type", __func__);
        return deformable_type_map.at(deformable_type);
    }

    std::string GetTaskTypeString(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequiredDebugLog<std::string>(nh, "task_type", __func__);
    }

    /**
     *  Maps the ros param "task_type" into an enum TaskType
     */
    TaskType GetTaskType(ros::NodeHandle& nh)
    {
        const static std::unordered_map<std::string, TaskType> task_map
        {
            {"rope_cylinder_coverage",                  TaskType::ROPE_CYLINDER_COVERAGE},
            {"rope_cylinder_coverage_two_grippers",     TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS},
            {"cloth_cylinder_coverage",                 TaskType::CLOTH_CYLINDER_COVERAGE},
            {"cloth_table_coverage",                    TaskType::CLOTH_TABLE_COVERAGE},
            {"cloth_colab_folding",                     TaskType::CLOTH_COLAB_FOLDING},
            {"cloth_wafr",                              TaskType::CLOTH_WAFR},
            {"cloth_single_pole",                       TaskType::CLOTH_SINGLE_POLE},
            {"cloth_wall",                              TaskType::CLOTH_WALL},
            {"cloth_double_slit",                       TaskType::CLOTH_DOUBLE_SLIT},
            {"rope_maze",                               TaskType::ROPE_MAZE},
            {"rope_zig_match",                          TaskType::ROPE_ZIG_MATCH},
            {"rope_table_linear_motion",                TaskType::ROPE_TABLE_LINEAR_MOTION},
            {"cloth_table_linear_motion",               TaskType::CLOTH_TABLE_LINEAR_MOTION},
            {"rope_table_penetration",                  TaskType::ROPE_TABLE_PENTRATION},
            {"live_cloth_placemat",                     TaskType::CLOTH_PLACEMAT},
            {"cloth_placemat_live_robot_linear_motion", TaskType::CLOTH_PLACEMAT_LINEAR_MOTION},
            {"live_cloth_mflag",                        TaskType::CLOTH_MFLAG},
            {"live_rope_simple",                        TaskType::ROPE_SIMPLE_COVERAGE_TWO_GRIPPERS},
            {"live_rope_engine_assembly",               TaskType::ROPE_ENGINE_ASSEMBLY_LIVE},
            {"rope_hooks",                              TaskType::ROPE_HOOKS},
            {"rope_hooks_simple",                       TaskType::ROPE_HOOKS_SIMPLE},
            {"rope_hooks_simple_long_rope",             TaskType::ROPE_HOOKS_SIMPLE},
            {"rope_hooks_simple_super_long_rope",       TaskType::ROPE_HOOKS_SIMPLE},
            {"rope_hooks_simple_short_rope",            TaskType::ROPE_HOOKS_SIMPLE},
            {"rope_hooks_multi",                        TaskType::ROPE_HOOKS_MULTI},
            {"cloth_hooks_simple",                      TaskType::CLOTH_HOOKS_SIMPLE},
            {"cloth_hooks_complex",                     TaskType::CLOTH_HOOKS_COMPLEX},
            {"engine_assembly",                         TaskType::ROPE_ENGINE_ASSEMBLY},
            {"rope_generic_dijkstras_coverage",         TaskType::ROPE_GENERIC_DIJKSTRAS_COVERAGE},
            {"cloth_generic_dijkstras_coverage",        TaskType::CLOTH_GENERIC_DIJKSTRAS_COVERAGE},
            {"rope_generic_fixed_coverage",             TaskType::ROPE_GENERIC_FIXED_COVERAGE},
            {"cloth_generic_fixed_coverage",            TaskType::CLOTH_GENERIC_FIXED_COVERAGE},
        };

        const auto task_type = GetTaskTypeString(nh);
        return task_map.at(task_type);
    }

    double GetMaxTime(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "task/max_time", __func__);
    }

    double GetMaxStretchFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "task/max_stretch_factor", __func__);
    }

    double GetMaxBandLength(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "task/max_band_length", __func__);
    }

    float GetMaxStrain(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<float>(nh, "max_strain", __func__);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Error calculation settings
    ////////////////////////////////////////////////////////////////////////////

    double GetErrorThresholdAlongNormal(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "task/error_threshold_along_normal", __func__);
    }

    double GetErrorThresholdDistanceToNormal(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "task/error_threshold_distance_to_normal", __func__);
    }

    double GetErrorThresholdTaskDone(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "task/error_threshold_task_done", __func__);
    }

    double GetDesiredMotionScalingFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "task/desired_motion_scale_factor", 1.0);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Gripper Size Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetGripperApperture(ros::NodeHandle& nh)   // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                // TODO: why did Dmitry's code use 0.5f here?
                return ROSHelpers::GetParamDebugLog(nh, "rope_gripper_apperture", 0.03f);

            case DeformableType::CLOTH:
                // TODO: This number is actually the "closed gap"
                //       The original number was 0.1f
                return ROSHelpers::GetParamDebugLog(nh, "cloth_gripper_apperture", 0.006f);

            default:
                ROS_FATAL_STREAM_NAMED("params", "Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
                assert(false && "This code should be unreachable");
        }
        // NO USE HERE, just for compilation
        return ROSHelpers::GetParamDebugLog(nh, "rope_gripper_apperture", 0.03f);
    }

    // TODO: where is this still used? Is it being used correctly vs ControllerMinDistToObstacles?
    double GetRobotGripperRadius()                   // METERS
    {
        return 0.023;
    }

    // Used by the "older" avoidance code, I.e. LeastSquaresControllerWithObjectAvoidance
    double GetRobotMinGripperDistanceToObstacles()   // METERS
    {
        return 0.005;
    }

    double GetControllerMinDistanceToObstacles(ros::NodeHandle& nh) // METERS
    {
        return ROSHelpers::GetParam(nh, "controller_min_distance_to_obstacles", 0.07);
    }

    double GetRRTMinGripperDistanceToObstacles(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/min_gripper_distance_to_obstacles", __func__);
        assert(val >= 0.0);
        return val;
    }

    double GetRRTTargetMinDistanceScaleFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/target_min_distance_scale_factor", __func__);
        assert(val >= 1.0);
        return val;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Table Size Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetTableSurfaceX(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_x", 0.0f);
    }

    float GetTableSurfaceY(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_y", 0.0f);
    }

    float GetTableSurfaceZ(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_z", 0.7f);
    }

    float GetTableHalfExtentsX(ros::NodeHandle& nh)  // METERS
    {
        return ROSHelpers::GetParamRequired<float>(nh, "table_x_half_extents", __func__);
    }

    float GetTableHalfExtentsY(ros::NodeHandle& nh)  // METERS
    {
        return ROSHelpers::GetParamRequired<float>(nh, "table_y_half_extents", __func__);
    }

    float GetTableHeight(ros::NodeHandle& nh)        // METERS
    {
        return ROSHelpers::GetParam(nh, "table_height", GetTableSurfaceZ(nh));
    }

    float GetTableLegWidth(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParam(nh, "table_leg_width", 0.05f);
    }

    float GetTableThickness(ros::NodeHandle& nh)     // METERS
    {
        return ROSHelpers::GetParam(nh, "table_thickness", 0.05f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cylinder Size Settings
    // TODO: Update launch files to contain these defaults
    ////////////////////////////////////////////////////////////////////////////

    float GetCylinderRadius(ros::NodeHandle& nh)           // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "cylinder_radius", 0.15f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cylinder_radius", 0.10f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cylinder_radius", 0.04f);

            default:
                return ROSHelpers::GetParamRequired<float>(nh, "cylinder_radius", __func__);
        }
    }

    float GetCylinderHeight(ros::NodeHandle& nh)           // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "cylinder_height", 0.3f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cylinder_height", 0.3f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cylinder_height", 1.0f);

            default:
                return ROSHelpers::GetParamRequired<float>(nh, "cylinder_height", __func__);
        }
    }

    float GetCylinderCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "cylinder_com_x", GetTableSurfaceX(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cylinder_com_x", GetTableSurfaceX(nh) - GetClothXSize(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cylinder_com_x", -0.3f);

            default:
                return ROSHelpers::GetParamRequired<float>(nh, "cylinder_com_x", __func__);
        }
    }

    float GetCylinderCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "cylinder_com_y", GetTableSurfaceY(nh) + GetCylinderRadius(nh) * 5.0f / 3.0f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cylinder_com_y", GetTableSurfaceY(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cylinder_com_y", 0.0f);

            default:
                return ROSHelpers::GetParamRequired<float>(nh, "cylinder_com_y", __func__);
        }
    }

    float GetCylinderCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "cylinder_com_z", GetTableSurfaceZ(nh) + GetCylinderHeight(nh) / 2.0f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cylinder_com_z", GetTableSurfaceZ(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cylinder_com_z", 1.0f);

            default:
                return ROSHelpers::GetParamRequired<float>(nh, "cylinder_com_z", __func__);
        }
    }

    // Cylinder Size settings for WAFR task case

    float GetWafrCylinderRadius(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_second_cylinder_radius", 0.025f);
    }

    float GetWafrCylinderHeight(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_second_cylinder_height", 0.51f);
    }

    float GetWafrCylinderRelativeCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_cylinder_relative_com_x", - 0.15f);
    }

    float GetWafrCylinderRelativeCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_cylinder_relative_com_y", 0.0f);
    }

    float GetWafrCylinderRelativeCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_cylinder_relative_com_z", 0.2f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope Maze Wall Size and Visibility Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetWallHeight(ros::NodeHandle& nh)             // METERS
    {
        return ROSHelpers::GetParamRequired<float>(nh, "wall_height", __func__);
    }

    float GetWallCenterOfMassZ(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParamRequired<float>(nh, "wall_com_z", __func__);
    }

    float GetOuterWallsAlpha(ros::NodeHandle& nh)        // 0.0 thru 1.0 (inclusive)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "outer_walls_alpha", __func__);
        assert(val >= 0.0 && val <= 1.0);
        return val;
    }

    float GetFloorDividerAlpha(ros::NodeHandle& nh)      // 0.0 thru 1.0 (inclusive)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "floor_divider_alpha", __func__);
        assert(val >= 0.0 && val <= 1.0);
        return val;
    }

    float GetFirstFloorAlpha(ros::NodeHandle& nh)        // 0.0 thru 1.0 (inclusive)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "first_floor_alpha", __func__);
        assert(val >= 0.0 && val <= 1.0);
        return val;
    }

    float GetSecondFloorAlpha(ros::NodeHandle& nh)       // 0.0 thru 1.0 (inclusive)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "second_floor_alpha", __func__);
        assert(val >= 0.0 && val <= 1.0);
        return val;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetRopeSegmentLength(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_segment_length", 0.025f);
    }

    float GetRopeRadius(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_radius", 0.01f);
    }

    int GetRopeNumLinks(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "rope_num_links", 49);
    }

    float GetRopeExtensionVectorX(ros::NodeHandle& nh)
    {
        return (float)ROSHelpers::GetParam(nh, "rope_extension_x", 1.0);
    }

    float GetRopeExtensionVectorY(ros::NodeHandle& nh)
    {
        return (float)ROSHelpers::GetParam(nh, "rope_extension_y", 0.0);
    }

    float GetRopeExtensionVectorZ(ros::NodeHandle& nh)
    {
        return (float)ROSHelpers::GetParam(nh, "rope_extension_z", 0.0);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope starting position settings
    ////////////////////////////////////////////////////////////////////////////

    float GetRopeCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParamRequired<float>(nh, "rope_com_x", __func__);
    }

    float GetRopeCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParamRequired<float>(nh, "rope_com_y", __func__);
    }

    float GetRopeCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParamRequired<float>(nh, "rope_com_z", __func__);
    }

    ////////////////////////////////////////////////////////////////////
    // Cloth settings
    ////////////////////////////////////////////////////////////////////

    float GetClothXSize(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_x_size", 0.5f);
    }

    float GetClothYSize(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_y_size", 0.5f);
    }

    float GetClothCenterOfMassX(ros::NodeHandle& nh)   // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::CLOTH_COLAB_FOLDING:
            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "cloth_com_x", GetTableSurfaceX(nh) + GetClothXSize(nh) / 2.0f);

            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_com_x", GetCylinderCenterOfMassX(nh) + GetCylinderRadius(nh) * 1.5f + GetClothXSize(nh) / 2.0f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
            default:
                return ROSHelpers::GetParamRequired<float>(nh, "cloth_com_x", __func__);
        }
    }

    float GetClothCenterOfMassY(ros::NodeHandle& nh)   // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::CLOTH_COLAB_FOLDING:
            case TaskType::CLOTH_TABLE_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_com_y", GetTableSurfaceY(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_com_y", GetCylinderCenterOfMassY(nh));

            default:
                return ROSHelpers::GetParamRequired<float>(nh, "cloth_com_y", __func__);
        }
    }

    float GetClothCenterOfMassZ(ros::NodeHandle& nh)   // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::CLOTH_COLAB_FOLDING:
            case TaskType::CLOTH_TABLE_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_com_z", GetTableSurfaceZ(nh) + 0.01f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_com_z", GetCylinderCenterOfMassZ(nh));

            default:
                return ROSHelpers::GetParamRequired<float>(nh, "cloth_com_z", __func__);
        }
    }

    float GetClothLinearStiffness(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<float>(nh, "cloth_linear_stiffness", __func__);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cloth BulletPhysics settings
    ////////////////////////////////////////////////////////////////////////////

    int GetClothNumControlPointsX(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cloth_num_control_points_x", 45);
    }

    int GetClothNumControlPointsY(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cloth_num_control_points_y", 45);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Generic target patch settings
    ////////////////////////////////////////////////////////////////////////////

    float GetCoverRegionXMin(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_x_min", 0.0f);
    }

    size_t GetCoverRegionXSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_x_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    float GetCoverRegionXRes(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_x_res", 0.01f);
    }

    float GetCoverRegionYMin(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_y_min", 0.0f);
    }

    size_t GetCoverRegionYSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_y_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    float GetCoverRegionYRes(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_y_res", 0.01f);
    }

    float GetCoverRegionZMin(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_z_min", 0.0f);
    }

    size_t GetCoverRegionZSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_z_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    float GetCoverRegionZRes(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_z_res", 0.01f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Simulator settings
    ////////////////////////////////////////////////////////////////////////////

    size_t GetNumSimstepsPerGripperCommand(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "num_simsteps_per_gripper_command", 4);
    }

    float GetSettlingTime(ros::NodeHandle& nh, const float default_time)
    {
        return ROSHelpers::GetParam(nh, "settle_time", default_time);
    }

    double GetTFWaitTime(ros::NodeHandle& nh, const double default_time)
    {
        return ROSHelpers::GetParam(nh, "tf_wait_time", default_time);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Robot settings
    ////////////////////////////////////////////////////////////////////////////

    double GetRobotControlPeriod(ros::NodeHandle& nh) // SECONDS
    {
        return ROSHelpers::GetParamDebugLog(nh, "robot_control_rate", 0.01);
    }

    double GetMaxGripperVelocityNorm(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog(nh, "max_gripper_velocity", 0.2);
    }

    double GetMaxDOFVelocityNorm(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "max_dof_velocity", 1.0);
    }

    ////////////////////////////////////////////////////////////////////////////
    // World size settings for Graph/Dijkstras - DEFINED IN BULLET FRAME, but WORLD SIZES
    ////////////////////////////////////////////////////////////////////////////

    double GetWorldXStep(ros::NodeHandle& nh)    // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_x_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_x_step", 0.02);

            default:
                ROS_FATAL_STREAM_NAMED("params", "Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
                assert(false && "This code should be unreachable");
        }
        // NO USE HERE, just for compilation
        return ROSHelpers::GetParam(nh, "world_x_step", 0.05);
    }

    double GetWorldXMinBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_x_min", GetTableSurfaceX(nh) - GetTableHalfExtentsX(nh));

            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "world_x_min", GetClothCenterOfMassX(nh) - 1.4 * GetClothXSize(nh));

            default:
                return ROSHelpers::GetParamRequired<double>(nh, "world_x_min", __func__);
        }
    }

    double GetWorldXMaxBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_x_max", GetTableSurfaceX(nh) + GetTableHalfExtentsX(nh));

            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "world_x_max", GetClothCenterOfMassX(nh) + 0.4 * GetClothXSize(nh));

            default:
                return ROSHelpers::GetParamRequired<double>(nh, "world_x_max", __func__);
        }
    }

    int64_t GetWorldXNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldXMaxBulletFrame(nh) - GetWorldXMinBulletFrame(nh))/GetWorldXStep(nh)) + 1;
    }

    double GetWorldYStep(ros::NodeHandle& nh)    // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_y_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_y_step", 0.02);

            default:
                ROS_FATAL_STREAM_NAMED("params", "Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
                assert(false && "This code should be unreachable");
        }
        // NO USE HERE, just for compilation
        return ROSHelpers::GetParam(nh, "world_y_step", 0.05);
    }

    double GetWorldYMinBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "world_y_min", GetTableSurfaceY(nh) - GetTableHalfExtentsY(nh));

            case TaskType::CLOTH_COLAB_FOLDING:
                return -0.05;

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_y_min", GetClothCenterOfMassY(nh) - 0.65 * GetClothYSize(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
            case TaskType::CLOTH_SINGLE_POLE:
                return ROSHelpers::GetParam(nh, "world_y_min", GetClothCenterOfMassY(nh) - 0.75 * GetClothYSize(nh));

            default:
                return ROSHelpers::GetParamRequired<double>(nh, "world_y_min", __func__);
        }
    }

    double GetWorldYMaxBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "world_y_max", GetTableSurfaceY(nh) + GetTableHalfExtentsY(nh));

            case TaskType::CLOTH_COLAB_FOLDING:
                return 0.05;

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_y_max", GetClothCenterOfMassY(nh) + 0.65 * GetClothYSize(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
            case TaskType::CLOTH_SINGLE_POLE:
                return ROSHelpers::GetParam(nh, "world_y_max", GetClothCenterOfMassY(nh) + 0.75 * GetClothYSize(nh));

            default:
                return ROSHelpers::GetParamRequired<double>(nh, "world_y_max", __func__);
        }
    }

    int64_t GetWorldYNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldYMaxBulletFrame(nh) - GetWorldYMinBulletFrame(nh))/GetWorldYStep(nh)) + 1;
    }

    double GetWorldZStep(ros::NodeHandle& nh)    // METERS
    {
        switch (GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_z_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_z_step", 0.02);

            default:
                ROS_FATAL_STREAM_NAMED("params", "Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
                assert(false && "This code should be unreachable");
        }
        // NO USE HERE, just for compilation
        return ROSHelpers::GetParam(nh, "world_z_step", 0.05);
    }

    double GetWorldZMinBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "world_z_min", GetTableSurfaceZ(nh));

            case TaskType::CLOTH_COLAB_FOLDING:
                return -0.05;

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_z_min", GetClothCenterOfMassY(nh) - 0.65 * GetClothXSize(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "world_z_min", GetClothCenterOfMassZ(nh) - 1.0 * GetClothXSize(nh));

            default:
                return ROSHelpers::GetParamRequired<double>(nh, "world_z_min", __func__);
        }
    }

    double GetWorldZMaxBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "world_z_max", GetTableSurfaceZ(nh) + GetCylinderHeight(nh) + GetRopeRadius(nh) * 5.0);

            case TaskType::CLOTH_COLAB_FOLDING:
                return 0.05;

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_z_max", GetClothCenterOfMassY(nh) + 0.1 * GetClothXSize(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "world_z_max", GetClothCenterOfMassZ(nh) + 0.5 * GetClothXSize(nh));

            default:
                return ROSHelpers::GetParamRequired<double>(nh, "world_z_max", __func__);
        }
    }

    int64_t GetWorldZNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldZMaxBulletFrame(nh) - GetWorldZMinBulletFrame(nh))/GetWorldZStep(nh)) + 1;
    }

    double GetWorldResolution(ros::NodeHandle& nh) // METERS
    {
        const double x_step = GetWorldXStep(nh);
        const double y_step = GetWorldYStep(nh);
        const double z_step = GetWorldZStep(nh);

        if (x_step != y_step || x_step != z_step)
        {
            throw_arc_exception(std::invalid_argument, "World resolution is only valid if x_step == y_step == z_step");
        }
        return x_step;
    }

    // Is used as a scale factor relative to GetWorldResolution.
    // The resulting voxel sizes in the SDF are
    // GetWorldResolution() / GetSDFResolutionScale() in size.
    int GetSDFResolutionScale(ros::NodeHandle& nh)
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_TABLE_COVERAGE:
            case TaskType::CLOTH_COLAB_FOLDING:
            case TaskType::CLOTH_WAFR:
            case TaskType::ROPE_ZIG_MATCH:
            case TaskType::ROPE_TABLE_LINEAR_MOTION:
            case TaskType::CLOTH_TABLE_LINEAR_MOTION:
            case TaskType::ROPE_TABLE_PENTRATION:
            case TaskType::CLOTH_PLACEMAT_LINEAR_MOTION:
                return ROSHelpers::GetParam(nh, "sdf_resolution_scale", 2);

            default:
                return ROSHelpers::GetParamRequired<int>(nh, "sdf_resolution_scale", __func__);
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner trial type settings
    ////////////////////////////////////////////////////////////////////////////

    TrialType GetTrialType(ros::NodeHandle& nh)
    {
        const static std::unordered_map<std::string, TrialType> task_map
        {
            {"diminishing_rigidity_single_model_least_squares_stretching_avoidance_controller",     TrialType::DIMINISHING_RIGIDITY_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_AVOIDANCE_CONTROLLER},
            {"adaptive_jacobian_single_model_least_squares_stretching_avoidance_controller",        TrialType::ADAPTIVE_JACOBIAN_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_AVOIDANCE_CONTROLLER},
            {"diminishing_rigidity_single_model_least_squares_stretching_constraint_controller",    TrialType::DIMINISHING_RIGIDITY_SINGLE_MODEL_LEAST_SQUARES_STRETCHING_CONSTRAINT_CONTROLLER},
            {"constraint_single_model_constraint_controller",                                       TrialType::CONSTRAINT_SINGLE_MODEL_CONSTRAINT_CONTROLLER},
            {"diminishing_rigidity_single_model_constraint_controller",                             TrialType::DIMINISHING_RIGIDITY_SINGLE_MODEL_CONSTRAINT_CONTROLLER},
            {"multi_model_bandit_test",                                                             TrialType::MULTI_MODEL_BANDIT_TEST},
            {"multi_model_controller_test",                                                         TrialType::MULTI_MODEL_CONTROLLER_TEST},
            {"multi_model_accuracy_test",                                                           TrialType::MULTI_MODEL_ACCURACY_TEST}
        };

        const auto trial_type = ROSHelpers::GetParamRequired<std::string>(nh, "trial_type", __func__);
        return task_map.at(trial_type);
    }

    MABAlgorithm GetMABAlgorithm(ros::NodeHandle& nh)
    {
        const static std::unordered_map<std::string, MABAlgorithm> algorithm_map
        {
            {"UCB", UCB1Normal},
            {"KFMANB", KFMANB},
            {"KFMANDB", KFMANDB}
        };

        const auto mab_algorithm = ROSHelpers::GetParamRequired<std::string>(nh, "multi_model/bandit_algorithm", __func__);
        return algorithm_map.at(mab_algorithm);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Diminishing Rigidity Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetDefaultDeformability(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "diminishing_rigidity/default_deformability", __func__);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Adaptive Jacobian Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetAdaptiveModelLearningRate(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "adaptive_model/adaptive_model_learning_rate", 1e-6);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Constraint Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetConstraintTranslationalDir(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "constraint_model/translational_dir_deformability", __func__);
    }

    double GetConstraintTranslationalDis(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "constraint_model/translational_dis_deformability", __func__);
    }

    double GetConstraintRotational(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "constraint_model/rotational_dis_deformability", __func__);
    }

    double GetConstraintTranslationalOldVersion(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "constraint_model/translational_old_version_deformability", __func__);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Bandit Multi-model settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetCollectResultsForAllModels(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "multi_model/collect_results_for_all_models", false);
    }

    double GetRewardScaleAnnealingFactor(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/reward_scale_annealing_factor", __func__);
        assert(0.0 <= factor && factor < 1.0);
        return factor;
    }

    double GetRewardScaleFactorStart(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/reward_std_dev_factor_start", __func__);
        assert(0.0 <= factor);
        return factor;
    }

    double GetProcessNoiseFactor(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/process_noise_factor", __func__);
        assert(0.0 <= factor);
        return factor;
    }

    double GetObservationNoiseFactor(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/observation_noise_factor", __func__);
        assert(0.0 <= factor);
        return factor;
    }

    double GetCorrelationStrengthFactor(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/correlation_strength_factor", __func__);
        assert(0.0 <= factor && factor <= 1.0);
        return factor;
    }

    double GetDeformabilityRangeMin(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/deformability_range_min", __func__);
        assert(0.0 <= factor);
        return factor;
    }

    double GetDeformabilityRangeMax(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/deformability_range_max", __func__);
        assert(0.0 <= factor);
        return factor;
    }

    double GetDeformabilityRangeStep(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/deformability_range_step", __func__);
        assert(0.0 <= factor);
        return factor;
    }

    double GetAdaptiveLearningRateRangeMin(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/adaptive_learning_rate_min", __func__);
        assert(0.0 < factor);
        return factor;
    }

    double GetAdaptiveLearningRateRangeMax(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/adaptive_learning_rate_max", __func__);
        assert(0.0 < factor);
        return factor;
    }

    double GetAdaptiveLearningRateRangeStep(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "multi_model/adaptive_learning_rate_step", __func__);
        assert(0.0 < factor);
        return factor;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetUseRandomSeed(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "use_random_seed", false);
    }

    size_t GetPlannerSeed(ros::NodeHandle& nh)
    {
        size_t seed;
        if (GetUseRandomSeed(nh))
        {
            assert(!nh.hasParam("static_seed") && "If use_random_seed is set, static_seed must not be");
            seed = std::chrono::system_clock::now().time_since_epoch().count();
        }
        else
        {
            std::stringstream ss;
            ss << std::hex << ROSHelpers::GetParamRequired<std::string>(nh, "static_seed", __func__);
            ss >> seed;
        }
        return seed;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner - Stuck detection settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetEnableStuckDetection(ros::NodeHandle& nh)
    {
        const auto enable_stuck_detection = ROSHelpers::GetParam<bool>(nh, "enable_stuck_detection", false);
        return enable_stuck_detection;
    }

    size_t GetNumLookaheadSteps(ros::NodeHandle& nh)
    {
        const auto steps = ROSHelpers::GetParamRequired<int>(nh, "stuck_detection/num_lookahead_steps", __func__);
        assert(steps >= 1);
        return (size_t)steps;
    }

    double GetRubberBandOverstretchPredictionAnnealingFactor(ros::NodeHandle& nh)
    {
        const auto factor = ROSHelpers::GetParamRequired<double>(nh, "stuck_detection/band_overstretch_prediction_annealing_factor", __func__);
        assert(0.0 <= factor && factor < 1.0);
        return factor;
    }

    size_t GetMaxGrippersPoseHistoryLength(ros::NodeHandle& nh)
    {
        size_t length = ROSHelpers::GetParam(nh, "stuck_detection/max_pose_history_steps", 20);
        assert(length >= 1);
        return length;
    }

    double GetErrorDeltaThresholdForProgress(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "stuck_detection/error_delta_threshold_for_progress", __func__);
    }

    double GetGrippersDistanceDeltaThresholdForProgress(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "stuck_detection/grippers_distance_delta_threshold_for_progress", __func__);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner - RRT settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetRRTReuseOldResults(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<bool>(nh, "rrt/reuse_old_results", __func__);
    }

    bool GetRRTStoreNewResults(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<bool>(nh, "rrt/store_new_results", __func__);
    }

    double GetRRTHomotopyDistancePenalty()
    {
        return 1e3;
    }

    double GetRRTBandDistance2ScalingFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/band_dist2_scaling_factor", __func__);
    }

    size_t GetRRTBandMaxPoints(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<int>(nh, "rrt/band_max_points", __func__);
    }

    double GetRRTMaxRobotDOFStepSize(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/max_robot_dof_step_size", __func__);
    }

    double GetRRTMinRobotDOFStepSize(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/min_robot_dof_step_size", __func__);
    }

    double GetRRTMaxGripperRotation(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/max_gripper_rotation", __func__);
    }

    double GetRRTGoalBias(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/goal_bias", __func__);
    }

    double GetRRTBestNearRadius(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/best_near_radius", __func__);
    }

    double GetRRTFeasibilityDistanceScaleFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/feasibility_distance_scale_factor", __func__);
    }

    int64_t GetRRTMaxShortcutIndexDistance(ros::NodeHandle& nh)
    {
        return (int64_t)ROSHelpers::GetParamRequired<int>(nh, "rrt/max_shortcut_index_distance", __func__);
    }

    uint32_t GetRRTMaxSmoothingIterations(ros::NodeHandle& nh)
    {
        return (uint32_t)ROSHelpers::GetParamRequired<int>(nh, "rrt/max_smoothing_iterations",  __func__);
    }

    double GetRRTSmoothingBandDistThreshold(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/smoothing_band_dist_threshold", __func__);
    }

    double GetRRTTimeout(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/timeout", __func__);
        assert(val > 0.0);
        return val;
    }

    size_t GetRRTNumTrials(ros::NodeHandle& nh)
    {
        return (size_t)ROSHelpers::GetParam(nh, "rrt/num_trials", 1);
    }

    double GetRRTPlanningXMinBulletFrame(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_x_min", __func__);
    }

    double GetRRTPlanningXMaxBulletFrame(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_x_max", __func__);
    }

    double GetRRTPlanningYMinBulletFrame(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_y_min", __func__);
    }

    double GetRRTPlanningYMaxBulletFrame(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_y_max", __func__);
    }

    double GetRRTPlanningZMinBulletFrame(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_z_min", __func__);
    }

    double GetRRTPlanningZMaxBulletFrame(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_z_max", __func__);
    }

    bool GetUseCBiRRTStyleProjection(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<bool>(nh, "rrt/use_cbirrt_style_projection", __func__);
    }

    size_t GetRRTForwardTreeExtendIterations(ros::NodeHandle& nh)
    {
        return (size_t)ROSHelpers::GetParamRequired<int>(nh, "rrt/forward_tree_extend_iterations", __func__);
    }

    size_t GetRRTBackwardTreeExtendIterations(ros::NodeHandle& nh)
    {
        return (size_t)ROSHelpers::GetParamRequired<int>(nh, "rrt/backward_tree_extend_iterations", __func__);
    }

    bool GetRRTUseBruteForceNN(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<bool>(nh, "rrt/use_brute_force_nn", __func__);
    }

    size_t GetRRTKdTreeGrowThreshold(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<int>(nh, "rrt/kd_tree_grow_threshold", __func__);
    }

    bool GetRRTTestPathsInBullet(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<bool>(nh, "rrt/test_paths_in_bullet", __func__);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Transition Learning Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetTransitionMistakeThreshold(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "transition_estimation/mistake_distance_threshold", __func__);
    }

    double GetTransitionDefaultPropagationConfidence(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "transition_estimation/default_propagation_confidence", __func__);
    }

    double GetTransitionDefaultBandDistThreshold(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "transition_estimation/default_band_dist_threshold", __func__);
    }

    double GetTransitionConfidenceThreshold(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "transition_estimation/confidence_threshold", __func__);
    }

    double GetTransitionTemplateMisalignmentScaleFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "transition_estimation/template_misalignment_scale_factor", __func__);
    }

    double GetTransitionTightenDeltaScaleFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "transition_estimation/tighten_delta_scale_factor", __func__);
    }

    double GetTransitionHomotopyChangesScaleFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "transition_estimation/homotopy_changes_scale_factor", __func__);
    }

    ClassifierType GetClassifierType(ros::NodeHandle& nh)
    {
        const static std::unordered_map<std::string, ClassifierType> classifier_type_map
        {
            {"none",    ClassifierType::None},
            {"kNN",     ClassifierType::kNN},
            {"svm",     ClassifierType::SVM},
            {"mlp",     ClassifierType::MLP},
            {"voxnet",  ClassifierType::VOXNET},
        };

        const auto classifier_type = ROSHelpers::GetParamRequiredDebugLog<std::string>(nh, "classifier/type", __func__);
        return classifier_type_map.at(classifier_type);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Pure Jacobian based motion controller paramters
    ////////////////////////////////////////////////////////////////////////////

    bool GetJacobianControllerOptimizationEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "jacobian_controller/optimization_enabled", false);
    }

    double GetCollisionScalingFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "jacobian_controller/collision_scaling_factor", __func__);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Stretching constraint controller parameters
    ////////////////////////////////////////////////////////////////////////////

    StretchingConstraintControllerSolverType GetStretchingConstraintControllerSolverType(ros::NodeHandle& nh)
    {
        const static std::unordered_map<std::string, StretchingConstraintControllerSolverType> solver_map
        {
            {"random_sampling",     StretchingConstraintControllerSolverType::RANDOM_SAMPLING},
            {"nomad_optimization",  StretchingConstraintControllerSolverType::NOMAD_OPTIMIZATION},
            {"gradient_descent",    StretchingConstraintControllerSolverType::GRADIENT_DESCENT},
        };

        const auto solver_type = ROSHelpers::GetParamRequired<std::string>(nh, "stretching_constraint_controller/solver_type", __func__);
        return solver_map.at(solver_type);
    }

    int64_t GetMaxSamplingCounts(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<int>(nh, "stretching_constraint_controller/max_sampling_counts", __func__);
    }

    bool GetUseFixedGripperDeltaSize(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<bool>(nh, "stretching_constraint_controller/fix_step_size", __func__);
    }

    double GetStretchingCosineThreshold(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "stretching_constraint_controller/stretching_cosine_threshold", __func__);
    }

    bool GetVisualizeOverstretchCones(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "stretching_constraint_controller/visualize_overstretch_cones", true);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Straight line motion parameters for testing model accuracy
    // Note: these parameters are gripper velocities *in gripper frame*
    ////////////////////////////////////////////////////////////////////////////

    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double, 6, 1>>>
    GetGripperDeltaTrajectory(ros::NodeHandle& nh, const std::string& gripper_name)
    {
        const std::string base_param_name = "straight_line_motion_controller/" + gripper_name + "_deltas/";

        std::vector<double> t;
        if (!nh.getParam(base_param_name + "t", t))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << base_param_name + "t" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> vx;
        if (!nh.getParam(base_param_name + "vx", vx))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << base_param_name + "vx" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> vy;
        if (!nh.getParam(base_param_name + "vy", vy))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << base_param_name + "vy" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> vz;
        if (!nh.getParam(base_param_name + "vz", vz))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << base_param_name + "vz" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> wx;
        if (!nh.getParam(base_param_name + "wx", wx))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << base_param_name + "wx" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> wy;
        if (!nh.getParam(base_param_name + "wy", wy))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << base_param_name + "wy" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> wz;
        if (!nh.getParam(base_param_name + "wz", wz))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << base_param_name + "wz" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        assert(t.size() == vx.size());
        assert(t.size() == vy.size());
        assert(t.size() == vz.size());
        assert(t.size() == wx.size());
        assert(t.size() == wy.size());

        std::vector<Eigen::Matrix<double, 6, 1>> deltas(t.size());assert(t.size() == wz.size());
        for (size_t ind = 0; ind < t.size(); ++ind)
        {
            deltas[ind](0) = vx[ind];
            deltas[ind](1) = vy[ind];
            deltas[ind](2) = vz[ind];
            deltas[ind](3) = wx[ind];
            deltas[ind](4) = wy[ind];
            deltas[ind](5) = wz[ind];
        }

        return {t, deltas};
    }

    double GetGripperStraightLineMotionTransX(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/vx", __func__);
    }

    double GetGripperStraightLineMotionTransY(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/vy", __func__);
    }

    double GetGripperStraightLineMotionTransZ(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/vz", __func__);
    }

    double GetGripperStraightLineMotionAngularX(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/wx", __func__);
    }

    double GetGripperStraightLineMotionAngularY(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/wy", __func__);
    }

    double GetGripperStraightLineMotionAngularZ(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/wz", __func__);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Logging functionality
    ////////////////////////////////////////////////////////////////////////////

    bool GetBanditsLoggingEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "bandits_logging_enabled", false);
    }

    bool GetControllerLoggingEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "controller_logging_enabled", false);
    }

    std::string GetLogFolder(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "log_folder", "/tmp/");
    }

    std::string GetDataFolder(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "data_folder", "/tmp/");
    }

    std::string GetDijkstrasStorageLocation(ros::NodeHandle& nh)
    {
        const std::string base_path = GetLogFolder(nh);
        const std::string task_name = ROSHelpers::GetParamRequired<std::string>(nh, "task_type", __func__);
        const std::string default_dijkstras_file_path =
                base_path + "../" + task_name + ".dijkstras_serialized";
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "dijkstras_file_path", default_dijkstras_file_path);
    }

    std::string GetCollisionMapStorageLocation(ros::NodeHandle& nh)
    {
        const std::string base_path = GetLogFolder(nh);
        const std::string task_name = ROSHelpers::GetParamRequired<std::string>(nh, "task_type", __func__);
        const std::string default_collision_map_file_path =
                base_path + "../" + task_name + ".collision_map_serialized";
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "collision_map_file_path", default_collision_map_file_path);
    }

    bool GetScreenshotsEnabled(ros::NodeHandle& nh)
    {
        const bool screenshots_enabled = ROSHelpers::GetParam(nh, "screenshots_enabled", false);
        // The viewer must be enabled for screen shots to be enabled
        assert(!screenshots_enabled || ROSHelpers::GetParam(nh, "start_bullet_viewer", true));
        return screenshots_enabled;
    }

    std::string GetScreenshotFolder(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "screenshot_folder", GetLogFolder(nh) + "screenshots/");
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS Topic settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetTestRobotMotionTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "test_robot_motion_topic", "test_robot_motion");
    }

    std::string GetExecuteRobotMotionTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "execute_robot_motion_topic", "execute_robot_motion");
    }

    std::string GetTestRobotMotionMicrostepsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "test_robot_motion_microsteps_topic" , "test_robot_motion_microsteps");
    }

    std::string GetGenerateTransitionDataTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "generate_transition_data_topic" , "generate_transition_data");
    }

    std::string GetTestRobotPathsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "test_robot_paths_topic" , "test_robot_paths");
    }

    std::string GetWorldStateTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "world_state_topic", "world_state");
    }

    std::string GetCoverPointsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_cover_points_topic", "get_cover_points");
    }

    std::string GetCoverPointNormalsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_cover_point_normals_topic", "get_cover_point_normals");
    }

    std::string GetMirrorLineTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_mirror_line_topic", "get_mirror_line");
    }

    std::string GetFreeSpaceGraphTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_free_space_graph_topic", "get_free_space_graph");
    }

    std::string GetSignedDistanceFieldTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_signed_distance_field_topic", "get_signed_distance_field");
    }

    std::string GetGripperNamesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_names_topic", "get_gripper_names");
    }

    std::string GetGripperAttachedNodeIndicesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_attached_node_indices_topic", "get_gripper_attached_node_indices");
    }

    std::string GetGripperStretchingVectorInfoTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_stretching_vector_topic", "get_gripper_stretching_vector");
    }

    std::string GetGripperPoseTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_pose_topic", "get_gripper_pose");
    }

    std::string GetRobotConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_robot_configuration_topic", "get_robot_configuration");
    }

    std::string GetObjectInitialConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_object_initial_configuration_topic", "get_object_initial_configuration");
    }

    std::string GetObjectCurrentConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_object_current_configuration_topic", "get_object_current_configuration");
    }

    std::string GetRopeCurrentNodeTransformsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_rope_current_node_transforms_topic", "get_rope_current_node_transforms");
    }

    std::string GetVisualizationMarkerTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "visualization_marker_topic", "visualization_marker");
    }

    std::string GetVisualizationMarkerArrayTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "visualization_marker_array_topic", "visualization_marker_vector");
    }

    std::string GetClearVisualizationsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "clear_visualizations_topic", "clear_visualizations");
    }

    std::string GetConfidenceTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "confidence_topic", "confidence");
    }

    std::string GetConfidenceImageTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "confidence_image_topic", "confidence_image");
    }

    std::string GetGripperCollisionCheckTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_collision_check_topic", "get_gripper_collision_check");
    }

    std::string GetRestartSimulationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "restart_simulation_topic", "restart_simulation");
    }

    std::string GetTerminateSimulationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "terminate_simulation_topic", "terminate_simulation");
    }

    ////////////////////////////////////////////////////////////////////////////
    // Live Robot Settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetGripper0Name(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "gripper0_name", "left");
    }

    std::string GetGripper1Name(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "gripper1_name", "right");
    }

    std::string GetGripper0TFName(ros::NodeHandle& nh)
    {
        const std::string name = GetGripper0Name(nh);
        assert(name == "left" || name == "right");
        const bool use_victor = ROSHelpers::GetParamRequired<bool>(nh, "use_victor", __func__);
        const bool use_val = ROSHelpers::GetParamRequired<bool>(nh, "use_val", __func__);
        assert ((use_victor ^ use_val) && "Only one of Victor or Val can be specified");
        if (use_victor)
        {
            return "victor_" + name + "_gripper";
        }
        else if (use_val)
        {
            return name + "gripper_tip";
        }
        else
        {
            assert(false && "This should not be possible");
        }
        return "no use here, just for compilation";
    }

    std::string GetGripper1TFName(ros::NodeHandle& nh)
    {
        const std::string name = GetGripper1Name(nh);
        assert(name == "left" || name == "right");
        const bool use_victor = ROSHelpers::GetParamRequired<bool>(nh, "use_victor", __func__);
        const bool use_val = ROSHelpers::GetParamRequired<bool>(nh, "use_val", __func__);
        assert ((use_victor ^ use_val) && "Only one of Victor or Val can be specified");
        if (use_victor)
        {
            return "victor_" + name + "_gripper";
        }
        else if (use_val)
        {
            return name + "gripper_tip";
        }
        else
        {
            assert(false && "This should not be possible");
        }
        return "no use here, just for compilation";
    }

    size_t GetGripperAttachedIdx(ros::NodeHandle& nh, const std::string& gripper_name)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, gripper_name + "_gripper_attached_node_idx", __func__);
        assert(val >= 0);
        return (size_t)val;
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS TF Frame name settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetBulletFrameName()
    {
        return "bullet_origin";
    }

    std::string GetTaskFrameName()
    {
        return GetBulletFrameName();
    }

    std::string GetWorldFrameName()
    {
        return "world_origin";
    }

    std::string GetTableFrameName()
    {
        return "table_surface";
    }
}
