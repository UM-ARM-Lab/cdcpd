#ifndef ROS_PARAMS_HPP
#define ROS_PARAMS_HPP

#include <string>
#include <vector>
#include <Eigen/Core>
#include <ros/ros.h>

#include "deformable_manipulation_experiment_params/task_enums.h"

namespace smmap
{
    ////////////////////////////////////////////////////////////////////////////
    // Visualization Settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetDisableSmmapVisualizations(ros::NodeHandle& nh);
    bool GetVisualizeObjectDesiredMotion(ros::NodeHandle& nh);
    bool GetVisualizeGripperMotion(ros::NodeHandle& nh);
    bool GetVisualizeObjectPredictedMotion(ros::NodeHandle& nh);
    bool GetVisualizeRRT(ros::NodeHandle& nh, const bool default_vis = true);
    bool GetVisualizeFreeSpaceGraph(ros::NodeHandle& nh);
    bool GetVisualizeCorrespondences(ros::NodeHandle& nh);
    bool VisualizeStrainLines(ros::NodeHandle& nh);

    int GetViewerWidth(ros::NodeHandle& nh);     // Pixels
    int GetViewerHeight(ros::NodeHandle& nh);    // Pixels

    ////////////////////////////////////////////////////////////////////////////
    // Task and Deformable Type parameters
    ////////////////////////////////////////////////////////////////////////////

    std::string GetTestId(ros::NodeHandle& nh);
    DeformableType GetDeformableType(ros::NodeHandle& nh);
    std::string GetTaskTypeString(ros::NodeHandle& nh);
    TaskType GetTaskType(ros::NodeHandle& nh);
    double GetMaxTime(ros::NodeHandle& nh);
    double GetMaxStretchFactor(ros::NodeHandle& nh);
    double GetMaxBandLength(ros::NodeHandle& nh);
    float GetMaxStrain(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Error calculation settings
    ////////////////////////////////////////////////////////////////////////////

    double GetErrorThresholdAlongNormal(ros::NodeHandle& nh);
    double GetErrorThresholdDistanceToNormal(ros::NodeHandle& nh);
    double GetErrorThresholdTaskDone(ros::NodeHandle& nh);
    double GetDesiredMotionScalingFactor(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Gripper Size Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetGripperApperture(ros::NodeHandle& nh);                     // METERS
    // TODO: where is this still used? Is it being used correctly vs ControllerMinDistToObstacles?
    double GetRobotGripperRadius();                                     // METERS
    // Used by the "older" avoidance code, I.e. LeastSquaresControllerWithObjectAvoidance
    double GetRobotMinGripperDistanceToObstacles();                     // METERS
    double GetControllerMinDistanceToObstacles(ros::NodeHandle& nh);    // METERS
    double GetRRTMinGripperDistanceToObstacles(ros::NodeHandle& nh);    // METERS
    double GetRRTTargetMinDistanceScaleFactor(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Table Size Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetTableSurfaceX(ros::NodeHandle& nh);        // METERS
    float GetTableSurfaceY(ros::NodeHandle& nh);        // METERS
    float GetTableSurfaceZ(ros::NodeHandle& nh);        // METERS
    float GetTableHalfExtentsX(ros::NodeHandle& nh);    // METERS
    float GetTableHalfExtentsY(ros::NodeHandle& nh);    // METERS
    float GetTableHeight(ros::NodeHandle& nh);          // METERS
    float GetTableLegWidth(ros::NodeHandle& nh);        // METERS
    float GetTableThickness(ros::NodeHandle& nh);       // METERS

    ////////////////////////////////////////////////////////////////////////////
    // Cylinder Size Settings
    // TODO: Update launch files to contain these defaults
    ////////////////////////////////////////////////////////////////////////////

    float GetCylinderRadius(ros::NodeHandle& nh);           // METERS
    float GetCylinderHeight(ros::NodeHandle& nh);           // METERS
    float GetCylinderCenterOfMassX(ros::NodeHandle& nh);    // METERS
    float GetCylinderCenterOfMassY(ros::NodeHandle& nh);    // METERS
    float GetCylinderCenterOfMassZ(ros::NodeHandle& nh);    // METERS

    // Cylinder Size settings for WAFR task case
    float GetWafrCylinderRadius(ros::NodeHandle& nh);       // METERS
    float GetWafrCylinderHeight(ros::NodeHandle& nh);       // METERS
    float GetWafrCylinderRelativeCenterOfMassX(ros::NodeHandle& nh);    // METERS
    float GetWafrCylinderRelativeCenterOfMassY(ros::NodeHandle& nh);    // METERS
    float GetWafrCylinderRelativeCenterOfMassZ(ros::NodeHandle& nh);    // METERS

    ////////////////////////////////////////////////////////////////////////////
    // Rope Maze Wall Size and Visibility Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetWallHeight(ros::NodeHandle& nh);           // METERS
    float GetWallCenterOfMassZ(ros::NodeHandle& nh);    // METERS
    float GetOuterWallsAlpha(ros::NodeHandle& nh);      // 0.0 thru 1.0 (inclusive)
    float GetFloorDividerAlpha(ros::NodeHandle& nh);    // 0.0 thru 1.0 (inclusive)
    float GetFirstFloorAlpha(ros::NodeHandle& nh);      // 0.0 thru 1.0 (inclusive)
    float GetSecondFloorAlpha(ros::NodeHandle& nh);     // 0.0 thru 1.0 (inclusive)

    ////////////////////////////////////////////////////////////////////////////
    // Rope Settings
    ////////////////////////////////////////////////////////////////////////////

    float GetRopeSegmentLength(ros::NodeHandle& nh);    // METERS
    float GetRopeRadius(ros::NodeHandle& nh);           // METERS
    int GetRopeNumLinks(ros::NodeHandle& nh);
    float GetRopeExtensionVectorX(ros::NodeHandle& nh);
    float GetRopeExtensionVectorY(ros::NodeHandle& nh);
    float GetRopeExtensionVectorZ(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Rope starting position settings
    ////////////////////////////////////////////////////////////////////////////

    float GetRopeCenterOfMassX(ros::NodeHandle& nh);    // METERS
    float GetRopeCenterOfMassY(ros::NodeHandle& nh);    // METERS
    float GetRopeCenterOfMassZ(ros::NodeHandle& nh);    // METERS

    ////////////////////////////////////////////////////////////////////////////
    // Cloth settings
    ////////////////////////////////////////////////////////////////////////////

    float GetClothXSize(ros::NodeHandle& nh);           // METERS
    float GetClothYSize(ros::NodeHandle& nh);           // METERS
    float GetClothCenterOfMassX(ros::NodeHandle& nh);   // METERS
    float GetClothCenterOfMassY(ros::NodeHandle& nh);   // METERS
    float GetClothCenterOfMassZ(ros::NodeHandle& nh);   // METERS
    float GetClothLinearStiffness(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Cloth BulletPhysics settings
    ////////////////////////////////////////////////////////////////////////////

    int GetClothNumControlPointsX(ros::NodeHandle& nh);
    int GetClothNumControlPointsY(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Generic target patch settings
    ////////////////////////////////////////////////////////////////////////////

    float GetCoverRegionXMin(ros::NodeHandle& nh);  // METERS
    size_t GetCoverRegionXSteps(ros::NodeHandle& nh);
    float GetCoverRegionXRes(ros::NodeHandle& nh);  // METERS
    float GetCoverRegionYMin(ros::NodeHandle& nh);  // METERS
    size_t GetCoverRegionYSteps(ros::NodeHandle& nh);
    float GetCoverRegionYRes(ros::NodeHandle& nh);  // METERS
    float GetCoverRegionZMin(ros::NodeHandle& nh);  // METERS
    size_t GetCoverRegionZSteps(ros::NodeHandle& nh);
    float GetCoverRegionZRes(ros::NodeHandle& nh);  // METERS

    ////////////////////////////////////////////////////////////////////////////
    // Simulator settings
    ////////////////////////////////////////////////////////////////////////////

    size_t GetNumSimstepsPerGripperCommand(ros::NodeHandle& nh);
    float GetSettlingTime(ros::NodeHandle& nh, const float default_time = 4.0);
    double GetTFWaitTime(ros::NodeHandle& nh, const double default_time = 4.0);

    ////////////////////////////////////////////////////////////////////////////
    // Robot settings
    ////////////////////////////////////////////////////////////////////////////

    double GetRobotControlPeriod(ros::NodeHandle& nh);      // SECONDS
    double GetMaxGripperVelocityNorm(ros::NodeHandle& nh);  // SE(3) velocity
    double GetMaxDOFVelocityNorm(ros::NodeHandle& nh);      // rad/s

    ////////////////////////////////////////////////////////////////////////////
    // World size settings for Graph/Dijkstras - DEFINED IN BULLET FRAME, but WORLD SIZES
    ////////////////////////////////////////////////////////////////////////////

    double GetWorldXStep(ros::NodeHandle& nh);              // METERS
    double GetWorldXMinBulletFrame(ros::NodeHandle& nh);    // METERS
    double GetWorldXMaxBulletFrame(ros::NodeHandle& nh);    // METERS
    int64_t GetWorldXNumSteps(ros::NodeHandle& nh);
    double GetWorldYStep(ros::NodeHandle& nh);              // METERS
    double GetWorldYMinBulletFrame(ros::NodeHandle& nh);    // METERS
    double GetWorldYMaxBulletFrame(ros::NodeHandle& nh);    // METERS
    int64_t GetWorldYNumSteps(ros::NodeHandle& nh);
    double GetWorldZStep(ros::NodeHandle& nh);              // METERS
    double GetWorldZMinBulletFrame(ros::NodeHandle& nh);    // METERS
    double GetWorldZMaxBulletFrame(ros::NodeHandle& nh);    // METERS
    int64_t GetWorldZNumSteps(ros::NodeHandle& nh);
    double GetWorldResolution(ros::NodeHandle& nh);         // METERS

    // Is used as a scale factor relative to GetWorldResolution.
    // The resulting voxel sizes in the SDF are
    // GetWorldResolution() / GetSDFResolutionScale() in size.
    int GetSDFResolutionScale(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Planner trial type settings
    ////////////////////////////////////////////////////////////////////////////

    TrialType GetTrialType(ros::NodeHandle& nh);
    MABAlgorithm GetMABAlgorithm(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Diminishing Rigidity Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetDefaultDeformability(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Adaptive Jacobian Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetAdaptiveModelLearningRate(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Constraint Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetConstraintTranslationalDir(ros::NodeHandle& nh);
    double GetConstraintTranslationalDis(ros::NodeHandle& nh);
    double GetConstraintRotational(ros::NodeHandle& nh);
    double GetConstraintTranslationalOldVersion(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Bandit Multi-model settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetCollectResultsForAllModels(ros::NodeHandle& nh);
    double GetRewardScaleAnnealingFactor(ros::NodeHandle& nh);
    double GetRewardScaleFactorStart(ros::NodeHandle& nh);
    double GetProcessNoiseFactor(ros::NodeHandle& nh);
    double GetObservationNoiseFactor(ros::NodeHandle& nh);
    double GetCorrelationStrengthFactor(ros::NodeHandle& nh);
    double GetDeformabilityRangeMin(ros::NodeHandle& nh);
    double GetDeformabilityRangeMax(ros::NodeHandle& nh);
    double GetDeformabilityRangeStep(ros::NodeHandle& nh);
    double GetAdaptiveLearningRateRangeMin(ros::NodeHandle& nh);
    double GetAdaptiveLearningRateRangeMax(ros::NodeHandle& nh);
    double GetAdaptiveLearningRateRangeStep(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Planner settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetUseRandomSeed(ros::NodeHandle& nh);
    size_t GetPlannerSeed(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Planner - Stuck detection settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetEnableStuckDetection(ros::NodeHandle& nh);
    size_t GetNumLookaheadSteps(ros::NodeHandle& nh);
    double GetRubberBandOverstretchPredictionAnnealingFactor(ros::NodeHandle& nh);
    size_t GetMaxGrippersPoseHistoryLength(ros::NodeHandle& nh);
    double GetErrorDeltaThresholdForProgress(ros::NodeHandle& nh);
    double GetGrippersDistanceDeltaThresholdForProgress(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Planner - RRT settings
    ////////////////////////////////////////////////////////////////////////////

    bool GetRRTReuseOldResults(ros::NodeHandle& nh);
    bool GetRRTStoreNewResults(ros::NodeHandle& nh);
    double GetRRTHomotopyDistancePenalty();
    double GetRRTBandDistance2ScalingFactor(ros::NodeHandle& nh);
    size_t GetRRTBandMaxPoints(ros::NodeHandle& nh);
    double GetRRTMaxRobotDOFStepSize(ros::NodeHandle& nh);
    double GetRRTMinRobotDOFStepSize(ros::NodeHandle& nh);
    double GetRRTMaxGripperRotation(ros::NodeHandle& nh);
    double GetRRTGoalBias(ros::NodeHandle& nh);
    double GetRRTBestNearRadius(ros::NodeHandle& nh);
    double GetRRTFeasibilityDistanceScaleFactor(ros::NodeHandle& nh);
    int64_t GetRRTMaxShortcutIndexDistance(ros::NodeHandle& nh);
    uint32_t GetRRTMaxSmoothingIterations(ros::NodeHandle& nh);
    double GetRRTSmoothingBandDistThreshold(ros::NodeHandle& nh);
    double GetRRTTimeout(ros::NodeHandle& nh);
    size_t GetRRTNumTrials(ros::NodeHandle& nh);
    double GetRRTPlanningXMinBulletFrame(ros::NodeHandle& nh);
    double GetRRTPlanningXMaxBulletFrame(ros::NodeHandle& nh);
    double GetRRTPlanningYMinBulletFrame(ros::NodeHandle& nh);
    double GetRRTPlanningYMaxBulletFrame(ros::NodeHandle& nh);
    double GetRRTPlanningZMinBulletFrame(ros::NodeHandle& nh);
    double GetRRTPlanningZMaxBulletFrame(ros::NodeHandle& nh);
    bool GetUseCBiRRTStyleProjection(ros::NodeHandle& nh);
    size_t GetRRTForwardTreeExtendIterations(ros::NodeHandle& nh);
    size_t GetRRTBackwardTreeExtendIterations(ros::NodeHandle& nh);
    bool GetRRTUseBruteForceNN(ros::NodeHandle& nh);
    size_t GetRRTKdTreeGrowThreshold(ros::NodeHandle& nh);
    bool GetRRTTestPathsInBullet(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Transition Learning Parameters
    ////////////////////////////////////////////////////////////////////////////

    double GetTransitionMistakeThreshold(ros::NodeHandle& nh);
    double GetTransitionDefaultPropagationConfidence(ros::NodeHandle& nh);
    double GetTransitionDefaultBandDistThreshold(ros::NodeHandle& nh);
    double GetTransitionConfidenceThreshold(ros::NodeHandle& nh);
    double GetTransitionTemplateMisalignmentScaleFactor(ros::NodeHandle& nh);
    double GetTransitionTightenDeltaScaleFactor(ros::NodeHandle& nh);
    double GetTransitionHomotopyChangesScaleFactor(ros::NodeHandle& nh);

    ClassifierType GetClassifierType(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Pure Jacobian based motion controller paramters
    ////////////////////////////////////////////////////////////////////////////

    bool GetJacobianControllerOptimizationEnabled(ros::NodeHandle& nh);
    double GetCollisionScalingFactor(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Stretching constraint controller parameters
    ////////////////////////////////////////////////////////////////////////////

    StretchingConstraintControllerSolverType GetStretchingConstraintControllerSolverType(ros::NodeHandle& nh);
    int64_t GetMaxSamplingCounts(ros::NodeHandle& nh);
    bool GetUseFixedGripperDeltaSize(ros::NodeHandle& nh);
    double GetStretchingCosineThreshold(ros::NodeHandle& nh);
    bool GetVisualizeOverstretchCones(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Straight line motion parameters for testing model accuracy
    // Note: these parameters are gripper velocities *in gripper frame*
    ////////////////////////////////////////////////////////////////////////////

    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double, 6, 1>>>
    GetGripperDeltaTrajectory(ros::NodeHandle& nh, const std::string& gripper_name);
    double GetGripperStraightLineMotionTransX(ros::NodeHandle& nh);
    double GetGripperStraightLineMotionTransY(ros::NodeHandle& nh);
    double GetGripperStraightLineMotionTransZ(ros::NodeHandle& nh);
    double GetGripperStraightLineMotionAngularX(ros::NodeHandle& nh);
    double GetGripperStraightLineMotionAngularY(ros::NodeHandle& nh);
    double GetGripperStraightLineMotionAngularZ(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Logging functionality
    ////////////////////////////////////////////////////////////////////////////

    bool GetBanditsLoggingEnabled(ros::NodeHandle& nh);
    bool GetControllerLoggingEnabled(ros::NodeHandle& nh);
    std::string GetLogFolder(ros::NodeHandle& nh);
    std::string GetDataFolder(ros::NodeHandle& nh);
    std::string GetDijkstrasStorageLocation(ros::NodeHandle& nh);
    std::string GetCollisionMapStorageLocation(ros::NodeHandle& nh);
    bool GetScreenshotsEnabled(ros::NodeHandle& nh);
    std::string GetScreenshotFolder(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // ROS Topic settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetTestRobotMotionTopic(ros::NodeHandle& nh);
    std::string GetExecuteRobotMotionTopic(ros::NodeHandle& nh);
    std::string GetTestRobotMotionMicrostepsTopic(ros::NodeHandle& nh);
    std::string GetGenerateTransitionDataTopic(ros::NodeHandle& nh);
    std::string GetTestRobotPathsTopic(ros::NodeHandle& nh);
    std::string GetWorldStateTopic(ros::NodeHandle& nh);
    std::string GetCoverPointsTopic(ros::NodeHandle& nh);
    std::string GetCoverPointNormalsTopic(ros::NodeHandle& nh);
    std::string GetMirrorLineTopic(ros::NodeHandle& nh);
    std::string GetFreeSpaceGraphTopic(ros::NodeHandle& nh);
    std::string GetSignedDistanceFieldTopic(ros::NodeHandle& nh);
    std::string GetGripperNamesTopic(ros::NodeHandle& nh);
    std::string GetGripperAttachedNodeIndicesTopic(ros::NodeHandle& nh);
    std::string GetGripperStretchingVectorInfoTopic(ros::NodeHandle& nh);
    std::string GetGripperPoseTopic(ros::NodeHandle& nh);
    std::string GetRobotConfigurationTopic(ros::NodeHandle& nh);
    std::string GetObjectInitialConfigurationTopic(ros::NodeHandle& nh);
    std::string GetObjectCurrentConfigurationTopic(ros::NodeHandle& nh);
    std::string GetRopeCurrentNodeTransformsTopic(ros::NodeHandle& nh);
    std::string GetVisualizationMarkerTopic(ros::NodeHandle& nh);
    std::string GetVisualizationMarkerArrayTopic(ros::NodeHandle& nh);
    std::string GetClearVisualizationsTopic(ros::NodeHandle& nh);
    std::string GetConfidenceTopic(ros::NodeHandle& nh);
    std::string GetConfidenceImageTopic(ros::NodeHandle& nh);
    std::string GetGripperCollisionCheckTopic(ros::NodeHandle& nh);
    std::string GetRestartSimulationTopic(ros::NodeHandle& nh);
    std::string GetTerminateSimulationTopic(ros::NodeHandle& nh);

    ////////////////////////////////////////////////////////////////////////////
    // Live Robot Settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetGripper0Name(ros::NodeHandle& nh);
    std::string GetGripper1Name(ros::NodeHandle& nh);
    std::string GetGripper0TFName(ros::NodeHandle& nh);
    std::string GetGripper1TFName(ros::NodeHandle& nh);
    size_t GetGripperAttachedIdx(ros::NodeHandle& nh, const std::string& gripper_name);

    ////////////////////////////////////////////////////////////////////////////
    // ROS TF Frame name settings
    ////////////////////////////////////////////////////////////////////////////

    std::string GetBulletFrameName();
    std::string GetTaskFrameName();
    std::string GetWorldFrameName();
    std::string GetTableFrameName();
}

#endif // ROS_PARAMS_HPP
