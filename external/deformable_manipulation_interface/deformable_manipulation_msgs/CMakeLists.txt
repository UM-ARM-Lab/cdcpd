cmake_minimum_required(VERSION 2.8.3)
project(deformable_manipulation_msgs)

set(CATKIN_PACKAGES
  actionlib_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  sdf_tools
)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    ConfidenceStamped.msg
    VectorPose.msg
    VectorConfiguration.msg
    WorldState.msg
    RobotPathTest.msg
    RobotPathTestResult.msg
    TransitionTest.msg
    TransitionTestResult.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    ExecuteRobotMotion.srv
    GetGripperNames.srv
    GetGripperAttachedNodeIndices.srv
    GetGripperCollisionReport.srv
    GetGripperPose.srv
    GetGripperStretchingVectorInfo.srv
    GetPointSet.srv
    GetVector3Set.srv
    GetPoseSet.srv
    GetMirrorLine.srv
    GetFreeSpaceGraph.srv
    GetSignedDistanceField.srv
    GetRobotConfiguration.srv
    TestRobotMotionMicrosteps.srv
    TransitionTestingVisualization.srv
)

## Generate actions in the 'action' folder
add_action_files(
    FILES
    TestRobotMotion.action
    GenerateTransitionData.action
    TestRobotPaths.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    ${CATKIN_PACKAGES}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
#    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
                    message_runtime
)

###########
## Build ##
###########

# Just mark this as a source in order to force it to show up in the IDE
add_custom_target(${PROJECT_NAME}_headers SOURCES include/${PROJECT_NAME}/messages.h)
