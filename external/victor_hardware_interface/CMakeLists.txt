cmake_minimum_required(VERSION 2.8.3)
project(victor_hardware_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_PACKAGES
    arc_utilities
    diagnostic_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    lcm)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES
    CartesianControlModeLimits.msg
    CartesianImpedanceParameters.msg
    CartesianPathExecutionParameters.msg
    CartesianValueQuantity.msg
    ControlMode.msg
    ControlModeParameters.msg
    JointImpedanceParameters.msg
    JointPathExecutionParameters.msg
    JointValueQuantity.msg
    MotionCommand.msg
    MotionStatus.msg
    Robotiq3FingerActuatorCommand.msg
    Robotiq3FingerActuatorStatus.msg
    Robotiq3FingerCommand.msg
    Robotiq3FingerObjectStatus.msg
    Robotiq3FingerStatus.msg
    Robotiq3FingerStatus_sync.msg
    GraspStatus.msg)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES
    SetControlMode.srv
    GetControlMode.srv)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    diagnostic_msgs
    geometry_msgs
    sensor_msgs
    std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS    include 
    LIBRARIES       ${PROJECT_NAME} 
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
                    message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include lcm_types SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -Wconversion")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wconversion -O3 -g -flto")

## Declare C++ executables
add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/iiwa_hardware_interface.hpp
    include/${PROJECT_NAME}/robotiq_3finger_hardware_interface.hpp
    include/${PROJECT_NAME}/wrist_force_torque_hardware_interface.hpp
    include/${PROJECT_NAME}/takktile_hardware_interface.hpp
    include/${PROJECT_NAME}/minimal_arm_wrapper_interface.hpp
    src/${PROJECT_NAME}/iiwa_hardware_interface.cpp
    src/${PROJECT_NAME}/robotiq_3finger_hardware_interface.cpp
    src/${PROJECT_NAME}/wrist_force_torque_hardware_interface.cpp
    src/${PROJECT_NAME}/takktile_hardware_interface.cpp
    src/${PROJECT_NAME}/minimal_arm_wrapper_interface.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_loopback_test_node src/loopback_test_node.cpp)
add_dependencies(${PROJECT_NAME}_loopback_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_loopback_test_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_arm_wrapper_node src/arm_wrapper_node.cpp)
add_dependencies(${PROJECT_NAME}_arm_wrapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_arm_wrapper_node ${PROJECT_NAME} ${catkin_LIBRARIES})
