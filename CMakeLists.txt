cmake_minimum_required(VERSION 3.1)
project(cdcpd_ros)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# https://blog.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "Release")
#set(default_build_type "RelWithDebInfo")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(WARNING "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  genmsg
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  std_msgs
  pcl_ros
  arc_utilities
  message_generation
  cdcpd
)
find_package(CGAL REQUIRED COMPONENTS Core Qt5)
find_package(Qt5 QUIET COMPONENTS Qt5 Core Xml Script OpenGL Svg)

add_message_files(
  FILES
  Float32MultiArrayStamped.msg
  CartesianControlModeLimits.msg
  CartesianImpedanceParameters.msg
  CartesianPathExecutionParameters.msg
  CartesianValueQuantity.msg
  ControlMode.msg
  ControlModeParameters.msg
  GraspStatus.msg
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
)

generate_messages(
  DEPENDENCIES
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  std_msgs)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cdcpd_ros
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

# Node for live data
# add_executable(${PROJECT_NAME}
#   src/${PROJECT_NAME}/kinect_sub.cpp
#   src/cdcpd_node.cpp
# )
# set_target_properties(${PROJECT_NAME} PROPERTIES
#     OUTPUT_NAME node PREFIX ""
#     CXX_STANDARD 17
#     CXX_STANDARD_REQUIRED YES)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME} PRIVATE
#   ${catkin_LIBRARIES}
#   cdcpd::cdcpd
# )
#target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -Wextra -Wconversion -Wshadow)

# Node for bagged data
add_executable(${PROJECT_NAME}_bagfile
  src/cdcpd_bagfile.cpp
)
set_target_properties(${PROJECT_NAME}_bagfile PROPERTIES
    OUTPUT_NAME bagfile PREFIX ""
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES)
add_dependencies(${PROJECT_NAME}_bagfile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_bagfile PRIVATE
  ${catkin_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Qt5
  Qt5::Gui
)
target_compile_options(${PROJECT_NAME}_bagfile PRIVATE -O3 -Wall -Wextra -Wconversion -Werror -Wshadow)
