cmake_minimum_required(VERSION 2.8.3)
project(deformable_manipulation_experiment_params)

set(CATKIN_PACKAGES
  arc_utilities
  roscpp
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

################################################
## Declare ROS messages, services and actions ##
################################################

## Messages defined in deformable_manipulation_msgs package

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
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${CATKIN_PACKAGES}
    DEPENDS EIGEN3
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O3 -flto -Wall -Wextra -Wpedantic -Wconversion -Werror")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

## Library file for the whole project
FILE(GLOB hpp_SRC "include/${PROJECT_NAME}/*.hpp")
FILE(GLOB h_SRC "include/${PROJECT_NAME}/*.h")
FILE(GLOB cpp_SRC "src/*.cpp")

add_custom_target(${PROJECT_NAME}_headers SOURCES ${hpp_SRC} ${h_SRC})
add_library(${PROJECT_NAME} ${cpp_SRC})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
# install(TARGETS deformable_manipulation_experiment_params deformable_manipulation_experiment_params_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_deformable_manipulation_experiment_params.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
