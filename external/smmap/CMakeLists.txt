cmake_minimum_required(VERSION 3.1)
project(smmap)

set(CATKIN_PACKAGES
    roscpp
    rospy
    std_msgs
    actionlib
    arc_utilities
    smmap_utilities
    smmap_models
    geometry_msgs
    kinematics_toolbox
    sdf_tools
    deformable_manipulation_msgs
    deformable_manipulation_experiment_params
    tf2_ros
    tf2_eigen
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
find_package(Flann REQUIRED)
set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
set(FLANN_LIBRARIES ${FLANN_LIBRARY})
find_package(SVM REQUIRED)

find_package(Torch REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Messages defined in deformable_manipulation_msgs package

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${PROJECT_NAME}_infrastructure ${PROJECT_NAME}_controllers ${PROJECT_NAME}_transitions ${PROJECT_NAME}_planning
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS         EIGEN3 FLANN SVM
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${SVM_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror=missing-field-initializers -DEIGEN_DONT_PARALLELIZE")

unset(USE_OPEN_MP CACHE)
option(USE_OPEN_MP "Set to ON if we want to use OpenMP" ON)
if(USE_OPEN_MP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -DUSE_OPEN_MP")
    message(STATUS "Enabling OpenMP for SMMAP library")
else(USE_OPEN_MP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas")
    message(WARNING "Disabling OpenMP for SMMAP library")
endif(USE_OPEN_MP)

############# Infrastructure ###################################################

add_custom_target(${PROJECT_NAME}_infrastructure_headers SOURCES
    include/${PROJECT_NAME}/point_reflector.hpp
    include/${PROJECT_NAME}/robot_interface.h
    include/${PROJECT_NAME}/task_specification.h
    include/${PROJECT_NAME}/task_specification_implementions.h
)

add_library(${PROJECT_NAME}_infrastructure
    src/robot_interface.cpp
    src/task_specification.cpp
    src/task_specification_implementations.cpp
)
add_dependencies(${PROJECT_NAME}_infrastructure ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_infrastructure ${catkin_LIBRARIES})

############# Controllers ###########################################################

add_library(${PROJECT_NAME}_controllers
    include/${PROJECT_NAME}/deformable_controller.h
    include/${PROJECT_NAME}/least_squares_stretching_constraint_controller.h
    include/${PROJECT_NAME}/straight_line_controller.h
    include/${PROJECT_NAME}/stretching_constraint_controller.h
    include/${PROJECT_NAME}/least_squares_controller_with_object_avoidance.h
    src/least_squares_controller_with_object_avoidance.cpp
    src/stretching_constraint_controller.cpp
    src/straight_line_controller.cpp
    src/least_squares_stretching_constraint_controller.cpp
)
add_dependencies(${PROJECT_NAME}_controllers ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_controllers ${catkin_LIBRARIES})

############# Transitions ######################################################

add_custom_target(${PROJECT_NAME}_transitions_headers SOURCES
    include/${PROJECT_NAME}/transition_estimation.h
    include/${PROJECT_NAME}/parabola.h
    include/${PROJECT_NAME}/quinlan_rubber_band.h
    include/${PROJECT_NAME}/min_max_transformer.hpp
    include/${PROJECT_NAME}/classifier.h
    include/${PROJECT_NAME}/no_classifier.h
    include/${PROJECT_NAME}/knn_classifier.h
    include/${PROJECT_NAME}/svm_classifier.h
    include/${PROJECT_NAME}/mlp_classifier.h
    include/${PROJECT_NAME}/voxnet_classifier.h
)

add_library(${PROJECT_NAME}_transitions
    src/transition_estimation.cpp
    src/parabola.cpp
    src/quinlan_rubber_band.cpp
    src/classifier.cpp
    src/knn_classifier.cpp
    src/svm_classifier.cpp
    src/mlp_classifier.cpp
    src/voxnet_classifier.cpp
)
add_dependencies(${PROJECT_NAME}_transitions ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_transitions
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_infrastructure
    ${SVM_LIBRARIES}
    ${TORCH_LIBRARIES}
    FLANN::FLANN
)
target_compile_definitions(${PROJECT_NAME}_transitions PUBLIC "-D _DENSE_REP")

############# Planning #########################################################

add_custom_target(${PROJECT_NAME}_planning_headers SOURCES
    include/${PROJECT_NAME}/band_rrt.h
    include/${PROJECT_NAME}/conversions.h
)

add_library(${PROJECT_NAME}_planning
    src/band_rrt.cpp
    src/conversions.cpp
)
add_dependencies(${PROJECT_NAME}_planning ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_planning
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_infrastructure
    ${PROJECT_NAME}_controllers
    ${PROJECT_NAME}_transitions
    FLANN::FLANN
)
set_target_properties(${PROJECT_NAME}_planning PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${FLANN_DEFINITIONS}")
