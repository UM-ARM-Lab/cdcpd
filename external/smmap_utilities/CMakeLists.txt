cmake_minimum_required(VERSION 2.8.3)
project(smmap_utilities)

set(CATKIN_PACKAGES
    arc_utilities
    geometry_msgs
    kinematics_toolbox
    deformable_manipulation_experiment_params
    deformable_manipulation_msgs
)

## Add our local cmake modules to the list that can be found
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(GUROBI REQUIRED)
find_package(NOMAD REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS         EIGEN3
                    GUROBI
                    NOMAD
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GUROBI_INCLUDE_DIRS}
    ${NOMAD_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -flto -O3 -g -Wall -Wextra -Wpedantic -Wconversion -Werror=return-type -Werror=missing-field-initializers")

#############
## Library ##
#############

## Library file for the whole project
FILE(GLOB hpp_SRC "include/${PROJECT_NAME}/*.hpp")
FILE(GLOB h_SRC "include/${PROJECT_NAME}/*.h")
FILE(GLOB cpp_SRC "src/*.cpp")

add_custom_target(${PROJECT_NAME}_headers SOURCES ${hpp_SRC} ${h_SRC})
add_library(${PROJECT_NAME} ${cpp_SRC})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GUROBI_LIBRARIES} ${NOMAD_LIBRARIES})

