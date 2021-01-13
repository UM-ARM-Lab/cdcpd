cmake_minimum_required(VERSION 2.8.3)
project(smmap_utilities)

set(CATKIN_PACKAGES
        arc_utilities
        geometry_msgs
        kinematics_toolbox
        deformable_manipulation_experiment_params
        deformable_manipulation_msgs
        sdf_tools
        )

## Add our local cmake modules to the list that can be found
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(GUROBI)
find_package(NOMAD)


###################################
## catkin specific configuration ##
###################################
set(CATKIN_EXPORT_LIBRARIES ${PROJECT_NAME})
if (GUROBI_FOUND)
    list(APPEND CATKIN_EXPORT_LIBRARIES ${PROJECT_NAME}_gurobi_solvers)
endif ()
if (NOMAD_FOUND)
    list(APPEND CATKIN_EXPORT_LIBRARIES ${PROJECT_NAME}_nomad_solvers)
endif ()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${CATKIN_EXPORT_LIBRARIES}
        CATKIN_DEPENDS ${CATKIN_PACKAGES}
        DEPENDS EIGEN3
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
        )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto -O3 -Wall -Wextra -Wpedantic -Wconversion -Werror=return-type -Werror=missing-field-initializers")

#############
## Library ##
#############
set(SRC
        src/grippers.cpp
        src/neighbours.cpp
        src/visualization_tools.cpp
        src/multiarm_bandits.cpp
        src/ros_communication_helpers.cpp
        src/trajectory.cpp)
add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

if (NOMAD_FOUND)
    add_library(${PROJECT_NAME}_nomad_solvers src/nomad_solvers.cpp)
    target_link_libraries(${PROJECT_NAME}_nomad_solvers ${catkin_LIBRARIES} ${NOMAD_LIBRARIES})
    add_dependencies(${PROJECT_NAME}_nomad_solvers ${catkin_EXPORTED_TARGETS})
    target_include_directories(${PROJECT_NAME}_nomad_solvers PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${NOMAD_INCLUDE_DIRS})
endif ()

if (GUROBI_FOUND)
    add_library(${PROJECT_NAME}_gurobi_solvers src/gurobi_solvers.cpp)
    target_link_libraries(${PROJECT_NAME}_gurobi_solvers ${catkin_LIBRARIES} ${GUROBI_LIBRARIES})
    add_dependencies(${PROJECT_NAME}_gurobi_solvers ${catkin_EXPORTED_TARGETS})
    target_include_directories(${PROJECT_NAME}_gurobi_solvers PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GUROBI_INCLUDE_DIRS})
endif ()
