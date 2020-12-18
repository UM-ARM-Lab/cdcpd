cmake_minimum_required(VERSION 2.8.3)
project(arc_utilities)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
<<<<<<< HEAD
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs 
	OPTIONAL_COMPONENTS pybind11_catkin rosconsole)
=======
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS filesystem)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(DIRECTORY msg FILES )

## Generate services in the 'srv' folder
# add_service_files(DIRECTORY srv FILES )

## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES geometry_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs DEPENDS Eigen3 Boost)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wconversion -Werror -Wshadow")

add_custom_target(${PROJECT_NAME}_headers SOURCES
<<<<<<< HEAD
        include/${PROJECT_NAME}/abb_irb1600_145_fk_fast.hpp
        include/${PROJECT_NAME}/aligned_eigen_types.hpp
        include/${PROJECT_NAME}/arc_exceptions.hpp
        include/${PROJECT_NAME}/arc_helpers.hpp
        include/${PROJECT_NAME}/base64_helpers.hpp
        include/${PROJECT_NAME}/dijkstras.hpp
        include/${PROJECT_NAME}/dynamic_spatial_hashed_voxel_grid.hpp
        include/${PROJECT_NAME}/eigen_helpers_conversions.hpp
        include/${PROJECT_NAME}/eigen_helpers.hpp
        include/${PROJECT_NAME}/eigen_ros_conversions.hpp
        include/${PROJECT_NAME}/eigen_std_conversions.hpp
        include/${PROJECT_NAME}/eigen_transforms.hpp
        include/${PROJECT_NAME}/eigen_typedefs.hpp
        include/${PROJECT_NAME}/enumerate.h
        include/${PROJECT_NAME}/filesystem.hpp
        include/${PROJECT_NAME}/first_order_deformation.h
        include/${PROJECT_NAME}/geometry_msgs_builders.hpp
        include/${PROJECT_NAME}/get_neighbours.hpp
        include/${PROJECT_NAME}/graph.hpp
        include/${PROJECT_NAME}/hash.hpp
        include/${PROJECT_NAME}/iiwa_14_fk_fast.hpp
        include/${PROJECT_NAME}/iiwa_7_fk_fast.hpp
        include/${PROJECT_NAME}/listener.hpp
        include/${PROJECT_NAME}/log.hpp
        include/${PROJECT_NAME}/math_helpers.hpp
        include/${PROJECT_NAME}/maybe.hpp
        include/${PROJECT_NAME}/moveit_ostream_operators.hpp
        include/${PROJECT_NAME}/moveit_pose_type.hpp
        include/${PROJECT_NAME}/ostream_operators.hpp
        include/${PROJECT_NAME}/path_utils.hpp
        include/${PROJECT_NAME}/pretty_print.hpp
        include/${PROJECT_NAME}/ros_helpers.hpp
        include/${PROJECT_NAME}/serialization_eigen.hpp
        include/${PROJECT_NAME}/serialization.hpp
        include/${PROJECT_NAME}/serialization_ros.hpp
        include/${PROJECT_NAME}/simple_astar_planner.hpp
        include/${PROJECT_NAME}/simple_dtw.hpp
        include/${PROJECT_NAME}/simple_hausdorff_distance.hpp
        include/${PROJECT_NAME}/simple_hierarchical_clustering.hpp
        include/${PROJECT_NAME}/simple_kmeans_clustering.hpp
        include/${PROJECT_NAME}/simple_linked_robot_model.hpp
        include/${PROJECT_NAME}/simple_prm_planner.hpp
        include/${PROJECT_NAME}/simple_robot_model_interface.hpp
        include/${PROJECT_NAME}/simple_robot_models.hpp
        include/${PROJECT_NAME}/simple_rrt_planner.hpp
        include/${PROJECT_NAME}/simple_se2_robot_model.hpp
        include/${PROJECT_NAME}/simple_se3_robot_model.hpp
        include/${PROJECT_NAME}/stl_wrappers.hpp
        include/${PROJECT_NAME}/thin_plate_spline.hpp
        include/${PROJECT_NAME}/time_optimal_trajectory_parametrization.hpp
        include/${PROJECT_NAME}/timing.hpp
        include/${PROJECT_NAME}/trajectory_utils.h
        include/${PROJECT_NAME}/ur_10_fk_fast.hpp
        include/${PROJECT_NAME}/vector_math.hpp
        include/${PROJECT_NAME}/voxel_grid.hpp
        include/${PROJECT_NAME}/zlib_helpers.hpp
        )

# Utility library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/zlib_helpers.cpp
        src/${PROJECT_NAME}/base64_helpers.cpp
        src/${PROJECT_NAME}/timing.cpp
        src/${PROJECT_NAME}/first_order_deformation.cpp
        src/${PROJECT_NAME}/time_optimal_trajectory_parametrization.cpp
        )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} z)

if (pybind11_catkin_FOUND)
  pybind_add_module(roscpp_initializer src/roscpp_initializer.cpp)
  target_include_directories(roscpp_initializer SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(roscpp_initializer PUBLIC ${catkin_LIBRARIES})
endif ()

=======
    include/${PROJECT_NAME}/log.hpp
    include/${PROJECT_NAME}/maybe.hpp
    include/${PROJECT_NAME}/ros_helpers.hpp
    include/${PROJECT_NAME}/abb_irb1600_145_fk_fast.hpp
    include/${PROJECT_NAME}/iiwa_7_fk_fast.hpp
    include/${PROJECT_NAME}/iiwa_14_fk_fast.hpp
    include/${PROJECT_NAME}/ur_10_fk_fast.hpp
    include/${PROJECT_NAME}/arc_exceptions.hpp
    include/${PROJECT_NAME}/arc_helpers.hpp
    include/${PROJECT_NAME}/pretty_print.hpp
    include/${PROJECT_NAME}/zlib_helpers.hpp
    include/${PROJECT_NAME}/base64_helpers.hpp
    include/${PROJECT_NAME}/voxel_grid.hpp
    include/${PROJECT_NAME}/dynamic_spatial_hashed_voxel_grid.hpp
    include/${PROJECT_NAME}/aligned_eigen_types.hpp
    include/${PROJECT_NAME}/eigen_typedefs.hpp
    include/${PROJECT_NAME}/eigen_helpers.hpp
    include/${PROJECT_NAME}/eigen_helpers_conversions.hpp
    include/${PROJECT_NAME}/simple_rrt_planner.hpp
    include/${PROJECT_NAME}/simple_astar_planner.hpp
    include/${PROJECT_NAME}/simple_prm_planner.hpp
    include/${PROJECT_NAME}/simple_kmeans_clustering.hpp
    include/${PROJECT_NAME}/simple_hierarchical_clustering.hpp
    include/${PROJECT_NAME}/simple_hausdorff_distance.hpp
    include/${PROJECT_NAME}/simple_dtw.hpp
    include/${PROJECT_NAME}/timing.hpp
    include/${PROJECT_NAME}/dijkstras.hpp
    include/${PROJECT_NAME}/path_utils.hpp
    include/${PROJECT_NAME}/first_order_deformation.h
    include/${PROJECT_NAME}/get_neighbours.hpp
    include/${PROJECT_NAME}/serialization.hpp
    include/${PROJECT_NAME}/serialization_eigen.hpp
    include/${PROJECT_NAME}/serialization_ros.hpp
    include/${PROJECT_NAME}/filesystem.hpp
    include/${PROJECT_NAME}/simple_se2_robot_model.hpp
    include/${PROJECT_NAME}/simple_se3_robot_model.hpp
    include/${PROJECT_NAME}/simple_linked_robot_model.hpp
    include/${PROJECT_NAME}/simple_robot_models.hpp
    include/${PROJECT_NAME}/simple_robot_model_interface.hpp
    include/${PROJECT_NAME}/time_optimal_trajectory_parametrization.hpp
    include/${PROJECT_NAME}/thin_plate_spline.hpp
    include/${PROJECT_NAME}/math_helpers.hpp
    include/${PROJECT_NAME}/vector_math.hpp
    include/${PROJECT_NAME}/hash.hpp
    include/${PROJECT_NAME}/geometry_msgs_builders.hpp
    )

# Utility library
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}/zlib_helpers.cpp
    src/${PROJECT_NAME}/base64_helpers.cpp
    src/${PROJECT_NAME}/timing.cpp
    src/${PROJECT_NAME}/first_order_deformation.cpp
    src/${PROJECT_NAME}/time_optimal_trajectory_parametrization.cpp
    )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} z)

>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
###########
## Tests ##
###########

if (${CATKIN_ENABLE_TESTING})
  find_package(GTest)

  # Simple test node for simple_hierarchical_clustering
  add_executable(test_hierarchical_clustering src/test_hierarchical_clustering.cpp)
  add_dependencies(test_hierarchical_clustering ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_hierarchical_clustering ${PROJECT_NAME} ${catkin_LIBRARIES})

  # Simple test node for utility functions
  add_executable(test_arc_utilities src/test_arc_utilities.cpp)
  add_dependencies(test_arc_utilities ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_arc_utilities ${PROJECT_NAME} ${catkin_LIBRARIES})

  # Simple test node for Eigen3 math
  add_executable(test_eigen_math src/test_eigen_math.cpp)
  add_dependencies(test_eigen_math ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_eigen_math ${PROJECT_NAME} ${catkin_LIBRARIES})

  # Simple test node for averaging math
  add_executable(test_averaging src/test_averaging.cpp)
  add_dependencies(test_averaging ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_averaging ${PROJECT_NAME} ${catkin_LIBRARIES})

  # Simple test node for DistanceToLine function
  add_executable(test_closest_point src/test_closest_point.cpp)
  add_dependencies(test_closest_point ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_closest_point ${PROJECT_NAME} ${catkin_LIBRARIES})

  # Simple test for SO(3) distance functions (to be expanded to all distance functions later)
  add_executable(test_distance_functions src/test_distance_functions.cpp)
  add_dependencies(test_distance_functions ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_distance_functions ${PROJECT_NAME} ${catkin_LIBRARIES})

  # Simple test ROS (de)Serialization functions
  add_executable(test_ros_serialization src/test_ros_serialization.cpp)
  add_dependencies(test_ros_serialization ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_ros_serialization ${PROJECT_NAME} ${catkin_LIBRARIES})

  # Simple thin plate sline tests in 2D and 3D
  add_executable(test_thin_plate_spline src/test_thin_plate_spline.cpp)
  add_dependencies(test_thin_plate_spline ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_thin_plate_spline ${PROJECT_NAME} ${catkin_LIBRARIES})

  # To build tests, install google's testing framework
  # https://github.com/google/googletest/


  catkin_add_gtest(test_timings tests/timings_tests.cpp)
  target_link_libraries(test_timings ${PROJECT_NAME})
  catkin_add_gtest(test_eigen_serialization tests/test_eigen_serialization.cpp)
  target_link_libraries(test_eigen_serialization ${PROJECT_NAME})
  catkin_add_gtest(test_graph tests/test_graph.cpp)
  target_link_libraries(test_graph ${catkin_LIBRARIES})
  catkin_add_gtest(test_dijkstras tests/test_dijkstras.cpp)
  target_link_libraries(test_dijkstras ${catkin_LIBRARIES})
  catkin_add_gtest(test_math_helpers tests/test_math_helpers.cpp)
  target_link_libraries(test_math_helpers ${catkin_LIBRARIES})
endif ()


#############
## Install ##
#############

## Mark library for installation
install(TARGETS ${PROJECT_NAME}
<<<<<<< HEAD
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
=======
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    )
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
