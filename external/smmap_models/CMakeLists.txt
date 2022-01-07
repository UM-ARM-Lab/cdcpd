cmake_minimum_required(VERSION 3.0.2)
project(smmap_models)

find_package(catkin REQUIRED smmap_utilities arc_utilities deformable_manipulation_msgs sdf_tools)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smmap_models
)

file(GLOB SRC src/*.cpp)
add_library(${PROJECT_NAME} ${SRC})
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
