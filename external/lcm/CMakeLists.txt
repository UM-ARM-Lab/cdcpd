cmake_minimum_required(VERSION 2.8.3)
project(lcm)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
# Add FindFlann.cmake to the path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
find_package(GLib REQUIRED)
set(GLib_LIBRARIES ${GLib_LIBRARY})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} DEPENDS GLib)

###########
## Build ##
###########

message(AUTHOR_WARNING "------- Deprecated definitions disabled for LCM build")

add_definitions(-Wno-deprecated-declarations)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${GLib_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME} include/${PROJECT_NAME}/dbg.h
                            include/${PROJECT_NAME}/lcm.h
                            include/${PROJECT_NAME}/lcm_internal.h
                            include/${PROJECT_NAME}/lcm_coretypes.h
                            include/${PROJECT_NAME}/ringbuffer.h
                            include/${PROJECT_NAME}/udpm_util.h
                            include/${PROJECT_NAME}/eventlog.h
                            include/${PROJECT_NAME}/ioutils.h
                            include/${PROJECT_NAME}/lcm-cpp.hpp
                            include/${PROJECT_NAME}/lcm-cpp-impl.hpp
                            include/${PROJECT_NAME}/lcmtypes/channel_port_map_update_t.h
                            include/${PROJECT_NAME}/lcmtypes/channel_to_port_t.h
                            src/${PROJECT_NAME}/lcm.c
                            src/${PROJECT_NAME}/lcm_memq.c
                            src/${PROJECT_NAME}/lcm_file.c
                            src/${PROJECT_NAME}/ringbuffer.c
                            src/${PROJECT_NAME}/udpm_util.c
                            src/${PROJECT_NAME}/eventlog.c
                            src/${PROJECT_NAME}/lcm_mpudpm.c
                            src/${PROJECT_NAME}/lcm_udpm.c
                            src/${PROJECT_NAME}/lcm_udp.c
                            src/${PROJECT_NAME}/lcm_tcpq.c
                            src/${PROJECT_NAME}/lcmtypes/channel_port_map_update_t.c
                            src/${PROJECT_NAME}/lcmtypes/channel_to_port_t.c)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GLib_LIBRARIES})
