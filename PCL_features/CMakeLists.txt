cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(vfh_wrapper)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find source files
file(GLOB SOURCES features.cpp)

add_library(features SHARED ${SOURCES})

target_link_libraries(features ${PCL_LIBRARIES})

SET(COMPILE_FLAGS "-std=c++11 -O3")
add_definitions(${COMPILE_FLAGS})
