cmake_minimum_required(VERSION 3.9)
project(cdcpd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_PACKAGES
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# TODO Components?
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GUROBI REQUIRED)
find_package(FAISS REQUIRED)
find_package(Fgt REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters visualization features kdtree)

# horrid hack to remove the space at the beginning of the list 
list(REMOVE_AT PCL_DEFINITIONS 0)

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       cdcpd
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS
)

include_directories(include PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
include_directories(include SYSTEM PUBLIC 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
include_directories(include SYSTEM PRIVATE 
    ${catkin_INCLUDE_DIRS}
    ${GUROBI_INCLUDE_DIRS}
    ${FAISS_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS})

### The library itself
add_library(cdcpd SHARED
    src/cdcpd.cpp
    src/optimizer.cpp
    src/past_template_matcher.cpp
)

target_link_libraries(cdcpd PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen)
target_link_libraries(cdcpd PRIVATE
    ${catkin_LIBRARIES}
    ${GUROBI_LIBRARIES}
    ${FAISS_LIBRARY}
    ${CERES_LIBRARIES}
    Fgt::Library-C++
	)
add_dependencies(cdcpd ${catkin_EXPORTED_TARGETS})
target_compile_definitions(cdcpd PUBLIC ${PCL_DEFINITIONS})
target_compile_options(cdcpd PRIVATE -Wall -Wextra -Wshadow -g)

# Set to private to not force downstream libraries to link against OpenMP
# https://cliutils.gitlab.io/modern-cmake/chapters/packages/OpenMP.html
# https://stackoverflow.com/a/51448364
find_package(OpenMP REQUIRED)
target_link_libraries(cdcpd PRIVATE OpenMP::OpenMP_CXX)


add_library(cdcpd::cdcpd ALIAS cdcpd)

### Installation
configure_file(cdcpd-config.cmake.in "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" @ONLY)
install(TARGETS cdcpd EXPORT cdcpd-targets DESTINATION lib/${PROJECT_NAME})
install(FILES
	include/cdcpd/cdcpd.h 
	include/cdcpd/optimizer.h
	include/cdcpd/past_template_matcher.h
	DESTINATION 
	include/cdcpd)
install(FILES "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" DESTINATION lib/${PROJECT_NAME})

install(EXPORT cdcpd-targets NAMESPACE cdcpd:: DESTINATION lib/${PROJECT_NAME})

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()
