cmake_minimum_required(VERSION 3.9)
project(cdcpd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_PACKAGES
        geometric_shapes
        geometry_msgs
        image_geometry
        image_transport
        moveit_msgs
        moveit_ros_planning
        moveit_visual_tools
        pybind11_catkin
        pyrosmsg
        sdf_tools
        sensor_msgs
        smmap_models
        smmap_utilities
        std_msgs
        )

# https://blog.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "Release")
#set(default_build_type "RelWithDebInfo")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(WARNING "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# TODO Components?
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GUROBI REQUIRED)
find_package(FAISS REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core Qt5)
find_package(Qt5 QUIET COMPONENTS Qt5 Core Xml Script OpenGL Svg)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters visualization features kdtree)

# horrid hack to remove the space at the beginning of the list 
list(REMOVE_AT PCL_DEFINITIONS 0)

catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES cdcpd
        CATKIN_DEPENDS ${CATKIN_PACKAGES}
        DEPENDS OpenCV Eigen3 GUROBI FIASS Qt5 PCL # NOTE: adding CGAL errors, but it's working so whatever
)

### The library itself
add_library(cdcpd SHARED
        src/cdcpd.cpp
        src/optimizer.cpp
        src/obs_util.cpp
        src/past_template_matcher.cpp
        )
target_include_directories(cdcpd PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_include_directories(cdcpd SYSTEM PUBLIC
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${GUROBI_INCLUDE_DIRS}
        ${FAISS_INCLUDE_DIRS})
target_link_libraries(cdcpd PUBLIC
        lapack
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        ${catkin_LIBRARIES}
        ${GUROBI_LIBRARIES}
        ${FAISS_LIBRARY}
        ${OpenCV_LIBS}
        CGAL::CGAL
        CGAL::CGAL_Qt5
        )

add_dependencies(cdcpd ${catkin_EXPORTED_TARGETS})
target_compile_definitions(cdcpd PUBLIC ${PCL_DEFINITIONS})
target_compile_options(cdcpd PRIVATE -Wall -Wextra)

# Set to private to not force downstream libraries to link against OpenMP
# https://cliutils.gitlab.io/modern-cmake/chapters/packages/OpenMP.html
# https://stackoverflow.com/a/51448364
find_package(OpenMP REQUIRED)
target_link_libraries(cdcpd PRIVATE OpenMP::OpenMP_CXX)

add_library(cdcpd::cdcpd ALIAS cdcpd)

if (pybind11_catkin_FOUND)
    pybind_add_module(pycdcpd src/bindings.cpp)
    target_link_libraries(pycdcpd PUBLIC cdcpd)
endif()

add_subdirectory(tests)

### Installation
configure_file(cdcpd-config.cmake.in "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" @ONLY)
install(TARGETS cdcpd EXPORT cdcpd-targets DESTINATION lib/${PROJECT_NAME})
install(FILES
        include/cdcpd/cdcpd.h
        include/cdcpd/optimizer.h
        include/cdcpd/obs_util.h
        include/cdcpd/past_template_matcher.h
        DESTINATION
        include/cdcpd)
install(FILES "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" DESTINATION lib/${PROJECT_NAME})

install(EXPORT cdcpd-targets NAMESPACE cdcpd:: DESTINATION lib/${PROJECT_NAME})

add_executable(cdcpd_node
        src/cdcpd_ros/kinect_sub.cpp
        src/cdcpd_node.cpp
        )
add_dependencies(cdcpd_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cdcpd_node PRIVATE cdcpd ${catkin_LIBRARIES})