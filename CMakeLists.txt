cmake_minimum_required(VERSION 3.5)
project(cdcpd VERSION 0.1
    DESCRIPTION "Constrained Deformable Coherent Point Drift implementation"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(libcdcpd SHARED
    src/cdcpd.cpp
    src/optimizer.cpp
    src/past_template_matcher.cpp
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

find_package(PCL 1.8 REQUIRED COMPONENTS common io filters visualization)
# TODO Components?
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GUROBI REQUIRED)

# horrid hack to remove the space at the beginning of the list 
list(REMOVE_AT PCL_DEFINITIONS 0)

target_include_directories(libcdcpd PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(libcdcpd SYSTEM PUBLIC 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
target_include_directories(libcdcpd SYSTEM PRIVATE 
    ${GUROBI_INCLUDE_DIRS})

target_link_libraries(libcdcpd PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen)
target_link_libraries(libcdcpd PRIVATE
    ${GUROBI_LIBRARIES})
target_compile_definitions(libcdcpd PUBLIC ${PCL_DEFINITIONS})

add_library(cdcpd::libcdcpd ALIAS libcdcpd)


### Installation
configure_file(cdcpd-config.cmake.in "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" @ONLY)
install(TARGETS libcdcpd EXPORT cdcpd-targets DESTINATION lib/${PROJECT_NAME})
install(FILES include/cdcpd/cdcpd.h DESTINATION include/cdcpd)
install(FILES "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" DESTINATION lib/${PROJECT_NAME})

set_target_properties(libcdcpd
    PROPERTIES
        OUTPUT_NAME "cdcpd"
)
### 
### ### Installation
### install(TARGETS libcdcpd 
###     EXPORT cdcpd-targets 
###     LIBRARY DESTINATION lib/${PROJECT_NAME}
###     INCLUDES DESTINATION include/${PROJECT_NAME})
### # install(FILES include/cdcpd/cdcpd.h include/cdcpd/optimizer.h DESTINATION include/cdcpd)
### # install(FILES "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" DESTINATION lib/${PROJECT_NAME})
### include(CMakePackageConfigHelpers)
### write_basic_package_version_file(
###     cdcpd-config-version.cmake
###     VERSION ${PACKAGE_VERSION}
###     COMPATIBILITY AnyNewerVersion
###     )
install(EXPORT cdcpd-targets NAMESPACE cdcpd:: DESTINATION lib/${PROJECT_NAME})
### configure_file(cdcpd-config.cmake.in cdcpd-config.cmake @ONLY)
### install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cdcpd-config.cmake"
###               "${CMAKE_CURRENT_BINARY_DIR}/cdcpd-config-version.cmake"
###               DESTINATION lib/cmake/${PROJECT_NAME}
###         )
### End installation

### Testing

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_subdirectory(tests)
endif()

### End Testing
