cmake_minimum_required(VERSION 3.5)
project(cdcpd VERSION 0.1
    DESCRIPTION "Constrained Deformable Coherent Point Drift implementation"
    LANGUAGES CXX)

add_library(libcdcpd SHARED
    src/cdcpd.cpp
)

find_package(PCL 1.8 REQUIRED COMPONENTS common io)

# horrid hack to remove the space at the beginning of the list 
list(REMOVE_AT PCL_DEFINITIONS 0)

target_include_directories(libcdcpd PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    ${PCL_INCLUDE_DIRS})

target_link_libraries(libcdcpd ${PCL_LIBRARIES})
# target_link_directories(libcdcpd PUBLIC ${PCL_LIBRARY_DIRS})
target_compile_definitions(libcdcpd PUBLIC ${PCL_DEFINITIONS})


# target_compile_features(libminisat
#     PUBLIC
#       cxx_attributes
#       cxx_defaulted_functions
#       cxx_deleted_functions
#       cxx_final
# )
# 
# target_compile_definitions(libcdcpd PUBLIC __STDC_LIMIT_MACROS __STDC_FORMAT_MACROS)

### Installation
configure_file(cdcpd-config.cmake.in "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" @ONLY)
install(TARGETS libcdcpd EXPORT cdcpd-targets DESTINATION lib/${PROJECT_NAME})
install(FILES include/cdcpd/cdcpd.h DESTINATION include/cdcpd)
install(FILES "${PROJECT_BINARY_DIR}/cdcpd-config.cmake" DESTINATION lib/${PROJECT_NAME})
install(EXPORT cdcpd-targets NAMESPACE cdcpd:: DESTINATION lib/${PROJECT_NAME})
### End installation

### Testing

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_subdirectory(tests)
endif()

### End Testing
