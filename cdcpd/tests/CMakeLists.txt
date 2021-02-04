add_executable(basic-input-test basic_input.cpp)
target_include_directories(basic-input-test PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(basic-input-test cdcpd)
add_test(NAME basic-input-test COMMAND $<TARGET_FILE:basic-input-test>)

add_executable(mesh_conversion_test mesh_conversion_test.cpp)
target_link_libraries(mesh_conversion_test PUBLIC cdcpd)
target_compile_definitions(mesh_conversion_test PUBLIC CGAL_USE_BASIC_VIEWER)

