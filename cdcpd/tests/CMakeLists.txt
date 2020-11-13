add_executable(basic-input-test basic_input.cpp)
target_include_directories(basic-input-test PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(basic-input-test cdcpd)
add_test(NAME basic-input-test COMMAND $<TARGET_FILE:basic-input-test>)
