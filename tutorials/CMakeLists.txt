add_executable(simple-rbt-tutorial-config config_main.cpp calibration.cpp)
target_link_libraries(simple-rbt-tutorial-config PRIVATE simple_rbt_ipc)
set_target_properties(simple-rbt-tutorial-config PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
