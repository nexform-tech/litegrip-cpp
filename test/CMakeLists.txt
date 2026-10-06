# Tests use no third-party framework (the library is zero-dependency, and so is
# its test setup): each test file is a standalone executable that returns
# non-zero on failure and is registered with CTest.

# T1: version smoke test. T2–T4 add the protocol / model / safety parity tests
# (golden vectors lifted from litegrip_driver/tests/test_protocol.py).
set(LITEGRIP_CPP_TESTS
  test_version.cpp
  test_headers.cpp
  test_protocol.cpp
  test_motor.cpp
  test_golden.cpp
  test_transport.cpp
  test_json.cpp
  test_calibration.cpp
  test_bus.cpp
  test_safety.cpp
  test_gripper.cpp
  test_motion.cpp
  test_actions.cpp
  test_control_loop.cpp
)

foreach(test_src IN LISTS LITEGRIP_CPP_TESTS)
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_link_libraries(${test_name} PRIVATE litegrip_cpp::litegrip_cpp)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# test_calibration verifies the SHIPPED factory_calibration.json, so it needs
# the source data directory. Test-only: the library itself resolves its data
# location at run time and never bakes in a machine-dependent path.
target_compile_definitions(test_calibration PRIVATE
  LITEGRIP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../calibration")
target_compile_definitions(test_gripper PRIVATE
  LITEGRIP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../calibration")

# test_safety loads the shipped versioned baselines. The library discovers its
# data directory at run time (no baked-in path), so point it at the source tree.
set_tests_properties(test_safety PROPERTIES
  ENVIRONMENT "LITEGRIP_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/..")
set_tests_properties(test_control_loop PROPERTIES
  ENVIRONMENT "LITEGRIP_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/..")
