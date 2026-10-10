cmake_minimum_required(VERSION 3.16)
project(litegrip_cpp VERSION 0.1.0 LANGUAGES CXX)

# litegrip_cpp — C++ SDK for the LiteGrip adaptive two-finger gripper.
#
# Design constraints (see ../PLAN-litegrip-cpp.md):
#   * C++17, zero third-party dependencies (stdlib + Linux SocketCAN only).
#   * No framework dependencies — usable from any plain C++ program.
#   * Installable into a normal prefix (find_package + pkg-config).

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(LITEGRIP_CPP_IS_TOP_LEVEL ON)
else()
  set(LITEGRIP_CPP_IS_TOP_LEVEL OFF)
endif()

option(LITEGRIP_CPP_BUILD_TESTS "Build litegrip_cpp unit tests" ${LITEGRIP_CPP_IS_TOP_LEVEL})
option(LITEGRIP_CPP_BUILD_EXAMPLES "Build litegrip_cpp examples" ${LITEGRIP_CPP_IS_TOP_LEVEL})

# Must come before any $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> use:
# the variable is only defined once GNUInstallDirs is included, and an empty
# value silently drops the include directory from the exported target.
include(GNUInstallDirs)

find_package(Threads REQUIRED)

# ── library ───────────────────────────────────────────────────────────────
# T1 froze the API surface (headers). T2 adds the can/* layer; bus/LiteGrip
# land in T3 and safety/ControlLoop in T4.
add_library(litegrip_cpp
  src/version.cpp
  src/exceptions.cpp
  src/constants.cpp
  src/can/protocol.cpp
  src/can/motor.cpp
  src/can/transport.cpp
  src/can/controller.cpp
  src/json.cpp
  src/calibration.cpp
  src/hold_policy.cpp
  src/bus.cpp
  src/safety.cpp
  src/gripper.cpp
  src/motion.cpp
  src/probe.cpp
  src/control_loop.cpp
)
add_library(litegrip_cpp::litegrip_cpp ALIAS litegrip_cpp)

target_compile_features(litegrip_cpp PUBLIC cxx_std_17)
target_link_libraries(litegrip_cpp PUBLIC Threads::Threads)

target_include_directories(litegrip_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(litegrip_cpp PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(litegrip_cpp PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ── install ───────────────────────────────────────────────────────────────
include(CMakePackageConfigHelpers)

install(TARGETS litegrip_cpp
  EXPORT litegrip_cppTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Calibration / safety data (factory_calibration.json, safety_limits_*.json).
# These are *data*: a deployment may override them, but the factory defaults
# must ship so load_calibration()'s read-only fallback works out of the box.
# The path is mirrored by litegrip_cpp_DATA_DIR in the CMake package config.
install(DIRECTORY calibration/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/litegrip_cpp/calibration
)

install(EXPORT litegrip_cppTargets
  FILE litegrip_cppTargets.cmake
  NAMESPACE litegrip_cpp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/litegrip_cpp
)

configure_package_config_file(
  cmake/litegrip_cpp-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/litegrip_cpp-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/litegrip_cpp
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/litegrip_cpp-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/litegrip_cpp-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/litegrip_cpp-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/litegrip_cpp
)

# ── pkg-config ────────────────────────────────────────────────────────────
configure_file(cmake/litegrip_cpp.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/litegrip_cpp.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/litegrip_cpp.pc
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)

# ── tests / examples (no third-party test framework: plain assert-based) ──
if(LITEGRIP_CPP_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(LITEGRIP_CPP_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
