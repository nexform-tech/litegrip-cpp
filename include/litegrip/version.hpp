// litegrip/version.hpp — build-time version of the litegrip_cpp SDK.

#pragma once

// Must stay in sync with CMakeLists.txt's project(... VERSION ...).
#define LITEGRIP_CPP_VERSION_MAJOR 0
#define LITEGRIP_CPP_VERSION_MINOR 1
#define LITEGRIP_CPP_VERSION_PATCH 0
#define LITEGRIP_CPP_VERSION_STRING "0.1.0"

namespace litegrip {

/// Version as a "MAJOR.MINOR.PATCH" string.
const char* version() noexcept;

int version_major() noexcept;
int version_minor() noexcept;
int version_patch() noexcept;

}  // namespace litegrip
