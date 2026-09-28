// litegrip_cpp — ROS-agnostic C++ SDK for the LiteGrip adaptive two-finger gripper.

#include "litegrip/version.hpp"

namespace litegrip {

const char* version() noexcept { return LITEGRIP_CPP_VERSION_STRING; }

int version_major() noexcept { return LITEGRIP_CPP_VERSION_MAJOR; }
int version_minor() noexcept { return LITEGRIP_CPP_VERSION_MINOR; }
int version_patch() noexcept { return LITEGRIP_CPP_VERSION_PATCH; }

}  // namespace litegrip
