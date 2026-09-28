// constants.cpp — error-code descriptions.
//
// The Python original's messages were Chinese; the C++ SDK uses English (see
// PLAN-litegrip-cpp.md). The *set* of codes and their meanings is unchanged.

#include "litegrip/constants.hpp"

#include <cstdio>

namespace litegrip {

std::string describe_error(int code) {
  switch (code) {
    case 0x0:
      return "disabled";
    case 0x1:
      return "enabled";
    case 0x8:
      return "overvoltage fault (OV)";
    case 0x9:
      return "undervoltage fault (UV)";
    case 0xA:
      return "overcurrent fault (OC)";
    case 0xB:
      return "MOS over-temperature fault";
    case 0xC:
      return "coil over-temperature fault";
    case 0xD:
      return "communication loss (CAN timeout)";
    case 0xE:
      return "overload fault";
    default:
      break;
  }
  char buf[48];
  std::snprintf(buf, sizeof(buf), "unknown error (0x%X)", code);
  return buf;
}

}  // namespace litegrip
