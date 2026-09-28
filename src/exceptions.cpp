// exceptions.cpp — LiteGripError::render.

#include "litegrip/exceptions.hpp"

#include <cstdio>

namespace litegrip {

std::string LiteGripError::render(const std::string& message, int error_code) {
  if (error_code == kNoErrorCode) {
    return message;
  }
  // Matches the Python SDK's __str__: "<message> [0xNNNN]".
  char suffix[16];
  std::snprintf(suffix, sizeof(suffix), " [0x%04X]", error_code);
  return message + suffix;
}

}  // namespace litegrip
