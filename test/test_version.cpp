// test_version.cpp — smoke test that the header/library wiring works at all.
// Deliberately dependency-free (no gtest): returns 1 on failure.

#include <cstring>
#include <iostream>

#include "litegrip/version.hpp"

int main() {
  if (std::strlen(litegrip::version()) == 0) {
    std::cerr << "FAIL: version() returned an empty string\n";
    return 1;
  }
  if (litegrip::version_major() != LITEGRIP_CPP_VERSION_MAJOR) {
    std::cerr << "FAIL: version_major() disagrees with the macro\n";
    return 1;
  }
  std::cout << "litegrip_cpp " << litegrip::version() << " OK\n";
  return 0;
}
