// version_info.cpp — smallest possible consumer of the SDK.

#include <iostream>

#include "litegrip/version.hpp"

int main() {
  std::cout << "litegrip_cpp " << litegrip::version() << "\n";
  return 0;
}
