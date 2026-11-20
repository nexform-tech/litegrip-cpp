// open_close.cpp — drive the jaws open and closed.
//
// Usage: litegrip_example_open_close [channel] [cycles] [speed_mm_s]
//
// This drives real hardware. Clear the jaw travel first; each move presses
// lightly onto a calibrated stop.

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "litegrip/litegrip.hpp"

int main(int argc, char** argv) {
  std::string channel = "can0";
  int cycles = 1;
  double speed_mm_s = 20.0;
  if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 ||
                   std::strcmp(argv[1], "-h") == 0)) {
    std::cout << "usage: " << argv[0]
              << " [channel] [cycles] [speed_mm_s]\n";
    return 0;
  }
  if (argc > 1) channel = argv[1];
  if (argc > 2) cycles = std::atoi(argv[2]);
  if (argc > 3) speed_mm_s = std::atof(argv[3]);
  if (cycles <= 0 || speed_mm_s <= 0.0) {
    std::cerr << "cycles and speed must be positive\n";
    return 2;
  }

  try {
    litegrip::Gripper gripper(channel);
    if (!gripper.config().calibrated) {
      std::cerr << "no usable calibration — run a calibration first\n";
      return 2;
    }
    gripper.connect();
    gripper.enable();
    for (int i = 0; i < cycles; ++i) {
      std::cout << "cycle " << (i + 1) << "/" << cycles << ": open\n";
      if (!gripper.open(speed_mm_s)) {
        std::cerr << "open did not reach the calibrated open limit\n";
        return 1;
      }
      std::cout << "  open: " << gripper.get_gripper_distance() << " mm\n";
      std::cout << "cycle " << (i + 1) << "/" << cycles << ": close\n";
      if (!gripper.close(speed_mm_s)) {
        std::cerr << "close did not reach the calibrated closed limit\n";
        return 1;
      }
      std::cout << "  closed: " << gripper.get_gripper_distance() << " mm\n";
    }
    gripper.disable();
    gripper.disconnect();
    return 0;
  } catch (const litegrip::LiteGripError& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  }
}
