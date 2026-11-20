// move_to.cpp — move to an absolute opening (mm) or motor angle (rad).
//
// Usage:
//   litegrip_example_move_to [channel] [distance_mm] [speed_mm_s]
//   litegrip_example_move_to [channel] --angle RAD [speed_mm_s]
//
// Clear the jaw travel first. The target is clamped into the calibrated travel.

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>

#include "litegrip/litegrip.hpp"

int main(int argc, char** argv) {
  std::string channel = "can0";
  std::optional<double> distance_mm = 40.0;
  std::optional<double> angle_rad;
  double speed_mm_s = 30.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "usage: " << argv[0]
                << " [channel] [distance_mm] [speed_mm_s]\n"
                   "       " << argv[0]
                << " [channel] --angle RAD [speed_mm_s]\n";
      return 0;
    }
    if (arg == "--angle") {
      if (i + 1 >= argc) {
        std::cerr << "--angle needs a value\n";
        return 2;
      }
      angle_rad = std::atof(argv[++i]);
      distance_mm.reset();
    } else if (i == 1) {
      channel = arg;
    } else if (distance_mm.has_value() && i == 2) {
      distance_mm = std::atof(arg.c_str());
    } else {
      speed_mm_s = std::atof(arg.c_str());
    }
  }
  if (!distance_mm.has_value() && !angle_rad.has_value()) {
    std::cerr << "give a distance or --angle\n";
    return 2;
  }
  if (speed_mm_s <= 0.0) {
    std::cerr << "speed must be positive\n";
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
    const bool ok = distance_mm.has_value()
                        ? gripper.set_gripper_distance(*distance_mm, speed_mm_s)
                        : gripper.set_motor_angle(*angle_rad, speed_mm_s);
    const litegrip::GripperState state = gripper.get_state();
    std::cout << "target "
              << (distance_mm.has_value()
                      ? std::to_string(*distance_mm) + " mm"
                      : std::to_string(*angle_rad) + " rad")
              << ": " << (ok ? "reached" : "NOT reached") << "\n";
    std::cout << "  position " << state.position_rad << " rad ("
              << state.position_mm << " mm)\n";
    gripper.disable();
    gripper.disconnect();
    return ok ? 0 : 1;
  } catch (const litegrip::LiteGripError& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  }
}
