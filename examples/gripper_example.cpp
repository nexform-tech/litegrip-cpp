// gripper_example.cpp — connect, status, open/close, distance, grasp/release.
//
// Usage: litegrip_example_gripper_example [channel] [--no-motion]
//
// This program drives real hardware. Make sure the jaws' travel is clear.
// Pass --no-motion to connect and read status only.

#include <cstring>
#include <iostream>
#include <string>

#include "litegrip/litegrip.hpp"

namespace {

void print_status(litegrip::Gripper& gripper) {
  const litegrip::MotorData data = gripper.get_motor_data();
  const litegrip::MotorStatus status = gripper.get_motor_status();
  std::cout << "  position " << data.Position << " rad"
            << " | speed " << data.Speed << " rad/s"
            << " | torque " << data.Torque << " Nm"
            << " | status " << status.Status
            << " | MOS " << status.DriverTemp << " C"
            << " | coil " << status.MotorTemp << " C\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string channel = "can0";
  bool motion = true;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--help") == 0 ||
        std::strcmp(argv[i], "-h") == 0) {
      std::cout << "usage: " << argv[0]
                << " [channel] [--no-motion]\n";
      return 0;
    }
    if (std::strcmp(argv[i], "--no-motion") == 0) {
      motion = false;
    } else {
      channel = argv[i];
    }
  }

  try {
    litegrip::Gripper gripper(channel);

    std::cout << "litegrip_cpp " << gripper.get_version() << " on "
              << gripper.channel() << "\n";
    std::cout << "calibrated: "
              << (gripper.config().calibrated ? "yes" : "no") << "\n";

    gripper.connect();
    std::cout << "connected; refreshing status\n";
    gripper.refresh_status();
    print_status(gripper);

    if (!motion) {
      gripper.disconnect();
      return 0;
    }

    gripper.enable();
    std::cout << "enabled; closing\n";
    if (!gripper.close()) {
      std::cerr << "close did not press onto the calibrated stop\n";
    }
    std::cout << "opening\n";
    if (!gripper.open()) {
      std::cerr << "open did not press onto the calibrated stop\n";
    }

    std::cout << "positioning to 40 mm\n";
    if (!gripper.set_gripper_distance(40.0)) {
      std::cerr << "distance move did not reach its target\n";
    }
    print_status(gripper);

    std::cout << "grasping at 5 N for 1 s\n";
    if (!gripper.grasp(5.0, 1.0)) {
      std::cerr << "grasp failed (no bite, guard, protection or bus error)\n";
    }
    print_status(gripper);

    std::cout << "releasing 10 mm\n";
    if (!gripper.release(10.0)) {
      std::cerr << "release did not reach its target\n";
    }

    gripper.disable();
    gripper.disconnect();
    std::cout << "done\n";
    return 0;
  } catch (const litegrip::LiteGripError& error) {
    std::cerr << "litegrip error: " << error.what() << "\n";
    return 1;
  }
}
