// teleop_slave.cpp — teleoperation follower: follow a UDP opening target.
//
// Usage:
//   litegrip_example_teleop_slave [--channel can1] [--peer 127.0.0.1]
//       [--port 17449] [--master-port 17448] [--rate 200] [--kp 100]
//       [--kd 2] [--watchdog-ms 200]
//
// The received value is an absolute opening in mm and is converted with this
// gripper's own calibration. The slave publishes its squeeze back to the master
// (the master renders it only with --force-feedback). If no master frame
// arrives for the watchdog window, the last target is held. Ctrl+C stops and
// disables the gripper.

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include "litegrip/litegrip.hpp"
#include "udp_teleop.hpp"

namespace {
volatile std::sig_atomic_t g_running = 1;
void on_sigint(int) { g_running = 0; }

double now_s() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

int main(int argc, char** argv) {
  std::string channel = "can1";
  std::string peer = "127.0.0.1";
  int port = 17449;
  int master_port = 17448;
  double rate_hz = 200.0;
  double kp = 100.0;
  double kd = 2.0;
  double watchdog_ms = 200.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "usage: " << argv[0]
                << " [--channel can1] [--peer host] [--port 17449]\n"
                   "       [--master-port 17448] [--rate 200] [--kp 100] "
                   "[--kd 2] [--watchdog-ms 200]\n";
      return 0;
    }
    auto value = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << name << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--channel") {
      channel = value("--channel");
    } else if (arg == "--peer") {
      peer = value("--peer");
    } else if (arg == "--port") {
      port = std::atoi(value("--port").c_str());
    } else if (arg == "--master-port") {
      master_port = std::atoi(value("--master-port").c_str());
    } else if (arg == "--rate") {
      rate_hz = std::atof(value("--rate").c_str());
    } else if (arg == "--kp") {
      kp = std::atof(value("--kp").c_str());
    } else if (arg == "--kd") {
      kd = std::atof(value("--kd").c_str());
    } else if (arg == "--watchdog-ms") {
      watchdog_ms = std::atof(value("--watchdog-ms").c_str());
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 2;
    }
  }
  if (rate_hz <= 0.0 || watchdog_ms <= 0.0) {
    std::cerr << "rate and watchdog must be positive\n";
    return 2;
  }

  std::signal(SIGINT, on_sigint);
  try {
    example::UdpSocket udp(port);
    litegrip::Gripper follower(channel);
    if (!follower.config().calibrated) {
      std::cerr << "no usable calibration — run a calibration first\n";
      return 2;
    }
    follower.connect();
    follower.enable();
    follower.refresh_status();
    const litegrip::GripperState initial = follower.get_state(true);
    if (!initial.has_data() || initial.is_stale()) {
      std::cerr << "no fresh status frame from the follower\n";
      return 2;
    }

    std::cout << "slave on " << channel << "; " << rate_hz << " Hz; listening "
              << "on port " << port << "; master " << peer << ":"
              << master_port << "; watchdog " << watchdog_ms << " ms\n";

    const double period = 1.0 / rate_hz;
    const double watchdog_s = watchdog_ms / 1000.0;
    double last_status = 0.0;
    double target = follower.get_gripper_distance();
    std::optional<double> last_rx;
    bool warned_stale = false;

    while (g_running) {
      const double t0 = now_s();
      while (const auto value = udp.recv_double()) {
        target = *value;
        last_rx = t0;
        warned_stale = false;
      }
      if (!last_rx.has_value()) {
        // No master yet: hold the current opening.
        target = follower.get_gripper_distance();
      } else if (t0 - *last_rx > watchdog_s && !warned_stale) {
        std::cerr << "warning: no master frame for " << watchdog_ms
                  << " ms — holding the last target\n";
        warned_stale = true;
      }

      follower.follow_gripper_distance(
          target, kp, kd, 0.0, 0.0,
          [&](double force) { udp.send_double(peer, master_port, force); });

      if (t0 - last_status >= 1.0) {
        last_status = t0;
        const litegrip::GripperState state = follower.get_state(false);
        std::cout << "slave target " << target << " mm | position "
                  << state.position_mm << " mm | force " << state.force_n
                  << " N | master age "
                  << (last_rx.has_value()
                          ? std::to_string(t0 - *last_rx) + "s"
                          : "n/a")
                  << "\n";
      }
      const double elapsed = now_s() - t0;
      if (elapsed < period) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(period - elapsed));
      }
    }
    follower.disable();
    follower.disconnect();
    return 0;
  } catch (const litegrip::LiteGripError& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  }
}
