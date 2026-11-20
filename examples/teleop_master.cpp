// teleop_master.cpp — teleoperation leader: publish the opening over UDP.
//
// Usage:
//   litegrip_example_teleop_master [--channel can0] [--peer 127.0.0.1]
//       [--port 17448] [--slave-port 17449] [--rate 200] [--force-feedback]
//
// The leader is enabled and then sent zero-gain MIT frames, so it is
// back-drivable. --force-feedback also renders the slave's squeeze (off by
// default). This is a demonstration, not a production teleop stack: no
// authentication, no retransmission, no ready handshake. Support the leader
// before starting and clear the jaw travel.

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
  std::string channel = "can0";
  std::string peer = "127.0.0.1";
  int port = 17448;
  int slave_port = 17449;
  double rate_hz = 200.0;
  bool force_feedback = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "usage: " << argv[0]
                << " [--channel can0] [--peer host] [--port 17448]\n"
                   "       [--slave-port 17449] [--rate 200] "
                   "[--force-feedback]\n";
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
    } else if (arg == "--slave-port") {
      slave_port = std::atoi(value("--slave-port").c_str());
    } else if (arg == "--rate") {
      rate_hz = std::atof(value("--rate").c_str());
    } else if (arg == "--force-feedback") {
      force_feedback = true;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 2;
    }
  }
  if (rate_hz <= 0.0) {
    std::cerr << "rate must be positive\n";
    return 2;
  }

  std::signal(SIGINT, on_sigint);
  try {
    example::UdpSocket udp(port);
    litegrip::Gripper leader(channel);
    if (!leader.config().calibrated) {
      std::cerr << "no usable calibration — run a calibration first\n";
      return 2;
    }
    leader.connect();
    leader.enable();
    leader.configure_force_feedback(force_feedback);
    leader.refresh_status();
    const litegrip::GripperState initial = leader.get_state(true);
    if (!initial.has_data() || initial.is_stale()) {
      std::cerr << "no fresh status frame from the leader\n";
      return 2;
    }

    std::cout << "master on " << channel << "; " << rate_hz << " Hz -> "
              << peer << ":" << slave_port << "; feedback port " << port
              << "; force feedback " << (force_feedback ? "on" : "off")
              << "\n";

    const double period = 1.0 / rate_hz;
    double last_status = 0.0;
    double last_force = 0.0;
    std::optional<double> last_force_rx;
    while (g_running) {
      const double t0 = now_s();
      while (const auto force = udp.recv_double()) {
        last_force = *force;
        last_force_rx = t0;
      }

      const litegrip::GripperState state = leader.get_state(false);
      const double distance = state.position_mm;
      if (force_feedback) {
        const double age = last_force_rx.has_value() ? t0 - *last_force_rx : 0.0;
        leader.render_force_feedback(last_force, age);
      } else {
        // Zero-gain frame: keeps the leader enabled and back-drivable.
        leader.follow_gripper_distance(distance, 0.0, 0.0);
      }
      udp.send_double(peer, slave_port, distance);

      if (t0 - last_status >= 1.0) {
        last_status = t0;
        std::cout << "master " << distance << " mm | force " << last_force
                  << " N (age "
                  << (last_force_rx.has_value()
                          ? std::to_string(t0 - *last_force_rx) + "s"
                          : "n/a")
                  << ")\n";
      }
      const double elapsed = now_s() - t0;
      if (elapsed < period) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(period - elapsed));
      }
    }
    leader.disable();
    leader.disconnect();
    return 0;
  } catch (const litegrip::LiteGripError& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  }
}
