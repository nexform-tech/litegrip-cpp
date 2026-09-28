// test_gripper.cpp — LiteGrip's behaviour that does NOT need hardware.
//
// Construction, the "not connected" refusals, calibration file plumbing, the
// move semantics connect_raii() needs, and the fact that the default safety
// limits are the packaged baseline. Motion and enable paths need the real
// device and are covered there.

#include <cmath>
#include <iostream>
#include <string>
#include <unistd.h>
#include <utility>

#include "litegrip/exceptions.hpp"
#include "litegrip/gripper.hpp"
#include "litegrip/safety.hpp"

#ifndef LITEGRIP_TEST_DATA_DIR
#error "LITEGRIP_TEST_DATA_DIR must be defined by CMake"
#endif

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

template <typename Fn>
void check_throws(const char* what, Fn&& fn) {
  try {
    fn();
    std::cerr << "FAIL: " << what << " (did not throw)\n";
    ++g_failures;
  } catch (const litegrip::LiteGripError&) {
    // expected
  }
}

const std::string kFactoryPath =
    std::string(LITEGRIP_TEST_DATA_DIR) + "/factory_calibration.json";

}  // namespace

int main() {
  // ── construction must not touch the bus ───────────────────────────────
  {
    litegrip::LiteGrip gripper;
    check(!gripper.is_connected(), "a fresh gripper is not connected");
    check(!gripper.is_enabled(), "a fresh gripper is not enabled");
    check(gripper.channel() == litegrip::DefaultParams::kCanChannel,
          "default channel");
    check(gripper.can_id() == litegrip::GripperParams::kCanId, "default can_id");
    check(!gripper.mst_id().has_value(), "mst_id is unset before connect");
  }

  // ── the default safety limits are the packaged baseline ───────────────
  {
    litegrip::LiteGrip gripper;
    const litegrip::SafetyLimits& limits = gripper.safety().limits();
    check(std::fabs(limits.red_min_rad - litegrip::kPackageRedMin) < 1e-12,
          "the default guard uses the packaged open red line");
    check(std::fabs(limits.red_max_rad - litegrip::kPackageRedMax) < 1e-12,
          "the default guard uses the packaged closed red line");
    check(limits.enabled, "the default guard has the red lines enabled");
    check(gripper.safety().mode() == litegrip::FrameMode::kNormal,
          "the default guard starts in normal mode");
  }

  // ── everything that needs the bus refuses while disconnected ──────────
  {
    litegrip::LiteGrip gripper;
    check_throws("init refuses when not connected",
                 [&] { gripper.init(); });
    check_throws("enable refuses when not connected",
                 [&] { gripper.enable(); });
    check_throws("disable refuses when not connected",
                 [&] { gripper.disable(); });
    check_throws("clear_fault refuses when not connected",
                 [&] { gripper.clear_fault(); });
    check_throws("get_state refuses when not connected",
                 [&] { gripper.get_state(); });
    check_throws("get_position_rad refuses when not connected",
                 [&] { gripper.get_position_rad(); });
    check_throws("goto_mm refuses when not connected",
                 [&] { gripper.goto_mm(40.0); });
    check_throws("goto_rad refuses when not connected",
                 [&] { gripper.goto_rad(-0.5); });
    check_throws("move_to refuses when not connected",
                 [&] { gripper.move_to(-0.5); });
    check_throws("read_param refuses when not connected",
                 [&] { gripper.read_param(7); });
    check_throws("write_param refuses when not connected",
                 [&] { gripper.write_param(7, 1.0); });

    // These report rather than throw.
    check(!gripper.send_mit_frame(0.0, 0.0, 0.0), "send_mit_frame reports false");
    check(!gripper.poll(0.0), "poll reports false");
    gripper.stop();  // a no-op, must not throw
  }

  // ── connecting to a missing interface surfaces ConnectError ───────────
  {
    litegrip::GripperConfig config;
    config.can_channel = "lg_no_such_iface";
    litegrip::LiteGrip gripper(config);
    check_throws("connect to a missing interface throws",
                 [&] { gripper.connect(); });
    check(!gripper.is_connected(), "stays disconnected after a failed connect");
  }

  // ── config plumbing ───────────────────────────────────────────────────
  {
    litegrip::GripperConfig config;
    config.can_channel = "can1";
    config.can_id = 0x0A;
    config.kp = 150.0;
    litegrip::LiteGrip gripper(config);
    check(gripper.config().can_channel == "can1", "config carried through");
    check(gripper.config().can_id == 0x0A, "can_id carried through");
    check(gripper.config().kp == 150.0, "kp carried through");
    gripper.config().kd = 3.0;
    check(gripper.config().kd == 3.0, "config is mutable");
  }

  // ── calibration file plumbing (no hardware involved) ──────────────────
  {
    litegrip::LiteGrip gripper;
    // An explicit path loads even without a connection, matching the Python
    // SDK's documented order (load after connect, before enable).
    check(gripper.load_calibration(kFactoryPath), "explicit calibration loads");
    check(std::fabs(gripper.config().pos_closed_rad - 0.114) < 1e-12,
          "closed limit loaded from the factory file");
    check(std::fabs(gripper.config().pos_open_rad - (-1.491)) < 1e-12,
          "open limit loaded from the factory file");
    check(std::fabs(gripper.config().rad_to_mm - 74.8) < 1e-12,
          "rad_to_mm loaded from the factory file");
    check(gripper.config().can_id == 8, "can_id loaded from the factory file");
    check(gripper.mst_id().has_value() && *gripper.mst_id() == 24,
          "mst_id loaded from the factory file");

    check(!gripper.load_calibration("/tmp/litegrip_gripper_test/none.json"),
          "a missing calibration reports false and keeps the fallback order");
  }
  {
    // Saving writes the current config back out.
    litegrip::GripperConfig config;
    config.pos_closed_rad = 1.2;
    config.pos_open_rad = -0.1;
    config.rad_to_mm = 65.0;
    litegrip::LiteGrip gripper(config);
    const std::string path = "/tmp/litegrip_gripper_test/saved.json";
    check(gripper.save_calibration(path) == path,
          "save_calibration returns the path written");

    litegrip::LiteGrip reloaded;
    check(reloaded.load_calibration(path), "the saved calibration reloads");
    check(std::fabs(reloaded.config().pos_closed_rad - 1.2) < 1e-6,
          "closed limit round-trips through the file");
    check(std::fabs(reloaded.config().rad_to_mm - 65.0) < 1e-6,
          "rad_to_mm round-trips through the file");
  }
  {
    // Without an explicit path the default user path is used, and a missing file
    // falls through to the packaged factory calibration.
    ::setenv("LITEGRIP_CALIB", "/tmp/litegrip_gripper_test/absent.json", 1);
    ::setenv("LITEGRIP_FACTORY_CALIB", kFactoryPath.c_str(), 1);
    litegrip::LiteGrip gripper;
    check(gripper.load_calibration(), "falls back to the factory calibration");
    check(std::fabs(gripper.config().rad_to_mm - 74.8) < 1e-12,
          "factory values applied via the fallback");
    ::unsetenv("LITEGRIP_CALIB");
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
  }

  // ── info and the move semantics connect_raii needs ────────────────────
  {
    litegrip::GripperConfig config;
    config.can_id = 0x08;
    litegrip::LiteGrip gripper(config);
    const litegrip::GripperInfo info = gripper.get_info();
    check(info.can_id == 0x08, "get_info reports can_id");
    check(info.motor_type == "DM4310", "get_info reports the motor type");
    check(info.model == "LiteGrip", "get_info reports the model");
  }
  {
    litegrip::LiteGrip original;
    litegrip::LiteGrip moved(std::move(original));
    check(!moved.is_connected(), "a moved-to gripper is not connected");
    // The moved-from object must be inert: its destructor must not try to close
    // a bus it no longer owns.
    check(!original.is_connected(), "a moved-from gripper reports disconnected");
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " gripper check(s) failed\n";
    return 1;
  }
  std::cout << "gripper checks OK (no hardware touched)\n";
  return 0;
}
