// test_gripper.cpp — Gripper facade behaviour that does NOT need hardware.
//
// Construction and automatic calibration loading, the "not connected"
// refusals of every getter, calibration file plumbing, the convenience
// constructor, and the noexcept teardown guarantee. Motion and enable paths
// need the real device and are covered there.

#include <cmath>
#include <cstdio>
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "litegrip/calibration.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/gripper.hpp"
#include "litegrip/safety.hpp"
#include "litegrip/teardown.hpp"
#include "litegrip/version.hpp"

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

void check_near(double got, double want, double tol, const char* what) {
  if (!(std::fabs(got - want) < tol)) {
    std::cerr << "FAIL: " << what << " (got " << got << ", want " << want
              << ")\n";
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
const char* const kFixtureDir = "/tmp/litegrip_gripper_test";

void seed(const std::string& path, const std::string& text) {
  FILE* file = std::fopen(path.c_str(), "w");
  if (file == nullptr) {
    std::cerr << "FAIL: could not write fixture " << path << "\n";
    ++g_failures;
    return;
  }
  std::fwrite(text.data(), 1, text.size(), file);
  std::fclose(file);
}

/// Environment isolation so the user's own ~/.litegrip files and shell
/// overrides cannot leak into the checks.
struct Environment {
  std::string home = std::string(kFixtureDir) + "/home";
  std::string dot = home + "/.litegrip";

  Environment() {
    ::mkdir(kFixtureDir, 0755);
    ::mkdir(home.c_str(), 0755);
    ::mkdir(dot.c_str(), 0755);
    const char* old_home = ::getenv("HOME");
    old_home_ = old_home != nullptr ? old_home : "";
    ::setenv("HOME", home.c_str(), 1);
    ::unsetenv("LITEGRIP_CALIB");
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
    for (const char* name :
         {"can0_calibration.json", "can1_calibration.json",
          "can2_calibration.json"}) {
      ::unlink((dot + "/" + name).c_str());
    }
  }
  ~Environment() {
    ::setenv("HOME", old_home_.c_str(), 1);
    ::unsetenv("LITEGRIP_CALIB");
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
  }

 private:
  std::string old_home_;
};

}  // namespace

int main() {
  Environment env;
  const double factory_scale = [] {
    const auto factory = litegrip::read_calibration_file(kFactoryPath);
    return factory.has_value() ? factory->rad_to_mm : 0.0;
  }();

  // ── construction must not touch the bus ───────────────────────────────
  {
    litegrip::Gripper gripper;
    check(!gripper.is_connected(), "a fresh gripper is not connected");
    check(!gripper.is_enabled(), "a fresh gripper is not enabled");
    check(gripper.channel() == litegrip::DefaultParams::kCanChannel,
          "default channel");
    check(gripper.can_id() == litegrip::GripperParams::kCanId, "default can_id");
    check(gripper.mst_id() == 0x18, "mst_id defaults to 0x18");
    check(!gripper.config().calibrated,
          "no calibration available => stays uncalibrated");
  }

  // ── the convenience constructor ───────────────────────────────────────
  {
    litegrip::Gripper gripper("can7", 0x09, 0x19);
    check(gripper.channel() == "can7", "convenience ctor: channel");
    check(gripper.can_id() == 0x09, "convenience ctor: can_id");
    check(gripper.mst_id() == 0x19, "convenience ctor: mst_id");
    litegrip::Gripper defaults("can8");
    check(defaults.can_id() == 0x08 && defaults.mst_id() == 0x18,
          "convenience ctor defaults can_id=0x08 / mst_id=0x18");
  }

  // ── the constructor auto-loads the factory calibration ────────────────
  {
    ::setenv("LITEGRIP_FACTORY_CALIB", kFactoryPath.c_str(), 1);
    litegrip::Gripper gripper;
    check(gripper.config().calibrated,
          "the constructor auto-loads the bundled factory calibration");
    check_near(gripper.config().pos_closed_rad, 0.029564, 1e-12,
               "closed limit loaded");
    check_near(gripper.config().pos_open_rad, -1.141184, 1e-12,
               "open limit loaded");
    check_near(gripper.config().rad_to_mm, factory_scale, 1e-12,
               "scale loaded");
    check_near(gripper.config().work_stroke_mm, 80.0, 1e-12,
               "work stroke loaded");
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
  }

  // ── the per-channel file outranks the factory file, and the env var wins ─
  {
    ::setenv("LITEGRIP_FACTORY_CALIB", kFactoryPath.c_str(), 1);
    seed(env.dot + "/can0_calibration.json",
         "{\"channel\": \"can0\", \"zero_position_rad\": 0.114, "
         "\"max_position_rad\": -1.491, \"rad_to_mm\": 74.8}");
    litegrip::Gripper from_channel;  // auto-load in the constructor
    check_near(from_channel.config().rad_to_mm, 74.8, 1e-12,
               "auto-load prefers the per-channel file");

    const std::string env_path = std::string(kFixtureDir) + "/env_calib.json";
    seed(env_path,
         "{\"channel\": \"can0\", \"zero_position_rad\": 0.2, "
         "\"max_position_rad\": -1.3, \"rad_to_mm\": 65.0}");
    ::setenv("LITEGRIP_CALIB", env_path.c_str(), 1);
    litegrip::Gripper from_env;
    check_near(from_env.config().rad_to_mm, 65.0, 1e-12,
               "auto-load prefers $LITEGRIP_CALIB over the per-channel file");
    ::unsetenv("LITEGRIP_CALIB");
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
    ::unlink((env.dot + "/can0_calibration.json").c_str());
  }

  // ── save_calibration() writes the channel's own file, load reads it back ─
  {
    litegrip::GripperConfig config;
    config.can_channel = "can2";
    config.pos_closed_rad = 1.2;
    config.pos_open_rad = -0.1;
    config.rad_to_mm = 65.0;
    config.work_stroke_mm = 75.0;

    litegrip::Gripper saved(config);
    const std::string path = saved.save_calibration();
    check(path == env.dot + "/can2_calibration.json",
          "save_calibration() writes the per-channel path");
    check(saved.load_calibration(path), "the saved calibration reloads");
    check_near(saved.config().pos_closed_rad, 1.2, 1e-6,
               "closed limit round-trips");
    check(saved.config().calibrated, "a saved calibration reloads as calibrated");

    // A new can2 instance picks it up automatically.
    litegrip::Gripper auto_loaded(config);
    check(auto_loaded.config().calibrated,
          "the constructor auto-loads this channel's saved file");
    check_near(auto_loaded.config().rad_to_mm, 65.0, 1e-6,
               "the auto-loaded file is the one just saved");
  }

  // ── an unreadable explicit path is not replaced by another file ───────
  {
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
    ::unsetenv("LITEGRIP_CALIB");
    const std::string custom = std::string(kFixtureDir) + "/explicit.json";
    seed(custom,
         "{\"channel\": \"can0\", \"zero_position_rad\": 1.2, "
         "\"max_position_rad\": -0.1, \"rad_to_mm\": 65.0}");
    litegrip::Gripper gripper;
    check(gripper.load_calibration(custom), "an explicit path loads");
    check_near(gripper.config().rad_to_mm, 65.0, 1e-12,
               "the explicit calibration is in effect");
    check(!gripper.load_calibration("/tmp/litegrip_gripper_test/none.json"),
          "a missing explicit path returns false");
    check_near(gripper.config().rad_to_mm, 65.0, 1e-12,
               "a missing explicit path does not fall back to another file");
  }

  // ── the bundled factory file is the last-resort fallback for any channel ─
  {
    ::setenv("LITEGRIP_FACTORY_CALIB", kFactoryPath.c_str(), 1);
    litegrip::Gripper can1("can1");
    check(can1.config().calibrated,
          "a can1 unit falls back to the bundled factory calibration");
    check_near(can1.config().rad_to_mm, factory_scale, 1e-12,
               "the factory values are in effect on can1");
    check(can1.load_calibration(kFactoryPath),
          "an explicit path loads as well");
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
  }

  // ── the default safety limits are the packaged baseline ───────────────
  {
    litegrip::Gripper gripper;
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
    litegrip::Gripper gripper;
    check_throws("enable refuses when not connected",
                 [&] { gripper.enable(); });
    check_throws("disable refuses when not connected",
                 [&] { gripper.disable(); });
    check_throws("clear_fault refuses when not connected",
                 [&] { gripper.clear_fault(); });
    check_throws("set_zero refuses when not connected",
                 [&] { gripper.set_zero(); });
    check_throws("calibrate refuses when not connected",
                 [&] { gripper.calibrate(); });
    check_throws("open refuses when not connected", [&] { gripper.open(); });
    check_throws("close refuses when not connected", [&] { gripper.close(); });
    check_throws("set_gripper_distance refuses when not connected",
                 [&] { gripper.set_gripper_distance(40.0); });
    check_throws("set_motor_angle refuses when not connected",
                 [&] { gripper.set_motor_angle(-0.5); });
    check_throws("follow_gripper_distance refuses when not connected",
                 [&] { gripper.follow_gripper_distance(40.0); });
    check_throws("get_gripper_force refuses when not connected",
                 [&] { gripper.get_gripper_force(); });
    check(!gripper.render_force_feedback(20.0),
          "force feedback is off by default");
    gripper.configure_force_feedback(true);
    check_throws("enabled force feedback refuses when not connected",
                 [&] { gripper.render_force_feedback(20.0); });
    check_throws("negative feedback gain is refused", [&] {
      gripper.configure_force_feedback(true, -1.0);
    });
    check_throws("feedback max_tau above 3.5 is refused", [&] {
      gripper.configure_force_feedback(true, 0.1, 2.0, 4.0);
    });
    check_throws("grasp refuses when not connected", [&] { gripper.grasp(); });
    check_throws("release refuses when not connected",
                 [&] { gripper.release(10.0); });

    // Getters throw rather than returning plausible zeros.
    check_throws("get_state refuses when not connected",
                 [&] { gripper.get_state(); });
    check_throws("refresh_status refuses when not connected",
                 [&] { gripper.refresh_status(); });
    check_throws("get_motor_data refuses when not connected",
                 [&] { gripper.get_motor_data(); });
    check_throws("get_motor_status refuses when not connected",
                 [&] { gripper.get_motor_status(); });
    check_throws("get_motor_position refuses when not connected",
                 [&] { gripper.get_motor_position(); });
    check_throws("get_motor_speed refuses when not connected",
                 [&] { gripper.get_motor_speed(); });
    check_throws("get_motor_torque refuses when not connected",
                 [&] { gripper.get_motor_torque(); });
    check_throws("get_gripper_distance refuses when not connected",
                 [&] { gripper.get_gripper_distance(); });
    check_throws("get_driver_temp refuses when not connected",
                 [&] { gripper.get_driver_temp(); });
    check_throws("get_motor_temp refuses when not connected",
                 [&] { gripper.get_motor_temp(); });
    check_throws("get_status_raw refuses when not connected",
                 [&] { gripper.get_status_raw(); });

    // ...but the SDK version is a build-time property, not device state.
    check(gripper.get_version() == litegrip::version(),
          "get_version reports the SDK version without a bus");
  }

  // ── connecting to a missing interface surfaces ConnectError ───────────
  {
    litegrip::GripperConfig config;
    config.can_channel = "lg_no_such_iface";
    litegrip::Gripper gripper(config);
    check_throws("connect to a missing interface throws",
                 [&] { gripper.connect(); });
    check(!gripper.is_connected(), "stays disconnected after a failed connect");
  }

  // ── config plumbing ───────────────────────────────────────────────────
  {
    litegrip::GripperConfig config;
    config.can_channel = "can1";
    config.can_id = 0x0A;
    config.mst_id = 0x1A;
    config.kp = 150.0;
    litegrip::Gripper gripper(config);
    check(gripper.config().can_channel == "can1", "config carried through");
    check(gripper.config().can_id == 0x0A, "can_id carried through");
    check(gripper.config().mst_id == 0x1A, "mst_id carried through");
    check(gripper.config().kp == 150.0, "kp carried through");
  }

  // ── unit conversion is pure config arithmetic (no bus) ────────────────
  {
    litegrip::GripperConfig config;  // shipped placeholder limits
    check(!config.calibrated, "a fresh config is not calibrated");
    check(config.close_sign() > 0.0,
          "the placeholder ordering implies a normal mount");
    check(config.opening_mm(config.pos_open_rad) > 0.0,
          "the open stop reads a positive opening");
    check(config.rad_for_opening_mm(0.0) == config.pos_closed_rad,
          "0 mm maps back to the closed stop");
  }
  {
    litegrip::GripperConfig config;
    config.pos_closed_rad = -1.491;  // reverse: closing is the small angle
    config.pos_open_rad = 0.114;
    config.rad_to_mm = 74.8;
    config.calibrated = true;
    check(config.close_sign() < 0.0, "reverse close_sign is -1");
    check(config.opening_mm(config.pos_closed_rad) == 0.0,
          "reverse: the closed stop is 0 mm");
    const double mm = 40.0;
    check(std::fabs(config.opening_mm(config.rad_for_opening_mm(mm)) - mm) <
              1e-9,
          "reverse: mm round-trips through rad");
  }

  // ── the teardown step wrapper (~Gripper must not terminate) ────────────
  {
    bool ran = false;
    check(litegrip::teardown_step_quietly("step", [&] { ran = true; }),
          "a teardown step that succeeds reports success");
    check(ran, "...and it really ran");
  }
  {
    const bool completed = litegrip::teardown_step_quietly(
        "disable", [] { throw litegrip::CommError("CAN send failed"); });
    check(!completed, "a failing teardown step reports failure, not a throw");
  }
  {
    const bool completed = litegrip::teardown_step_quietly(
        "disable", [] { throw std::runtime_error("not ours"); });
    check(!completed, "a non-SDK exception is contained too");
  }
  {  // Public and callable by hand: safe from any state, including never
     // having been connected.
    litegrip::Gripper gripper;
    gripper.disconnect();
    gripper.disconnect();
    check(!gripper.is_connected(), "disconnect() is safe on a fresh gripper");
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " gripper check(s) failed\n";
    return 1;
  }
  std::cout << "gripper checks OK (no hardware touched)\n";
  return 0;
}
