// test_gripper.cpp — LiteGrip's behaviour that does NOT need hardware.
//
// Construction, the "not connected" refusals, calibration file plumbing, the
// move semantics connect_raii() needs, and the fact that the default safety
// limits are the packaged baseline. Motion and enable paths need the real
// device and are covered there.

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <sys/stat.h>
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

// Fixtures written with std::fopen need their directory created first: on a
// clean machine nothing has yet, and the checks must not depend on an earlier
// run having left it behind.
const char* const kFixtureDir = "/tmp/litegrip_gripper_test";

}  // namespace

int main() {
  ::mkdir(kFixtureDir, 0755);

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
    // zero() is calibrate()+save_calibration(); the probe is what refuses, and
    // it must refuse before anything could write a file.
    check_throws("zero refuses when not connected",
                 [&] { gripper.zero(); });
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
    check(std::fabs(gripper.config().pos_closed_rad - 0.052071) < 1e-12,
          "closed limit loaded from the factory file");
    check(std::fabs(gripper.config().pos_open_rad - (-1.357481)) < 1e-12,
          "open limit loaded from the factory file");
    check(std::fabs(gripper.config().rad_to_mm - 61.01229326764816) < 1e-12,
          "rad_to_mm loaded from the factory file");
    check(gripper.config().can_id == 8, "can_id loaded from the factory file");
    check(gripper.mst_id().has_value() && *gripper.mst_id() == 24,
          "mst_id loaded from the factory file");
    check(gripper.config().calibrated, "the factory file counts as calibrated");
    check(gripper.config().mount() == std::string("normal"),
          "the factory file's limits spell out the normal mount");
    // Carried for the Python SDK's benefit: this SDK loads and re-emits the
    // work stroke but nothing here acts on it (see
    // GripperConfig::work_stroke_mm).
    check(std::fabs(gripper.config().work_stroke_mm - 80.0) < 1e-12,
          "work_stroke_mm loaded from the factory file");
    check(litegrip::GripperConfig{}.work_stroke_mm == 0.0,
          "the default work stroke is 0, meaning no limit");

    check(!gripper.load_calibration("/tmp/litegrip_gripper_test/none.json"),
          "a missing calibration reports false and keeps the fallback order");
  }
  {
    // A file that does not mention the work stroke leaves the config's own
    // value alone — absence is "no opinion", like the calibrated flag.
    const std::string path = "/tmp/litegrip_gripper_test/no_work_stroke.json";
    FILE* file = std::fopen(path.c_str(), "w");
    check(file != nullptr, "write a calibration without a work stroke");
    if (file != nullptr) {
      const std::string text =
          "{\"zero_position_rad\": 1.2, \"max_position_rad\": -0.1, "
          "\"rad_to_mm\": 65.0}";
      std::fwrite(text.data(), 1, text.size(), file);
      std::fclose(file);
    }
    litegrip::GripperConfig config;
    config.work_stroke_mm = 75.0;
    litegrip::LiteGrip gripper(config);
    check(gripper.load_calibration(path), "the file loads");
    check(std::fabs(gripper.config().work_stroke_mm - 75.0) < 1e-12,
          "a file without a work stroke leaves the configured one in place");
  }
  {
    // Saving writes the current config back out.
    litegrip::GripperConfig config;
    config.pos_closed_rad = 1.2;
    config.pos_open_rad = -0.1;
    config.rad_to_mm = 65.0;
    config.work_stroke_mm = 75.0;
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
    check(std::fabs(reloaded.config().work_stroke_mm - 75.0) < 1e-6,
          "work_stroke_mm round-trips through the file");
    check(reloaded.config().calibrated,
          "a saved calibration reloads as calibrated");
  }
  {
    // Without an explicit path the default user path is used, and a missing file
    // falls through to the packaged factory calibration.
    ::setenv("LITEGRIP_CALIB", "/tmp/litegrip_gripper_test/absent.json", 1);
    ::setenv("LITEGRIP_FACTORY_CALIB", kFactoryPath.c_str(), 1);
    litegrip::LiteGrip gripper;
    check(gripper.load_calibration(), "falls back to the factory calibration");
    check(std::fabs(gripper.config().rad_to_mm - 61.01229326764816) < 1e-12,
          "factory values applied via the fallback");
    ::unsetenv("LITEGRIP_CALIB");
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
  }

  // ── mount direction is pure config arithmetic (no bus) ────────────────
  {
    litegrip::GripperConfig config;  // the shipped placeholder limits
    // The placeholders happen to order close above open like a normal mount,
    // but nothing has measured anything yet: mount() must not claim a
    // direction, because a claim is not a reading.
    check(!config.calibrated, "a fresh config is not calibrated");
    check(!config.mount().has_value(), "mount is unclaimed until calibrated");
    check(config.close_sign() > 0.0,
          "the placeholder ordering implies a normal mount");

    // For a normal mount the new unit conversion is bit-for-bit the old
    // formula: close_sign = +1 makes the extra multiply exact.
    const double position_rad = 0.4237;
    check(config.opening_mm(position_rad) ==
              (config.pos_closed_rad - position_rad) * config.rad_to_mm,
          "normal-mount mm is exactly the legacy formula");
    check(config.opening_mm(config.pos_open_rad) > 0.0,
          "the open stop reads a positive opening");
    check(config.rad_for_opening_mm(0.0) == config.pos_closed_rad,
          "0 mm maps back to the closed stop");
    check(config.force_n_from_torque(0.5) == 0.5 * config.nm_to_n,
          "normal-mount force is exactly torque * nm_to_n");
  }
  {
    litegrip::GripperConfig config;
    config.pos_closed_rad = -1.491;  // reverse: closing is the small angle
    config.pos_open_rad = 0.114;
    config.rad_to_mm = 74.8;
    config.calibrated = true;

    check(config.close_sign() < 0.0, "reverse close_sign is -1");
    check(config.mount() == std::string("reverse"),
          "the reverse mount reads back");
    check(config.opening_mm(config.pos_closed_rad) == 0.0,
          "reverse: the closed stop is 0 mm");
    check(config.opening_mm(config.pos_open_rad) > 0.0,
          "reverse: the open stop reads a positive opening");
    const double mm = 40.0;
    check(std::fabs(config.opening_mm(config.rad_for_opening_mm(mm)) - mm) <
              1e-9,
          "reverse: mm round-trips through rad");
    // The force reading follows the same sign: under a reverse mount a
    // positive motor torque is an opening torque, so it must read negative.
    check(config.force_n_from_torque(0.3) < 0.0,
          "reverse: a positive motor torque reads as opening");
  }

  // ── mount templates (no bus, no hardware) ─────────────────────────────
  {
    // The tests run from the build tree; point the data-dir discovery at the
    // source tree so the packaged templates resolve.
    const std::string data_dir = std::string(LITEGRIP_TEST_DATA_DIR) + "/..";
    ::setenv("LITEGRIP_DATA_DIR", data_dir.c_str(), 1);

    litegrip::LiteGrip gripper;
    check(gripper.load_template("normal"), "the normal template loads");
    check(gripper.config().calibrated, "a template counts as calibrated");
    check(gripper.config().mount() == std::string("normal"),
          "the normal template reads back as normal");

    litegrip::LiteGrip reverse;
    check(reverse.load_template("reverse"), "the reverse template loads");
    check(reverse.config().mount() == std::string("reverse"),
          "the reverse template reads back as reverse");
    check(reverse.config().close_sign() < 0.0,
          "the reverse template flips the sign");
    check(std::fabs(reverse.config().pos_closed_rad - (-1.491)) < 1e-12 &&
              std::fabs(reverse.config().pos_open_rad - 0.114) < 1e-12,
          "the reverse template's limits are the swapped pair");

    // A template carries no identity or tuning: declaring the mount must not
    // clobber the caller's arguments.
    litegrip::GripperConfig tuned;
    tuned.can_channel = "can1";
    tuned.can_id = 0x0A;
    tuned.kp = 321.0;
    tuned.kd = 4.5;
    litegrip::LiteGrip preserved(tuned);
    check(preserved.load_template("reverse"), "a template loads over a tuning");
    check(preserved.can_id() == 0x0A && preserved.config().kp == 321.0 &&
              preserved.config().kd == 4.5 &&
              preserved.config().can_channel == "can1",
          "the template did not clobber the id, channel or gains");
    check(!preserved.mst_id().has_value(),
          "the template did not pin the auto-detected mst_id");

    // An unknown name is a hard error that lists the valid names.
    litegrip::LiteGrip unknown;
    bool threw = false;
    try {
      unknown.load_template("sideways");
    } catch (const litegrip::CommandError& error) {
      threw = true;
      const std::string message = error.what();
      check(message.find("normal") != std::string::npos &&
                message.find("reverse") != std::string::npos,
            "the unknown-template error lists the valid names");
    }
    check(threw, "an unknown template name throws");
    check(!unknown.config().calibrated, "a failed template load changes nothing");

    // A template that cannot be read must NOT fall back to the factory file:
    // that file is a normal mount, and quietly answering "reverse" with
    // "normal" is the one failure the template name exists to prevent.
    const std::string empty_dir = std::string(kFixtureDir) + "/no_data";
    ::mkdir(empty_dir.c_str(), 0755);
    ::setenv("LITEGRIP_DATA_DIR", empty_dir.c_str(), 1);
    litegrip::LiteGrip unreadable;
    bool threw_strict = false;
    try {
      unreadable.load_template("reverse");
    } catch (const litegrip::CommandError&) {
      threw_strict = true;
    }
    check(threw_strict, "an unreadable template throws instead of falling back");
    check(!unreadable.config().calibrated, "the strict failure loaded nothing");
    ::unsetenv("LITEGRIP_DATA_DIR");
  }

  // ── the automatic load chain: one file per channel ────────────────────
  {
    // A two-gripper machine: both at CAN id 0x08, told apart by the channel
    // alone. Seed a private HOME, and pin the factory fallback so the real
    // user's files cannot leak into the checks.
    const std::string home = std::string(kFixtureDir) + "/home";
    const std::string dot_litegrip = home + "/.litegrip";
    ::mkdir(home.c_str(), 0755);
    ::mkdir(dot_litegrip.c_str(), 0755);
    const char* const old_home = ::getenv("HOME");
    const std::string saved_home = old_home != nullptr ? old_home : "";
    ::setenv("HOME", home.c_str(), 1);
    ::unsetenv("LITEGRIP_CALIB");
    ::setenv("LITEGRIP_FACTORY_CALIB", kFactoryPath.c_str(), 1);

    const auto seed = [](const std::string& path, const std::string& text) {
      FILE* file = std::fopen(path.c_str(), "w");
      if (file != nullptr) {
        std::fwrite(text.data(), 1, text.size(), file);
        std::fclose(file);
      }
    };

    // Nothing under kFixtureDir is cleaned between runs, and the assertions
    // below read that directory: the first can1 check is precisely that its
    // per-channel file does not exist yet. Clear the slots this block writes
    // before reading any of them, so a second run starts where the first did
    // instead of adopting the previous run's file and failing two checks.
    for (const char* const name : {"litegrip_calibration.json",
                                   "can1_calibration.json",
                                   "can2_calibration.json"}) {
      ::unlink((dot_litegrip + "/" + name).c_str());
    }

    // The legacy slot holds a *can0* calibration (the pre-per-channel era had
    // one shared file). A can0 unit adopts it...
    seed(dot_litegrip + "/litegrip_calibration.json",
         "{\"channel\": \"can0\", \"zero_position_rad\": 0.114, "
         "\"max_position_rad\": -1.491, \"rad_to_mm\": 65.0}");
    litegrip::LiteGrip can0;
    check(can0.load_calibration(), "a can0 unit adopts the can0 legacy file");
    check(std::fabs(can0.config().rad_to_mm - 65.0) < 1e-12,
          "the legacy file's own scale is in effect, not the factory fallback");

    // ...while a can1 unit must skip it — and skip the can0 factory file
    // too, failing loudly instead of silently adopting can0's direction and
    // travel.
    litegrip::GripperConfig can1_config;
    can1_config.can_channel = "can1";
    litegrip::LiteGrip can1(can1_config);
    check(!can1.load_calibration(),
          "a can1 unit adopts neither the can0 legacy file nor can0's factory "
          "file");
    check(!can1.config().calibrated, "the failed chain left it uncalibrated");

    // Its own per-channel file wins over both.
    seed(dot_litegrip + "/can1_calibration.json",
         "{\"channel\": \"can1\", \"zero_position_rad\": -1.491, "
         "\"max_position_rad\": 0.114, \"rad_to_mm\": 74.8}");
    check(can1.load_calibration(), "the per-channel file is adopted");
    check(can1.config().close_sign() < 0.0,
          "the per-channel (reverse) direction is in effect");
    check(can1.config().calibrated, "a successful chain load calibrates");

    // An explicit path is the caller's override: another channel's file loads
    // anyway (with a warning), because naming the file is unambiguous.
    litegrip::LiteGrip explicit_load;
    check(explicit_load.load_calibration(dot_litegrip +
                                         "/can1_calibration.json"),
          "an explicit path loads regardless of its channel field");
    check(explicit_load.config().close_sign() < 0.0,
          "the explicit file's direction is adopted");

    // Saving without a path lands in the channel's own file — the one the
    // chain reads first, so it is picked up next run without being told.
    litegrip::GripperConfig saver_config;
    saver_config.can_channel = "can2";
    litegrip::LiteGrip saver(saver_config);
    check(saver.save_calibration() == dot_litegrip + "/can2_calibration.json",
          "save_calibration() writes the per-channel file");

    if (old_home != nullptr) {
      ::setenv("HOME", saved_home.c_str(), 1);
    } else {
      ::unsetenv("HOME");
    }
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
  }

  // ── disable_on_disconnect ─────────────────────────────────────────────
  {
    litegrip::LiteGrip gripper;
    check(gripper.disable_on_disconnect(),
          "disconnect picks the disabling behaviour by default");
    gripper.set_disable_on_disconnect(false);
    check(!gripper.disable_on_disconnect(), "the flag is settable");
    litegrip::LiteGrip moved(std::move(gripper));
    check(!moved.disable_on_disconnect(), "the flag moves with the object");
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
