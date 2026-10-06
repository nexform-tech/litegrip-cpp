// test_calibration.cpp — calibration file load/save and path resolution.
//
// Uses the REAL packaged factory_calibration.json (its directory is passed in
// by CMake as LITEGRIP_TEST_DATA_DIR), so the shipped data file is verified
// rather than a fixture invented for the test.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "litegrip/calibration.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/json.hpp"
#include "litegrip/models.hpp"

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

const std::string kFactoryPath =
    std::string(LITEGRIP_TEST_DATA_DIR) + "/factory_calibration.json";

// The fixtures below are written with std::fopen, which creates the file but
// not its directory. Create it here: on a clean machine nothing has yet, and
// the checks must not depend on an earlier run having left it behind.
const char* const kFixtureDir = "/tmp/litegrip_calib_test";

}  // namespace

int main() {
  ::mkdir(kFixtureDir, 0755);

  // ── the shipped factory calibration ───────────────────────────────────
  {
    const auto factory = litegrip::read_calibration_file(kFactoryPath);
    check(factory.has_value(), "factory_calibration.json parses");
    if (factory.has_value()) {
      check(factory->has_closed && factory->has_open && factory->has_rad_to_mm,
            "required fields present");
      // Values from the file, unchanged from the Python SDK's copy.
      check_near(factory->zero_position_rad, 0.114, 1e-12, "zero_position_rad");
      check_near(factory->max_position_rad, -1.491, 1e-12, "max_position_rad");
      check_near(factory->rad_to_mm, 74.8, 1e-12, "rad_to_mm");
      check(factory->can_id.has_value() && *factory->can_id == 8, "can_id");
      check(factory->mst_id.has_value() && *factory->mst_id == 24, "mst_id");
      check(factory->canfd_mode.has_value() && !*factory->canfd_mode,
            "canfd_mode");
      check(factory->motor_type.has_value() && *factory->motor_type == "DM4310",
            "motor_type");
      check(factory->kp.has_value() && std::fabs(*factory->kp - 100.0) < 1e-12,
            "kp");
      check(factory->grasp_torque_threshold.has_value() &&
                std::fabs(*factory->grasp_torque_threshold - 0.5) < 1e-12,
            "grasp_torque_threshold");
    }
  }

  // ── malformed / missing ───────────────────────────────────────────────
  check(!litegrip::read_calibration_file(kFactoryPath + ".nope").has_value(),
        "missing file yields no value");
  {
    const std::string path = std::string(kFixtureDir) + "/broken.json";
    const std::string text = "{\"can_id\": 8}";  // required keys absent
    FILE* file = std::fopen(path.c_str(), "w");
    check(file != nullptr, "write fixture");
    if (file != nullptr) {
      std::fwrite(text.data(), 1, text.size(), file);
      std::fclose(file);
    }
    check(!litegrip::read_calibration_file(path).has_value(),
          "file without required keys is rejected");

    const std::string garbage = std::string(kFixtureDir) + "/garbage.json";
    file = std::fopen(garbage.c_str(), "w");
    if (file != nullptr) {
      const std::string bad = "{not json";
      std::fwrite(bad.data(), 1, bad.size(), file);
      std::fclose(file);
    }
    check(!litegrip::read_calibration_file(garbage).has_value(),
          "malformed JSON is rejected");
  }

  // ── write then read back ──────────────────────────────────────────────
  {
    litegrip::GripperConfig config;
    config.can_channel = "can0";
    config.pos_closed_rad = 1.775959;
    config.pos_open_rad = -0.064279;
    config.rad_to_mm = 65.21;
    config.kp = 5.0;
    config.kd = 2.0;
    config.grasp_torque_threshold = 0.5;

    const std::string path = std::string(kFixtureDir) + "/roundtrip.json";
    // The explicit can_id/mst_id/motor_type arguments are authoritative for
    // those three fields; the rest comes from the config.
    litegrip::write_calibration_file(path, config, 8, 18, "DM4310");

    const auto reloaded = litegrip::read_calibration_file(path);
    check(reloaded.has_value(), "written calibration reads back");
    if (reloaded.has_value()) {
      check_near(reloaded->zero_position_rad, 1.775959, 1e-6, "closed round-trip");
      check_near(reloaded->max_position_rad, -0.064279, 1e-6, "open round-trip");
      check_near(reloaded->rad_to_mm, 65.21, 1e-6, "rad_to_mm round-trip");
      check(reloaded->mst_id.has_value() && *reloaded->mst_id == 18,
            "mst_id round-trip");
      check(reloaded->kp.has_value() && std::fabs(*reloaded->kp - 5.0) < 1e-9,
            "kp round-trip");
      check(reloaded->channel.has_value() && *reloaded->channel == "can0",
            "channel round-trip");
      check(reloaded->calibrated.has_value() && *reloaded->calibrated,
            "the written file is stamped calibrated");
    }

    // travel_range_rad is written as the absolute difference.
    const auto document = litegrip::json::Value::parse_file(path);
    check(document.has_value(), "written file is valid JSON");
    if (document.has_value()) {
      check_near(document->get_number("travel_range_rad", 0.0),
                 std::fabs(-0.064279 - 1.775959), 1e-6,
                 "travel_range_rad is absolute");
      check(document->get_bool("calibrated", false),
            "the JSON itself carries the calibrated flag");
    }
  }

  // ── mount templates ───────────────────────────────────────────────────
  {
    // calibration_template_path() discovers the data directory at run time;
    // in the test tree its root is the calibration/ directory's parent.
    const std::string data_dir = std::string(LITEGRIP_TEST_DATA_DIR) + "/..";
    ::setenv("LITEGRIP_DATA_DIR", data_dir.c_str(), 1);

    const std::vector<std::string> names =
        litegrip::list_calibration_templates();
    check(names.size() == 2 && names[0] == "normal" && names[1] == "reverse",
          "the templates are normal, then reverse");

    const auto normal = litegrip::read_calibration_file(
        litegrip::calibration_template_path("normal"));
    check(normal.has_value(), "the normal template parses");
    if (normal.has_value()) {
      // Same numbers as the Python SDK's calibration_normal.json: the closed
      // stop is the numerically LARGER angle (a normal mount).
      check_near(normal->zero_position_rad, 0.114, 1e-12, "normal closed");
      check_near(normal->max_position_rad, -1.491, 1e-12, "normal open");
      check_near(normal->rad_to_mm, 74.8, 1e-12, "normal rad_to_mm");
      check(normal->calibrated.has_value() && *normal->calibrated,
            "the template declares itself calibrated");
      // A template states a *direction*: it must carry no identity and no
      // tuning, or loading it would silently rewrite what the caller set.
      check(!normal->channel.has_value() && !normal->can_id.has_value() &&
                !normal->mst_id.has_value() && !normal->kp.has_value() &&
                !normal->kd.has_value() && !normal->canfd_mode.has_value() &&
                !normal->grasp_torque_threshold.has_value(),
            "the template carries no channel / ids / gains");
    }

    const auto reverse = litegrip::read_calibration_file(
        litegrip::calibration_template_path("reverse"));
    check(reverse.has_value(), "the reverse template parses");
    if (normal.has_value() && reverse.has_value()) {
      check_near(reverse->zero_position_rad, normal->max_position_rad, 1e-12,
                 "reverse swaps the closed limit");
      check_near(reverse->max_position_rad, normal->zero_position_rad, 1e-12,
                 "reverse swaps the open limit");
      check_near(reverse->rad_to_mm, normal->rad_to_mm, 1e-12,
                 "both templates share the same scale");
    }

    // An unknown name is a hard error, and the message names the valid ones.
    bool threw = false;
    try {
      litegrip::calibration_template_path("sideways");
    } catch (const litegrip::CommandError& error) {
      threw = true;
      const std::string message = error.what();
      check(message.find("normal") != std::string::npos &&
                message.find("reverse") != std::string::npos,
            "the unknown-template error lists the valid names");
    }
    check(threw, "an unknown template name throws CommandError");

    // With no template available, the resolver returns the primary candidate
    // — never the factory path. (Under ctest the working directory holds no
    // calibration/ tree, so the data-dir candidate is conclusive.)
    const std::string empty_dir = std::string(kFixtureDir) + "/no_data";
    ::mkdir(empty_dir.c_str(), 0755);
    ::setenv("LITEGRIP_DATA_DIR", empty_dir.c_str(), 1);
    check(litegrip::calibration_template_path("normal") ==
              empty_dir + "/calibration/normal.json",
          "an unavailable template resolves to its own candidate, not the "
          "factory file");
    ::unsetenv("LITEGRIP_DATA_DIR");
  }

  // ── the calibrated flag ───────────────────────────────────────────────
  {
    // The shipped factory file predates the flag; absence leaves no opinion,
    // which every consumer reads as "yes".
    const auto factory = litegrip::read_calibration_file(kFactoryPath);
    check(factory.has_value() && !factory->calibrated.has_value(),
          "a file without the flag reports no opinion about being calibrated");

    const std::string path = std::string(kFixtureDir) + "/uncal.json";
    FILE* file = std::fopen(path.c_str(), "w");
    check(file != nullptr, "write uncalibrated fixture");
    if (file != nullptr) {
      const std::string text =
          "{\"calibrated\": false, \"zero_position_rad\": 0.1, "
          "\"max_position_rad\": -1.5, \"rad_to_mm\": 74.8}";
      std::fwrite(text.data(), 1, text.size(), file);
      std::fclose(file);
    }
    const auto uncal = litegrip::read_calibration_file(path);
    check(uncal.has_value() && uncal->calibrated.has_value() &&
              !*uncal->calibrated,
          "calibrated=false is decoded as such");
  }

  // ── what a save stamps ────────────────────────────────────────────────
  {
    litegrip::GripperConfig config;
    const std::string path = std::string(kFixtureDir) + "/stamps.json";
    // mst_id 0 means "unknown": it must be omitted, never stamped as a falsy
    // 0 that would later pin auto-detection to RX filter 0x000.
    litegrip::write_calibration_file(path, config, 8, 0, "DM4310");

    const auto document = litegrip::json::Value::parse_file(path);
    check(document.has_value(), "the stamped file parses");
    if (document.has_value()) {
      check(document->get_bool("calibrated", false),
            "a written calibration declares itself calibrated");
      check(!document->contains("mst_id"),
            "an unknown mst_id is omitted, not written as 0");
    }
    const auto reloaded = litegrip::read_calibration_file(path);
    check(reloaded.has_value() && reloaded->calibrated.has_value() &&
              *reloaded->calibrated,
          "the stamped flag reads back");
    check(reloaded.has_value() && !reloaded->mst_id.has_value(),
          "the omitted mst_id stays absent on reload");
  }

  // ── path resolution / overrides ───────────────────────────────────────
  {
    const std::string explicit_calib =
        std::string(kFixtureDir) + "/env_calib.json";
    ::setenv("LITEGRIP_CALIB", explicit_calib.c_str(), 1);
    check(litegrip::default_calibration_path() == explicit_calib,
          "LITEGRIP_CALIB overrides the default calib path");
    ::unsetenv("LITEGRIP_CALIB");

    // Without the override: $HOME/.litegrip/litegrip_calibration.json
    const std::string fake_home = std::string(kFixtureDir) + "/home";
    ::setenv("HOME", fake_home.c_str(), 1);
    check(litegrip::default_calibration_path() ==
              fake_home + "/.litegrip/litegrip_calibration.json",
          "default calib path is under $HOME/.litegrip");

    ::setenv("LITEGRIP_FACTORY_CALIB", kFactoryPath.c_str(), 1);
    check(litegrip::factory_calibration_path() == kFactoryPath,
          "LITEGRIP_FACTORY_CALIB overrides the factory path");
    ::unsetenv("LITEGRIP_FACTORY_CALIB");
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " calibration check(s) failed\n";
    return 1;
  }
  std::cout << "calibration checks OK\n";
  return 0;
}
