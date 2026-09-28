// test_calibration.cpp — calibration file load/save and path resolution.
//
// Uses the REAL packaged factory_calibration.json (its directory is passed in
// by CMake as LITEGRIP_TEST_DATA_DIR), so the shipped data file is verified
// rather than a fixture invented for the test.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>

#include "litegrip/calibration.hpp"
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

}  // namespace

int main() {
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
    const std::string path = "/tmp/litegrip_calib_test/broken.json";
    const std::string text = "{\"can_id\": 8}";  // required keys absent
    FILE* file = std::fopen(path.c_str(), "w");
    check(file != nullptr, "write fixture");
    if (file != nullptr) {
      std::fwrite(text.data(), 1, text.size(), file);
      std::fclose(file);
    }
    check(!litegrip::read_calibration_file(path).has_value(),
          "file without required keys is rejected");

    const std::string garbage = "/tmp/litegrip_calib_test/garbage.json";
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

    const std::string path = "/tmp/litegrip_calib_test/roundtrip.json";
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
    }

    // travel_range_rad is written as the absolute difference.
    const auto document = litegrip::json::Value::parse_file(path);
    check(document.has_value(), "written file is valid JSON");
    if (document.has_value()) {
      check_near(document->get_number("travel_range_rad", 0.0),
                 std::fabs(-0.064279 - 1.775959), 1e-6,
                 "travel_range_rad is absolute");
    }
  }

  // ── path resolution / overrides ───────────────────────────────────────
  {
    const std::string explicit_calib = "/tmp/litegrip_calib_test/env_calib.json";
    ::setenv("LITEGRIP_CALIB", explicit_calib.c_str(), 1);
    check(litegrip::default_calibration_path() == explicit_calib,
          "LITEGRIP_CALIB overrides the default calib path");
    ::unsetenv("LITEGRIP_CALIB");

    // Without the override: $HOME/.litegrip/litegrip_calibration.json
    ::setenv("HOME", "/tmp/litegrip_calib_test/home", 1);
    check(litegrip::default_calibration_path() ==
              "/tmp/litegrip_calib_test/home/.litegrip/litegrip_calibration.json",
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
