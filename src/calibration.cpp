// calibration.cpp — calibration file load/save and path resolution.
//
// The JSON keys and units are identical to the Python SDK's, so the two can be
// diffed during parallel validation.

#include "litegrip/calibration.hpp"

#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "litegrip/exceptions.hpp"
#include "litegrip/json.hpp"

namespace litegrip {
namespace {

bool file_exists(const std::string& path) {
  struct stat info {};
  return ::stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

const char* env_or_null(const char* name) {
  const char* value = std::getenv(name);
  return (value != nullptr && value[0] != '\0') ? value : nullptr;
}

}  // namespace

std::string default_calibration_path() {
  if (const char* override_path = env_or_null(kCalibEnvVar)) {
    return override_path;
  }
  const char* home = env_or_null("HOME");
  if (home == nullptr) {
    return ".litegrip/litegrip_calibration.json";
  }
  return std::string(home) + "/.litegrip/litegrip_calibration.json";
}

std::string factory_calibration_path() {
  // No machine-dependent path is baked into the source: the installed location
  // is discovered at run time, with an explicit override for deployments that
  // move the data elsewhere.
  if (const char* explicit_path = env_or_null("LITEGRIP_FACTORY_CALIB")) {
    return explicit_path;
  }

  std::vector<std::string> candidates;
  if (const char* data_dir = env_or_null("LITEGRIP_DATA_DIR")) {
    candidates.push_back(std::string(data_dir) + "/calibration/factory_calibration.json");
  }
  candidates.emplace_back("calibration/factory_calibration.json");
  candidates.emplace_back("../calibration/factory_calibration.json");
  candidates.emplace_back("factory_calibration.json");

  for (const std::string& candidate : candidates) {
    if (file_exists(candidate)) {
      return candidate;
    }
  }
  // Nothing found: return the primary candidate so the caller's error message
  // names a path someone can actually act on.
  return candidates.front();
}

std::optional<CalibrationFile> read_calibration_file(const std::string& path) {
  const auto document = json::Value::parse_file(path);
  if (!document.has_value() || !document->is_object()) {
    return std::nullopt;
  }

  // The three fields the calibration is meaningless without. A file missing any
  // of them is treated as malformed (the Python original would raise KeyError).
  for (const char* required : {"zero_position_rad", "max_position_rad",
                              "rad_to_mm"}) {
    if (!document->contains(required)) {
      return std::nullopt;
    }
  }

  CalibrationFile out;
  out.has_closed = true;
  out.zero_position_rad = document->get_number("zero_position_rad", 0.0);
  out.has_open = true;
  out.max_position_rad = document->get_number("max_position_rad", 0.0);
  out.has_rad_to_mm = true;
  out.rad_to_mm = document->get_number("rad_to_mm", 0.0);

  // Optional fields, present from newer calibration files onwards.
  if (document->contains("can_id")) {
    out.can_id = document->get_int("can_id", 0);
  }
  if (document->contains("mst_id")) {
    out.mst_id = document->get_int("mst_id", 0);
  }
  if (document->contains("channel")) {
    out.channel = document->get_string("channel", "");
  }
  if (document->contains("canfd_mode")) {
    out.canfd_mode = document->get_bool("canfd_mode", false);
  }
  if (document->contains("kp")) {
    out.kp = document->get_number("kp", 0.0);
  }
  if (document->contains("kd")) {
    out.kd = document->get_number("kd", 0.0);
  }
  if (document->contains("grasp_torque_threshold")) {
    out.grasp_torque_threshold =
        document->get_number("grasp_torque_threshold", 0.0);
  }
  if (document->contains("motor_type")) {
    out.motor_type = document->get_string("motor_type", "");
  }
  return out;
}

void write_calibration_file(const std::string& path, const GripperConfig& config,
                            int can_id, int mst_id,
                            const std::string& motor_type) {
  // Key order matches the Python SDK's save_calibration(), so files produced by
  // either implementation diff cleanly.
  json::Value document = json::Value::make_object();
  document.set("channel", config.can_channel);
  document.set("can_id", can_id);
  document.set("mst_id", mst_id);
  document.set("canfd_mode", config.canfd_mode);
  document.set("zero_position_rad", config.pos_closed_rad);
  document.set("max_position_rad", config.pos_open_rad);
  document.set("travel_range_rad",
               std::abs(config.pos_open_rad - config.pos_closed_rad));
  document.set("rad_to_mm", config.rad_to_mm);
  document.set("motor_type", motor_type);
  document.set("kp", config.kp);
  document.set("kd", config.kd);
  document.set("grasp_torque_threshold", config.grasp_torque_threshold);

  if (!document.write_file(path)) {
    throw CommError("cannot write calibration file: " + path);
  }
}

}  // namespace litegrip
