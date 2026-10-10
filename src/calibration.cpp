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

/// Candidate locations for a packaged data file, strongest first.
///
/// No machine-dependent path is baked into the source: the installed location
/// is discovered at run time, with an explicit override for deployments that
/// move the data elsewhere.
std::vector<std::string> data_file_candidates(const std::string& relative) {
  std::vector<std::string> candidates;
  if (const char* data_dir = env_or_null("LITEGRIP_DATA_DIR")) {
    candidates.push_back(std::string(data_dir) + "/calibration/" + relative);
  }
  candidates.emplace_back("calibration/" + relative);
  candidates.emplace_back("../calibration/" + relative);
  candidates.emplace_back(relative);
  return candidates;
}

std::string resolve_data_file(const std::string& relative) {
  const std::vector<std::string> candidates = data_file_candidates(relative);
  for (const std::string& candidate : candidates) {
    if (file_exists(candidate)) {
      return candidate;
    }
  }
  // Nothing found: return the primary candidate so the caller's error message
  // names a path someone can actually act on.
  return candidates.front();
}

/// The packaged mount templates, in declaration order (normal first).
///
/// A template declares a *direction* and nothing else: it deliberately carries
/// no channel, CAN ids or gains, because adopting an identity or a tuning from
/// a file that exists to state a direction would silently rewrite what the
/// caller set. Mirrors the Python SDK's CALIB_TEMPLATES.
struct CalibrationTemplate {
  const char* name;
  const char* file;
};

constexpr CalibrationTemplate kCalibrationTemplates[] = {
    {"normal", "normal.json"},
    {"reverse", "reverse.json"},
};

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

std::string default_calibration_path(const std::string& channel) {
  if (const char* override_path = env_or_null(kCalibEnvVar)) {
    return override_path;
  }
  const char* home = env_or_null("HOME");
  if (home == nullptr) {
    return ".litegrip/" + channel + "_calibration.json";
  }
  return std::string(home) + "/.litegrip/" + channel + "_calibration.json";
}

std::string factory_calibration_path() {
  // An explicit override wins: deployments that move the data elsewhere use
  // it, and the tests pin the fallback order with it.
  if (const char* explicit_path = env_or_null("LITEGRIP_FACTORY_CALIB")) {
    return explicit_path;
  }
  // Otherwise the installed location is discovered at run time — no
  // machine-dependent path is baked into the source.
  return resolve_data_file("factory_calibration.json");
}

std::vector<std::string> list_calibration_templates() {
  std::vector<std::string> names;
  for (const CalibrationTemplate& entry : kCalibrationTemplates) {
    names.emplace_back(entry.name);
  }
  return names;
}

std::string calibration_template_path(const std::string& name) {
  for (const CalibrationTemplate& entry : kCalibrationTemplates) {
    if (name == entry.name) {
      return resolve_data_file(entry.file);
    }
  }

  std::string available;
  for (const CalibrationTemplate& entry : kCalibrationTemplates) {
    if (!available.empty()) {
      available += ", ";
    }
    available += std::string("'") + entry.name + "'";
  }
  // Strict on purpose: answering an unknown (or mistyped) name with the
  // factory file would turn a request for a reverse mount into a normal one
  // without a sound. Mirrors the Python SDK's _resolve_template().
  throw CommandError("unknown calibration template '" + name +
                     "'; the available templates are " + available +
                     " (an explicit calibration file path can still be passed "
                     "to load_calibration)");
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
  if (document->contains("work_stroke_mm")) {
    out.work_stroke_mm = document->get_number("work_stroke_mm", 0.0);
  }
  // Absence leaves the optional empty, which every consumer reads as "yes":
  // files written before the flag existed always came from a real calibration
  // run. Mirrors the Python SDK's `data.get("calibrated", True)`.
  if (document->contains("calibrated")) {
    out.calibrated = document->get_bool("calibrated", true);
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
  document.set("canfd_mode", config.canfd_mode);
  // Anything written from a live config states a direction on purpose, so the
  // numbers are a claim even when they were never measured. Mirrors the
  // Python SDK stamping the flag unconditionally.
  document.set("calibrated", true);
  document.set("zero_position_rad", config.pos_closed_rad);
  document.set("max_position_rad", config.pos_open_rad);
  document.set("travel_range_rad",
               std::abs(config.pos_open_rad - config.pos_closed_rad));
  document.set("rad_to_mm", config.rad_to_mm);
  document.set("work_stroke_mm", config.work_stroke_mm);
  document.set("motor_type", motor_type);
  document.set("kp", config.kp);
  document.set("kd", config.kd);
  document.set("grasp_torque_threshold", config.grasp_torque_threshold);
  // Only stamp mst_id when it is actually known. Writing a falsy 0 would pin
  // auto-detection: load_calibration would install an RX filter of 0x000,
  // every reply from the motor would be dropped, and enable() would fail
  // after a long retry loop. (Python appends this key last, same reason.)
  if (mst_id != 0) {
    document.set("mst_id", mst_id);
  }

  if (!document.write_file(path)) {
    throw CommError("cannot write calibration file: " + path);
  }
}

}  // namespace litegrip
