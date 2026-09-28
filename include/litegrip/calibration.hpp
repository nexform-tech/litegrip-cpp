// litegrip/calibration.hpp — calibration JSON load/save.
//
// Format is deliberately identical to the Python SDK's (same keys, same units)
// so the two can be compared side by side during the port's parallel-validation
// phase (see PLAN-litegrip-cpp.md D8).
//
// Path resolution mirrors the Python original:
//   * env var LITEGRIP_CALIB, else ~/.litegrip/litegrip_calibration.json
//   * when that file does not exist, fall back to the packaged, read-only
//     factory_calibration.json

#pragma once

#include <optional>
#include <string>

#include "litegrip/models.hpp"

namespace litegrip {

/// Environment variable overriding the user calibration path.
inline constexpr const char* kCalibEnvVar = "LITEGRIP_CALIB";

/// Default user calibration path: $LITEGRIP_CALIB, else ~/.litegrip/litegrip_calibration.json.
///
/// A stable absolute location, so a calibration saved without an explicit path
/// is picked up next run regardless of the process working directory.
std::string default_calibration_path();

/// Path of the packaged factory calibration (read-only fallback).
std::string factory_calibration_path();

/// Calibration file contents, decoded.
struct CalibrationFile {
  bool has_closed = false;
  double zero_position_rad = 0.0;  // closed limit
  bool has_open = false;
  double max_position_rad = 0.0;   // open limit
  bool has_rad_to_mm = false;
  double rad_to_mm = 0.0;

  // Optional fields (present in newer calibration files).
  std::optional<int> can_id;
  std::optional<int> mst_id;
  std::optional<std::string> channel;
  std::optional<bool> canfd_mode;
  std::optional<double> kp;
  std::optional<double> kd;
  std::optional<double> grasp_torque_threshold;
  std::optional<std::string> motor_type;
};

/// Read + decode a calibration file. Returns nullopt when the file is missing
/// or malformed.
std::optional<CalibrationFile> read_calibration_file(const std::string& path);

/// Write a calibration file (creates parent directories). Throws CommError on
/// I/O failure.
void write_calibration_file(const std::string& path, const GripperConfig& config,
                            int can_id, int mst_id, const std::string& motor_type);

}  // namespace litegrip
