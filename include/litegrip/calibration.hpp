// litegrip/calibration.hpp — calibration JSON load/save.
//
// Format is identical to the Python SDK's (same keys, same units), so the two
// can be compared side by side.
//
// Path resolution:
//   * the automatic chain (constructor / no-argument load_calibration()) reads
//     $LITEGRIP_CALIB when set, then ~/.litegrip/<channel>_calibration.json,
//     then the packaged factory_calibration.json
//   * save_calibration() writes ~/.litegrip/<channel>_calibration.json by
//     default, so the next constructor picks the measured calibration up
//   * a candidate that declares a *different* channel is skipped (not fatal):
//     several LiteGrips may share CAN id 0x08 on one machine, and then the
//     channel is the only identity key.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "litegrip/models.hpp"

namespace litegrip {

/// Environment variable naming an explicit calibration file to load.
inline constexpr const char* kCalibEnvVar = "LITEGRIP_CALIB";

/// Environment variable naming the directory that holds the packaged data
/// (`<dir>/calibration/<file>`), searched before anything else.
///
/// `litegrip_cpp_DATA_DIR` in the CMake package config reports the installed
/// location. Use one of the two when the run-time search cannot find the
/// factory calibration — a deployment that moves the data, for instance.
inline constexpr const char* kDataDirEnvVar = "LITEGRIP_DATA_DIR";

/// Per-channel user calibration path: `~/.litegrip/<channel>_calibration.json`
/// (`channel` defaults to can0).
///
/// This is what save_calibration() writes and what the automatic load chain
/// reads after `$LITEGRIP_CALIB`. It deliberately does NOT apply the
/// `$LITEGRIP_CALIB` override: that variable names a file to LOAD, while this
/// function answers "where does this channel's own measurement live?".
std::string default_calibration_path(
    const std::string& channel = DefaultParams::kCanChannel);

/// Path of the packaged factory calibration (read-only fallback).
///
/// $LITEGRIP_FACTORY_CALIB wins outright. Otherwise the file is searched for in
/// this order, first hit first:
///
///   1. `$LITEGRIP_DATA_DIR/calibration/`
///   2. next to the running program: `<dir>/../share/litegrip_cpp/calibration/`,
///      then `<dir>/../calibration/`, then `<dir>/calibration/`, where `<dir>`
///      is the program's own directory, read from `/proc/self/exe`
///   3. the process working directory: `calibration/`, `../calibration/`, `.`
///
/// Nothing machine-dependent is compiled in: entry 2 is what lets an installed
/// SDK find its own baseline from any working directory. Entry 3 is last
/// because it is the only one that depends on where the caller happens to stand.
///
/// When no candidate exists the strongest is returned anyway, so the caller's
/// error message can name a path to act on.
std::string factory_calibration_path();

/// The locations searched for a packaged data file, strongest first, for a
/// program running from `program_dir` (pass an empty string for a program whose
/// own directory cannot be read).
///
/// Exposed for diagnosis: when a baseline is not found, this is how a caller
/// prints what was tried instead of guessing. The order is the one documented
/// on factory_calibration_path().
std::vector<std::string> calibration_data_candidates(
    const std::string& relative, const std::string& program_dir);

/// The packaged data file `relative`, resolved for a program running from
/// `program_dir`: the first candidate that exists, or the strongest candidate
/// when none does (so the caller's error message names a path to act on).
std::string resolve_calibration_data_file(const std::string& relative,
                                          const std::string& program_dir);

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
  std::optional<double> kp;
  std::optional<double> kd;
  std::optional<std::string> motor_type;
  /// How far open() may travel, counted from the closed zero; 0 = no limit.
  std::optional<double> work_stroke_mm;

  /// Whether the numbers in this file are a real measurement. Absent means
  /// yes, matching the Python SDK's `data.get("calibrated", True)`.
  std::optional<bool> calibrated;
};

/// Read + decode a calibration file. Returns nullopt when the file is missing
/// or malformed.
std::optional<CalibrationFile> read_calibration_file(const std::string& path);

/// Write a calibration file (creates parent directories). Throws CommError on
/// I/O failure.
void write_calibration_file(const std::string& path, const GripperConfig& config,
                            int can_id, int mst_id, const std::string& motor_type);

}  // namespace litegrip
