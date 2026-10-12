// litegrip/calibration.hpp — calibration JSON load/save.
//
// Format is deliberately identical to the Python SDK's (same keys, same units)
// so the two can be compared side by side during the port's parallel-validation
// phase (see PLAN-litegrip-cpp.md D8).
//
// Path resolution mirrors the Python original:
//   * per channel: env LITEGRIP_CALIB, else ~/.litegrip/<channel>_calibration.json
//   * then the legacy single-file location ~/.litegrip/litegrip_calibration.json
//   * then the packaged, read-only factory_calibration.json
// A candidate that declares a *different* channel is skipped, not adopted:
// every LiteGrip ships at CAN id 0x08, so on a two-gripper machine the channel
// is the only identity key (see LiteGrip::load_calibration).
//
// Mount templates (normal.json / reverse.json) declare a direction by name and
// never fall back to the factory file — the fallback would be a *normal*
// mount, which is exactly what a "reverse" request must not silently become.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "litegrip/models.hpp"

namespace litegrip {

/// Environment variable overriding the user calibration path.
inline constexpr const char* kCalibEnvVar = "LITEGRIP_CALIB";

/// Environment variable naming the directory that holds the packaged data
/// (`<dir>/calibration/<file>`), searched before anything else.
///
/// `litegrip_cpp_DATA_DIR` in the CMake package config reports the installed
/// location. Use one of the two when the run-time search cannot find the
/// baseline — a deployment that moves the data, for instance.
inline constexpr const char* kDataDirEnvVar = "LITEGRIP_DATA_DIR";

/// Legacy user calibration path: $LITEGRIP_CALIB, else
/// ~/.litegrip/litegrip_calibration.json.
///
/// The pre-per-channel location, kept so a calibration saved by an older
/// version still loads; it is now the *legacy* entry of load_calibration()'s
/// automatic chain rather than the primary one. A calibration saved here is
/// still picked up next run regardless of the process working directory.
std::string default_calibration_path();

/// Per-channel user calibration path: $LITEGRIP_CALIB, else
/// ~/.litegrip/<channel>_calibration.json.
///
/// Every LiteGrip ships at CAN id 0x08, so when two grippers share a machine
/// the *channel* is the only thing that tells them apart — which is why the
/// per-channel file comes first and why a candidate naming a different channel
/// is skipped rather than adopted. Mirrors gripper.default_calib_path().
std::string default_calibration_path(const std::string& channel);

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
/// SDK (`<prefix>/bin/grip` beside `<prefix>/share/litegrip_cpp/calibration/`)
/// find its own baseline from any working directory. Entry 3 is last because it
/// is the only one that depends on where the caller happens to stand.
///
/// When no candidate exists the strongest is returned anyway, so the caller's
/// error message can name a path to act on. Mirrors gripper._FACTORY_CALIB.
std::string factory_calibration_path();

/// Names of the packaged mount templates, in declaration order.
///
/// A template declares a *direction* — which end of the travel is closed — and
/// nothing else. It deliberately carries no channel, CAN ids or gains: adopting
/// a device identity or a tuning from a file that exists to state a direction
/// would silently rewrite what the caller set. Mirrors gripper.list_templates().
std::vector<std::string> list_calibration_templates();

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

/// Path of a packaged mount template.
///
/// Found by the same search as factory_calibration_path(). Throws CommandError
/// for an unknown name — never falls back to the factory file. The factory file
/// is a *normal* mount, so answering a request for "reverse" with "normal" is
/// precisely the failure the template name exists to prevent. Mirrors
/// gripper._resolve_template().
std::string calibration_template_path(const std::string& name);

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
  /// How far open() may travel, counted from the closed zero; 0 = no limit.
  /// Unlike the Python SDK, this SDK does not act on it yet — see
  /// GripperConfig::work_stroke_mm.
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
