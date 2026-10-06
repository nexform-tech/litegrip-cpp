// test_actions.cpp — the actions layer (actions.hpp / actions.cpp).
//
// The Python suite's TestEnable ported case for case, plus zero()'s config
// plumbing, motion_config() reaching the engine, and the session slot. No
// hardware: FakeActionsHost implements both seams (MotionIo + ActionsHost)
// around the shared kinematic motor in fake_motion.hpp, and the retry
// interval is a counted no-op sleep.

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fake_motion.hpp"
#include "litegrip/actions.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/gripper.hpp"

namespace {

using namespace litegrip;
using namespace fake_motion;

int g_failures = 0;

void check(bool ok, const std::string& what) {
  if (!ok) {
    std::printf("FAIL %s\n", what.c_str());
    ++g_failures;
  }
}

void check_near(double got, double want, double eps, const std::string& what) {
  if (!(std::fabs(got - want) <= eps)) {
    std::printf("FAIL %s: got %.12g want %.12g (eps %g)\n", what.c_str(), got,
                want, eps);
    ++g_failures;
  }
}

template <typename Ex, typename Fn>
void check_throws(Fn&& fn, const std::string& what) {
  try {
    fn();
  } catch (const Ex&) {
    return;
  } catch (const std::exception& e) {
    std::printf("FAIL %s: wrong exception: %s\n", what.c_str(), e.what());
    ++g_failures;
    return;
  }
  std::printf("FAIL %s: no exception\n", what.c_str());
  ++g_failures;
}

/// The gripper seam the actions layer drives: MotionIo + ActionsHost over the
/// shared kinematic motor. enable_once() answers from a scripted queue and
/// sets motor.err = 1 on success, exactly like fake_can.py's initialize()
/// (an empty queue answers True — "a good motor"; a failure leaves err
/// untouched, which is what lets a real fault survive to be cleared).
class FakeActionsHost : public ActionsHost {
 private:
  GripperConfig cfg_;
  bool enabled_ = true;

 public:
  explicit FakeActionsHost(GripperConfig cfg, double start_rad = -1.0,
                           int err = 0,
                           std::optional<double> block_rad = std::nullopt,
                           double sticky_rad = 0.0, bool stops = false)
      : cfg_(std::move(cfg)),
        motor_(start_rad, block_rad, sticky_rad, err,
               stops ? std::optional<double>(kLimitLo) : std::nullopt,
               stops ? std::optional<double>(kLimitHi) : std::nullopt) {}

  // ── MotionIo ──────────────────────────────────────────────────────────
  const GripperConfig& config() const noexcept override { return cfg_; }
  bool is_enabled() const noexcept override { return enabled_; }
  void check_enabled() const override {
    if (!enabled_) {
      throw NotInitializedError("not enabled");
    }
  }

  GripperState get_state(bool wait) override {
    (void)wait;  // the fake is instantaneous either way
    GripperState st;
    const double s = cfg_.close_sign();
    st.position_rad = motor_.reported_pos();
    st.velocity_rad_s = motor_.vel;
    st.torque_nm = motor_.tau;
    st.error_code = motor_.err;
    st.position_mm =
        (cfg_.pos_closed_rad - st.position_rad) * s * cfg_.rad_to_mm;
    st.force_n = s * st.torque_nm * cfg_.nm_to_n;
    st.data_age_s = 0.0;
    return st;
  }

  bool send_mit_frame(double q, double kp, double kd, double dq,
                      double tau) override {
    motor_.step(q, kp, dq, tau, kDt);
    frames.push_back(Frame{q, kp, kd, dq, tau, motor_.pos, motor_.tau});
    holding = tau != 0.0;
    return true;
  }

  // ── ActionsHost ───────────────────────────────────────────────────────
  bool enable_once() override {
    ++initialize_calls;
    const bool ok = pop_result();
    if (ok) {
      motor_.err = 1;
    }
    enabled_ = ok;
    return ok;
  }

  bool disable_once() override {
    ++disable_calls;
    enabled_ = false;
    return true;
  }

  bool clear_fault() override {
    ++clear_fault_calls;
    motor_.err = 0;
    return true;
  }

  CalibrationData calibrate(double kp, double kd, double step_rad,
                            double stall_delta, int stall_cycles, int max_iter,
                            std::optional<double> tau_limit) override {
    ++calibrate_calls;
    calibrate_kp = kp;
    calibrate_kd = kd;
    calibrate_step_rad = step_rad;
    calibrate_stall_delta = stall_delta;
    calibrate_stall_cycles = stall_cycles;
    calibrate_max_iter = max_iter;
    calibrate_tau_limit = tau_limit;
    return canned_calibration;
  }

  std::string save_calibration(std::optional<std::string> path) override {
    ++save_calls;
    saved_path = path;
    return path.has_value() ? *path : std::string("fake-calibration.json");
  }

  /// fake_can.py's initialize() answer: scripted, and an empty queue means
  /// "a good motor" (True), not a refusal.
  bool pop_result() {
    if (initialize_results.empty()) {
      return true;
    }
    const bool value = initialize_results.front();
    initialize_results.erase(initialize_results.begin());
    return value;
  }

  std::vector<bool> initialize_results;
  int initialize_calls = 0;
  int clear_fault_calls = 0;
  int disable_calls = 0;
  int calibrate_calls = 0;
  int save_calls = 0;
  double calibrate_kp = 0.0;
  double calibrate_kd = 0.0;
  double calibrate_step_rad = 0.0;
  double calibrate_stall_delta = 0.0;
  int calibrate_stall_cycles = 0;
  int calibrate_max_iter = 0;
  std::optional<double> calibrate_tau_limit;
  std::optional<std::string> saved_path;
  CalibrationData canned_calibration;
  FakeMotor motor_;
  std::vector<Frame> frames;
  bool holding = false;
};

// ── TestEnable, ported case for case from the Python suite ───────────────

void test_enable_retries_until_err_is_one() {
  FakeActionsHost host(test_config());
  host.initialize_results = {false, false, true};
  SafetyGuard guard = make_guard();
  std::vector<double> sleeps;
  MotionConfig cfg;
  cfg.sleep_fn = [&](double s) { sleeps.push_back(s); };
  GripperActions actions(host, guard, cfg);

  const EnableResult res = actions.enable();
  check(res.ok, "enable: ok after the third attempt");
  check(res.tries == 3, "enable: tries (py: 3)");
  check(res.state.has_value() && res.state->error_code == 1,
        "enable: the readback shows error_code 1");
  check(host.initialize_calls == 3, "enable: initialize calls (py: 3)");
  check(host.is_enabled(), "enable: the motor is enabled");
  check(host.clear_fault_calls == 0, "enable: no fault was cleared");
  check(sleeps.size() == 2, "enable: two retry waits");
  check_near(sleeps.empty() ? 0.0 : sleeps[0], 0.2, 1e-12,
             "enable: retry interval (py: 0.2)");
}

void test_enable_gives_up_when_never_enabled() {
  FakeActionsHost host(test_config());
  host.initialize_results = {false, false, false};
  SafetyGuard guard = make_guard();
  MotionConfig cfg;
  cfg.sleep_fn = [](double) {};
  GripperActions actions(host, guard, cfg);

  const EnableResult res = actions.enable();
  check(!res.ok, "enable: gives up");
  check(res.tries == 3, "enable: tries exhausted (py: 3)");
  check(res.state.has_value() && res.state->error_code == 0,
        "enable: the last readback shows error_code 0");
  check(!host.is_enabled(), "enable: still disabled");
}

void test_enable_real_fault_is_cleared_first() {
  FakeActionsHost host(test_config(), -1.0, /*err=*/0x9);
  host.initialize_results = {false, true};
  SafetyGuard guard = make_guard();
  MotionConfig cfg;
  cfg.sleep_fn = [](double) {};
  GripperActions actions(host, guard, cfg);

  const EnableResult res = actions.enable();
  check(res.ok, "enable: ok on the second attempt");
  check(host.clear_fault_calls == 1, "enable: fault cleared once (py: 1)");
  check(res.tries == 2, "enable: tries (py: 2)");
}

void test_enable_explicit_retries_override() {
  FakeActionsHost host(test_config());
  host.initialize_results = {false, true};
  SafetyGuard guard = make_guard();
  MotionConfig cfg;
  cfg.sleep_fn = [](double) {};
  GripperActions actions(host, guard, cfg);

  const EnableResult res = actions.enable(1);
  check(!res.ok, "enable: one attempt is not enough here");
  check(res.tries == 1, "enable: tries == the explicit 1 (py: 1)");
  check(host.initialize_calls == 1, "enable: one initialize call");
}

void test_enable_result_shape() {
  const EnableResult res;
  check(!res.ok, "EnableResult default is falsy");
  check(!static_cast<bool>(res), "EnableResult default bool is false");
  check(res.tries == 0, "EnableResult default tries 0");
  check(!res.state.has_value(), "EnableResult default has no state");
}

// ── zero(): the config's calib_* values, then save ───────────────────────

void test_zero_passes_config_and_saves() {
  FakeActionsHost host(test_config());
  SafetyGuard guard = make_guard();
  MotionConfig cfg;
  cfg.sleep_fn = [](double) {};
  GripperActions actions(host, guard, cfg);

  const CalibrationData data = actions.zero();
  check(host.calibrate_calls == 1, "zero: one calibrate run");
  check_near(host.calibrate_kp, 20.0, 0.0, "zero: calib_kp (py: 20.0)");
  check_near(host.calibrate_kd, 2.0, 0.0, "zero: calib_kd (py: 2.0)");
  check_near(host.calibrate_step_rad, 0.05, 0.0,
             "zero: calib_step_rad (py: 0.05)");
  check_near(host.calibrate_stall_delta, 0.0015, 0.0,
             "zero: calib_stall_delta (py: 0.0015)");
  check(host.calibrate_stall_cycles == 5, "zero: calib_stall_cycles (py: 5)");
  check(host.calibrate_max_iter == 200, "zero: calib_max_iter (py: 200)");
  check(host.calibrate_tau_limit.has_value() &&
            *host.calibrate_tau_limit == 2.0,
        "zero: calib_tau_limit (py: 2.0)");
  check(host.save_calls == 1, "zero: saved once");
  check(host.saved_path == std::nullopt, "zero: saved to the default path");
  check(data.zero_position == host.canned_calibration.zero_position,
        "zero: returns the calibration data");
}

void test_zero_follows_the_motion_config() {
  FakeActionsHost host(test_config());
  SafetyGuard guard = make_guard();
  MotionConfig cfg;
  cfg.sleep_fn = [](double) {};
  GripperActions actions(host, guard, cfg);

  actions.config.calib_kp = 33.0;
  actions.config.calib_tau_limit = 1.0;
  actions.zero();
  check_near(host.calibrate_kp, 33.0, 0.0, "zero: follows a changed calib_kp");
  check(host.calibrate_tau_limit.has_value() &&
            *host.calibrate_tau_limit == 1.0,
        "zero: follows a changed calib_tau_limit");
}

// ── motion_config plumbing: a write reaches the engine ───────────────────

void test_motion_config_reaches_the_engine() {
  FakeActionsHost host(test_config(), -1.0, /*err=*/1, std::nullopt, 0.0,
                       /*stops=*/true);
  SafetyGuard guard = make_guard();
  GripperActions actions(host, guard, instant_config());

  const MoveResult at_default = actions.close();
  check(at_default.ok, "config plumbing: default-speed close presses the stop");
  check(!host.frames.empty(), "config plumbing: frames were sent");
  if (host.frames.empty()) {
    return;
  }
  check_near(host.frames.front().dq, 50.0 / kRadToMm, 1e-12,
             "config plumbing: default close speed is 50 mm/s");
  const std::size_t frames_default = host.frames.size();
  host.frames.clear();
  // The first close pressed onto the stop, which sits outside the red lines;
  // restart the second measurement back inside them so both runs measure the
  // same ramp (a start from outside would first run the recovery drive-in,
  // and its frames are not speed frames).
  host.motor_.pos = -1.0;

  actions.config.speed_mm_s = 25.0;
  const MoveResult at_half = actions.close();
  check(at_half.ok, "config plumbing: half-speed close presses the stop");
  check(!host.frames.empty(), "config plumbing: frames were sent");
  if (host.frames.empty()) {
    return;
  }
  check_near(host.frames.front().dq, 25.0 / kRadToMm, 1e-12,
             "config plumbing: the per-frame dq follows speed_mm_s");
  check(host.frames.size() > frames_default,
        "config plumbing: a slower close takes more frames");
}

// ── LiteGrip forwarding, accessors and the session slot ──────────────────

void test_litegrip_accessors_and_session() {
  LiteGrip gripper;  // default config; construction must not touch the bus
  check(!gripper.session().has_value(), "session: empty on a fresh gripper");

  gripper.motion_config().speed_mm_s = 25.0;
  check_near(gripper.motion_config().speed_mm_s, 25.0, 0.0,
             "motion_config: read back after write");
  check(&gripper.actions().config == &gripper.motion_config(),
        "actions() and motion_config() alias the same config");

  LiteGrip moved(std::move(gripper));
  check_near(moved.motion_config().speed_mm_s, 25.0, 0.0,
             "the config moves with the gripper");
  check(!moved.session().has_value(), "session: still empty after a move");
  check(&moved.actions().config == &moved.motion_config(),
        "the moved-to gripper's actions point at itself");
}

void test_litegrip_refusals_while_disconnected() {
  LiteGrip gripper;
  // enable()'s attempt is caught and logged, and the readback then refuses —
  // the same shape as Python, where get_state() raises.
  check_throws<NotInitializedError>([&] { gripper.enable(); },
                                    "LiteGrip::enable while disconnected");
  check_throws<NotInitializedError>([&] { gripper.zero(); },
                                    "LiteGrip::zero while disconnected");
}

}  // namespace

int main() {
  test_enable_retries_until_err_is_one();
  test_enable_gives_up_when_never_enabled();
  test_enable_real_fault_is_cleared_first();
  test_enable_explicit_retries_override();
  test_enable_result_shape();
  test_zero_passes_config_and_saves();
  test_zero_follows_the_motion_config();
  test_motion_config_reaches_the_engine();
  test_litegrip_accessors_and_session();
  test_litegrip_refusals_while_disconnected();

  if (g_failures == 0) {
    std::printf("test_actions: all checks passed\n");
    return 0;
  }
  std::printf("test_actions: %d check(s) failed\n", g_failures);
  return 1;
}
