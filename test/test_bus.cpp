// test_bus.cpp — GripperBus lifecycle, error paths and the HoldPolicy seam.
//
// Hardware-dependent paths (connect / init / motion) are NOT exercised here:
// they need the real device. What is covered is the behaviour that must hold
// without one — construction, the "not connected" refusals, config plumbing,
// and the pluggable hold policy (R2).

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "litegrip/bus.hpp"
#include "litegrip/exceptions.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

// A stand-in for the user's rewrite of the hold behaviour: proves the seam is
// usable without touching anything else.
class TrackingPolicy : public litegrip::HoldPolicy {
 public:
  void init(litegrip::GripperBus&, const litegrip::GripperConfig&) override {
    ++init_calls;
  }
  void hold(litegrip::GripperBus&, const litegrip::GripperConfig&) override {
    ++hold_calls;
  }
  const char* name() const noexcept override { return "TrackingPolicy"; }

  int init_calls = 0;
  int hold_calls = 0;
};

}  // namespace

int main() {
  // ── construction must not touch any hardware ──────────────────────────
  {
    litegrip::GripperBus bus;
    check(!bus.is_connected(), "a fresh bus is not connected");
    check(!bus.is_initialized(), "a fresh bus is not initialised");
    check(bus.motor() == nullptr, "a fresh bus has no motor");
    check(bus.config().can_id == litegrip::GripperParams::kCanId,
          "config defaults carried through");
  }

  // ── every bus operation refuses while disconnected ────────────────────
  {
    litegrip::GripperBus bus;
    check(!bus.control_mit(0.0, 10.0, 1.0), "control_mit refuses when closed");
    check(!bus.control_mit_stream(0.0, 10.0, 1.0, 0.01),
          "control_mit_stream refuses when closed");
    check(!bus.poll(0.0), "poll refuses when closed");
    check(!bus.update_state(0.01), "update_state refuses when closed");
    check(!bus.refresh_status(0.01), "refresh_status refuses when closed");
    check(!bus.enable(), "enable refuses when closed");
    check(!bus.disable(), "disable refuses when closed");
    check(!bus.clear_fault(), "clear_fault refuses when closed");
    check(bus.get_position() == 0.0, "position defaults to 0");
    check(bus.get_velocity() == 0.0, "velocity defaults to 0");
    check(bus.get_torque() == 0.0, "torque defaults to 0");
    check(bus.get_error() == -1, "error defaults to -1 when there is no motor");

    bool threw = false;
    try {
      bus.read_param(7);
    } catch (const litegrip::NotInitializedError&) {
      threw = true;
    } catch (const litegrip::LiteGripError&) {
      threw = true;
    }
    check(threw, "read_param throws while disconnected");
  }

  // ── init on a closed bus is an explicit error, not a silent false ─────
  {
    litegrip::GripperBus bus;
    bool threw = false;
    try {
      bus.init();
    } catch (const litegrip::NotInitializedError&) {
      threw = true;
    } catch (const litegrip::LiteGripError&) {
      threw = true;
    }
    check(threw, "init on a closed bus throws");
  }

  // ── connecting to a missing interface surfaces ConnectError ───────────
  {
    litegrip::GripperConfig config;
    config.can_channel = "lg_no_such_iface";
    litegrip::GripperBus bus(config);
    bool threw = false;
    try {
      bus.connect();
    } catch (const litegrip::ConnectError&) {
      threw = true;
    } catch (const litegrip::LiteGripError&) {
      threw = true;
    }
    check(threw, "connect to a missing interface throws");
    check(!bus.is_connected(), "bus stays disconnected after a failed connect");
    // A failed connect must leave it usable for a retry.
    check(!bus.control_mit(0.0, 0.0, 0.0), "still refuses after a failed connect");
  }

  // ── config plumbing ───────────────────────────────────────────────────
  {
    litegrip::GripperConfig config;
    config.can_channel = "can1";
    config.can_id = 0x09;
    config.mst_id = 0x19;
    config.canfd_mode = true;
    config.kp = 123.0;
    litegrip::GripperBus bus(config);
    check(bus.config().can_channel == "can1", "channel carried into the bus");
    check(bus.config().can_id == 0x09, "can_id carried into the bus");
    check(bus.config().mst_id.has_value() && *bus.config().mst_id == 0x19,
          "mst_id carried into the bus");
    check(bus.config().canfd_mode, "canfd_mode carried into the bus");

    // The accessor is mutable, matching the Python SDK's model.
    bus.config().kp = 200.0;
    check(bus.config().kp == 200.0, "config is mutable through the accessor");
  }

  // ── the R2 hold seam ──────────────────────────────────────────────────
  {
    litegrip::GripperBus bus;
    check(std::string(bus.hold_policy().name()) == "DefaultHoldPolicy",
          "default hold policy installed");

    auto policy = std::make_unique<TrackingPolicy>();
    TrackingPolicy* observer = policy.get();
    bus.set_hold_policy(std::move(policy));
    check(std::string(bus.hold_policy().name()) == "TrackingPolicy",
          "injected hold policy is in effect");
    check(observer->init_calls == 0 && observer->hold_calls == 0,
          "injected policy is not called until init/enable");

    // A null policy must not replace a working one.
    bus.set_hold_policy(nullptr);
    check(std::string(bus.hold_policy().name()) == "TrackingPolicy",
          "null policy is ignored");
  }

  // ── the refresh retry policy ──────────────────────────────────────────
  //
  // The first refresh_status() after connect() is answered late by the mst_id
  // detection's parameter read rather than by its own request, so one attempt
  // is lost on a healthy link. The policy is driven with lambdas here because
  // the failure it covers needs a motor that answers 0.5 s late.
  {
    int sends = 0;
    const bool ok =
        litegrip::await_refresh_reply(0.5, [&] { ++sends; },
                                      [](double) { return false; });
    check(!ok, "refresh: nothing answering reports failure");
    check(sends == 2, "refresh: ...after exactly two attempts, not more");
  }
  {
    // The bug's shape: the first attempt is lost, the second is answered.
    int sends = 0;
    const bool ok = litegrip::await_refresh_reply(
        0.5, [&] { ++sends; }, [&](double) { return sends == 2; });
    check(ok, "refresh: a lost first attempt is retried and succeeds");
    check(sends == 2, "refresh: the answered attempt ends the retry");
  }
  {
    std::vector<double> budgets;
    int sends = 0;
    const bool ok = litegrip::await_refresh_reply(
        0.5, [&] { ++sends; },
        [&](double budget) {
          budgets.push_back(budget);
          return true;
        });
    check(ok && sends == 1, "refresh: an answered first attempt sends once");
    check(budgets.size() == 1 && budgets[0] == 0.25,
          "refresh: an attempt gets half the caller's budget");
  }

  // ── a wait = true state read waits the staleness horizon ───────────────
  //
  // #50: the wait and is_stale() have to describe the same thing. A wait
  // shorter than kStaleAfterS can come back empty-handed while the snapshot it
  // returns is still younger than the horizon — is_stale() false — so a cached
  // read is handed back as a measurement and the guard the README documents
  // misses it. Waiting the horizon is what makes a miss mean "stale".
  {
    double budget = -1.0;
    const bool fresh =
        litegrip::await_fresh_state([&](double b) { budget = b; return true; });
    check(budget == litegrip::kStaleAfterS,
          "state wait: the budget handed to the wait is the staleness horizon");
    check(fresh, "state wait: an arriving frame is reported fresh");
  }
  {
    double budget = -1.0;
    const bool miss =
        litegrip::await_fresh_state([&](double b) { budget = b; return false; });
    check(budget == litegrip::kStaleAfterS,
          "state wait: a miss waits the whole horizon before answering");
    check(!miss, "state wait: a miss is reported as not fresh");
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " bus check(s) failed\n";
    return 1;
  }
  std::cout << "bus checks OK (no hardware touched)\n";
  return 0;
}
