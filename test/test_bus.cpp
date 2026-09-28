// test_bus.cpp — GripperBus lifecycle, error paths and the HoldPolicy seam.
//
// Hardware-dependent paths (connect / init / motion) are NOT exercised here:
// they need the real device. What is covered is the behaviour that must hold
// without one — construction, the "not connected" refusals, config plumbing,
// and the pluggable hold policy (R2).

#include <iostream>
#include <memory>
#include <string>

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

  if (g_failures != 0) {
    std::cerr << g_failures << " bus check(s) failed\n";
    return 1;
  }
  std::cout << "bus checks OK (no hardware touched)\n";
  return 0;
}
