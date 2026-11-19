// test_can_poll_drain.cpp — MotorController::poll must drain the RX queue and
// let the newest status frame define the state (Python PR #86,
// tests/test_can_poll_drain.py).
//
// The motor answers every control frame with a status frame. Reading one frame
// per call left the rest queued; the age was stamped when a frame was READ, so
// an old position looked fresh. On 2026-11-18 a ramp after a force hold started
// from the hold's first pose (78 mm) while the jaws were at 26 mm.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "litegrip/can/controller.hpp"
#include "litegrip/can/protocol.hpp"
#include "litegrip/can/transport.hpp"

using litegrip::can::CanFrame;
using litegrip::can::CanIo;
using litegrip::can::float_to_uint;
using litegrip::can::kMaxDrainFrames;
using litegrip::can::MotorController;

namespace {

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

/// A transport that hands out pre-queued frames, then goes quiet.
class QueueTransport : public CanIo {
 public:
  explicit QueueTransport(std::vector<CanFrame> frames)
      : frames_(std::move(frames)) {}

  void set_id_filter(const std::vector<int>&) override {}
  void send(int, const std::uint8_t*, std::size_t, double) override {}
  void send_multi(int, const std::uint8_t*, std::size_t, int, double) override {}
  std::optional<CanFrame> recv(double) override {
    if (frames_.empty()) {
      return std::nullopt;
    }
    CanFrame frame = frames_.front();
    frames_.erase(frames_.begin());
    return frame;
  }
  void drain() override { frames_.clear(); }
  void close() override {}

 private:
  std::vector<CanFrame> frames_;
};

/// One DM status frame (q given; dq = tau = 0).
CanFrame status_frame(double position_rad, int err = 1, int can_id = 0x08,
                      int mst_id = 0x18) {
  const std::uint32_t q = float_to_uint(position_rad, -12.5, 12.5, 16);
  CanFrame frame;
  frame.can_id = mst_id;
  frame.dlc = 8;
  frame.data[0] = static_cast<std::uint8_t>(((err & 0x0F) << 4) |
                                            (can_id & 0x0F));
  frame.data[1] = static_cast<std::uint8_t>((q >> 8) & 0xFF);
  frame.data[2] = static_cast<std::uint8_t>(q & 0xFF);
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 30;
  frame.data[7] = 30;
  return frame;
}

void test_the_newest_queued_frame_wins() {
  QueueTransport transport({
      status_frame(-1.0),
      status_frame(-0.5),
      status_frame(-0.2),
  });
  MotorController controller(transport);
  auto& motor = controller.add_motor(0x08, 0x18);

  litegrip::can::MotorState* updated = controller.poll();

  check(updated == &motor, "drain: poll reports the motor");
  check(motor.rx_count() == 3, "drain: the queue was not drained");
  check_near(motor.position(), -0.2, 0.001,
             "drain: the OLDEST frame defined the state");
}

void test_foreign_frames_between_status_frames_do_not_stop_the_drain() {
  CanFrame foreign;
  foreign.can_id = 0x123;
  foreign.dlc = 8;
  QueueTransport transport({
      status_frame(-1.0),
      foreign,
      status_frame(-0.3),
  });
  MotorController controller(transport);
  auto& motor = controller.add_motor(0x08, 0x18);

  controller.poll();

  check(motor.rx_count() == 2, "foreign: two status frames were decoded");
  check_near(motor.position(), -0.3, 0.001,
             "foreign: the foreign frame stopped the drain");
}

void test_a_single_frame_is_still_reported() {
  QueueTransport transport({status_frame(0.25)});
  MotorController controller(transport);
  auto& motor = controller.add_motor(0x08, 0x18);

  litegrip::can::MotorState* updated = controller.poll();

  check(updated == &motor, "single: poll reports the motor");
  check(motor.rx_count() == 1, "single: one frame decoded");
  check_near(motor.position(), 0.25, 0.001, "single: position");
}

void test_an_empty_queue_returns_null() {
  QueueTransport transport({});
  MotorController controller(transport);
  controller.add_motor(0x08, 0x18);

  check(controller.poll() == nullptr, "empty: poll returns nullptr");
}

void test_poll_until_sees_the_target_update() {
  QueueTransport transport({
      status_frame(-1.0, 1, 0x08, 0x18),
      status_frame(-0.5, 1, 0x09, 0x19),
  });
  MotorController controller(transport);
  auto& target = controller.add_motor(0x08, 0x18);
  controller.add_motor(0x09, 0x19);

  check(controller.poll_until(target, 0.05),
        "multi-motor: the target's update was hidden by the newest frame");
  check(target.rx_count() == 1, "multi-motor: the target was decoded once");
}

void test_the_drain_is_bounded() {
  std::vector<CanFrame> frames;
  for (int i = 0; i < kMaxDrainFrames + 5; ++i) {
    frames.push_back(status_frame(0.0));
  }
  QueueTransport transport(std::move(frames));
  MotorController controller(transport);
  auto& motor = controller.add_motor(0x08, 0x18);

  controller.poll();
  check(motor.rx_count() == kMaxDrainFrames,
        "bound: the drain stopped at the cap");

  controller.poll();
  check(motor.rx_count() == kMaxDrainFrames + 5,
        "bound: the remainder was left for the next poll");
}

}  // namespace

int main() {
  test_the_newest_queued_frame_wins();
  test_foreign_frames_between_status_frames_do_not_stop_the_drain();
  test_a_single_frame_is_still_reported();
  test_an_empty_queue_returns_null();
  test_poll_until_sees_the_target_update();
  test_the_drain_is_bounded();

  if (g_failures == 0) {
    std::printf("test_can_poll_drain: all checks passed\n");
    return 0;
  }
  std::printf("test_can_poll_drain: %d failures\n", g_failures);
  return 1;
}
