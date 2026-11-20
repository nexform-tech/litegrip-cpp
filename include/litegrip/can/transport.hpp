// litegrip/can/transport.hpp — raw SocketCAN transport (classic CAN).
//
// Port of litegrip_driver/litegrip/can/transport.py. Nothing but Linux
// SocketCAN and the C++ standard library; no libsocketcan, no third-party
// dependency.
//
// The DM4310 protocol is classic CAN (8-byte frames), and this SDK is classic
// CAN only: no CAN FD socket mode, no FD frame layout.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace litegrip::can {

/// A single classic CAN frame.
struct CanFrame {
  int can_id = 0;
  std::array<std::uint8_t, 8> data{};
  std::size_t dlc = 0;
  bool is_extended = false;
  double timestamp = 0.0;  // monotonic seconds

  const std::uint8_t* bytes() const noexcept { return data.data(); }
};

/// The transport surface MotorController drives. CanTransport is the real
/// SocketCAN implementation; tests substitute a queue-backed fake, exactly as
/// the Python suite's duck-typed transport does.
///
/// Kept to the operations the controller performs, and called from one thread
/// at a time, like CanTransport itself.
class CanIo {
 public:
  virtual ~CanIo() = default;

  virtual void set_id_filter(const std::vector<int>& can_ids) = 0;
  virtual void send(int can_id, const std::uint8_t* data, std::size_t size,
                    double timeout_s = 0.1) = 0;
  virtual void send_multi(int can_id, const std::uint8_t* data,
                          std::size_t size, int count,
                          double interval_s = 0.002) = 0;
  virtual std::optional<CanFrame> recv(double timeout_s = 0.0) = 0;
  virtual void drain() = 0;
  virtual void close() = 0;
};

/// SocketCAN transport: opens one CAN socket and provides send/recv.
///
/// Not thread-safe. `recv()` may be called from a control loop; `send()` from
/// the same thread.
class CanTransport : public CanIo {
 public:
  explicit CanTransport(std::string channel);
  ~CanTransport();

  CanTransport(const CanTransport&) = delete;
  CanTransport& operator=(const CanTransport&) = delete;
  CanTransport(CanTransport&& other) noexcept;
  CanTransport& operator=(CanTransport&& other) noexcept;

  /// Open and bind the CAN socket. Throws ConnectError if the interface is
  /// missing or down.
  void open();

  void close() override;
  bool is_open() const noexcept { return fd_ >= 0; }

  const std::string& channel() const noexcept { return channel_; }

  /// Restrict which CAN ids the kernel delivers to this socket.
  ///
  /// On a shared bus (e.g. a full arm) other devices chatter at high rate;
  /// without a hardware filter every foreign frame lands in this socket's RX
  /// buffer, a one-frame-per-cycle reader falls behind, and the motor's own
  /// status frames are delayed or dropped — making read-back appear frozen
  /// while the motor is actually moving. Empty clears the filter.
  void set_id_filter(const std::vector<int>& can_ids) override;

  /// Send one frame. Retries on EAGAIN until `timeout_s`.
  ///
  /// Throws CommError when the send buffer stays full for that whole time: the
  /// frame did NOT go out, and a caller that cannot send has to find out or it
  /// reports motion that never happened.
  void send(int can_id, const std::uint8_t* data, std::size_t size,
            double timeout_s = 0.1) override;

  /// Send the same frame `count` times, spacing by `interval_s`.
  /// DM motors need repeated config commands for reliability.
  void send_multi(int can_id, const std::uint8_t* data, std::size_t size,
                  int count, double interval_s = 0.002) override;

  /// Receive one frame; nullopt on timeout. `timeout_s == 0` = non-blocking.
  std::optional<CanFrame> recv(double timeout_s = 0.0) override;

  /// Read and discard everything pending.
  void drain() override;

 private:
  void apply_id_filter();

  std::string channel_;
  int fd_ = -1;
  std::vector<int> accept_ids_;
};

}  // namespace litegrip::can
