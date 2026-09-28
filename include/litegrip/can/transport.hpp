// litegrip/can/transport.hpp — raw SocketCAN transport.
//
// Port of litegrip_driver/litegrip/can/transport.py. Nothing but Linux
// SocketCAN and the C++ standard library; no libsocketcan, no third-party
// dependency.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace litegrip::can {

/// CAN interface mode.
enum class CanMode : int {
  kCan = 0,    // classic CAN, 8-byte payload
  kCanFd = 1,  // CAN FD, up to 64-byte payload
};

/// Classic CAN frame MTU / CAN FD frame MTU, as reported by SIOCGIFMTU.
inline constexpr int kCanMtu = 16;
inline constexpr int kCanFdMtu = 72;

/// A single CAN frame.
struct CanFrame {
  int can_id = 0;
  std::array<std::uint8_t, 64> data{};
  std::size_t dlc = 0;
  bool is_extended = false;
  bool is_fd = false;
  double timestamp = 0.0;  // monotonic seconds

  const std::uint8_t* bytes() const noexcept { return data.data(); }
};

/// SocketCAN transport: opens one CAN socket and provides send/recv.
///
/// Not thread-safe. `recv()` may be called from a control loop; `send()` from
/// the same thread.
class CanTransport {
 public:
  /// `mode` is a *preference*: open() reconciles it with the interface MTU
  /// (classic CAN sockets on FD-capable interfaces may silently drop frames).
  explicit CanTransport(std::string channel, CanMode mode = CanMode::kCan);
  ~CanTransport();

  CanTransport(const CanTransport&) = delete;
  CanTransport& operator=(const CanTransport&) = delete;
  CanTransport(CanTransport&& other) noexcept;
  CanTransport& operator=(CanTransport&& other) noexcept;

  /// Open and bind the CAN socket. Throws ConnectError if the interface is
  /// missing or down.
  void open();

  void close();
  bool is_open() const noexcept { return fd_ >= 0; }

  const std::string& channel() const noexcept { return channel_; }
  CanMode mode() const noexcept { return mode_; }

  /// Restrict which CAN ids the kernel delivers to this socket.
  ///
  /// On a shared bus (e.g. a full arm) other devices chatter at high rate;
  /// without a hardware filter every foreign frame lands in this socket's RX
  /// buffer, a one-frame-per-cycle reader falls behind, and the motor's own
  /// status frames are delayed or dropped — making read-back appear frozen
  /// while the motor is actually moving. Empty clears the filter.
  void set_id_filter(const std::vector<int>& can_ids);
  void add_id_filter(int can_id);

  /// Send one frame. Retries on EAGAIN until `timeout_s`; on exhaustion the
  /// frame is dropped and a warning is logged (matching the Python original).
  void send(int can_id, const std::uint8_t* data, std::size_t size,
            double timeout_s = 0.1);

  /// Send the same frame `count` times, spacing by `interval_s`.
  /// DM motors need repeated config commands for reliability.
  void send_multi(int can_id, const std::uint8_t* data, std::size_t size,
                  int count, double interval_s = 0.002);

  /// Receive one frame; nullopt on timeout. `timeout_s == 0` = non-blocking.
  std::optional<CanFrame> recv(double timeout_s = 0.0);

  /// Read and discard everything pending.
  void drain();

 private:
  void apply_id_filter();

  std::string channel_;
  CanMode mode_;
  int fd_ = -1;
  std::vector<int> accept_ids_;
};

/// Read interface MTU (kCanMtu / kCanFdMtu); nullopt on failure.
std::optional<int> iface_mtu(int fd, const std::string& iface) noexcept;

}  // namespace litegrip::can
