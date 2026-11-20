// transport.cpp — classic-CAN SocketCAN transport.
//
// Port of litegrip_driver/litegrip/can/transport.py. Uses the kernel's own
// can_frame layout (<linux/can.h>) instead of the Python original's
// hand-rolled struct format — same wire layout, less to get wrong.
//
// One deliberate divergence: the Python code computed
// `is_extended = bool(can_id & CAN_EFF_MASK)` (CAN_EFF_MASK = 0x1FFFFFFF),
// which is true for *every* standard frame. That value is never consumed by
// the SDK, so rather than port a latent bug the canonical kernel flag
// CAN_EFF_FLAG is used here.

#include "litegrip/can/transport.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <utility>

#include "litegrip/exceptions.hpp"

namespace litegrip::can {
namespace {

double monotonic_now() noexcept {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string errno_text() { return std::strerror(errno); }

/// The 11-bit id as the log lines print it ("0x08").
std::string hex_id(int can_id) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%03X",
                static_cast<unsigned>(can_id) & CAN_SFF_MASK);
  return buf;
}

std::string describe_interface(const std::string& channel) {
  return "CAN interface '" + channel + "'";
}

}  // namespace

CanTransport::CanTransport(std::string channel) : channel_(std::move(channel)) {}

CanTransport::~CanTransport() { close(); }

CanTransport::CanTransport(CanTransport&& other) noexcept
    : channel_(std::move(other.channel_)),
      fd_(other.fd_),
      accept_ids_(std::move(other.accept_ids_)) {
  other.fd_ = -1;
}

CanTransport& CanTransport::operator=(CanTransport&& other) noexcept {
  if (this != &other) {
    close();
    channel_ = std::move(other.channel_);
    fd_ = other.fd_;
    accept_ids_ = std::move(other.accept_ids_);
    other.fd_ = -1;
  }
  return *this;
}

void CanTransport::open() {
  if (is_open()) {
    return;
  }

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw ConnectError("socket(PF_CAN) failed: " + errno_text());
  }

  const unsigned int ifindex = ::if_nametoindex(channel_.c_str());
  if (ifindex == 0) {
    ::close(fd);
    throw ConnectError(describe_interface(channel_) + " not found");
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = static_cast<int>(ifindex);
  if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    const std::string why = errno_text();
    ::close(fd);
    throw ConnectError("bind(" + describe_interface(channel_) +
                       ") failed: " + why);
  }

  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }

  fd_ = fd;
  if (!accept_ids_.empty()) {
    apply_id_filter();
  }
}

void CanTransport::close() {
  if (fd_ < 0) {
    return;
  }
  ::close(fd_);
  fd_ = -1;
}

void CanTransport::set_id_filter(const std::vector<int>& can_ids) {
  accept_ids_.clear();
  for (const int id : can_ids) {
    const int masked = id & CAN_SFF_MASK;
    bool seen = false;
    for (const int existing : accept_ids_) {
      if (existing == masked) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      accept_ids_.push_back(masked);
    }
  }
  if (is_open()) {
    apply_id_filter();
  }
}

void CanTransport::apply_id_filter() {
  if (fd_ < 0) {
    return;
  }

  // An empty filter list means "receive nothing" to SocketCAN, so removing the
  // filter must be done with a single match-all entry instead.
  std::vector<can_filter> filters;
  if (!accept_ids_.empty()) {
    filters.reserve(accept_ids_.size());
    for (const int id : accept_ids_) {
      filters.push_back(can_filter{static_cast<canid_t>(id),
                                   static_cast<canid_t>(CAN_SFF_MASK)});
    }
  } else {
    filters.push_back(can_filter{0, 0});
  }

  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                   static_cast<socklen_t>(filters.size() *
                                          sizeof(can_filter))) < 0) {
    std::fprintf(stderr, "[litegrip] failed to set CAN RX filter on %s: %s\n",
                 channel_.c_str(), errno_text().c_str());
  }
}

void CanTransport::send(int can_id, const std::uint8_t* data, std::size_t size,
                        double timeout_s) {
  if (!is_open()) {
    throw CommError("transport not open");
  }
  if (data == nullptr && size > 0) {
    throw CommandError("null payload with non-zero size");
  }
  if (size > sizeof(can_frame::data)) {
    throw CommandError("classic CAN frames hold at most 8 bytes");
  }

  struct can_frame frame{};
  frame.can_id = static_cast<canid_t>(can_id) & CAN_SFF_MASK;
  frame.can_dlc = static_cast<__u8>(size);
  if (size > 0) {
    std::memcpy(frame.data, data, size);
  }
  const char* bytes = reinterpret_cast<const char*>(&frame);

  // The socket is non-blocking: retry while the send buffer is full.
  const double deadline = monotonic_now() + timeout_s;
  while (true) {
    const ssize_t written = ::write(fd_, bytes, CAN_MTU);
    if (written >= 0) {
      return;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      throw CommError("CAN send failed on " + channel_ + ": " + errno_text());
    }
    const double remaining = deadline - monotonic_now();
    if (remaining <= 0.0) {
      // A full buffer means this frame did NOT go out. Returning quietly made
      // the caller treat it as sent — on the bench that chain is what produced
      // "moved 0.000 mm" reported as success.
      std::fprintf(stderr,
                   "[litegrip] CAN send timeout on %s (buffer full >%.1fs), "
                   "frame to 0x%03X not sent\n",
                   channel_.c_str(), timeout_s, can_id & CAN_SFF_MASK);
      throw CommError("CAN send timeout on " + channel_ +
                      ": the send buffer stayed full for more than " +
                      std::to_string(timeout_s) + " s, so the frame to 0x" +
                      hex_id(can_id) + " was not sent");
    }
    struct pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLOUT;
    ::poll(&pfd, 1, static_cast<int>(remaining * 1000.0) + 1);
  }
}

void CanTransport::send_multi(int can_id, const std::uint8_t* data,
                              std::size_t size, int count, double interval_s) {
  for (int i = 0; i < count; ++i) {
    send(can_id, data, size);
    if (count > 1 && interval_s > 0.0) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(interval_s));
    }
  }
}

std::optional<CanFrame> CanTransport::recv(double timeout_s) {
  if (!is_open()) {
    return std::nullopt;
  }

  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  const int timeout_ms =
      timeout_s <= 0.0 ? 0 : static_cast<int>(timeout_s * 1000.0) + 1;
  const int ready = ::poll(&pfd, 1, timeout_ms);
  if (ready <= 0) {
    return std::nullopt;
  }

  std::uint8_t buffer[CAN_MTU] = {};
  const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
  if (n < static_cast<ssize_t>(sizeof(struct can_frame))) {
    return std::nullopt;
  }

  struct can_frame frame{};
  std::memcpy(&frame, buffer, sizeof(frame));

  CanFrame out;
  out.timestamp = monotonic_now();
  out.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  out.can_id = static_cast<int>(frame.can_id & CAN_SFF_MASK);
  out.dlc = frame.can_dlc > sizeof(out.data) ? sizeof(out.data) : frame.can_dlc;
  std::memcpy(out.data.data(), frame.data, out.dlc);
  return out;
}

void CanTransport::drain() {
  while (recv(0.0).has_value()) {
  }
}

}  // namespace litegrip::can
