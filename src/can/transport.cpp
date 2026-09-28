// transport.cpp — SocketCAN transport.
//
// Port of litegrip_driver/litegrip/can/transport.py. Uses the kernel's own
// can_frame / canfd_frame layouts (<linux/can.h>) instead of the Python
// original's hand-rolled struct formats — same wire layout, less to get wrong.
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
#include <sys/ioctl.h>
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

std::string describe_interface(const std::string& channel) {
  return "CAN interface '" + channel + "'";
}

}  // namespace

std::optional<int> iface_mtu(int fd, const std::string& iface) noexcept {
  if (fd < 0) {
    return std::nullopt;
  }
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFMTU, &ifr) < 0) {
    return std::nullopt;
  }
  return ifr.ifr_mtu;
}

CanTransport::CanTransport(std::string channel, CanMode mode)
    : channel_(std::move(channel)), mode_(mode) {}

CanTransport::~CanTransport() { close(); }

CanTransport::CanTransport(CanTransport&& other) noexcept
    : channel_(std::move(other.channel_)),
      mode_(other.mode_),
      fd_(other.fd_),
      accept_ids_(std::move(other.accept_ids_)) {
  other.fd_ = -1;
}

CanTransport& CanTransport::operator=(CanTransport&& other) noexcept {
  if (this != &other) {
    close();
    channel_ = std::move(other.channel_);
    mode_ = other.mode_;
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

  // Some USB-CAN adapters (gs_usb) and kernel versions need CAN FD socket mode
  // on FD-capable interfaces; a classic CAN socket on such an interface may
  // silently drop frames. Reconcile the requested mode with the interface MTU.
  if (const auto mtu = iface_mtu(fd, channel_)) {
    if (*mtu >= kCanFdMtu && mode_ == CanMode::kCan) {
      mode_ = CanMode::kCanFd;
    } else if (*mtu < kCanFdMtu && mode_ == CanMode::kCanFd) {
      std::fprintf(stderr,
                   "[litegrip] %s has classic-CAN MTU %d but CAN FD was "
                   "requested; using classic CAN.\n",
                   describe_interface(channel_).c_str(), *mtu);
      mode_ = CanMode::kCan;
    }
  }

  if (mode_ == CanMode::kCanFd) {
    const int on = 1;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
      std::fprintf(stderr,
                   "[litegrip] failed to enable CAN FD on %s (%s); falling "
                   "back to classic CAN.\n",
                   describe_interface(channel_).c_str(), errno_text().c_str());
      mode_ = CanMode::kCan;
    }
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

void CanTransport::add_id_filter(int can_id) {
  std::vector<int> ids = accept_ids_;
  ids.push_back(can_id);
  set_id_filter(ids);
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

  const void* frame = nullptr;
  std::size_t frame_len = 0;
  struct can_frame classic{};
  struct canfd_frame fd_frame{};

  if (mode_ == CanMode::kCanFd) {
    if (size > sizeof(fd_frame.data)) {
      throw CommandError("CAN FD frames hold at most 64 bytes");
    }
    fd_frame.can_id = static_cast<canid_t>(can_id) & CAN_EFF_MASK;
    fd_frame.len = static_cast<__u8>(size);
    if (size > 0) {
      std::memcpy(fd_frame.data, data, size);
    }
    frame = &fd_frame;
    frame_len = CANFD_MTU;
  } else {
    if (size > sizeof(classic.data)) {
      throw CommandError("classic CAN frames hold at most 8 bytes");
    }
    classic.can_id = static_cast<canid_t>(can_id) & CAN_SFF_MASK;
    classic.can_dlc = static_cast<__u8>(size);
    if (size > 0) {
      std::memcpy(classic.data, data, size);
    }
    frame = &classic;
    frame_len = CAN_MTU;
  }

  // The socket is non-blocking: retry while the send buffer is full.
  const double deadline = monotonic_now() + timeout_s;
  const char* bytes = static_cast<const char*>(frame);
  while (true) {
    const ssize_t written = ::write(fd_, bytes, frame_len);
    if (written >= 0) {
      return;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      throw CommError("CAN send failed on " + channel_ + ": " + errno_text());
    }
    const double remaining = deadline - monotonic_now();
    if (remaining <= 0.0) {
      std::fprintf(stderr,
                   "[litegrip] CAN send timeout on %s (buffer full >%.1fs), "
                   "frame to 0x%03X dropped\n",
                   channel_.c_str(), timeout_s, can_id & CAN_SFF_MASK);
      return;
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

  std::uint8_t buffer[CANFD_MTU] = {};
  const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
  if (n < 0) {
    return std::nullopt;
  }

  CanFrame out;
  out.timestamp = monotonic_now();

  if (mode_ == CanMode::kCanFd) {
    if (n < static_cast<ssize_t>(sizeof(struct canfd_frame))) {
      return std::nullopt;
    }
    struct canfd_frame fd_frame{};
    std::memcpy(&fd_frame, buffer, sizeof(fd_frame));
    out.is_extended = (fd_frame.can_id & CAN_EFF_FLAG) != 0;
    out.is_fd = true;
    out.can_id = static_cast<int>(fd_frame.can_id & CAN_SFF_MASK);
    out.dlc = fd_frame.len > sizeof(out.data) ? sizeof(out.data) : fd_frame.len;
    std::memcpy(out.data.data(), fd_frame.data, out.dlc);
  } else {
    if (n < static_cast<ssize_t>(sizeof(struct can_frame))) {
      return std::nullopt;
    }
    struct can_frame classic{};
    std::memcpy(&classic, buffer, sizeof(classic));
    out.is_extended = (classic.can_id & CAN_EFF_FLAG) != 0;
    out.is_fd = false;
    out.can_id = static_cast<int>(classic.can_id & CAN_SFF_MASK);
    out.dlc = classic.can_dlc > sizeof(out.data) ? sizeof(out.data)
                                                 : classic.can_dlc;
    std::memcpy(out.data.data(), classic.data, out.dlc);
  }
  return out;
}

void CanTransport::drain() {
  while (recv(0.0).has_value()) {
  }
}

}  // namespace litegrip::can
