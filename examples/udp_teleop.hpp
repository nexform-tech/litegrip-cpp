// udp_teleop.hpp — tiny UDP helper shared by the teleoperation examples.
//
// Examples are allowed to use the standard library and POSIX; the SDK itself
// stays dependency-free.  IPv4 literals only (127.0.0.1 by default).

#pragma once

#include <arpa/inet.h>
#include <endian.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

namespace example {

class UdpSocket {
 public:
  explicit UdpSocket(int port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
    }
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) != 0) {
      const std::string error = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("bind: " + error);
    }
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }

  ~UdpSocket() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  bool send_double(const std::string& host, int port, double value) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
      return false;
    }
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    bits = htobe64(bits);
    return ::sendto(fd_, &bits, sizeof(bits), 0,
                    reinterpret_cast<const sockaddr*>(&address),
                    sizeof(address)) == static_cast<ssize_t>(sizeof(bits));
  }

  /// One datagram, or nullopt when none is queued / the value is not finite.
  std::optional<double> recv_double() {
    std::uint64_t bits = 0;
    const ssize_t received =
        ::recvfrom(fd_, &bits, sizeof(bits), 0, nullptr, nullptr);
    if (received != static_cast<ssize_t>(sizeof(bits))) {
      return std::nullopt;
    }
    bits = be64toh(bits);
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
    return value;
  }

 private:
  int fd_ = -1;
};

}  // namespace example
