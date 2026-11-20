// test_transport.cpp — classic-CAN SocketCAN transport checks that do NOT
// transmit.
//
// Deliberately non-intrusive: a live gripper (can0 up, carrier present) may be
// attached to this machine, so this test only ever
//   * exercises the error path for a missing interface,
//   * opens and closes a socket on a real CAN interface (bind only).
// It never calls send(), so it cannot disturb a running stack.
//
// The interface-dependent part is skipped (not failed) when can0 is absent, so
// the suite still works on machines without CAN hardware.

#include <net/if.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include "litegrip/can/transport.hpp"
#include "litegrip/exceptions.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

}  // namespace

int main() {
  // ── opening a missing interface must throw ConnectError ───────────────
  {
    litegrip::can::CanTransport transport("lg_no_such_iface");
    bool threw = false;
    try {
      transport.open();
    } catch (const litegrip::ConnectError&) {
      threw = true;
    } catch (const litegrip::LiteGripError&) {
      threw = true;
    }
    check(threw, "opening a missing interface throws");
    check(!transport.is_open(), "transport stays closed after a failed open");
  }

  // ── send before open must throw, not corrupt anything ─────────────────
  {
    litegrip::can::CanTransport transport("can0");
    bool threw = false;
    try {
      const std::uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      transport.send(0x08, payload, sizeof(payload));
    } catch (const litegrip::LiteGripError&) {
      threw = true;
    }
    check(threw, "send on a closed transport throws");
  }

  // ── open/close a real CAN interface (bind only, no traffic) ───────────
  const bool can0_present = ::if_nametoindex("can0") != 0;
  if (!can0_present) {
    std::cout << "can0 not present; interface open/close check skipped\n";
  } else {
    litegrip::can::CanTransport transport("can0");
    transport.open();
    check(transport.is_open(), "can0 opens");
    check(transport.channel() == "can0", "channel name preserved");
    transport.close();
    check(!transport.is_open(), "can0 closes");

    // Re-opening must be idempotent rather than leaking a socket.
    transport.open();
    transport.open();
    check(transport.is_open(), "re-open is idempotent");
    transport.close();
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " transport check(s) failed\n";
    return 1;
  }
  std::cout << "transport checks OK (no frames transmitted)\n";
  return 0;
}
