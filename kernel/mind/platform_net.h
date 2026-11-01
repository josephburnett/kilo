// platform_net: the network seams for the laptop interface (docs/fast-boot.md
// §5, net/host_server.h). The Circle kernel implements them over its TCP/IP
// stack; kernel/host/net_posix.h implements them with POSIX sockets, so the
// same server runs on the laptop against the virtual body.
//
// Same rules as platform.h: every call returns at once (a platform whose
// sockets block, as Circle's do, buffers in a task of its own and serves
// these calls from the buffers), and no virtual destructors.
//
// Addressing is the platform's: the server only sees connections. On the Pi
// the kernel asks DHCP for KILO_HOST_DHCP_TIMEOUT_MS and otherwise takes the
// fixed link-local KILO_HOST_FALLBACK_IP_INIT/KILO_HOST_FALLBACK_PREFIX
// (protocol/spec.toml [host]); on the laptop it is the bound address and port.
#ifndef KILO_MIND_PLATFORM_NET_H
#define KILO_MIND_PLATFORM_NET_H
#include <stdint.h>

namespace kilo {

// One accepted TCP connection: a byte stream to one laptop client.
class Conn {
 public:
  // Up to n bytes into buf: the count, 0 when nothing has arrived, -1 when
  // the peer closed or the connection failed (then close() it).
  virtual int32_t read(uint8_t* buf, uint32_t n) = 0;
  // Queue up to n bytes: the count taken (short or 0 when the send buffer is
  // full; the caller keeps the rest), -1 when the connection failed.
  virtual int32_t write(const uint8_t* buf, uint32_t n) = 0;
  // Close and free the slot for the next accept().
  virtual void close() = 0;
  virtual bool is_open() const = 0;

 protected:
  ~Conn() = default;
};

// The listening socket and a fixed set of connection slots.
class Listener {
 public:
  // A connection that arrived since the last call: its slot index
  // (0..slots()-1), now open; -1 when none is waiting. One that arrives
  // while every slot is open is closed by the listener at once.
  virtual int32_t accept() = 0;
  // The slot's connection (open or not), never null for slot < slots().
  virtual Conn* conn(uint32_t slot) = 0;
  virtual uint32_t slots() const = 0;

 protected:
  ~Listener() = default;
};

}  // namespace kilo

#endif
