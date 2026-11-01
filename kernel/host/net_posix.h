// net_posix: the network seams (mind/platform_net.h) with POSIX TCP sockets,
// so kilo-mind-host serves the laptop channel (mind/net/host_server.h) to
// `kilo --host 127.0.0.1:PORT` on the same machine. Non-blocking throughout.
#ifndef KILO_HOST_NET_POSIX_H
#define KILO_HOST_NET_POSIX_H
#include "platform_net.h"

namespace kilo {

class PosixConn final : public Conn {
 public:
  int32_t read(uint8_t* buf, uint32_t n) override;
  int32_t write(const uint8_t* buf, uint32_t n) override;
  void close() override;
  bool is_open() const override { return fd_ >= 0; }

  int fd() const { return fd_; }
  void adopt(int fd) { fd_ = fd; }

 private:
  int fd_ = -1;
};

class PosixListener final : public Listener {
 public:
  static constexpr uint32_t kSlots = 4;

  // Bind and listen on spec "[addr:]port" (addr default 127.0.0.1; port 0 =
  // any free one, see port()). False and error() on failure.
  bool open(const char* spec);
  void close();
  const char* error() const { return error_; }
  uint16_t port() const { return port_; }
  int fd() const { return fd_; }

  int32_t accept() override;
  Conn* conn(uint32_t slot) override { return &conns_[slot]; }
  uint32_t slots() const override { return kSlots; }

 private:
  int fd_ = -1;
  uint16_t port_ = 0;
  PosixConn conns_[kSlots];
  char error_[128] = {};
};

}  // namespace kilo

#endif
