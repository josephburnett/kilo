// posix: the mind's platform seams (mind/platform.h) on Linux, so the
// portable core runs on the laptop against the virtual body or a real
// NyBoard on a USB adapter.
#ifndef KILO_HOST_POSIX_H
#define KILO_HOST_POSIX_H
#include "platform.h"

namespace kilo {

// A tty or pty, raw 8N1, non-blocking. Python counterpart: FdTransport and
// SerialTransport in kilo/link/transport.py.
class PosixSerial final : public Serial {
 public:
  // Open path. A real tty also gets HUPCL cleared (closing must not drop
  // DTR: on the NyBoard every DTR edge resets the 328P) and its stale input
  // flushed; a pty keeps what the virtual body wrote before the open, as
  // FdTransport does. Returns false and sets error() on failure.
  bool open(const char* path);
  void close();
  const char* error() const { return error_; }
  int fd() const { return fd_; }

  int32_t read(uint8_t* buf, uint32_t n) override;
  int32_t write(const uint8_t* buf, uint32_t n) override;
  // 4 KiB less what TIOCOUTQ says is queued: an estimate (see posix.cpp).
  uint32_t tx_free() override;
  // termios2 BOTHER: 250000 is no Bxxx constant. A pty takes the setting and ignores it.
  bool set_baud(uint32_t baud) override;

 private:
  int fd_ = -1;
  char error_[128] = {};
};

class MonotonicClock final : public Clock {
 public:
  uint64_t now_us() override;
};

// Lines to stderr with the time since start: "   1.234 I HELLO proto ...".
class StderrLog final : public Log {
 public:
  StderrLog(Clock& clock, LogLevel min) : clock_(clock), min_(min) {}
  void write(LogLevel level, const char* line) override;

 private:
  Clock& clock_;
  LogLevel min_;
  uint64_t start_us_ = 0;
  bool started_ = false;
};

}  // namespace kilo

#endif
