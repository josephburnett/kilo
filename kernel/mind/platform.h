// platform: the seams between the portable mind core and what it runs on.
// The Circle kernel implements them over the PL011, the system timer and its
// logger; kernel/host/ implements them with POSIX (a tty or pty,
// clock_gettime, stderr), so the same core runs against the virtual body on
// the laptop.
//
// The core never blocks: every call here returns at once, except
// KernelStore's (see there), which the core makes only while the link is
// paused. No virtual
// destructors (a deleting destructor would pull in operator delete): the
// platform owns its objects for the life of the program.
#ifndef KILO_MIND_PLATFORM_H
#define KILO_MIND_PLATFORM_H
#include <stdint.h>

namespace kilo {

// A byte stream (the body link's UART). Python counterpart:
// kilo/link/transport.py's Transport, minus the blocking read timeout.
class Serial {
 public:
  // Up to n bytes into buf: the count, 0 when nothing has arrived, -1 when
  // the device failed (a pty whose other end is gone).
  virtual int32_t read(uint8_t* buf, uint32_t n) = 0;
  // Queue up to n bytes: the count taken (short when the TX buffer is full;
  // the link counts that), -1 when the device failed.
  virtual int32_t write(const uint8_t* buf, uint32_t n) = 0;
  // Bytes write() takes whole right now. The link writes a frame only when
  // this covers all of it, so a full buffer defers the frame instead of
  // cutting it. A device that cannot see its buffer returns an estimate;
  // write() coming up short anyway is still counted (tx_short).
  virtual uint32_t tx_free() = 0;
  // Line rate in baud, 8N1. False if the device cannot do it.
  virtual bool set_baud(uint32_t baud) = 0;

 protected:
  ~Serial() = default;
};

// Monotonic microseconds since some fixed point (boot).
class Clock {
 public:
  virtual uint64_t now_us() = 0;

 protected:
  ~Clock() = default;
};

enum class LogLevel : uint8_t { Debug = 0, Info = 1, Warn = 2, Error = 3 };

// A log sink: one preformatted line per call, no trailing newline.
// util/log.h formats into it.
class Log {
 public:
  virtual void write(LogLevel level, const char* line) = 0;

 protected:
  ~Log() = default;
};

// The body's RESET line: GPIO4 on the Pi, an emulated open drain (only ever
// driven low or released, never driven high; mind/kilo/tools/gpio.py). For
// flashing the body (docs/fast-boot.md P4); nothing in the link uses it.
class Gpio {
 public:
  virtual void reset_assert() = 0;   // drive low: the 328P is held in reset
  virtual void reset_release() = 0;  // input: the board's pull-up lets it run

 protected:
  ~Gpio() = default;
};

// Random bytes fit for keys: the host channel's ephemeral Noise keys
// (net/host_server.h). The Circle kernel's come from the SoC's hardware RNG
// (CBcmRandomNumberGenerator, hashed), kilo-mind-host's from getrandom().
// No other part of the core needs it (the behaviour RNG is seeded, not secret).
class Random {
 public:
  // n bytes into buf; false if the source failed or looks broken: then
  // buf must not be used (the server refuses the handshake: fail closed).
  virtual bool fill(uint8_t* buf, uint32_t n) = 0;

 protected:
  ~Random() = default;
};

// The SD card's FAT partition, for the mind's own kernel image
// (update/updater.h: kernel7.img, kernel7.new, kernel7.old, bare names).
// One file open at a time. Unlike the seams above these calls may block for
// as long as the card takes (the Circle kernel's SD driver busy-waits): the
// updater makes them only while the link is paused, a bounded slice per
// pass. kilo-mind-host implements it on a directory (--card DIR).
class KernelStore {
 public:
  enum class Result : uint8_t { Ok, Missing, Failed };

  // Open name for writing, created or emptied.
  virtual Result create(const char* name) = 0;
  // Open name for reading; Missing if there is none.
  virtual Result open(const char* name) = 0;
  // All len bytes onto the open file, or Failed.
  virtual Result write(const uint8_t* data, uint32_t len) = 0;
  // Up to len bytes of the open file: the count, 0 at its end, -1 failed.
  virtual int32_t read(uint8_t* buf, uint32_t len) = 0;
  // Close the open file. After writing, Ok means its data are on the card.
  virtual Result close() = 0;
  // Missing if there is none.
  virtual Result remove(const char* name) = 0;
  // from must exist (else Missing); to must not.
  virtual Result rename(const char* from, const char* to) = 0;
  // Directory and allocation table changes are on the card.
  virtual Result sync() = 0;

 protected:
  ~KernelStore() = default;
};

}  // namespace kilo

#endif
