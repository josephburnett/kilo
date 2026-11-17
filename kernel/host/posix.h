// posix: the mind's platform seams (mind/platform.h) on Linux, so the
// portable core runs on the laptop against the virtual body or a real
// NyBoard on a USB adapter.
#ifndef KILO_HOST_POSIX_H
#define KILO_HOST_POSIX_H
#include <stddef.h>
#include "net/authorized_keys.h"
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

// The body's RESET line on the laptop (docs/fast-boot.md P4). There is no
// GPIO4 here: with a path (kilo-mind-host --reset-fifo PATH) each edge is
// one byte written to that named pipe, 'L' for reset_assert() and 'H' for
// reset_release(), which a fake body reads
// (kernel/host/tests/fake_optiboot.py). Without one, or when nothing reads
// the pipe, the edge is only logged. The pipe is opened non-blocking at
// each edge until it opens, so the reader may start after the mind.
class FifoResetGpio final : public Gpio {
 public:
  FifoResetGpio(const char* path, Log& log) : path_(path), log_(log) {}
  ~FifoResetGpio();
  void reset_assert() override { edge('L'); }
  void reset_release() override { edge('H'); }

 private:
  void edge(char level);
  const char* path_;
  Log& log_;
  int fd_ = -1;
};

// The SD card's FAT partition on the laptop (kilo-mind-host --card DIR): the
// kernel updater's files (kernel7.img, kernel7.new, kernel7.old) in a
// directory, so the integration test can check what an update leaves there.
// close() after writing fsyncs the file, sync() the directory.
class DirStore final : public KernelStore {
 public:
  DirStore(const char* dir, Log& log) : dir_(dir), log_(log) {}
  ~DirStore();
  Result create(const char* name) override;
  Result open(const char* name) override;
  Result write(const uint8_t* data, uint32_t len) override;
  int32_t read(uint8_t* buf, uint32_t len) override;
  Result close() override;
  Result remove(const char* name) override;
  Result rename(const char* from, const char* to) override;
  Result sync() override;

 private:
  void path(char* out, size_t cap, const char* name) const;
  Result fail(const char* what, const char* name);
  const char* dir_;
  Log& log_;
  int fd_ = -1;
  bool writing_ = false;
};

// --card DIR for the flight recorder's saver (mind/platform.h RecordCard):
// paths "kilorec/NAME" under DIR.
class DirRecordCard final : public RecordCard {
 public:
  DirRecordCard(const char* dir, Log& log) : dir_(dir), log_(log) {}
  ~DirRecordCard();
  bool ready() override { return true; }
  Result make_dir(const char* path) override;
  Result create(const char* path) override;
  Result append(const char* path) override;
  Result open(const char* path) override;
  Result seek(uint32_t offset) override;
  Result write(const uint8_t* data, uint32_t len) override;
  int32_t read(uint8_t* buf, uint32_t len) override;
  Result sync() override;
  Result close() override;
  Result remove(const char* path) override;
  Result rename(const char* from, const char* to) override;
  Result list_begin(const char* dir) override;
  Result list_next(char* name, uint32_t n, uint32_t* size) override;

 private:
  void path(char* out, size_t cap, const char* name) const;
  Result open_mode(const char* name, int flags);
  const char* dir_;
  Log& log_;
  int fd_ = -1;
  void* dirp_ = nullptr;
  char list_dir_[512] = {};
};

// The host channel's ephemeral keys (mind/platform.h Random): getrandom().
class PosixRandom final : public Random {
 public:
  bool fill(uint8_t* buf, uint32_t n) override;
};

// kilo-mind-host --authorized-keys FILE: the laptops' public keys, in the
// format of kernel/keys/authorized_keys (which kernel/keys/gen_keys.py
// compiles into the Circle kernel): per line, the 32-byte X25519 public key
// in base64 (44 characters), then optionally its name; blank lines and
// lines starting with # are skipped. At most max keys into keys; false and
// a message in error on a bad line or an unreadable file.
bool load_authorized_keys(const char* path, AuthorizedKey* keys, uint32_t max, uint32_t* n, char* error,
                          size_t cap);

}  // namespace kilo

#endif
