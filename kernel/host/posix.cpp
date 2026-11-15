#include "posix.h"
#include "util/log.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace kilo {

bool PosixSerial::open(const char* path) {
  fd_ = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    snprintf(error_, sizeof error_, "open %s: %s", path, strerror(errno));
    return false;
  }
  struct termios t;
  if (tcgetattr(fd_, &t) != 0) {
    snprintf(error_, sizeof error_, "%s is not a tty: %s", path, strerror(errno));
    close();
    return false;
  }
  cfmakeraw(&t);  // 8N1, no echo, no translation
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~(tcflag_t)(CRTSCTS | HUPCL);
  if (tcsetattr(fd_, TCSANOW, &t) != 0) {
    snprintf(error_, sizeof error_, "tcsetattr %s: %s", path, strerror(errno));
    close();
    return false;
  }
  char real[256];
  bool pty = realpath(path, real) != nullptr && strncmp(real, "/dev/pts/", 9) == 0;
  if (!pty) tcflush(fd_, TCIFLUSH);
  return true;
}

void PosixSerial::close() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

int32_t PosixSerial::read(uint8_t* buf, uint32_t n) {
  ssize_t r = ::read(fd_, buf, n);
  if (r > 0) return (int32_t)r;
  if (r < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
  // 0 is end of file: the other end of the pty is gone (EIO on Linux too)
  return -1;
}

int32_t PosixSerial::write(const uint8_t* buf, uint32_t n) {
  ssize_t w = ::write(fd_, buf, n);
  if (w >= 0) return (int32_t)w;
  if (errno == EAGAIN || errno == EINTR) return 0;  // TX buffer full: dropped, like a UART nobody drains
  return -1;
}

uint32_t PosixSerial::tx_free() {
  // Linux does not say how much a tty's write() takes now, only what is
  // still queued (TIOCOUTQ). Assume a 4 KiB buffer (n_tty's and most USB
  // serial drivers' order of size) less what is queued. A pty reports 0
  // queued, so this never holds a frame back there; a short write() stays
  // the backstop (tx_short).
  constexpr int kAssumedBuffer = 4096;
  int queued = 0;
  if (ioctl(fd_, TIOCOUTQ, &queued) != 0 || queued < 0) return (uint32_t)kAssumedBuffer;
  return queued >= kAssumedBuffer ? 0u : (uint32_t)(kAssumedBuffer - queued);
}

uint64_t MonotonicClock::now_us() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

void StderrLog::write(LogLevel level, const char* line) {
  if (level < min_) return;
  uint64_t now = clock_.now_us();
  if (!started_) {
    started_ = true;
    start_us_ = now;
  }
  uint64_t t = now - start_us_;
  static const char kTag[] = {'D', 'I', 'W', 'E'};
  fprintf(stderr, "%4u.%03u %c %s\n", (unsigned)(t / 1000000u), (unsigned)(t / 1000u % 1000u),
          kTag[(int)level], line);
}

FifoResetGpio::~FifoResetGpio() {
  if (fd_ >= 0) ::close(fd_);
}

void FifoResetGpio::edge(char level) {
  const char* what = level == 'L' ? "RESET low" : "RESET released";
  if (path_ == nullptr) {
    logf(log_, LogLevel::Warn, "gpio: %s (no --reset-fifo: nothing to pulse)", what);
    return;
  }
  if (fd_ < 0) fd_ = ::open(path_, O_WRONLY | O_NONBLOCK);  // ENXIO while no reader
  if (fd_ < 0 || ::write(fd_, &level, 1) != 1) {
    logf(log_, LogLevel::Warn, "gpio: %s: %s: %s", what, path_, strerror(errno));
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    return;
  }
  logf(log_, LogLevel::Debug, "gpio: %s (%s)", what, path_);
}

// ---- DirStore ----------------------------------------------------------------------

DirStore::~DirStore() {
  if (fd_ >= 0) ::close(fd_);
}

void DirStore::path(char* out, size_t cap, const char* name) const { snprintf(out, cap, "%s/%s", dir_, name); }

KernelStore::Result DirStore::fail(const char* what, const char* name) {
  logf(log_, LogLevel::Warn, "card: %s %s/%s: %s", what, dir_, name, strerror(errno));
  return Result::Failed;
}

KernelStore::Result DirStore::create(const char* name) {
  if (fd_ >= 0) return Result::Failed;
  char p[512];
  path(p, sizeof p, name);
  fd_ = ::open(p, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd_ < 0) return fail("cannot create", name);
  writing_ = true;
  return Result::Ok;
}

KernelStore::Result DirStore::open(const char* name) {
  if (fd_ >= 0) return Result::Failed;
  char p[512];
  path(p, sizeof p, name);
  fd_ = ::open(p, O_RDONLY);
  if (fd_ < 0) return errno == ENOENT ? Result::Missing : fail("cannot open", name);
  writing_ = false;
  return Result::Ok;
}

KernelStore::Result DirStore::write(const uint8_t* data, uint32_t len) {
  if (fd_ < 0 || !writing_) return Result::Failed;
  while (len > 0) {
    ssize_t w = ::write(fd_, data, len);
    if (w < 0 && errno == EINTR) continue;
    if (w <= 0) return fail("cannot write", "kernel7.new");
    data += w;
    len -= (uint32_t)w;
  }
  return Result::Ok;
}

int32_t DirStore::read(uint8_t* buf, uint32_t len) {
  if (fd_ < 0 || writing_) return -1;
  for (;;) {
    ssize_t n = ::read(fd_, buf, len);
    if (n < 0 && errno == EINTR) continue;
    return n < 0 ? -1 : (int32_t)n;
  }
}

KernelStore::Result DirStore::close() {
  if (fd_ < 0) return Result::Failed;
  bool ok = !writing_ || fsync(fd_) == 0;
  ok = ::close(fd_) == 0 && ok;
  fd_ = -1;
  return ok ? Result::Ok : fail("cannot close", "(open file)");
}

KernelStore::Result DirStore::remove(const char* name) {
  char p[512];
  path(p, sizeof p, name);
  if (::unlink(p) == 0) return Result::Ok;
  return errno == ENOENT ? Result::Missing : fail("cannot remove", name);
}

KernelStore::Result DirStore::rename(const char* from, const char* to) {
  char a[512], b[512];
  path(a, sizeof a, from);
  path(b, sizeof b, to);
  struct stat st;
  if (::stat(a, &st) != 0) return errno == ENOENT ? Result::Missing : fail("cannot stat", from);
  if (::stat(b, &st) == 0) {  // as FatFs: never onto an existing file
    logf(log_, LogLevel::Warn, "card: cannot rename %s to %s: it exists", from, to);
    return Result::Failed;
  }
  return ::rename(a, b) == 0 ? Result::Ok : fail("cannot rename", from);
}

KernelStore::Result DirStore::sync() {
  int fd = ::open(dir_, O_RDONLY | O_DIRECTORY);
  if (fd < 0) return fail("cannot open", ".");
  bool ok = fsync(fd) == 0;
  ::close(fd);
  return ok ? Result::Ok : fail("cannot sync", ".");
}

}  // namespace kilo
