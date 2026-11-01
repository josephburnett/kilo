#include "posix.h"
#include "util/log.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
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

}  // namespace kilo
