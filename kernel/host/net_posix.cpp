#include "net_posix.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

namespace kilo {

namespace {

bool set_nonblocking(int fd) {
  int fl = fcntl(fd, F_GETFL, 0);
  return fl >= 0 && fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0;
}

}  // namespace

int32_t PosixConn::read(uint8_t* buf, uint32_t n) {
  if (fd_ < 0) return -1;
  ssize_t r = ::recv(fd_, buf, n, 0);
  if (r > 0) return (int32_t)r;
  if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
  return -1;  // 0: the peer closed
}

int32_t PosixConn::write(const uint8_t* buf, uint32_t n) {
  if (fd_ < 0) return -1;
  ssize_t w = ::send(fd_, buf, n, MSG_NOSIGNAL);
  if (w >= 0) return (int32_t)w;
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
  return -1;
}

void PosixConn::close() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

bool PosixListener::open(const char* spec) {
  char host[64] = "127.0.0.1";
  const char* colon = strrchr(spec, ':');
  const char* port_s = spec;
  if (colon != nullptr) {
    size_t n = (size_t)(colon - spec);
    if (n == 0 || n >= sizeof host) {
      snprintf(error_, sizeof error_, "bad listen address %s", spec);
      return false;
    }
    memcpy(host, spec, n);
    host[n] = '\0';
    port_s = colon + 1;
  }
  char* end;
  long port = strtol(port_s, &end, 10);
  if (*port_s == '\0' || *end != '\0' || port < 0 || port > 65535) {
    snprintf(error_, sizeof error_, "bad listen port %s", spec);
    return false;
  }
  struct sockaddr_in a;
  memset(&a, 0, sizeof a);
  a.sin_family = AF_INET;
  a.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, host, &a.sin_addr) != 1) {
    snprintf(error_, sizeof error_, "bad listen address %s (IPv4 dotted quad)", host);
    return false;
  }
  fd_ = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  if (fd_ < 0 || setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one) != 0 ||
      bind(fd_, (struct sockaddr*)&a, sizeof a) != 0 || listen(fd_, 4) != 0 || !set_nonblocking(fd_)) {
    snprintf(error_, sizeof error_, "listen on %s:%ld: %s", host, port, strerror(errno));
    close();
    return false;
  }
  socklen_t len = sizeof a;
  getsockname(fd_, (struct sockaddr*)&a, &len);
  port_ = ntohs(a.sin_port);
  return true;
}

void PosixListener::close() {
  for (PosixConn& c : conns_) c.close();
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

int32_t PosixListener::accept() {
  if (fd_ < 0) return -1;
  int fd = ::accept(fd_, nullptr, nullptr);
  if (fd < 0) return -1;
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);  // small frames, sent as made
  if (!set_nonblocking(fd)) {
    ::close(fd);
    return -1;
  }
  for (uint32_t i = 0; i < kSlots; i++) {
    if (!conns_[i].is_open()) {
      conns_[i].adopt(fd);
      return (int32_t)i;
    }
  }
  ::close(fd);  // every slot busy
  return -1;
}

}  // namespace kilo
