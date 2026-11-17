// mdns-host: the mind's mDNS responder (mind/net/mdns.h) on a loopback UDP
// port, so kernel/host/test.sh can run the laptop's resolver (`kilo find
// --ask 127.0.0.1:PORT`) against the kernel's own answers. Only replies to
// a query's sender leave it; the probes and announcements, which go to the
// multicast group, are dropped: nothing reaches the real network.
//
//   mdns-host [--name kilo] [--address A.B.C.D] [--build HEX] [--duration S]
//   prints "mdns-host port N" once it answers
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "net/mdns.h"

using namespace kilo;

static uint64_t now_us() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

int main(int argc, char** argv) {
  const char* name = "kilo";
  const char* address = "192.168.88.9";
  uint32_t build = 0x2d7789e1;
  double duration = 30;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (strcmp(argv[i], "--name") == 0) name = argv[i + 1];
    else if (strcmp(argv[i], "--address") == 0) address = argv[i + 1];
    else if (strcmp(argv[i], "--build") == 0) build = (uint32_t)strtoul(argv[i + 1], nullptr, 16);
    else if (strcmp(argv[i], "--duration") == 0) duration = atof(argv[i + 1]);
    else {
      fprintf(stderr, "usage: mdns-host [--name N] [--address A.B.C.D] [--build HEX] [--duration S]\n");
      return 64;
    }
  }
  uint8_t ip[4];
  if (inet_pton(AF_INET, address, ip) != 1) return 64;
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (s < 0 || bind(s, (sockaddr*)&a, sizeof a) != 0) return 1;
  socklen_t al = sizeof a;
  getsockname(s, (sockaddr*)&a, &al);

  mdns::Responder r;
  mdns::Out out;
  r.begin(name, 5300, build);
  uint64_t t0 = now_us();
  r.set_address(ip, t0);
  bool told = false;
  while (now_us() - t0 < (uint64_t)(duration * 1e6)) {
    uint64_t t = now_us();
    while (r.poll(t, &out)) {
    }  // to the group: dropped
    if (r.claimed() && !told) {
      printf("mdns-host port %u\n", (unsigned)ntohs(a.sin_port));
      fflush(stdout);
      told = true;
    }
    pollfd p = {s, POLLIN, 0};
    if (poll(&p, 1, 5) <= 0) continue;
    uint8_t buf[1500];
    sockaddr_in from = {};
    socklen_t fl = sizeof from;
    ssize_t n = recvfrom(s, buf, sizeof buf, 0, (sockaddr*)&from, &fl);
    if (n <= 0) continue;
    uint8_t src[4];
    memcpy(src, &from.sin_addr.s_addr, 4);
    if (!r.receive(buf, (uint32_t)n, src, ntohs(from.sin_port), now_us(), &out)) continue;
    if (memcmp(out.ip, src, 4) != 0) continue;  // a multicast answer: dropped
    sockaddr_in to = {};
    to.sin_family = AF_INET;
    memcpy(&to.sin_addr.s_addr, out.ip, 4);
    to.sin_port = htons(out.port);
    sendto(s, out.data, out.len, 0, (sockaddr*)&to, sizeof to);
  }
  const mdns::Stats& st = r.stats();
  fprintf(stderr, "mdns-host: %u queries, %u answered (%u legacy unicast)\n", (unsigned)st.queries,
          (unsigned)st.answered, (unsigned)st.legacy);
  return 0;
}
