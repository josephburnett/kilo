// The host channel's Noise cost on this machine (`make bench`), as the
// Circle kernel's boot bench measures it on the Pi (src/plannerbench.cpp
// "noise bench"): the responder's three X25519 steps (a main-loop pass
// each in HostServer), the initiator's side, and a full record sealed and
// opened. Optimised as the kernel is, no sanitizers.
#include <chrono>
#include <cstdio>
#include <cstring>
#include "net/noise.h"
#include "proto/kilo_protocol.h"
#include "third_party/monocypher/monocypher.h"

using namespace kilo::noise;

namespace {

double now_us() {
  using namespace std::chrono;
  return (double)duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count() / 1000.0;
}

struct Stat {
  double sum = 0, max = 0;
  int n = 0;
  void add(double v) {
    sum += v;
    max = v > max ? v : max;
    n++;
  }
  double mean() const { return n ? sum / n : 0; }
};

}  // namespace

int main() {
  const int kRounds = 200, kRecords = 2000;
  static const uint8_t prologue[] = "kilo noise bench";
  Stat step[3], ini[2], seal, open;
  bool ok = true;
  for (int r = 0; r < kRounds; r++) {
    uint8_t s[32], s_pub[32], ei[32], er[32], m1[32], m2[48];
    for (int i = 0; i < 32; i++) {
      s[i] = (uint8_t)(i * 7 + 1 + r);
      ei[i] = (uint8_t)(i * 5 + 3 + r);
      er[i] = (uint8_t)(i * 3 + 9 + r);
    }
    crypto_x25519_public_key(s_pub, s);
    HandshakeKN a, b;
    CipherState is, ir, rs, rr;
    double t = now_us();
    a.init_initiator(prologue, sizeof prologue, s, s_pub, ei);
    a.write_message1(nullptr, 0, m1);
    ini[0].add(now_us() - t);
    b.init_responder(prologue, sizeof prologue, s_pub, er);
    ok = ok && b.read_message1(m1, 32, nullptr);
    t = now_us();
    b.responder_keygen();
    step[0].add(now_us() - t);
    t = now_us();
    ok = ok && b.responder_ee();
    step[1].add(now_us() - t);
    t = now_us();
    ok = ok && b.responder_se(nullptr, 0, m2, rs, rr);
    step[2].add(now_us() - t);
    t = now_us();
    ok = ok && a.read_message2(m2, 48, nullptr, is, ir);
    ini[1].add(now_us() - t);
    if (r == 0) {
      static uint8_t rec[KILO_HOST_RECORD_MAX + kTagLen];
      std::memset(rec, 0x5A, sizeof rec);
      for (int k = 0; k < kRecords; k++) {
        t = now_us();
        ok = ok && rs.encrypt(nullptr, 0, rec, KILO_HOST_RECORD_MAX, rec);
        double t1 = now_us();
        ok = ok && ir.decrypt(nullptr, 0, rec, sizeof rec, rec);
        seal.add(t1 - t);
        open.add(now_us() - t1);
      }
    }
  }
  std::printf(
      "noise bench (host, %d handshakes): X25519 keygen mean %.1f max %.1f us, ee mean %.1f max %.1f us, se mean "
      "%.1f max %.1f us (the mind's handshake: a pass each); the laptop's side %.1f + %.1f us\n",
      kRounds, step[0].mean(), step[0].max, step[1].mean(), step[1].max, step[2].mean(), step[2].max, ini[0].mean(),
      ini[1].mean());
  std::printf("noise bench (host): a %u-byte record: seal mean %.2f max %.1f us, open mean %.2f max %.1f us%s\n",
              (unsigned)KILO_HOST_RECORD_MAX, seal.mean(), seal.max, open.mean(), open.max,
              ok ? "" : "  FAILED: a step refused");
  return ok ? 0 : 1;
}
