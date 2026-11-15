// Fakes of the network seams (mind/platform_net.h) for the HostServer
// tests, with the laptop's end of the secure channel (the spec's [host]
// comments): FakeConn sends the clear hello and Noise message 1 when it
// connects, reads the mind's answer and message 2, and from then on seals
// what a test sends into records and opens what the server wrote. The
// Noise code is the core's own (mind/net/noise.h); test_noise.cpp holds it
// to known answers from other implementations, and kernel/host/test.sh
// runs the Python client (noiseprotocol) against kilo-mind-host.
#ifndef KILO_HOST_TESTS_FAKE_NET_H
#define KILO_HOST_TESTS_FAKE_NET_H
#include <cstring>
#include <deque>
#include <string>
#include <vector>
#include "net/authorized_keys.h"
#include "net/host_frame.h"
#include "net/noise.h"
#include "platform.h"
#include "platform_net.h"
#include "tests/check.h"
#include "third_party/monocypher/monocypher.h"

namespace kilo {

// Throwaway key pairs: the laptop the tests' mind knows, and one it does not.
struct TestKey {
  uint8_t priv[32];
  uint8_t pub[32];
  explicit TestKey(uint8_t seed) {
    for (int i = 0; i < 32; i++) priv[i] = (uint8_t)(seed * 37 + i * 11 + 1);
    crypto_x25519_public_key(pub, priv);
  }
};

inline const TestKey& laptop_key() {
  static TestKey k(1);
  return k;
}
inline const TestKey& stranger_key() {
  static TestKey k(2);
  return k;
}

// The tests' list: the laptop's key among others (the lookup goes over all).
inline const AuthorizedKey* test_keys(uint32_t* n) {
  static AuthorizedKey keys[3];
  static bool made = false;
  if (!made) {
    TestKey a(7), b(8);
    std::memcpy(keys[0].key, a.pub, 32);
    std::strcpy(keys[0].name, "someone@else");
    std::memcpy(keys[1].key, laptop_key().pub, 32);
    std::strcpy(keys[1].name, "test@laptop");
    std::memcpy(keys[2].key, b.pub, 32);
    std::strcpy(keys[2].name, "third@one");
    made = true;
  }
  *n = 3;
  return keys;
}

// Deterministic "random" bytes (a counter through BLAKE2b); fail = the source broke.
class FakeRandom final : public Random {
 public:
  bool fail = false;
  uint32_t calls = 0;
  bool fill(uint8_t* buf, uint32_t n) override {
    if (fail) return false;
    uint8_t seed[4] = {(uint8_t)calls, (uint8_t)(calls >> 8), 0x5A, 0xA5};
    calls++;
    uint8_t h[64];
    crypto_blake2b(h, 64, seed, 4);
    for (uint32_t i = 0; i < n; i++) buf[i] = h[i % 64];
    return true;
  }
};

class FakeConn final : public Conn {
 public:
  std::vector<uint8_t> in;   // what the client sent, read from pos on
  size_t pos = 0;
  std::vector<uint8_t> out;  // what the server wrote
  bool open = false;
  bool peer_closed = false;
  int32_t write_room = -1;   // >= 0: take at most this many bytes per write
  HostRx rx;                 // the client's decoder of the opened records

  // The client's side.
  bool started = false;      // the hello is sent
  bool answered = false;     // the mind's clear answer is in
  uint8_t status = 0xFF;     // its status (KILO_HOST_HELLO_STATUS_*)
  bool established = false;  // message 2 authenticated: records both ways
  bool hs_failed = false;    // message 2 did not authenticate
  bool bad_record = false;   // a record from the server did not open
  noise::HandshakeKN hs;
  noise::CipherState send_cs, recv_cs;
  std::vector<uint8_t> raw;  // the server's bytes not yet parsed
  uint8_t e_seed = 0x33;
  bool first_record = true;  // send the empty first record once message 2 is in (as kilo.hostlink does)

  int32_t read(uint8_t* buf, uint32_t n) override {
    if (!open) return -1;
    size_t k = std::min<size_t>(n, in.size() - pos);
    if (k == 0) return peer_closed ? -1 : 0;
    std::memcpy(buf, in.data() + pos, k);
    pos += k;
    return (int32_t)k;
  }
  int32_t write(const uint8_t* buf, uint32_t n) override {
    if (!open) return -1;
    uint32_t k = write_room >= 0 && (uint32_t)write_room < n ? (uint32_t)write_room : n;
    out.insert(out.end(), buf, buf + k);
    return (int32_t)k;
  }
  void close() override { open = false; }
  bool is_open() const override { return open; }

  // The hello naming claimed, then message 1; the handshake runs with priv
  // (another key's private key: the "wrong private key" case).
  void start(const uint8_t priv[32], const uint8_t claimed[32], uint8_t version = KILO_HOST_VERSION) {
    uint8_t hello[KILO_HOST_HELLO_LEN];
    std::memcpy(hello, KILO_HOST_HELLO_MAGIC, 4);
    hello[4] = version;
    hello[5] = hello[6] = hello[7] = 0;
    std::memcpy(hello + 8, claimed, 32);
    std::vector<uint8_t> prologue(KILO_HOST_NOISE_PROLOGUE, KILO_HOST_NOISE_PROLOGUE + std::strlen(KILO_HOST_NOISE_PROLOGUE));
    prologue.insert(prologue.end(), hello, hello + sizeof hello);
    uint8_t e[32];
    for (int i = 0; i < 32; i++) e[i] = (uint8_t)(e_seed + i * 3);
    hs.init_initiator(prologue.data(), (uint32_t)prologue.size(), priv, claimed, e);
    uint8_t m1[32];
    hs.write_message1(nullptr, 0, m1);
    in.insert(in.end(), hello, hello + sizeof hello);
    in.insert(in.end(), m1, m1 + 32);
    started = true;
  }
  void start() { start(laptop_key().priv, laptop_key().pub); }

  // Read what the server wrote: its answer, message 2 (then the client's
  // first record, empty, as kilo.hostlink sends it), records.
  void pump() {
    raw.insert(raw.end(), out.begin(), out.end());
    out.clear();
    if (!answered && raw.size() >= KILO_HOST_REPLY_LEN) {
      CHECKF(std::memcmp(raw.data(), KILO_HOST_HELLO_MAGIC, 4) == 0 && raw[4] == KILO_HOST_VERSION,
             "the mind's answer");
      status = raw[5];
      answered = true;
      raw.erase(raw.begin(), raw.begin() + KILO_HOST_REPLY_LEN);
    }
    if (answered && status == KILO_HOST_HELLO_STATUS_OK && !established && !hs_failed &&
        raw.size() >= noise::HandshakeKN::kMsg2Len) {
      if (hs.read_message2(raw.data(), noise::HandshakeKN::kMsg2Len, nullptr, send_cs, recv_cs)) {
        established = true;
        if (first_record) send_record(nullptr, 0);
      } else {
        hs_failed = true;
      }
      raw.erase(raw.begin(), raw.begin() + noise::HandshakeKN::kMsg2Len);
    }
    while (established && raw.size() >= 2) {
      uint32_t len = raw[0] | (raw[1] << 8);
      if (raw.size() < 2 + len) break;
      CHECKF(len >= noise::kTagLen && len <= KILO_HOST_RECORD_MAX + noise::kTagLen, "record length %u", len);
      std::vector<uint8_t> p(raw.begin() + 2, raw.begin() + 2 + len);
      bool ok = recv_cs.decrypt(nullptr, 0, p.data(), len, p.data());
      CHECKF(ok, "a record from the server did not open");
      if (!ok) bad_record = true;
      raw.erase(raw.begin(), raw.begin() + 2 + len);
      plain.insert(plain.end(), p.begin(), p.begin() + (len - noise::kTagLen));
    }
  }

  // One record holding these bytes (the plaintext: frames, or anything).
  void send_record(const uint8_t* p, uint32_t n) {
    std::vector<uint8_t> r(2 + n + noise::kTagLen);
    r[0] = (uint8_t)(n + noise::kTagLen);
    r[1] = (uint8_t)((n + noise::kTagLen) >> 8);
    CHECK(send_cs.encrypt(nullptr, 0, p, n, r.data() + 2));
    in.insert(in.end(), r.begin(), r.end());
  }
  void send_raw(const std::vector<uint8_t>& b) { in.insert(in.end(), b.begin(), b.end()); }

  // One frame in a record of its own (kilo.hostlink packs several; test_host checks that too).
  void send(uint8_t type, uint8_t seq, const void* payload = nullptr, uint32_t len = 0) {
    CHECKF(established, "send before the handshake");
    uint8_t f[KILO_HOST_MAX_FRAME];
    uint32_t n = host_frame_encode(type, seq, (const uint8_t*)payload, len, f);
    send_record(f, n);
  }
  // The frames the server wrote since the last call.
  std::vector<HostFrame> take() {
    pump();
    std::vector<HostFrame> got;
    for (uint8_t b : plain) {
      HostRxStatus st = rx.push(b);
      if (st == HostRxStatus::Frame) got.push_back(rx.frame());
      CHECKF(st == HostRxStatus::Frame || st == HostRxStatus::None, "server wrote a broken frame");
    }
    plain.clear();
    return got;
  }

 private:
  std::vector<uint8_t> plain;  // opened records, not yet framed
};

class FakeListener final : public Listener {
 public:
  static constexpr uint32_t kConns = kHostSlots + 1;  // one more than the server serves
  FakeConn conns[kConns];
  std::deque<int32_t> pending;

  int32_t accept() override {
    if (pending.empty()) return -1;
    int32_t s = pending.front();
    pending.pop_front();
    conns[s].open = true;
    return s;
  }
  Conn* conn(uint32_t slot) override { return &conns[slot]; }
  uint32_t slots() const override { return kConns; }

  // A new connection in slot that has sent nothing yet.
  FakeConn& connect_raw(int32_t slot) {
    conns[slot] = FakeConn();
    conns[slot].e_seed = (uint8_t)(0x30 + slot);
    pending.push_back(slot);
    return conns[slot];
  }
  // ... that has sent the laptop's hello and message 1.
  FakeConn& connect(int32_t slot) {
    FakeConn& c = connect_raw(slot);
    c.start();
    return c;
  }
};

}  // namespace kilo

#endif
