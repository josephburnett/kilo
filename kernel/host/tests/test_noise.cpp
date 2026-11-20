// The Noise layer (mind/net/noise.h) against known answers from elsewhere
// (tests/noise_vectors.txt, tests/gen_noise_vectors.py): the published
// cacophony and snow vectors for Noise_KN_25519_ChaChaPoly_BLAKE2b, vectors
// from the Python noiseprotocol package in the host channel's own shape,
// HMAC-BLAKE2b and HKDF from Python's hmac and hashlib. Both roles must
// reproduce every handshake message, the handshake hash and every
// transport ciphertext byte for byte. Then what must fail: a tampered
// message 2, the wrong static private key, a tampered, replayed or
// truncated transport message, the nonce limit.
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "net/noise.h"
#include "tests/check.h"
#include "tests/fakes.h"
#include "third_party/monocypher/monocypher.h"

using namespace kilo;
using namespace kilo::noise;
using Bytes = std::vector<uint8_t>;

namespace {

Bytes pub_of(const Bytes& priv) {
  Bytes p(32);
  crypto_x25519_public_key(p.data(), priv.data());
  return p;
}

struct Msg {
  char who;
  Bytes payload, ct;
};

struct Vector {
  std::string name;
  Bytes prologue, s, e_i, e_r, hash;
  std::vector<Msg> msgs;
};

struct Pair {
  HandshakeKN ini, resp;
  CipherState i_send, i_recv, r_send, r_recv;
};

// The handshake of v through both roles; false at the first mismatch.
bool handshake(const Vector& v, Pair& p) {
  Bytes s_pub = pub_of(v.s);
  p.ini.init_initiator(v.prologue.data(), (uint32_t)v.prologue.size(), v.s.data(), s_pub.data(), v.e_i.data());
  p.resp.init_responder(v.prologue.data(), (uint32_t)v.prologue.size(), s_pub.data(), v.e_r.data());
  const Msg& m1 = v.msgs[0];
  Bytes out(kDhLen + m1.payload.size());
  p.ini.write_message1(m1.payload.data(), (uint32_t)m1.payload.size(), out.data());
  CHECKF(out == m1.ct, "%s: message 1", v.name.c_str());
  Bytes got(m1.payload.size());
  CHECKF(p.resp.read_message1(m1.ct.data(), (uint32_t)m1.ct.size(), got.data()) && got == m1.payload,
         "%s: message 1 read", v.name.c_str());
  const Msg& m2 = v.msgs[1];
  Bytes out2(kDhLen + m2.payload.size() + kTagLen);
  p.resp.responder_keygen();
  bool ok = p.resp.responder_ee();
  ok = ok && p.resp.responder_se(m2.payload.data(), (uint32_t)m2.payload.size(), out2.data(), p.r_send, p.r_recv);
  CHECKF(ok && out2 == m2.ct, "%s: message 2", v.name.c_str());
  Bytes got2(m2.payload.size());
  bool read = p.ini.read_message2(m2.ct.data(), (uint32_t)m2.ct.size(), got2.data(), p.i_send, p.i_recv);
  CHECKF(read && got2 == m2.payload, "%s: message 2 read", v.name.c_str());
  CHECKF(Bytes(p.ini.handshake_hash(), p.ini.handshake_hash() + kHashLen) ==
             Bytes(p.resp.handshake_hash(), p.resp.handshake_hash() + kHashLen),
         "%s: both hashes", v.name.c_str());
  if (!v.hash.empty())
    CHECKF(Bytes(p.ini.handshake_hash(), p.ini.handshake_hash() + kHashLen) == v.hash, "%s: handshake hash",
           v.name.c_str());
  return ok && read;
}

int g_vectors = 0, g_transport = 0;

void replay(const Vector& v) {
  CHECKF(v.msgs.size() >= 2 && v.s.size() == 32 && v.e_i.size() == 32 && v.e_r.size() == 32, "%s: shape",
         v.name.c_str());
  static Pair p;
  p = Pair();
  if (!handshake(v, p)) return;
  for (size_t k = 2; k < v.msgs.size(); k++) {
    const Msg& m = v.msgs[k];
    CipherState& send = m.who == 'I' ? p.i_send : p.r_send;
    CipherState& recv = m.who == 'I' ? p.r_recv : p.i_recv;
    Bytes ct(m.payload.size() + kTagLen);
    CHECK(send.encrypt(nullptr, 0, m.payload.data(), (uint32_t)m.payload.size(), ct.data()));
    CHECKF(ct == m.ct, "%s: transport %zu (%c, %zu bytes)", v.name.c_str(), k, m.who, m.payload.size());
    Bytes in = m.ct;  // decrypted in place, as the host server does
    CHECKF(recv.decrypt(nullptr, 0, in.data(), (uint32_t)in.size(), in.data()) &&
               Bytes(in.begin(), in.begin() + (long)m.payload.size()) == m.payload,
           "%s: transport %zu read", v.name.c_str(), k);
    g_transport++;
  }
  g_vectors++;
}

std::vector<Vector> load(const char* path, int* macs) {
  std::ifstream in(path);
  CHECKF(in.good(), "cannot read %s", path);
  std::vector<Vector> vs;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string what;
    ss >> what;
    if (what == "noise") {
      Vector v;
      std::string pro, s, ei, er, h;
      ss >> v.name >> pro >> s >> ei >> er >> h;
      v.prologue = unhex(pro);
      v.s = unhex(s);
      v.e_i = unhex(ei);
      v.e_r = unhex(er);
      v.hash = unhex(h);
      vs.push_back(v);
    } else if (what == "msg") {
      std::string who, pl, ct;
      ss >> who >> pl >> ct;
      vs.back().msgs.push_back({who[0], unhex(pl), unhex(ct)});
    } else if (what == "hmac") {
      std::string k, d, o;
      ss >> k >> d >> o;
      Bytes key = unhex(k), data = unhex(d), out(kHashLen);
      hmac_blake2b(out.data(), key.data(), (uint32_t)key.size(), data.data(), (uint32_t)data.size());
      CHECKF(out == unhex(o), "hmac key %zu data %zu", key.size(), data.size());
      // The same message in two parts (how HKDF's second output is made).
      if (data.size() > 1) {
        hmac_blake2b(out.data(), key.data(), (uint32_t)key.size(), data.data(), 1, data.data() + 1,
                     (uint32_t)data.size() - 1);
        CHECKF(out == unhex(o), "hmac in two parts, key %zu data %zu", key.size(), data.size());
      }
      (*macs)++;
    } else if (what == "hkdf") {
      std::string c, i, o1, o2;
      ss >> c >> i >> o1 >> o2;
      Bytes ck = unhex(c), ikm = unhex(i), a(kHashLen), b(kHashLen);
      hkdf2(ck.data(), ikm.data(), (uint32_t)ikm.size(), a.data(), b.data());
      CHECKF(a == unhex(o1) && b == unhex(o2), "hkdf ikm %zu", ikm.size());
      (*macs)++;
    } else if (what == "fp") {
      std::string pub, fp;
      ss >> pub >> fp;
      char out[17];
      fingerprint(unhex(pub).data(), out);
      CHECKF(fp == out, "fingerprint %s: %s", fp.c_str(), out);
      (*macs)++;
    }
  }
  return vs;
}

// What must fail, on the host channel's own vector.
void test_failures(const Vector& v) {
  static Pair p;
  // A flipped bit anywhere in message 2: the initiator refuses it.
  for (size_t at : {size_t(0), size_t(31), size_t(32), v.msgs[1].ct.size() - 1}) {
    Vector t = v;
    t.msgs[1].ct[at] ^= 0x10;
    p = Pair();
    Bytes s_pub = pub_of(t.s);
    p.ini.init_initiator(t.prologue.data(), (uint32_t)t.prologue.size(), t.s.data(), s_pub.data(), t.e_i.data());
    Bytes m1(kDhLen);
    p.ini.write_message1(nullptr, 0, m1.data());
    CHECKF(!p.ini.read_message2(t.msgs[1].ct.data(), (uint32_t)t.msgs[1].ct.size(), nullptr, p.i_send, p.i_recv),
           "message 2 with byte %zu flipped was taken", at);
  }
  // Another private key for the public key the responder knows: message 2 does not authenticate.
  {
    p = Pair();
    Bytes s_pub = pub_of(v.s), other(32, 0x42);
    p.ini.init_initiator(v.prologue.data(), (uint32_t)v.prologue.size(), other.data(), s_pub.data(), v.e_i.data());
    Bytes m1(kDhLen);
    p.ini.write_message1(nullptr, 0, m1.data());
    CHECK(m1 == v.msgs[0].ct);  // message 1 does not depend on s
    CHECK(!p.ini.read_message2(v.msgs[1].ct.data(), (uint32_t)v.msgs[1].ct.size(), nullptr, p.i_send, p.i_recv));
  }
  // Another prologue (a changed clear byte): message 2 does not authenticate.
  {
    Vector t = v;
    t.prologue.back() ^= 1;
    p = Pair();
    Bytes s_pub = pub_of(t.s);
    p.ini.init_initiator(t.prologue.data(), (uint32_t)t.prologue.size(), t.s.data(), s_pub.data(), t.e_i.data());
    Bytes m1(kDhLen);
    p.ini.write_message1(nullptr, 0, m1.data());
    CHECK(!p.ini.read_message2(v.msgs[1].ct.data(), (uint32_t)v.msgs[1].ct.size(), nullptr, p.i_send, p.i_recv));
  }
  // A low-order ephemeral key (all zero): the responder's ee is zero and it refuses.
  {
    p = Pair();
    Bytes s_pub = pub_of(v.s), zero(32, 0);
    p.resp.init_responder(v.prologue.data(), (uint32_t)v.prologue.size(), s_pub.data(), v.e_r.data());
    CHECK(p.resp.read_message1(zero.data(), 32, nullptr));
    p.resp.responder_keygen();
    CHECK(!p.resp.responder_ee());
  }
  // Transport: tampered, replayed, truncated; the nonce limit.
  p = Pair();
  CHECK(handshake(v, p));
  Bytes pt(100, 7), ct(100 + kTagLen), out(100 + kTagLen);
  CHECK(p.i_send.encrypt(nullptr, 0, pt.data(), 100, ct.data()));
  Bytes bad = ct;
  bad[50] ^= 1;
  CHECK(!p.r_recv.decrypt(nullptr, 0, bad.data(), (uint32_t)bad.size(), out.data()) && p.r_recv.nonce() == 0);
  bad = ct;
  bad.back() ^= 0x80;  // the tag
  CHECK(!p.r_recv.decrypt(nullptr, 0, bad.data(), (uint32_t)bad.size(), out.data()));
  CHECK(!p.r_recv.decrypt(nullptr, 0, ct.data(), 15, out.data()));       // shorter than a tag
  CHECK(!p.r_recv.decrypt(nullptr, 0, ct.data(), 100, out.data()));      // cut: the last 16 are not the tag
  CHECK(p.r_recv.decrypt(nullptr, 0, ct.data(), (uint32_t)ct.size(), out.data()) && p.r_recv.nonce() == 1);
  CHECK(Bytes(out.begin(), out.begin() + 100) == pt);
  CHECK(!p.r_recv.decrypt(nullptr, 0, ct.data(), (uint32_t)ct.size(), out.data()));  // the same record again
  p.i_send.set_nonce(kNonceLimit - 1);
  CHECK(p.i_send.encrypt(nullptr, 0, pt.data(), 100, ct.data()));
  CHECK(!p.i_send.encrypt(nullptr, 0, pt.data(), 100, ct.data()) && p.i_send.nonce() == kNonceLimit);
  p.r_recv.set_nonce(kNonceLimit - 1);
  CHECK(p.r_recv.decrypt(nullptr, 0, ct.data(), (uint32_t)ct.size(), out.data()));
  p.r_recv.set_nonce(kNonceLimit);
  CHECK(!p.r_recv.decrypt(nullptr, 0, ct.data(), (uint32_t)ct.size(), out.data()));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: test_noise NOISE_VECTORS.txt\n");
    return 2;
  }
  int macs = 0;
  std::vector<Vector> vs = load(argv[1], &macs);
  int published = 0;
  for (const Vector& v : vs) {
    replay(v);
    published += v.name.rfind("kilo-", 0) != 0;
  }
  CHECKF(published == 2 && vs.size() >= 5, "%d published vectors, %zu in all", published, vs.size());
  for (const Vector& v : vs)
    if (v.name == "kilo-0") test_failures(v);
  CHECKF(macs >= 16, "%d hmac/hkdf/fingerprint lines", macs);
  std::printf("  %d Noise vectors (%d published), %d transport messages, %d hmac/hkdf/fingerprint\n", g_vectors,
              published, g_transport, macs);
  return report("noise");
}
