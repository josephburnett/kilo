// noise: the Noise Protocol Framework (revision 34) for the host channel,
// exactly one protocol: Noise_KN_25519_ChaChaPoly_BLAKE2b
// (kernel/README.md "Who may talk to Kilo: keys"). The primitives are
// Monocypher's (third_party/monocypher): X25519, BLAKE2b, the IETF
// ChaCha20-Poly1305 AEAD; HMAC-BLAKE2b and HKDF are written here as the
// spec defines them (RFC 2104 HMAC over BLAKE2b, block 128, not keyed BLAKE2b).
//
//   KN:
//     -> s            (pre-message: the responder knows the initiator's static key)
//     ...
//     -> e
//     <- e, ee, se
//
// The initiator is the laptop (a static key the mind's list names); the
// responder is the mind (no static key, no secret). The responder's second
// message costs three X25519 scalar multiplications (its ephemeral key, ee,
// se); HandshakeKN splits them into three calls so that a cooperative main
// loop does one per pass (net/host_server.cpp). The initiator side is here
// for the unit tests and the boot bench.
//
// Known-answer tests: kernel/host/tests/test_noise.cpp against vectors from
// the Python `noiseprotocol` package and the published cacophony and snow
// vectors (tests/gen_noise_vectors.py).
#ifndef KILO_MIND_NET_NOISE_H
#define KILO_MIND_NET_NOISE_H
#include <stdint.h>

namespace kilo {
namespace noise {

constexpr char kProtocolName[] = "Noise_KN_25519_ChaChaPoly_BLAKE2b";
constexpr uint32_t kDhLen = 32;    // X25519 keys and shared secrets
constexpr uint32_t kKeyLen = 32;   // ChaChaPoly keys
constexpr uint32_t kHashLen = 64;  // BLAKE2b
constexpr uint32_t kBlockLen = 128;
constexpr uint32_t kTagLen = 16;
// A CipherState refuses to encrypt or decrypt from this nonce on: far
// beyond anything a connection sends (2^62 records), and far below 2^64 - 1,
// which Noise reserves. The host server closes the connection instead.
constexpr uint64_t kNonceLimit = 1ull << 62;

// HMAC-BLAKE2b(key, a || b) (RFC 2104 with BLAKE2b-512, block 128 bytes).
void hmac_blake2b(uint8_t out[kHashLen], const uint8_t* key, uint32_t key_len, const uint8_t* a, uint32_t a_len,
                  const uint8_t* b = nullptr, uint32_t b_len = 0);
// Noise's HKDF with two outputs: temp = HMAC(ck, ikm), out1 = HMAC(temp,
// 0x01), out2 = HMAC(temp, out1 || 0x02).
void hkdf2(const uint8_t ck[kHashLen], const uint8_t* ikm, uint32_t ikm_len, uint8_t out1[kHashLen],
           uint8_t out2[kHashLen]);

// The first 8 bytes of BLAKE2b-512(public key), as 16 lower-case hex digits
// and a null: the short name of a key in logs and `kilo key show`
// (kilo/hostkey.py fingerprint()).
void fingerprint(const uint8_t key[kDhLen], char out[17]);

// Noise's CipherState: ChaChaPoly with the 64-bit counter nonce (little
// endian, after 4 zero bytes).
class CipherState {
 public:
  void init(const uint8_t key[kKeyLen]);
  void clear();  // wipes the key
  bool has_key() const { return has_key_; }
  uint64_t nonce() const { return n_; }
  void set_nonce(uint64_t n) { n_ = n; }  // Noise's SetNonce; for the tests (the limit)
  // out = ciphertext || tag (len + kTagLen bytes); without a key, out =
  // plain (Noise's EncryptWithAd). out may be plain. False (out untouched)
  // when the nonce is at kNonceLimit.
  bool encrypt(const uint8_t* ad, uint32_t ad_len, const uint8_t* plain, uint32_t len, uint8_t* out);
  // in = ciphertext || tag (len >= kTagLen with a key); out = len - kTagLen
  // plaintext bytes (out may be in). False on a bad tag (out untouched, the
  // nonce not advanced) or at kNonceLimit. Without a key, out = in.
  bool decrypt(const uint8_t* ad, uint32_t ad_len, const uint8_t* in, uint32_t len, uint8_t* out);

 private:
  uint8_t k_[kKeyLen] = {};
  uint64_t n_ = 0;
  bool has_key_ = false;
};

// Noise's SymmetricState (ck, h and the handshake's CipherState).
class SymmetricState {
 public:
  void init();  // InitializeSymmetric(kProtocolName)
  void mix_hash(const uint8_t* data, uint32_t len);
  void mix_key(const uint8_t* ikm, uint32_t len);
  // EncryptAndHash / DecryptAndHash: out has len (+ kTagLen with a key) /
  // len (- kTagLen) bytes.
  bool encrypt_and_hash(const uint8_t* plain, uint32_t len, uint8_t* out);
  bool decrypt_and_hash(const uint8_t* in, uint32_t len, uint8_t* out);
  // Split(): c1 initiator -> responder, c2 responder -> initiator. Then wipes ck.
  void split(CipherState& c1, CipherState& c2);
  bool has_key() const { return cs_.has_key(); }
  const uint8_t* h() const { return h_; }
  void wipe();

 private:
  uint8_t ck_[kHashLen] = {};
  uint8_t h_[kHashLen] = {};
  CipherState cs_;
};

// One KN handshake, either role. Secrets (ephemeral private key, ck, DH
// outputs) are wiped as soon as they are used up and by wipe().
class HandshakeKN {
 public:
  static constexpr uint32_t kMsg1Len = kDhLen;            // -> e (with an empty payload)
  static constexpr uint32_t kMsg2Len = kDhLen + kTagLen;  // <- e, ee, se (an empty payload's tag)

  // The responder: the initiator's static public key (the pre-message) and
  // its own ephemeral private key (32 random bytes; X25519 clamps them).
  void init_responder(const uint8_t* prologue, uint32_t prologue_len, const uint8_t rs[kDhLen],
                      const uint8_t e_priv[kDhLen]);
  // msg1 = e || payload (len >= kDhLen); payload_out gets len - kDhLen bytes.
  bool read_message1(const uint8_t* msg, uint32_t len, uint8_t* payload_out);
  // Message 2 in three steps, one X25519 each, in this order:
  void responder_keygen();  // e.pub = X25519(e, base)
  bool responder_ee();      // MixHash(e.pub), MixKey(X25519(e, re)); false: a zero shared secret (bad re)
  // MixKey(X25519(e, rs)), out = e.pub || EncryptAndHash(payload) (kDhLen +
  // len + kTagLen bytes), then Split: send responder -> initiator, recv the
  // other way. False: a zero shared secret.
  bool responder_se(const uint8_t* payload, uint32_t len, uint8_t* out, CipherState& send, CipherState& recv);

  // The initiator: its static key pair and its ephemeral private key.
  void init_initiator(const uint8_t* prologue, uint32_t prologue_len, const uint8_t s_priv[kDhLen],
                      const uint8_t s_pub[kDhLen], const uint8_t e_priv[kDhLen]);
  // out = e.pub || payload (kDhLen + len bytes).
  void write_message1(const uint8_t* payload, uint32_t len, uint8_t* out);
  // msg2 = re || ciphertext || tag; payload_out gets len - kDhLen - kTagLen
  // bytes. Then Split: send initiator -> responder. False: a bad tag (not
  // the responder this key was meant for, or tampered) or a zero DH.
  bool read_message2(const uint8_t* msg, uint32_t len, uint8_t* payload_out, CipherState& send, CipherState& recv);

  // The handshake hash h after Split (channel binding; the tests compare it).
  const uint8_t* handshake_hash() const { return hash_; }
  void wipe();

 private:
  bool dh_mix(const uint8_t priv[kDhLen], const uint8_t pub[kDhLen]);
  void start(const uint8_t* prologue, uint32_t prologue_len, const uint8_t s_pub[kDhLen], const uint8_t e_priv[kDhLen]);

  SymmetricState ss_;
  uint8_t s_priv_[kDhLen] = {};  // the initiator's static private key (initiator only)
  uint8_t e_priv_[kDhLen] = {};
  uint8_t e_pub_[kDhLen] = {};
  uint8_t re_[kDhLen] = {};
  uint8_t rs_[kDhLen] = {};      // the initiator's static public key (responder only)
  uint8_t hash_[kHashLen] = {};
};

}  // namespace noise
}  // namespace kilo

#endif
