#include "net/noise.h"
#include "third_party/monocypher/monocypher.h"
#include "util/mem.h"

namespace kilo {
namespace noise {

namespace {

static_assert(sizeof(kProtocolName) - 1 <= kHashLen, "the name fits h: h = name, zero padded");

void store64_le(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i));
}

// The 12-byte ChaChaPoly nonce: 4 zero bytes, then n as 64 bits little endian.
void nonce12(uint64_t n, uint8_t out[12]) {
  mem_zero(out, 4);
  store64_le(out + 4, n);
}

void blake2b_update(crypto_blake2b_ctx* ctx, const uint8_t* p, uint32_t n) {
  if (n != 0) crypto_blake2b_update(ctx, p, n);
}

}  // namespace

void hmac_blake2b(uint8_t out[kHashLen], const uint8_t* key, uint32_t key_len, const uint8_t* a, uint32_t a_len,
                  const uint8_t* b, uint32_t b_len) {
  uint8_t k[kBlockLen];
  mem_zero(k, sizeof k);
  if (key_len > kBlockLen) crypto_blake2b(k, kHashLen, key, key_len);
  else mem_copy(k, key, key_len);
  uint8_t pad[kBlockLen];
  uint8_t inner[kHashLen];
  crypto_blake2b_ctx ctx;
  for (uint32_t i = 0; i < kBlockLen; i++) pad[i] = (uint8_t)(k[i] ^ 0x36);
  crypto_blake2b_init(&ctx, kHashLen);
  crypto_blake2b_update(&ctx, pad, kBlockLen);
  blake2b_update(&ctx, a, a_len);
  blake2b_update(&ctx, b, b_len);
  crypto_blake2b_final(&ctx, inner);
  for (uint32_t i = 0; i < kBlockLen; i++) pad[i] = (uint8_t)(k[i] ^ 0x5c);
  crypto_blake2b_init(&ctx, kHashLen);
  crypto_blake2b_update(&ctx, pad, kBlockLen);
  crypto_blake2b_update(&ctx, inner, kHashLen);
  crypto_blake2b_final(&ctx, out);
  crypto_wipe(k, sizeof k);
  crypto_wipe(pad, sizeof pad);
  crypto_wipe(inner, sizeof inner);
  crypto_wipe(&ctx, sizeof ctx);
}

void hkdf2(const uint8_t ck[kHashLen], const uint8_t* ikm, uint32_t ikm_len, uint8_t out1[kHashLen],
           uint8_t out2[kHashLen]) {
  uint8_t temp[kHashLen];
  hmac_blake2b(temp, ck, kHashLen, ikm, ikm_len);
  const uint8_t one = 0x01, two = 0x02;
  hmac_blake2b(out1, temp, kHashLen, &one, 1);
  hmac_blake2b(out2, temp, kHashLen, out1, kHashLen, &two, 1);
  crypto_wipe(temp, sizeof temp);
}

void fingerprint(const uint8_t key[kDhLen], char out[17]) {
  uint8_t h[kHashLen];
  crypto_blake2b(h, kHashLen, key, kDhLen);
  static const char hex[] = "0123456789abcdef";
  for (int i = 0; i < 8; i++) {
    out[2 * i] = hex[h[i] >> 4];
    out[2 * i + 1] = hex[h[i] & 15];
  }
  out[16] = '\0';
}

// ---- CipherState -----------------------------------------------------------------

void CipherState::init(const uint8_t key[kKeyLen]) {
  mem_copy(k_, key, kKeyLen);
  n_ = 0;
  has_key_ = true;
}

void CipherState::clear() {
  crypto_wipe(k_, sizeof k_);
  n_ = 0;
  has_key_ = false;
}

bool CipherState::encrypt(const uint8_t* ad, uint32_t ad_len, const uint8_t* plain, uint32_t len, uint8_t* out) {
  if (!has_key_) {
    if (out != plain) mem_copy(out, plain, len);
    return true;
  }
  if (n_ >= kNonceLimit) return false;
  uint8_t nonce[12];
  nonce12(n_, nonce);
  crypto_aead_ctx ctx;
  crypto_aead_init_ietf(&ctx, k_, nonce);
  crypto_aead_write(&ctx, out, out + len, ad, ad_len, plain, len);
  crypto_wipe(&ctx, sizeof ctx);
  n_++;
  return true;
}

bool CipherState::decrypt(const uint8_t* ad, uint32_t ad_len, const uint8_t* in, uint32_t len, uint8_t* out) {
  if (!has_key_) {
    if (out != in) mem_copy(out, in, len);
    return true;
  }
  if (len < kTagLen || n_ >= kNonceLimit) return false;
  uint32_t n = len - kTagLen;
  uint8_t nonce[12];
  nonce12(n_, nonce);
  crypto_aead_ctx ctx;
  crypto_aead_init_ietf(&ctx, k_, nonce);
  // The tag is copied out first: out may be in, and in place the plaintext
  // overwrites the ciphertext, never the tag after it (n bytes only).
  uint8_t mac[kTagLen];
  mem_copy(mac, in + n, kTagLen);
  int bad = crypto_aead_read(&ctx, out, mac, ad, ad_len, in, n);
  crypto_wipe(&ctx, sizeof ctx);
  if (bad != 0) return false;
  n_++;
  return true;
}

// ---- SymmetricState --------------------------------------------------------------

void SymmetricState::init() {
  mem_zero(h_, sizeof h_);
  mem_copy(h_, kProtocolName, sizeof(kProtocolName) - 1);
  mem_copy(ck_, h_, kHashLen);
  cs_.clear();
}

void SymmetricState::mix_hash(const uint8_t* data, uint32_t len) {
  crypto_blake2b_ctx ctx;
  crypto_blake2b_init(&ctx, kHashLen);
  crypto_blake2b_update(&ctx, h_, kHashLen);
  blake2b_update(&ctx, data, len);
  crypto_blake2b_final(&ctx, h_);
}

void SymmetricState::mix_key(const uint8_t* ikm, uint32_t len) {
  uint8_t ck[kHashLen], temp_k[kHashLen];
  hkdf2(ck_, ikm, len, ck, temp_k);
  mem_copy(ck_, ck, kHashLen);
  cs_.init(temp_k);  // HASHLEN 64: the first 32 bytes
  crypto_wipe(ck, sizeof ck);
  crypto_wipe(temp_k, sizeof temp_k);
}

bool SymmetricState::encrypt_and_hash(const uint8_t* plain, uint32_t len, uint8_t* out) {
  if (!cs_.encrypt(h_, kHashLen, plain, len, out)) return false;
  mix_hash(out, len + (cs_.has_key() ? kTagLen : 0u));
  return true;
}

bool SymmetricState::decrypt_and_hash(const uint8_t* in, uint32_t len, uint8_t* out) {
  // h is mixed with the ciphertext, which decrypting in place would overwrite.
  uint8_t h_next[kHashLen];
  crypto_blake2b_ctx ctx;
  crypto_blake2b_init(&ctx, kHashLen);
  crypto_blake2b_update(&ctx, h_, kHashLen);
  blake2b_update(&ctx, in, len);
  crypto_blake2b_final(&ctx, h_next);
  if (!cs_.decrypt(h_, kHashLen, in, len, out)) return false;
  mem_copy(h_, h_next, kHashLen);
  return true;
}

void SymmetricState::split(CipherState& c1, CipherState& c2) {
  uint8_t k1[kHashLen], k2[kHashLen];
  hkdf2(ck_, nullptr, 0, k1, k2);
  c1.init(k1);
  c2.init(k2);
  crypto_wipe(k1, sizeof k1);
  crypto_wipe(k2, sizeof k2);
  crypto_wipe(ck_, sizeof ck_);
  cs_.clear();
}

void SymmetricState::wipe() {
  crypto_wipe(ck_, sizeof ck_);
  crypto_wipe(h_, sizeof h_);
  cs_.clear();
}

// ---- HandshakeKN -----------------------------------------------------------------

void HandshakeKN::start(const uint8_t* prologue, uint32_t prologue_len, const uint8_t s_pub[kDhLen],
                        const uint8_t e_priv[kDhLen]) {
  ss_.init();
  ss_.mix_hash(prologue, prologue_len);
  ss_.mix_hash(s_pub, kDhLen);  // the pre-message: -> s
  mem_copy(e_priv_, e_priv, kDhLen);
}

// MixKey(DH(priv, pub)); false (nothing mixed) for an all-zero result, a
// low-order public key: fail closed rather than run on a known key.
bool HandshakeKN::dh_mix(const uint8_t priv[kDhLen], const uint8_t pub[kDhLen]) {
  static const uint8_t zero[kDhLen] = {};
  uint8_t dh[kDhLen];
  crypto_x25519(dh, priv, pub);
  bool ok = crypto_verify32(dh, zero) != 0;
  if (ok) ss_.mix_key(dh, kDhLen);
  crypto_wipe(dh, sizeof dh);
  return ok;
}

void HandshakeKN::init_responder(const uint8_t* prologue, uint32_t prologue_len, const uint8_t rs[kDhLen],
                                 const uint8_t e_priv[kDhLen]) {
  wipe();
  mem_copy(rs_, rs, kDhLen);
  start(prologue, prologue_len, rs, e_priv);
}

bool HandshakeKN::read_message1(const uint8_t* msg, uint32_t len, uint8_t* payload_out) {
  if (len < kDhLen) return false;
  mem_copy(re_, msg, kDhLen);
  ss_.mix_hash(re_, kDhLen);
  return ss_.decrypt_and_hash(msg + kDhLen, len - kDhLen, payload_out);  // no key yet: plaintext
}

void HandshakeKN::responder_keygen() { crypto_x25519_public_key(e_pub_, e_priv_); }

bool HandshakeKN::responder_ee() {
  ss_.mix_hash(e_pub_, kDhLen);
  return dh_mix(e_priv_, re_);
}

bool HandshakeKN::responder_se(const uint8_t* payload, uint32_t len, uint8_t* out, CipherState& send,
                               CipherState& recv) {
  bool ok = dh_mix(e_priv_, rs_);
  crypto_wipe(e_priv_, sizeof e_priv_);
  if (!ok) return false;
  mem_copy(out, e_pub_, kDhLen);
  if (!ss_.encrypt_and_hash(payload, len, out + kDhLen)) return false;
  mem_copy(hash_, ss_.h(), kHashLen);
  ss_.split(recv, send);  // c1 initiator -> responder: what the responder receives
  return true;
}

void HandshakeKN::init_initiator(const uint8_t* prologue, uint32_t prologue_len, const uint8_t s_priv[kDhLen],
                                 const uint8_t s_pub[kDhLen], const uint8_t e_priv[kDhLen]) {
  wipe();
  mem_copy(s_priv_, s_priv, kDhLen);
  start(prologue, prologue_len, s_pub, e_priv);
}

void HandshakeKN::write_message1(const uint8_t* payload, uint32_t len, uint8_t* out) {
  crypto_x25519_public_key(e_pub_, e_priv_);
  mem_copy(out, e_pub_, kDhLen);
  ss_.mix_hash(e_pub_, kDhLen);
  ss_.encrypt_and_hash(payload, len, out + kDhLen);  // no key: plaintext, cannot fail
}

bool HandshakeKN::read_message2(const uint8_t* msg, uint32_t len, uint8_t* payload_out, CipherState& send,
                                CipherState& recv) {
  bool ok = len >= kDhLen + kTagLen;
  if (ok) {
    mem_copy(re_, msg, kDhLen);
    ss_.mix_hash(re_, kDhLen);
    ok = dh_mix(e_priv_, re_) && dh_mix(s_priv_, re_) &&  // ee, se
         ss_.decrypt_and_hash(msg + kDhLen, len - kDhLen, payload_out);
  }
  crypto_wipe(e_priv_, sizeof e_priv_);
  crypto_wipe(s_priv_, sizeof s_priv_);
  if (!ok) return false;
  mem_copy(hash_, ss_.h(), kHashLen);
  ss_.split(send, recv);
  return true;
}

void HandshakeKN::wipe() {
  ss_.wipe();
  crypto_wipe(s_priv_, sizeof s_priv_);
  crypto_wipe(e_priv_, sizeof e_priv_);
  mem_zero(e_pub_, sizeof e_pub_);
  mem_zero(re_, sizeof re_);
  mem_zero(rs_, sizeof rs_);
  mem_zero(hash_, sizeof hash_);
}

}  // namespace noise
}  // namespace kilo
