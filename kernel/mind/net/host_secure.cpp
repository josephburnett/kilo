// HostServer's secure layer (net/host_server.h "Who may connect", the
// spec's [host] comments): the clear hello, the Noise KN handshake one
// X25519 per pass, the record layer under the frames, the handshake
// deadline. The frames themselves are host_server.cpp's.
#include "net/host_server.h"
#include "third_party/monocypher/monocypher.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {

namespace {

constexpr uint32_t kHelloLen = KILO_HOST_HELLO_LEN;
constexpr uint32_t kReplyLen = KILO_HOST_REPLY_LEN;
constexpr uint32_t kFirstFlight = kHelloLen + noise::HandshakeKN::kMsg1Len;  // the hello, then -> e
constexpr uint32_t kTagLen = noise::kTagLen;
constexpr uint64_t kHandshakeUs = KILO_HOST_HANDSHAKE_TIMEOUT_MS * 1000ull;
constexpr char kMagic[] = KILO_HOST_HELLO_MAGIC;
constexpr char kPrologue[] = KILO_HOST_NOISE_PROLOGUE;
constexpr uint32_t kPrologueLen = sizeof(kPrologue) - 1 + kHelloLen;
static_assert(sizeof(kMagic) - 1 == 4, "spec host.hello_magic");
static_assert(kHelloLen == 8 + noise::kDhLen, "spec host.hello_len: magic, version, flags, reserved, key");
static_assert(HostServer::kRecordBytes <= 0xFFFF, "a record's length is a u16");

uint32_t elapsed_us(Clock& clock, uint64_t since) { return (uint32_t)(clock.now_us() - since); }

}  // namespace

int32_t find_authorized_key(const AuthorizedKey* keys, uint32_t n, const uint8_t key[32]) {
  uint32_t found = 0, at = 0;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t eq = (uint32_t)(crypto_verify32(keys[i].key, key) + 1);  // 1 if equal, else 0
    uint32_t take = (0u - eq) & ~found;                               // all ones for the first match
    at = (at & ~take) | (i & take);
    found |= take;
  }
  return found != 0 ? (int32_t)at : -1;
}

// ---- the hello -------------------------------------------------------------------

// Read the clear hello and message 1, no more: the client sends nothing
// else before message 2.
void HostServer::read_hello(uint32_t i, uint64_t now) {
  Client& c = clients_[i];
  uint32_t had = c.in_len;
  int32_t n = c.conn->read(c.in + had, kFirstFlight - had);
  if (n < 0) {
    drop(i, "closed");
    return;
  }
  if (n == 0) return;
  c.in_len += (uint32_t)n;
  for (uint32_t k = had; k < c.in_len && k < 4; k++) {
    if (c.in[k] != (uint8_t)kMagic[k]) {
      stats_.hello_refused++;
      drop(i, "closed: no host channel hello (a plaintext or version 1 client?), unanswered");
      return;
    }
  }
  if (had < kHelloLen && c.in_len >= kHelloLen) {
    if (c.in[4] != KILO_HOST_VERSION || c.in[5] != 0 || c.in[6] != 0 || c.in[7] != 0) {
      logf(log_, LogLevel::Warn, "host: client %d speaks host channel version %u (flags %02x), not %u", (int)i,
           (unsigned)c.in[4], (unsigned)c.in[5], (unsigned)KILO_HOST_VERSION);
      return refuse(i, KILO_HOST_HELLO_STATUS_BAD_VERSION, "refused: another host channel version");
    }
    c.key = find_authorized_key(keys_, n_keys_, c.in + 8);
    if (c.key < 0) {
      char fp[17];
      noise::fingerprint(c.in + 8, fp);
      logf(log_, LogLevel::Warn, "host: client %d: key %s is not authorized (%u keys known)", (int)i, fp,
           (unsigned)n_keys_);
      return refuse(i, KILO_HOST_HELLO_STATUS_KEY_UNKNOWN, "refused: an unknown key");
    }
  }
  if (c.in_len == kFirstFlight) hello_complete(i, now);
}

// The clear answer, then close: nothing more is read.
void HostServer::refuse(uint32_t i, uint8_t status, const char* why) {
  Client& c = clients_[i];
  stats_.hello_refused++;
  mem_copy(c.out, kMagic, 4);
  c.out[4] = KILO_HOST_VERSION;
  c.out[5] = status;
  c.out[6] = c.out[7] = 0;
  c.out_head = 0;
  c.out_len = kReplyLen;
  c.closing = true;
  c.closing_why = why;
}

void HostServer::hello_complete(uint32_t i, uint64_t now) {
  Client& c = clients_[i];
  uint8_t e[noise::kDhLen];
  if (random_ == nullptr || !random_->fill(e, sizeof e)) {
    crypto_wipe(e, sizeof e);
    logf(log_, LogLevel::Error, "host: client %d: no random bytes for the handshake: refused", (int)i);
    return refuse(i, KILO_HOST_HELLO_STATUS_REFUSED, "refused: no randomness");
  }
  uint8_t prologue[kPrologueLen];
  mem_copy(prologue, kPrologue, sizeof(kPrologue) - 1);
  mem_copy(prologue + sizeof(kPrologue) - 1, c.in, kHelloLen);
  c.hs.init_responder(prologue, kPrologueLen, keys_[c.key].key, e);
  crypto_wipe(e, sizeof e);
  c.hs.read_message1(c.in + kHelloLen, noise::HandshakeKN::kMsg1Len, nullptr);  // no payload, no key: cannot fail
  c.in_len = 0;
  c.phase = Phase::Keygen;
  c.hs_start_us = now;
  c.hs_slowest_us = 0;
}

// ---- the handshake: one X25519 per pass, for all connections -----------------------

void HostServer::handshake_step(uint64_t now) {
  for (uint32_t k = 0; k < kSlots; k++) {
    uint32_t i = (hs_next_ + k) % kSlots;
    Client& c = clients_[i];
    if (!c.open || c.closing || (c.phase != Phase::Keygen && c.phase != Phase::Ee && c.phase != Phase::Se)) continue;
    hs_next_ = (i + 1) % kSlots;
    uint64_t t0 = clock_.now_us();
    if (c.phase == Phase::Keygen) {
      c.hs.responder_keygen();
      c.phase = Phase::Ee;
    } else if (c.phase == Phase::Ee) {
      if (!c.hs.responder_ee()) {
        stats_.auth_failed++;
        drop(i, "closed: a bad ephemeral key in message 1");
        return;
      }
      c.phase = Phase::Se;
    } else {
      mem_copy(c.out, kMagic, 4);
      c.out[4] = KILO_HOST_VERSION;
      c.out[5] = KILO_HOST_HELLO_STATUS_OK;
      c.out[6] = c.out[7] = 0;
      if (!c.hs.responder_se(nullptr, 0, c.out + kReplyLen, c.send, c.recv)) {
        stats_.auth_failed++;
        drop(i, "closed: a bad static key (se)");
        return;
      }
      c.hs.wipe();
      c.out_head = 0;
      c.out_len = kReplyLen + noise::HandshakeKN::kMsg2Len;
      c.phase = Phase::Proving;
      c.hs_us = elapsed_us(clock_, c.hs_start_us);
    }
    uint32_t dt = elapsed_us(clock_, t0);
    if (dt > c.hs_slowest_us) c.hs_slowest_us = dt;
    (void)now;
    return;  // one per pass
  }
}

// Connections that are not clients after KILO_HOST_HANDSHAKE_TIMEOUT_MS.
void HostServer::expire(uint64_t now) {
  for (uint32_t i = 0; i < kSlots; i++) {
    Client& c = clients_[i];
    if (c.open && c.phase != Phase::Open && now - c.opened_us > kHandshakeUs) {
      stats_.handshake_timeouts++;
      drop(i, "closed: no handshake within the timeout");
    }
  }
}

// ---- records -----------------------------------------------------------------------

// Up to kReadChunk * kReadsPerPoll bytes a pass into the record being read;
// each whole record is opened and its frames handled.
void HostServer::read_records(uint32_t i, uint64_t now) {
  uint32_t budget = kReadChunk * kReadsPerPoll;
  while (budget > 0) {
    Client& c = clients_[i];
    if (!c.open) return;
    uint32_t want;
    if (c.in_len < 2) {
      want = 2 - c.in_len;
    } else {
      uint32_t len = (uint32_t)c.in[0] | ((uint32_t)c.in[1] << 8);
      want = 2 + len - c.in_len;
    }
    if (want > budget) want = budget;
    int32_t n = c.conn->read(c.in + c.in_len, want);
    if (n < 0) {
      drop(i, "closed");
      return;
    }
    if (n == 0) return;
    budget -= (uint32_t)n;
    c.in_len += (uint32_t)n;
    if (c.in_len == 2) {
      uint32_t len = (uint32_t)c.in[0] | ((uint32_t)c.in[1] << 8);
      if (len < kTagLen || len > KILO_HOST_RECORD_MAX + kTagLen) {
        stats_.auth_failed++;
        drop(i, "closed: a bad record length");
        return;
      }
    } else if (c.in_len > 2 && c.in_len == 2 + ((uint32_t)c.in[0] | ((uint32_t)c.in[1] << 8))) {
      record_in(i, now);
    }
  }
}

void HostServer::record_in(uint32_t i, uint64_t now) {
  Client& c = clients_[i];
  uint32_t len = c.in_len - 2;
  c.in_len = 0;
  if (!c.recv.decrypt(nullptr, 0, c.in + 2, len, c.in + 2)) {
    stats_.auth_failed++;
    drop(i, c.recv.nonce() >= noise::kNonceLimit ? "closed: the receive nonce is used up"
                                                 : "closed: a record did not authenticate");
    return;
  }
  if (c.phase == Phase::Proving) {
    c.phase = Phase::Open;
    stats_.handshakes++;
    char fp[17];
    noise::fingerprint(keys_[c.key].key, fp);
    logf(log_, LogLevel::Info, "noise: client %d: handshake with key %s (%s) in %u.%u ms, slowest step %u us",
         (int)i, fp, keys_[c.key].name, (unsigned)(c.hs_us / 1000u), (unsigned)(c.hs_us % 1000u / 100u),
         (unsigned)c.hs_slowest_us);
    logf(log_, LogLevel::Info, "host: client %d is a client (%u of %u)", (int)i, (unsigned)clients(),
         (unsigned)kMaxClients);
  }
  c.last_rx_us = now;
  uint32_t plain = len - kTagLen;
  for (uint32_t k = 0; k < plain; k++) {
    HostRxStatus st = c.rx.push(c.in[2 + k]);
    if (st == HostRxStatus::Frame) {
      handle(i, c.rx.frame(), now);
      if (!c.open) return;
    } else if (st != HostRxStatus::None) {
      stats_.out_of_step++;
      drop(i, st == HostRxStatus::BadCrc ? "bad CRC: out of step" : "bad length: out of step");
      return;
    }
  }
}

// The next record: whole frames from the ring, up to KILO_HOST_RECORD_MAX
// bytes, encrypted in place in c.out. False: the connection was dropped.
bool HostServer::seal(uint32_t i) {
  Client& c = clients_[i];
  uint32_t n = 0;
  while (n < c.tx_len) {
    uint32_t at = c.tx_head + n;
    uint32_t f = 2u + ((uint32_t)c.tx[at % kTxBytes] | ((uint32_t)c.tx[(at + 1) % kTxBytes] << 8));
    if (n + f > KILO_HOST_RECORD_MAX) break;
    n += f;
  }
  uint32_t first = kTxBytes - c.tx_head < n ? kTxBytes - c.tx_head : n;
  mem_copy(c.out + 2, c.tx + c.tx_head, first);
  mem_copy(c.out + 2 + first, c.tx, n - first);
  c.tx_head = (c.tx_head + n) % kTxBytes;
  c.tx_len -= n;
  if (c.tx_len == 0) c.tx_head = 0;
  if (!c.send.encrypt(nullptr, 0, c.out + 2, n, c.out + 2)) {
    drop(i, "closed: the send nonce is used up");
    return false;
  }
  uint32_t len = n + kTagLen;
  c.out[0] = (uint8_t)len;
  c.out[1] = (uint8_t)(len >> 8);
  c.out_head = 0;
  c.out_len = 2 + len;
  return true;
}

}  // namespace kilo
