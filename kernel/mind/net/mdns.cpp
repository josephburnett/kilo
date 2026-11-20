#include "net/mdns.h"
#include "util/fmt.h"
#include "util/mem.h"
#include "util/str.h"

namespace kilo {
namespace mdns {

namespace {

constexpr uint8_t kGroup[4] = {224, 0, 0, 251};
constexpr uint16_t kFlagResponse = 0x8400;  // QR, AA
constexpr const char* kService = "_kilo._tcp.local";
constexpr const char* kServices = "_services._dns-sd._udp.local";
constexpr uint32_t kTextMax = 256;  // a dotted name read or built

// The records the responder has, as bits.
enum : uint8_t { RecA = 1, RecPtr = 2, RecSrv = 4, RecTxt = 8, RecSvc = 16 };

uint16_t rd16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }

bool zero_ip(const uint8_t ip[4]) { return (ip[0] | ip[1] | ip[2] | ip[3]) == 0; }
bool same_ip(const uint8_t a[4], const uint8_t b[4]) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3];
}

// A datagram being built; any overflow marks it bad (never sent).
class Writer {
 public:
  explicit Writer(Out* out) : b_(out->data) {}
  void u8(uint8_t v) {
    if (n_ + 1 > kMaxOut) ok_ = false;
    else b_[n_++] = v;
  }
  void u16(uint16_t v) {
    u8((uint8_t)(v >> 8));
    u8((uint8_t)v);
  }
  void u32(uint32_t v) {
    u16((uint16_t)(v >> 16));
    u16((uint16_t)v);
  }
  void bytes(const void* p, uint32_t n) {
    const uint8_t* s = static_cast<const uint8_t*>(p);
    for (uint32_t i = 0; i < n; i++) u8(s[i]);
  }
  // A dotted name as labels, uncompressed.
  void name(const char* dotted) {
    const char* s = dotted;
    while (*s != '\0') {
      uint32_t len = 0;
      while (s[len] != '\0' && s[len] != '.') len++;
      if (len == 0 || len > 63) {
        ok_ = false;
        return;
      }
      u8((uint8_t)len);
      bytes(s, len);
      s += len;
      if (*s == '.') s++;
    }
    u8(0);
  }
  void patch16(uint32_t at, uint16_t v) {
    if (at + 2 <= n_) {
      b_[at] = (uint8_t)(v >> 8);
      b_[at + 1] = (uint8_t)v;
    }
  }
  uint32_t size() const { return n_; }
  bool ok() const { return ok_; }

 private:
  uint8_t* b_;
  uint32_t n_ = 0;
  bool ok_ = true;
};

void header(Writer& w, uint16_t id, uint16_t flags, uint16_t qd, uint16_t an, uint16_t ns, uint16_t ar) {
  w.u16(id);
  w.u16(flags);
  w.u16(qd);
  w.u16(an);
  w.u16(ns);
  w.u16(ar);
}

// Skips one resource record's fixed part and data at *off; its type and
// the name's text are returned. False if malformed.
bool read_rr(const uint8_t* p, uint32_t n, uint32_t* off, char* name, uint16_t* type, const uint8_t** rdata,
             uint16_t* rdlen) {
  uint32_t o;
  if (!read_name(p, n, *off, name, kTextMax, &o)) return false;
  if (o + 10 > n) return false;
  *type = rd16(p + o);
  *rdlen = rd16(p + o + 8);
  o += 10;
  if (o + *rdlen > n) return false;
  *rdata = p + o;
  *off = o + *rdlen;
  return true;
}

}  // namespace

bool read_name(const uint8_t* p, uint32_t n, uint32_t off, char* out, uint32_t outn, uint32_t* next) {
  uint32_t w = 0;
  bool jumped = false;
  int hops = 0;
  if (outn == 0) return false;
  for (;;) {
    if (off >= n) return false;
    uint8_t len = p[off];
    if (len == 0) {
      if (!jumped) *next = off + 1;
      break;
    }
    if ((len & 0xC0) == 0xC0) {  // a pointer to an earlier name
      if (off + 1 >= n || ++hops > 16) return false;
      uint32_t to = (uint32_t)((len & 0x3F) << 8) | p[off + 1];
      if (!jumped) *next = off + 2;
      jumped = true;
      if (to >= off) return false;  // only backwards: no loops
      off = to;
      continue;
    }
    if ((len & 0xC0) != 0 || off + 1 + len > n) return false;
    if (w != 0) {
      if (w + 1 >= outn) return false;
      out[w++] = '.';
    }
    if (w + len >= outn) return false;
    for (uint32_t i = 0; i < len; i++) {
      char c = (char)p[off + 1 + i];
      if (c == '\0' || c == '.') return false;  // not a name we could compare
      out[w++] = c;
    }
    off += 1u + len;
  }
  out[w] = '\0';
  return true;
}

// ---- the responder -----------------------------------------------------------------

void Responder::begin(const char* name, uint16_t port, uint32_t build) {
  uint32_t i = 0;
  while (name[i] != '\0' && i < 20) {  // room for "-NN"
    base_[i] = name[i];
    i++;
  }
  base_[i] = '\0';
  mem_copy(name_, base_, i + 1);
  suffix_ = 1;
  port_ = port;
  format(txt_, sizeof txt_, "build=%08x", (unsigned)build);
  mem_zero(ip_, sizeof ip_);
  state_ = State::Idle;
  step_ = 0;
  stats_ = {};
}

void Responder::rename() {
  suffix_++;
  format(name_, sizeof name_, "%s-%u", base_, (unsigned)suffix_);
}

void Responder::set_address(const uint8_t ip[4], uint64_t now_us) {
  if (same_ip(ip, ip_)) return;
  mem_copy(ip_, ip, 4);
  if (zero_ip(ip_)) return;  // poll() waits for one
  // the first address: claim the name; a later one: tell the caches
  state_ = claimed() ? State::Announcing : State::Probing;
  step_ = 0;
  due_us_ = now_us;
}

bool Responder::allow(uint64_t now_us) {
  if (now_us - window_us_ >= 1000000u || now_us < window_us_) {
    window_us_ = now_us;
    window_n_ = 0;
  }
  if (window_n_ >= kRepliesPerSecond) {
    stats_.limited++;
    return false;
  }
  window_n_++;
  return true;
}

namespace {

struct Names {
  char host[kNameMax + 8];
  char instance[kNameMax + 24];
};

void names_of(const char* name, Names* n) {
  format(n->host, sizeof n->host, "%s.local", name);
  format(n->instance, sizeof n->instance, "%s.%s", name, kService);
}

// One record; legacy: short TTLs and no cache-flush bits (RFC 6762 6.7).
void record(Writer& w, uint8_t rec, const Names& nm, const uint8_t ip[4], uint16_t port, const char* txt,
            bool legacy) {
  uint16_t flush = legacy ? 0 : kClassFlush;
  uint32_t host_ttl = legacy ? kTtlLegacy : kTtlHost;
  uint32_t shared_ttl = legacy ? kTtlLegacy : kTtlShared;
  uint32_t len_at;
  switch (rec) {
    case RecA:
      w.name(nm.host);
      w.u16(kTypeA);
      w.u16((uint16_t)(kClassIn | flush));
      w.u32(host_ttl);
      w.u16(4);
      w.bytes(ip, 4);
      break;
    case RecPtr:
      w.name(kService);
      w.u16(kTypePtr);
      w.u16(kClassIn);  // shared: never flushed
      w.u32(shared_ttl);
      len_at = w.size();
      w.u16(0);
      w.name(nm.instance);
      w.patch16(len_at, (uint16_t)(w.size() - len_at - 2));
      break;
    case RecSvc:
      w.name(kServices);
      w.u16(kTypePtr);
      w.u16(kClassIn);
      w.u32(shared_ttl);
      len_at = w.size();
      w.u16(0);
      w.name(kService);
      w.patch16(len_at, (uint16_t)(w.size() - len_at - 2));
      break;
    case RecSrv:
      w.name(nm.instance);
      w.u16(kTypeSrv);
      w.u16((uint16_t)(kClassIn | flush));
      w.u32(host_ttl);
      len_at = w.size();
      w.u16(0);
      w.u16(0);  // priority
      w.u16(0);  // weight
      w.u16(port);
      w.name(nm.host);
      w.patch16(len_at, (uint16_t)(w.size() - len_at - 2));
      break;
    case RecTxt: {
      uint32_t n = 0;
      while (txt[n] != '\0') n++;
      w.name(nm.instance);
      w.u16(kTypeTxt);
      w.u16((uint16_t)(kClassIn | flush));
      w.u32(shared_ttl);
      w.u16((uint16_t)(n + 1));
      w.u8((uint8_t)n);
      w.bytes(txt, n);
      break;
    }
  }
}

int count(uint8_t set) {
  int c = 0;
  for (; set != 0; set &= (uint8_t)(set - 1)) c++;
  return c;
}

void records(Writer& w, uint8_t set, const Names& nm, const uint8_t ip[4], uint16_t port, const char* txt,
             bool legacy) {
  static const uint8_t kOrder[] = {RecPtr, RecSvc, RecSrv, RecTxt, RecA};
  for (uint8_t r : kOrder)
    if (set & r) record(w, r, nm, ip, port, txt, legacy);
}

}  // namespace

bool Responder::conflicting(const uint8_t* p, uint32_t n, uint32_t off, uint16_t ancount) const {
  char host[kNameMax + 8];
  format(host, sizeof host, "%s.local", name_);
  char name[kTextMax];
  for (uint16_t i = 0; i < ancount; i++) {
    uint16_t type, rdlen;
    const uint8_t* rd;
    if (!read_rr(p, n, &off, name, &type, &rd, &rdlen)) return false;
    if (!str_ieq(name, host)) continue;
    if (state_ == State::Probing) return true;  // anyone else answering for the name we want
    if (type == kTypeA && rdlen == 4 && !same_ip(rd, ip_)) return true;  // ours, with another address
  }
  return false;
}

bool Responder::receive(const uint8_t* p, uint32_t n, const uint8_t src[4], uint16_t sport, uint64_t now_us,
                        Out* out) {
  if (state_ == State::Idle || zero_ip(ip_)) return false;
  if (n < 12) {
    stats_.ignored++;
    return false;
  }
  if (same_ip(src, ip_)) return false;  // our own, looped back
  uint16_t id = rd16(p), flags = rd16(p + 2), qd = rd16(p + 4), an = rd16(p + 6);
  if (((flags >> 11) & 0xF) != 0) {  // not a standard query or response
    stats_.ignored++;
    return false;
  }
  // the questions (a response may echo them)
  Names nm;
  names_of(name_, &nm);
  char name[kTextMax];
  uint32_t off = 12;
  uint8_t answer = 0;
  bool all_qu = true;
  for (uint16_t i = 0; i < qd; i++) {
    uint32_t next;
    if (!read_name(p, n, off, name, sizeof name, &next) || next + 4 > n) {
      stats_.ignored++;
      return false;
    }
    uint16_t qtype = rd16(p + next), qclass = rd16(p + next + 2);
    off = next + 4;
    uint16_t cls = (uint16_t)(qclass & ~kClassFlush);
    if (cls != kClassIn && cls != kTypeAny) continue;
    bool any = qtype == kTypeAny;
    uint8_t asked = 0;
    if (str_ieq(name, nm.host) && (any || qtype == kTypeA)) asked |= RecA;
    if (str_ieq(name, kService) && (any || qtype == kTypePtr)) asked |= RecPtr;
    if (str_ieq(name, kServices) && (any || qtype == kTypePtr)) asked |= RecSvc;
    if (str_ieq(name, nm.instance)) {
      if (any || qtype == kTypeSrv) asked |= RecSrv;
      if (any || qtype == kTypeTxt) asked |= RecTxt;
    }
    if (asked != 0 && !(qclass & kClassFlush)) all_qu = false;
    answer |= asked;
  }
  uint32_t questions_end = off;

  if (flags & 0x8000) {  // a response: does another host claim our name?
    if (conflicting(p, n, off, an)) {
      stats_.conflicts++;
      rename();
      state_ = State::Probing;
      step_ = 0;
      due_us_ = now_us;
    }
    return false;
  }

  stats_.queries++;
  if (answer == 0 || !claimed()) return false;  // not ours, or not ours yet
  if (!allow(now_us)) return false;

  // the related records go in the additional section (RFC 6763 12)
  uint8_t extra = 0;
  if (answer & RecPtr) extra |= RecSrv | RecTxt | RecA;
  if (answer & RecSrv) extra |= RecA;
  extra &= (uint8_t)~answer;

  bool legacy = sport != kPort;
  Writer w(out);
  uint32_t qbytes = legacy ? questions_end - 12 : 0;
  header(w, legacy ? id : 0, kFlagResponse, legacy ? qd : 0, (uint16_t)count(answer), 0, (uint16_t)count(extra));
  if (legacy) w.bytes(p + 12, qbytes);  // at the same offset: their compression pointers stay valid
  records(w, answer, nm, ip_, port_, txt_, legacy);
  records(w, extra, nm, ip_, port_, txt_, legacy);
  if (!w.ok()) {
    stats_.ignored++;
    return false;
  }
  out->len = w.size();
  if (legacy) {
    mem_copy(out->ip, src, 4);
    out->port = sport;
    stats_.legacy++;
  } else if (all_qu) {
    mem_copy(out->ip, src, 4);
    out->port = kPort;
    stats_.unicast++;
  } else {
    mem_copy(out->ip, kGroup, 4);
    out->port = kPort;
  }
  stats_.answered++;
  return true;
}

void Responder::probe(Out* out) {
  Names nm;
  names_of(name_, &nm);
  Writer w(out);
  header(w, 0, 0, 1, 0, 1, 0);
  w.name(nm.host);
  w.u16(kTypeAny);
  w.u16((uint16_t)(kClassIn | kClassFlush));  // QU
  // the authority section: what we are about to claim (RFC 6762 8.2)
  w.name(nm.host);
  w.u16(kTypeA);
  w.u16(kClassIn);
  w.u32(kTtlHost);
  w.u16(4);
  w.bytes(ip_, 4);
  out->len = w.size();
  mem_copy(out->ip, kGroup, 4);
  out->port = kPort;
  stats_.probes++;
}

void Responder::announce(Out* out) {
  Names nm;
  names_of(name_, &nm);
  Writer w(out);
  uint8_t all = RecA | RecPtr | RecSrv | RecTxt | RecSvc;
  header(w, 0, kFlagResponse, 0, (uint16_t)count(all), 0, 0);
  records(w, all, nm, ip_, port_, txt_, false);
  out->len = w.size();
  mem_copy(out->ip, kGroup, 4);
  out->port = kPort;
  stats_.announcements++;
}

bool Responder::poll(uint64_t now_us, Out* out) {
  if (zero_ip(ip_) || now_us < due_us_) return false;
  if (state_ == State::Probing) {
    if (step_ < kProbes) {
      probe(out);
      step_++;
      due_us_ = now_us + kProbeGapUs;
      return true;
    }
    state_ = State::Announcing;  // three probes and no one objected: the name is ours
    step_ = 0;
  }
  if (state_ == State::Announcing) {
    if (step_ < kAnnouncements) {
      announce(out);
      step_++;
      due_us_ = now_us + kAnnounceGapUs;
      return true;
    }
    state_ = State::Claimed;
  }
  return false;
}

}  // namespace mdns
}  // namespace kilo
