// The mDNS responder (net/mdns.h): the name reader, probing and the name
// conflict, the announcements, and what each kind of query gets back and
// where it goes (legacy unicast, QU, multicast); malformed datagrams and a
// flood of queries. The laptop's side (mind/kilo/mdns.py) has its own tests
// against the same wire format (mind/tests/test_mdns.py).
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "net/mdns.h"
#include "util/str.h"
#include "tests/check.h"

using namespace kilo;
using namespace kilo::mdns;

namespace {

const uint8_t kIp[4] = {192, 168, 88, 9};
const uint8_t kLaptop[4] = {192, 168, 88, 20};
const uint8_t kGroupIp[4] = {224, 0, 0, 251};

// ---- a query builder and a response parser, independent of the responder ----------

struct Q {
  std::string name;
  uint16_t type;
  bool qu;
};

void put16(std::vector<uint8_t>& b, uint16_t v) {
  b.push_back((uint8_t)(v >> 8));
  b.push_back((uint8_t)v);
}

void put_name(std::vector<uint8_t>& b, const std::string& dotted) {
  size_t i = 0;
  while (i < dotted.size()) {
    size_t j = dotted.find('.', i);
    if (j == std::string::npos) j = dotted.size();
    b.push_back((uint8_t)(j - i));
    b.insert(b.end(), dotted.begin() + (long)i, dotted.begin() + (long)j);
    i = j + 1;
  }
  b.push_back(0);
}

std::vector<uint8_t> query(uint16_t id, const std::vector<Q>& qs) {
  std::vector<uint8_t> b;
  put16(b, id);
  put16(b, 0);
  put16(b, (uint16_t)qs.size());
  put16(b, 0);
  put16(b, 0);
  put16(b, 0);
  for (const Q& q : qs) {
    put_name(b, q.name);
    put16(b, q.type);
    put16(b, (uint16_t)(kClassIn | (q.qu ? kClassFlush : 0)));
  }
  return b;
}

struct RR {
  std::string name;
  uint16_t type, cls;
  uint32_t ttl;
  std::vector<uint8_t> rdata;
  std::string target;  // PTR, SRV: the name in the data
  uint16_t port = 0;   // SRV
  std::string text;    // TXT: the first string
  int section;         // 1 answer, 2 authority, 3 additional
};

struct Msg {
  bool ok = false;
  uint16_t id = 0, flags = 0;
  std::vector<std::string> questions;
  std::vector<RR> rrs;
  const RR* find(const std::string& name, uint16_t type, int section = 0) const {
    for (const RR& r : rrs)
      if (str_ieq(r.name.c_str(), name.c_str()) && r.type == type && (section == 0 || r.section == section)) return &r;
    return nullptr;
  }
};

uint16_t g16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }

Msg parse(const uint8_t* p, uint32_t n) {
  Msg m;
  if (n < 12) return m;
  m.id = g16(p);
  m.flags = g16(p + 2);
  uint16_t counts[4] = {g16(p + 4), g16(p + 6), g16(p + 8), g16(p + 10)};
  uint32_t off = 12;
  char name[256];
  for (int i = 0; i < counts[0]; i++) {
    uint32_t next;
    if (!read_name(p, n, off, name, sizeof name, &next) || next + 4 > n) return m;
    m.questions.push_back(name);
    off = next + 4;
  }
  for (int s = 1; s <= 3; s++)
    for (int i = 0; i < counts[s]; i++) {
      RR r;
      uint32_t next;
      if (!read_name(p, n, off, name, sizeof name, &next) || next + 10 > n) return m;
      r.name = name;
      r.type = g16(p + next);
      r.cls = g16(p + next + 2);
      r.ttl = ((uint32_t)g16(p + next + 4) << 16) | g16(p + next + 6);
      uint16_t len = g16(p + next + 8);
      uint32_t rd = next + 10;
      if (rd + len > n) return m;
      r.rdata.assign(p + rd, p + rd + len);
      uint32_t dummy;
      if (r.type == kTypePtr && read_name(p, n, rd, name, sizeof name, &dummy)) r.target = name;
      if (r.type == kTypeSrv && len >= 7) {
        r.port = g16(p + rd + 4);
        if (read_name(p, n, rd + 6, name, sizeof name, &dummy)) r.target = name;
      }
      if (r.type == kTypeTxt && len >= 1 && p[rd] + 1u <= len) r.text.assign((const char*)p + rd + 1, p[rd]);
      r.section = s;
      m.rrs.push_back(r);
      off = rd + len;
    }
  m.ok = off == n;
  return m;
}

bool ip_is(const uint8_t a[4], const uint8_t b[4]) { return std::memcmp(a, b, 4) == 0; }

// A responder past its probes and announcements, at t (us).
struct Claimed {
  Responder r;
  uint64_t t = 0;
  Out out;
  Claimed() {
    r.begin("kilo", 5300, 0x2d7789e1);
    r.set_address(kIp, t);
    for (int i = 0; i < 40; i++) {
      t += 100000;
      while (r.poll(t, &out)) {
      }
    }
  }
  bool ask(const std::vector<uint8_t>& q, uint16_t sport, const uint8_t* from = kLaptop) {
    t += 1000;
    return r.receive(q.data(), (uint32_t)q.size(), from, sport, t, &out);
  }
  Msg reply() const { return parse(out.data, out.len); }
};

// ---- the name reader --------------------------------------------------------------------

void test_read_name() {
  char out[64];
  uint32_t next = 0;
  // "kilo.local" at 0, then "_kilo._tcp" + a pointer to "local" (offset 5)
  std::vector<uint8_t> b;
  put_name(b, "kilo.local");
  b.insert(b.end(), {5, '_', 'k', 'i', 'l', 'o', 4, '_', 't', 'c', 'p', 0xC0, 5});
  CHECK(read_name(b.data(), (uint32_t)b.size(), 0, out, sizeof out, &next) && std::string(out) == "kilo.local" &&
        next == 12);
  CHECK(read_name(b.data(), (uint32_t)b.size(), 12, out, sizeof out, &next) &&
        std::string(out) == "_kilo._tcp.local" && next == b.size());
  CHECK(str_ieq("KILO.Local", "kilo.local") && !str_ieq("kilo.local", "kilo.locale") && !str_ieq("kilo", "kilo-2"));
  // a pointer to itself, a pointer forward, a cut name, a too-long name: refused
  const uint8_t self[] = {0xC0, 0x00};
  CHECK(!read_name(self, sizeof self, 0, out, sizeof out, &next));
  const uint8_t fwd[] = {0xC0, 0x02, 0x00};
  CHECK(!read_name(fwd, sizeof fwd, 0, out, sizeof out, &next));
  const uint8_t cut[] = {4, 'k', 'i'};
  CHECK(!read_name(cut, sizeof cut, 0, out, sizeof out, &next));
  CHECK(!read_name(b.data(), (uint32_t)b.size(), 0, out, 8, &next));
  const uint8_t dot[] = {3, 'a', '.', 'b', 0};  // a dot inside a label cannot be compared as text
  CHECK(!read_name(dot, sizeof dot, 0, out, sizeof out, &next));
}

// ---- probing and the announcements ---------------------------------------------------------

void test_probe_then_announce() {
  Responder r;
  Out out;
  r.begin("kilo", 5300, 0x2d7789e1);
  CHECK(!r.poll(0, &out) && r.state() == Responder::State::Idle);  // no address: silent
  r.set_address(kIp, 1000);
  std::vector<uint64_t> probes, announcements;
  for (uint64_t t = 1000; t < 6000000; t += 1000) {
    while (r.poll(t, &out)) {
      Msg m = parse(out.data, out.len);
      CHECK(m.ok && ip_is(out.ip, kGroupIp) && out.port == 5353);
      if (m.flags == 0) {
        probes.push_back(t);
        // a query for our name (ANY, QU) with what we claim in the authority section
        CHECK(m.questions.size() == 1 && m.questions[0] == "kilo.local");
        const RR* a = m.find("kilo.local", kTypeA, 2);
        CHECK(a != nullptr && a->rdata == std::vector<uint8_t>(kIp, kIp + 4));
      } else {
        announcements.push_back(t);
        CHECK(m.flags == 0x8400);
        const RR* a = m.find("kilo.local", kTypeA);
        const RR* ptr = m.find("_kilo._tcp.local", kTypePtr);
        const RR* srv = m.find("kilo._kilo._tcp.local", kTypeSrv);
        const RR* txt = m.find("kilo._kilo._tcp.local", kTypeTxt);
        const RR* svc = m.find("_services._dns-sd._udp.local", kTypePtr);
        CHECK(a && a->rdata == std::vector<uint8_t>(kIp, kIp + 4) && a->cls == (kClassIn | kClassFlush) &&
              a->ttl == kTtlHost);
        CHECK(ptr && ptr->target == "kilo._kilo._tcp.local" && ptr->cls == kClassIn);
        CHECK(srv && srv->port == 5300 && srv->target == "kilo.local");
        CHECK(txt && txt->text == "build=2d7789e1");
        CHECK(svc && svc->target == "_kilo._tcp.local");
      }
    }
    if (t == 100000) {  // a query while probing: the name is not ours yet
      std::vector<uint8_t> q = query(1, {{"kilo.local", kTypeA, false}});
      CHECK(!r.receive(q.data(), (uint32_t)q.size(), kLaptop, 40000, t, &out));
    }
  }
  CHECKF(probes.size() == 3 && probes[1] - probes[0] == kProbeGapUs && probes[2] - probes[1] == kProbeGapUs,
         "%zu probes", probes.size());
  CHECKF(announcements.size() == 3 && announcements[0] - probes[2] == kProbeGapUs &&
             announcements[1] - announcements[0] == kAnnounceGapUs,
         "%zu announcements", announcements.size());
  CHECK(r.state() == Responder::State::Claimed && std::string(r.hostname()) == "kilo");
  CHECK(r.stats().probes == 3 && r.stats().announcements == 3);
}

void test_conflict_while_probing() {
  Responder r;
  Out out;
  r.begin("kilo", 5300, 1);
  r.set_address(kIp, 0);
  CHECK(r.poll(0, &out));  // the first probe
  // another host answers for kilo.local
  const uint8_t other[4] = {192, 168, 88, 30};
  std::vector<uint8_t> resp;
  put16(resp, 0);
  put16(resp, 0x8400);
  put16(resp, 0);
  put16(resp, 1);
  put16(resp, 0);
  put16(resp, 0);
  put_name(resp, "kilo.local");
  put16(resp, kTypeA);
  put16(resp, kClassIn | kClassFlush);
  put16(resp, 0);
  put16(resp, 120);
  put16(resp, 4);
  resp.insert(resp.end(), other, other + 4);
  CHECK(!r.receive(resp.data(), (uint32_t)resp.size(), other, 5353, 100000, &out));
  CHECK(std::string(r.hostname()) == "kilo-2" && r.state() == Responder::State::Probing && r.stats().conflicts == 1);
  // the probes start again, for the new name
  CHECK(r.poll(100000, &out) && parse(out.data, out.len).questions[0] == "kilo-2.local");
  for (uint64_t t = 100000; t < 5000000; t += 10000)
    while (r.poll(t, &out)) {
    }
  CHECK(r.claimed());
  // kilo-2.local is answered, kilo.local is not
  std::vector<uint8_t> q = query(7, {{"kilo.local", kTypeA, false}});
  CHECK(!r.receive(q.data(), (uint32_t)q.size(), kLaptop, 40000, 5000000, &out));
  q = query(8, {{"kilo-2.local", kTypeA, false}});
  CHECK(r.receive(q.data(), (uint32_t)q.size(), kLaptop, 40000, 5001000, &out));
  Msg m = parse(out.data, out.len);
  CHECK(m.find("kilo-2.local", kTypeA) != nullptr);
  // the same answer with our own address, from someone else (a cache): no conflict
  std::vector<uint8_t> agree;
  put16(agree, 0);
  put16(agree, 0x8400);
  put16(agree, 0);
  put16(agree, 1);
  put16(agree, 0);
  put16(agree, 0);
  put_name(agree, "kilo-2.local");
  put16(agree, kTypeA);
  put16(agree, kClassIn | kClassFlush);
  put16(agree, 0);
  put16(agree, 120);
  put16(agree, 4);
  agree.insert(agree.end(), kIp, kIp + 4);
  r.receive(agree.data(), (uint32_t)agree.size(), other, 5353, 5002000, &out);
  CHECK(std::string(r.hostname()) == "kilo-2" && r.claimed());
  // ... and with another address: a conflict even once claimed
  std::memcpy(&agree[agree.size() - 4], other, 4);
  r.receive(agree.data(), (uint32_t)agree.size(), other, 5353, 5003000, &out);
  CHECK(std::string(r.hostname()) == "kilo-3" && r.state() == Responder::State::Probing);
}

void test_new_address_is_announced() {
  Claimed c;
  const uint8_t lease[4] = {192, 168, 88, 28};
  c.r.set_address(lease, c.t);
  int probes = 0, announcements = 0;
  for (int i = 0; i < 50; i++) {
    c.t += 100000;
    while (c.r.poll(c.t, &c.out)) {
      Msg m = c.reply();
      if (m.flags == 0) probes++;
      else {
        announcements++;
        const RR* a = m.find("kilo.local", kTypeA);
        CHECK(a && a->rdata == std::vector<uint8_t>(lease, lease + 4));
      }
    }
  }
  CHECK(probes == 0 && announcements == 3);
}

// ---- answers --------------------------------------------------------------------------

void test_legacy_unicast_a() {
  Claimed c;
  std::vector<uint8_t> q = query(0x1234, {{"KILO.local", kTypeA, false}});
  CHECK(c.ask(q, 40123));
  Msg m = c.reply();
  CHECK(m.ok && ip_is(c.out.ip, kLaptop) && c.out.port == 40123);  // straight back to the asker's port
  CHECK(m.id == 0x1234 && m.flags == 0x8400 && m.questions.size() == 1 && m.questions[0] == "KILO.local");
  const RR* a = m.find("kilo.local", kTypeA, 1);
  CHECK(a && a->rdata == std::vector<uint8_t>(kIp, kIp + 4) && a->ttl == kTtlLegacy && a->cls == kClassIn);
  CHECK(c.r.stats().legacy == 1 && c.r.stats().answered == 1);
}

void test_multicast_and_qu() {
  Claimed c;
  // from port 5353, no QU: on the group, ID 0, no questions
  CHECK(c.ask(query(55, {{"kilo.local", kTypeA, false}}), 5353));
  Msg m = c.reply();
  CHECK(m.ok && ip_is(c.out.ip, kGroupIp) && c.out.port == 5353 && m.id == 0 && m.questions.empty());
  const RR* a = m.find("kilo.local", kTypeA);
  CHECK(a && a->cls == (kClassIn | kClassFlush) && a->ttl == kTtlHost);
  // QU: unicast to the sender's 5353
  CHECK(c.ask(query(56, {{"kilo.local", kTypeAny, true}}), 5353));
  CHECK(ip_is(c.out.ip, kLaptop) && c.out.port == 5353 && c.r.stats().unicast == 1);
}

void test_service_queries() {
  Claimed c;
  CHECK(c.ask(query(9, {{"_kilo._tcp.local", kTypePtr, false}}), 41000));
  Msg m = c.reply();
  CHECK(m.ok);
  const RR* ptr = m.find("_kilo._tcp.local", kTypePtr, 1);
  CHECK(ptr && ptr->target == "kilo._kilo._tcp.local");
  // the rest in the additional section: where (SRV), the build (TXT), the address (A)
  const RR* srv = m.find("kilo._kilo._tcp.local", kTypeSrv, 3);
  const RR* txt = m.find("kilo._kilo._tcp.local", kTypeTxt, 3);
  const RR* a = m.find("kilo.local", kTypeA, 3);
  CHECK(srv && srv->port == 5300 && txt && txt->text == "build=2d7789e1" && a);
  // the instance: SRV and TXT; then ANY on it
  CHECK(c.ask(query(10, {{"kilo._kilo._tcp.local", kTypeTxt, false}}), 41000));
  m = c.reply();
  CHECK(m.find("kilo._kilo._tcp.local", kTypeTxt, 1) && !m.find("kilo._kilo._tcp.local", kTypeSrv));
  CHECK(c.ask(query(11, {{"kilo._kilo._tcp.local", kTypeAny, false}}), 41000));
  m = c.reply();
  CHECK(m.find("kilo._kilo._tcp.local", kTypeTxt, 1) && m.find("kilo._kilo._tcp.local", kTypeSrv, 1) &&
        m.find("kilo.local", kTypeA, 3));
  // service browsing
  CHECK(c.ask(query(12, {{"_services._dns-sd._udp.local", kTypePtr, false}}), 41000));
  m = c.reply();
  const RR* svc = m.find("_services._dns-sd._udp.local", kTypePtr, 1);
  CHECK(svc && svc->target == "_kilo._tcp.local");
  // two questions in one query, the second with a compressed name: both answered
  std::vector<uint8_t> q = query(13, {{"kilo.local", kTypeA, false}});
  q[5] = 2;  // QDCOUNT
  q.insert(q.end(), {5, '_', 'k', 'i', 'l', 'o', 4, '_', 't', 'c', 'p', 0xC0, 17});  // "local" at 12 + 5
  put16(q, kTypePtr);
  put16(q, kClassIn);
  CHECK(c.ask(q, 41000));
  m = c.reply();
  CHECK(m.ok && m.questions.size() == 2 && m.questions[1] == "_kilo._tcp.local" &&
        m.find("kilo.local", kTypeA, 1) && m.find("_kilo._tcp.local", kTypePtr, 1));
}

void test_not_ours() {
  Claimed c;
  CHECK(!c.ask(query(1, {{"printer.local", kTypeA, false}}), 41000));
  CHECK(!c.ask(query(2, {{"kilo.local", kTypeAaaa, false}}), 41000));  // no IPv6 address
  CHECK(!c.ask(query(3, {{"_http._tcp.local", kTypePtr, false}}), 5353));
  CHECK(!c.ask(query(4, {{"kilo.local", kTypeA, false}}), 41000, kIp));  // our own, looped back
  CHECK(c.r.stats().answered == 0);
}

void test_malformed_and_flood() {
  Claimed c;
  uint32_t ignored = c.r.stats().ignored;
  std::vector<uint8_t> shortp = {0, 1, 0, 0};
  CHECK(!c.ask(shortp, 41000));
  std::vector<uint8_t> cut = query(5, {{"kilo.local", kTypeA, false}});
  cut.resize(cut.size() - 3);
  CHECK(!c.ask(cut, 41000));
  CHECK(c.r.stats().ignored == ignored + 2);
  // random bytes (the sanitizers watch): never an answer out of bounds
  std::srand(7);
  for (int i = 0; i < 20000; i++) {
    std::vector<uint8_t> junk = query((uint16_t)i, {{"kilo.local", kTypeA, false}});
    int flips = 1 + std::rand() % 6;
    for (int k = 0; k < flips; k++) junk[(size_t)std::rand() % junk.size()] = (uint8_t)std::rand();
    junk.resize((size_t)std::rand() % (junk.size() + 8) + 1, 0xC0);
    c.t += 60000;  // under the reply limit
    if (c.r.receive(junk.data(), (uint32_t)junk.size(), kLaptop, 41000, c.t, &c.out))
      CHECK(c.out.len <= kMaxOut && parse(c.out.data, c.out.len).ok);
  }
  // a flood: at most kRepliesPerSecond answers a second
  Claimed f;
  int sent = 0;
  for (int i = 0; i < 100; i++) {
    std::vector<uint8_t> q = query((uint16_t)i, {{"kilo.local", kTypeA, false}});
    sent += f.r.receive(q.data(), (uint32_t)q.size(), kLaptop, 41000, f.t + (uint64_t)i * 1000, &f.out);
  }
  CHECKF(sent == (int)kRepliesPerSecond && f.r.stats().limited == 100 - kRepliesPerSecond, "%d sent", sent);
}

}  // namespace

int main() {
  test_read_name();
  test_probe_then_announce();
  test_conflict_while_probing();
  test_new_address_is_announced();
  test_legacy_unicast_a();
  test_multicast_and_qu();
  test_service_queries();
  test_not_ours();
  test_malformed_and_flood();
  return report("test_mdns");
}
