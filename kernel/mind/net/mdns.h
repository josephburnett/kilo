// mdns: the mind's multicast DNS responder (RFC 6762 and DNS-SD, RFC 6763),
// so a laptop finds Kilo by name, whatever address DHCP gave him today:
// `kilo find`, `kilo status --host kilo.local` (mind/kilo/mdns.py).
//
// What it answers, on UDP port 5353:
//   kilo.local                    A      the mind's address
//   _kilo._tcp.local              PTR    kilo._kilo._tcp.local (the laptop service)
//   kilo._kilo._tcp.local         SRV    port KILO_HOST_PORT on kilo.local
//   kilo._kilo._tcp.local         TXT    "build=xxxxxxxx" (the mind's build)
//   _services._dns-sd._udp.local  PTR    _kilo._tcp.local (service browsers)
// (ANY asks for all of a name's records.) "kilo" is the name it claims: if
// another host on the network already answers for kilo.local, it takes
// kilo-2, then kilo-3, ...
//
// Where the answer goes, as RFC 6762 asks:
//   - a query from a port other than 5353 (a "legacy unicast" query, as
//     simple resolvers send, `kilo find` among them) gets its answer by
//     unicast to that address and port, with the query's ID and questions;
//   - a query from port 5353 with the "unicast response" bit (QU) on its
//     question gets a unicast answer to the sender's port 5353;
//   - any other query gets its answer on the multicast group 224.0.0.251.
//
// The name: when it gets an address the responder first probes (three
// queries for its name, 250 ms apart; an answer from another host is a
// conflict: the next name), then announces all its records three times, a
// second apart. A new address (DHCP after the link-local fallback, a new
// lease) is announced again the same way, without probing.
//
// Pure logic: datagrams in, datagrams out, the time passed in. The platform
// owns the socket (the Circle kernel: kernel/src/net.cpp CKiloMdns, a task of
// its own that never blocks). No allocation; one reply in flight.
#ifndef KILO_MIND_NET_MDNS_H
#define KILO_MIND_NET_MDNS_H
#include <stdint.h>

namespace kilo {
namespace mdns {

constexpr uint16_t kPort = 5353;
constexpr uint32_t kMaxOut = 512;  // our datagrams; a query read may be larger (the platform's buffer)
constexpr uint32_t kNameMax = 32;  // the claimed host name, null included
constexpr uint32_t kTtlHost = 120;     // A, SRV: the records that change with the address
constexpr uint32_t kTtlShared = 4500;  // PTR, TXT
constexpr uint32_t kTtlLegacy = 10;    // every record in a legacy unicast answer (RFC 6762 6.7)
constexpr uint64_t kProbeGapUs = 250000;
constexpr uint64_t kAnnounceGapUs = 1000000;
constexpr int kProbes = 3;
constexpr int kAnnouncements = 3;
// Replies at most this many a second (a flood of queries costs bounded sends).
constexpr uint32_t kRepliesPerSecond = 20;

// DNS types and classes the responder uses.
enum : uint16_t {
  kTypeA = 1,
  kTypePtr = 12,
  kTypeTxt = 16,
  kTypeAaaa = 28,
  kTypeSrv = 33,
  kTypeAny = 255,
  kClassIn = 1,
  kClassFlush = 0x8000,  // in an answer: cache flush (a unique record); in a question: QU
};

// One datagram to send.
struct Out {
  uint8_t data[kMaxOut];
  uint32_t len;
  uint8_t ip[4];
  uint16_t port;
};

struct Stats {
  uint32_t queries;     // queries read (any name)
  uint32_t answered;    // replies sent
  uint32_t unicast;     // ... of them by unicast (QU)
  uint32_t legacy;      // ... of them to a legacy resolver's port
  uint32_t ignored;     // malformed datagrams, responses outside the probe
  uint32_t limited;     // replies not sent: over kRepliesPerSecond
  uint32_t conflicts;   // another host answered for our name
  uint32_t probes;      // probes sent
  uint32_t announcements;
};

class Responder {
 public:
  enum class State : uint8_t { Idle, Probing, Announcing, Claimed };

  // name: the host name to claim ("kilo"), also the service instance's
  // name; port: the laptop service's TCP port; build: in the TXT record.
  void begin(const char* name, uint16_t port, uint32_t build);
  // The interface's address now (0.0.0.0: none, nothing is sent). A change
  // starts the probe (first address) or the announcements (later ones).
  void set_address(const uint8_t ip[4], uint64_t now_us);
  // One datagram that arrived from src:sport. True with *out to send.
  bool receive(const uint8_t* p, uint32_t n, const uint8_t src[4], uint16_t sport, uint64_t now_us, Out* out);
  // A probe or an announcement when due. True with *out to send.
  bool poll(uint64_t now_us, Out* out);

  const char* hostname() const { return name_; }
  State state() const { return state_; }
  bool claimed() const { return state_ == State::Announcing || state_ == State::Claimed; }
  const Stats& stats() const { return stats_; }

 private:
  bool conflicting(const uint8_t* p, uint32_t n, uint32_t off, uint16_t ancount) const;
  void rename();
  bool allow(uint64_t now_us);
  void probe(Out* out);
  void announce(Out* out);

  char base_[kNameMax] = {};
  char name_[kNameMax] = {};
  uint32_t suffix_ = 1;
  uint16_t port_ = 0;
  char txt_[24] = {};
  uint8_t ip_[4] = {};
  State state_ = State::Idle;
  int step_ = 0;          // probes or announcements sent in this round
  uint64_t due_us_ = 0;   // the next probe or announcement
  uint64_t window_us_ = 0;
  uint32_t window_n_ = 0;
  Stats stats_ = {};
};

// ---- the wire format, for the responder and its tests ----------------------------

// The name at off (labels, compression pointers followed) as dotted text in
// out (at most outn bytes, null included). *next: the offset after the name
// where it was read. False if malformed or too long.
bool read_name(const uint8_t* p, uint32_t n, uint32_t off, char* out, uint32_t outn, uint32_t* next);
// ASCII case-insensitive, as DNS names compare.
bool name_eq(const char* a, const char* b);

}  // namespace mdns
}  // namespace kilo

#endif
