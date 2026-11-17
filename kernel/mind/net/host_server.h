// host_server: the laptop interface (docs/fast-boot.md §5, phase P2): one
// TCP listener, the channel protocol/spec.toml [host] defines, served to up
// to KILO_HOST_MAX_CLIENTS connections by a cooperative poll() from the
// mind's main loop (non-blocking, no threads, fixed buffers). Python
// counterpart: the daemon's Unix socket (kilo/daemon/clients.py, ops.py),
// and its client kilo/hostlink.py.
//
// What it serves:
// - commands, one reply each (RESULT, or STATUS for STATUS_REQ): STATUS_REQ,
//   CONTROL, POSTURE, PARAM_GET/PARAM_SET (UNSUPPORTED until P3), TELEM_SUB,
//   LOG_SUB, BRIDGE_OPEN, BRIDGE_CLOSE, REBOOT;
// - TELEM: the link's latest TELEMETRY and counters, at each client's rate;
// - LOG_LINE: the RAM log (util/logring.h), the last lines then live;
// - the raw bridge: BRIDGE_OPEN pauses the link (bridge_open(): the main
//   loop stops calling link.poll()) and forwards the UART both ways;
// - REBOOT (controller only): REST latched on the link, RESULT OK sent,
//   then reboot_requested() is true; the platform restarts (the Circle
//   kernel returns ShutdownReboot once the answer had time to leave).
// - flashing the body (P4, docs/fast-boot.md §6), with a Flasher and a
//   FlashImage attached (attach_flasher()): FLASH_BEGIN, FLASH_DATA and
//   FLASH_COMMIT fill the image and start the flasher, which this server
//   then polls; FLASH_STATUS goes to the client that committed. While the
//   flasher has the UART, link_paused() is true as for the bridge.
// - updating the mind's own kernel on its SD card, with a KernelUpdater
//   attached (attach_updater()): UPDATE_BEGIN, UPDATE_DATA and UPDATE_COMMIT
//   fill and check the image, then this server polls the updater, which
//   writes it a slice per pass; UPDATE_STATUS goes to the client that
//   committed. While it writes, link_paused() is true (the SD card stalls
//   the loop). On DONE: REST latched, the DONE status sent, then
//   reboot_requested(), as for REBOOT. A flash and an update exclude each
//   other (FLASHING, UPDATING).
// - the flight recorder, with a Recorder attached (attach_recorder(),
//   record/recorder.h) and its card Saver (attach_saver(), record/saver.h):
//   WALK_LIST answers WALK_INFO, one run per request (this boot's, then the
//   card's earlier boots'); WALK_FETCH streams a run's span and REC_FETCH
//   any range of blocks (or an earlier boot's log) to its client as
//   WALK_DATA, a few chunks a pass and only into room in that client's
//   send ring (kFetchChunks, one a pass while a run is recorded), then END;
//   REC_STATUS_REQ, METRICS_REQ (the MetricsSource), and for the
//   controller REC_SAVE, REC_DELETE and REC_BENCH (BENCH_RESULT back to
//   it). Card reads go through the saver, a card call a pass when safe.
//
// Who may connect (kernel/README.md "Who may talk to Kilo: keys", the spec's
// [host] comments): every connection starts with a clear hello naming the
// laptop's static public key, which must be in the list set_keys() gives,
// then the Noise KN handshake (net/noise.h), then encrypted records that
// carry the frames below. The responder's three X25519 operations run one
// per pass, for all connections together (poll(): handshake_step()). A
// connection is a client only once a record from it authenticates; until
// then it is served nothing, is not counted (clients(), STATUS.clients) and
// is closed KILO_HOST_HANDSHAKE_TIMEOUT_MS after it opened. kSlots is one
// more than kMaxClients: a new connection always gets a slot, and when that
// makes more than kMaxClients open, the oldest one that is not yet a client
// is closed (the new one itself if every other is a client).
//
// Control: one controller, the first client to send CONTROL take=1; the
// others are observers (STATUS, TELEM, LOG_LINE) and get NOT_CONTROLLER for
// POSTURE, PARAM_SET, REBOOT and the bridge. When the controller gives up
// control, closes, goes out of step, or asked for keepalive and was silent for
// KILO_HOST_KEEPALIVE_TIMEOUT_MS, the bridge closes (if open) and the mind
// latches SET_POSTURE REST (the daemon's end of a hold, kilo/daemon/sessions.py).
//
// Back-pressure: each client has a kTxBytes send ring. A reply
// that does not fit drops the client (it stopped reading); TELEM that does not
// fit is skipped (counted); log lines wait (a client that falls more than the
// ring behind sees a gap in LOG_LINE.seq); bridge RX is read from the UART
// only as far as the controller's ring has room.
#ifndef KILO_MIND_NET_HOST_SERVER_H
#define KILO_MIND_NET_HOST_SERVER_H
#include <stdint.h>
#include "flash/flasher.h"
#include "flash/image.h"
#include "link/serial_link.h"
#include "net/authorized_keys.h"
#include "net/host_frame.h"
#include "net/noise.h"
#include "platform.h"
#include "platform_net.h"
#include "record/recorder.h"
#include "record/saver.h"
#include "update/updater.h"
#include "util/logring.h"

namespace kilo {

class HostServer final : public record::BenchSink {
 public:
  static constexpr uint32_t kMaxClients = KILO_HOST_MAX_CLIENTS;
  static constexpr uint32_t kSlots = kHostSlots;  // the listener's slots: kMaxClients + 1 (see above)
  static constexpr uint32_t kTxBytes = 4096;  // per client: about 25 TELEM, or a log tail of 24 long lines
  static constexpr uint32_t kRecordBytes = 2 + KILO_HOST_RECORD_MAX + noise::kTagLen;  // a whole record on the wire
  // WALK_DATA chunks queued per client per pass: 8 (1.6 KB, what two
  // records seal), 1 while a walk is being recorded (the walk's passes stay
  // as they were but for one 213-byte frame).
  static constexpr uint32_t kFetchChunks = 8;
  static constexpr uint32_t kFetchChunksRecording = 1;

  struct Stats {
    uint32_t accepted;
    uint32_t refused;       // accepted into a slot past kSlots, or with kMaxClients clients (closed at once)
    uint32_t closed;
    uint32_t out_of_step;   // bad length or CRC: closed
    uint32_t handshakes;    // connections that became clients (a record from them authenticated)
    uint32_t hello_refused; // a bad magic, version or unknown key, or no randomness: closed
    uint32_t auth_failed;   // a bad message 1, a record that did not authenticate or a bad record length: closed
    uint32_t handshake_timeouts;  // not a client within KILO_HOST_HANDSHAKE_TIMEOUT_MS: closed
    uint32_t evicted;       // closed while still in the handshake, to free a slot for a newer connection
    uint32_t requests;
    uint32_t telem_sent;
    uint32_t telem_skipped; // a client's ring was full
    uint32_t log_sent;
    uint32_t bridge_rx_bytes;
    uint32_t bridge_tx_bytes;
    uint32_t bridge_tx_short;  // UART bytes from the laptop the Serial did not take
    uint32_t control_lost;     // ends of control that latched REST
    uint32_t flash_status_sent;
    uint32_t flash_bytes_rx;   // FLASH_DATA bytes taken into the image
    uint32_t update_status_sent;
    uint32_t update_bytes_rx;  // UPDATE_DATA bytes taken into the image
    uint32_t walk_bytes_sent;  // WALK_DATA bytes queued
    uint32_t walk_fetches;     // WALK_DATA END sent: whole walks downloaded
  };

  // mind_build: STATUS.mind_build (the git hash, 0 = unknown). The link and
  // serial are the body link's; serial is read and written directly only
  // while the bridge is open.
  HostServer(Listener& listener, SerialLink& link, Serial& serial, Clock& clock, Log& log, LogRing& ring,
             uint32_t mind_build);

  // Accept, read and answer requests, keepalive, bridge, streams, flush.
  // Never blocks; call it every pass of the main loop (a few ms apart).
  void poll();

  // Serve the flash channel with this flasher and image (both outlive the
  // server). Without them FLASH_* are answered UNSUPPORTED. poll() then
  // polls the flasher: do not poll it yourself as well.
  void attach_flasher(Flasher& flasher, FlashImage& image);
  // Serve UPDATE_* with this updater (it outlives the server). Without one
  // they are answered UNSUPPORTED (kilo-mind-host without --card). poll()
  // then polls it.
  void attach_updater(KernelUpdater& updater);
  // Serve WALK_LIST, WALK_FETCH, REC_STATUS_REQ and REC_FETCH from this
  // recorder (it outlives the server). Without one they are answered UNSUPPORTED.
  void attach_recorder(const record::Recorder& rec) { rec_ = &rec; }
  // ... and the card through this saver (REC_SAVE, REC_DELETE, REC_BENCH,
  // earlier boots, saved logs). Without one those are UNSUPPORTED.
  void attach_saver(record::Saver& saver) { saver_ = &saver; }
  // METRICS_REQ's lines.
  void attach_metrics(record::MetricsSource& m) { metrics_ = &m; }
  // record::BenchSink: REC_BENCH's results, to the client that asked.
  void bench_result(const kilo_host_bench_result_t& r) override;
  // The laptops' public keys a connection may name (they outlive the
  // server), and the source of the ephemeral keys. Without them (or with
  // no keys) every hello is refused: fail closed.
  void set_keys(const AuthorizedKey* keys, uint32_t n, Random& random);

  // The bridge is open: the link is paused, do not poll() or request() it.
  bool bridge_open() const { return bridge_; }
  // The bridge is open or the flasher has the UART: do not poll() the link,
  // request(), set goals that must go out, or stream FEET. The main loop
  // tests this instead of bridge_open().
  bool link_paused() const { return bridge_ || (flasher_ != nullptr && flasher_->owns_uart()) || updating(); }
  // A flash is running (from FLASH_COMMIT until DONE or FAILED).
  bool flashing() const { return flasher_ != nullptr && flasher_->busy(); }
  // A kernel update is being checked and written (from UPDATE_COMMIT until
  // DONE or FAILED): the SD card is in use and the loop stalls on it.
  bool updating() const { return updater_ != nullptr && updater_->busy(); }
  // A controller sent REBOOT and has its OK, or a kernel update is DONE:
  // the platform should restart (after a few passes, so the answer
  // leaves). Stays set.
  bool reboot_requested() const { return reboot_; }
  // The controlling client's slot, -1 for none.
  int32_t controller() const { return controller_; }
  // Connections that are clients (a record from them authenticated), and all open ones.
  uint32_t clients() const;
  uint32_t connections() const;
  const Stats& stats() const { return stats_; }

 private:
  static constexpr uint32_t kReadChunk = 256;
  static constexpr uint32_t kReadsPerPoll = 4;     // bounded work per client per pass (1 KB read)
  static constexpr uint32_t kRecordsPerFlush = 2;  // ... and at most two records sealed (2 KB)

  // A connection's place in the handshake (the spec's [host] comments).
  enum class Phase : uint8_t {
    Hello,    // reading the clear hello and Noise message 1
    Keygen,   // waiting for a pass to make the ephemeral key (X25519)
    Ee,       // ... for ee
    Se,       // ... for se; then the answer and message 2 go out
    Proving,  // waiting for the client's first record
    Open,     // a client: its records carry frames
  };

  struct Client {
    bool open;
    Phase phase;
    bool closing;              // a clear refusal is queued: close once it is written
    const char* closing_why;
    Conn* conn;
    uint64_t opened_us;        // accept(): the handshake deadline counts from here
    int32_t key;               // the hello's key: its index in keys_
    uint64_t hs_start_us;      // message 1 complete
    uint32_t hs_us;            // message 1 complete to message 2 sealed: the mind's handshake
    uint32_t hs_slowest_us;    // the slowest of the three X25519 steps
    noise::HandshakeKN hs;
    noise::CipherState send, recv;
    uint8_t in[kRecordBytes];  // the hello and message 1, then the record being read
    uint32_t in_len;
    uint8_t out[kRecordBytes]; // the clear answer and message 2, then the record being written
    uint32_t out_head;
    uint32_t out_len;
    HostRx rx;
    uint8_t tx[kTxBytes];      // frames waiting to be sealed into a record
    uint32_t tx_head;  // first byte to send
    uint32_t tx_len;   // bytes queued
    uint8_t out_seq;   // seq of this client's stream frames
    uint64_t last_rx_us;
    bool keepalive;
    uint32_t telem_period_us;  // 0 = off
    uint64_t telem_next_us;
    bool log_on;
    uint32_t log_seq;          // the next history line to send
    uint32_t log_status;       // the next status line to send (LogRing::status_get)
    uint8_t log_min;
    bool fetch_on;             // WALK_FETCH / REC_FETCH: sending fetch_id's blocks
    bool fetch_card;           // the card part (the saver reads), then RAM (this boot) or the end
    bool fetch_ram;            // a RAM part follows the card's
    uint32_t fetch_id;         // WALK_DATA.id: the run, 0 for a range
    uint32_t fetch_off;        // bytes sent
    uint32_t fetch_total;      // 0: not known (the card)
    uint32_t fetch_seq, fetch_last, fetch_boff;  // RAM: the block, the last one, the offset in it
    uint32_t fetch_last_used;  // the last block's length when the fetch began (it may be open)
    uint64_t fetch_from_us, fetch_to_us;  // the range (RAM after the card)
  };

  void accept(uint64_t now);
  void evict();
  void read_client(uint32_t i, uint64_t now);
  // The secure layer (net/host_secure.cpp).
  void read_hello(uint32_t i, uint64_t now);
  void hello_complete(uint32_t i, uint64_t now);
  void refuse(uint32_t i, uint8_t status, const char* why);
  void handshake_step(uint64_t now);
  void read_records(uint32_t i, uint64_t now);
  void record_in(uint32_t i, uint64_t now);
  void expire(uint64_t now);
  bool seal(uint32_t i);
  void handle(uint32_t i, const HostFrame& f, uint64_t now);
  void drop(uint32_t i, const char* why);
  void lose_control(const char* why);
  void close_bridge(const char* why);
  void pump_bridge();
  void streams(uint64_t now);
  void reboot(uint32_t i, uint8_t seq);
  void flash_begin(uint32_t i, uint8_t seq, const uint8_t* p);
  void flash_commit(uint32_t i, uint8_t seq);
  void drop_image(const char* why);
  void flash_status();
  void update_begin(uint32_t i, uint8_t seq, const uint8_t* p);
  void update_commit(uint32_t i, uint8_t seq);
  void drop_update(const char* why);
  void update_status();
  void update_reboot();
  // The flight recorder (net/host_records.cpp).
  void walk_list(uint32_t i, uint8_t seq, const uint8_t* p, uint64_t now);
  void walk_fetch(uint32_t i, uint8_t seq, const uint8_t* p);
  void rec_status(uint32_t i, uint8_t seq, uint64_t now);
  void rec_fetch(uint32_t i, uint8_t seq, const uint8_t* p, uint64_t now);
  void metrics(uint32_t i, uint8_t seq, const uint8_t* p);
  void rec_bench(uint32_t i, uint8_t seq, const uint8_t* p);
  void rec_save(uint32_t i, uint8_t seq, const uint8_t* p);
  void rec_delete(uint32_t i, uint8_t seq, const uint8_t* p);
  // Plan a fetch of [from_us, to_us] of boot (0: this one): false if nothing is there.
  bool plan_fetch(uint32_t i, uint32_t id, uint32_t boot, uint64_t from_us, uint64_t to_us);
  bool plan_ram(Client& c, uint64_t from_us, uint64_t to_us, uint32_t min_seq);
  void stop_fetch(uint32_t i);
  void pump_fetch(uint32_t i);
  void pump_log(uint32_t i);
  bool body_limp() const;
  // Write out what is sealed, sealing at most a few records per call (all:
  // until the ring is empty or the connection takes no more, for an OK that
  // must leave now).
  void flush(uint32_t i, bool all = false);

  bool enqueue(uint32_t i, uint8_t type, uint8_t seq, const void* payload, uint32_t len);
  void reply(uint32_t i, uint8_t seq, const void* payload, uint32_t len, uint8_t type);
  void result(uint32_t i, uint8_t seq, uint8_t request, uint8_t code, const char* text = "");
  uint32_t room(uint32_t i) const { return kTxBytes - clients_[i].tx_len; }

  uint8_t state_flags(uint64_t now) const;
  uint16_t telemetry_age_ms(uint64_t now) const;
  void fill_link_stats(kilo_host_link_stats_t& out) const;

  Listener& listener_;
  SerialLink& link_;
  Serial& serial_;
  Clock& clock_;
  Log& log_;
  LogRing& ring_;
  uint32_t mind_build_;
  uint64_t start_us_;

  Client clients_[kSlots] = {};
  const AuthorizedKey* keys_ = nullptr;
  uint32_t n_keys_ = 0;
  Random* random_ = nullptr;
  uint32_t hs_next_ = 0;  // where handshake_step() looks first (round robin)
  int32_t controller_ = -1;
  bool bridge_ = false;
  bool reboot_ = false;
  Stats stats_ = {};

  // The flash channel (attach_flasher()).
  Flasher* flasher_ = nullptr;
  FlashImage* image_ = nullptr;
  int32_t flash_client_ = -1;    // the client receiving into the image, then the one FLASH_STATUS goes to
  bool receiving_ = false;       // FLASH_BEGIN accepted, no COMMIT yet
  bool persist_ = false;         // FLASH_BEGIN.flags PERSIST
  uint32_t flash_sent_ = 0;      // the flasher's changes() last sent

  // Kernel updates (attach_updater()).
  KernelUpdater* updater_ = nullptr;
  int32_t update_client_ = -1;   // the client receiving, then the one UPDATE_STATUS goes to
  uint32_t update_sent_ = 0;     // the updater's changes() last sent

  const record::Recorder* rec_ = nullptr;  // attach_recorder()
  record::Saver* saver_ = nullptr;         // attach_saver()
  record::MetricsSource* metrics_ = nullptr;
  int32_t bench_client_ = -1;
};

}  // namespace kilo

#endif
