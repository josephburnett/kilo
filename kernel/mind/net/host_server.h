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
#include "net/host_frame.h"
#include "platform.h"
#include "platform_net.h"
#include "util/logring.h"

namespace kilo {

class HostServer {
 public:
  static constexpr uint32_t kMaxClients = KILO_HOST_MAX_CLIENTS;
  static constexpr uint32_t kTxBytes = 4096;  // per client: about 25 TELEM, or a log tail of 24 long lines

  struct Stats {
    uint32_t accepted;
    uint32_t refused;       // accepted into a slot past kMaxClients (closed at once)
    uint32_t closed;
    uint32_t out_of_step;   // bad length or CRC: closed
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

  // The bridge is open: the link is paused, do not poll() or request() it.
  bool bridge_open() const { return bridge_; }
  // The bridge is open or the flasher has the UART: do not poll() the link,
  // request(), set goals that must go out, or stream FEET. The main loop
  // tests this instead of bridge_open().
  bool link_paused() const { return bridge_ || (flasher_ != nullptr && flasher_->owns_uart()); }
  // A flash is running (from FLASH_COMMIT until DONE or FAILED).
  bool flashing() const { return flasher_ != nullptr && flasher_->busy(); }
  // A controller sent REBOOT and has its OK: the platform should restart
  // (after a few passes, so the answer leaves). Stays set.
  bool reboot_requested() const { return reboot_; }
  // The controlling client's slot, -1 for none.
  int32_t controller() const { return controller_; }
  uint32_t clients() const;
  const Stats& stats() const { return stats_; }

 private:
  struct Client {
    bool open;
    Conn* conn;
    HostRx rx;
    uint8_t tx[kTxBytes];
    uint32_t tx_head;  // first byte to send
    uint32_t tx_len;   // bytes queued
    uint8_t out_seq;   // seq of this client's stream frames
    uint64_t last_rx_us;
    bool keepalive;
    uint32_t telem_period_us;  // 0 = off
    uint64_t telem_next_us;
    bool log_on;
    uint32_t log_seq;          // the next line to send
    uint8_t log_min;
  };

  void accept(uint64_t now);
  void read_client(uint32_t i, uint64_t now);
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
  void flush(uint32_t i);

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

  Client clients_[kMaxClients] = {};
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
};

}  // namespace kilo

#endif
