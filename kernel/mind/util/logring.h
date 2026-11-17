// logring: the mind's log in RAM. It is a Log sink (platform.h): util/log.h
// writes into it, and it passes every line on to the platform's own sink
// (the screen, stderr) and to a tap (the flight recorder, record/recorder.h,
// which keeps every line with the rest of the session). The laptop interface
// (net/host_server.h) reads it for `kilo log --host`: the history kept, then
// each new line. Python counterpart: the daemon's log handler that `kilo
// daemon` tails (kilo/daemon/logs.py).
//
// Two kinds of line:
// - history: what happened (remote keys acted on, walks, tricks, refusals,
//   warnings, errors, the network coming up). Kept in a ring of capacity()
//   lines: kLines (4096, a whole play session and more) once the platform
//   attach()es the storage, kSmallLines until then (the tests). Numbered
//   from 0 in the order logged (seq); a reader keeps the seq of the next
//   line it wants and asks get(seq). A reader that falls more than the ring
//   behind finds its lines overwritten and skips to first_seq(): the gap in
//   seq numbers tells the client what it lost, and first_seq() is how many
//   lines are gone for good.
// - status (status(): the once-a-second link, loop and body counters,
//   percepts, a held IR key's repeats): routine, many a second. Never
//   history: with a laptop reading the log (set_watched(), the HostServer)
//   they go into a small ring of their own (kStatusLines), numbered apart
//   with kStatusBit set, sent live; without one only to the platform's sink.
//   Warnings and errors written to status() are history.
// The flight recorder gets both kinds always.
#ifndef KILO_MIND_UTIL_LOGRING_H
#define KILO_MIND_UTIL_LOGRING_H
#include <stdint.h>
#include "platform.h"
#include "proto/kilo_protocol.h"
#include "util/log.h"

namespace kilo {

struct LogEntry {
  uint32_t seq;    // history: 0..; status: kStatusBit | 0..
  uint32_t t_ms;   // since the ring was made (the mind's start)
  LogLevel level;
  uint8_t len;     // text bytes, without the null
  char text[kLogLineMax];
};

static_assert(KILO_HOST_LOG_TEXT_MAX + 1 == kLogLineMax, "LOG_LINE text holds one log line");

class LogRing final : public Log {
 public:
  static constexpr uint32_t kLines = KILO_HOST_LOG_RING_LINES;  // the platforms' history (attach())
  static constexpr uint32_t kSmallLines = 64;                   // until then
  static constexpr uint32_t kStatusLines = 32;
  static constexpr uint32_t kStatusBit = 0x80000000u;

  // next: where every line also goes (null: nowhere).
  LogRing(Clock& clock, Log* next = nullptr);

  // The history's storage: n lines (kLines), allocated once by the
  // platform; the lines so far are copied in. Before the Mind starts.
  void attach(LogEntry* lines, uint32_t n);
  // Every line, history and status, also goes here (the flight recorder).
  void set_tap(Log* tap) { tap_ = tap; }

  void write(LogLevel level, const char* line) override;

  // The sink for status lines. Warnings and errors are kept as write() keeps them.
  Log& status() { return status_; }
  void set_watched(bool watched) { watched_ = watched; }
  bool watched() const { return watched_; }

  uint32_t capacity() const { return cap_; }
  uint32_t next_seq() const { return next_seq_; }  // the seq the next history line gets: lines logged so far
  uint32_t first_seq() const {
    uint32_t f = next_seq_ > cap_ ? next_seq_ - cap_ : 0;
    return f > floor_ ? f : floor_;
  }
  // The history line with this seq, or null when it is overwritten or not logged yet.
  const LogEntry* get(uint32_t seq) const;
  // Status lines: numbered 0.. without kStatusBit here.
  uint32_t status_next() const { return status_next_; }
  uint32_t status_first() const { return status_next_ > kStatusLines ? status_next_ - kStatusLines : 0; }
  const LogEntry* status_get(uint32_t n) const;

 private:
  struct Status final : Log {
    explicit Status(LogRing& ring) : ring_(ring) {}
    void write(LogLevel level, const char* line) override;
    LogRing& ring_;
  };
  void fill(LogEntry& e, uint32_t seq, LogLevel level, const char* line);

  Status status_{*this};
  bool watched_ = false;
  Clock& clock_;
  Log* next_;
  Log* tap_ = nullptr;
  uint64_t start_us_;
  uint32_t next_seq_ = 0;
  LogEntry* lines_;
  uint32_t cap_ = kSmallLines;
  uint32_t floor_ = 0;  // lines below were lost before attach()
  LogEntry small_[kSmallLines] = {};
  uint32_t status_next_ = 0;
  LogEntry status_lines_[kStatusLines] = {};
};

}  // namespace kilo

#endif
