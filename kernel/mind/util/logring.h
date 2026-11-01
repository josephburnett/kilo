// logring: the mind's log in RAM, a fixed ring of the last lines. It is a
// Log sink (platform.h): util/log.h writes into it, and it passes every line
// on to the platform's own sink (the screen, stderr). The laptop interface
// (net/host_server.h) reads it for `kilo log --host`: the last lines on
// connect, then each new one. Python counterpart: the daemon's log handler
// that `kilo daemon` tails (kilo/daemon/logs.py).
//
// Lines are numbered from 0 in the order logged (seq); a reader keeps the
// seq of the next line it wants and asks get(seq). A reader that falls more
// than kLines behind finds its lines overwritten and skips to first_seq():
// the gap in seq numbers tells the client what it lost.
#ifndef KILO_MIND_UTIL_LOGRING_H
#define KILO_MIND_UTIL_LOGRING_H
#include <stdint.h>
#include "platform.h"
#include "proto/kilo_protocol.h"
#include "util/log.h"

namespace kilo {

struct LogEntry {
  uint32_t seq;
  uint32_t t_ms;   // since the ring was made (the mind's start)
  LogLevel level;
  uint8_t len;     // text bytes, without the null
  char text[kLogLineMax];
};

static_assert(KILO_HOST_LOG_TEXT_MAX + 1 == kLogLineMax, "LOG_LINE text holds one log line");

class LogRing final : public Log {
 public:
  static constexpr uint32_t kLines = KILO_HOST_LOG_RING_LINES;

  // next: where every line also goes (null: nowhere).
  LogRing(Clock& clock, Log* next = nullptr);

  void write(LogLevel level, const char* line) override;

  uint32_t next_seq() const { return next_seq_; }  // the seq the next line gets: lines logged so far
  uint32_t first_seq() const { return next_seq_ > kLines ? next_seq_ - kLines : 0; }
  // The line with this seq, or null when it is overwritten or not logged yet.
  const LogEntry* get(uint32_t seq) const;

 private:
  Clock& clock_;
  Log* next_;
  uint64_t start_us_;
  uint32_t next_seq_ = 0;
  LogEntry lines_[kLines] = {};
};

}  // namespace kilo

#endif
