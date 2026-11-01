#include "util/logring.h"

namespace kilo {

LogRing::LogRing(Clock& clock, Log* next) : clock_(clock), next_(next), start_us_(clock.now_us()) {}

void LogRing::write(LogLevel level, const char* line) {
  LogEntry& e = lines_[next_seq_ % kLines];
  e.seq = next_seq_++;
  e.t_ms = (uint32_t)((clock_.now_us() - start_us_) / 1000u);
  e.level = level;
  uint32_t n = 0;
  while (line[n] != '\0' && n < kLogLineMax - 1) {
    e.text[n] = line[n];
    n++;
  }
  e.text[n] = '\0';
  e.len = (uint8_t)n;
  if (next_ != nullptr) next_->write(level, line);
}

const LogEntry* LogRing::get(uint32_t seq) const {
  if (seq >= next_seq_ || seq < first_seq()) return nullptr;
  return &lines_[seq % kLines];
}

}  // namespace kilo
