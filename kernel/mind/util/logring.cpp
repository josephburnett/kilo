#include "util/logring.h"
#include "util/mem.h"

namespace kilo {

LogRing::LogRing(Clock& clock, Log* next) : clock_(clock), next_(next), start_us_(clock.now_us()), lines_(small_) {}

void LogRing::attach(LogEntry* lines, uint32_t n) {
  if (lines == nullptr || n < kSmallLines) return;
  floor_ = first_seq();
  for (uint32_t s = floor_; s < next_seq_; s++) lines[s % n] = lines_[s % cap_];
  lines_ = lines;
  cap_ = n;
}

void LogRing::fill(LogEntry& e, uint32_t seq, LogLevel level, const char* line) {
  e.seq = seq;
  e.t_ms = (uint32_t)((clock_.now_us() - start_us_) / 1000u);
  e.level = level;
  uint32_t n = 0;
  while (line[n] != '\0' && n < kLogLineMax - 1) {
    e.text[n] = line[n];
    n++;
  }
  e.text[n] = '\0';
  e.len = (uint8_t)n;
}

void LogRing::write(LogLevel level, const char* line) {
  uint32_t seq = next_seq_++;
  fill(lines_[seq % cap_], seq, level, line);
  if (tap_ != nullptr) tap_->write(level, line);
  if (next_ != nullptr) next_->write(level, line);
}

void LogRing::Status::write(LogLevel level, const char* line) {
  LogRing& r = ring_;
  if (level >= LogLevel::Warn) {
    r.write(level, line);
    return;
  }
  if (r.watched_) {
    uint32_t n = r.status_next_++;
    r.fill(r.status_lines_[n % kStatusLines], kStatusBit | n, level, line);
  }
  if (r.tap_ != nullptr) r.tap_->write(level, line);
  if (r.next_ != nullptr) r.next_->write(level, line);
}

const LogEntry* LogRing::get(uint32_t seq) const {
  if (seq >= next_seq_ || seq < first_seq()) return nullptr;
  return &lines_[seq % cap_];
}

const LogEntry* LogRing::status_get(uint32_t n) const {
  if (n >= status_next_ || n < status_first()) return nullptr;
  return &status_lines_[n % kStatusLines];
}

}  // namespace kilo
