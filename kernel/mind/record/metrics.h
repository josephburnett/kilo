// metrics: the session's counters since boot, kept by the mind (mind.h
// fills them; the recorder counts runs), served as METRICS lines (`kilo
// metrics --host`, `kilo status --host`) and saved with the log on the card.
// Plain counters: incrementing one is all the loop pays.
#ifndef KILO_MIND_RECORD_METRICS_H
#define KILO_MIND_RECORD_METRICS_H
#include <stdint.h>

namespace kilo {
namespace record {

// A few counts by a short name (an end reason, a refusal): the first
// kSlots names get a slot each, the rest count as "other".
class NamedCounts {
 public:
  static constexpr uint32_t kSlots = 10;
  static constexpr uint32_t kNameMax = 24;
  void add(const char* name);
  uint32_t n() const { return n_; }
  const char* name(uint32_t i) const { return names_[i]; }
  uint32_t count(uint32_t i) const { return counts_[i]; }
  uint32_t other() const { return other_; }
  uint32_t total() const { return total_; }

 private:
  char names_[kSlots][kNameMax] = {};
  uint32_t counts_[kSlots] = {};
  uint32_t n_ = 0, other_ = 0, total_ = 0;
};

struct Metrics {
  // remote keys (first presses; a held key's repeats apart)
  uint32_t keys = 0, keys_acted = 0, keys_ignored = 0, keys_no_action = 0, keys_laptop = 0, key_repeats = 0;
  uint32_t key_counts[32] = {};  // by IR code's key index (remote/keys.h), first presses
  // runs
  uint32_t walks = 0, walks_ok = 0, walk_holds = 0, walk_catches = 0;
  NamedCounts walk_ends;
  uint32_t tricks = 0;
  NamedCounts trick_ends;   // DONE, STOPPED, PREEMPTED, FELL, FAILED, REFUSED
  NamedCounts trick_names;  // started, by trick
  NamedCounts refusals;     // trick refusals by why, goals the body refused
  // the body
  uint32_t falls = 0, lying = 0, watchdogs = 0, low_battery = 0, events = 0, body_reboots = 0;
  uint32_t battery_mv = 0, battery_min_mv = 0;  // 0: no reading yet
  // the platform's loop (the Circle kernel sets them each second)
  uint32_t loop_max_us = 0, loop_period_max_us = 0;
};

}  // namespace record
}  // namespace kilo

#endif
