#include "sound/player.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"
#include "util/str.h"

namespace kilo {
namespace sound {

// ---- names ----------------------------------------------------------------------------------

void family(const char* name, char* out) {
  uint32_t n = 0;
  while (name[n] != '\0' && n < kNameMax) n++;
  while (n > 0 && name[n - 1] >= '0' && name[n - 1] <= '9') n--;
  mem_copy(out, name, n);
  out[n] = '\0';
}

bool is_variant(const char* name) {
  uint32_t n = 0;
  while (name[n] != '\0') n++;
  return n > 0 && name[n - 1] >= '0' && name[n - 1] <= '9';
}

bool valid_name(const char* name) {
  uint32_t n = 0;
  for (; name[n] != '\0'; n++) {
    char c = name[n];
    if (n >= kNameMax || !((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
  }
  return n > 0;
}

uint32_t gain_q16(uint8_t volume) {
  uint32_t v = volume > 100 ? 100 : volume;
  return v * v * kUnity / 10000;
}

// ---- the player -----------------------------------------------------------------------------

SoundPlayer::SoundPlayer(Clock& clock, Log& log, uint64_t seed) : clock_(clock), log_(log), rng_(seed ^ 0x50D5ull) {}

void SoundPlayer::set_sounds(const Sound* sounds, uint32_t n) {
  sounds_ = n != 0 ? sounds : nullptr;
  n_ = sounds_ != nullptr ? n : 0;
  pending_ = nullptr;
  last_ = nullptr;
}

void SoundPlayer::set_enabled(bool on) {
  if (on == on_) return;
  on_ = on;
  if (!on) {
    pending_ = nullptr;
    if (out_ != nullptr && out_->playing()) out_->stop();
  }
  logf(log_, LogLevel::Info, "sound: %s", on ? "on" : "off");
}

void SoundPlayer::set_volume(uint8_t v) {
  volume_ = v > 100 ? 100 : v;
}

const Sound* SoundPlayer::find(const char* name) const {
  for (uint32_t i = 0; i < n_; i++)
    if (str_eq(sounds_[i].name, name)) return &sounds_[i];
  return nullptr;
}

uint32_t SoundPlayer::total_ms() const {
  uint64_t ms = 0;
  for (uint32_t i = 0; i < n_; i++)
    if (sounds_[i].rate != 0) ms += (uint64_t)sounds_[i].samples * 1000u / sounds_[i].rate;
  return (uint32_t)ms;
}

// A variant by its name; a family: one of its members at random, not the last one played if there are more.
const Sound* SoundPlayer::pick(const char* name) {
  if (is_variant(name)) return find(name);
  const Sound* members[32];
  uint32_t m = 0;
  char fam[kNameMax + 1];
  for (uint32_t i = 0; i < n_ && m < 32; i++) {
    family(sounds_[i].name, fam);
    if (str_eq(fam, name)) members[m++] = &sounds_[i];
  }
  if (m == 0) return nullptr;
  if (m == 1) return members[0];
  uint32_t k = (uint32_t)(rng_.random() * m);
  if (k >= m) k = m - 1;
  if (members[k] == last_) k = (k + 1 + (uint32_t)(rng_.random() * (m - 1))) % m;
  return members[k];
}

uint8_t SoundPlayer::request(const char* name, const char* why) {
  requests_++;
  if (n_ == 0 || out_ == nullptr) {
    none_++;
    return KILO_SOUND_RESULT_NO_SOUNDS;
  }
  if (!on_) {
    off_++;
    return KILO_SOUND_RESULT_OFF;
  }
  if (asleep_) {
    asleep_n_++;
    return KILO_SOUND_RESULT_ASLEEP;
  }
  const Sound* s = pick(name);
  if (s == nullptr) {
    unknown_++;
    logf(log_, LogLevel::Info, "sound: no sound \"%s\" (%s)", name, why);
    return KILO_SOUND_RESULT_UNKNOWN;
  }
  if (pending_ != nullptr) replaced_++;  // newest wins
  pending_ = s;
  pending_at_ = clock_.now_us();
  format(asked_, sizeof asked_, "%s", name);
  format(why_, sizeof why_, "%s", why);
  if (playing()) return KILO_SOUND_RESULT_QUEUED;
  poll();  // nothing plays: at once if the gap has passed
  return KILO_SOUND_RESULT_QUEUED;
}

bool SoundPlayer::playing() { return out_ != nullptr && out_->playing(); }

bool SoundPlayer::busy() { return pending_ != nullptr || playing(); }

void SoundPlayer::suspend(bool asleep) {
  if (asleep == asleep_) return;
  asleep_ = asleep;
  if (asleep) pending_ = nullptr;
  if (out_ != nullptr) out_->suspend(asleep);
}

void SoundPlayer::begin() {
  const Sound* s = pending_;
  pending_ = nullptr;
  if (!out_->play(*s, gain_q16(volume_))) {
    failed_++;
    logf(log_, LogLevel::Warn, "sound: %s: the output did not start", s->name);
    return;
  }
  played_++;
  last_ = s;
  was_playing_ = true;
  ever_played_ = true;
  char line[112];
  format(line, sizeof line, "sound play name=%s asked=%s why=%s volume=%u ms=%u", s->name, asked_, why_,
         (unsigned)volume_, (unsigned)((uint64_t)s->samples * 1000u / s->rate));
  logf(log_, LogLevel::Info, "%s", line);
  if (host_ != nullptr) host_->mark(line);
}

void SoundPlayer::poll() {
  if (out_ == nullptr) return;
  out_->poll();
  uint64_t now = clock_.now_us();
  bool sounding = out_->playing();
  if (was_playing_ && !sounding) {
    was_playing_ = false;
    ended_at_ = now;
  }
  if (pending_ == nullptr) return;
  if (now - pending_at_ > kStaleUs) {
    stale_++;
    logf(log_, LogLevel::Info, "sound: %s (%s) dropped: waited over %u ms", pending_->name, why_,
         (unsigned)(kStaleUs / 1000u));
    pending_ = nullptr;
    return;
  }
  if (sounding || (ever_played_ && now - ended_at_ < kGapUs)) return;
  begin();
}

void SoundPlayer::fill(kilo_host_sound_status_t& s, uint8_t result) {
  mem_zero(&s, sizeof s);
  OutputStats o;
  if (out_ != nullptr) out_->stats(o);
  s.result = result;
  s.enabled = on_ ? 1 : 0;
  s.volume = volume_;
  s.sounds = (uint8_t)(n_ > 255 ? 255 : n_);
  s.state = out_ != nullptr ? o.state : (uint8_t)KILO_SOUND_STATE_NONE;
  s.played = played_;
  s.requests = requests_;
  s.unknown = unknown_;
  s.dropped = dropped();
  s.late = o.late;
  s.callback_max_us = o.callback_max_us;
}

void SoundPlayer::metrics_line(char* out, uint32_t n) {
  OutputStats o;
  if (out_ != nullptr) out_->stats(o);
  static const char* const kState[] = {"none", "idle", "playing", "asleep", "failed"};
  uint8_t st = out_ != nullptr ? o.state : (uint8_t)KILO_SOUND_STATE_NONE;
  format(out, n,
         "sound: on=%u volume=%u sounds=%u state=%s played=%u requests=%u unknown=%u dropped=%u replaced=%u stale=%u "
         "last=%s starts=%u late=%u interval_max_us=%u callback_max_us=%u callback_mean_us=%u",
         on_ ? 1u : 0u, (unsigned)volume_, (unsigned)n_, st < 5 ? kState[st] : "?", (unsigned)played_,
         (unsigned)requests_, (unsigned)unknown_, (unsigned)dropped(), (unsigned)replaced_, (unsigned)stale_,
         last_ != nullptr ? last_->name : "-", (unsigned)o.starts, (unsigned)o.late, (unsigned)o.interval_max_us,
         (unsigned)o.callback_max_us, (unsigned)o.callback_mean_us);
}

// ---- the behaviour's own sounds -------------------------------------------------------------

Chatter::Chatter(uint64_t seed) : rng_(seed ^ 0xC4A77Eull) {}

const char* Chatter::poll(double now, Mode mode, bool quiet) {
  if (mode != mode_) {
    mode_ = mode;
    next_at_ = now + rng_.uniform(kFirstMinS, kFirstMaxS);
    return nullptr;
  }
  if (mode == Mode::None || now < next_at_) return nullptr;
  bool sitting = mode == Mode::Sitting;
  next_at_ = now + (sitting ? rng_.uniform(kSitMinS, kSitMaxS) : rng_.uniform(kStandMinS, kStandMaxS));
  if (quiet) return nullptr;
  double r = rng_.random();
  const char* what = nullptr;
  if (sitting) what = r < 0.6 ? "purr" : r < 0.75 ? "trill" : r < 0.85 ? "meow" : nullptr;
  else what = r < 0.3 ? "trill" : r < 0.5 ? "meow" : nullptr;
  if (what != nullptr) said_++;
  return what;
}

}  // namespace sound
}  // namespace kilo
