#include "signal/signal.h"

namespace kilo {
namespace signals {

const char* signal_name(Signal s) {
  switch (s) {
    case Signal::Ready: return "ready";
    case Signal::Done: return "done";
    case Signal::Refused: return "refused";
    case Signal::Failed: return "failed";
    case Signal::GaveUp: return "gave_up";
    case Signal::Saved: return "saved";
    case Signal::Flashing: return "flashing";
    case Signal::GuardRed: return "guard_red";
  }
  return "?";
}

const char* signal_sound(Signal s) {
  switch (s) {
    case Signal::Ready: return "mew";
    case Signal::Done: return "trill";
    case Signal::Refused:
    case Signal::Failed:
    case Signal::GuardRed: return "hiss";
    case Signal::GaveUp: return "meow";
    case Signal::Saved:
    case Signal::Flashing: return nullptr;
  }
  return nullptr;
}

bool same(const Look& a, const Look& b) { return a.mask == b.mask && a.r == b.r && a.g == b.g && a.b == b.b; }

const char* look_name(const Look& l) {
  struct Named {
    const Look* look;
    const char* name;
  };
  static const Named kNames[] = {{&kOff, "off"},       {&kGreen, "green"},   {&kBlue, "blue"},
                                 {&kRed, "red"},       {&kOrange, "orange"}, {&kWhite, "white"},
                                 {&kPurple, "purple"}, {&kDimGreen, "dim green"}};
  for (const Named& n : kNames)
    if (same(*n.look, l)) return n.name;
  return "other";
}

uint64_t EyeLanguage::length(Signal s) {
  if (s == Signal::Saved) return kSavedOnUs + kSavedOffUs;
  return (uint64_t)kFlashes * (kFlashOnUs + kFlashOffUs);
}

void EyeLanguage::show(Signal s, uint64_t now) {
  if (s == Signal::GuardRed) return;
  if (s == Signal::Flashing) {
    flashing_ = true;
    return;
  }
  flashing_ = false;  // the flash's outcome (or anything after it) ends the steady white
  if (s == Signal::Ready) {
    ready_ = true;
    return;
  }
  Shot shot = {true, false, s, now, 0};
  if (!playing_.used) {
    playing_ = shot;
    return;
  }
  if (waiting_.used) dropped_++;  // the newer one takes its place
  waiting_ = shot;
}

Look EyeLanguage::frame(Signal s, uint64_t t) {
  if (s == Signal::Saved) return t < kSavedOnUs ? kWhite : kOff;
  uint64_t in = t % (kFlashOnUs + kFlashOffUs);
  if (in >= kFlashOnUs) return kOff;
  switch (s) {
    case Signal::Refused: return kPurple;
    case Signal::Failed:
    case Signal::GaveUp: return kRed;
    default: return kGreen;
  }
}

static Look guard_look(guard::Eyes e) {
  switch (e) {
    case guard::Eyes::Blue: return kBlue;
    case guard::Eyes::Red: return kRed;
    case guard::Eyes::Orange: return kOrange;
    case guard::Eyes::Green: break;
  }
  return kGreen;
}

Look EyeLanguage::look(const State& st, uint64_t now) {
  if (playing_.used && playing_.started && now - playing_.start >= length(playing_.s)) playing_.used = false;
  if (!playing_.used && waiting_.used) {
    playing_ = waiting_;
    waiting_.used = false;
  }
  bool guard_says = ready_ && (st.guard == guard::Eyes::Blue || st.guard == guard::Eyes::Red);
  if (playing_.used && !playing_.started) {
    if (!guard_says) {
      playing_.started = true;
      playing_.start = now;
      shown_++;
    } else if (now - playing_.asked > kWaitUs) {
      playing_.used = false;
      dropped_++;
      if (waiting_.used && now - waiting_.asked > kWaitUs) {
        waiting_.used = false;
        dropped_++;
      }
    }
  }
  if (flashing_) return kWhite;
  if (guard_says) return guard_look(st.guard);
  if (playing_.used && playing_.started) return frame(playing_.s, now - playing_.start);
  if (!ready_) return kOff;
  if (st.asleep) return now % kSleepEveryUs < kSleepBlinkUs ? kDimGreen : kOff;
  if (st.busy) return (now / kBusyHalfUs) % 2 == 0 ? kWhite : kOff;
  return guard_look(st.guard);
}

void EyeDriver::poll(const Look& l, bool driven, uint64_t now) {
  if (!driven) {
    has_ = false;
    return;
  }
  const uint8_t p[KILO_SET_EYES_LEN] = {l.mask, l.r, l.g, l.b};
  // the link's goal is this look (a laptop may have set another, a STALE refusal dropped it)
  const Goal* g = link_.goal(KILO_MSG_SET_EYES);
  bool latched = g != nullptr && g->len == sizeof p;
  for (uint32_t i = 0; latched && i < sizeof p; i++) latched = g->payload[i] == p[i];
  if (!has_ || !same(l, sent_) || !latched) {
    if (!link_.set_goal(KILO_MSG_SET_EYES, p, sizeof p)) return;
    has_ = true;
    sent_ = l;
    last_us_ = now;
    repeats_ = 0;
    changes_++;
    return;
  }
  if (repeats_ >= 2 || now - last_us_ < kRepeatUs) return;
  if (link_.set_goal(KILO_MSG_SET_EYES, p, sizeof p)) {  // the same again: marked to go out now
    repeats_++;
    last_us_ = now;
  }
}

}  // namespace signals
}  // namespace kilo
