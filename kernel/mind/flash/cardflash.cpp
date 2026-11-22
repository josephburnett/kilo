#include "flash/cardflash.h"
#include <stdarg.h>
#include "link/names.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"
#include "util/str.h"

namespace kilo {

const char* card_outcome_name(CardOutcome o) {
  switch (o) {
    case CardOutcome::Flashed: return "flashed";
    case CardOutcome::Failed: return "failed";
    case CardOutcome::Refused: return "refused";
    case CardOutcome::Same: return "same";
    case CardOutcome::None: break;
  }
  return "none";
}

// ---- body-result.txt ---------------------------------------------------------------------

namespace {

bool same_image(const Manifest& a, const Manifest& b) {
  return a.build == b.build && a.size == b.size && a.crc32 == b.crc32;
}

// The line [p, end) is "image: build XXXXXXXX size N crc32 XXXXXXXX" or "image: none".
bool parse_image(const char* p, const char* end, Manifest& m) {
  m = {};
  if (skip_lit(p, end, "none") != nullptr) return true;
  if ((p = skip_lit(p, end, "build ")) == nullptr || (p = parse_hex32(p, end, &m.build)) == nullptr) return false;
  if ((p = skip_lit(p, end, " size ")) == nullptr || (p = parse_dec_u32(p, end, &m.size)) == nullptr) return false;
  if ((p = skip_lit(p, end, " crc32 ")) == nullptr || parse_hex32(p, end, &m.crc32) == nullptr) return false;
  return true;
}

}  // namespace

bool parse_card_record(const char* text, uint32_t len, CardRecord& out) {
  const char* p = text;
  const char* end = text + len;
  bool magic = false, have_outcome = false, have_image = false;
  CardRecord r = {CardOutcome::None, {}};
  while (p < end) {
    const char* eol = p;
    while (eol < end && *eol != '\n') eol++;
    const char* line_end = eol > p && eol[-1] == '\r' ? eol - 1 : eol;
    const char* v;
    if (!magic) {
      if (skip_lit(p, line_end, kResultMagic) != line_end) return false;
      magic = true;
    } else if ((v = skip_lit(p, line_end, "outcome: ")) != nullptr) {
      static const CardOutcome kAll[] = {CardOutcome::Flashed, CardOutcome::Failed, CardOutcome::Refused,
                                         CardOutcome::Same};
      for (CardOutcome o : kAll) {
        const char* q = skip_lit(v, line_end, card_outcome_name(o));
        if (q == line_end) {
          r.outcome = o;
          have_outcome = true;
        }
      }
    } else if ((v = skip_lit(p, line_end, "image: ")) != nullptr) {
      have_image = parse_image(v, line_end, r.image);
    } else if (skip_lit(p, line_end, "log:") != nullptr) {
      break;  // the key lines come before the log
    }
    p = eol < end ? eol + 1 : end;
  }
  if (!magic || !have_outcome || !have_image) return false;
  out = r;
  return true;
}

// ---- the decision ------------------------------------------------------------------------

CardAction decide_card_flash(const CardFacts& f, const char** why) {
  if (!f.has_manifest) {
    *why = "no body.json on the card";
    return CardAction::Nothing;
  }
  if (f.image_bad != nullptr) {
    *why = f.image_bad;
    return CardAction::Refuse;
  }
  if (f.has_hello && f.body_build == f.manifest.build) {
    *why = "the body already runs the card's build";
    return CardAction::Same;
  }
  if (f.has_record && f.record.outcome == CardOutcome::Failed && same_image(f.record.image, f.manifest)) {
    *why = "a flash of this same image failed at an earlier boot (body-result.txt)";
    return CardAction::NotRetried;
  }
  if (f.has_battery && f.batt_mv < CardFlash::kMinBatteryMv) {
    *why = "the battery is too low to flash";
    return CardAction::Refuse;
  }
  *why = f.has_hello ? "the body runs another build" : "no HELLO from the body (no firmware, a broken one, or off)";
  return CardAction::Flash;
}

// ---- the boot check ----------------------------------------------------------------------

CardFlash::CardFlash(KernelStore& card, FlashImage& image, Flasher& flasher, SerialLink& link, Clock& clock, Log& log)
    : card_(card), image_(image), flasher_(flasher), link_(link), clock_(clock), log_(log) {}

void CardFlash::begin() {
  begin_us_ = clock_.now_us();
  hello_seen_us_ = 0;
  step_ = Step::Waiting;
}

void CardFlash::poll(bool card_free) {
  uint64_t now = clock_.now_us();
  switch (step_) {
    case Step::Idle:
    case Step::Done:
      return;
    case Step::Waiting: {
      const BodyState& b = link_.state();
      if (b.has_hello && hello_seen_us_ == 0) hello_seen_us_ = now;
      if (!b.has_hello) {
        if (now - begin_us_ < kHelloWaitUs) return;
      } else if (!b.has_telemetry && now - hello_seen_us_ < kTelemetryWaitUs) {
        return;  // the battery reading comes with the first TELEMETRY, 40 ms after the HELLO normally
      }
      if (!card_free || flasher_.busy()) return;  // a laptop's flash, the bridge or an update first
      decide();
      return;
    }
    case Step::Announce:
      if (now - flash_us_ < kAnnounceUs) return;
      if (!card_free || flasher_.busy()) return;  // a laptop's flash or an update came in between: wait
      start_flash();
      return;
    case Step::Flashing: {
      if (flasher_.busy() || !card_free) return;
      // A failed flash: up to kHelloWaitUs for the body's HELLO (the link asks again), so the result file
      // says what it runs now and the eyes can show the failure (a body without firmware never answers).
      const BodyState& b = link_.state();
      if (ended_us_ == 0) ended_us_ = now;
      bool hello = b.has_hello && b.hello_at_us >= flash_us_;
      if (flasher_.status().phase == KILO_FLASH_PHASE_FAILED && !hello && now - ended_us_ < kHelloWaitUs) return;
      finish_flash();
      return;
    }
  }
}

int32_t CardFlash::read_file(const char* name, char* buf, uint32_t cap) {
  KernelStore::Result r = card_.open(name);
  if (r == KernelStore::Result::Missing) return -1;
  if (r != KernelStore::Result::Ok) return -3;
  uint32_t n = 0;
  int32_t got;
  while ((got = card_.read((uint8_t*)buf + n, cap - n)) > 0) {
    n += (uint32_t)got;
    if (n == cap) {
      uint8_t extra;
      got = card_.read(&extra, 1);
      card_.close();
      return got > 0 ? -2 : (got < 0 ? -3 : (int32_t)n);
    }
  }
  card_.close();
  return got < 0 ? -3 : (int32_t)n;
}

// body.bin into image_, against m: null when ready, else why not (into why_).
const char* CardFlash::load_image(const Manifest& m) {
  uint8_t e = image_.begin(m);
  if (e != KILO_FLASH_ERROR_NONE) {
    format(why_, sizeof why_, "body.json: %s (build %08x, size %u; at most %u bytes)", flash_error_name(e),
           (unsigned)m.build, (unsigned)m.size, (unsigned)FlashImage::kMaxSize);
    return why_;
  }
  KernelStore::Result r = card_.open(kImageFile);
  if (r != KernelStore::Result::Ok) {
    image_.clear();
    format(why_, sizeof why_, "%s %s (body.json is there: a copy that did not finish?)", kImageFile,
           r == KernelStore::Result::Missing ? "is missing" : "cannot be read");
    return why_;
  }
  uint8_t chunk[512];
  uint32_t got_total = 0;
  bool read_failed = false, longer = false;
  for (;;) {
    int32_t got = card_.read(chunk, sizeof chunk);
    if (got < 0) {
      read_failed = true;
      break;
    }
    if (got == 0) break;
    uint32_t take = (uint32_t)got;
    if (got_total + take > m.size) {
      longer = true;
      take = got_total < m.size ? m.size - got_total : 0;
    }
    if (take > 0) image_.write(got_total, chunk, take);
    got_total += (uint32_t)got;
  }
  card_.close();
  if (read_failed) {
    image_.clear();
    format(why_, sizeof why_, "%s cannot be read (after %u bytes)", kImageFile, (unsigned)got_total);
    return why_;
  }
  if (longer) {
    image_.clear();
    format(why_, sizeof why_, "%s is %u bytes or more, body.json says %u: they do not belong together", kImageFile,
           (unsigned)got_total, (unsigned)m.size);
    return why_;
  }
  uint32_t crc = image_.crc();
  e = image_.finish();
  if (e != KILO_FLASH_ERROR_NONE) {
    if (e == KILO_FLASH_ERROR_SHORT)
      format(why_, sizeof why_, "%s is %u bytes, body.json says %u: half copied?", kImageFile, (unsigned)got_total,
             (unsigned)m.size);
    else
      format(why_, sizeof why_, "%s: %s (crc32 %08x, body.json says %08x): damaged or half copied?", kImageFile,
             flash_error_name(e), (unsigned)crc, (unsigned)m.crc32);
    return why_;
  }
  return nullptr;
}

void CardFlash::load(CardFacts& f) {
  char text[kManifestMax + 1];
  int32_t n = read_file(kManifestFile, text, kManifestMax);
  f.has_manifest = n != -1;
  if (n == -1) return;
  if (n < 0) {
    format(why_, sizeof why_, "%s %s", kManifestFile, n == -2 ? "is too big for a manifest" : "cannot be read");
    f.image_bad = why_;
    return;
  }
  text[n] = '\0';
  const char* bad = parse_manifest(text, (uint32_t)n, f.manifest);
  if (bad != nullptr) {
    format(why_, sizeof why_, "%s is not a manifest (%s): damaged or half copied?", kManifestFile, bad);
    f.image_bad = why_;
    f.manifest = {};
    return;
  }
  f.image_bad = load_image(f.manifest);

  char head[kRecordReadMax];
  int32_t h = read_file(kResultFile, head, sizeof head);
  if (h == -2) h = (int32_t)sizeof head;  // only its head matters; read_file read that much
  f.has_record = h > 0 && parse_card_record(head, (uint32_t)h, f.record);
}

void CardFlash::decide() {
  first_seq_ = ring_ != nullptr ? ring_->next_seq() : 0;
  uint64_t t0 = clock_.now_us();
  CardFacts f = {};
  load(f);
  const BodyState& b = link_.state();
  f.has_hello = b.has_hello;
  f.body_build = b.has_hello ? b.hello.fw_build : 0;
  f.has_battery = b.has_telemetry && !(b.telemetry.flags & KILO_TELEMETRY_FLAGS_NO_BATTERY) && b.telemetry.batt_mv > 0;
  f.batt_mv = f.has_battery ? b.telemetry.batt_mv : 0;
  facts_ = f;
  const char* why = nullptr;
  action_ = decide_card_flash(f, &why);
  uint32_t card_ms = (uint32_t)((clock_.now_us() - t0) / 1000u);

  char body[48];
  if (f.has_hello) format(body, sizeof body, "the body runs build %08x", (unsigned)f.body_build);
  else format(body, sizeof body, "no HELLO from the body");
  char card[64];
  if (f.has_manifest && f.image_bad == nullptr)
    format(card, sizeof card, "the card has build %08x (%u bytes)", (unsigned)f.manifest.build, (unsigned)f.manifest.size);
  else
    format(card, sizeof card, "%s", f.has_manifest ? "the card's image is bad" : "no image on the card");

  switch (action_) {
    case CardAction::Nothing:
      logf(log_, LogLevel::Info, "boot flash check: %s, %s: nothing to do (card read in %u ms)", body, card,
           (unsigned)card_ms);
      step_ = Step::Done;
      return;
    case CardAction::Refuse:
      if (why != why_)  // the battery (an image's fault is in why_ already)
        format(why_, sizeof why_, "%s: %u mV, under %u mV (too low to flash safely); charge it", why, (unsigned)f.batt_mv,
               (unsigned)kMinBatteryMv);
      logf(log_, LogLevel::Warn, "boot flash check: %s, %s: NOT flashing:", body, card);
      logf(log_, LogLevel::Warn, "boot flash check: %s", why_);
      record(CardOutcome::Refused, why_);
      tell(signals::Signal::Refused, "card flash refused");
      step_ = Step::Done;
      return;
    case CardAction::Same:
      format(why_, sizeof why_, "%s", why);
      logf(log_, LogLevel::Info, "boot flash check: %s, %s: no flash (card read in %u ms)", body, card,
           (unsigned)card_ms);
      if (!(f.has_record && same_image(f.record.image, f.manifest) &&
            (f.record.outcome == CardOutcome::Flashed || f.record.outcome == CardOutcome::Same)))
        record(CardOutcome::Same, why_);
      else
        outcome_ = CardOutcome::Same;
      image_.clear();
      step_ = Step::Done;
      return;
    case CardAction::NotRetried:
      logf(log_, LogLevel::Warn, "boot flash check: %s, %s: NOT flashing: %s", body, card, why);
      logf(log_, LogLevel::Warn, "boot flash check: it is tried again only once body.bin or body.json change "
           "(kilo card writes them again)");
      image_.clear();
      step_ = Step::Done;
      return;
    case CardAction::Flash:
      break;
  }

  format(why_, sizeof why_, "%s: flashing the card's build %08x", f.has_hello ? body : why,
         (unsigned)f.manifest.build);
  if (f.has_battery)
    logf(log_, LogLevel::Info, "boot flash check: %s, %s, battery %u mV: flashing", body, card, (unsigned)f.batt_mv);
  else
    logf(log_, LogLevel::Info, "boot flash check: %s, %s, no battery reading: flashing", body, card);
  tell(signals::Signal::Flashing, "card flash");  // the eyes white, sent while the link is still ours
  flash_us_ = clock_.now_us();
  step_ = Step::Announce;
}

// kAnnounceUs after Flashing: the walk stopped, then the flasher takes the UART.
void CardFlash::start_flash() {
  if (host_ != nullptr) host_->card_before_flash();
  flash_us_ = clock_.now_us();
  if (!flasher_.start(image_, false)) {  // poll() checked it is not busy: not expected
    logf(log_, LogLevel::Warn, "boot flash check: the flasher is busy: not flashing");
    tell(signals::Signal::Failed, "card flash not started");
    image_.clear();
    step_ = Step::Done;
    return;
  }
  step_ = Step::Flashing;
}

void CardFlash::finish_flash() {
  const Flasher::Status& s = flasher_.status();
  if (s.phase == KILO_FLASH_PHASE_DONE) {
    format(why_, sizeof why_, "flashed build %08x; the body's HELLO after it reports build %08x reset %s",
           (unsigned)s.build, (unsigned)s.fw_build, reset_cause_name(s.reset_cause));
    record(CardOutcome::Flashed, why_);  // no signal: the Mind's ready (dark -> green) follows, which ends the white
  } else {
    format(why_, sizeof why_, "the flash FAILED: %s (STK500 command %02x, attempt %u of %u)", flash_error_name(s.error),
           (unsigned)s.stk_cmd, (unsigned)s.attempt, (unsigned)Flasher::kAttempts);
    record(CardOutcome::Failed, why_);
    tell(signals::Signal::Failed, "card flash failed");
  }
  image_.clear();
  step_ = Step::Done;
}

namespace {

struct Text {
  char* buf;
  uint32_t cap;
  uint32_t n;
  void add(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    if (n + 1 >= cap) return;
    va_list ap;
    va_start(ap, fmt);
    size_t k = vformat(buf + n, cap - n, fmt, ap);
    va_end(ap);
    n += (uint32_t)k;
    if (n >= cap) n = cap - 1;
  }
};

}  // namespace

void CardFlash::record(CardOutcome outcome, const char* why) {
  outcome_ = outcome;
  const CardFacts& f = facts_;
  const BodyState& b = link_.state();
  const Flasher::Status& s = flasher_.status();
  Text t = {text_, sizeof text_, 0};
  t.add("%s\n", kResultMagic);
  t.add("outcome: %s\n", card_outcome_name(outcome));
  if (f.manifest.build != 0 || f.manifest.size != 0)  // body.json parsed
    t.add("image: build %08x size %u crc32 %08x\n", (unsigned)f.manifest.build, (unsigned)f.manifest.size,
          (unsigned)f.manifest.crc32);
  else
    t.add("image: none\n");
  t.add("why: %s\n", why);
  if (f.has_hello) t.add("body before: build %08x\n", (unsigned)f.body_build);
  else t.add("body before: no HELLO\n");
  if (outcome == CardOutcome::Flashed || outcome == CardOutcome::Failed) {
    if (b.has_hello && b.hello_at_us >= flash_us_)
      t.add("body after: build %08x reset %s\n", (unsigned)b.hello.fw_build, reset_cause_name(b.hello.reset_cause));
    else
      t.add("body after: no HELLO since the flash began\n");
    t.add("error: %s\n", outcome == CardOutcome::Failed ? flash_error_name(s.error) : "none");
  }
  if (f.has_battery) t.add("battery: %u mV\n", (unsigned)f.batt_mv);
  else t.add("battery: no reading\n");
  t.add("mind: build %08x, %u ms after the link came up\n", (unsigned)mind_build_, (unsigned)ms_since_begin());
  const char* next = "";
  switch (outcome) {
    case CardOutcome::Flashed:
    case CardOutcome::Same:
      next = "nothing to do: the body runs the card's build";
      break;
    case CardOutcome::Failed:
      next = "not retried while body.bin and body.json stay as they are; kilo card writes them again (and removes "
             "this file) to retry";
      break;
    case CardOutcome::Refused:
      next = f.image_bad != nullptr ? "checked again: copy body.bin and body.json whole (kilo card)"
                                    : "checked again: charge the battery first";
      break;
    case CardOutcome::None:
      break;
  }
  t.add("next boot: %s\n", next);
  t.add("log:\n");
  if (ring_ != nullptr) {
    uint32_t last = ring_->next_seq();
    uint32_t from = first_seq_;
    if (last - from > kLogLines) from = last - kLogLines;
    if (from < ring_->first_seq()) from = ring_->first_seq();
    for (uint32_t seq = from; seq < last; seq++) {
      const LogEntry* e = ring_->get(seq);
      if (e == nullptr) continue;
      static const char kLevel[] = "DIWE";
      t.add("  %u.%03u %c %s\n", (unsigned)(e->t_ms / 1000u), (unsigned)(e->t_ms % 1000u),
            kLevel[(uint8_t)e->level & 3u], e->text);
    }
  }
  if (!write_result())
    logf(log_, LogLevel::Warn, "boot flash check: %s is NOT on the card (outcome %s)", kResultFile,
         card_outcome_name(outcome));
  else
    logf(log_, LogLevel::Info, "boot flash check: wrote %s (outcome %s, %u bytes)", kResultFile,
         card_outcome_name(outcome), (unsigned)t.n);
}

// body-result.new, then the rename over body-result.txt: a power cut leaves the old file or the new one whole.
bool CardFlash::write_result() {
  uint32_t n = 0;
  while (n < sizeof text_ && text_[n] != '\0') n++;
  if (card_.create(kResultNewFile) != KernelStore::Result::Ok) return false;
  bool ok = card_.write((const uint8_t*)text_, n) == KernelStore::Result::Ok;
  ok = card_.close() == KernelStore::Result::Ok && ok;
  if (!ok) {
    card_.remove(kResultNewFile);
    return false;
  }
  KernelStore::Result r = card_.remove(kResultFile);
  if (r == KernelStore::Result::Failed) return false;
  if (card_.rename(kResultNewFile, kResultFile) != KernelStore::Result::Ok) return false;
  return card_.sync() == KernelStore::Result::Ok;
}

void CardFlash::tell(signals::Signal s, const char* why) {
  if (host_ != nullptr) host_->card_signal(s, why);
}

}  // namespace kilo
