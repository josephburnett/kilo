// What Kilo tells the owner (signal/signal.h, docs/signals.md): the eye
// language's patterns and its priorities (the guard's blue and red over
// everything but the reprogramming white; a one-shot waits for green and is
// dropped after kWaitUs; one waits behind the one playing), the SET_EYES it
// sends (a change again twice, so the body's echo window cannot lose it),
// and the Mind's call sites on a scripted body: ready after the first HELLO,
// a refused key, the C5 twice busy blink, light sleep dark.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "mind.h"
#include "signal/signal.h"
#include "tests/check.h"
#include "tests/mind_rig.h"

using namespace kilo;
using namespace kilo::signals;

namespace {

constexpr uint64_t kMs = 1000;
const State kCalm = {guard::Eyes::Green, false, false};

// The looks of one EyeLanguage from t0 to t1 (us), every 10 ms, with the same state.
std::vector<std::string> looks(EyeLanguage& e, const State& st, uint64_t t0, uint64_t t1) {
  std::vector<std::string> out;
  for (uint64_t t = t0; t < t1; t += 10 * kMs) {
    const char* n = look_name(e.look(st, t));
    if (out.empty() || out.back() != n) out.push_back(n);
  }
  return out;
}

void test_names_and_sounds() {
  CHECK(std::strcmp(signal_sound(Signal::Ready), "mew") == 0 && std::strcmp(signal_sound(Signal::Done), "trill") == 0);
  CHECK(std::strcmp(signal_sound(Signal::Refused), "hiss") == 0 && std::strcmp(signal_sound(Signal::Failed), "hiss") == 0);
  CHECK(std::strcmp(signal_sound(Signal::GaveUp), "meow") == 0 && std::strcmp(signal_sound(Signal::GuardRed), "hiss") == 0);
  CHECK(signal_sound(Signal::Saved) == nullptr && signal_sound(Signal::Flashing) == nullptr);
  CHECK(std::strcmp(signal_name(Signal::GaveUp), "gave_up") == 0);
  // the colours: the guard's own levels, every one distinct
  const Look all[] = {kOff, kGreen, kBlue, kRed, kOrange, kWhite, kPurple, kDimGreen};
  for (const Look& a : all)
    for (const Look& b : all) CHECK(&a == &b || !same(a, b));
  CHECK(kGreen.g == guard::kEyeLevel && kOrange.g == guard::kOrangeGreen && kWhite.mask == 0x3F);
}

void test_patterns() {
  {  // not ready yet: dark, whatever the guard, busy or sleep say; then ready: steady green, for good
    EyeLanguage e;
    CHECK(!e.ready() && looks(e, kCalm, 0, 3000 * kMs) == std::vector<std::string>({"off"}));
    CHECK(same(e.look({guard::Eyes::Red, false, true}, 3000 * kMs), kOff) &&
          same(e.look({guard::Eyes::Blue, true, false}, 3000 * kMs), kOff));
    e.show(Signal::Ready, 3000 * kMs);
    CHECK(e.ready() && !e.playing() && e.shown() == 0);  // no flashes: dark -> green is the signal
    CHECK(looks(e, kCalm, 3000 * kMs, 6000 * kMs) == std::vector<std::string>({"green"}));
    e.show(Signal::Ready, 6000 * kMs);  // again (awake after light sleep): nothing changes
    CHECK(looks(e, kCalm, 6000 * kMs, 7000 * kMs) == std::vector<std::string>({"green"}));
  }
  {  // the steady states
    EyeLanguage e;
    e.show(Signal::Ready, 0);
    CHECK(looks(e, kCalm, 0, 3000 * kMs) == std::vector<std::string>({"green"}));
    CHECK(looks(e, {guard::Eyes::Orange, false, false}, 0, 1000 * kMs) == std::vector<std::string>({"orange"}));
    // busy: white half a second, dark half a second
    std::vector<std::string> busy = looks(e, {guard::Eyes::Green, false, true}, 0, 2000 * kMs);
    CHECK(busy == std::vector<std::string>({"white", "off", "white", "off"}));
    CHECK(same(e.look({guard::Eyes::Green, false, true}, 499 * kMs), kWhite) &&
          same(e.look({guard::Eyes::Green, false, true}, 500 * kMs), kOff));
    // asleep: dark, a 100 ms dim green blink every 5 s
    State asleep = {guard::Eyes::Green, true, true};  // (asleep wins over busy)
    int blink_ms = 0;
    for (uint64_t t = 0; t < 10000 * kMs; t += kMs) blink_ms += same(e.look(asleep, t), kDimGreen) ? 1 : 0;
    CHECK(blink_ms == 200 && same(e.look(asleep, 2000 * kMs), kOff));
  }
  {  // three quick flashes, then the state again
    struct C {
      Signal s;
      const char* colour;
    };
    const C cases[] = {{Signal::Done, "green"}, {Signal::Refused, "purple"}, {Signal::Failed, "red"},
                       {Signal::GaveUp, "red"}};
    for (const C& c : cases) {
      EyeLanguage e;
      e.show(Signal::Ready, 0);
      e.show(c.s, 1000 * kMs);
      std::vector<std::string> l = looks(e, kCalm, 1000 * kMs, 3000 * kMs);
      std::vector<std::string> want = {c.colour, "off", c.colour, "off", c.colour, "off", "green"};
      CHECKF(l == want, "%s: %zu looks", signal_name(c.s), l.size());
      CHECK(e.shown() == 1 && e.dropped() == 0);
      CHECK(EyeLanguage::length(c.s) == 900 * kMs);
    }
  }
  {  // saved: one long white flash
    EyeLanguage e;
    e.show(Signal::Ready, 0);
    e.show(Signal::Saved, 0);
    CHECK(looks(e, kCalm, 0, 2000 * kMs) == std::vector<std::string>({"white", "off", "green"}));
  }
  {  // a one-shot over busy and over sleep, which come back after it
    EyeLanguage e;
    e.show(Signal::Ready, 0);
    e.show(Signal::Done, 0);
    std::vector<std::string> l = looks(e, {guard::Eyes::Green, false, true}, 0, 1500 * kMs);
    CHECK(l.size() >= 7 && l[0] == "green" && l[5] == "off" && l[6] == "white");
  }
}

void test_priorities() {
  const State blue = {guard::Eyes::Blue, false, false}, red = {guard::Eyes::Red, false, true};
  {  // the guard's blue and red win over busy, and over a flash already playing
    EyeLanguage e;
    e.show(Signal::Ready, 0);
    CHECK(same(e.look(red, 0), kRed) && same(e.look(blue, 600 * kMs), kBlue));
    e.show(Signal::Refused, 1000 * kMs);
    CHECK(same(e.look(kCalm, 1000 * kMs), kPurple));
    CHECK(same(e.look(blue, 1050 * kMs), kBlue));  // a hand in his face mid-flash: blue
    CHECK(same(e.look(kCalm, 1350 * kMs), kPurple));  // gone: the flash goes on where its time is
  }
  {  // a one-shot asked while blue waits for green ...
    EyeLanguage e;
    e.show(Signal::Ready, 0);
    e.show(Signal::Done, 0);
    for (uint64_t t = 0; t < 2000 * kMs; t += 10 * kMs) CHECK(same(e.look(blue, t), kBlue));
    CHECK(!e.playing() && e.shown() == 0);
    CHECK(same(e.look(kCalm, 2000 * kMs), kGreen) && e.playing());  // ... and then plays whole
    CHECK(looks(e, kCalm, 2000 * kMs, 4000 * kMs) == std::vector<std::string>({"green", "off", "green", "off", "green", "off", "green"}));
  }
  {  // ... but at most kWaitUs: then it is dropped, not shown late
    EyeLanguage e;
    e.show(Signal::Ready, 0);
    e.show(Signal::Failed, 0);
    for (uint64_t t = 0; t <= EyeLanguage::kWaitUs + 10 * kMs; t += 10 * kMs) e.look(blue, t);
    CHECK(e.dropped() == 1 && !e.playing());
    CHECK(looks(e, kCalm, 3100 * kMs, 5000 * kMs) == std::vector<std::string>({"green"}));
  }
  {  // one waits behind the playing one; a third replaces the waiting one
    EyeLanguage e;
    e.show(Signal::Ready, 0);
    e.show(Signal::Done, 0);
    e.look(kCalm, 0);
    e.show(Signal::Refused, 100 * kMs);
    e.show(Signal::Failed, 200 * kMs);  // replaces Refused
    std::vector<std::string> l = looks(e, kCalm, 0, 3000 * kMs);
    std::vector<std::string> want = {"green", "off", "green", "off", "green", "off", "red", "off",
                                     "red",   "off", "red",   "off", "green"};
    CHECKF(l == want, "%zu looks", l.size());
    CHECK(e.shown() == 2 && e.dropped() == 1);
  }
  {  // the reprogramming white over everything, until ready (it worked) ...
    EyeLanguage e;
    e.show(Signal::Flashing, 0);
    CHECK(e.flashing() && same(e.look(red, 0), kWhite) && same(e.look(kCalm, 20000 * kMs), kWhite));
    e.show(Signal::Ready, 20000 * kMs);
    CHECK(!e.flashing() && looks(e, kCalm, 20000 * kMs, 22000 * kMs) == std::vector<std::string>({"green"}));
  }
  {  // ... or the failure's red flashes, before ready (at once: the guard is not shown yet), then dark
    EyeLanguage e;
    e.show(Signal::Flashing, 0);
    e.show(Signal::Failed, 20000 * kMs);
    CHECK(!e.flashing() && looks(e, blue, 20000 * kMs, 22000 * kMs) ==
                               std::vector<std::string>({"red", "off", "red", "off", "red", "off"}));
    e.show(Signal::Ready, 22000 * kMs);  // the body answers: he is usable
    CHECK(same(e.look(kCalm, 22000 * kMs), kGreen));
  }
  {  // a refused card flash before ready: purple x3, then dark until ready
    EyeLanguage e;
    CHECK(same(e.look(kCalm, 0), kOff));
    e.show(Signal::Refused, 1000 * kMs);
    CHECK(looks(e, kCalm, 1000 * kMs, 3000 * kMs) ==
          std::vector<std::string>({"purple", "off", "purple", "off", "purple", "off"}));
  }
  {  // the guard's hiss: a sound only, the eyes are the guard's red already
    EyeLanguage e;
    e.show(Signal::Ready, 0);
    e.show(Signal::GuardRed, 0);
    CHECK(!e.playing() && same(e.look(kCalm, 10 * kMs), kGreen) && e.shown() == 0);
  }
}

// ---- the SET_EYES it sends ----------------------------------------------------------------------------

struct LinkRig {
  FakeBody body;
  CaptureLog log;
  SerialLink link{body.serial, body.clock, log};
  EyeDriver driver{link};
  LinkRig() {
    body.eyes = true;
    link.connect();
    for (int i = 0; i < 50; i++) pass(kCalmLook());
  }
  static Look kCalmLook() { return kGreen; }
  void pass(const Look& l) {
    link.mark_alive();
    body.step();
    link.poll();
    driver.poll(l, true, body.clock.now_us());
    link.poll();
    body.react();
  }
};

void test_driver_repeats_a_change() {
  LinkRig r;
  size_t before = r.body.count(KILO_MSG_SET_EYES);
  CHECK(before >= 1 && same(r.driver.sent(), kGreen));
  uint64_t t0 = r.body.clock.now_us();
  for (int i = 0; i < 25; i++) r.pass(kPurple);  // 100 ms
  std::vector<Sent> s = r.body.of(KILO_MSG_SET_EYES);
  int purple = 0;
  for (size_t i = before; i < s.size(); i++)
    if (s[i].payload.size() == 4 && s[i].payload[1] == kLevel && s[i].payload[3] == kLevel) purple++;
  // the change and its two repeats (40 and 80 ms later), whatever the 5 Hz refresh adds
  CHECKF(purple >= 3 && purple <= 4, "%d purple SET_EYES in %u ms", purple,
         (unsigned)((r.body.clock.now_us() - t0) / 1000));
  CHECK(r.driver.changes() == 2);
  // the same look for 1 s: only the refresh (5 Hz)
  size_t mid = r.body.count(KILO_MSG_SET_EYES);
  for (int i = 0; i < 250; i++) r.pass(kPurple);
  size_t refreshes = r.body.count(KILO_MSG_SET_EYES) - mid;
  CHECKF(refreshes >= 4 && refreshes <= 6, "%zu SET_EYES in 1 s of the same look", refreshes);
  // not driven (old firmware): nothing more
  size_t off = r.body.count(KILO_MSG_SET_EYES);
  r.link.clear_goal(KILO_MSG_SET_EYES);
  for (int i = 0; i < 50; i++) {
    r.link.mark_alive();
    r.body.step();
    r.link.poll();
    r.driver.poll(kRed, false, r.body.clock.now_us());
    r.body.react();
  }
  CHECK(r.body.count(KILO_MSG_SET_EYES) == off);
}

// ---- the Mind's call sites ----------------------------------------------------------------------------

struct EyeRig : MindRig {
  explicit EyeRig(const MindConfig& c) : MindRig(c) { body.eyes = true; }
  // The SET_EYES colours sent since index from, as names, consecutive repeats folded.
  std::vector<std::string> sent_looks(size_t from) const {
    std::vector<std::string> out;
    std::vector<Sent> s = body.of(KILO_MSG_SET_EYES);
    for (size_t i = from; i < s.size(); i++) {
      if (s[i].payload.size() != 4) continue;
      Look l = {s[i].payload[0], s[i].payload[1], s[i].payload[2], s[i].payload[3]};
      const char* n = look_name(l);
      if (out.empty() || out.back() != n) out.push_back(n);
    }
    return out;
  }
  const char* now() const { return look_name(mind.eye_driver().sent()); }
};

void test_mind_ready_and_refused_key() {
  MindConfig c = MindRig::remote_cfg();
  c.rear_up_key = 1;  // A1 on (KILO_REAR_UP=1)
  EyeRig r(c);
  r.body.batt_mv = 7300;  // under the rear-up's 7400 mV, above tired
  r.start(1500);
  std::vector<std::string> l = r.sent_looks(0);
  // dark until ready (the HELLO, then TELEMETRY), then steady green: no flashes
  CHECKF(l.back() == "green" && (l.size() == 1 || (l.size() == 2 && l[0] == "off")),
         "%zu looks after the first HELLO (first %s)", l.size(), l.empty() ? "-" : l[0].c_str());
  CHECK(r.log.has("signal ready why=ready") && r.mind.ready() && r.mind.eyes().shown() == 0);
  // a refused key (A1's rear-up on a low battery): three purple flashes
  size_t from = r.body.count(KILO_MSG_SET_EYES);
  r.key(0x42, 1500);
  l = r.sent_looks(from);
  CHECKF(l.size() == 7 && l[0] == "purple" && l[2] == "purple" && l[4] == "purple" && l[6] == "green",
         "%zu looks after a refused key (first %s)", l.size(), l.empty() ? "-" : l[0].c_str());
  CHECK(r.log.has("signal refused why=key refused"));
}

void test_mind_ready_waits_for_the_card_check() {
  // the boot-time card flash check holds ready: dark (the body alive, the guard green) until it is over
  EyeRig r(MindRig::remote_cfg());
  r.mind.hold_ready(true);
  r.start(3000);
  std::vector<std::string> l = r.sent_looks(0);
  CHECKF(l == std::vector<std::string>({"off"}) && !r.mind.ready(), "%zu looks while held (first %s)", l.size(),
         l.empty() ? "-" : l[0].c_str());
  CHECK(!r.log.has("signal ready"));
  r.mind.hold_ready(false);
  r.run_ms(200);
  CHECK(r.mind.ready() && std::strcmp(r.now(), "green") == 0 && r.log.has("signal ready why=ready"));
  CHECK(r.sent_looks(0) == std::vector<std::string>({"off", "green"}));
}

void test_mind_measure_busy() {
  EyeRig r(MindRig::remote_cfg());
  r.start(1500);
  r.key(0x90, 1500);  // C5: the calibration posture
  size_t from = r.body.count(KILO_MSG_SET_EYES);
  r.key(0x90, 3000);  // C5 again: stand, wait for him to be still, measure: busy
  std::vector<std::string> l = r.sent_looks(from);
  int whites = 0;
  for (const std::string& s : l) whites += s == "white" ? 1 : 0;
  CHECKF(whites >= 2 && l.size() >= 4, "%d white blinks in 3 s", whites);
  CHECK(r.mind.remote().measure_pending() || r.mind.measure().active());
  // any key stops it (STOPPED: no flash), the eyes green again
  r.key(0x02, 3000);  // (the routine's end waits up to 1.5 s for the body)
  CHECKF(!r.mind.measure().active() && !r.mind.remote().measure_pending() && std::strcmp(r.now(), "green") == 0,
         "measure %d pending %d eyes %s", r.mind.measure().active(), r.mind.remote().measure_pending(), r.now());
  CHECK(!r.log.has("signal failed why=measurement"));
}

void test_mind_measure_stops_itself() {
  // he stops by himself (tipped past 25 degrees): three red flashes
  EyeRig r(MindRig::remote_cfg());
  r.start(1500);
  r.key(0x90, 1500);
  r.key(0x90, 3000);
  CHECK(r.mind.measure().active());
  size_t at = r.body.count(KILO_MSG_SET_EYES);
  r.body.pitch = 30 * 16;
  r.run_ms(3000);
  r.body.pitch = 0;
  CHECK(!r.mind.measure().active() && r.log.has("signal failed why=measurement stopped itself"));
  std::vector<std::string> l = r.sent_looks(at);
  int reds = 0;
  for (const std::string& s : l) reds += s == "red" ? 1 : 0;
  CHECKF(reds == 3, "%d red flashes after the routine stopped itself", reds);
}

void test_mind_light_sleep_dark() {
  MindConfig c = MindRig::remote_cfg();
  c.light_sleep = true;
  EyeRig r(c);
  r.start(1500);
  r.key(0xA2, 40000);  // A7: rest, the servos off, 30 s later asleep
  CHECK(r.mind.sleep().asleep());
  size_t from = r.body.count(KILO_MSG_SET_EYES);
  r.run_ms(10000);
  std::vector<std::string> l = r.sent_looks(from);
  int blinks = 0, other = 0;
  for (const std::string& s : l) {
    if (s == "dim green") blinks++;
    else if (s != "off") other++;
  }
  CHECKF(blinks == 2 && other == 0, "asleep 10 s: %d blinks, %d other looks", blinks, other);
  r.key(0x02, 1500);  // B6 wakes him: green again (the mew; no flashes)
  CHECK(!r.mind.sleep().asleep() && r.log.has("signal ready why=wake") && std::strcmp(r.now(), "green") == 0);
  CHECK(r.mind.eyes().shown() == 0);
}

}  // namespace

int main() {
  test_names_and_sounds();
  test_patterns();
  test_priorities();
  test_driver_repeats_a_change();
  test_mind_ready_and_refused_key();
  test_mind_ready_waits_for_the_card_check();
  test_mind_measure_busy();
  test_mind_measure_stops_itself();
  test_mind_light_sleep_dark();
  return report("signal");
}
