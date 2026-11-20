// Kilo's sounds (sound/): the mu-law table, the names, the renderer (the
// DC ramp, the fades, the volume, the rate conversion, no clicks), the
// clock-driven output, the player (families, the queue, the gap, stale
// requests, off, asleep, no sounds), the behaviour's chatter, and the
// Mind's hooks on a scripted body: the first HELLO's mew, B2's meow, light
// sleep silencing the output and the wake's mew, the guard's hiss when it
// backs off, the purr while the A5 behaviour sits, each start a MARK in
// the flight recorder, and the metrics line after the power lines. The
// sounds are synthetic tones made here (kernel/sounds/ is not read).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "mind.h"
#include "sound/player.h"
#include "sound/render.h"
#include "sound/sound.h"
#include "tests/body_rig.h"
#include "tests/check.h"

using namespace kilo;
using namespace kilo::sound;

namespace {

// ---- synthetic sounds ------------------------------------------------------------------------

// kernel/tools/pack_sounds.py ulaw_encode()
uint8_t ulaw_encode(int sample) {
  int sign = sample < 0 ? 0x80 : 0;
  if (sample < 0) sample = -sample;
  if (sample > 32635) sample = 32635;
  sample += 0x84;
  int v = (sample >> 7) & 0xFF, exponent = 0;
  while (v > 1) {
    v >>= 1;
    exponent++;
  }
  int mantissa = (sample >> (exponent + 3)) & 0x0F;
  return (uint8_t)(~(sign | (exponent << 4) | mantissa) & 0xFF);
}

struct Bank {
  std::vector<std::vector<uint8_t>> data;
  std::vector<std::string> names;
  std::vector<Sound> sounds;
  // a tone: hz (0: a constant level), seconds, amplitude 0..32767
  void add(const char* name, double hz, double s, double amp, uint32_t rate = 22050) {
    std::vector<uint8_t> d((size_t)(s * rate));
    for (size_t i = 0; i < d.size(); i++)
      d[i] = ulaw_encode((int)(hz > 0 ? amp * std::sin(2 * M_PI * hz * (double)i / rate) : amp));
    data.push_back(d);
    names.push_back(name);
    sounds.clear();
    for (size_t k = 0; k < data.size(); k++)
      sounds.push_back({names[k].c_str(), rate, (uint32_t)data[k].size(), data[k].data()});
  }
  static Bank cat() {
    Bank b;
    b.add("meow1", 600, 0.5, 12000);
    b.add("meow2", 700, 0.4, 12000);
    b.add("meow3", 650, 0.6, 12000);
    b.add("mew", 900, 0.2, 9000);
    b.add("purr", 30, 1.5, 9000);
    b.add("hiss", 3000, 0.4, 9000);
    b.add("trill", 1200, 0.3, 9000);
    return b;
  }
};

// ---- names, the table, the gain ---------------------------------------------------------------

void test_ulaw_and_names() {
  CHECK(kUlaw.v[0xFF] == 0 && kUlaw.v[0x7F] == 0);
  CHECK(kUlaw.v[0x80] == 32124 && kUlaw.v[0x00] == -32124);
  for (int s : {-32000, -1000, -3, 0, 5, 700, 12000, 32000}) {  // the encoder and the table agree within a step
    int back = kUlaw.v[ulaw_encode(s)];
    int tol = std::abs(s) / 16 + 8;
    CHECKF(std::abs(back - s) <= tol, "%d -> %d", s, back);
  }
  char f[kNameMax + 1];
  family("meow2", f);
  CHECK(std::strcmp(f, "meow") == 0);
  family("mew", f);
  CHECK(std::strcmp(f, "mew") == 0);
  family("a12", f);
  CHECK(std::strcmp(f, "a") == 0);
  CHECK(is_variant("meow3") && !is_variant("meow") && !is_variant(""));
  CHECK(valid_name("meow1") && valid_name("big_hiss") && !valid_name("Meow") && !valid_name("") &&
        !valid_name("a-b") && !valid_name("sixteen_letters_"));
  CHECK(gain_q16(100) == kUnity && gain_q16(0) == 0 && gain_q16(50) == kUnity / 4 && gain_q16(200) == kUnity);
  CHECK(gain_q16(70) == 70u * 70u * kUnity / 10000u);
}

// ---- the renderer -----------------------------------------------------------------------------

struct Out {
  std::vector<int> v;  // one channel
  void take(const unsigned* w, uint32_t n) {
    for (uint32_t i = 0; i < n; i += 2) {
      CHECK(w[i] == w[i + 1]);  // mono on both channels
      v.push_back((int)w[i]);
    }
  }
};

// Render frames frames in chunks of 512, as the device asks.
uint32_t run(Renderer& r, Out& o, uint32_t frames) {
  std::vector<unsigned> buf(1024);
  uint32_t got = 0;
  while (frames > 0) {
    uint32_t n = frames > 512 ? 512 : frames;
    uint32_t w = r.render(buf.data(), n);
    if (w == 0) return got;
    o.take(buf.data(), w);
    got += w / 2;
    frames -= n;
  }
  return got;
}

int max_step(const std::vector<int>& v, size_t from = 0, size_t to = SIZE_MAX) {
  int m = 0;
  for (size_t i = from + 1; i < v.size() && i < to; i++) m = std::max(m, std::abs(v[i] - v[i - 1]));
  return m;
}

void test_renderer_ramp_and_silence() {
  const uint32_t max = 5668;  // the Pi 2's PWM range at 44.1 kHz, less one
  Renderer r(kOutRate, max);
  std::vector<unsigned> buf(1024);
  CHECK(r.settled_off() && r.render(buf.data(), 512) == 0);  // off: nothing to send, the device stops
  Out o;
  r.power(true);
  run(r, o, kOutRate / 5);  // 200 ms
  CHECK(r.at_level() && o.v.back() == (int)max / 2);
  CHECK(o.v.front() < 5);  // from 0 ...
  for (size_t i = 1; i < o.v.size(); i++) CHECK(o.v[i] >= o.v[i - 1]);  // ... rising, never a jump
  int ramp_frames = 0;
  for (int x : o.v) ramp_frames += x < (int)max / 2;
  CHECKF(std::abs(ramp_frames - (int)(kRampMs * kOutRate / 1000)) < 10, "%d frames of ramp", ramp_frames);
  CHECK(max_step(o.v) <= 2);
  // down again, then render() says stop
  o.v.clear();
  r.power(false);
  uint32_t got = run(r, o, kOutRate);
  CHECKF(got < kOutRate / 5 && o.v.back() == 0 && r.settled_off(), "%u frames, last %d", got, o.v.back());
  CHECK(max_step(o.v) <= 2);
  CHECK(r.render(buf.data(), 512) == 0);
}

void test_renderer_clip_fades_and_volume() {
  const uint32_t max = 5668;
  Bank b;
  b.add("level", 0, 0.5, 30000);  // a constant: the worst case for a click at either end
  b.add("tone", 1000, 0.5, 30000);
  Renderer r(kOutRate, max);
  r.power(true);
  Out o;
  run(r, o, kOutRate / 5);
  o.v.clear();
  r.start(b.sounds[0], kUnity);
  CHECK(r.sounding());
  run(r, o, kOutRate);  // the clip (0.5 s), then silence
  CHECK(!r.sounding() && r.clips_ended() == 1);
  const int mid = (int)max / 2;
  int peak = 0;
  for (int x : o.v) peak = std::max(peak, x - mid);
  CHECKF(peak > mid * 85 / 100 && peak <= mid, "peak %d of %d", peak, mid);
  // the fade in: kFadeInMs to the top; the fade out: kFadeOutMs back to silence; no step a click
  size_t up = 0;
  while (up < o.v.size() && o.v[up] - mid < peak) up++;
  CHECKF(std::abs((int)up - (int)(kFadeInMs * kOutRate / 1000)) <= 3, "fade in %zu frames", up);
  size_t end = o.v.size();
  while (end > 0 && o.v[end - 1] == mid) end--;
  size_t down = end;
  while (down > 0 && o.v[down - 1] - mid < peak) down--;
  CHECKF(std::abs((int)(end - down) - (int)(kFadeOutMs * kOutRate / 1000)) <= 3, "fade out %zu frames", end - down);
  CHECKF(max_step(o.v) <= peak / 200 + 2, "largest step %d", max_step(o.v));
  int len = (int)end;
  CHECKF(std::abs(len - (int)(0.5 * kOutRate)) < 50, "%d frames for 0.5 s", len);
  // the tone: 1 kHz at the output (rate conversion), crossings counted
  o.v.clear();
  r.start(b.sounds[1], kUnity);
  run(r, o, kOutRate / 2);
  int cross = 0;
  for (size_t i = 1; i < o.v.size(); i++) cross += (o.v[i - 1] < mid) != (o.v[i] < mid);
  CHECKF(std::abs(cross - 1000) <= 6, "%d crossings in 0.5 s at 1 kHz", cross);
  // volume: 0 is silence, 50 a quarter of the amplitude
  o.v.clear();
  r.start(b.sounds[0], 0);
  run(r, o, kOutRate / 10);
  CHECK(max_step(o.v) == 0 && o.v.front() == mid);
  o.v.clear();
  r.start(b.sounds[0], gain_q16(50));
  run(r, o, kOutRate / 10);
  int q = 0;
  for (int x : o.v) q = std::max(q, x - mid);
  CHECKF(std::abs(q - peak / 4) <= 3, "volume 50: %d (full %d)", q, peak);
  // stop(): faded out over kFadeOutMs, not cut
  o.v.clear();
  r.start(b.sounds[0], kUnity);
  run(r, o, kOutRate / 10);
  r.stop();
  size_t at = o.v.size();
  run(r, o, kOutRate / 10);
  size_t quiet = at;
  while (quiet < o.v.size() && o.v[quiet] != mid) quiet++;
  CHECKF((int)(quiet - at) >= (int)(kFadeOutMs * kOutRate / 1000) - 3, "stopped in %zu frames", quiet - at);
  CHECK(max_step(o.v, at - 10) <= peak / 200 + 2 && !r.sounding());
  // a sound longer than 65536 samples (3 s at 22050 Hz) plays to its end
  Bank lng;
  lng.add("long", 200, 4.0, 10000);
  o.v.clear();
  r.start(lng.sounds[0], kUnity);
  run(r, o, 5 * kOutRate);
  size_t e = o.v.size();
  while (e > 0 && o.v[e - 1] == mid) e--;
  CHECKF(std::abs((int)e - 4 * (int)kOutRate) < 100, "4 s sound: %zu frames", e);
}

void test_renderer_waits_for_the_level() {
  Bank b;
  b.add("tone", 500, 0.2, 20000);
  Renderer r(kOutRate, 5668);
  r.power(true);
  r.start(b.sounds[0], kUnity);  // asked before the level is up: it waits for the ramp
  Out o;
  run(r, o, kOutRate / 2);
  size_t ramp = kRampMs * kOutRate / 1000;
  for (size_t i = 1; i < ramp - 5; i++) CHECK(o.v[i] >= o.v[i - 1]);  // the ramp is clean
  CHECK(r.clips_ended() == 1);
  r.start(b.sounds[0], kUnity);
  r.power(false);  // off cuts it: nothing plays without the level
  CHECK(!r.sounding());
}

// ---- the clock's output -----------------------------------------------------------------------

void test_timed_output() {
  Bank b;
  b.add("tone", 500, 0.5, 20000);
  FakeClock clock;
  TimedOutput out(clock);
  OutputStats st;
  out.stats(st);
  CHECK(st.state == KILO_SOUND_STATE_IDLE && st.starts == 0);
  CHECK(out.play(b.sounds[0], kUnity));
  int ms = 0;
  while (out.playing() && ms < 2000) {
    clock.advance_ms(4);
    out.poll();
    ms += 4;
  }
  CHECKF(ms >= 560 && ms <= 600, "played for %d ms (80 ms ramp + 500 ms)", ms);  // the ramp, then the sound
  out.stats(st);
  uint32_t chunks = st.chunks;
  for (int i = 0; i < 100; i++) {  // stopped between sounds: ramps down, then nothing is rendered
    clock.advance_ms(4);
    out.poll();
  }
  out.stats(st);
  uint32_t after = st.chunks;
  for (int i = 0; i < 100; i++) {
    clock.advance_ms(4);
    out.poll();
  }
  out.stats(st);
  CHECKF(st.chunks == after && after > chunks, "chunks %u %u %u", chunks, after, st.chunks);
  // asleep: refused; awake: plays again
  out.suspend(true);
  out.stats(st);
  CHECK(st.state == KILO_SOUND_STATE_ASLEEP && !out.play(b.sounds[0], kUnity));
  out.suspend(false);
  CHECK(out.play(b.sounds[0], kUnity) && out.playing());
  out.stats(st);
  CHECK(st.starts == 2 && st.state == KILO_SOUND_STATE_PLAYING);
  // asleep while playing: faded, silent, stopped
  out.suspend(true);
  for (int i = 0; i < 50; i++) {
    clock.advance_ms(4);
    out.poll();
  }
  CHECK(!out.playing() && out.renderer().settled_off());
}

// ---- the player -------------------------------------------------------------------------------

struct PlayerRig {
  FakeClock clock;
  CaptureLog log;
  Bank bank = Bank::cat();
  TimedOutput out{clock};
  SoundPlayer p{clock, log, 7};
  PlayerRig() {
    p.set_sounds(bank.sounds.data(), (uint32_t)bank.sounds.size());
    p.set_output(&out);
  }
  void run_ms(int ms) {
    for (int i = 0; i < ms; i += 4) {
      clock.advance_ms(4);
      p.poll();
    }
  }
  // run until nothing plays or waits
  int drain(int max_ms = 10000) {
    int ms = 0;
    while (p.busy() && ms < max_ms) {
      run_ms(4);
      ms += 4;
    }
    return ms;
  }
};

void test_player_families_and_results() {
  PlayerRig r;
  CHECK(r.p.count() == 7 && r.p.find("purr") != nullptr && r.p.find("meow") == nullptr);
  CHECK(r.p.request("nosuch", "test") == KILO_SOUND_RESULT_UNKNOWN && r.p.unknown() == 1);
  CHECK(r.p.request("meow4", "test") == KILO_SOUND_RESULT_UNKNOWN);  // a variant that is not there
  CHECK(r.p.request("meow2", "test") == KILO_SOUND_RESULT_QUEUED && r.p.playing());  // at once
  CHECK(std::strcmp(r.p.last()->name, "meow2") == 0);
  r.drain();
  // the family: each meow from the three, never the same twice in a row
  std::vector<std::string> got;
  for (int i = 0; i < 30; i++) {
    CHECK(r.p.request("meow", "test") == KILO_SOUND_RESULT_QUEUED);
    r.drain();
    got.push_back(r.p.last()->name);
  }
  int counts[3] = {};
  for (size_t i = 0; i < got.size(); i++) {
    CHECK(got[i].compare(0, 4, "meow") == 0);
    counts[got[i][4] - '1']++;
    if (i > 0) CHECKF(got[i] != got[i - 1], "%s twice", got[i].c_str());
  }
  CHECKF(counts[0] > 3 && counts[1] > 3 && counts[2] > 3, "meow1 %d meow2 %d meow3 %d", counts[0], counts[1], counts[2]);
  CHECK(r.p.request("mew", "test") == KILO_SOUND_RESULT_QUEUED);  // a family of one
  r.drain();
  CHECK(std::strcmp(r.p.last()->name, "mew") == 0);
  CHECK(r.log.has("sound play name=mew asked=mew why=test volume=70 ms=200"));
  // off, asleep
  r.p.set_enabled(false);
  CHECK(r.p.request("mew", "test") == KILO_SOUND_RESULT_OFF && r.p.refused_off() == 1);
  r.p.set_enabled(true);
  r.p.suspend(true);
  CHECK(r.p.request("mew", "test") == KILO_SOUND_RESULT_ASLEEP && r.p.refused_asleep() == 1);
  r.p.suspend(false);
  // no sounds in the image, or no output
  FakeClock c2;
  CaptureLog l2;
  SoundPlayer none(c2, l2, 1);
  CHECK(none.request("meow", "test") == KILO_SOUND_RESULT_NO_SOUNDS && none.refused_none() == 1);
  none.poll();
  kilo_host_sound_status_t st;
  none.fill(st, KILO_SOUND_RESULT_OK);
  CHECK(st.sounds == 0 && st.state == KILO_SOUND_STATE_NONE && st.dropped == 1 && st.enabled == 1 && st.volume == 70);
  r.p.set_volume(150);
  CHECK(r.p.volume() == 100);
}

void test_player_queue_gap_and_stale() {
  PlayerRig r;
  CHECK(r.p.request("purr", "a") == KILO_SOUND_RESULT_QUEUED);  // 1.5 s
  r.run_ms(100);
  CHECK(r.p.request("hiss", "b") == KILO_SOUND_RESULT_QUEUED);  // waits
  CHECK(r.p.request("trill", "c") == KILO_SOUND_RESULT_QUEUED);  // replaces it: newest wins
  CHECK(r.p.replaced() == 1 && std::strcmp(r.p.last()->name, "purr") == 0);
  // the purr ends at about 1.58 s; the trill waited more than kStaleUs by then: dropped, not played late
  r.run_ms(3000);
  CHECK(r.p.stale() == 1 && r.p.played() == 1);
  // the gap: a request right after a sound ends waits kGapUs
  CHECK(r.p.request("mew", "d") == KILO_SOUND_RESULT_QUEUED && r.p.played() == 2);
  int ms = 0;
  while (r.p.playing()) {
    r.run_ms(4);
    ms += 4;
  }
  CHECK(r.p.request("mew", "e") == KILO_SOUND_RESULT_QUEUED && r.p.played() == 2);
  int wait = 0;
  while (r.p.played() == 2 && wait < 3000) {
    r.run_ms(4);
    wait += 4;
  }
  CHECKF(wait >= 990 && wait <= 1020, "the next sound %d ms after the last ended", wait);
  // B2 held down as fast as one can press: at most one sound per (length + gap)
  PlayerRig q;
  for (int i = 0; i < 100; i++) {  // a press every 100 ms for 10 s
    q.p.request("meow", "key:B2");
    q.run_ms(100);
  }
  CHECKF(q.p.played() <= 10 && q.p.played() >= 5, "%u meows in 10 s", (unsigned)q.p.played());
  char line[256];
  q.p.metrics_line(line, 201);
  CHECKF(std::strlen(line) <= 200 && std::strncmp(line, "sound: on=1 volume=70 sounds=7 ", 31) == 0, "%s", line);
}

// ---- the behaviour's chatter ------------------------------------------------------------------

void test_chatter() {
  Chatter c(3);
  double t = 0;
  std::vector<std::pair<double, std::string>> said;
  for (; t < 3600; t += 0.1)
    if (const char* w = c.poll(t, Chatter::Mode::Sitting, false)) said.push_back({t, w});
  int purrs = 0;
  for (auto& s : said) purrs += s.second == "purr";
  CHECKF(said.size() >= 20 && said.size() <= 80, "%zu sounds in an hour sitting", said.size());
  CHECKF(purrs * 2 > (int)said.size(), "%d purrs of %zu", purrs, said.size());
  CHECK(said.front().first >= Chatter::kFirstMinS);
  for (size_t i = 1; i < said.size(); i++) CHECK(said[i].first - said[i - 1].first >= Chatter::kSitMinS - 0.2);
  // none, and nothing while quiet
  Chatter d(3);
  int n = 0;
  for (t = 0; t < 600; t += 0.1) n += d.poll(t, Chatter::Mode::None, false) != nullptr;
  for (t = 600; t < 1200; t += 0.1) n += d.poll(t, Chatter::Mode::Sitting, true) != nullptr;
  CHECK(n == 0);
  // standing: rarer
  Chatter e(3);
  int st = 0;
  for (t = 0; t < 3600; t += 0.1) st += e.poll(t, Chatter::Mode::Standing, false) != nullptr;
  CHECKF(st >= 3 && st < (int)said.size(), "%d sounds in an hour standing", st);
  // seeded
  Chatter f(3);
  std::vector<std::pair<double, std::string>> again;
  for (t = 0; t < 3600; t += 0.1)
    if (const char* w = f.poll(t, Chatter::Mode::Sitting, false)) again.push_back({t, w});
  CHECK(again == said);
}

// ---- the Mind's hooks -------------------------------------------------------------------------

constexpr uint8_t kA7 = 0xA2, kB6 = 0x02, kA4 = 0x68, kB2 = 0x38, kA5 = 0xE0;

struct Rig {
  FakeBody body;
  CaptureLog log;
  Bank bank = Bank::cat();
  TimedOutput out{body.clock};
  Mind mind;
  std::vector<uint8_t> mem;
  explicit Rig(MindConfig cfg, bool with_sounds = true) : mind(body.serial, body.clock, log, nullptr, prep(cfg, with_sounds)) {
    mem.resize(4u << 20);
    mind.attach_recorder(mem.data(), (uint32_t)mem.size());
    mind.start();
    run_ms(200);
  }
  MindConfig prep(MindConfig c, bool with_sounds) {
    if (with_sounds) {
      c.sounds = bank.sounds.data();
      c.sound_count = (uint32_t)bank.sounds.size();
      c.sound_output = &out;
    }
    return c;
  }
  static MindConfig remote_cfg() {
    MindConfig c;
    c.remote = true;
    c.light_sleep = true;
    c.seed = 11;
    return c;
  }
  void pass() {
    body.step();
    mind.poll();
    body.react();
  }
  void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) pass();
  }
  void key(uint8_t cmd) {
    body.event(KILO_EVENT_IR_KEY, cmd);
    run_ms(40);
  }
  int played(const char* why) const {
    int n = 0;
    std::string w = std::string("why=") + why + " ";
    for (const std::string& l : log.lines) n += l.find("sound play name=") == 0 && l.find(w) != std::string::npos;
    return n;
  }
  // A MARK with this text in the flight recorder's RAM.
  bool marked(const char* text) {
    const record::FlightStore& f = mind.recorder().flight();
    for (uint32_t s = f.first_seq(); !f.empty() && s <= f.last_seq(); s++) {
      std::string b((const char*)f.block(s), f.used(s));
      if (b.find(text) != std::string::npos) return true;
    }
    return false;
  }
};

void test_mind_ready_and_b2() {
  Rig r(Rig::remote_cfg());
  CHECK(r.played("ready") == 1 && r.log.has("sound play name=mew asked=mew why=ready "));
  CHECK(r.log.has("sound: 7 sounds (3.9 s) in the image, on, volume 70"));
  CHECK(r.marked("sound play name=mew asked=mew why=ready"));
  r.run_ms(2000);
  r.key(kB2);
  CHECK(r.log.has("remote B2 (0x38, meow): meow queued"));
  r.run_ms(1500);
  CHECK(r.played("key:B2") == 1 && r.marked("why=key:B2"));
  CHECK(r.mind.metrics().keys_acted >= 1);
  // the remote's own state is untouched: no walk, no posture
  CHECK(!r.mind.walker().active() && !r.mind.remote().busy());
  // the metrics line, after the power lines
  uint32_t n = r.mind.metric_lines();
  CHECK(n == 11 + power::Sleep::kMetricLines + 1);
  char line[256];
  r.mind.metric_line(n - 1, line, 201);
  CHECKF(std::strncmp(line, "sound: on=1 volume=70 sounds=7 ", 31) == 0 && std::strstr(line, " played=2 ") != nullptr,
         "%s", line);
  r.mind.metric_line(11, line, 201);
  CHECK(std::strncmp(line, "power: ", 7) == 0);
  // a laptop holding the body: B2 is ignored like every key
  // no sounds in this image (KILO_SOUND=0, or none packed): B2 says so, nothing breaks
  Rig q(Rig::remote_cfg(), false);
  q.key(kB2);
  CHECK(q.log.has("remote B2 (0x38, meow): meow no sounds in this image") && q.mind.sound().refused_none() == 2);
  CHECK(q.log.has("sound: no sounds in this image (no output)"));
}

void test_mind_light_sleep() {
  Rig r(Rig::remote_cfg());
  r.run_ms(1500);
  r.key(kA7);
  r.run_ms(31000);
  CHECK(r.mind.sleep().asleep() && r.mind.sound().asleep());
  r.run_ms(200);
  OutputStats st;
  r.out.stats(st);
  CHECK(st.state == KILO_SOUND_STATE_ASLEEP && r.out.renderer().settled_off());
  // B2 while asleep wakes him (as any key): the wake's mew, not B2's meow (asked while the output slept)
  r.key(kB2);
  r.run_ms(1500);
  CHECK(!r.mind.sleep().asleep() && !r.mind.sound().asleep());
  CHECK(r.played("wake") == 1 && r.played("key:B2") == 0 && r.mind.sound().refused_asleep() == 1);
  // asleep again, a moving key wakes him: mew
  r.run_ms(31000);
  CHECK(r.mind.sleep().asleep());
  r.key(kB6);
  r.run_ms(500);
  CHECK(r.played("wake") == 2);
}

void test_mind_guard_hiss() {
  MindConfig c = Rig::remote_cfg();
  Rig r(c);
  r.run_ms(1500);
  r.key(kA4);  // trot forward
  r.run_ms(1500);
  CHECK(r.mind.walker().active());
  r.body.dist_mm = 120;  // very close: RED
  r.body.flags |= KILO_TELEMETRY_FLAGS_DIST_FRESH;
  r.run_ms(1000);
  CHECKF(r.mind.guard().pullbacks() == 1, "pullbacks %u", (unsigned)r.mind.guard().pullbacks());
  r.run_ms(800);
  CHECK(r.played("guard red") == 1 && r.marked("name=hiss asked=hiss why=guard red"));
  r.run_ms(8000);
  CHECK(r.played("guard red") == 1);  // once per event
}

void test_mind_behaviour_purrs() {
  MindConfig c = Rig::remote_cfg();
  c.light_sleep = false;
  Rig r(c);
  r.run_ms(1500);
  r.key(kA5);  // the behaviour runner: idles, stands or sits
  int said = 0;
  for (int s = 0; s < 400 && said == 0; s++) {
    r.run_ms(1000);
    said = r.played("behaviour");
  }
  CHECKF(said >= 1, "the behaviour said nothing in 400 s");
  // B6 stops the runner: no more
  r.key(kB6);
  int before = r.played("behaviour");
  r.run_ms(200000);
  CHECK(r.played("behaviour") == before);
}

}  // namespace

int main() {
  test_ulaw_and_names();
  test_renderer_ramp_and_silence();
  test_renderer_clip_fades_and_volume();
  test_renderer_waits_for_the_level();
  test_timed_output();
  test_player_families_and_results();
  test_player_queue_gap_and_stale();
  test_chatter();
  test_mind_ready_and_b2();
  test_mind_light_sleep();
  test_mind_guard_hiss();
  test_mind_behaviour_purrs();
  return report("sound");
}
