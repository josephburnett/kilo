// The trick model, the compiled-in library, the keys a trick becomes and the
// player streaming them to a scripted body (tricks/player.h). The library's
// CRC, trick_id and duration are checked against what Python computed
// (kLibraryCheck, from gen_library.py); the guard cases follow
// mind/tests/test_rear_up.py.
#include <cstdio>
#include <cstring>
#include <functional>
#include <vector>
#include "link/serial_link.h"
#include "tests/body_rig.h"
#include "tests/check.h"
#include "tricks/library.h"
#include "tricks/player.h"

using namespace kilo;
using namespace kilo::tricks;

namespace {

// The keys of a trick in play order: the last one's `at` is the trick's length.
uint32_t key_ticks(const Trick& t, uint8_t speed, std::vector<TrickKey>* out = nullptr) {
  KeySeq s(t, speed);
  TrickKey k;
  uint32_t n = 0, last = 0;
  while (s.next(&k) && n++ < 100000) {
    if (out != nullptr) out->push_back(k);
    last = k.at;
  }
  return last;
}

void test_library_matches_python() {
  // the 13 first-version tricks, the authored rear_up's three stages, the random mode's 9 (standing, the face
  // wash is the engine's: engine/paw.h)
  CHECK(kLibrarySize == 25);
  for (uint32_t i = 0; i < kLibrarySize; i++) {
    const Trick& t = kLibrary[i];
    const char* p = trick_problem(t);
    CHECKF(p == nullptr, "%s: %s", t.name, p ? p : "");
    CHECKF(trick_crc16(t) == kLibraryCheck[i].crc16, "%s crc %04x", t.name, trick_crc16(t));
    CHECKF(trick_id_for(t) == kLibraryCheck[i].trick_id, "%s id %u", t.name, trick_id_for(t));
    uint32_t ticks = 0;
    CHECK(total_ticks(t, kSpeedUnity, &ticks) && ticks == kLibraryCheck[i].total_ticks);
    for (uint8_t speed : {32, 64, 96}) {  // the keys end where the model's duration says
      uint32_t want = 0;
      total_ticks(t, speed, &want);
      CHECKF(key_ticks(t, speed) == want, "%s at %u: %u vs %u", t.name, speed, key_ticks(t, speed), want);
    }
  }
  CHECK(find_trick("stand") == find_trick("up") && find_trick("up") != nullptr);
  CHECK(find_trick("wsf") != nullptr && std::strcmp(find_trick("wsf")->name, "wash_face") == 0);
  CHECK(find_trick("nope") == nullptr);
  const Trick* ru = find_trick("rear_up");  // authored: no legacy token, a slow stop, SIT at the end
  CHECK(ru != nullptr && ru->legacy[0] == '\0' && ru->stop_ticks == 75 && ru->end_posture == KILO_POSTURE_SIT);
  CHECK(ru->flags == 0 && find_trick("") == nullptr && find_trick("stand")->stop_ticks == 0);
  const uint8_t stage_stop[2] = {100, 125};  // rear_up_2 (40 deg), rear_up_3 (54 deg): further to come down
  const char* const stage_name[2] = {"rear_up_2", "rear_up_3"};
  for (int i = 0; i < 2; i++) {
    const Trick* st = find_trick(stage_name[i]);
    CHECKF(st != nullptr && st->legacy[0] == '\0' && st->stop_ticks == stage_stop[i] &&
               st->end_posture == KILO_POSTURE_SIT && st->flags == 0 && st->n_frames > ru->n_frames,
           "%s", stage_name[i]);
  }
  CHECK(ru->n_guards == 3 && ru->guards != nullptr && std::strcmp(ru->guards[0].name, "back") == 0 &&
        std::strcmp(ru->guards[2].name, "up") == 0 &&
        ru->roll_max_a16 == 8 * 16 && find_trick("stand")->n_guards == 0);
  const Trick* s = find_trick("stretch");  // the expected attitude: the balance's reference in its keys
  CHECK(s->exp_roll == 0 && s->exp_pitch == 15);
}

// Every leg set in frame 0 (the stream drives every leg), pan moving.
#define LEGS 30, 30, -30, -30, 30, 30, -30, -30
const tricks::Keyframe kFrames4[] = {
    {2, 0, 1, {0, -128, -128, LEGS}},
    {3, 1, 1, {5, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128}},
    {4, 0, 1, {10, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128}},
    {5, 2, 1, {0, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128}},
};

void test_timing() {
  const uint8_t speeds[6] = {16, 32, 64, 96, 128, 255};
  const uint32_t want[6] = {40, 20, 10, 7, 5, 3};
  for (int i = 0; i < 6; i++) CHECK(phase_ticks(10, speeds[i]) == want[i]);
  CHECK(phase_ticks(0, 64) == 0);
  // play_order(4, 1, 2, 3) == [0, 1, 2, 1, 2, 1, 2, 3]: 2 + 3x(4 + 4) + 7 ticks
  Trick t = {"t", "t", 4, 1, 2, 3, 0xFF01, 0, KILO_POSTURE_STAND, 0, 0, kFrames4, 0};
  uint32_t n = 0;
  CHECK(trick_problem(t) == nullptr);
  std::vector<TrickKey> keys;
  CHECK(total_ticks(t, 64, &n) && n == 2 + 3 * 8 + 7 && key_ticks(t, 64, &keys) == n);
  // the keys: frame 0's move; frame 1's move and hold, frame 2's move, three times; frame 3's move and hold
  const uint32_t at[] = {2, 5, 6, 10, 13, 14, 18, 21, 22, 26, 31, 33};
  const uint8_t fr[] = {0, 1, 1, 2, 1, 1, 2, 1, 1, 2, 3, 3};
  bool same = keys.size() == sizeof at / sizeof at[0];
  for (size_t i = 0; same && i < keys.size(); i++)
    same = keys[i].at == at[i] && keys[i].frame == fr[i] && keys[i].hold == (i == 2 || i == 5 || i == 8 || i == 11) &&
           keys[i].ease == (keys[i].hold ? kLinear : kCosine);
  CHECK(same);
  t.repeats = 1;  // 0 and 1: once
  CHECK(total_ticks(t, 64, &n) && n == 2 + 8 + 7 && key_ticks(t, 64) == n);
  t.repeats = 0;
  CHECK(total_ticks(t, 64, &n) && n == 2 + 8 + 7 && key_ticks(t, 64) == n);
  // a step: the jump one tick in, then the rest of the move as a hold; linear: no EASE
  tricks::Keyframe st[2] = {{2, 0, 1, {0, -128, -128, LEGS}}, {6, 0, kStep, {9, -128, -128, LEGS}}};
  Trick s = {"s", "s", 2, 0, 0, 0, 0xFF01, 0, KILO_POSTURE_STAND, 0, 0, st, 0};
  keys.clear();
  CHECK(key_ticks(s, 64, &keys) == 8 && keys.size() == 3 && keys[1].at == 3 && keys[1].ease == kLinear &&
        !keys[1].hold && keys[2].at == 8 && keys[2].hold);
  st[1].flags = kLinear;
  keys.clear();
  CHECK(key_ticks(s, 64, &keys) == 8 && keys.size() == 2 && keys[1].ease == kLinear);
  // the angles: "unchanged" is the frame before's by index; mirrored: legs L/R, pan and tail negated
  CHECK(trick_angle(t, 2, 0, false) == 10 * 16 && trick_angle(t, 2, 3, false) == 30 * 16 &&
        trick_angle(t, 2, 0, true) == -10 * 16 && trick_angle(t, 0, 5, true) == -30 * 16 &&
        trick_angle(t, 3, 1, false) == KILO_HEAD_NOT_HELD);
  // test_player_forever: a loop until stopped has no end
  Trick f = {"f", "f", 2, 0, 1, kRepeatForever, 0xFF01, 0, KILO_POSTURE_STAND, 0, 0, kFrames4, 0};
  CHECK(!total_ticks(f, 64, &n));
  KeySeq q(f);
  TrickKey k;
  int got = 0;
  for (int i = 0; i < 5000; i++) got += q.next(&k) ? 1 : 0;
  CHECK(got == 5000);
  // validation
  Trick bad = t;
  bad.loop_end = 4;
  CHECK(trick_problem(bad) != nullptr);
  bad = t;
  bad.joint_mask = 0;
  CHECK(trick_problem(bad) != nullptr);
  bad = t;
  bad.n_frames = 25;
  CHECK(trick_problem(bad) != nullptr);
  bad = t;
  bad.joint_mask = 0x0001;  // the legs not driven
  CHECK(trick_problem(bad) != nullptr && std::strstr(trick_problem(bad), "leg") != nullptr);
  tricks::Keyframe over[1] = {{2, 0, 1, {-128, 50, -128, LEGS}}};
  Trick o = {"o", "o", 1, 0, 0, 0, 0xFF02, 0, KILO_POSTURE_STAND, 0, 0, over, 0};
  CHECK(trick_problem(o) != nullptr);  // head tilt 50 > 45
  over[0].ang[1] = 45;
  CHECK(trick_problem(o) == nullptr);
  over[0].flags = 3;
  CHECK(trick_problem(o) != nullptr);  // easing 3 reserved
  over[0].flags = 1;
  over[0].ang[4] = -128;  // frame 0 leaves a leg joint unchanged
  CHECK(trick_problem(o) != nullptr);
  CHECK(speed_q6(1.0) == 64 && speed_q6(0.25) == 16 && speed_q6(0.2) == 0 && speed_q6(3.98) == 255 &&
        speed_q6(4.0) == 0);
}

// ---- the player on a link --------------------------------------------------------------

struct Rig : LinkListener {
  FakeBody body;
  CaptureLog log;
  SerialLink link{body.serial, body.clock, log};
  TrickPlayer player{link, body.clock, log};
  int max_queued = 0;  // the most keys ahead of the body's tick at once

  Rig() {
    link.set_listener(this);
    link.connect();
    run_ms(60);
  }
  void on_telemetry(const kilo_telemetry_t& t) override { player.on_telemetry(t); }
  void on_event(const kilo_event_t& e) override { player.on_event(e); }
  void pass() {
    body.step();
    link.mark_alive();
    link.poll();
    player.poll();
    body.react();
    int ahead = 0;
    for (const kilo_pose_t& k : keys())
      if ((int16_t)(k.tick - body.tick) >= 0) ahead++;
    if (ahead > max_queued) max_queued = ahead;
  }
  void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) pass();
  }
  bool run_until(std::function<bool()> pred, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) {
      if (pred()) return true;
      pass();
    }
    return pred();
  }
  std::vector<kilo_pose_t> keys() const {
    std::vector<kilo_pose_t> out;
    for (const Sent& s : body.of(KILO_MSG_POSE)) {
      kilo_pose_t k;
      std::memcpy(&k, s.payload.data(), sizeof k);
      out.push_back(k);
    }
    return out;
  }
  // The SET_POSTURE goals sent: posture and transition.
  std::vector<std::pair<uint8_t, uint8_t>> postures() const {
    std::vector<std::pair<uint8_t, uint8_t>> out;
    for (const Sent& s : body.of(KILO_MSG_SET_POSTURE)) out.push_back({s.payload[1], s.payload[2]});
    return out;
  }
};

void test_plays_through_the_stream_to_done() {
  Rig r;
  const Trick* hi = find_trick("hi");
  uint8_t sit[4] = {0, KILO_POSTURE_SIT, 0, 0};
  r.link.set_goal(KILO_MSG_SET_POSTURE, sit, sizeof sit);
  r.run_ms(40);
  uint32_t play = r.player.start(*hi);
  CHECK(play == 1 && r.player.busy());
  r.body.sent.clear();
  CHECK(r.run_until([&] { return r.player.playing(); }, 100));
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) == nullptr);  // a refresh would end the stream
  r.run_ms(8);
  std::vector<kilo_pose_t> k = r.keys();
  CHECK(!k.empty());
  // frame 0 as joint angles: every leg, the head (hi drives it), eased, no balance, the epoch
  const uint8_t all = KILO_POSE_FLAGS_JOINT_LF | KILO_POSE_FLAGS_JOINT_RF | KILO_POSE_FLAGS_JOINT_RB |
                      KILO_POSE_FLAGS_JOINT_LB;
  CHECK(k[0].epoch == 4 && k[0].flags == (all | KILO_POSE_FLAGS_EASE) && k[0].support == 0);
  CHECK(k[0].head[0] == hi->frames[0].ang[0] * 16 && k[0].head[2] == hi->frames[0].ang[2] * 16 &&
        k[0].leg[0] == hi->frames[0].ang[3] * 16 && k[0].leg[1] == hi->frames[0].ang[7] * 16 &&
        k[0].leg[6] == hi->frames[0].ang[6] * 16 && k[0].leg[7] == hi->frames[0].ang[10] * 16);
  // the first key at its dur from the body's next tick
  CHECKF((int16_t)(k[0].tick - r.body.tick) >= (int)hi->frames[0].dur - 3, "key %u body %u", k[0].tick, r.body.tick);
  // the rest go out as the body comes near the one before: never more than the body's queue
  bool done = r.run_until([&] { return !r.player.busy(); }, 6000);
  CHECK(done && r.player.result().state == PlayState::Done && r.player.result().reason == tricks::kDone &&
        r.player.result().play == play);
  std::vector<TrickKey> want;
  uint32_t total = key_ticks(*hi, 64, &want);
  k = r.keys();
  CHECKF(k.size() == want.size() && r.max_queued <= (int)KILO_POSE_DEPTH, "%zu keys of %zu, %d ahead at most",
         k.size(), want.size(), r.max_queued);
  bool spaced = true;
  for (size_t i = 1; i < k.size(); i++) spaced = spaced && (uint16_t)(k[i].tick - k[0].tick) == want[i].at - want[0].at;
  CHECK(spaced);
  // the end: the end posture over XFADE_TICKS ended the stream, the goal cleared again
  auto p = r.postures();
  CHECK(!p.empty() && p.back().first == hi->end_posture && p.back().second == KILO_PARAM_XFADE_TICKS_DEFAULT);
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) == nullptr && r.body.mode == KILO_MODE_IDLE);
  uint32_t ms = r.player.result().elapsed_ms;
  CHECKF(ms + 40 >= total * 20 && ms <= total * 20 + 120, "elapsed %u ms for %u ticks", ms, total);
  CHECK(r.log.has("trick hi: DONE after"));
  // again, mirrored and faster: legs L/R, pan negated, phases at 1.5x
  r.body.sent.clear();
  CHECK(r.player.start(*hi, true, 96) == 2);
  CHECK(r.player.start(*hi) == 0);  // still playing
  r.run_ms(200);
  k = r.keys();
  CHECKF(k.size() >= 2 && k[0].head[0] == -hi->frames[0].ang[0] * 16 && k[0].leg[0] == hi->frames[0].ang[4] * 16 &&
        (uint16_t)(k[1].tick - k[0].tick) == phase_ticks(hi->frames[0].hold, 96), "%zu keys, head %d leg %d dt %u", k.size(), k.size() ? k[0].head[0] : 0, k.size() ? k[0].leg[0] : 0, k.size() > 1 ? (unsigned)(uint16_t)(k[1].tick - k[0].tick) : 0u);
  CHECK(r.run_until([&] { return !r.player.busy(); }, 6000) && r.player.result().reason == tricks::kDone);
}

void test_stop_and_endings() {
  // stop(): the end posture over the trick's stop_ticks, then STOPPED once the body has left the stream
  Rig r;
  const Trick* ru = find_trick("rear_up");
  r.player.start(*ru);
  r.run_ms(300);
  CHECK(r.player.playing());
  r.player.stop();
  r.run_ms(100);
  auto p = r.postures();
  CHECK(!p.empty() && p.back().first == KILO_POSTURE_SIT && p.back().second == ru->stop_ticks);
  CHECK(r.player.result().state == PlayState::Done && r.player.result().reason == tricks::kStopped);
  size_t n = r.keys().size();
  r.run_ms(200);
  CHECK(r.keys().size() == n);  // no key after the stop
  // a fall: FELL, no posture (the reflex holds the joints)
  r.player.start(*find_trick("hi"));
  r.run_ms(200);
  size_t np = r.postures().size();
  r.body.mode = KILO_MODE_FALLEN;
  r.body.event(KILO_EVENT_FELL, 1);
  r.run_ms(40);
  CHECK(r.player.result().state == PlayState::Done && r.player.result().reason == tricks::kFell &&
        r.postures().size() == np);
  r.body.mode = KILO_MODE_IDLE;
  // someone else's posture ended the stream: PREEMPTED
  r.player.start(*find_trick("hi"));
  r.run_ms(200);
  r.body.event(KILO_EVENT_STREAM_STOPPED, 0);
  r.run_ms(40);
  CHECK(r.player.result().state == PlayState::Done && r.player.result().reason == tricks::kPreempted);
  // POSE refused: Failed with the reason
  r.player.start(*find_trick("hi"));
  r.run_ms(200);
  r.body.event(KILO_EVENT_GOAL_REFUSED, (int16_t)((KILO_MSG_POSE << 8) | KILO_GOAL_REFUSED_REASON_LOW_BATT));
  r.run_ms(40);
  CHECK(r.player.result().state == PlayState::Failed && std::strstr(r.player.result().error, "LOW_BATT") != nullptr);
}

void test_the_body_never_gets_there() {
  // the body's clock stops (no TELEMETRY): Failed kDoneMarginUs after the last key's tick
  Rig r;
  const Trick* sit = find_trick("sit");  // 25 ticks
  r.player.start(*sit);
  r.run_ms(60);
  CHECK(r.player.playing());
  r.body.telemetry_on = false;
  r.run_ms(1400);
  CHECK(r.player.result().state == PlayState::Running);
  r.run_ms(300);
  CHECK(r.player.result().state == PlayState::Failed && r.log.has("the body did not reach the last key within 1000 ms"));
  // a body never heard: no key, Failed after kStartWaitUs
  Rig q;
  q.body.telemetry_on = false;
  q.link.disconnect();
  q.link.connect();
  q.player.start(*sit);
  q.run_ms(2100);
  CHECK(q.player.result().state == PlayState::Failed && q.keys().empty());
  // stop() while starting: Failed "stopped", nothing sent
  Rig s;
  s.body.telemetry_on = false;
  s.link.disconnect();
  s.link.connect();
  s.player.start(*sit);
  s.run_ms(8);
  s.player.stop();
  CHECK(s.player.result().state == PlayState::Failed && !s.player.busy() && s.keys().empty());
}

}  // namespace

// test_rear_up.py test_guard_watch, the same cases (TELEMETRY: pitch nose down +, a16)
const tricks::Keyframe kGuardFrames[] = {
    {10, 30, 1, {-128, -128, 0, -128, -128, -128, -128, -128, -128, -128, -128}},
    {10, 30, 1, {-128, -128, 0, -128, -128, -128, -128, -128, -128, -128, -128}},
    {10, 30, 1, {-128, -128, 0, -128, -128, -128, -128, -128, -128, -128, -128}},
};
const TrickGuard kGuardMid[] = {{1, 10 * 16, 14 * 16, 3 * 16, "mid"}};

const char* feed_until(GuardWatch& w, int n, double (*nose)(double), double (*roll)(double), uint64_t* at,
                       bool (*skip)(double) = nullptr) {
  for (int i = 0; i < n; i++) {
    double t = i * 0.04;
    if (skip != nullptr && skip(t)) continue;
    const char* why = w.feed((uint64_t)(i * 40000), (int16_t)(-nose(t) * 16), (int16_t)(roll(t) * 16));
    if (why != nullptr) {
      *at = (uint64_t)(i * 40000);
      return why;
    }
  }
  return nullptr;
}

void test_guard_watch() {
  Trick t = {"g", "", 3, 0, 0, 0, 0x0004, 0, KILO_POSTURE_STAND, 0, 0, kGuardFrames, 0, kGuardMid, 1, 8 * 16};
  uint64_t at = 0;
  GuardWatch w;
  w.reset(t, kSpeedUnity);
  CHECK(w.decide_us(0) == 78u * 20000u);
  CHECK(feed_until(w, 60, [](double) { return 12.0; }, [](double) { return 0.5; }, &at) == nullptr);
  w.reset(t, kSpeedUnity);
  const char* why = feed_until(w, 60, [](double x) { return x > 1.0 ? 8.0 : 12.0; }, [](double) { return 0.5; }, &at);
  CHECKF(why != nullptr && std::strncmp(why, "guard mid: nose up 8.0", 22) == 0 && at > 1560000 && at < 1640000,
         "%s at %u", why ? why : "-", (unsigned)at);
  w.reset(t, kSpeedUnity);
  why = feed_until(w, 60, [](double) { return 12.0; }, [](double x) { return x > 1.4 ? 4.0 : 0.0; }, &at);
  CHECKF(why != nullptr && std::strcmp(why, "guard mid: roll 4.0 deg past 3.0") == 0, "%s", why ? why : "-");
  w.reset(t, kSpeedUnity);
  why = feed_until(w, 60, [](double) { return 12.0; }, [](double x) { return x >= 0.2 ? 9.0 : 0.0; }, &at);
  CHECKF(why != nullptr && std::strcmp(why, "roll 9.0 deg past 8.0") == 0 && at == 240000, "%s", why ? why : "-");
  w.reset(t, kSpeedUnity);
  why = feed_until(w, 60, [](double) { return 12.0; }, [](double) { return 0.0; }, &at,
                   [](double x) { return x > 1.25 && x < 1.6; });
  CHECK(why != nullptr && std::strstr(why, "no TELEMETRY") != nullptr);
  w.reset(t, 32);  // half speed: the hold comes twice as late
  CHECK(feed_until(w, 120, [](double) { return 12.0; }, [](double) { return 0.0; }, &at) == nullptr);
}

// test_rear_up.py test_guard_watch_push_roll, the same cases
const TrickGuard kGuardPush[] = {{1, 10 * 16, 14 * 16, 3 * 16, "mid", 2 * 16}};

void test_guard_watch_push_roll() {
  Trick t = {"g", "", 3, 0, 0, 0, 0x0004, 0, KILO_POSTURE_STAND, 0, 0, kGuardFrames, 0, kGuardPush, 1, 8 * 16};
  uint64_t at = 0;
  GuardWatch w;
  w.reset(t, kSpeedUnity);
  const char* why = feed_until(w, 60, [](double) { return 12.0; }, [](double x) { return x < 0.9 ? 2.0 : 5.5; }, &at);
  CHECKF(why != nullptr && std::strcmp(why, "guard mid: roll 5.5 deg, 3.5 while it moved (limit 2.0)") == 0 &&
             at == 960000,
         "%s at %u", why ? why : "-", (unsigned)at);
  w.reset(t, kSpeedUnity);
  CHECK(feed_until(w, 60, [](double) { return 12.0; }, [](double x) { return x < 0.8 ? 1.0 : 2.5; }, &at) == nullptr);
  w.reset(t, kSpeedUnity);
  CHECK(feed_until(w, 60, [](double) { return 12.0; }, [](double x) { return x > 0.99 && x < 1.01 ? 4.5 : 0.0; },
                   &at) == nullptr);
}

int main() {
  test_library_matches_python();
  test_timing();
  test_plays_through_the_stream_to_done();
  test_stop_and_endings();
  test_the_body_never_gets_there();
  test_guard_watch();
  test_guard_watch_push_roll();
  return report("tricks");
}
