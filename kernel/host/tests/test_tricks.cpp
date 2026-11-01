// The trick model, the compiled-in library and the player on a link to a
// scripted body. Cases follow mind/tests/test_trick_player.py and
// test_tricks.py; the library's CRC, trick_id and duration are checked
// against what Python computed (kLibraryCheck, from gen_library.py).
#include <cstdio>
#include <cstring>
#include "link/serial_link.h"
#include "proto/crc16.h"
#include "tests/body_rig.h"
#include "tests/check.h"
#include "tricks/library.h"
#include "tricks/player.h"

using namespace kilo;
using namespace kilo::tricks;

namespace {

uint32_t timer_ticks(const Trick& t, uint8_t speed) {
  KeyframeTimer p(t, speed);
  uint32_t n = 0;
  while (!p.step() && n < 100000) n++;
  return n;
}

void test_library_matches_python() {
  CHECK(kLibrarySize == 13);
  for (uint32_t i = 0; i < kLibrarySize; i++) {
    const Trick& t = kLibrary[i];
    const char* p = trick_problem(t);
    CHECKF(p == nullptr, "%s: %s", t.name, p ? p : "");
    CHECKF(trick_crc16(t) == kLibraryCheck[i].crc16, "%s crc %04x", t.name, trick_crc16(t));
    CHECKF(trick_id_for(t) == kLibraryCheck[i].trick_id, "%s id %u", t.name, trick_id_for(t));
    uint32_t ticks = 0;
    CHECK(total_ticks(t, kSpeedUnity, &ticks) && ticks == kLibraryCheck[i].total_ticks);
    for (uint8_t speed : {32, 64, 96}) {  // test_player_matches_the_model_duration
      uint32_t want = 0;
      total_ticks(t, speed, &want);
      CHECKF(timer_ticks(t, speed) == want, "%s at %u: %u vs %u", t.name, speed, timer_ticks(t, speed), want);
    }
  }
  CHECK(find_trick("stand") == find_trick("up") && find_trick("up") != nullptr);
  CHECK(find_trick("wsf") != nullptr && std::strcmp(find_trick("wsf")->name, "wash_face") == 0);
  CHECK(find_trick("nope") == nullptr);
  const Trick* s = find_trick("stretch");  // test_trick_id_and_header_carry_expected_attitude
  kilo_trick_begin_t b;
  trick_begin(*s, 9, &b);
  CHECK(b.exp_roll == 0 && b.exp_pitch == 15 && b.trick_id == 9 && sizeof b == 11);
}

const kilo_keyframe_t kFrames4[] = {
    {2, 0, 1, {0, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128}},
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
  Trick t = {"t", "t", 4, 1, 2, 3, 0x0001, 0, KILO_POSTURE_STAND, 0, 0, kFrames4};
  uint32_t n = 0;
  CHECK(trick_problem(t) == nullptr);
  CHECK(total_ticks(t, 64, &n) && n == 2 + 3 * 8 + 7 && timer_ticks(t, 64) == n);
  t.repeats = 1;  // 0 and 1: once
  CHECK(total_ticks(t, 64, &n) && n == 2 + 8 + 7);
  t.repeats = 0;
  CHECK(total_ticks(t, 64, &n) && n == 2 + 8 + 7 && timer_ticks(t, 64) == n);
  // test_player_forever: Player([2, 3], [0, 1], 0, 1, 255)
  Trick f = {"f", "f", 2, 0, 1, kRepeatForever, 0x0001, 0, KILO_POSTURE_STAND, 0, 0, kFrames4 + 1};
  CHECK(!total_ticks(f, 64, &n));
  KeyframeTimer p(f);
  bool ended = false;
  for (int i = 0; i < 5000; i++) ended = ended || p.step();
  CHECK(!ended && p.running());
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
  kilo_keyframe_t over[1] = {{2, 0, 1, {-128, 50, -128, -128, -128, -128, -128, -128, -128, -128, -128}}};
  Trick o = {"o", "o", 1, 0, 0, 0, 0x0002, 0, KILO_POSTURE_STAND, 0, 0, over};
  CHECK(trick_problem(o) != nullptr);  // head tilt 50 > 45
  over[0].ang[1] = 45;
  CHECK(trick_problem(o) == nullptr);
  over[0].flags = 3;
  CHECK(trick_problem(o) != nullptr);  // easing 3 reserved
  CHECK(speed_q6(1.0) == 64 && speed_q6(0.25) == 16 && speed_q6(0.2) == 0 && speed_q6(3.98) == 255 &&
        speed_q6(4.0) == 0);
}

// ---- the player on a link --------------------------------------------------------------

struct Rig : LinkListener {
  FakeBody body;
  CaptureLog log;
  SerialLink link{body.serial, body.clock, log};
  TrickPlayer player{link, body.clock, log};

  Rig() {
    link.set_listener(this);
    link.connect();
    run_ms(60);
  }
  void on_event(const kilo_event_t& e) override { player.on_event(e); }
  void pass() {
    body.step();
    link.mark_alive();
    link.poll();
    player.collect();
    player.poll();
    body.react();
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
  std::vector<uint8_t> commands() const {
    std::vector<uint8_t> out;
    for (const Sent& s : body.sent)
      if (s.type >= KILO_MSG_TRICK_BEGIN && s.type <= KILO_MSG_TRICK_STOP) out.push_back(s.type);
    return out;
  }
};

void test_upload_is_cached_and_plays_to_done() {
  Rig r;
  const Trick* hi = find_trick("hi");
  // the body holds nothing: the lone TRICK_COMMIT is refused NO_TRICK
  bool committed = false;
  r.body.answer = [&](const Sent& s) -> uint8_t {
    if (s.type == KILO_MSG_TRICK_COMMIT && !committed) {
      committed = true;
      return KILO_ACK_STATUS_NO_TRICK;
    }
    return KILO_ACK_STATUS_OK;
  };
  uint32_t play = r.player.start(*hi);
  CHECK(play == 1 && r.player.busy());
  r.body.sent.clear();
  CHECK(r.run_until([&] { return r.body.count(KILO_MSG_TRICK_PLAY) == 1; }, 400));
  std::vector<uint8_t> want = {KILO_MSG_TRICK_COMMIT, KILO_MSG_TRICK_BEGIN, KILO_MSG_TRICK_FRAMES,
                               KILO_MSG_TRICK_FRAMES, KILO_MSG_TRICK_COMMIT, KILO_MSG_TRICK_PLAY};
  CHECK(r.commands() == want);
  // what went out is the library trick: header, frames in two chunks, CRC
  std::vector<Sent> begin = r.body.of(KILO_MSG_TRICK_BEGIN);
  kilo_trick_begin_t b;
  std::memcpy(&b, begin[0].payload.data(), sizeof b);
  CHECK(b.n_frames == 6 && b.loop_start == 2 && b.loop_end == 3 && b.repeats == 3 && b.trick_id == trick_id_for(*hi));
  std::vector<uint8_t> frames;
  for (const Sent& s : r.body.of(KILO_MSG_TRICK_FRAMES)) frames.insert(frames.end(), s.payload.begin() + 2, s.payload.end());
  CHECK(frames.size() == 6 * 14 && kilo_crc16(frames.data(), (uint16_t)frames.size()) == trick_crc16(*hi));
  kilo_trick_commit_t c;
  std::memcpy(&c, r.body.of(KILO_MSG_TRICK_COMMIT)[1].payload.data(), sizeof c);
  CHECK(c.crc16 == trick_crc16(*hi));
  kilo_trick_play_t p;
  std::memcpy(&p, r.body.of(KILO_MSG_TRICK_PLAY)[0].payload.data(), sizeof p);
  CHECK(p.epoch == 4 && p.trick_id == trick_id_for(*hi) && p.speed_q6 == 64 && p.flags == 0);
  r.run_ms(40);
  CHECK(r.player.result().state == PlayState::Running && r.player.result().uploaded);
  // TRICK_DONE for another trick does not end it; ours does
  r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)((KILO_TRICK_DONE_REASON_DONE << 8) | (p.trick_id ^ 1)));
  r.run_ms(20);
  CHECK(r.player.result().state == PlayState::Running);
  r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)((KILO_TRICK_DONE_REASON_DONE << 8) | p.trick_id));
  r.run_ms(20);
  CHECK(r.player.result().state == PlayState::Done && r.player.result().reason == KILO_TRICK_DONE_REASON_DONE &&
        r.player.result().play == play && !r.player.busy());
  CHECK(r.log.has("trick hi: TRICK_DONE DONE"));
  // again: cached, TRICK_PLAY alone (mirrored, faster)
  r.body.sent.clear();
  CHECK(r.player.start(*hi, true, 96) == 2);
  r.run_ms(40);
  CHECK(r.commands() == std::vector<uint8_t>{KILO_MSG_TRICK_PLAY});
  std::memcpy(&p, r.body.of(KILO_MSG_TRICK_PLAY)[0].payload.data(), sizeof p);
  CHECK(p.speed_q6 == 96 && p.flags == KILO_TRICK_PLAY_FLAGS_MIRROR);
  CHECK(r.player.start(*hi) == 0);  // still playing
  r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)((KILO_TRICK_DONE_REASON_DONE << 8) | p.trick_id));
  r.run_ms(20);
  // after a reboot the cache is asked again: TRICK_COMMIT OK, no upload
  r.body.epoch = 0;
  r.body.tick = 1;
  r.body.hello();  // unsolicited HELLO: a reboot
  r.run_ms(100);
  CHECK(r.link.state().reboots == 1);
  r.body.sent.clear();
  r.player.start(*hi);
  r.run_ms(40);
  CHECK((r.commands() == std::vector<uint8_t>{KILO_MSG_TRICK_COMMIT, KILO_MSG_TRICK_PLAY}));
}

void test_play_refusals_and_no_trick() {
  Rig r;
  const Trick* sit = find_trick("sit");
  // NO_TRICK on the play: uploaded again once and played again
  int plays = 0;
  r.body.answer = [&](const Sent& s) -> uint8_t {
    if (s.type == KILO_MSG_TRICK_PLAY && plays++ == 0) return KILO_ACK_STATUS_NO_TRICK;
    return KILO_ACK_STATUS_OK;
  };
  r.player.start(*sit);
  r.body.sent.clear();
  r.run_ms(100);
  std::vector<uint8_t> want = {KILO_MSG_TRICK_COMMIT, KILO_MSG_TRICK_PLAY, KILO_MSG_TRICK_BEGIN,
                               KILO_MSG_TRICK_FRAMES, KILO_MSG_TRICK_COMMIT, KILO_MSG_TRICK_PLAY};
  CHECK(r.commands() == want);
  CHECK(r.player.result().state == PlayState::Running);
  r.player.stop();
  r.run_ms(20);
  CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1);
  r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)((KILO_TRICK_DONE_REASON_STOPPED << 8) | trick_id_for(*sit)));
  r.run_ms(20);
  CHECK(r.player.result().state == PlayState::Done && r.player.result().reason == KILO_TRICK_DONE_REASON_STOPPED);
  // BUSY from LIMP: the play fails with the status
  r.body.answer = [&](const Sent& s) -> uint8_t {
    return s.type == KILO_MSG_TRICK_PLAY ? KILO_ACK_STATUS_BUSY : KILO_ACK_STATUS_OK;
  };
  r.player.start(*sit);
  r.run_ms(60);
  CHECK(r.player.result().state == PlayState::Failed && std::strstr(r.player.result().error, "BUSY") != nullptr);
  // the latched posture goal was released for the play
  CHECK(r.link.goal(KILO_MSG_SET_POSTURE) == nullptr);
}

void test_play_timeout_stops_the_trick() {
  Rig r;
  const Trick* sit = find_trick("sit");  // 25 ticks: timeout (25 + 1) x 20 ms + 1 s = 1.52 s
  r.player.start(*sit);
  r.run_ms(60);
  CHECK(r.player.result().state == PlayState::Running);
  r.body.sent.clear();
  r.run_ms(1400);
  CHECK(r.player.result().state == PlayState::Running && r.body.count(KILO_MSG_TRICK_STOP) == 0);
  r.run_ms(200);
  CHECK(r.player.result().state == PlayState::Failed && r.body.count(KILO_MSG_TRICK_STOP) == 1);
  CHECK(r.log.has("no TRICK_DONE within 1520 ms"));
}

void test_stop_while_uploading() {
  Rig r;
  r.body.answer = [](const Sent& s) -> uint8_t {
    return s.type == KILO_MSG_TRICK_COMMIT ? KILO_ACK_STATUS_NO_TRICK : KILO_ACK_STATUS_OK;
  };
  r.player.start(*find_trick("push_up"));
  r.run_ms(8);
  r.player.stop();
  CHECK(r.player.result().state == PlayState::Failed);
  r.run_ms(60);
  CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1 && r.body.count(KILO_MSG_TRICK_PLAY) == 0 && !r.player.busy());
}

}  // namespace

int main() {
  test_library_matches_python();
  test_timing();
  test_upload_is_cached_and_plays_to_done();
  test_play_refusals_and_no_trick();
  test_play_timeout_stops_the_trick();
  test_stop_while_uploading();
  return report("tricks");
}
