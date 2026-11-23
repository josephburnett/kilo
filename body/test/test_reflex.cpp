// The fall reflex (core/reflex.*, body.cpp; doc §3g) and the head layer
// (SET_HEAD, arbiter.cpp; doc §3c layer 1) (Appendix H "reflex", "arbiter"):
// the detector needs FALL_SAMPLES consecutive samples (a dip restarts it,
// a single bad sample never trips), both sides and pitch falls, the hold
// until still, then LYING (REST, LIMP, the reflex lets go), FALL_OK
// suppression and the mind's get-up taking the joints from the reflex,
// LIMP/CALIB/IMU_FAULT gating, the epoch and STALE / FALLEN refusals, a
// stream ended by the fall, a more important reflex taking over; head
// hold, slew, timeout, release, epoch release, balance exclusion and owner
// precedence.
#include <cstdlib>
#include <cstring>
#include <vector>
#include "../src/core/body.h"
#include "../src/core/fx.h"
#include "../src/core/link.h"
#include "../src/core/params.h"
#include "../src/core/posture.h"
#include "../src/core/reflex.h"
#include "check.h"
#include "body_fixture.h"
#include "fake_eeprom.h"
#include "fake_link_hal.h"

static Config cfg;
static Body& b = g_body;
static uint16_t counts[16];
static TickIn in;
static std::vector<Event> evs;  // every event since reset()

static void reset(uint8_t state = CFG_MIGRATED) {
  fixture_body(b, cfg, state);
  g_lh.reset();
  link_init(1, 0);
  in = fixture_upright();
  evs.clear();
}

static void drain() { fixture_drain(b.ev, evs); }

static void tick(int n = 1) {
  for (int i = 0; i < n; i++) {
    body_tick(b, in, counts);
    drain();
  }
}

static int count(uint8_t code, int arg = -1000000) {
  drain();
  int n = 0;
  for (const Event& e : evs)
    if (e.code == code && (arg == -1000000 || e.arg == arg)) n++;
  return n;
}
static const Event* last(uint8_t code) {
  drain();
  for (size_t i = evs.size(); i-- > 0;)
    if (evs[i].code == code) return &evs[i];
  return nullptr;
}

static void stand() {
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  tick(40);
}
static void att(int roll_deg, int pitch_deg, int rate_dps = 0) {
  in.att.roll = (int16_t)(roll_deg * 16);
  in.att.pitch = (int16_t)(pitch_deg * 16);
  in.att.rate_roll = in.att.rate_pitch = in.att.rate_yaw = (int16_t)(rate_dps * 16);
}
static bool at_posture(uint8_t p) {
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++)
    if (b.arb.out[k] != joint_clamp(joint_id((uint8_t)k), posture_angle(p, (uint8_t)k))) return false;
  return true;
}

static bool all_owned(uint8_t own, uint8_t from = 0) {
  for (uint8_t k = from; k < DRIVEN_COUNT; k++)
    if (b.arb.owner[k] != own) return false;
  return true;
}
static bool legs_at(uint8_t p) {
  for (uint8_t k = 3; k < DRIVEN_COUNT; k++)
    if (b.arb.out[k] != joint_clamp(joint_id((uint8_t)k), posture_angle(p, (uint8_t)k))) return false;
  return true;
}
static uint8_t head_goal(uint8_t epoch, uint8_t mode, int16_t pan, int16_t tilt, int16_t tail, uint16_t dps) {
  uint8_t p[10] = {epoch, mode};
  std::memcpy(p + 2, &pan, 2);
  std::memcpy(p + 4, &tilt, 2);
  std::memcpy(p + 6, &tail, 2);
  std::memcpy(p + 8, &dps, 2);
  return body_goal_head(b, p);
}

// ---- A minimal stream: STAND's legs as joint angles, reached in `dt` ticks
// (flags: FALL_OK for the mind's get-up), the pan driven or left to IDLE.
static uint8_t pose(uint8_t flags, int16_t pan = JOINT_OFF, int dt = 20, uint8_t epoch = 0xFF) {
  uint8_t p[KILO_POSE_LEN];
  const int16_t head[3] = {pan, JOINT_OFF, JOINT_OFF};
  fixture_pose(p, epoch == 0xFF ? b.epoch : epoch, (uint16_t)(b.tick + dt), (uint8_t)(FIXTURE_ALL_JOINT | flags),
               FIXTURE_STAND_LEGS, head);
  return body_pose(b, p);
}

// ---------------------------------------------------------------------------
static void test_detector() {
  Attitude a = Attitude();
  uint8_t over = 0;
  auto feed = [&](int roll, int pitch) {
    a.roll = (int16_t)(roll * 16);
    a.pitch = (int16_t)(pitch * 16);
    return fall_detect(over, a);
  };
  // Exactly 75 deg is not over; 75 deg + 1/16 is.
  for (int i = 0; i < 20; i++) CHECK(feed(75, -75) == 0);
  // Five over, then a dip: no trip, and the count restarts.
  for (int i = 0; i < 5; i++) CHECK(feed(-80, 0) == 0);
  CHECK(feed(-74, 0) == 0 && over == 0);
  for (int i = 0; i < 5; i++) CHECK(feed(-80, 0) == 0);
  CHECK(feed(-80, 0) == -1);  // the sixth consecutive one
  CHECK(over == 0);
  // A single bad sample (P0 finding 8 read -70.5 deg; here even -170) never trips.
  for (int i = 0; i < 100; i++) CHECK(feed(i % 3 == 0 ? -170 : 10, 0) == 0);
  // Both sides, on the back, and nose/tail down (pitch folded about 90 deg).
  struct Case {
    int roll, pitch, want;
  } cases[] = {{88, 2, 1},    {-88, 2, -1},  {179, -178, 1}, {-179, 178, -1}, {5, 85, 2},
               {5, -85, -2},  {-40, 100, 2}, {30, -104, -2}, {90, 110, 1},    {-90, -120, -1},
               {76, 0, 1},    {0, 76, 2}};
  for (const Case& c : cases) {
    over = 0;
    int got = 0;
    for (int i = 0; i < FALL_SAMPLES; i++) got = feed(c.roll, c.pitch);
    CHECKF(got == c.want, "roll %d pitch %d: got %d want %d", c.roll, c.pitch, got, c.want);
  }
  a = Attitude();
  a.rate_yaw = 20 * 16;
  CHECK(!fall_still(a));
  a.rate_yaw = -19 * 16;
  CHECK(fall_still(a));
}

// Tip over from STAND: count, trip, hold, wait still, LYING: REST, LIMP 2 s
// later, the reflex lets go. The body does not get up by itself.
static void test_fall_and_lie(bool right_up) {
  reset();
  stand();
  uint8_t e0 = b.epoch;
  int side = right_up ? 1 : -1;
  // 5 samples over, a dip, 5 again: nothing.
  att(85 * side, 0, 200);
  tick(5);
  att(70 * side, 0, 200);
  tick();
  att(85 * side, 0, 200);
  tick(5);
  CHECK(b.fall.state == FS_UPRIGHT && count(EV_FELL) == 0 && b.epoch == e0);
  // Missing samples neither count nor reset.
  in.imu = IMU_NONE;
  tick(3);
  CHECK(b.fall.state == FS_UPRIGHT);
  in.imu = IMU_FRESH;
  tick();  // the sixth consecutive fresh sample over
  CHECK(b.fall.state == FS_FALLEN && b.holder == RH_FALL && body_fallen(b));
  CHECK(count(EV_FELL, side) == 1 && b.epoch == (uint8_t)(e0 + 1) && last(EV_FELL)->epoch == b.epoch);
  CHECK(body_mode(b) == KILO_MODE_FALLEN && all_owned(OWN_REFLEX));
  CHECK(b.notes.count >= 1);  // the chirp
  // REFLEX holds the pose while the body still moves.
  int16_t held[DRIVEN_COUNT];
  std::memcpy(held, b.arb.out, sizeof held);
  tick(50);
  CHECK(std::memcmp(held, b.arb.out, sizeof held) == 0 && b.fall.state == FS_FALLEN);
  // Still for 9 ticks, a twitch, then 10: LYING on the 10th.
  att(85 * side, 0, 0);
  tick(9);
  att(85 * side, 0, 25);
  tick();
  att(85 * side, 0, 0);
  tick(9);
  CHECK(b.fall.state == FS_FALLEN && count(EV_LYING) == 0);
  tick();
  CHECK(b.fall.state == FS_LYING && count(EV_LYING, side) == 1 && body_mode(b) == KILO_MODE_FALLEN);
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_REST && b.arb.reflex.posture == POSTURE_REST);
  // REST over REST_MOVE_TICKS (never more than the slew per tick), LIMP
  // FALL_LIMP_TICKS later, then the reflex lets go.
  int16_t prev[DRIVEN_COUNT];
  std::memcpy(prev, b.arb.out, sizeof prev);
  for (int t = 0; t < REST_MOVE_TICKS; t++) {
    tick();
    for (uint8_t k = 0; k < DRIVEN_COUNT; k++) {
      CHECK(std::abs(b.arb.out[k] - prev[k]) <= PARAM(SLEW_MAX_A16));
      prev[k] = b.arb.out[k];
    }
  }
  CHECK(at_posture(POSTURE_REST));
  tick(FALL_LIMP_TICKS - 1);
  CHECK(b.arb.out[0] != JOINT_OFF);
  tick();
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) CHECK(b.arb.out[k] == JOINT_OFF);
  tick();
  CHECK(b.fall.state == FS_UPRIGHT && b.holder == RH_NONE && body_mode(b) == KILO_MODE_LIMP);
  CHECK(b.epoch == (uint8_t)(e0 + 1));
  tick(100);  // LIMP: no detection although still on its side
  CHECK(count(EV_FELL) == 1 && count(EV_LYING) == 1);
}

// Nose or tail down: the same, LYING with the pitch arg.
static void test_pitch_fall() {
  for (int sign = -1; sign <= 1; sign += 2) {
    reset();
    stand();
    att(10, 88 * sign, 100);
    tick(FALL_SAMPLES);
    CHECK(b.fall.state == FS_FALLEN && count(EV_FELL, 2 * sign) == 1);
    tick(30);
    CHECK(b.fall.state == FS_FALLEN);
    att(10, 88 * sign, 0);
    tick(FALL_STILL_TICKS);
    CHECK(b.fall.state == FS_LYING && count(EV_LYING, 2 * sign) == 1);
  }
  // An IMU fault while fallen: nothing can be judged, LYING at once.
  reset();
  stand();
  att(-90, 0, 100);
  tick(FALL_SAMPLES);
  CHECK(b.fall.state == FS_FALLEN);
  b.imu_fault = 1;
  tick();
  CHECK(b.fall.state == FS_LYING && count(EV_LYING, -1) == 1);
}

// Gating: LIMP, CALIB, IMU_FAULT, another reflex, FALL_OK; a stream without
// FALL_OK ends with the fall (STREAM_STOPPED, the new epoch).
static void test_gating_and_streams() {
  reset();  // cold: LIMP
  att(-90, 0, 0);
  tick(50);
  CHECK(b.fall.state == FS_UPRIGHT && count(EV_FELL) == 0);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_CALIB, 0, 0) == GOAL_OK);
  tick(50);
  CHECK(b.fall.state == FS_UPRIGHT);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_REST, 0, 0) == GOAL_OK);
  b.imu_fault = 1;
  tick(50);
  CHECK(b.fall.state == FS_UPRIGHT);
  b.imu_fault = 0;
  tick(FALL_SAMPLES);
  CHECK(b.fall.state == FS_FALLEN);  // REST is not exempt

  // A FALL_OK stream: suppressed while it is on; the count restarts after.
  reset();
  stand();
  CHECK(pose(KILO_POSE_FLAGS_FALL_OK) == GOAL_OK);
  att(-90, 0, 0);
  tick(100);
  CHECK(body_stream_on(b) && b.fall.state == FS_UPRIGHT && b.fall.over == 0);
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);  // the posture ends the stream
  CHECK(!body_stream_on(b));
  tick(FALL_SAMPLES - 1);
  CHECK(b.fall.state == FS_UPRIGHT);
  tick();
  CHECK(b.fall.state == FS_FALLEN);

  // A stream without FALL_OK: STREAM_STOPPED with the new epoch, REFLEX
  // takes every joint at once.
  reset();
  stand();
  CHECK(pose(0) == GOAL_OK);
  tick(10);
  uint8_t e0 = b.epoch;
  att(0, -80, 0);
  tick(FALL_SAMPLES);
  const Event* d = last(EV_STREAM_STOPPED);
  CHECK(d && d->arg == 0 && d->epoch == (uint8_t)(e0 + 1));
  CHECK(!body_stream_on(b) && all_owned(OWN_REFLEX) && count(EV_FELL, -2) == 1);
  // ...and is refused while fallen, or with the old epoch.
  CHECK(pose(0) == GOAL_FALLEN);
  CHECK(pose(KILO_POSE_FLAGS_FALL_OK, JOINT_OFF, 20, e0) == GOAL_STALE);
}

// The mind's get-up (decided 2026-11-22: the body has no built-in one): a
// FALL_OK stream takes the joints from the fall reflex, FALLEN or LYING;
// any other is refused; no fall is detected while it is on; once a posture
// ends it, a body still down falls again.
static void test_getup() {
  reset();
  stand();
  att(-110, 0, 100);
  tick(FALL_SAMPLES);
  CHECK(b.fall.state == FS_FALLEN && count(EV_FELL, -1) == 1);
  CHECK(pose(0) == GOAL_FALLEN);  // not FALL_OK
  tick(5);
  CHECK(pose(KILO_POSE_FLAGS_FALL_OK) == GOAL_OK);  // still moving: taken from FALLEN
  CHECK(b.fall.state == FS_UPRIGHT && b.holder == RH_NONE && !b.arb.reflex_active && body_stream_on(b));
  tick();
  CHECK(all_owned(OWN_STREAM, 3) && all_owned(OWN_IDLE, 0) == false && b.arb.owner[0] == OWN_IDLE);
  CHECK(body_mode(b) == KILO_MODE_STREAM);
  tick(100);  // still on his side: no fall while it is on
  CHECK(count(EV_FELL) == 1 && body_stream_on(b));
  // Ended by a posture while he is still down: the reflex trips again, and
  // once he is still he lies down.
  CHECK(body_goal_posture(b, b.epoch, POSTURE_REST, 0, 0) == GOAL_OK);
  att(-110, 0, 0);
  tick(FALL_SAMPLES + FALL_STILL_TICKS);
  CHECK(count(EV_FELL) == 2 && b.fall.state == FS_LYING && count(EV_LYING) == 1);
  // Taken from LYING too (REST under way, before LIMP).
  tick(30);
  CHECK(pose(KILO_POSE_FLAGS_FALL_OK) == GOAL_OK);
  CHECK(b.fall.state == FS_UPRIGHT && b.holder == RH_NONE && body_stream_on(b));
  tick(40);
  CHECK(legs_at(POSTURE_STAND));  // the stream's key: STAND's legs as joint angles (the head IDLE's)
  // After LIMP the reflex has let go: a stream is refused BUSY (a posture first).
  reset();
  stand();
  att(-110, 0, 0);
  tick(FALL_SAMPLES + FALL_STILL_TICKS + REST_MOVE_TICKS + FALL_LIMP_TICKS + 2);
  CHECK(b.fall.state == FS_UPRIGHT && body_mode(b) == KILO_MODE_LIMP);
  CHECK(pose(KILO_POSE_FLAGS_FALL_OK) == GOAL_BUSY);
}

// Goals: old epoch STALE; current epoch FALLEN except LIMP, which ends the
// reflex; after RECOVERED, goals with the new epoch work.
static void test_goals_while_fallen() {
  reset();
  stand();
  uint8_t old = b.epoch;
  att(90, 0, 0);
  tick(FALL_SAMPLES);
  uint8_t now = b.epoch;
  CHECK(now != old);
  CHECK(body_goal_posture(b, old, POSTURE_SIT, 0, 0) == GOAL_STALE);
  CHECK(body_goal_posture(b, now, POSTURE_SIT, 0, 0) == GOAL_FALLEN);
  CHECK(body_goal_posture(b, old, POSTURE_REST, 0, 0) == GOAL_FALLEN);  // REST: never stale, but fallen
  CHECK(head_goal(old, 1, 0, 0, 0, 0) == GOAL_STALE);
  CHECK(head_goal(now, 1, 0, 0, 0, 0) == GOAL_FALLEN);
  CHECK(head_goal(old, 0, 0, 0, 0, 0) == GOAL_OK);  // a release is always fine
  CHECK(b.fall.state == FS_FALLEN && b.holder == RH_FALL);
  // LIMP ends it (with any epoch): the Pi's emergency stop.
  CHECK(body_goal_posture(b, old, POSTURE_LIMP, 0, 0) == GOAL_OK);
  CHECK(b.fall.state == FS_UPRIGHT && b.holder == RH_NONE && !b.arb.reflex_active);
  tick();
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) CHECK(b.arb.out[k] == JOINT_OFF);
  CHECK(body_mode(b) == KILO_MODE_LIMP && count(EV_LYING) == 0);
  // Through the link: SET_POSTURE refused FALLEN as EVENT GOAL_REFUSED.
  reset();
  stand();
  att(90, 0, 0);
  tick(FALL_SAMPLES);
  g_lh.take();
  uint8_t p[4] = {b.epoch, POSTURE_SIT, 0, 0};
  g_lh.feed(frame(KILO_MSG_SET_POSTURE, 3, std::vector<uint8_t>(p, p + 4)));
  link_poll();
  const Event* r = last(EV_GOAL_REFUSED);
  CHECK(r && r->arg == ((KILO_MSG_SET_POSTURE << 8) | KILO_GOAL_REFUSED_REASON_FALLEN));
}

// A more important reflex takes over during the fall: the watchdog's REST.
static void test_watchdog_takes_over() {
  reset();
  stand();
  body_link_frame(b);  // arm the watchdog
  att(-90, 0, 100);  // still moving: FALLEN, not yet LYING
  tick(FALL_SAMPLES);
  CHECK(b.fall.state == FS_FALLEN);
  tick(WD_REST_TICKS + 2);
  CHECK(b.holder == RH_WATCHDOG && b.fall.state == FS_UPRIGHT);
  CHECK(arbiter_idle_posture(b.arb) == POSTURE_REST && count(EV_LYING) == 0);
  // The link comes back: the watchdog lets go, detection re-arms, trips again.
  body_link_frame(b);
  tick(FALL_SAMPLES);
  CHECK(b.fall.state == FS_FALLEN && count(EV_FELL) == 2);
}

// ---- Head layer --------------------------------------------------------------
static void test_head() {
  reset();
  stand();
  CHECK(head_goal(b.epoch, 2, 0, 0, 0, 0) == GOAL_BAD_ARG);
  CHECK(head_goal((uint8_t)(b.epoch + 1), 1, 0, 0, 0, 0) == GOAL_STALE);
  CHECK(!arbiter_head_held(b.arb));
  // Pan to +40 deg at 100 deg/s (2 deg/tick); tilt -10 at once; tail not held.
  CHECK(head_goal(b.epoch, 1, 40 * 16, -10 * 16, JOINT_OFF, 100) == GOAL_OK);
  CHECK(arbiter_head_held(b.arb) && b.arb.head_step == 32);
  tick();
  CHECK(b.arb.out[0] == 32 && b.arb.out[1] == -32);  // both held joints at 2 deg/tick
  CHECK(b.arb.owner[0] == OWN_PI_HOLD && b.arb.owner[1] == OWN_PI_HOLD && b.arb.owner[2] == OWN_IDLE);
  for (int t = 2; t <= 20; t++) {
    tick();
    CHECKF(b.arb.out[0] == t * 32, "pan at tick %d: %d", t, b.arb.out[0]);
  }
  tick(5);
  CHECK(b.arb.out[0] == 40 * 16 && b.arb.out[1] == -10 * 16 && b.arb.out[2] == 0);
  // Refreshed at 5 Hz it stays; balance corrections never reach held joints.
  b.bal.enable = 1;
  att(8, -6, 0);
  for (int t = 0; t < 300; t++) {
    if (t % 10 == 0) CHECK(head_goal(b.epoch, 1, 40 * 16, -10 * 16, JOINT_OFF, 100) == GOAL_OK);
    tick();
  }
  CHECK(b.bal.active && (b.bal.ax[BAL_ROLL].corr || b.bal.ax[BAL_PITCH].corr));
  CHECK(b.arb.out[0] == 40 * 16 && b.arb.out[1] == -10 * 16);
  CHECK(b.arb.out[2] == joint_clamp(2, (int16_t)(posture_angle(POSTURE_STAND, 2) + balance_joint(b.bal, 2))));
  b.bal.enable = 0;
  att(0, 0, 0);
  tick(60);
  // Not refreshed: released after HEAD_HOLD_TICKS, then a crossfade back to
  // the posture (bounded steps), owner IDLE again.
  CHECK(head_goal(b.epoch, 1, 40 * 16, -10 * 16, JOINT_OFF, 0) == GOAL_OK);
  tick(HEAD_HOLD_TICKS - 1);
  CHECK(arbiter_head_held(b.arb));
  tick();  // the 100th tick without a refresh is the last one held
  CHECK(!arbiter_head_held(b.arb) && b.arb.out[0] == 40 * 16);
  tick();
  CHECK(b.arb.owner[0] == OWN_IDLE && b.arb.out[0] < 40 * 16 && b.arb.out[0] > 0);
  int16_t prev = b.arb.out[0];
  for (int t = 1; t < PARAM(XFADE_TICKS); t++) {
    tick();
    CHECK(b.arb.out[0] <= prev && prev - b.arb.out[0] <= PARAM(SLEW_MAX_A16));
    prev = b.arb.out[0];
  }
  CHECK(b.arb.out[0] == 0 && b.arb.out[1] == 0);
  // step 0: only the safety slew (10 deg/tick).
  CHECK(head_goal(b.epoch, 1, -60 * 16, 0, 0, 0) == GOAL_OK);
  tick();
  CHECK(b.arb.out[0] == -PARAM(SLEW_MAX_A16));
  tick(10);
  CHECK(b.arb.out[0] == -60 * 16);
  // Out-of-range targets: the safety stage clamps them (tilt limit -80).
  CHECK(head_goal(b.epoch, 1, 32767, -32767, 0, 0) == GOAL_OK);
  tick(40);
  CHECK(b.arb.out[0] == 120 * 16 && b.arb.out[1] == -80 * 16);
  // mode 0 releases (any epoch), with the crossfade.
  CHECK(head_goal((uint8_t)(b.epoch + 5), 0, 0, 0, 0, 0) == GOAL_OK);
  CHECK(!arbiter_head_held(b.arb));
  tick(PARAM(XFADE_TICKS) + 20);
  CHECK(at_posture(POSTURE_STAND));
  // An epoch bump releases it.
  CHECK(head_goal(b.epoch, 1, 30 * 16, 0, 0, 0) == GOAL_OK);
  tick(5);
  body_epoch_bump(b);
  CHECK(!arbiter_head_held(b.arb));
  // Low battery refuses a hold.
  b.batt.level = BATT_LOW;
  CHECK(head_goal(b.epoch, 1, 0, 0, 0, 0) == GOAL_LOW_BATT);
  b.batt.level = BATT_OK;
  // Telemetry: flags bit 5 (HEAD_HELD) and the head owner nibble PI_HOLD.
  CHECK(head_goal(b.epoch, 1, 30 * 16, 0, 0, 0) == GOAL_OK);
  tick(2);
  g_lh.take();
  if (b.tick % 2) tick();  // TELEMETRY is due on even ticks (TELEM_DIV 2)
  g_lh.take();
  link_tick();
  int seen = 0;
  for (const TxFrame& f : g_lh.take()) {
    if (f.type != KILO_MSG_TELEMETRY) continue;
    kilo_telemetry_t m;
    std::memcpy(&m, f.payload.data(), sizeof m);
    CHECK((m.flags & KILO_TELEMETRY_FLAGS_HEAD_HELD) && (m.owners >> 4) == KILO_OWNER_PI_HOLD &&
          (m.owners & 15) == KILO_OWNER_IDLE);
    seen++;
  }
  CHECK(seen >= 1);
}

// Precedence: a stream whose pose drives the head beats the hold; one that
// leaves the head to IDLE leaves it to the hold; REFLEX beats both (and its
// epoch bump releases).
static void test_head_precedence() {
  reset();
  stand();
  CHECK(head_goal(b.epoch, 1, -50 * 16, 0, JOINT_OFF, 0) == GOAL_OK);
  tick(10);
  CHECK(b.arb.out[0] == -50 * 16);
  CHECK(pose(0, 30 * 16, 20) == GOAL_OK);
  for (int t = 0; t < 30; t++) {
    if (t % 10 == 0) head_goal(b.epoch, 1, -50 * 16, 0, JOINT_OFF, 0);
    tick();
  }
  CHECK(b.arb.owner[0] == OWN_STREAM && b.arb.out[0] == 30 * 16);  // the key's pan 30, reached
  CHECK(body_goal_posture(b, b.epoch, POSTURE_STAND, 0, 0) == GOAL_OK);
  head_goal(b.epoch, 1, -50 * 16, 0, JOINT_OFF, 0);
  tick(20);
  CHECK(b.arb.owner[0] == OWN_PI_HOLD && b.arb.out[0] == -50 * 16);  // back to the hold
  // A stream that leaves the head to IDLE: the head stays held.
  CHECK(pose(0, JOINT_OFF, 20) == GOAL_OK);
  head_goal(b.epoch, 1, -50 * 16, 0, JOINT_OFF, 0);
  tick(10);
  CHECK(b.arb.owner[0] == OWN_PI_HOLD && b.arb.owner[3] == OWN_STREAM && b.arb.out[0] == -50 * 16);
  // A fall: REFLEX everywhere, the hold is gone (epoch bump).
  att(-90, 0, 0);
  tick(FALL_SAMPLES + 1);
  CHECK(all_owned(OWN_REFLEX) && !arbiter_head_held(b.arb) && b.fall.state == FS_FALLEN);
}

int main() {
  test_detector();
  test_fall_and_lie(false);
  test_fall_and_lie(true);
  test_pitch_fall();
  test_gating_and_streams();
  test_getup();
  test_goals_while_fallen();
  test_watchdog_takes_over();
  test_head();
  test_head_precedence();
  return report("reflex");
}
