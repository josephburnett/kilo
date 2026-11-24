// The state estimate and the intents (kernel/mind/intent/, docs/intent.md):
// 1. the estimate and the generated tables against the Python reference's vectors
//    (tests/intent_vectors.txt, made by tests/gen_intent_vectors.py from mind/kilo/intent);
// 2. the estimate in the Mind: it always runs, logs every change and counts it;
// 3. intents off (the default): the remote's posture keys latch SET_POSTURE as before;
// 4. the engine's moves (engine/posture.h) as the intents play them: staged where that keeps the
//    weight further inside, through STANDING between sitting and resting, at the tempo asked;
// 5. intents on, in the Mind against the scripted body: REST -> B6 plays the engine's move and ends STANDING;
//    a body that does not end where the model says (failed steps, then REST);
//    Kilo on his side (the commanded posture is not believed; the fall reflex is waited for);
//    LIMP (REST first); a walk key drops the intent.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "intent/estimate.h"
#include "intent/intent.h"
#include "remote/client.h"
#include "tests/check.h"
#include "tests/mind_rig.h"
#include "tricks/library.h"

using namespace kilo;
using namespace kilo::intent;

namespace {

// ---- 1. the vectors -------------------------------------------------------------------------
void test_vectors(const char* path) {
  FILE* f = std::fopen(path, "r");
  CHECKF(f != nullptr, "cannot open %s", path);
  if (f == nullptr) return;
  char line[512];
  Estimator est;
  int rows = 0, cs = 0, ss = 0, edges = 0, nexts = 0, nextrs = 0, moves = 0;
  while (std::fgets(line, sizeof line, f) != nullptr) {
    if (line[0] == '#') continue;
    int a[16];
    long long t, since;
    char name[64];
    if (std::sscanf(line, "row %d %d %d %d %d %d", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) == 6) {
      rows++;
      CHECK(a[0] < kRowCount);
      const Row& r = kRows[a[0]];
      CHECKF((int)r.state == a[1] && r.pitch == a[2] && r.roll == a[3] && r.tol_pitch == a[4] && r.tol_roll == a[5],
             "row %d", a[0]);
    } else if (std::sscanf(line, "c %d %d %d %d -> %d %d %d %d", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6],
                           &a[7]) == 8) {
      cs++;
      Answer ans = classify((int16_t)a[0], (int16_t)a[1], (Cmd)a[2], (State)a[3]);
      CHECKF((int)ans.state == a[4] && ans.confidence == a[5] && ans.err_pitch == a[6] && ans.err_roll == a[7],
             "classify(%d, %d, cmd %d, prev %d) = %d %d %d %d, Python %d %d %d %d", a[0], a[1], a[2], a[3],
             (int)ans.state, ans.confidence, ans.err_pitch, ans.err_roll, a[4], a[5], a[6], a[7]);
    } else if (std::strncmp(line, "seq ", 4) == 0) {
      est = Estimator();
    } else if (std::sscanf(line, "s %lld %d %d %d %d %d %d %d -> %d %d %d %lld %d %d", &t, &a[0], &a[1], &a[2], &a[3],
                           &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &since, &a[10], &a[11]) == 14) {
      ss++;
      Sample s = {(uint64_t)t, (int16_t)a[0], (int16_t)a[1], (int16_t)a[2], (int16_t)a[3], (int16_t)a[4],
                  (uint8_t)a[5], (Cmd)a[6]};
      bool ch = est.feed(s);
      const Estimate& e = est.estimate();
      CHECKF((int)ch == a[7] && (int)e.state == a[8] && e.confidence == a[9] && (long long)e.since_us == since &&
                 e.err_pitch == a[10] && e.err_roll == a[11],
             "sample at %lld: %d %s %d since %lld err %d %d; Python %d state %d %d since %lld err %d %d", t, (int)ch,
             state_name(e.state), e.confidence, (long long)e.since_us, e.err_pitch, e.err_roll, a[7], a[8], a[9],
             since, a[10], a[11]);
    } else if (std::sscanf(line, "edge %d %d %d %d %d %63s %d", &a[0], &a[1], &a[2], &a[3], &a[4], name, &a[5]) == 7) {
      edges++;
      CHECK(a[0] < kEdgeCount);
      const Edge& e = kEdges[a[0]];
      bool move_ok = e.move == nullptr ? std::strcmp(name, "-") == 0 : std::strcmp(name, e.move->name) == 0;
      CHECKF((int)e.from == a[1] && (int)e.to == a[2] && (int)e.kind == a[3] && e.posture == a[4] && move_ok &&
                 (int)e.mirror == a[5],
             "edge %d", a[0]);
    } else if (std::sscanf(line, "nextr %d %d %d %d", &a[0], &a[1], &a[2], &a[3]) == 4) {
      nextrs++;
      CHECK(a[0] < kStates);
      for (int g = 0; g < kGoals; g++) CHECKF(kNextRighting[a[0]][g] == a[1 + g], "nextr %d %d", a[0], g);
    } else if (std::sscanf(line, "next %d %d %d %d", &a[0], &a[1], &a[2], &a[3]) == 4) {
      nexts++;
      CHECK(a[0] < kStates);
      for (int g = 0; g < kGoals; g++) CHECKF(kNext[a[0]][g] == a[1 + g], "next %d %d", a[0], g);
    } else if (std::sscanf(line, "move %63s %d %d %d", name, &a[0], &a[1], &a[2]) == 4) {
      CHECK(moves < kMoveCount);
      const tricks::Trick& m = kMoves[moves];
      uint32_t ticks = 0;
      CHECK(tricks::total_ticks(m, tricks::kSpeedUnity, &ticks));
      CHECKF(std::strcmp(m.name, name) == 0 && tricks::trick_crc16(m) == a[0] && tricks::trick_id_for(m) == a[1] &&
                 (int)ticks == a[2] && kMoveCheck[moves].crc16 == a[0] && kMoveCheck[moves].trick_id == a[1],
             "move %s: crc %u id %u ticks %u", name, (unsigned)tricks::trick_crc16(m),
             (unsigned)tricks::trick_id_for(m), (unsigned)ticks);
      CHECKF(tricks::trick_problem(m) == nullptr, "move %s: %s", name, tricks::trick_problem(m));
      moves++;
    }
  }
  std::fclose(f);
  CHECKF(rows == kRowCount && cs > 5000 && ss > 800 && edges == kEdgeCount && nexts == kStates && nextrs == kStates &&
             moves == kMoveCount,
         "vectors read: %d rows, %d classify, %d samples, %d edges, %d next, %d moves", rows, cs, ss, edges, nexts,
         moves);
}

// Every table entry leads somewhere sensible: following kNext from any state reaches the goal within
// kMaxSteps, never through MOVING, and a fallen state's step is the one placeholder kind.
void test_table() {
  for (int s = 0; s < kStates; s++) {
    for (int g = 0; g < kGoals; g++) {
      State st = (State)s, goal = kGoalStates[g];
      if (st == State::Moving) {
        CHECK(kNext[s][g] == -1);
        continue;
      }
      int steps = 0;
      while (st != goal && steps <= Intent::kMaxSteps) {
        int idx = kNext[(int)st][g];
        CHECKF(idx >= 0 && idx < kEdgeCount, "no step from %s to %s", state_name(st), state_name(goal));
        if (idx < 0) break;
        const Edge& e = kEdges[idx];
        CHECK(e.from == st && e.to != State::Moving);
        if (fallen(st)) CHECK(e.kind == EdgeKind::WaitReflex);
        if (e.kind == EdgeKind::Engine) CHECK(e.move == nullptr && !fallen(e.from) && e.from != State::Unknown);
        st = e.to;
        steps++;
      }
      CHECKF(st == goal && steps <= 3, "%s to %s: %d steps", state_name((State)s), state_name(goal), steps);
    }
  }
}

// The righting experiment's table: the same for upright states; a side or his back by a Right edge (a
// FALL_OK righting move ending in REST, or on a side), nose or tail down nothing; up within four steps.
void test_table_righting() {
  for (int s = 0; s < kStates; s++) {
    for (int g = 0; g < kGoals; g++) {
      State st = (State)s, goal = kGoalStates[g];
      if (!fallen(st)) {
        CHECK(kNextRighting[s][g] == kNext[s][g]);
        continue;
      }
      if (st == State::OnEnd) {
        CHECK(kNextRighting[s][g] == -1);
        continue;
      }
      int steps = 0;
      while (st != goal && steps <= Intent::kMaxSteps) {
        int idx = kNextRighting[(int)st][g];
        CHECKF(idx >= 0 && idx < kEdgeCount, "no righting step from %s to %s", state_name(st), state_name(goal));
        if (idx < 0) break;
        const Edge& e = kEdges[idx];
        CHECK(e.from == st);
        if (fallen(st)) {
          CHECK(e.kind == EdgeKind::Right && e.move != nullptr && (e.move->flags & tricks::kTrickFallOk) &&
                e.move->end_posture == KILO_POSTURE_REST && e.move->roll_max_a16 == 0);
          CHECK(e.mirror == (st == State::OnLeftSide));
        }
        st = e.to;
        steps++;
      }
      CHECKF(st == goal && steps <= 4, "righting %s to %s: %d steps", state_name((State)s), state_name(goal), steps);
    }
  }
}

// ---- the Mind against a scripted body that plays the tricks' POSE streams ------------------------
constexpr int16_t kDeg = 16;

struct Rig : MindRig {
  size_t seen = 0;          // body.sent read so far
  bool playing = false;     // a POSE stream is on (the first key started it; a SET_POSTURE ends it)
  int stops = 0;            // streams ended by a SET_POSTURE before their last key's tick
  uint16_t last_key = 0;    // the tick of the last POSE key seen
  bool obey = true;         // the body ends a move, and takes a posture, at the attitude the model predicts
  int16_t stuck_pitch = 0, stuck_roll = 0;   // ...else here
  int plays = 0;
  int last_posture = -1;    // the SET_POSTURE code last seen (the goal is refreshed at 5 Hz: only a change moves him)

  explicit Rig(bool intent) : MindRig(cfg(intent)) {}
  static MindConfig cfg(bool intent) {
    MindConfig c = remote_cfg();
    c.intent = intent;
    return c;
  }
  void attitude_of(uint8_t posture) {
    if (!obey) {
      body.pitch = stuck_pitch;
      body.roll = stuck_roll;
      return;
    }
    for (int i = 0; i < kRowCount; i++)
      if (posture_of(kRows[i].state) == posture) {
        body.pitch = kRows[i].pitch;
        body.roll = kRows[i].roll;
      }
    if (posture == KILO_POSTURE_STANCE) body.pitch = body.roll = 0;
  }
  void pass() override {
    MindRig::pass();
    for (; seen < body.sent.size(); seen++) {
      const Sent& s = body.sent[seen];
      if (s.type == KILO_MSG_POSE && s.payload.size() == sizeof(kilo_pose_t)) {
        kilo_pose_t k;
        std::memcpy(&k, s.payload.data(), sizeof k);
        last_key = k.tick;
        if (!playing) {
          playing = true;
          plays++;
          body.mode = KILO_MODE_STREAM;  // (from FALLEN too: a FALL_OK stream takes the joints)
          last_posture = -1;
        }
        continue;
      }
      if (s.type == KILO_MSG_SET_POSTURE && s.payload.size() == 4 && playing) {
        // the trick's end posture ends the stream: the body takes it
        playing = false;
        if ((int16_t)(last_key - body.tick) > 0) stops++;
        body.mode = KILO_MODE_IDLE;
        last_posture = s.payload[1];
        attitude_of(s.payload[1]);
        continue;
      }
      if (s.type == KILO_MSG_SET_POSTURE && s.payload.size() == 4 && !playing && body.mode != KILO_MODE_FALLEN &&
          s.payload[1] != last_posture) {
        last_posture = s.payload[1];
        if (s.payload[1] == KILO_POSTURE_LIMP) {
          body.mode = KILO_MODE_LIMP;
        } else {
          body.mode = KILO_MODE_IDLE;
          attitude_of(s.payload[1]);
        }
      }
    }
  }
  State state() { return mind.estimator().state(); }
  // The body put into a posture without the mind's doing (the start of a test).
  void put(uint8_t posture, const char* name) {
    client_posture(mind.link(), nullptr, name);
    run_ms(1500);
    (void)posture;
  }
};

void test_estimate_in_the_mind() {
  Rig r(false);
  r.attach_recorder();
  r.start();
  r.put(KILO_POSTURE_STAND, "stand");
  CHECKF(r.state() == State::Standing, "state %s", state_name(r.state()));
  CHECK(r.log.has("state estimate on; intents off"));
  CHECK(r.log.has("-> STANDING (pitch +2.3 roll +0.5, asked stand, confidence 100)"));
  // on his left side while told to stand: the commanded posture is not the answer
  r.body.roll = 88 * kDeg;
  r.run_ms(1500);
  CHECKF(r.state() == State::OnLeftSide, "state %s", state_name(r.state()));
  CHECK(r.log.has("state: STANDING -> ON_LEFT_SIDE (pitch +2.3 roll +88.0, asked stand, confidence 90)"));
  r.body.roll = -92 * kDeg;
  r.run_ms(1500);
  CHECK(r.state() == State::OnRightSide);
  r.body.roll = 0;
  r.body.pitch = -27 * kDeg;  // he sits, told to stand
  r.run_ms(1500);
  CHECK(r.state() == State::Sitting && r.mind.estimator().estimate().confidence < 60);
  // the metrics lines (the last two)
  char line[256];
  uint32_t n = r.mind.metric_lines();
  r.mind.metric_line(n - 2, line, 201);
  CHECKF(std::strncmp(line, "state: now=SITTING ", 19) == 0 && std::strstr(line, " ON_LEFT_SIDE=1") != nullptr &&
             std::strstr(line, " STANDING=1") != nullptr,
         "%s", line);
  r.mind.metric_line(n - 1, line, 201);
  CHECKF(std::strncmp(line, "intents: on=0 asked=0 reached=0 ", 32) == 0, "%s", line);
  CHECK(!r.mind.intent().active() && r.plays == 0);
}

// Intents off: B6 and A7 latch the body's posture at once, no trick, exactly as before.
void test_off_is_as_before() {
  Rig r(false);
  r.start();
  r.put(KILO_POSTURE_REST, "rest");
  CHECK(r.state() == State::Resting);
  r.key(0x02, 400);  // B6
  CHECK(r.posture_goal() == KILO_POSTURE_STAND && r.plays == 0 && r.body.count(KILO_MSG_POSE) == 0);
  CHECK(r.log.has("remote B6 (0x02, stand): posture stand"));
  r.run_ms(1500);
  r.key(0xA2, 400);  // A7
  CHECK(r.posture_goal() == KILO_POSTURE_REST && r.plays == 0);
  CHECK(r.mind.intent().counters().asked == 0);
}

// The engine's moves between the postures (engine/posture.h), from each posture to each other.
void test_engine_moves() {
  planner::BodyModel m;
  engine::PostureMove mv;
  engine::PostureGoal g;
  const uint8_t ps[3] = {KILO_POSTURE_STAND, KILO_POSTURE_SIT, KILO_POSTURE_REST};
  for (uint8_t a : ps)
    for (uint8_t b : ps) {
      if (a == b) continue;
      g.posture = b;
      g.tempo = 1;
      CHECKF(mv.build_from(m, a, g), "%s to %s: %s", engine::posture_word(a), engine::posture_word(b),
             mv.why() != nullptr ? mv.why() : "-");
      const tricks::Trick& t = mv.trick();
      engine::Joints to;
      engine::posture_joints(b, &to);
      // it ends exactly in the posture (the body then holds it: no move), stops past 25 deg of roll
      for (int k = 0; k < engine::kJoints; k++) CHECK(t.frames[t.n_frames - 1].ang[k] == (int8_t)to.a[k]);
      CHECK(t.end_posture == b && t.flags == 0 && t.roll_max_a16 == 25 * 16 && tricks::trick_problem(t) == nullptr);
      CHECKF(mv.margin() >= engine::kPostureMarginMm && mv.margin() >= mv.direct_margin(), "%s to %s: %d mm",
             engine::posture_word(a), engine::posture_word(b), (int)mv.margin());
      // the tempo: half the pace takes twice as long, the same way
      uint32_t t1 = 0, t2 = 0;
      tricks::total_ticks(t, tricks::kSpeedUnity, &t1);
      g.tempo = (planner::real)0.5;
      CHECK(mv.build_from(m, a, g));
      tricks::total_ticks(mv.trick(), tricks::kSpeedUnity, &t2);
      CHECKF(t2 >= 2 * t1 - 2 && t2 <= 2 * t1 + 2, "%u then %u ticks", (unsigned)t1, (unsigned)t2);
    }
  // stand <-> sit: all at once, as the body's own change, over its 25 ticks
  g.posture = KILO_POSTURE_SIT;
  g.tempo = 1;
  CHECK(mv.build_from(m, KILO_POSTURE_STAND, g) && mv.stages() == 1 && mv.trick().frames[0].dur == 25);
  // up from REST: the hind legs first (the body's own all-at-once comes within 4 mm of the edge)
  g.posture = KILO_POSTURE_STAND;
  CHECK(mv.build_from(m, KILO_POSTURE_REST, g) && mv.stages() == 2 && mv.direct_margin() < 4);
  CHECKF(std::strncmp(mv.plan_text(), "hind shoulders+hind knees, then front", 37) == 0, "%s", mv.plan_text());
  // down to REST: the front first; between sitting and resting: through standing, in one move
  g.posture = KILO_POSTURE_REST;
  CHECK(mv.build_from(m, KILO_POSTURE_STAND, g) && mv.stages() == 2);
  CHECK(mv.build_from(m, KILO_POSTURE_SIT, g) && mv.via() == KILO_POSTURE_STAND && mv.stages() == 3);
  g.posture = KILO_POSTURE_SIT;
  CHECK(mv.build_from(m, KILO_POSTURE_REST, g) && mv.via() == KILO_POSTURE_STAND && mv.stages() == 3);
}

void test_rest_to_stand() {
  Rig r(true);
  r.attach_recorder();
  r.start();
  CHECK(r.log.has("intents ON"));
  r.put(KILO_POSTURE_REST, "rest");
  CHECKF(r.state() == State::Resting, "state %s", state_name(r.state()));
  r.key(0x02, 200);  // B6: be STANDING
  CHECK(r.mind.intent().active() && r.mind.intent().goal() == State::Standing);
  CHECK(r.log.has("intent: be STANDING (remote B6; he is RESTING)"));
  r.run_ms(400);
  CHECKF(r.plays == 1 && r.log.has("step 1: RESTING -> STANDING by the engine's move (hind shoulders+hind knees, "
                                    "then front shoulders+front knees; tempo 100%"),
         "plays %d", r.plays);
  CHECK(r.posture_goal() == -1);  // the trick has the body: no posture goal under it
  r.run_ms(3000);
  CHECKF(!r.mind.intent().active() && r.mind.intent().reached(), "phase %d, ended %s", (int)r.mind.intent().phase(),
         r.mind.intent().ended() != nullptr ? r.mind.intent().ended() : "-");
  CHECK(r.state() == State::Standing && r.posture_goal() == KILO_POSTURE_STAND);
  CHECK(r.log.has("intent: STANDING reached (1 step,"));
  // sit from there (the behaviour's posture goes the same way, here quicker), then rest: one move,
  // through standing
  r.mind.posture("sit", -1, "test", 1.5);
  r.run_ms(4000);
  CHECK(r.state() == State::Sitting && r.plays == 2 && r.posture_goal() == KILO_POSTURE_SIT);
  CHECK(r.log.has("step 1: STANDING -> SITTING by the engine's move (all at once; tempo 150%"));
  r.key(0xA2, 200);  // A7: be RESTING
  r.run_ms(8000);
  CHECKF(r.state() == State::Resting && r.plays == 3 && r.mind.intent().reached(), "state %s plays %d",
         state_name(r.state()), r.plays);
  CHECK(r.log.has("step 1: SITTING -> RESTING by the engine's move (all at once; stand; front shoulders"));
  CHECK(r.log.has("intent: RESTING reached (1 step,"));
  const Intent::Counters& c = r.mind.intent().counters();
  CHECKF(c.asked == 3 && c.reached == 3 && c.moves == 3 && c.gave_up == 0 && c.failed_steps == 0,
         "asked %u reached %u moves %u", (unsigned)c.asked, (unsigned)c.reached, (unsigned)c.moves);
  // asked for where he is: nothing plays, the posture is latched
  r.key(0xA2, 600);
  CHECK(r.plays == 3 && r.posture_goal() == KILO_POSTURE_REST && r.mind.intent().reached());
}

// The body does not end where the model says (whatever is played, he ends sitting): three failed
// steps, then it gives up and leaves him in REST. (A move that ends lying where it meant standing
// cannot be seen: STANDING and RESTING are both level, told apart only by what was asked; the
// estimate's known limit, docs/intent.md.)
void test_failed_steps() {
  Rig r(true);
  r.start();
  r.put(KILO_POSTURE_REST, "rest");
  r.obey = false;
  r.stuck_pitch = kRows[1].pitch;
  r.stuck_roll = kRows[1].roll;  // SITTING's attitude
  r.key(0x02, 200);
  r.run_ms(20000);
  CHECKF(!r.mind.intent().active() && !r.mind.intent().reached() && r.plays == 3, "plays %d active %d", r.plays,
         (int)r.mind.intent().active());
  CHECK(r.log.has("intent: step 1 meant to end STANDING; he is SITTING"));
  CHECK(r.log.has("intent: STANDING given up: three steps did not end where the model said"));
  CHECK(r.mind.intent().counters().failed_steps == 3 && r.mind.intent().counters().gave_up == 1);
}

// On his side: the estimate says so although STAND was asked, and be(STANDING) waits for the body's
// fall reflex (the placeholder edge) instead of playing a move.
void test_on_his_side() {
  Rig r(true);
  r.start();
  r.put(KILO_POSTURE_STAND, "stand");
  r.body.roll = -89 * kDeg;
  r.body.mode = KILO_MODE_FALLEN;
  r.body.event(KILO_EVENT_FELL, -1);
  r.run_ms(1500);
  CHECK(r.state() == State::OnRightSide);
  r.key(0x02, 300);
  CHECK(r.log.has("he is ON_RIGHT_SIDE: waiting for a hand to right him (the body has no get-up)"));
  CHECK(r.mind.intent().phase() == Phase::Reflex && r.plays == 0);
  // a hand rights him while the reflex holds him (the body has no get-up of its own): he stands
  r.run_ms(800);
  r.body.roll = 0;
  r.body.mode = KILO_MODE_IDLE;
  r.body.support = 0xF;
  r.run_ms(2500);
  CHECKF(!r.mind.intent().active() && r.mind.intent().reached() && r.state() == State::Standing && r.plays == 0,
         "phase %d state %s", (int)r.mind.intent().phase(), state_name(r.state()));
  CHECK(r.posture_goal() == KILO_POSTURE_STAND);
  // again, and the reflex lays him down: given up, nothing sent to a fallen body
  r.body.roll = 91 * kDeg;
  r.body.mode = KILO_MODE_FALLEN;
  r.body.event(KILO_EVENT_FELL, 1);
  r.run_ms(1500);
  CHECK(r.state() == State::OnLeftSide);
  r.key(0x02, 300);
  r.body.event(KILO_EVENT_LYING, 0);
  r.run_ms(300);
  CHECK(!r.mind.intent().active() && !r.mind.intent().reached());
  CHECK(r.log.has("the body's fall reflex laid him down (no get-up without the righting experiment): he needs a hand"));
  CHECK(r.plays == 0);
}

// LIMP (after A7's light sleep): REST first, then the move.
void test_from_limp() {
  Rig r(true);
  r.start();
  r.put(KILO_POSTURE_REST, "rest");
  client_posture(r.mind.link(), nullptr, "limp");
  r.run_ms(1500);
  CHECK(r.body.mode == KILO_MODE_LIMP && r.state() == State::Resting);
  r.mind.posture("stand", -1, "test");
  r.run_ms(600);
  CHECK(r.log.has("intent: the servos are off: REST first") && r.plays == 0);
  r.run_ms(6000);
  CHECKF(r.mind.intent().reached() && r.state() == State::Standing && r.plays == 1, "state %s plays %d",
         state_name(r.state()), r.plays);
}

// A walk key while a move plays: the move is stopped and the intent dropped.
void test_cancelled_by_a_walk() {
  Rig r(true);
  r.start();
  r.put(KILO_POSTURE_STAND, "stand");
  r.mind.posture("sit", -1, "test");
  r.run_ms(300);
  CHECK(r.plays == 1 && r.mind.intent().active());
  r.mind.intent().cancel("test");
  r.run_ms(600);
  CHECK(!r.mind.intent().active() && r.stops == 1 && !r.playing);
  CHECK(r.mind.intent().counters().cancelled == 1);
  // another posture (CALIB, C5) is the body's own change and drops a pending intent
  r.mind.posture("stand", -1, "test");
  r.run_ms(100);
  r.mind.posture("calib", -1, "test");
  CHECK(!r.mind.intent().active() && r.posture_goal() == KILO_POSTURE_CALIB);
}

// Someone else starts a trick while an intent waits: the intent is dropped. A trick that was already
// there when the intent was asked (still being stopped) is not a reason.
void test_dropped_by_another_trick() {
  Rig r(true);
  r.start();
  r.put(KILO_POSTURE_STAND, "stand");
  r.body.roll = 89 * kDeg;  // on his side (this body has no fall reflex: the wait would run its 15 s)
  r.run_ms(1500);
  r.mind.posture("stand", -1, "test");
  r.run_ms(300);
  CHECK(r.mind.intent().phase() == Phase::Reflex);
  const char* why = nullptr;
  const tricks::Trick* hi = tricks::find_trick("hi");
  CHECK(hi != nullptr && r.mind.player().start(*hi, false, tricks::kSpeedUnity, &why) != 0);
  r.run_ms(200);
  CHECK(!r.mind.intent().active() && r.log.has("intent: STANDING dropped (a trick started)"));
  // asked while that trick still plays: the intent waits for it instead of dropping itself
  r.body.roll = 0;
  r.run_ms(100);
  CHECK(r.mind.player().busy());
  r.mind.posture("stand", -1, "test");
  r.run_ms(200);
  CHECK(r.mind.intent().active());
  r.run_ms(6000);
  CHECKF(!r.mind.intent().active() && r.mind.intent().reached(), "ended %s",
         r.mind.intent().ended() != nullptr ? r.mind.intent().ended() : "-");
}

}  // namespace

int main(int argc, char** argv) {
  const char* path = "tests/intent_vectors.txt";
  for (int i = 1; i < argc; i++)
    if (std::strstr(argv[i], "intent_vectors")) path = argv[i];
  test_vectors(path);
  test_table();
  test_table_righting();
  test_estimate_in_the_mind();
  test_off_is_as_before();
  test_engine_moves();
  test_rest_to_stand();
  test_failed_steps();
  test_on_his_side();
  test_from_limp();
  test_cancelled_by_a_walk();
  test_dropped_by_another_trick();
  return report("intent");
}
