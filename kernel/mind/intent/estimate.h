// estimate: which resting state Kilo is in, from every TELEMETRY, what the
// mind last asked for and the body model's predictions (intent.gen.h kRows).
// A port of mind/kilo/intent/estimate.py, whose docstring has the rules in
// full; kernel/host/tests/intent_vectors.txt checks the two agree sample
// for sample. In short:
//
// 1. MOVING while the body's mode is a moving one (GAIT, TRICK, BOOT) or it
//    has not been still (every rate under kStillA16) for kStillUs. Under
//    the fall reflex (FALLEN, RECOVERING) he is judged like any other time.
// 2. A fallen state from the attitude alone: ON_END (|pitch| folded about 90
//    deg at least kEndA16), ON_BACK (|roll| over kBackA16), ON_LEFT_SIDE or
//    ON_RIGHT_SIDE (|roll| from kSideA16; roll + is the right side up: he
//    lies on his left).
// 3. Otherwise upright: the states of kRows whose predicted pitch and roll
//    are within their tolerance of the measured FIT. The posture last asked
//    for is a prior, never the answer: it wins if it fits; if it does not
//    but others do, the nearest of those; with nothing asked and several
//    fitting, the state he was in before; nothing fits: UNKNOWN. Nothing
//    asked and the body LIMP: the prior is LIMP (he cannot be standing).
// 4. A new answer must hold kConfirmUs before it replaces the state.
//
// All integer arithmetic on TELEMETRY's a16 (1/16 deg), as the Python's.
// It only reads: nothing here moves Kilo.
#ifndef KILO_MIND_INTENT_ESTIMATE_H
#define KILO_MIND_INTENT_ESTIMATE_H
#include <stdint.h>
#include "intent/intent.gen.h"
#include "intent/table.h"

namespace kilo {
namespace intent {

// One TELEMETRY as the estimate reads it (estimate.py Sample).
struct Sample {
  uint64_t t_us;  // the mind's clock
  int16_t pitch, roll;
  int16_t rate_pitch, rate_roll, rate_yaw;
  uint8_t mode;
  Cmd cmd;
};

// classify()'s answer for a still body.
struct Answer {
  State state;
  uint8_t confidence;  // 0..100
  int16_t err_pitch;   // a16: measured minus the model's for the state (0 where the model has none)
  int16_t err_roll;
};

struct Estimate {
  State state = State::Unknown;
  uint8_t confidence = 0;
  uint64_t since_us = 0;
  int16_t err_pitch = 0, err_roll = 0;
};

// |pitch| folded about 90 deg (the body reflects it past 90 when he is upside down).
int16_t fold_pitch(int16_t pitch);
// The fallen state the attitude alone names; false: upright.
bool attitude_class(int16_t pitch, int16_t roll, State* out);
// The state a still body's attitude says, with the prior cmd and the resting state before (prev).
Answer classify(int16_t pitch, int16_t roll, Cmd cmd, State prev, const Row* rows = kRows, int n = kRowCount);

class Estimator {
 public:
  // Every TELEMETRY. True when the state changed.
  bool feed(const Sample& s);
  const Estimate& estimate() const { return est_; }
  State state() const { return est_.state; }
  // The last upright or fallen state held (MOVING and UNKNOWN do not replace it).
  State resting() const { return resting_; }
  // The body settled (still for kStillUs) at the last sample.
  bool settled() const { return settled_; }
  uint32_t changes() const { return changes_; }

 private:
  Estimate est_;
  State resting_ = State::Unknown;
  bool still_ = false;
  uint64_t still_since_ = 0;
  bool settled_ = false;
  State cand_ = State::Unknown;
  uint64_t cand_since_ = 0;
  bool started_ = false;
  uint32_t changes_ = 0;
};

}  // namespace intent
}  // namespace kilo

#endif
