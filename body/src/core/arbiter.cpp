#include "arbiter.h"
#include "pgm.h"
#include "fx.h"
#include "posture.h"
#include "params.h"

// ---- STREAM (core/stream.h) -----------------------------------------------
// The POSE stream owns every leg joint (canonical k >= 3) while it is on
// (moving, holding, settling) and the head joints its pose drives (j[k] !=
// JOINT_OFF), and supplies this tick's angles in j[] (body_tick's
// stream_joints). The crossfade, layers and safety stage then apply
// unchanged.
// Head layer (P1, doc §3c layer 1): a held head joint (k = 0..2) under IDLE
// steps toward the Pi's target from its last output, at most head_step a16
// per tick (the safety slew still applies after). A head joint the stream's
// pose drives is the stream's (a trick moving the head beats the hold); one
// it leaves to IDLE can be held (a walk looking around).
static inline bool head_holds(const Arbiter& a, uint8_t k, uint8_t own) {
  return k < 3 && a.head_ttl && own == OWN_IDLE && a.head_tgt[k] != JOINT_OFF;
}
// (The last output is within the joint limits and head_step <= 0x3FFF, so
// from +- step fits 16 bits whatever the target; the target is never
// subtracted.)
static inline int16_t head_layer(const Arbiter& a, uint8_t k) {
  int16_t from = a.last_on[k], s = a.head_step, t = a.head_tgt[k];
  if (t > from + s) return (int16_t)(from + s);
  if (t < from - s) return (int16_t)(from - s);
  return t;
}
// Balance layer (doc §3c layer 2, §3f): joint-space owners only. IDLE holds
// the built-in postures, STREAM joint-angle legs (body.cpp runs the
// controller while the next key has BALANCE and lets the corrections decay
// otherwise); a stream of feet and STANCE get the geometric balance through
// the IK instead (body.cpp passes bal = null then, core/stream.h); REFLEX
// never balances.
static inline int16_t balance_layer(uint8_t k, uint8_t own, int16_t base, const Balance* bal) {
  if (!bal || own == OWN_REFLEX) return base;
  return (int16_t)(base + balance_joint(*bal, k));
}
// ---------------------------------------------------------------------------

// a + (b - a) * e, e in Q14. The difference of two a16 angles fits 17 bits,
// so widen before multiplying.
KILO_NOINLINE static int16_t lerp(int16_t a, int16_t b, uint16_t e) {
  return (int16_t)(a + (((int32_t)(b - a) * e + 8192) >> 14));
}

KILO_NOINLINE static void mover_start(Mover& m, uint8_t posture, uint8_t ticks, const int16_t* cur) {
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) m.from[k] = cur[k];
  m.posture = posture;
  m.n = ticks;
  m.i = 0;
}

// legs: STANCE's leg angles this tick (from the IK, stream_joints), or null.
KILO_NOINLINE static int16_t mover_angle(const Mover& m, uint8_t k, const int16_t* legs) {
  if (m.posture == POSTURE_LIMP) return JOINT_OFF;
  int16_t from = m.from[k];
  if (m.posture == POSTURE_HOLD) return from;
  // STANCE: the head and tail of STAND, the legs from the IK.
  int16_t to = (m.posture == POSTURE_STANCE && k >= 3 && legs) ? legs[k - 3] : posture_angle(m.posture, k);
  // From limp there is nothing to interpolate from: the servo's real pose is
  // unknown. Command the target and let the slew limit pace it.
  if (from == JOINT_OFF) return to;
  return lerp(from, to, fx_ease_cos(m.i, m.n));
}

void arbiter_init(Arbiter& a, const int16_t* warm) {
  a.reflex_active = 0;
  a.head_ttl = 0;
  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) {
    int16_t v = warm ? warm[k] : JOINT_OFF;
    a.out[k] = v;
    // Slew reference for a limp joint: REST, the most likely resting pose
    // of an unpowered cat (and where legacy started).
    a.last_on[k] = v == JOINT_OFF ? posture_angle(POSTURE_REST, k) : v;
    a.owner[k] = warm ? OWN_REFLEX : OWN_IDLE;
    a.xn[k] = a.xi[k] = 0;
  }
  // Cold: IDLE is LIMP. Warm: IDLE and REFLEX both hold the read-back pose,
  // REFLEX owning it until the warm-boot hold ends.
  mover_start(a.idle, warm ? POSTURE_HOLD : POSTURE_LIMP, 0, a.out);
  if (warm) arbiter_reflex(a, POSTURE_HOLD, 0);
}

void arbiter_set_idle(Arbiter& a, uint8_t posture, uint8_t ticks, bool restart) {
  if (posture == a.idle.posture && !restart) return;
  mover_start(a.idle, posture, ticks ? ticks : (uint8_t)PARAM(POSTURE_TICKS), a.out);
}

uint8_t arbiter_idle_posture(const Arbiter& a) { return a.idle.posture; }

void arbiter_reflex(Arbiter& a, uint8_t posture, uint8_t ticks) {
  if (a.reflex_active && a.reflex.posture == posture) return;
  mover_start(a.reflex, posture, ticks, a.out);
  a.reflex_active = 1;
}

void arbiter_reflex_release(Arbiter& a) { a.reflex_active = 0; }

bool arbiter_reflex_settled(const Arbiter& a) { return a.reflex.i >= a.reflex.n; }


void arbiter_run(Arbiter& a, const JointCal& cal, uint16_t counts[JOINT_COUNT], const Balance* bal,
                 const int16_t* j, bool stream) {
  const int16_t* legs = (j && !stream) ? j + 3 : 0;  // STANCE's
  a.limited = 0;
  // Unused channels (joints 3-7) stay full-off; driven ones are set below.
  for (uint8_t c = 0; c < JOINT_COUNT; c++) counts[c] = PCA_FULL_OFF;
  if (a.idle.i < a.idle.n) a.idle.i++;
  if (a.reflex_active && a.reflex.i < a.reflex.n) a.reflex.i++;

  for (uint8_t k = 0; k < DRIVEN_COUNT; k++) {
    uint8_t id = joint_id(k);
    // 1. Owner selection, highest priority first.
    uint8_t own;
    if (a.reflex_active)
      own = OWN_REFLEX;
    else if (stream && j && (k >= 3 || j[k] != JOINT_OFF))
      own = OWN_STREAM;
    else
      own = OWN_IDLE;
    // 2. Crossfade on a change of owner, from the last output. STREAM and
    //    REFLEX start from the current pose themselves, so they get none;
    //    everything else fades over XFADE_TICKS.
    //    A head joint held by the Pi counts as owned by PI_HOLD (its base is
    //    replaced in step 3), so taking and releasing the hold are hand-overs
    //    too.
    bool held = head_holds(a, k, own);
    uint8_t tag = held ? (uint8_t)OWN_PI_HOLD : own;
    if (tag != a.owner[k]) {
      uint8_t n = (uint8_t)PARAM(XFADE_TICKS);
      if (own == OWN_STREAM || own == OWN_REFLEX) n = 0;
      a.owner[k] = tag;
      a.xsrc[k] = a.out[k];
      a.xn[k] = n;
      a.xi[k] = 0;
    }
    int16_t raw;
    if (own == OWN_REFLEX) {
      raw = mover_angle(a.reflex, k, legs);
    } else if (own == OWN_STREAM) {
      raw = j[k];
    } else {
      raw = mover_angle(a.idle, k, legs);
    }
    int16_t b = raw;
    if (a.xi[k] < a.xn[k]) {
      a.xi[k]++;
      if (raw != JOINT_OFF && a.xsrc[k] != JOINT_OFF) b = lerp(a.xsrc[k], raw, fx_ease_cos(a.xi[k], a.xn[k]));
    }
    // 3. Layers. Never applied to a limp joint. A held head joint takes the
    //    Pi's target instead of its base, and no balance correction.
    if (b != JOINT_OFF) b = held ? head_layer(a, k) : balance_layer(k, own, b, bal);
    // 4. Safety: limits, then slew against the last powered output, then
    //    counts. Limits come first so the slew never chases an angle the
    //    joint may not reach.
    if (b == JOINT_OFF) {
      a.out[k] = JOINT_OFF;
      continue;
    }
    int16_t t = joint_clamp(id, b);
    int16_t d = (int16_t)(t - a.last_on[k]);
    int16_t slew = PARAM(SLEW_MAX_A16);
    if (d > slew) d = slew;
    if (d < -slew) d = (int16_t)-slew;
    int16_t o = (int16_t)(a.last_on[k] + d);
    if (o != b) a.limited |= (uint8_t)(k < 3 ? 0x10u : 1u << ((k - 3) & 3u));
    a.last_on[k] = a.out[k] = o;
    counts[joint_channel(id)] = joint_counts(cal, id, o);
  }
  // The hold lasts head_ttl runs from the last SET_HEAD (HEAD_HOLD_MS).
  if (a.head_ttl) a.head_ttl--;
}
