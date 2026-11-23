#include "stream.h"
#include <string.h>
#include "fx.h"
#include "ik.h"
#include "params.h"
#include "pgm.h"

#define SLEW_POSE 8    // body pose pitch/roll: 0.5 deg per tick
#define POSE_MAX 480   // +-30 deg
#define SHARE_FULL 4u  // a leg's whole share of the correction, quarters
#define FLAGS_KNOWN (POSE_JOINT_LEGS | KILO_POSE_FLAGS_BALANCE | KILO_POSE_FLAGS_FALL_OK | KILO_POSE_FLAGS_EASE)

static_assert(sizeof(PoseKey) == 30, "PoseKey: 30 B");

static inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// v toward t by at most s.
KILO_NOINLINE static int16_t slew(int16_t v, int16_t t, int16_t s) {
  if (t > v + s) return (int16_t)(v + s);
  if (t < v - s) return (int16_t)(v - s);
  return t;
}

void stream_init(Stream& s) {
  memset(&s, 0, sizeof s);
  stream_stance_default(s);
}

void stream_stance_default(Stream& s) { s.h = (int16_t)(((int32_t)ik_reach_q2() * KILO_GAIT_STANCE_H_256) >> 8); }

void stream_pose_goal(Stream& s, const uint8_t* p) {
  for (uint8_t i = 0; i < 2; i++) {
    int16_t v = (int16_t)rd16(p + 2 * i);
    s.pose_g[i] = v > POSE_MAX ? POSE_MAX : v < -POSE_MAX ? (int16_t)-POSE_MAX : v;
  }
  s.dh_g = (int8_t)p[4];
  s.xs_g = (int8_t)p[5];
}

void stream_pose_step(Stream& s) {
  for (uint8_t i = 0; i < 2; i++) s.pose[i] = slew(s.pose[i], s.pose_g[i], SLEW_POSE);
  s.dh = (int8_t)slew(s.dh, s.dh_g, 1);
  s.xs = (int8_t)slew(s.xs, s.xs_g, 1);
}

// Payload offsets (kilo_pose_t; link.cpp static_asserts them): epoch 0,
// tick 1, flags 3, leg 4 (8 x i16: per leg its two values), head 20
// (3 x i16), pitch 26, roll 28, support 30.
uint8_t stream_check(const uint8_t* p) {
  uint8_t fl = p[3];
  if (fl & (uint8_t)~FLAGS_KNOWN) return 2;  // GOAL_BAD_ARG (core/body.h)
  for (uint8_t i = 0; i < 4; i++) {
    int16_t a = (int16_t)rd16(p + 4 + 4 * i), b = (int16_t)rd16(p + 6 + 4 * i);
    if (fl & (1u << i)) {
      if (a == JOINT_OFF || b == JOINT_OFF) return 2;
    } else if (a < -480 || a > 480 || b < 60 || b > 440) {
      return 2;
    }
  }
  // pitch, roll within +-30 deg: the reference adds SET_BODY_POSE's +-30 deg
  // and the balance subtracts it from the attitude, all in int16.
  for (uint8_t i = 0; i < 2; i++) {
    int16_t a = (int16_t)rd16(p + 26 + 2 * i);
    if (a < -POSE_MAX || a > POSE_MAX) return 2;
  }
  return (p[30] & 0xF0u) ? 2 : 0;
}

// The payload as a key: legs from (a, b) pairs into the shoulder and knee
// slots.
static void unpack(PoseKey& k, const uint8_t* p) {
  k.tick = rd16(p + 1);
  k.flags = p[3];
  k.support = p[30];
  for (uint8_t i = 0; i < 3; i++) k.v[i] = (int16_t)rd16(p + 20 + 2 * i);
  for (uint8_t i = 0; i < 4; i++) {
    k.v[3 + i] = (int16_t)rd16(p + 4 + 4 * i);
    k.v[7 + i] = (int16_t)rd16(p + 6 + 4 * i);
  }
  k.pitch = (int16_t)rd16(p + 26);
  k.roll = (int16_t)rd16(p + 28);
}

bool stream_push(Stream& s, const uint8_t* p, uint16_t now, const int16_t out[DRIVEN_COUNT]) {
  uint16_t t = rd16(p + 1);
  uint16_t ahead = (uint16_t)(t - now);
  uint8_t i = 0;
  // The key's place: after every key due before it.
  while (i < s.n && (uint16_t)(s.q[i].tick - now) < ahead) i++;
  bool same = i < s.n && s.q[i].tick == t, was_empty = s.n == 0;
  if (ahead == 0 || ahead > POSE_MAX_AHEAD || (!same && i == POSE_DEPTH)) {
    if (s.dropped < 0xFFFF) s.dropped++;
    return false;
  }
  if (!same) {
    if (s.n == POSE_DEPTH) s.n--;  // the latest key makes room
    memmove(&s.q[i + 1], &s.q[i], (uint8_t)(s.n - i) * sizeof(PoseKey));
    s.n++;
  }
  unpack(s.q[i], p);
  // Nothing was queued: the move toward this key starts now, from the pose
  // held (not from the last key reached, perhaps long ago).
  if (was_empty) s.cur.tick = now;
  s.fall_ok = p[3] & KILO_POSE_FLAGS_FALL_OK;
  s.rx_tick = now;
  if (!s.state) {
    // The stream takes the joints where they are: the first segment starts
    // from their outputs now, every leg as joint angles.
    PoseKey& c = s.cur;
    memcpy(c.v, out, sizeof c.v);
    c.tick = now;
    c.flags = POSE_JOINT_LEGS;
    c.support = 0xF;
    c.pitch = c.roll = 0;
    s.support = 0xF;
    s.state = ST_PLAY;
    s.share = 0x4444;  // from STANCE: every leg carries its whole share
  }
  return true;
}

void stream_stop(Stream& s) {
  s.state = ST_OFF;
  s.n = 0;
  s.held = 0;
  s.fall_ok = 0;
}

bool stream_balance(const Stream& s) {
  return ((s.n ? s.q[0].flags : s.cur.flags) & KILO_POSE_FLAGS_BALANCE) != 0;
}

int16_t stream_ref(const Stream& s, uint8_t axis) {
  int16_t r = s.pose[axis];
  if (s.state) r = (int16_t)(r + (axis ? s.cur.roll : s.cur.pitch));
  return r;
}

// c + (t - c) * w, w Q14 (the difference of two int16 fits 17 bits).
KILO_NOINLINE static int16_t toward(int16_t c, int16_t t, uint16_t w) {
  return (int16_t)(c + (((int32_t)(t - c) * w + 8192) >> 14));
}

// A foot (x, z q2) as joint angles (the plain IK: no body pose, no
// correction): a leg changing between a foot and joint angles.
static void foot_angles(uint8_t leg, const int16_t* v, int16_t* sh, int16_t* kn) {
  ik_leg(leg, v[3 + leg], v[7 + leg], sh, kn);
}

// One tick toward the next key q[0]: the fraction w (Q14) of what is left.
static void advance(Stream& s, uint16_t tick, const int16_t out[DRIVEN_COUNT]) {
  PoseKey& c = s.cur;
  const PoseKey& k = s.q[0];
  uint16_t m = (uint16_t)(k.tick - c.tick), i = (uint16_t)(tick - c.tick);
  uint16_t w = Q14_ONE;
  if (i < m) {
    if (k.flags & KILO_POSE_FLAGS_EASE) {
      uint16_t e0 = fx_ease_cos((uint16_t)(i - 1), m), e1 = fx_ease_cos(i, m);
      w = (uint16_t)(((uint32_t)(e1 - e0) << 14) / (uint32_t)(Q14_ONE - e0));
    } else {
      w = (uint16_t)(Q14_ONE / (uint16_t)(m - i + 1));
    }
  }
  for (uint8_t j = 0; j < 3; j++) {
    if (k.v[j] == JOINT_OFF) {
      c.v[j] = JOINT_OFF;  // left to IDLE
      continue;
    }
    if (c.v[j] == JOINT_OFF) c.v[j] = out[j];
    c.v[j] = toward(c.v[j], k.v[j], w);
  }
  for (uint8_t l = 0; l < 4; l++) {
    uint8_t bit = (uint8_t)(1u << l);
    int16_t ts = k.v[3 + l], tk = k.v[7 + l];
    if ((c.flags | k.flags) & bit) {  // in joint space
      if (!(k.flags & bit)) foot_angles(l, k.v, &ts, &tk);
      if (!(c.flags & bit)) foot_angles(l, c.v, &c.v[3 + l], &c.v[7 + l]);
      c.flags |= bit;
    }
    c.v[3 + l] = toward(c.v[3 + l], ts, w);
    c.v[7 + l] = toward(c.v[7 + l], tk, w);
  }
  c.pitch = toward(c.pitch, k.pitch, w);
  c.roll = toward(c.roll, k.roll, w);
  s.support = (uint8_t)(c.support & k.support);
  if (i < m) return;
  // Reached: the key is the pose now, the next segment starts here.
  c = k;
  s.support = c.support;
  s.n--;
  memmove(&s.q[0], &s.q[1], s.n * sizeof(PoseKey));
}

// A foot in the body's plane, doubled so that half the hip spacing stays
// whole (mm x 2): X forward from the body's centre (hips at +-Lb/2, the foot
// x ahead of its hip), Y to the right (hips at +-W/2). |X| <= 200 + 240,
// |Y| <= 150 at the parameters' limits.
KILO_NOINLINE static void foot_xy2(const Stream& s, uint8_t i, int16_t* X, int16_t* Y) {
  int16_t lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM);
  *X = (int16_t)((leg_front(i) ? lb : -lb) + ((s.cur.v[3 + i] + 1) >> 1));
  *Y = leg_left(i) ? (int16_t)-w : w;
}

// a x b for 2-D vectors in doubled mm, int16 x int16 products.
KILO_NOINLINE static int32_t cross(int16_t ax, int16_t ay, int16_t bx, int16_t by) {
  return (int32_t)ax * by - (int32_t)ay * bx;
}

// A foot's depth in whole mm.
static inline int16_t z_mm(const Stream& s, uint8_t i) { return (int16_t)((s.cur.v[7 + i] + 2) >> 2); }

// The ground's depth (mm below the hip) at foot `i` on the plane through
// the three support feet a, b, c (the leaning body on three feet: the walk
// tilts the body up to ~14 deg, so the support depths differ by up to
// ~16 mm). Barycentric: P - A = s (B - A) + t (C - A) with s = (P-A)x(C-A) / D,
// t = (B-A)x(P-A) / D, D = (B-A)x(C-A); z = zA + s (zB - zA) + t (zC - zA).
// In doubled mm the differences are <= 880 (X) and 300 (Y), each cross
// product <= 2 x 880 x 300 = 528000 and the numerator <= 2 x 95 x 528000 =
// 100 M: int32. The quotient (s, t may be large for a thin triangle) is
// capped at 127 mm before it goes back to int16, then the depth is rounded
// to the nearest mm and kept in 30..95 mm. -1 when the three are on one
// line (D = 0: no plane, the caller takes the mean).
KILO_NOINLINE static int16_t ground_plane(const Stream& s, uint8_t i, const uint8_t abc[3]) {
  int16_t X[4], Y[4], z[3];  // [0..2] the support feet, [3] foot i
  for (uint8_t k = 0; k < 4; k++) {
    uint8_t l = k < 3 ? abc[k] : i;
    foot_xy2(s, l, &X[k], &Y[k]);
    if (k < 3) z[k] = z_mm(s, l);
  }
  for (uint8_t k = 1; k < 4; k++) X[k] = (int16_t)(X[k] - X[0]), Y[k] = (int16_t)(Y[k] - Y[0]);
  int32_t d = cross(X[1], Y[1], X[2], Y[2]);
  if (!d) return -1;
  int32_t num = cross(X[3], Y[3], X[2], Y[2]) * (int16_t)(z[1] - z[0]) +
                cross(X[1], Y[1], X[3], Y[3]) * (int16_t)(z[2] - z[0]);
  if (d < 0) d = -d, num = -num;
  bool neg = num < 0;
  int32_t q32 = ((neg ? -num : num) + d / 2) / d;
  int16_t q = (int16_t)(q32 > 127 ? 127 : q32);
  q = (int16_t)(z[0] + (neg ? -q : q));
  return q < 30 ? 30 : q > 95 ? 95 : q;
}

// The settle: every foot-mode foot off the ground goes down (or up) toward
// the ground at its x by SETTLE_Q2 per tick. True when every leg is a foot
// and all four are down. The ground: the plane through the support feet
// when three are down (the body leans on them: a foot set at their mean
// depth would land above or into the floor), else the support feet's mean
// depth, else STANCE's default height when none are down.
static bool settle(Stream& s) {
  PoseKey& c = s.cur;
  int16_t sum = 0;
  uint8_t n = 0, abc[3];
  for (uint8_t i = 0; i < 4; i++)
    if ((c.support >> i) & 1u) {
      sum = (int16_t)(sum + z_mm(s, i));
      if (n < 3) abc[n] = i;
      n++;
    }
  int16_t mean = n ? (int16_t)((sum + n / 2) / n) : (int16_t)((((int32_t)ik_reach_q2() * KILO_GAIT_STANCE_H_256) >> 8) / 4);
  bool all = !(c.flags & POSE_JOINT_LEGS);
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t bit = (uint8_t)(1u << i);
    if ((c.support | c.flags) & bit) continue;
    int16_t p = n == 3 ? ground_plane(s, i, abc) : -1;
    int16_t g = (int16_t)((p < 0 ? mean : p) * 4);
    int16_t z = c.v[7 + i];
    if (z + SETTLE_Q2 < g)
      z = (int16_t)(z + SETTLE_Q2), all = false;
    else if (z > g + SETTLE_Q2)
      z = (int16_t)(z - SETTLE_Q2), all = false;
    else
      z = g, c.support |= bit;
    c.v[7 + i] = z;
  }
  s.support = c.support;
  return all;
}

uint8_t stream_step(Stream& s, uint16_t tick, const int16_t out[DRIVEN_COUNT]) {
  if (!s.state) return 0;
  uint8_t ev = 0;
  if (s.n) {
    s.state = ST_PLAY;
    s.held = 0;
    advance(s, tick, out);
  } else {
    if (s.state == ST_PLAY) {  // the last key reached, none queued: hold
      s.state = ST_HOLD;
      if (s.underruns < 0xFFFF) s.underruns++;
      ev = STREAM_EV_UNDERRUN;
    }
    if (s.state == ST_HOLD && ++s.held > POSE_HOLD_TICKS) s.state = ST_SETTLE;
    if (s.state == ST_SETTLE && settle(s)) ev |= STREAM_EV_SETTLED;
  }
  // Each leg's share of the correction: a quarter per tick toward its
  // support bit (stream.h).
  uint16_t sh = 0;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t q = (uint8_t)((s.share >> (4 * i)) & 0xFu);
    if ((s.support >> i) & 1u) {
      if (q < SHARE_FULL) q++;
    } else if (q) {
      q--;
    }
    sh |= (uint16_t)((uint16_t)q << (4 * i));
  }
  s.share = sh;
  return ev;
}

// (L/2) sin(a) in q2 for a length L in mm, rounded (truncating would lose up
// to 0.25 mm per leg, 0.2 deg of body tilt).
KILO_NOINLINE static int16_t half_sin_q2(int16_t len_mm, int16_t a) {
  return (int16_t)(((int32_t)len_mm * 2 * fx_sin(a) + 8192) >> 14);
}

void stream_joints(Stream& s, int16_t corr_roll, int16_t corr_pitch, int16_t j[DRIVEN_COUNT]) {
  bool play = s.state != ST_OFF;
  const PoseKey& c = s.cur;
  // Pose terms (doc §3d): the raised end or side gets the longer legs.
  // STANCE: pose and correction together on all four legs. A stream's
  // feet: the pose on all four, the correction on each leg by its share.
  int16_t pitch = s.pose[0], roll = s.pose[1];
  if (s.geo && !play) {
    pitch = (int16_t)(pitch + corr_pitch);
    roll = (int16_t)(roll + corr_roll);
  }
  int16_t lb = PARAM(LEG_LB_MM), w = PARAM(LEG_W_MM);
  int16_t dz_p = half_sin_q2(lb, pitch), dz_r = half_sin_q2(w, roll);
  int16_t dc_p = 0, dc_r = 0;
  if (s.geo && play) {
    dc_p = half_sin_q2(lb, corr_pitch);
    dc_r = half_sin_q2(w, corr_roll);
  }
  for (uint8_t i = 0; i < 3; i++) j[i] = play ? c.v[i] : JOINT_OFF;
  s.ik_cut = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (play && ((c.flags >> i) & 1u)) {
      j[3 + i] = c.v[3 + i];
      j[7 + i] = c.v[7 + i];
      continue;
    }
    int16_t ep = leg_front(i) ? -1 : 1, sr = leg_left(i) ? -1 : 1;
    int16_t x = (int16_t)(s.xs * 4);
    int16_t z = (int16_t)(s.dh * 4 + ep * dz_p + sr * dz_r);
    if (play) {
      x = (int16_t)(x + c.v[3 + i]);
      z = (int16_t)(z + c.v[7 + i]);
      int16_t share = (int16_t)((s.share >> (4 * i)) & 0xFu);
      z = (int16_t)(z + ((ep * dc_p + sr * dc_r) * share) / (int16_t)SHARE_FULL);
    } else {
      z = (int16_t)(z + s.h);
    }
    if (!ik_leg(i, x, z, &j[3 + i], &j[7 + i])) s.ik_cut |= (uint8_t)(1u << i);
  }
}
