#include "body.h"
#include <string.h>
#include "fx.h"
#include "posture.h"
#include "pgm.h"

void body_init(Body& b, Config* cfg, const int16_t* warm_angles, bool imu_ok, uint8_t epoch) {
  // Everything starts at zero (queues empty, counters 0, filter unseeded,
  // watchdog unarmed) except what is set below. The epoch is the boot epoch
  // from the reset record (core/reset.h), never the previous run's.
  memset(&b, 0, sizeof b);
  b.epoch = epoch;
  b.cfg = cfg;
  b.att.gz = Q14_ONE;  // upright until the first packet says otherwise
  b.imu_fault = imu_ok ? 0 : 1;
  balance_init(b.bal);
  stream_init(b.stream);
  // Warm boot: the PCA9685 kept driving the servos through our reset; keep
  // exactly that pose until we know which way is up. Cold boot: the servo
  // pose is unknown, so stay limp until the Pi asks.
  arbiter_init(b.arb, warm_angles);
  b.holder = warm_angles ? RH_WARM_HOLD : RH_NONE;
}

KILO_NOINLINE static void event(Body& b, uint8_t code, int16_t arg) { events_push(b.ev, code, arg, b.tick, b.epoch); }

// The POSE stream ends (a posture, an epoch bump, a reflex: arg 0; the
// settle after an underrun: arg 1). IDLE takes the joints in its posture
// (STANCE after a stream of feet, else the one it had) from the current
// output (the arbiter's crossfade). The corrections restart from zero under
// IDLE (the crossfade starts from the corrected output: nothing jumps).
// One STREAM_STOPPED per stream. True if a stream was on.
static bool stream_end(Body& b, int16_t arg) {
  if (!b.stream.state) return false;
  stream_stop(b.stream);
  if (!b.stream.geo) balance_reset(b.bal);
  event(b, EV_STREAM_STOPPED, arg);
  return true;
}

void body_epoch_bump(Body& b) {
  // A new epoch makes every goal the Pi sent before this moment stale, so
  // a goal re-sent from before a fall or a link loss cannot restart motion.
  // Latched motion goals stop here: the head hold, the POSE stream and the
  // body pose (it slews back to neutral). The IDLE posture is not a motion
  // goal, but its balance flag is: the Pi re-asserts it with the new epoch.
  // The stream's STREAM_STOPPED carries the new epoch.
  b.epoch++;
  Stream& f = b.stream;
  stream_end(b, 0);  // IDLE takes the joints where they are
  f.pose_g[0] = f.pose_g[1] = 0;
  f.dh_g = f.xs_g = 0;
  b.balance_req = 0;
  // SET_BALANCE's enable is a goal too (spec: goals go stale at a bump); a
  // client that set balance on and died must not leave every later STAND
  // balancing (P1 hardware finding B1). The gains stay: not motion.
  b.bal.enable = 0;
  arbiter_head_release(b.arb);
}

// Take the REFLEX owner for `who`, unless a more important reflex holds it.
KILO_NOINLINE static bool take_reflex(Body& b, uint8_t who, uint8_t posture, uint8_t ticks) {
  if (b.holder > who) return false;
  b.holder = who;
  arbiter_reflex(b.arb, posture, ticks);
  return true;
}

KILO_NOINLINE static void release_reflex(Body& b, uint8_t who) {
  if (b.holder != who) return;
  b.holder = RH_NONE;
  arbiter_reflex_release(b.arb);
}

// Lie down, then limp after `limp_ticks` (each reflex owns its timer). If
// already limp, stay limp: powering servos from an unknown pose to lie down
// would be worse than doing nothing.
static void rest_then_limp(Body& b, uint8_t who, uint16_t* timer, uint16_t limp_ticks) {
  if (arbiter_idle_posture(b.arb) == POSTURE_LIMP) {
    take_reflex(b, who, POSTURE_LIMP, 0);
    *timer = 0;
    return;
  }
  // The IDLE posture follows, so that releasing the reflex later is seamless.
  arbiter_set_idle(b.arb, POSTURE_REST, REST_MOVE_TICKS);
  *timer = take_reflex(b, who, POSTURE_REST, REST_MOVE_TICKS) ? (uint16_t)(REST_MOVE_TICKS + limp_ticks) : 0;
}

static void limp_now(Body& b, uint8_t who) {
  arbiter_set_idle(b.arb, POSTURE_LIMP, 0);
  take_reflex(b, who, POSTURE_LIMP, 0);
}

// Count a reflex's LIMP timer down; when it fires, go limp.
static void limp_timer_step(Body& b, uint8_t who, uint16_t* timer) {
  if (*timer && --*timer == 0 && b.holder == who) limp_now(b, who);
}

// ---- The fall reflex (doc §3g; the state machine is described in reflex.h) ----
static void fall_lying(Body& b) {
  b.fall.state = FS_LYING;
  rest_then_limp(b, RH_FALL, &b.fall.timer, FALL_LIMP_TICKS);
  event(b, EV_LYING, b.fall.axis);
}

static void fall_step(Body& b, bool fresh) {
  Fall& f = b.fall;
  // A more important reflex (watchdog REST, low battery) took the joints:
  // it owns them now; detection re-arms once they are released.
  if (f.state != FS_UPRIGHT && b.holder != RH_FALL) f.state = FS_UPRIGHT;
  if (f.state == FS_UPRIGHT) {
    uint8_t p = arbiter_idle_posture(b.arb);
    if (b.imu_fault || b.holder != RH_NONE || body_fall_suppressed(b) || p == POSTURE_LIMP ||
        p == POSTURE_CALIB) {
      f.over = 0;
      return;
    }
    if (!fresh) return;
    int8_t arg = fall_detect(f.over, b.att);
    if (!arg) return;
    // Fallen: every goal sent before now is stale (a stream ends), the
    // REFLEX holds the pose until the body is still.
    body_epoch_bump(b);
    b.holder = RH_FALL;
    arbiter_reflex(b.arb, POSTURE_HOLD, 0);
    f.state = FS_FALLEN;
    f.axis = arg;
    f.still = 0;
    event(b, EV_FELL, arg);
    notes_push(b.notes, 20, 15);  // a chirp (legacy beeps note 20 on a fall)
    return;
  }
  if (f.state == FS_LYING) {
    limp_timer_step(b, RH_FALL, &f.timer);
    if (!f.timer) {  // limp: let go (detection stays off in LIMP)
      release_reflex(b, RH_FALL);
      f.state = FS_UPRIGHT;
    }
    return;
  }
  // FALLEN. Without the IMU (which also stops fresh samples) nothing can be
  // judged: lie down now.
  if (b.imu_fault) return fall_lying(b);
  if (!fresh) return;
  f.still = fall_still(b.att) ? (uint8_t)(f.still + 1) : 0;
  if (f.still >= FALL_STILL_TICKS) fall_lying(b);
}

void body_link_frame(Body& b) {
  if (b.wd.stage) {
    event(b, EV_LINK_RESTORED, 0);
    // Stay in the current posture (IDLE already holds it); the Pi must send
    // a goal with the new epoch to move again.
    release_reflex(b, RH_WATCHDOG);
  }
  b.wd.armed = 1;
  b.wd.stage = 0;
  b.wd.since = 0;
  b.wd.limp_timer = 0;
}

static void watchdog_step(Body& b) {
  LinkWatchdog& w = b.wd;
  if (!w.armed) return;  // never heard from the Pi since boot: nothing to stop
  if (w.since < 0xFFFF) w.since++;
  if (w.stage == 0 && w.since >= WD_STOP_TICKS) {
    w.stage = 1;
    body_epoch_bump(b);  // also stops the POSE stream, the head hold
    event(b, EV_WATCHDOG, 0);
  }
  // (Stage 1's epoch bump ended a POSE stream; its queue had run dry and
  // its feet settled long before: link.pose_hold_ticks.)
  if (w.stage == 1 && w.since >= WD_REST_TICKS) {
    w.stage = 2;
    rest_then_limp(b, RH_WATCHDOG, &w.limp_timer, WD_LIMP_TICKS);
  }
  if (w.stage == 2) {
    limp_timer_step(b, RH_WATCHDOG, &w.limp_timer);
    if (!w.limp_timer) w.stage = 3;
  }
}

static void warm_hold_step(Body& b, bool imu_fresh) {
  if (b.holder != RH_WARM_HOLD) return;
  b.hold_ticks++;
  if (!(imu_fresh || b.hold_ticks >= WARM_HOLD_TICKS)) return;
  if (b.wd.armed && b.wd.since < WD_STOP_TICKS) {
    // The Pi is already talking to us: hand the held pose to IDLE (which
    // holds the same snapshot) and let the Pi decide.
    release_reflex(b, RH_WARM_HOLD);
    return;
  }
  // Nobody is talking to us: lie down under the watchdog policy (REST, then
  // LIMP 10 s later). The body never stands without the Pi.
  b.holder = RH_NONE;
  b.wd.armed = 1;
  b.wd.stage = 2;
  b.wd.since = WD_REST_TICKS;
  rest_then_limp(b, RH_WATCHDOG, &b.wd.limp_timer, WD_LIMP_TICKS);
}

static void battery_step(Body& b, uint16_t counts) {
  BattLevel prev = b.batt.level;
  BattLevel lvl = battery_sample(b.batt, battery_counts_to_mv(counts, b.cfg->img.batt_mv_per_64));
  if (lvl == prev) return;
  if (lvl > prev) {
    if (lvl == BATT_CRITICAL) {
      limp_now(b, RH_BATTERY);
      b.batt_limp_timer = 0;
    } else if (lvl == BATT_LOW) {
      body_epoch_bump(b);
      rest_then_limp(b, RH_BATTERY, &b.batt_limp_timer, BATT_LIMP_TICKS);
      // Low-battery chirp: the legacy melodyLowBattery opening, short.
      notes_push(b.notes, 15, 10);
      notes_push(b.notes, 11, 10);
      notes_push(b.notes, 13, 10);
    }
    event(b, EV_LOW_BATTERY, lvl);
    return;
  }
  // Recovering (CRITICAL never does: it is latched).
  if (prev >= BATT_LOW && lvl < BATT_LOW) {
    b.batt_limp_timer = 0;
    release_reflex(b, RH_BATTERY);  // IDLE holds REST or LIMP, as set above
  }
  if (lvl == BATT_OK) event(b, EV_BATTERY_OK, 0);
}

static void imu_step(Body& b, const TickIn& in) {
  if (in.imu == IMU_FRESH) {
    b.att = in.att;
    b.imu_stale_run = 0;
    b.imu_err_run = 0;
    return;
  }
  if (in.imu == IMU_ERR) {
    if (b.imu_err_run < 255) b.imu_err_run++;
  } else {
    if (b.imu_stale_run < 255) b.imu_stale_run++;
    if (b.imu_stale_total < 0xFFFF) b.imu_stale_total++;
  }
  if (!b.imu_fault && (b.imu_stale_run >= IMU_STALE_TICKS || b.imu_err_run >= IMU_ERR_RUN)) {
    // Balance and fall detection switch off (P1); re-init only while LIMP.
    b.imu_fault = 1;
    event(b, EV_IMU_FAULT, 0);
  }
}

void body_imu_recovered(Body& b) {
  b.imu_fault = 0;
  b.imu_stale_run = b.imu_err_run = 0;
  event(b, EV_IMU_OK, 0);
}

void body_tick(Body& b, const TickIn& in, uint16_t counts[JOINT_COUNT]) {
  b.tick++;
  // Step 2 happened in the caller (attitude_from_packet); keep the result.
  imu_step(b, in);
  // Step 4: reflexes and epoch bookkeeping, most important last so that it
  // wins the REFLEX owner within this same tick.
  warm_hold_step(b, in.imu == IMU_FRESH);
  fall_step(b, in.imu == IMU_FRESH);
  watchdog_step(b);
  if (in.batt_valid) battery_step(b, in.batt_counts);
  limp_timer_step(b, RH_BATTERY, &b.batt_limp_timer);
  // The POSE stream (core/stream.h). A reflex ends it at once. The pose for
  // this tick (or the held one), the underrun's event, and the hand-over to
  // IDLE STANCE when a stream of feet has settled every foot down.
  Stream& f = b.stream;
  if (b.arb.reflex_active) stream_end(b, 0);
  stream_pose_step(f);
  uint8_t fev = stream_step(f, b.tick, b.arb.last_on);
  if (fev & STREAM_EV_UNDERRUN) event(b, EV_POSE_UNDERRUN, (int16_t)(uint16_t)(b.tick - f.rx_tick));
  if (fev & STREAM_EV_SETTLED) {
    stream_stance_default(f);
    stream_end(b, 1);
  }
  uint8_t p = arbiter_idle_posture(b.arb);
  bool on = stream_on(f);
  // The balance (doc §3c layer 2, §3f). Joint-space for IDLE SIT and STAND
  // once their move has settled; never under REFLEX (which also covers the
  // warm-boot hold) and never in LIMP, CALIB, ZERO or REST (lying on the
  // belly there is nothing to balance). Outside those the corrections are
  // zeroed at once: the reflex or the new posture's mover starts from the
  // corrected output, so nothing jumps. Inside, a request turned off or an
  // IMU_FAULT lets them decay through the filter and slew.
  //
  // A stream with joint-angle legs balances in joint space while its next
  // key has BALANCE, against the key's predicted attitude; otherwise the
  // corrections decay through the filter and slew.
  //
  // A stream of feet and IDLE STANCE balance geometrically (§3f): the
  // corrections tilt the body through the IK's leg lengths instead of being
  // added to joints. The controller sees the residual: what the body reads
  // less the correction's own effect and the attitude the legs are told to
  // give it (measured - corr - ref, ref = stream_ref: the pose, plus the
  // stream's prediction for this tick while it plays). So corr = -Kp *
  // residual: on a slope at Kp ~1 the body ends at the reference to within
  // the dead band, with no integrator to wind up (corr is bounded by the
  // clamp), and the loop stays stable for any leg response between 0 and
  // 2x the model's. Leaving it for a joint-space owner zeroes the
  // corrections (they mean something else there).
  //
  // With a foot off the ground (this tick's support bits) the body can
  // tip: about a diagonal on two feet, and on three as soon as a shove, a
  // lag or a model error carries the mass over the triangle's edge. Then
  // what it reads is the tip (90 ms time constant), and the corrections,
  // which can only act through the feet it is pivoting on, push it further
  // over (the floor, 2026-10-14; the tipping plant, test_stream.cpp "three
  // feet"). So the corrections are held while any foot is up, neither
  // updated nor decayed, and for BAL_HOLD_TICKS after all four are down
  // again (the body still settles from the step). bal.hold counts that
  // down. The loop trims on four feet; on three the plan's own lean holds
  // the mass inside the triangle.
  bool joints = on && !stream_feet(f) && !b.arb.reflex_active;
  bool geo = !b.arb.reflex_active && !joints && (on || (p == POSTURE_STANCE && !arbiter_idle_moving(b.arb)));
  if (f.geo != geo) balance_reset(b.bal);
  f.geo = geo;
  int16_t ref_r = 0, ref_p = 0;
  if (joints) {
    ref_r = f.cur.roll;
    ref_p = f.cur.pitch;
    balance_step(b.bal, stream_balance(f) && !b.imu_fault, b.att, ref_r, ref_p);
  } else if (geo) {
    bool want = (b.balance_req || b.bal.enable) && !b.imu_fault;
    if (on && f.support != 0xF)
      b.bal.hold = BAL_HOLD_TICKS + 1;
    else if (b.bal.hold)
      b.bal.hold--;
    ref_p = stream_ref(f, 0);
    ref_r = stream_ref(f, 1);
    if (want && b.bal.hold) {
      b.bal.active = 1;
    } else {
      Attitude res = b.att;
      res.pitch = (int16_t)(res.pitch - b.bal.ax[BAL_PITCH].corr - ref_p);
      res.roll = (int16_t)(res.roll - b.bal.ax[BAL_ROLL].corr - ref_r);
      balance_step(b.bal, want, res, 0, 0);
    }
  } else if (b.arb.reflex_active || arbiter_idle_moving(b.arb) || !(p == POSTURE_SIT || p == POSTURE_STAND)) {
    balance_reset(b.bal);
  } else {
    bool want = (b.balance_req || b.bal.enable) && !b.imu_fault;
    ref_r = posture_roll(p);
    ref_p = posture_pitch(p);
    balance_step(b.bal, want, b.att, ref_r, ref_p);
  }
  // TELEMETRY ref_*: the reference while the controller runs, else 0.
  b.bal.ref[BAL_ROLL] = b.bal.active ? ref_r : 0;
  b.bal.ref[BAL_PITCH] = b.bal.active ? ref_p : 0;
  // This tick's angles (the stream's pose, or STANCE's legs).
  int16_t j[DRIVEN_COUNT];
  bool ik = on || p == POSTURE_STANCE;
  if (ik) stream_joints(f, b.bal.ax[BAL_ROLL].corr, b.bal.ax[BAL_PITCH].corr, j);
  // Steps 5-6: owners, crossfade, layers, limits, slew, counts.
  arbiter_run(b.arb, b.cfg->img.cal, counts, geo ? 0 : &b.bal, ik ? j : 0, on);
  if (ik) b.arb.limited |= f.ik_cut;
}

GoalResult body_goal_posture(Body& b, uint8_t epoch, uint8_t posture, uint8_t ticks, uint8_t flags) {
  if (posture >= POSTURE_COUNT) return GOAL_BAD_ARG;
  // LIMP and REST are always allowed: they are how the Pi makes Kilo safe.
  bool safe = posture == POSTURE_LIMP || posture == POSTURE_REST;
  if (!safe && epoch != b.epoch) return GOAL_STALE;
  if (body_fallen(b)) {
    // The fall reflex holds the joints until it lets go; only LIMP (the Pi's
    // emergency stop) ends it.
    if (posture != POSTURE_LIMP) return GOAL_FALLEN;
    b.fall.state = FS_UPRIGHT;
    release_reflex(b, RH_FALL);
  }
  if (!safe && b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  if ((posture == POSTURE_STAND || posture == POSTURE_STANCE) && b.cfg->state == CFG_DEFAULTED)
    return GOAL_UNCALIBRATED;
  // A posture ends a POSE stream (the mind gives the joints back): IDLE
  // takes them from where they are. After joint angles (a trick) its move
  // restarts there, over `ticks`, even when the posture is the one it had
  // (a trick ending in the posture it began from); after a stream of feet
  // (a walk, whose IDLE posture is STANCE) the hand-over is the usual
  // crossfade. STANCE from elsewhere starts at the default height.
  bool restart = stream_on(b.stream) && !stream_feet(b.stream);
  stream_end(b, 0);
  if (posture == POSTURE_STANCE && arbiter_idle_posture(b.arb) != POSTURE_STANCE) stream_stance_default(b.stream);
  b.balance_req = flags & 1;
  if (b.holder == RH_WARM_HOLD) release_reflex(b, RH_WARM_HOLD);  // the Pi takes over
  // LIMP is the Pi's emergency stop.
  if (posture == POSTURE_LIMP && b.holder != RH_NONE) arbiter_reflex(b.arb, POSTURE_LIMP, 0);
  arbiter_set_idle(b.arb, posture, ticks, restart);
  return GOAL_OK;
}

GoalResult body_goal_balance(Body& b, const uint8_t p[5]) {
  if (p[0] > 1 || p[3] > BAL_MAX_DEG_LIMIT || p[4] > BAL_MAX_DEG_LIMIT) return GOAL_BAD_ARG;
  Balance& s = b.bal;
  s.enable = p[0];
  s.kp_q8 = p[1];
  s.kd_ms = p[2];
  s.db_deg = p[3];
  s.max_deg = p[4];
  return GOAL_OK;
}


GoalResult body_goal_head(Body& b, const uint8_t* p) {
  // Payload (kilo_set_head_t, little-endian like both targets): epoch, mode,
  // pan, tilt, tail (i16 a16), max_speed_dps (u16).
  uint8_t mode = p[1];
  if (mode > KILO_HEAD_MODE_HOLD) return GOAL_BAD_ARG;
  Arbiter& a = b.arb;
  if (mode == KILO_HEAD_MODE_RELEASE) {  // calming down is never stale
    arbiter_head_release(a);
    return GOAL_OK;
  }
  if (p[0] != b.epoch) return GOAL_STALE;
  if (body_fallen(b)) return GOAL_FALLEN;
  if (b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  // Targets as sent (the safety stage clamps them to the joint limits);
  // JOINT_OFF (-32768) leaves that joint to its owner.
  memcpy(a.head_tgt, p + 2, sizeof a.head_tgt);
  // deg/s -> a16 per 20 ms tick (x 16 / 50), rounded up so a slow head
  // still moves. 0, or anything over 2500 deg/s (SLEW_MAX_A16's maximum),
  // leaves only the safety slew.
  uint16_t dps = (uint16_t)(p[8] | (p[9] << 8));
  a.head_step = (dps && dps <= 2500u) ? (int16_t)((dps * 8u + 24u) / 25u) : 0x3FFF;
  a.head_ttl = HEAD_HOLD_TICKS;
  return GOAL_OK;
}

GoalResult body_pose(Body& b, const uint8_t* p) {
  if (stream_check(p) != GOAL_OK) return GOAL_BAD_ARG;
  if (p[0] != b.epoch) return GOAL_STALE;
  bool fall_ok = p[3] & KILO_POSE_FLAGS_FALL_OK;
  if (body_fallen(b) && !fall_ok) return GOAL_FALLEN;
  if (b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  if (b.cfg->state == CFG_DEFAULTED) return GOAL_UNCALIBRATED;
  uint8_t idle = arbiter_idle_posture(b.arb);
  if (idle == POSTURE_LIMP || idle == POSTURE_CALIB) return GOAL_BUSY;
  if (body_fallen(b)) {  // the mind's get-up takes the joints from the fall reflex
    b.fall.state = FS_UPRIGHT;
    release_reflex(b, RH_FALL);
  }
  if (b.holder == RH_WARM_HOLD) release_reflex(b, RH_WARM_HOLD);  // the Pi takes over
  if (b.holder != RH_NONE) return GOAL_BUSY;
  bool first = !b.stream.state;
  // (Past, too far ahead or no room: dropped and counted, not refused.)
  if (!stream_push(b.stream, p, b.tick, b.arb.last_on) || !first) return GOAL_OK;
  // A new stream starts from the corrected output; the corrections restart
  // from zero under it.
  balance_reset(b.bal);
  if (stream_feet_key(p) && idle != POSTURE_STANCE) {
    // A stream of feet: IDLE's posture is STANCE (STAND's head and tail now,
    // STANCE's legs when the stream ends).
    arbiter_set_idle(b.arb, POSTURE_STANCE, 0);
    stream_stance_default(b.stream);
  }
  return GOAL_OK;
}

GoalResult body_goal_pose(Body& b, const uint8_t* p) {
  if (p[0] != b.epoch) return GOAL_STALE;
  if (body_fallen(b)) return GOAL_FALLEN;
  if (b.batt.level >= BATT_LOW) return GOAL_LOW_BATT;
  stream_pose_goal(b.stream, p + 1);
  return GOAL_OK;
}

uint8_t body_support(const Body& b) {
  if (stream_on(b.stream)) return b.stream.support;
  uint8_t p = arbiter_idle_posture(b.arb);
  if (b.arb.reflex_active || !(p == POSTURE_SIT || p == POSTURE_STAND || p == POSTURE_STANCE)) return 0;
  return 0xF;
}

uint8_t body_mode(const Body& b) {
  if (b.batt.level >= BATT_LOW) return KILO_MODE_LOW_BATT;
  if (b.wd.stage) return KILO_MODE_LINK_LOST;
  if (b.fall.state) return KILO_MODE_FALLEN;
  if (body_stream_on(b)) return KILO_MODE_STREAM;
  if (arbiter_idle_posture(b.arb) == POSTURE_LIMP && !b.arb.reflex_active) return KILO_MODE_LIMP;
  return KILO_MODE_IDLE;
}
