#include "planner/plan.h"

namespace kilo {
namespace planner {

static real smooth(real s) {  // 0..1 -> 0..1 with zero slope at both ends
  s = m_min((real)1.0, m_max((real)0.0, s));
  return ((real)1.0 - m_cos(kPi * s)) / (real)2.0;
}

static bool near(real a, real b) { return m_fabs(a - b) < (real)1e-6; }

// ---- the quadratic program ---------------------------------------------------------

// The least-norm w with a . w = b for each of k rows (a0, a1, a2, b) (plan.py _min_norm_eq).
static bool min_norm_eq(const double* const* A, int k, double w[3]) {
  if (k == 1) {
    const double* a = A[0];
    double g = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    if (g < 1e-12) return false;
    double lam = a[3] / g;
    w[0] = a[0] * lam;
    w[1] = a[1] * lam;
    w[2] = a[2] * lam;
    return true;
  }
  if (k == 2) {
    const double *p = A[0], *q = A[1];
    double g00 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
    double g01 = p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
    double g11 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
    double det = g00 * g11 - g01 * g01;
    if (d_fabs(det) < 1e-9) return false;
    double l0 = (p[3] * g11 - g01 * q[3]) / det;
    double l1 = (g00 * q[3] - g01 * p[3]) / det;
    for (int c = 0; c < 3; c++) w[c] = p[c] * l0 + q[c] * l1;
    return true;
  }
  double G[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) G[i][j] = A[i][0] * A[j][0] + A[i][1] * A[j][1] + A[i][2] * A[j][2];
  const double v[3] = {A[0][3], A[1][3], A[2][3]};
  double lam[3];
  if (!solve3(G, v, lam)) return false;
  for (int c = 0; c < 3; c++) w[c] = A[0][c] * lam[0] + A[1][c] * lam[1] + A[2][c] * lam[2];
  return true;
}

void Qp::init(const QpRow* rows, int n, double want, const double scale[3]) {
  // whiten: d = S w, so the cost is |w|^2 and a row's gradient becomes g S
  n_ = n;
  for (int c = 0; c < 3; c++) s_[c] = scale[c];
  for (int i = 0; i < n; i++) {
    for (int c = 0; c < 3; c++) W_[i][c] = rows[i].g[c] * scale[c];
    W_[i][3] = want - rows[i].m;
  }
  pc_ = 0;
  found = false;
}

bool Qp::feasible(const double w[3]) const {
  for (int i = 0; i < n_; i++)
    if (W_[i][0] * w[0] + W_[i][1] * w[1] + W_[i][2] * w[2] < W_[i][3] - 1e-6) return false;
  return true;
}

void Qp::try_set(const int* idx, int k) {
  const double* A[3];
  for (int i = 0; i < k; i++) A[i] = W_[idx[i]];
  double w[3];
  if (!min_norm_eq(A, k, w)) return;
  double nn = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
  if ((!has_best_ || nn < best_n_) && feasible(w)) {
    has_best_ = true;
    best_n_ = nn;
    for (int c = 0; c < 3; c++) best_[c] = w[c];
  }
}

// One step: the feasibility of 0 and the yield before size 1, then the
// active sets of one size per step (itertools.combinations order).
bool Qp::resume() {
  if (pc_ == 0) {
    const double zero[3] = {0, 0, 0};
    if (feasible(zero)) {
      found = true;
      step[0] = step[1] = step[2] = 0;
      return true;
    }
    has_best_ = false;
    size_ = 1;
    pc_ = 1;
    return false;
  }
  int idx[3];
  if (size_ == 1) {
    for (idx[0] = 0; idx[0] < n_; idx[0]++) try_set(idx, 1);
  } else if (size_ == 2) {
    for (idx[0] = 0; idx[0] < n_; idx[0]++)
      for (idx[1] = idx[0] + 1; idx[1] < n_; idx[1]++) try_set(idx, 2);
  } else {
    for (idx[0] = 0; idx[0] < n_; idx[0]++)
      for (idx[1] = idx[0] + 1; idx[1] < n_; idx[1]++)
        for (idx[2] = idx[1] + 1; idx[2] < n_; idx[2]++) try_set(idx, 3);
  }
  if (size_ < 3) {
    size_++;
    return false;
  }
  found = has_best_;
  if (found)
    for (int c = 0; c < 3; c++) step[c] = best_[c] * s_[c];
  return true;
}

// ---- one solve of the lean ------------------------------------------------------------

void LeanSolve::init(const BodyModel* m, const PlannerConfig* cfg, const Pattern& p, int k, const real xg[kLegs],
                     const real speed[kLegs], const real tgt[kLegs], real h, int shift, int swing) {
  m_ = m;
  cfg_ = cfg;
  h_ = h;
  nsup_ = 0;
  for (int leg = 0; leg < kLegs; leg++)
    if (!p.in_group(k, leg)) support_[nsup_++] = leg;
  const int ahead[2] = {shift, shift + swing};
  for (int c = 0; c < 2; c++)
    for (int leg = 0; leg < kLegs; leg++) {
      if (!p.in_group(k, leg)) configs_[c][leg] = xg[leg] - speed[leg] * (real)ahead[c];
      else configs_[c][leg] = ahead[c] == shift ? xg[leg] - speed[leg] * (real)shift : tgt[leg];
    }
  u_[0] = u_[1] = u_[2] = 0;
  it_ = 0;
  ci_ = 0;
  j_ = -1;
  nrows_ = 0;
  phase_ = Eval;
}

// Signed distance (mm, + inside) of the centre of mass from each edge of
// the support polygon, for the gait-frame feet shifted by lean u (three
// support feet: their triangle, counter-clockwise).
int LeanSolve::edge_margins(const real xs_gait[kLegs], const real u[3], real out[kLegs]) const {
  real xs[kLegs], zs[kLegs];
  for (int leg = 0; leg < kLegs; leg++) xs[leg] = xs_gait[leg] - u[0];
  leg_lengths(*m_, xs, h_, u[1], u[2], zs);
  V3 com, feet[kLegs];
  world_points(*m_, xs, zs, u[1], u[2], &com, feet);
  V2 poly[kLegs];
  int n;
  if (nsup_ == 3) {
    V3 a = feet[support_[0]], b = feet[support_[1]], c = feet[support_[2]];
    if ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) < 0) {
      V3 t = b;
      b = c;
      c = t;
    }
    poly[0] = {a.x, a.y};
    poly[1] = {b.x, b.y};
    poly[2] = {c.x, c.y};
    n = 3;
  } else {
    V2 pts[kLegs];
    for (int i = 0; i < nsup_; i++) pts[i] = {feet[support_[i]].x, feet[support_[i]].y};
    n = hull(pts, nsup_, poly);
  }
  for (int i = 0; i < n; i++) {
    V2 a = poly[i], b = poly[(i + 1) % n];
    real ex = b.x - a.x, ey = b.y - a.y;
    real L = m_sqrt(ex * ex + ey * ey);
    if (L == 0) L = 1;
    out[i] = (ex * (com.y - a.y) - ey * (com.x - a.x)) / L;
  }
  return n;
}

bool LeanSolve::resume() {
  const PlannerConfig& cfg = *cfg_;
  for (;;) {
    switch (phase_) {
      case Eval: {  // one margin evaluation, then the yield after it
        if (j_ < 0) {
          nb_ = edge_margins(configs_[ci_], u_, base_);
        } else {
          real du[3] = {u_[0], u_[1], u_[2]};
          du[j_] += (real)0.5;
          real m[kLegs];
          edge_margins(configs_[ci_], du, m);
          for (int e = 0; e < nb_; e++) grads_[j_][e] = (m[e] - base_[e]) / (real)0.5;
        }
        if (++j_ == 3) {
          for (int e = 0; e < nb_ && nrows_ < kMaxRows; e++, nrows_++) {
            for (int c = 0; c < 3; c++) rows_[nrows_].g[c] = grads_[c][e];
            rows_[nrows_].m = base_[e];
          }
          j_ = -1;
          if (++ci_ == 2) {
            ci_ = 0;
            phase_ = QpStart;
          }
        }
        return false;
      }
      case QpStart: {
        // minimise sum((d_j / scale_j)^2) s.t. m_e + g_e . d >= margin
        want_ = cfg.margin_mm;
        if (!(want_ >= -1e-9)) {
          result = {u_[0], u_[1], u_[2]};
          return true;
        }
        const double scale[3] = {cfg.dx_scale_mm, cfg.pitch_scale_deg, cfg.roll_scale_deg};
        qp_.init(rows_, nrows_, want_, scale);
        phase_ = QpRun;
        break;
      }
      case QpRun: {
        if (!qp_.resume()) return false;
        if (!qp_.found) {  // infeasible: ask for less
          want_ = want_ > 2.0 ? want_ - 2.0 : (want_ > 0 ? 0.0 : -1.0);
          if (want_ >= -1e-9) {
            const double scale[3] = {cfg.dx_scale_mm, cfg.pitch_scale_deg, cfg.roll_scale_deg};
            qp_.init(rows_, nrows_, want_, scale);
            break;
          }
          result = {u_[0], u_[1], u_[2]};
          return true;
        }
        const real lim[3] = {cfg.dx_max_mm, cfg.pitch_max_deg, cfg.roll_max_deg};
        for (int c = 0; c < 3; c++) {
          double v = (double)u_[c] + qp_.step[c];  // min(lim, max(-lim, u + step))
          double lo = v > -(double)lim[c] ? v : -(double)lim[c];
          u_[c] = (real)(lo < (double)lim[c] ? lo : (double)lim[c]);
        }
        if (++it_ == 2) {
          result = {u_[0], u_[1], u_[2]};
          return true;
        }
        nrows_ = 0;
        phase_ = Eval;
        break;
      }
    }
  }
}

bool LeanJob::matches(const Pattern* p, int k2, const real xg2[kLegs], const real s2[2], real stance2, int shift2,
                      const Lean& from) const {
  if (pattern != p || k != k2 || shift != shift2 || !near(stance, stance2)) return false;
  for (int i = 0; i < kLegs; i++)
    if (!near(xg[i], xg2[i])) return false;
  for (int i = 0; i < 2; i++)
    if (!near(s[i], s2[i])) return false;
  return near(lean_from.dx, from.dx) && near(lean_from.pitch, from.pitch) && near(lean_from.roll, from.roll);
}

// ---- the planner ---------------------------------------------------------------------

Planner::Planner(const BodyModel& model, const PlannerConfig& cfg)
    : model_(model), cfg_(cfg), pattern_(&default_pattern()) {
  stats_.min_margin = m_inf();
}

void Planner::start(const Pattern& p, Direction d, real speed_mm_s, uint32_t tick, real start_height) {
  tick_ = tick;
  lag_ = model_.lag_ticks(kTickMs);
  lead_ = ceil_i(lag_ - (real)1e-9);
  pattern_ = &p;
  direction_ = d;
  speed_mm_s_ = speed_mm_s < 0 ? default_speed(p) : speed_mm_s;
  pending_ = nullptr;
  for (int leg = 0; leg < kLegs; leg++) {
    xg_[leg] = 0;
    swing_[leg] = 0;
    caught_[leg] = false;
  }
  h_ = start_height < 0 ? model_.stance_height(205) : start_height;
  s_cur_[0] = s_cur_[1] = s_tgt_[0] = s_tgt_[1] = 0;
  stance_ticks_ = stance_tgt_ = 1;
  shift_ticks_ = 1;
  lean_ = lean_from_ = lean_to_ = Lean();
  tr_ticks_ = 1;
  k_ = -1;
  t_ = 0;
  S_ = 0;
  held_ = 0;
  skip_ = false;
  stopping_ = finishing_ = done_ = false;
  err_[0] = err_[1] = 0;
  prev_att_ = {0, 0};
  hist_n_ = (int)lag_ + 3;
  if (hist_n_ > kHistMax) hist_n_ = kHistMax;  // a servo lag over 580 ms: the prediction is cut there
  for (int i = 0; i < hist_n_; i++) att_hist_[i] = {0, 0};
  has_job_ = has_next_ = false;
  has_last_ = false;
  stats_ = {0, 0, 0, m_inf()};
  apply_timing();
  stance_ticks_ = stance_tgt_;
  start_slot();
  if (has_job_) {  // the first slot's lean: solved here, all of it
    run_job(&job_, -1);
    apply_job(job_);
  }
}

int Planner::swing_ticks() const {
  int w = round_i(pattern_->swing_ms / kTickMs);
  return w > 2 ? w : 2;
}

void Planner::apply_timing() {
  Timing tm = timing();
  real shift = cfg_.fixed_shift ? cfg_.shift_ms : tm.shift_ms;
  real n = pattern_->n_groups;
  stance_tgt_ = (n * (shift + pattern_->swing_ms) - pattern_->swing_ms) / kTickMs;
  int st = round_i(shift / kTickMs);
  shift_ticks_ = st > 0 ? st : 0;
  if (stopping_ || pending_ != nullptr) {
    s_tgt_[0] = s_tgt_[1] = 0;
  } else {
    side_strides(direction_, tm.stride_mm, *pattern_, s_tgt_);
  }
  has_next_ = false;  // the next slot's lean was solved for the old timing
}

void Planner::set_speed(real speed_mm_s) {
  speed_mm_s_ = m_fabs(speed_mm_s);
  apply_timing();
  if (has_job_) {  // this slot is still waiting for its lean: solve it for the new shift
    new_job(&job_, k_, xg_, s_cur_, stance_ticks_, lean_from_, h_);
  } else if (t_ < S_) {  // still in the shift: a new length, never shorter than its lean needs
    int a = t_ + 1, b = tr_ticks_ + (leaning() ? lead_ : 0), c = shift_ticks_;
    int m = a > b ? a : b;
    S_ = m > c ? m : c;
  }
}

void Planner::set_direction(Direction d) {
  direction_ = d;
  apply_timing();
}

void Planner::set_pattern(const Pattern& p) {
  if (stopping_) return;
  if (&p == pattern_) {
    if (pending_ != nullptr) {
      pending_ = nullptr;
      apply_timing();
    }
    return;
  }
  pending_ = &p;
  apply_timing();
}

void Planner::stop() {
  stopping_ = true;
  pending_ = nullptr;
  apply_timing();
}

bool Planner::home() const {
  if (!(m_fabs(s_cur_[0]) < (real)0.05 && m_fabs(s_cur_[1]) < (real)0.05)) return false;
  for (int leg = 0; leg < kLegs; leg++)
    if (!(m_fabs(xg_[leg]) < kRecentredMm)) return false;
  return true;
}

void Planner::start_slot() {
  has_job_ = false;
  if (pending_ != nullptr && home()) {
    // every foot is home and still: the new pattern starts from its first group
    pattern_ = pending_;
    pending_ = nullptr;
    k_ = -1;
    apply_timing();
    stance_ticks_ = stance_tgt_;
  }
  k_ = (k_ + 1) % pattern_->n_groups;
  t_ = 0;
  held_ = 0;
  for (int leg = 0; leg < kLegs; leg++) caught_[leg] = false;
  skip_ = false;
  if (stopping_ || changing()) {
    skip_ = true;
    for (int leg = 0; leg < kLegs; leg++)
      if (pattern_->in_group(k_, leg) &&
          !(m_fabs(s_cur_[0]) < (real)0.05 && m_fabs(s_cur_[1]) < (real)0.05 && m_fabs(xg_[leg]) < kRecentredMm))
        skip_ = false;
  }
  lean_from_ = lean_;
  bool had_next = has_next_;
  has_next_ = false;
  if (stopping_ && home()) finishing_ = true;
  if (finishing_) {
    lean_to_ = Lean();
    int mt = move_ticks(lean_from_, lean_to_, h_);
    tr_ticks_ = lead_ + 5 > mt ? lead_ + 5 : mt;
    S_ = tr_ticks_ + lead_;
    return;
  }
  S_ = shift_ticks_;
  if (leaning() && !skip_) {
    if (had_next && next_job_.matches(pattern_, k_, xg_, s_cur_, stance_ticks_, shift_ticks_, lean_from_)) {
      job_ = next_job_;
    } else {  // something changed since it was solved (a command, a catch): solved now, the slot waits
      new_job(&job_, k_, xg_, s_cur_, stance_ticks_, lean_from_, h_);
    }
    if (job_.has_result) apply_job(job_);
    else has_job_ = true;
  } else {
    lean_to_ = leaning() ? lean_from_ : Lean();
    int tr = 0;
    if (lean_to_ != lean_from_) {  // out of a leaning pattern: level first, as fast as gravity lets
      tr = move_ticks(lean_from_, lean_to_, h_);
      if (tr + lead_ > S_) S_ = tr + lead_;
    }
    int a = S_ - (tr ? lead_ : 0);
    int m = tr > 1 ? tr : 1;
    tr_ticks_ = m > a ? m : a;
  }
}

// Ticks the body needs to move from lean a to lean b on a cosine profile: a
// side or an end that drops falls no faster than half of gravity's
// drop_accel, and the fore-aft shift goes at most dx_speed_mm_s.
int Planner::move_ticks(const Lean& a, const Lean& b, real height) const {
  real h = height + model_.foot_r_mm;
  real need = 0;
  const real d_deg[2] = {b.roll - a.roll, b.pitch - a.pitch};
  const real lever[2] = {model_.track_mm / 2, model_.length_mm / 2};
  for (int i = 0; i < 2; i++) {
    real amp = m_fabs(radians(d_deg[i]));
    if (amp > 0) need = m_max(need, kPi * m_sqrt(amp / ((real)0.5 * model_.drop_accel(lever[i], h))));
  }
  need = m_max(need, kPi / 2 * m_fabs(b.dx - a.dx) / cfg_.dx_speed_mm_s);
  int t = ceil_i(need / (kTickMs / (real)1000.0));
  return t > 1 ? t : 1;
}

void Planner::slewed(real s[2], real* stance, real* h) const {
  for (int i = 0; i < 2; i++) s[i] = s[i] + m_max(-kStrideSlewMm, m_min(kStrideSlewMm, s_tgt_[i] - s[i]));
  *stance = m_min(*stance * ((real)1.0 + kStanceSlew), m_max(*stance / ((real)1.0 + kStanceSlew), stance_tgt_));
  *h += m_max(-kHeightSlewMm, m_min(kHeightSlewMm, height_target() - *h));
}

real Planner::height_target() const {
  if (finishing_) return model_.stance_height(205);
  return model_.stance_height(pattern_->body_h_256);
}

void Planner::new_job(LeanJob* job, int k, const real xg[kLegs], const real s[2], real stance,
                      const Lean& lean_from, real h_now) const {
  job->pattern = pattern_;
  job->k = k;
  for (int i = 0; i < kLegs; i++) job->xg[i] = xg[i];
  job->s[0] = s[0];
  job->s[1] = s[1];
  job->stance = stance;
  job->shift = shift_ticks_;
  job->lean_from = lean_from;
  job->has_result = false;
  for (int leg = 0; leg < kLegs; leg++) {
    job->speed[leg] = s[kLeft[leg] ? 0 : 1] / stance;
    job->tgt[leg] = s[kLeft[leg] ? 0 : 1] / (real)2.0;
  }
  job->h = model_.stance_height(pattern_->body_h_256);
  job->h_now = h_now;
  job->swing = swing_ticks();
  job->pc = 0;
}

// At most units steps of the job (plan.py _lean_steps, one next() each); true once done.
bool Planner::run_job(LeanJob* job, int units) const {
  for (int u = 0; !job->has_result && (units < 0 || u < units); u++) {
    // The slot's shift, lean and move ticks. The move must be done a servo
    // lag before the lift: when it needs longer than the speed's shift
    // gives, the shift grows and the lean is solved again for the later lift.
    bool finished = false;
    switch (job->pc) {
      case 0:
        job->S = job->shift;
        job->solve.init(&model_, &cfg_, *job->pattern, job->k, job->xg, job->speed, job->tgt, job->h, job->S,
                        job->swing);
        job->pc = 1;
        [[fallthrough]];
      case 1: {
        if (!job->solve.resume()) break;
        Lean lean_to = job->solve.result;
        int tr = move_ticks(job->lean_from, lean_to, job->h_now);
        if (tr + lead_ > job->S) {
          job->S = tr + lead_;
          job->solve.init(&model_, &cfg_, *job->pattern, job->k, job->xg, job->speed, job->tgt, job->h, job->S,
                          job->swing);
          job->pc = 2;
        } else {
          job->res_lean_to = lean_to;
          job->res_tr = tr;
          finished = true;
          break;
        }
      }
        [[fallthrough]];
      case 2: {
        if (!job->solve.resume()) break;
        job->res_lean_to = job->solve.result;
        job->res_tr = move_ticks(job->lean_from, job->res_lean_to, job->h_now);
        finished = true;
        break;
      }
    }
    if (finished) {
      int a = job->S - lead_, tr = job->res_tr;
      int m = a > 1 ? a : 1;
      int tr_ticks = m > tr ? m : tr;
      job->res_S = job->S > tr_ticks + lead_ ? job->S : tr_ticks + lead_;
      job->res_tr = tr_ticks;
      job->has_result = true;
    }
  }
  return job->has_result;
}

void Planner::apply_job(const LeanJob& job) {
  S_ = job.res_S;
  lean_to_ = job.res_lean_to;
  tr_ticks_ = job.res_tr;
  has_job_ = false;
}

// The feet, strides, stance time and height at the end of this slot's swing
// if nothing changes (the same arithmetic as next()).
void Planner::predict_slot_end(real xg[kLegs], real s[2], real* st, real* h) const {
  int W = swing_ticks();
  for (int i = 0; i < kLegs; i++) xg[i] = xg_[i];
  s[0] = s_cur_[0];
  s[1] = s_cur_[1];
  *st = stance_ticks_;
  *h = h_;
  for (int t = t_; t < S_ + W; t++) {
    slewed(s, st, h);
    real sw = smooth((real)(t - S_ + 1) / (real)W);
    for (int leg = 0; leg < kLegs; leg++) {
      int side = kLeft[leg] ? 0 : 1;
      if (pattern_->in_group(k_, leg)) {
        real x0 = swing_[leg];
        xg[leg] = x0 + (s[side] / (real)2.0 - x0) * sw;
      } else {
        xg[leg] -= s[side] / *st;
      }
    }
  }
}

const TickPlan& Planner::next() {
  const PlannerConfig& cfg = cfg_;
  const int W = swing_ticks();
  if (t_ >= S_ + W || (skip_ && t_ >= S_) || (finishing_ && t_ >= S_)) {
    if (finishing_) done_ = true;
    else start_slot();
  }
  // The plan waits (every foot where it is, the lean held, nothing slewed,
  // the slot's clock stopped) while this slot's lean is still being solved,
  // and while the gate holds the lift: a leg lifts only while the body
  // follows the plan.
  bool wait = false;
  if (has_job_) {
    if (run_job(&job_, kLeanUnitsPerTick)) {
      apply_job(job_);
    } else {
      wait = true;
      stats_.waits++;
    }
  }
  if (!wait && t_ == S_ && !skip_ && !finishing_ && held_ < cfg.hold_max &&
      m_max(m_fabs(err_[0]), m_fabs(err_[1])) > cfg.gate_deg) {
    wait = true;
    held_++;
    stats_.holds++;
  }

  const int k = k_;
  uint8_t lifting = 0;
  uint8_t n_lifting = 0;
  real lift[kLegs] = {0, 0, 0, 0};
  bool swinging = false;
  if (!wait) {
    slewed(s_cur_, &stance_ticks_, &h_);
    if (t_ < S_ || skip_ || finishing_) {
      for (int leg = 0; leg < kLegs; leg++) xg_[leg] -= side_speed(leg);
    } else {
      swinging = true;
      int j = t_ - S_ + 1;  // 1..swing
      real s = (real)j / (real)W;
      if (j == 1)
        for (int leg = 0; leg < kLegs; leg++) swing_[leg] = xg_[leg];  // where the feet leave the ground
      for (int leg = 0; leg < kLegs; leg++)
        if (!pattern_->in_group(k, leg)) xg_[leg] -= side_speed(leg);
      for (int leg = 0; leg < kLegs; leg++) {
        if (!pattern_->in_group(k, leg) || caught_[leg]) continue;
        if (tip_toward(leg) > cfg.catch_deg && j < W) {
          caught_[leg] = true;
          caught_j_[leg] = j - 1;
          caught_lift_[leg] = pattern_->step_h_mm * ((real)1.0 - m_cos(2 * kPi * (real)(j - 1) / (real)W)) / 2;
          stats_.catches++;
          has_next_ = false;  // it lands elsewhere: the next slot's lean is solved when it starts
        }
      }
      bool any_caught = false;
      for (int leg = 0; leg < kLegs; leg++) {
        if (!pattern_->in_group(k, leg)) continue;
        if (caught_[leg]) {
          any_caught = true;
          int kk = j - caught_j_[leg];
          lift[leg] = caught_lift_[leg] * ((real)1.0 - smooth((real)kk / (real)3.0));
        } else {
          real x0 = swing_[leg];
          xg_[leg] = x0 + (target_x(leg) - x0) * smooth(s);
          lift[leg] = pattern_->step_h_mm * ((real)1.0 - m_cos(2 * kPi * s)) / 2;
        }
      }
      if (any_caught) {
        bool all_down = true;
        for (int leg = 0; leg < kLegs; leg++)
          if (pattern_->in_group(k, leg) && !(lift[leg] < (real)1e-6)) all_down = false;
        if (all_down) {
          t_ = S_ + W;  // every caught foot is down: the slot is over
          for (int leg = 0; leg < kLegs; leg++)
            if (pattern_->in_group(k, leg)) lift[leg] = 0;
        }
      }
      for (int leg = 0; leg < kLegs; leg++)
        if (pattern_->in_group(k, leg) && lift[leg] > (real)1e-6) {
          lifting |= (uint8_t)(1u << leg);
          n_lifting++;
        }
    }
    // the lean: moves from the slot's start, complete tr_ticks later
    if (t_ < tr_ticks_) lean_ = lean_from_.lerp(lean_to_, smooth((real)(t_ + 1) / (real)tr_ticks_));
    else lean_ = lean_to_;
  }

  real xs[kLegs], zs[kLegs], xi[kLegs], zi[kLegs];
  for (int leg = 0; leg < kLegs; leg++) xs[leg] = xg_[leg] - lean_.dx;
  leg_lengths(model_, xs, h_, lean_.pitch, lean_.roll, zs);
  TickPlan& out = last_;
  uint8_t support = 0;
  for (int leg = 0; leg < kLegs; leg++) {
    out.x_raw[leg] = m_min((real)kXMax, m_max((real)kXMin, xs[leg]));
    out.z_raw[leg] = m_min((real)kZMax, m_max((real)kZMin, zs[leg] - lift[leg]));
    int x = round_i(out.x_raw[leg]);
    int z = round_i(out.z_raw[leg]);
    out.x[leg] = (int8_t)x;
    out.z[leg] = (uint8_t)z;
    xi[leg] = (real)x;
    zi[leg] = (real)z;
    if (lift[leg] <= (real)1e-6) support |= (uint8_t)(1u << leg);
  }
  Attitude att = fit_attitude(model_, xi, zi, support, prev_att_);
  prev_att_ = att;
  for (int i = 0; i + 1 < hist_n_; i++) att_hist_[i] = att_hist_[i + 1];
  att_hist_[hist_n_ - 1] = att;
  real pp, pr;
  lagged(&pp, &pr);
  pp = m_min(kAttMaxDeg, m_max(-kAttMaxDeg, pp));
  pr = m_min(kAttMaxDeg, m_max(-kAttMaxDeg, pr));
  real m = stability(model_, xi, zi, support, att.pitch, att.roll);
  if (n_lifting == 1) stats_.min_margin = m_min(stats_.min_margin, m);
  out.tick = tick_;
  out.support = support;
  out.pitch = pp;
  out.roll = pr;
  out.plan_pitch = att.pitch;
  out.plan_roll = att.roll;
  out.margin = m;
  out.lifting = lifting;
  out.n_lifting = n_lifting;
  has_last_ = true;
  tick_++;
  if (!wait) t_++;
  // The next slot's lean, solved a little every tick of this swing, from
  // where the feet will be when that slot starts; used if nothing changes.
  bool any_caught = false;
  for (int leg = 0; leg < kLegs; leg++) any_caught = any_caught || caught_[leg];
  if (swinging && !any_caught && leaning() && !changing() && !finishing_) {
    if (!has_next_) {
      real xg[kLegs], s[2], st, h;
      predict_slot_end(xg, s, &st, &h);
      new_job(&next_job_, (k_ + 1) % pattern_->n_groups, xg, s, st, lean_to_, h);
      has_next_ = true;
    }
    run_job(&next_job_, kLeanUnitsPerTick);
  }
  return out;
}

// The planned attitude a servo lag ago (linear between ticks).
void Planner::lagged(real* pitch, real* roll) const {
  int n = hist_n_ - 1;  // att_hist_[n] is this tick
  int i = floor_i(lag_);
  real f = lag_ - (real)i;
  int ia = n - i > 0 ? n - i : 0;
  int ib = n - i - 1 > 0 ? n - i - 1 : 0;
  Attitude a = att_hist_[ia], b = att_hist_[ib];
  *pitch = a.pitch + (b.pitch - a.pitch) * f;
  *roll = a.roll + (b.roll - a.roll) * f;
}

// How far (degrees) the body is tipping toward this leg beyond the plan.
real Planner::tip_toward(int leg) const {
  real side = kLeft[leg] ? (real)1.0 : (real)-1.0;  // a left leg: the left side going down is +roll
  real end = kFront[leg] ? (real)1.0 : (real)-1.0;  // a front leg: nose going down is +pitch
  return side * err_[1] + end * err_[0];
}

}  // namespace planner
}  // namespace kilo
