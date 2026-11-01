#include "planner/model.h"

namespace kilo {
namespace planner {

real BodyModel::total_g() const {
  return body.grams + battery.grams + pi.grams + 4 * (upper_leg_g + lower_leg_g);
}

V2 BodyModel::hip(int leg) const {
  return {kFront[leg] ? length_mm / 2 : -length_mm / 2, kLeft[leg] ? track_mm / 2 : -track_mm / 2};
}

real BodyModel::min_reach_mm() const {
  real c = m_cos(radians(knee_max_deg));
  return m_sqrt(l1_mm * l1_mm + l2_mm * l2_mm + 2 * l1_mm * l2_mm * c);
}

bool BodyModel::reachable(real x, real z) const {
  real d = m_sqrt(x * x + z * z);
  return min_reach_mm() - (real)1e-6 <= d && d <= reach_mm() + (real)1e-6 && z > 0;
}

void BodyModel::ik(int leg, real x, real z, real* t1, real* t2) const {
  real l1 = l1_mm, l2 = l2_mm;
  real d = m_sqrt(x * x + z * z);
  d = m_min(m_max(d, min_reach_mm()), reach_mm());
  real c = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  c = m_min((real)1.0, m_max((real)-1.0, c));
  real s = m_sqrt((real)1.0 - c * c) * (kFront[leg] ? (real)1.0 : (real)-1.0);
  *t2 = m_atan2(s, c);
  *t1 = m_atan2(x, z) - m_atan2(l2 * s, l1 + l2 * c);
}

void BodyModel::fk(real t1, real t2, real* x, real* z) const {
  *x = l1_mm * m_sin(t1) + l2_mm * m_sin(t1 + t2);
  *z = l1_mm * m_cos(t1) + l2_mm * m_cos(t1 + t2);
}

V3 BodyModel::com(const real xs[kLegs], const real zs[kLegs]) const {
  real m = body.grams + battery.grams + pi.grams;
  real sx = body.grams * body.x + battery.grams * battery.x + pi.grams * pi.x;
  real sy = body.grams * body.y + battery.grams * battery.y + pi.grams * pi.y;
  real sz = body.grams * body.z + battery.grams * battery.z + pi.grams * pi.z;
  real mu = upper_leg_g, ml = lower_leg_g;
  for (int leg = 0; leg < kLegs; leg++) {
    V2 h = hip(leg);
    real t1, t2;
    ik(leg, xs[leg], zs[leg], &t1, &t2);
    real kx = l1_mm * m_sin(t1), kz = l1_mm * m_cos(t1);  // knee, z down
    real fx = xs[leg], fz = zs[leg];
    real ux = kx * upper_leg_at, uz = kz * upper_leg_at;
    real lx = kx + (fx - kx) * lower_leg_at, lz = kz + (fz - kz) * lower_leg_at;
    sx += mu * (h.x + ux) + ml * (h.x + lx);
    sy += (mu + ml) * h.y;
    sz -= mu * uz + ml * lz;
    m += mu + ml;
  }
  return {sx / m, sy / m, sz / m};
}

void plane_coeffs(real pitch_deg, real roll_deg, real* a, real* b) {
  real p = radians(pitch_deg), r = radians(roll_deg);
  *a = m_tan(p) / m_cos(r);
  *b = m_tan(r);
}

Attitude attitude_of(real a, real b) {
  real roll = m_atan(b);
  return {degrees(m_atan(a * m_cos(roll))), degrees(roll)};
}

void leg_lengths(const BodyModel& m, const real xs[kLegs], real height, real pitch_deg, real roll_deg,
                 real zs[kLegs]) {
  real a, b;
  plane_coeffs(pitch_deg, roll_deg, &a, &b);
  real tot = 0;
  for (int leg = 0; leg < kLegs; leg++) {
    V2 h = m.hip(leg);
    zs[leg] = -a * (h.x + xs[leg]) - b * h.y;
    tot += zs[leg];
  }
  real off = height - tot / 4;
  for (int leg = 0; leg < kLegs; leg++) zs[leg] += off;
}

bool solve3(const double m[3][3], const double v[3], double x[3]) {
  double a = m[0][0], b = m[0][1], c = m[0][2];
  double d = m[1][0], e = m[1][1], f = m[1][2];
  double g = m[2][0], h = m[2][1], i = m[2][2];
  double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (d_fabs(det) < 1e-12) return false;
  x[0] = (v[0] * (e * i - f * h) - b * (v[1] * i - f * v[2]) + c * (v[1] * h - e * v[2])) / det;
  x[1] = (a * (v[1] * i - f * v[2]) - v[0] * (d * i - f * g) + c * (d * v[2] - v[1] * g)) / det;
  x[2] = (a * (e * v[2] - v[1] * h) - b * (d * v[2] - v[1] * g) + v[0] * (d * h - e * g)) / det;
  return true;
}

Attitude fit_attitude(const BodyModel& m, const real xs[kLegs], const real zs[kLegs], uint8_t support,
                      Attitude prev, double reg) {
  real a0r, b0r;
  plane_coeffs(prev.pitch, prev.roll, &a0r, &b0r);
  double a0 = a0r, b0 = b0r;
  // unknowns (a, b, H): a X + b Y - H = -(z + r); normal equations + reg on a, b
  double saa = 0, sab = 0, sah = 0, sbb = 0, sbh = 0, shh = 0;
  double ra = 0, rb = 0, rh = 0;
  double r = m.foot_r_mm;
  for (int leg = 0; leg < kLegs; leg++) {
    if (!(support >> leg & 1)) continue;
    V2 h = m.hip(leg);
    double X = (double)h.x + (double)xs[leg], Y = h.y;
    double t = -((double)zs[leg] + r);
    saa += X * X;
    sab += X * Y;
    sah -= X;
    sbb += Y * Y;
    sbh -= Y;
    shh += 1.0;
    ra += X * t;
    rb += Y * t;
    rh -= t;
  }
  saa += reg;
  sbb += reg;
  ra += reg * a0;
  rb += reg * b0;
  if (shh == 0) return prev;
  const double mat[3][3] = {{saa, sab, sah}, {sab, sbb, sbh}, {sah, sbh, shh}};
  const double v[3] = {ra, rb, rh};
  double sol[3];
  if (!solve3(mat, v, sol)) return prev;
  return attitude_of((real)sol[0], (real)sol[1]);
}

void rotation(real pitch_deg, real roll_deg, real R[3][3]) {
  real t = radians(pitch_deg), f = radians(roll_deg);
  real ct = m_cos(t), st = m_sin(t), cf = m_cos(f), sf = m_sin(f);
  R[0][0] = ct;
  R[0][1] = -st * sf;
  R[0][2] = st * cf;
  R[1][0] = 0;
  R[1][1] = cf;
  R[1][2] = sf;
  R[2][0] = -st;
  R[2][1] = -ct * sf;
  R[2][2] = ct * cf;
}

static V3 rot(const real R[3][3], real x, real y, real z) {
  return {R[0][0] * x + R[0][1] * y + R[0][2] * z, R[1][0] * x + R[1][1] * y + R[1][2] * z,
          R[2][0] * x + R[2][1] * y + R[2][2] * z};
}

void world_points(const BodyModel& m, const real xs[kLegs], const real zs[kLegs], real pitch_deg, real roll_deg,
                  V3* com, V3 feet[kLegs]) {
  real R[3][3];
  rotation(pitch_deg, roll_deg, R);
  V3 c = m.com(xs, zs);
  *com = rot(R, c.x, c.y, c.z);
  for (int leg = 0; leg < kLegs; leg++) {
    V2 h = m.hip(leg);
    feet[leg] = rot(R, h.x + xs[leg], h.y, -(zs[leg] + m.foot_r_mm));
  }
}

// Python's round(v, 6), for the hull's de-duplication.
static real round6(real v) {
  double s = (double)v * 1e6;
  int64_t i = (int64_t)s;
  if ((double)i > s) i--;
  double d = s - (double)i;
  if (d > 0.5 || (d == 0.5 && (i & 1))) i++;
  return (real)((double)i / 1e6);
}

static real cross(V2 o, V2 a, V2 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

int hull(const V2* in, int n, V2 out[kLegs]) {
  // sorted(set(rounded points))
  V2 pts[kLegs];
  int np = 0;
  for (int i = 0; i < n && np < kLegs; i++) {
    V2 p = {round6(in[i].x), round6(in[i].y)};
    bool dup = false;
    for (int j = 0; j < np; j++) dup = dup || (pts[j].x == p.x && pts[j].y == p.y);
    if (!dup) pts[np++] = p;
  }
  for (int i = 1; i < np; i++)  // insertion sort by (x, y)
    for (int j = i; j > 0 && (pts[j].x < pts[j - 1].x || (pts[j].x == pts[j - 1].x && pts[j].y < pts[j - 1].y));
         j--) {
      V2 t = pts[j];
      pts[j] = pts[j - 1];
      pts[j - 1] = t;
    }
  if (np <= 2) {
    for (int i = 0; i < np; i++) out[i] = pts[i];
    return np;
  }
  V2 lower[kLegs + 1], upper[kLegs + 1];
  int nl = 0, nu = 0;
  for (int i = 0; i < np; i++) {
    while (nl >= 2 && cross(lower[nl - 2], lower[nl - 1], pts[i]) <= (real)1e-9) nl--;
    lower[nl++] = pts[i];
  }
  for (int i = np - 1; i >= 0; i--) {
    while (nu >= 2 && cross(upper[nu - 2], upper[nu - 1], pts[i]) <= (real)1e-9) nu--;
    upper[nu++] = pts[i];
  }
  int k = 0;
  for (int i = 0; i < nl - 1; i++) out[k++] = lower[i];
  for (int i = 0; i < nu - 1 && k < kLegs; i++) out[k++] = upper[i];
  return k;
}

static real seg_dist(V2 p, V2 a, V2 b) {
  real dx = b.x - a.x, dy = b.y - a.y;
  real L2 = dx * dx + dy * dy;
  real t = L2 == 0 ? (real)0.0 : m_max((real)0.0, m_min((real)1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2));
  real ex = p.x - a.x - t * dx, ey = p.y - a.y - t * dy;
  return m_sqrt(ex * ex + ey * ey);
}

real margin(V2 p, const V2* poly, int n) {
  if (n == 0) return -m_inf();
  if (n == 1) {
    real dx = p.x - poly[0].x, dy = p.y - poly[0].y;
    return -m_sqrt(dx * dx + dy * dy);
  }
  if (n == 2) return -seg_dist(p, poly[0], poly[1]);
  bool inside = true;
  real best = m_inf();
  for (int i = 0; i < n; i++) {
    V2 a = poly[i], b = poly[(i + 1) % n];
    real ex = b.x - a.x, ey = b.y - a.y;
    real L = m_sqrt(ex * ex + ey * ey);
    real d = (ex * (p.y - a.y) - ey * (p.x - a.x)) / L;  // left of a->b (inside for CCW) is +
    if (d < 0) inside = false;
    best = m_min(best, d);
  }
  if (inside) return best;
  real near = m_inf();
  for (int i = 0; i < n; i++) near = m_min(near, seg_dist(p, poly[i], poly[(i + 1) % n]));
  return -near;
}

real stability(const BodyModel& m, const real xs[kLegs], const real zs[kLegs], uint8_t support, real pitch_deg,
               real roll_deg) {
  V3 com, feet[kLegs];
  world_points(m, xs, zs, pitch_deg, roll_deg, &com, feet);
  V2 pts[kLegs], poly[kLegs];
  int n = 0;
  for (int leg = 0; leg < kLegs; leg++)
    if (support >> leg & 1) pts[n++] = {feet[leg].x, feet[leg].y};
  int k = hull(pts, n, poly);
  return margin({com.x, com.y}, poly, k);
}

}  // namespace planner
}  // namespace kilo
