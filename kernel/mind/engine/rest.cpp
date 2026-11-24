#include "engine/rest.h"
#include "tricks/model.h"

namespace kilo {
namespace engine {

using planner::degrees;
using planner::kFront;
using planner::kLeft;
using planner::kLegs;
using planner::kPi;
using planner::m_atan2;
using planner::m_cos;
using planner::m_fabs;
using planner::m_max;
using planner::m_min;
using planner::m_sin;
using planner::m_sqrt;
using planner::radians;

namespace {

const int8_t kLimits[16][2] = KILO_JOINT_LIMITS_INIT;
constexpr real kTouchMm = (real)0.5;  // stance.TOUCH_MM: a point this close to the floor touches it
constexpr int kMaxTopples = 40;

const char* const kNames[kPoints] = {
    "lf_foot", "rf_foot", "rb_foot", "lb_foot", "lf_knee", "rf_knee", "rb_knee", "lb_knee", "lf_hip", "rf_hip",
    "rb_hip",  "lb_hip",  "tail",    "haunch_l", "haunch_r", "belly_l", "belly_r", "back_l", "back_r", "pi_l",
    "pi_r",    "nose_l",  "nose_r",  "top_l",   "top_r"};

real asin_(real x) {
  x = m_max((real)-1, m_min((real)1, x));
  return m_atan2(x, m_sqrt(m_max((real)0, 1 - x * x)));
}
real acos_(real x) {
  x = m_max((real)-1, m_min((real)1, x));
  return m_atan2(m_sqrt(m_max((real)0, 1 - x * x)), x);
}

V3 mul(const real R[3][3], V3 p) {
  return {R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z, R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z,
          R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z};
}

void matmul(const real A[3][3], const real B[3][3], real C[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
}

// Body -> world for a nose-up pitch and a left-down roll (stance.rot: Rx(roll) Ry(pitch)).
void rot(real pitch, real roll, real R[3][3]) {
  real p = radians(pitch), r = radians(roll);
  real cp = m_cos(p), sp = m_sin(p), cr = m_cos(r), sr = m_sin(r);
  const real Ry[3][3] = {{cp, 0, -sp}, {0, 1, 0}, {sp, 0, cp}};
  const real Rx[3][3] = {{1, 0, 0}, {0, cr, sr}, {0, -sr, cr}};
  matmul(Rx, Ry, R);
}

// Rodrigues: a turn of ang about the unit axis a.
void axis_rot(V3 a, real ang, real R[3][3]) {
  real s = m_sin(ang), c = 1 - m_cos(ang);
  const real K[3][3] = {{0, -a.z, a.y}, {a.z, 0, -a.x}, {-a.y, a.x, 0}};
  real K2[3][3];
  matmul(K, K, K2);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) R[i][j] = (i == j ? (real)1 : (real)0) + s * K[i][j] + c * K2[i][j];
}

// The tail's point frac of the way from its pivot to its tip at servo angle tail (sagittal.tail_point).
V3 tail_point(const BodyModel& m, real tail, real frac) {
  const BodyModel::Tail& t = m.tail;
  real ax = -m_cos(radians(t.axis_deg)), az = m_sin(radians(t.axis_deg));
  real dx = -m_cos(radians(t.droop_deg)), dz = -m_sin(radians(t.droop_deg));
  real ca = ax * dx + az * dz;
  real sa = m_sqrt(m_max((real)0, 1 - ca * ca));
  real ux = (dx - ca * ax) / sa, uz = (dz - ca * az) / sa;
  real s = radians(t.sign * tail);
  real r = t.len * frac, c = r * ca, w = r * sa;
  return {t.pivot_x + c * ax + w * m_cos(s) * ux, w * m_sin(s), t.pivot_z + c * az + w * m_cos(s) * uz};
}

// A leg's shoulder axle, knee and foot centre (sagittal.leg_points, each leg at its own side).
void leg_points(const BodyModel& m, Servos s, const Joints& j, int leg, V3* h, V3* k, V3* f) {
  real sh = applied(m, s, shoulder(leg), j.a[shoulder(leg)]);
  real kn = applied(m, s, knee(leg), j.a[knee(leg)]);
  real hx = kFront[leg] ? m.length_mm / 2 : -m.length_mm / 2;
  real y = kLeft[leg] ? m.track_mm / 2 : -m.track_mm / 2;
  real t1 = radians(-sh);
  real t2 = kFront[leg] ? radians(90 - kn) : radians(-kn - 90);
  real kx = hx + m.l1_mm * m_sin(t1), kz = -m.l1_mm * m_cos(t1);
  *h = {hx, y, 0};
  *k = {kx, y, kz};
  *f = {kx + m.l2_mm * m_sin(t1 + t2), y, kz - m.l2_mm * m_cos(t1 + t2)};
}

void points(const BodyModel& m, Servos s, const Joints& j, V3 p[kPoints], real r[kPoints]) {
  for (int leg = 0; leg < kLegs; leg++) {
    leg_points(m, s, j, leg, &p[kHipLF + leg], &p[kKneeLF + leg], &p[kFootLF + leg]);
    r[kFootLF + leg] = m.foot_r_mm;
    r[kKneeLF + leg] = 0;
    r[kHipLF + leg] = m.outline.hip_r;
  }
  p[kTailTip] = tail_point(m, applied(m, s, kTailJ, j.a[kTailJ]), 1);
  r[kTailTip] = 0;
  const BodyModel::Outline& o = m.outline;
  real w = o.half_width;
  for (int side = 0; side < 2; side++) {
    real y = side == 0 ? w : -w;
    p[kHaunchL + side] = {o.rear_x, y, o.belly_z};
    p[kBellyL + side] = {m.length_mm / 2, y, o.belly_z};
    p[kBackL + side] = {o.rear_x, y, o.back_z};
    p[kPiL + side] = {o.pi_rear_x, y, o.pi_top_z};
    p[kNoseL + side] = {o.nose_x, y * (real)0.5, o.nose_z};
    p[kTopL + side] = {m.length_mm / 2, y, o.pi_top_z};
  }
  for (int i = kHaunchL; i < kPoints; i++) r[i] = 0;
}

struct HullPt {
  real x, y;
  int8_t i;  // the point
};

real cross(const HullPt& o, const HullPt& a, const HullPt& b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// The 2-D convex hull (counter-clockwise; collinear points dropped), stance._hull2.
int hull(HullPt* pts, int n, HullPt* out) {
  for (int a = 1; a < n; a++)  // sort by (x, y)
    for (int b = a; b > 0 && (pts[b].x < pts[b - 1].x || (pts[b].x == pts[b - 1].x && pts[b].y < pts[b - 1].y));
         b--) {
      HullPt t = pts[b];
      pts[b] = pts[b - 1];
      pts[b - 1] = t;
    }
  if (n <= 2) {
    for (int a = 0; a < n; a++) out[a] = pts[a];
    return n;
  }
  HullPt lo[kPoints + 1], up[kPoints + 1];
  int nl = 0, nu = 0;
  for (int a = 0; a < n; a++) {
    while (nl >= 2 && cross(lo[nl - 2], lo[nl - 1], pts[a]) <= (real)1e-9) nl--;
    lo[nl++] = pts[a];
  }
  for (int a = n - 1; a >= 0; a--) {
    while (nu >= 2 && cross(up[nu - 2], up[nu - 1], pts[a]) <= (real)1e-9) nu--;
    up[nu++] = pts[a];
  }
  int k = 0;
  for (int a = 0; a < nl - 1; a++) out[k++] = lo[a];
  for (int a = 0; a < nu - 1; a++) out[k++] = up[a];
  return k;
}

// The signed distance of c from the polygon's boundary (+ inside; a point or a segment has no inside) and
// the nearest feature: an edge (a, b) or a point (a, b = -1). stance._poly_margin.
real poly_margin(const HullPt* poly, int n, real cx, real cy, int* fa, int* fb) {
  if (n == 1) {
    *fa = 0;
    *fb = -1;
    real dx = cx - poly[0].x, dy = cy - poly[0].y;
    return -m_sqrt(dx * dx + dy * dy);
  }
  int ne = n > 2 ? n : 1;
  real best_out = (real)1e18, best_in = (real)1e18;
  int oa = 0, ob = -1, ia = 0, ib = 1;
  bool inside = n > 2;
  for (int e = 0; e < ne; e++) {
    const HullPt& a = poly[e];
    const HullPt& b = poly[(e + 1) % n];
    real ex = b.x - a.x, ey = b.y - a.y;
    real L2 = ex * ex + ey * ey;
    if (L2 < (real)1e-12) L2 = (real)1e-12;  // (two points on one spot: the edge is a point)
    real t =((cx - a.x) * ex + (cy - a.y) * ey) / L2;
    real dist;
    int fa_ = e, fb_ = -1;
    if (t <= 0) {
      dist = m_sqrt((cx - a.x) * (cx - a.x) + (cy - a.y) * (cy - a.y));
    } else if (t >= 1) {
      dist = m_sqrt((cx - b.x) * (cx - b.x) + (cy - b.y) * (cy - b.y));
      fa_ = (e + 1) % n;
    } else {
      real qx = cx - a.x - t * ex, qy = cy - a.y - t * ey;
      dist = m_sqrt(qx * qx + qy * qy);
      fb_ = (e + 1) % n;
    }
    if (dist < best_out) {
      best_out = dist;
      oa = fa_;
      ob = fb_;
    }
    if (n > 2) {
      real d = (ex * (cy - a.y) - ey * (cx - a.x)) / m_sqrt(L2);
      if (d < 0) inside = false;
      if (d < best_in) {
        best_in = d;
        ia = e;
        ib = (e + 1) % n;
      }
    }
  }
  if (inside) {
    *fa = ia;
    *fb = ib;
    return best_in;
  }
  *fa = oa;
  *fb = ob;
  return -best_out;
}

}  // namespace

const char* point_name(int p) { return p >= 0 && p < kPoints ? kNames[p] : "?"; }

real applied(const BodyModel& m, Servos s, int k, real a) {
  int id = tricks::kCanonical[k];
  real lo = kLimits[id][0], hi = kLimits[id][1];
  if (s != Servos::Asked) {
    real zero = s == Servos::Reseated && id == 10 ? (real)KILO_MODEL_SERVO_RESEATED_RB_SHOULDER_CZP : m.czp[id];
    m.reach(id, zero, lo, hi, &lo, &hi);
  }
  return m_min(hi, m_max(lo, a));
}

V3 com(const BodyModel& m, Servos s, const Joints& j) {
  // the frame's lump is the body lump less the tail at servo 0, moved so that the whole robot's centre of
  // mass moves by the correction (sagittal.masses / stance.masses)
  real total = m.total_g();
  real frame_g = m.body.grams - m.tail.g;
  V3 t0 = tail_point(m, 0, m.tail.com_frac);
  real fx = (m.body.grams * m.body.x - m.tail.g * t0.x) / frame_g;
  real fz = (m.body.grams * m.body.z - m.tail.g * t0.z) / frame_g;
  real g = frame_g, sx = frame_g * (fx + m.com_dx_mm * total / frame_g), sy = frame_g * (m.com_dy_mm * total / frame_g),
       sz = frame_g * (fz + m.com_dz_mm * total / frame_g);
  sx += m.battery.grams * m.battery.x;
  sz += m.battery.grams * m.battery.z;
  sx += m.pi.grams * m.pi.x;
  sz += m.pi.grams * m.pi.z;
  g += m.battery.grams + m.pi.grams;
  for (int leg = 0; leg < kLegs; leg++) {
    V3 h, k, f;
    leg_points(m, s, j, leg, &h, &k, &f);
    real ux = h.x + (k.x - h.x) * m.upper_leg_at, uz = h.z + (k.z - h.z) * m.upper_leg_at;
    real lx = k.x + (f.x - k.x) * m.lower_leg_at, lz = k.z + (f.z - k.z) * m.lower_leg_at;
    sx += m.upper_leg_g * ux + m.lower_leg_g * lx;
    sy += (m.upper_leg_g + m.lower_leg_g) * h.y;
    sz += m.upper_leg_g * uz + m.lower_leg_g * lz;
    g += m.upper_leg_g + m.lower_leg_g;
  }
  V3 t = tail_point(m, applied(m, s, kTailJ, j.a[kTailJ]), m.tail.com_frac);
  sx += m.tail.g * t.x;
  sy += m.tail.g * t.y;
  sz += m.tail.g * t.z;
  g += m.tail.g;
  return {sx / g, sy / g, sz / g};
}

Rest rest(const BodyModel& m, Servos s, const Joints& j, const Rest* from) {
  V3 P[kPoints];
  real rad[kPoints];
  points(m, s, j, P, rad);
  V3 cb = com(m, s, j);
  Rest out = {};
  if (from != nullptr) {
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++) out.R[a][b] = from->R[a][b];
  } else {
    rot(0, 0, out.R);
  }
  V3 W[kPoints];
  real cx = 0, cy = 0, cz = 0, m0 = 0;
  uint32_t touch = 0;
  for (;;) {
    real floor = (real)1e18;
    for (int i = 0; i < kPoints; i++) {
      W[i] = mul(out.R, P[i]);
      floor = m_min(floor, W[i].z - rad[i]);
    }
    for (int i = 0; i < kPoints; i++) W[i].z -= floor;
    V3 c = mul(out.R, cb);
    cx = c.x;
    cy = c.y;
    cz = c.z - floor;
    HullPt pts[kPoints], poly[kPoints];
    int n = 0;
    touch = 0;
    for (int i = 0; i < kPoints; i++)
      if (W[i].z - rad[i] <= kTouchMm) {
        touch |= 1u << i;
        // (a duplicate point would be dropped by stance's set(); two body points never coincide on the floor here)
        pts[n++] = {W[i].x, W[i].y, (int8_t)i};
      }
    int np = hull(pts, n, poly);
    int fa = 0, fb = -1;
    m0 = poly_margin(poly, np, cx, cy, &fa, &fb);
    if (m0 >= 0 || out.topples >= kMaxTopples) break;
    // topple about the nearest support edge (or point), toward the centre of mass
    V3 p0 = {poly[fa].x, poly[fa].y, 0};
    V3 axis;
    if (fb >= 0) {
      axis = {poly[fb].x - poly[fa].x, poly[fb].y - poly[fa].y, 0};
    } else {
      real dx = cx - p0.x, dy = cy - p0.y;
      axis = {-dy, dx, 0};
    }
    real L = m_sqrt(axis.x * axis.x + axis.y * axis.y);
    if (L < (real)1e-9) break;
    axis = {axis.x / L, axis.y / L, 0};
    V3 u1 = {-axis.y, axis.x, 0};  // up x axis: horizontal, perpendicular to the axis
    if (u1.x * (cx - p0.x) + u1.y * (cy - p0.y) < 0) {
      axis = {-axis.x, -axis.y, 0};
      u1 = {-u1.x, -u1.y, 0};
    }
    // the smallest turn that brings another point down onto the floor
    real best = kPi / 2, best_drop = 0;
    for (int i = 0; i < kPoints; i++) {
      real alpha = (W[i].x - p0.x) * u1.x + (W[i].y - p0.y) * u1.y;
      real beta = W[i].z;
      real rho = m_sqrt(alpha * alpha + beta * beta);
      real th = kPi / 2;
      if (alpha > (real)1e-6 && rho > rad[i]) th = acos_(m_min((real)1, rad[i] / rho)) - m_atan2(alpha, beta);
      if (th < (real)1e-7) th = kPi / 2;
      if (th < best) {
        best = th;
        best_drop = beta - rad[i];
      }
    }
    real Rt[3][3];
    axis_rot(axis, best, Rt);
    if (mul(Rt, u1).z > 0) axis_rot(axis, -best, Rt);
    real Rn[3][3];
    matmul(Rt, out.R, Rn);
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++) out.R[a][b] = Rn[a][b];
    out.topples++;
    out.drop = m_max(out.drop, best_drop);
  }
  out.margin = m0;
  out.contacts = touch;
  out.com_h = cz;
  // attitude (stance.attitude): the pitch from the body's x axis, the roll from its y axis
  out.pitch = degrees(asin_(out.R[2][0]));
  out.roll = degrees(m_atan2(-out.R[2][1], out.R[2][2]));
  return out;
}

real telemetry_pitch(const BodyModel& m, const Rest& r) { return -r.pitch + m.imu_pitch_deg; }
real telemetry_roll(const BodyModel& m, const Rest& r) { return r.roll + m.imu_roll_deg; }

void Along::add(const Rest& r) {
  margin = m_min(margin, r.margin);
  drop = m_max(drop, r.drop);
  contacts |= r.contacts;
  roll_max = m_max(roll_max, m_fabs(r.roll));
  pitch_lo = m_min(pitch_lo, r.pitch);
  pitch_hi = m_max(pitch_hi, r.pitch);
}

void follow(const BodyModel& m, Servos s, const Joints& a, const Joints& b, Rest* cur, Along* along,
            real deg_per_sample) {
  real span = 0;
  for (int k = 0; k < kJoints; k++)
    span = m_max(span, m_fabs(applied(m, s, k, b.a[k]) - applied(m, s, k, a.a[k])));
  int n = (int)(span / deg_per_sample);
  if ((real)n < span / deg_per_sample) n++;
  if (n < 2) n = 2;
  for (int i = 1; i <= n; i++) {
    Joints j;
    real f = (real)i / (real)n;
    for (int k = 0; k < kJoints; k++) j.a[k] = a.a[k] + (b.a[k] - a.a[k]) * f;
    *cur = rest(m, s, j, cur);
    along->add(*cur);
  }
}

}  // namespace engine
}  // namespace kilo
