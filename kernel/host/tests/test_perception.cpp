// Perception against the Python detectors: replays perception_vectors.txt
// (gen_perception_vectors.py: the traces of mind/tests/test_perception.py
// through kilo.perception) and checks that every event comes out on the
// same sample with the same data, and the state each scenario ends in.
// Then the derived values of link/body_state (state.py) by hand, after
// mind/tests/test_state.py.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "link/body_state.h"
#include "perception/percepts.h"
#include "tests/check.h"

using namespace kilo;
using namespace kilo::perception;

namespace {

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    while (i < s.size() && s[i] == ' ') i++;
    size_t j = i;
    while (j < s.size() && s[j] != ' ') j++;
    if (j > i) out.push_back(s.substr(i, j - i));
    i = j;
  }
  return out;
}

bool num(const std::string& s, double* v) {
  if (s == "-") return false;
  *v = std::strtod(s.c_str(), nullptr);
  return true;
}

bool close(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

struct Replay {
  enum Kind { None, Us, Imu, Pc } kind = None;
  std::string name;
  UltrasonicDetector* us = nullptr;
  ImuDetector* imu = nullptr;
  Percepts* pc = nullptr;
  PerceptEvents got;
  int matched = 0;
  int line = 0;
  int scenarios = 0, events = 0, samples = 0;

  ~Replay() { drop(); }
  void drop() {
    delete us;
    delete imu;
    delete pc;
    us = nullptr;
    imu = nullptr;
    pc = nullptr;
  }
  void check_all_matched() {
    for (int i = matched; i < got.n; i++)
      CHECKF(false, "%s line %d: unexpected %s at %.3f", name.c_str(), line, percept_name(got.e[i].kind), got.e[i].t);
    got.n = 0;
    matched = 0;
  }
  void start(Kind k, const std::vector<std::string>& w) {
    check_all_matched();
    drop();
    kind = k;
    name = w.size() > 1 ? w[1] : "?";
    scenarios++;
    UltrasonicConfig uc;
    ImuConfig ic;
    for (size_t i = 2; i < w.size(); i++) {
      size_t eq = w[i].find('=');
      std::string key = w[i].substr(0, eq);
      double v = std::strtod(w[i].c_str() + eq + 1, nullptr);
      if (key == "near_mm") uc.near_mm = v;
      else if (key == "near_exit_mm") uc.near_exit_mm = v;
      else if (key == "pickup_deg") ic.pickup_deg = v;
      else CHECKF(false, "unknown config key %s", key.c_str());
    }
    if (k == Us) us = new UltrasonicDetector(uc);
    if (k == Imu) imu = new ImuDetector(ic);
    if (k == Pc) pc = new Percepts(uc, ic);
  }
  void event(const std::vector<std::string>& w) {
    if (matched >= got.n) {
      CHECKF(false, "%s line %d: missing %s at %s", name.c_str(), line, w[1].c_str(), w[2].c_str());
      return;
    }
    const PerceptEvent& e = got.e[matched++];
    events++;
    double t = std::strtod(w[2].c_str(), nullptr);
    CHECKF(w[1] == percept_name(e.kind) && e.t == t, "%s line %d: want %s at %s, got %s at %.6f", name.c_str(), line,
           w[1].c_str(), w[2].c_str(), percept_name(e.kind), e.t);
    // the data, as Python rounds it (1 decimal; 3 for the tap's times)
    auto val = [&](size_t i, bool has, double v, double tol) {
      double want;
      bool have = num(w[i], &want);
      CHECKF(have == has && (!have || close(v, want, tol)), "%s line %d: %s value %zu: want %s, got %.4f", name.c_str(),
             line, w[1].c_str(), i - 3, w[i].c_str(), has ? v : -999.0);
    };
    switch (e.kind) {
      case Percept::Near:
      case Percept::Clear:
        if (w[3] == "no_echo") CHECK(e.no_echo);
        else val(3, e.has_a, e.a, 0.051);
        break;
      case Percept::Approaching:
        val(3, true, e.a, 0.051);
        val(4, true, e.b, 0.051);
        break;
      case Percept::Tilted:
        CHECK(w[3] == tilt_direction_name(e.direction));
        val(4, true, e.a, 0.051);
        val(5, true, e.b, 0.051);
        break;
      case Percept::Level:
      case Percept::PutDown:
        val(3, true, e.a, 0.051);
        val(4, true, e.b, 0.051);
        break;
      case Percept::PickedUp:
        val(3, true, e.a, 0.051);
        val(4, true, e.b, 0.051);
        CHECK((w[5] == "1") == e.servos_powered);
        break;
      case Percept::Tap:
        val(3, true, e.a, 0.00051);
        val(4, true, e.b, 0.051);
        val(5, true, e.c, 0.00051);
        break;
      case Percept::Pet:
        val(3, true, e.a, 0.051);
        CHECK(std::atoi(w[4].c_str()) == (int)e.b);
        break;
      case Percept::Startle:
        val(3, e.has_a, e.a, 0.051);
        val(4, e.has_b, e.b, 0.051);
        break;
      default:
        CHECK(false);
    }
  }
  void state(const std::vector<std::string>& w) {
    check_all_matched();
    const std::string& k = w[1];
    double want = 0;
    bool has = num(w[2], &want);
    bool b = want != 0;
    if (kind == Us) {
      if (k == "near") CHECKF(us->near() == b, "%s: near", name.c_str());
      else if (k == "clear") CHECKF(us->clear() == b, "%s: clear", name.c_str());
      else if (k == "approaching") CHECKF(us->approaching() == b, "%s: approaching", name.c_str());
      else if (k == "distance")
        CHECKF(us->has_distance() == has && (!has || us->distance_mm() == want), "%s: distance", name.c_str());
    } else if (kind == Imu) {
      if (k == "tilted") CHECKF(imu->tilted() == b, "%s: tilted", name.c_str());
      else if (k == "picked_up") CHECKF(imu->picked_up() == b, "%s: picked_up", name.c_str());
      else if (k == "pet") CHECKF(imu->pet() == b, "%s: pet", name.c_str());
      else if (k == "ref_p") CHECKF(close(imu->ref_pitch(), want, 1e-9), "%s: ref_p %.6f", name.c_str(), imu->ref_pitch());
      else if (k == "ref_r") CHECKF(close(imu->ref_roll(), want, 1e-9), "%s: ref_r %.6f", name.c_str(), imu->ref_roll());
    } else if (kind == Pc) {
      bool v = k == "near" ? pc->near() : k == "approaching" ? pc->approaching() : k == "clear" ? pc->clear()
               : k == "tilted" ? pc->tilted() : k == "picked_up" ? pc->picked_up() : k == "tap" ? pc->tap()
               : k == "pet" ? pc->pet() : k == "startle" ? pc->startle() : b;
      if (k == "distance") {
        const UltrasonicDetector& u = pc->ultrasonic();
        CHECKF(u.has_distance() == has && (!has || u.distance_mm() == want), "%s: distance", name.c_str());
      } else {
        CHECKF(v == b, "%s: %s", name.c_str(), k.c_str());
      }
    }
  }
};

int replay(const char* path) {
  FILE* f = std::fopen(path, "r");
  if (f == nullptr) {
    std::printf("cannot open %s\n", path);
    return 1;
  }
  Replay r;
  char buf[512];
  while (std::fgets(buf, sizeof buf, f)) {
    r.line++;
    std::string s(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    std::vector<std::string> w = split(s);
    if (w.empty() || w[0] == "#") continue;
    const std::string& c = w[0];
    if (c == "U") r.start(Replay::Us, w);
    else if (c == "I") r.start(Replay::Imu, w);
    else if (c == "P") r.start(Replay::Pc, w);
    else if (c == "e") r.event(w);
    else if (c == "x") r.state(w);
    else if (c == "r") r.imu->set_reference(std::strtod(w[1].c_str(), nullptr), std::strtod(w[2].c_str(), nullptr));
    else if (c == "u" || c == "i" || c == "p") {
      r.check_all_matched();
      r.samples++;
      double t = std::strtod(w[1].c_str(), nullptr);
      if (c == "u") {
        double d = 0;
        bool has = num(w[2], &d);
        r.us->feed(t, has, d, r.got);
      } else if (c == "i") {
        ImuSample im = {};
        im.t = t;
        im.pitch = std::atoi(w[2].c_str()) / 16.0;
        im.roll = std::atoi(w[3].c_str()) / 16.0;
        im.has_rates = w[4] != "-";
        if (im.has_rates) {
          int a, b, cc;
          std::sscanf(w[4].c_str(), "%d,%d,%d", &a, &b, &cc);
          im.rates[0] = a / 16.0;
          im.rates[1] = b / 16.0;
          im.rates[2] = cc / 16.0;
        }
        im.has_flags = w[5] != "-";
        im.flags = (uint16_t)std::atoi(w[5].c_str());
        im.mode = std::atoi(w[6].c_str());
        r.imu->feed(im, r.got);
      } else {
        kilo_telemetry_t m = {};
        m.pitch = (int16_t)std::atoi(w[2].c_str());
        m.roll = (int16_t)std::atoi(w[3].c_str());
        m.rate_pitch = (int16_t)std::atoi(w[4].c_str());
        m.rate_roll = (int16_t)std::atoi(w[5].c_str());
        m.rate_yaw = (int16_t)std::atoi(w[6].c_str());
        m.dist_mm = (uint16_t)std::atoi(w[7].c_str());
        m.batt_mv = (uint16_t)std::atoi(w[8].c_str());
        m.mode = (uint8_t)std::atoi(w[9].c_str());
        m.flags = (uint16_t)std::atoi(w[10].c_str());
        const PerceptEvents& ev = r.pc->update(m, t);
        r.got = ev;
        // the same telemetry polled twice adds nothing
        CHECK(r.pc->update(m, t).n == 0);
      }
    } else {
      CHECKF(false, "line %d: unknown %s", r.line, c.c_str());
    }
  }
  r.check_all_matched();
  std::fclose(f);
  std::printf("perception vectors: %d scenarios, %d samples, %d events\n", r.scenarios, r.samples, r.events);
  CHECK(r.scenarios >= 30 && r.events >= 40);
  return 0;
}

// ---- body_state's derived values (mind/tests/test_state.py) --------------------------------

void test_body_state_derived() {
  CHECK(battery_percent(6000) == 0.0 && battery_percent(9000) == 100.0);
  CHECK(std::fabs(battery_percent(7700) - 57.5) < 1e-9 && battery_percent(7300) == 20.0);
  BodyState st;
  double v;
  CHECK(!st.distance_mm(&v) && !st.approach_mm_s(&v) && !st.battery_pct(&v) && st.upright() == -1);
  kilo_telemetry_t t = {};
  int16_t prev;
  t.mode = KILO_MODE_IDLE;
  t.batt_mv = 7700;
  t.flags = KILO_TELEMETRY_FLAGS_SERVOS_POWERED;
  const uint16_t d[6] = {500, 0, 480, 900, 440, 420};  // 0 = no echo, skipped; 900 a spike
  for (int i = 0; i < 6; i++) {
    t.dist_mm = d[i];
    st.on_telemetry(t, 1000000u + i * 100000u, &prev);
  }
  CHECK(st.distance_mm(&v) && v == 440.0);  // median of 900, 440, 420
  // medians 500 (t 1.0), 490 (1.2), 500 (1.3), 480 (1.4), 440 (1.5): (500 - 440) / 0.5 s
  CHECK(st.approach_mm_s(&v) && std::fabs(v - 120.0) < 1e-9);
  CHECK(st.battery_pct(&v) && std::fabs(v - 57.5) < 1e-9);
  CHECK(st.upright() == 1 && !st.fallen() && !st.moving());
  t.flags |= KILO_TELEMETRY_FLAGS_NO_BATTERY;
  t.pitch = 31 * 16;
  t.mode = KILO_MODE_RECOVERING;
  st.on_telemetry(t, 2000000u, &prev);
  CHECK(!st.battery_pct(&v) && st.upright() == 0 && st.fallen() && st.moving());
  t.mode = KILO_MODE_IDLE;
  t.pitch = 0;
  t.flags = KILO_TELEMETRY_FLAGS_TRICK;
  st.on_telemetry(t, 2100000u, &prev);
  CHECK(st.moving() && !st.fallen() && st.upright() == 1);
  st.reset_session();
  CHECK(!st.distance_mm(&v) && !st.approach_mm_s(&v));
}

void test_percepts_flags() {
  Percepts pc;
  CHECK(!pc.has_t() && !pc.near() && !pc.tap() && !pc.startle());
  kilo_telemetry_t m = {};
  m.mode = KILO_MODE_IDLE;
  m.flags = KILO_TELEMETRY_FLAGS_SERVOS_POWERED | KILO_TELEMETRY_FLAGS_DIST_FRESH;
  m.dist_mm = 100;
  for (int i = 0; i < 4; i++) pc.update(m, i * 0.08);
  CHECK(pc.near() && pc.recent_count() == 1 && pc.recent_event(0).kind == Percept::Near);
  double t;
  CHECK(pc.last_event_time(Percept::Near, &t) && t == 0.08 && !pc.last_event_time(Percept::Tap, &t));
}

}  // namespace

int main(int argc, char** argv) {
  const char* vectors = "tests/perception_vectors.txt";
  for (int i = 1; i < argc; i++)
    if (std::strstr(argv[i], "perception_vectors") != nullptr) vectors = argv[i];
  replay(vectors);
  test_body_state_derived();
  test_percepts_flags();
  return report("perception");
}
