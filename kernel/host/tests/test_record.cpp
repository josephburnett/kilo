// The walk recorder: the store's ring (whole walks dropped oldest first, a
// walk that outgrows it keeps its start and its end, the index's limit, a
// reader of a dropped walk told so, never another walk's bytes), a full
// 20-minute walk in record::kStoreBytes, the recorder in the Mind on a
// scripted body (the MARK lines, PLAN per planned tick, FEET and TELEMETRY
// both ways, the pre- and post-roll, the summary lines in the RAM log),
// and what the RAM log keeps of held IR keys and percepts.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "mind.h"
#include "record/recorder.h"
#include "record/store.h"
#include "tests/body_rig.h"
#include "tests/check.h"

using namespace kilo;
using record::WalkEntry;
using record::WalkStore;

namespace {

struct Rec {
  uint8_t kind;
  uint32_t t_us;
  std::vector<uint8_t> payload;
};

// Parse a recording; ok false if the bytes do not end on a record boundary.
std::vector<Rec> parse(const std::vector<uint8_t>& b, bool* ok) {
  std::vector<Rec> out;
  size_t i = 0;
  while (i + KILO_WALK_RECORD_HEADER_SIZE <= b.size()) {
    kilo_walk_record_header_t h;
    std::memcpy(&h, b.data() + i, sizeof h);
    i += sizeof h;
    if (i + h.len > b.size()) break;
    out.push_back({h.kind, h.t_us, std::vector<uint8_t>(b.begin() + (long)i, b.begin() + (long)(i + h.len))});
    i += h.len;
  }
  *ok = i == b.size();
  return out;
}

std::vector<uint8_t> read_all(const WalkStore& s, uint32_t id, int32_t* status = nullptr) {
  std::vector<uint8_t> out;
  uint8_t buf[KILO_HOST_WALK_CHUNK_MAX];
  for (;;) {
    uint32_t total = 0;
    int32_t n = s.read(id, (uint32_t)out.size(), buf, sizeof buf, &total);
    if (status != nullptr) *status = n;
    if (n <= 0) return out;
    out.insert(out.end(), buf, buf + n);
  }
}

std::string text(const Rec& r) { return std::string(r.payload.begin(), r.payload.end()); }

// A record whose payload says which walk and which record it is.
void put(WalkStore& s, uint32_t walk, uint32_t i, uint32_t len) {
  uint8_t p[255];
  for (uint32_t k = 0; k < len; k++) p[k] = (uint8_t)(walk * 31 + i * 7 + k);
  s.append(KILO_WALK_RECORD_RX, i, p, len);
}

bool walk_intact(const WalkStore& s, uint32_t walk, uint32_t len) {
  bool ok;
  std::vector<Rec> rs = parse(read_all(s, walk), &ok);
  if (!ok || rs.empty()) return false;
  for (const Rec& r : rs) {
    if (r.payload.size() != len) return false;
    for (uint32_t k = 0; k < len; k++)
      if (r.payload[k] != (uint8_t)(walk * 31 + r.t_us * 7 + k)) return false;
  }
  return true;
}

void test_store_drops_oldest_whole_walks() {
  std::vector<uint8_t> mem(WalkStore::kIndexBytes + 8 + 20000);
  WalkStore s;
  s.attach(mem.data(), (uint32_t)mem.size());
  CHECK(s.ready() && s.capacity() >= 19990 && s.capacity() <= 20008);
  CHECK(s.open(1000) == 1 && s.recording_id() == 1);
  uint32_t total = 0;
  for (uint32_t i = 0; i < 40; i++) put(s, 1, i, 50);  // 40 x 56 B
  s.close();
  CHECK(s.recording_id() == 0 && s.kept() == 1 && s.used() == 40u * 56u);
  // walks 2..20, 3 KB each: the ring wraps several times; the oldest go whole
  for (uint32_t w = 2; w <= 20; w++) {
    CHECK(s.open(w * 1000) == w);
    for (uint32_t i = 0; i < 25; i++) put(s, w, i, 114);  // 25 x 120 B
    s.close();
    total += 3000;
  }
  CHECK(s.used() <= s.capacity());
  const WalkEntry* oldest = s.after(0);
  CHECK(oldest != nullptr && s.dropped_walks() == oldest->id - 1);
  CHECKF(s.kept() == 21 - oldest->id, "kept %u, oldest %u", s.kept(), oldest != nullptr ? oldest->id : 0);
  for (uint32_t w = oldest->id; w <= 20; w++) CHECKF(walk_intact(s, w, 114), "walk %u", w);
  uint8_t b[8];
  uint32_t t;
  CHECK(s.read(1, 0, b, sizeof b, &t) == -1 && s.find(1) == nullptr);  // dropped: gone, never other bytes
  CHECK(s.dropped_records() == 0);
  // a reader in the middle of walk K when K is dropped finds it gone
  uint32_t k = s.after(0)->id;
  CHECK(s.read(k, 100, b, sizeof b, &t) == 8);
  for (uint32_t w = 21; w <= 30; w++) {
    s.open(w);
    for (uint32_t i = 0; i < 25; i++) put(s, w, i, 114);
    s.close();
  }
  CHECK(s.read(k, 108, b, sizeof b, &t) == -1);
  CHECK(walk_intact(s, 30, 114));
}

void test_store_walk_that_outgrows_it() {
  std::vector<uint8_t> mem(WalkStore::kIndexBytes + 8 + 12000);
  WalkStore s;
  s.attach(mem.data(), (uint32_t)mem.size());
  s.open(1);
  for (uint32_t i = 0; i < 10; i++) put(s, 1, i, 100);
  s.close();
  s.open(2);
  uint32_t i = 0, kept = 0;
  for (; i < 1000; i++) kept += s.append(KILO_WALK_RECORD_RX, i, "0123456789", 10) ? 1 : 0;
  const char* end = "end ok=1 streamed=1 reason=stopped";
  CHECK(s.append(KILO_WALK_RECORD_MARK, i, end, (uint32_t)std::strlen(end), nullptr, 0, true));
  s.close();
  const WalkEntry* e = s.find(2);
  CHECK(s.find(1) == nullptr && s.dropped_walks() == 1);  // the older walk made room first
  CHECK(e != nullptr && (e->flags & KILO_WALK_FLAGS_TRUNCATED) && s.dropped_records() == 1000 - kept);
  CHECKF(kept > 500 && kept < 1000, "kept %u", kept);
  bool ok;
  std::vector<Rec> rs = parse(read_all(s, 2), &ok);
  CHECK(ok && rs.size() == kept + 1 && rs.back().kind == KILO_WALK_RECORD_MARK && text(rs.back()) == end);
  CHECK(rs.front().t_us == 0 && text(rs.front()) == "0123456789");  // its start is kept
}

void test_store_index_limit() {
  std::vector<uint8_t> mem(WalkStore::kIndexBytes + 8 + 100000);
  WalkStore s;
  s.attach(mem.data(), (uint32_t)mem.size());
  for (uint32_t w = 1; w <= WalkStore::kMaxWalks + 2; w++) {
    s.open(w);
    put(s, w, 0, 20);
  }
  s.close();
  CHECK(s.kept() == WalkStore::kMaxWalks && s.dropped_walks() == 2 && s.after(0)->id == 3);
  CHECK(s.after(WalkStore::kMaxWalks + 2) == nullptr && s.after(50)->id == 51);
  for (uint32_t w = 3; w <= WalkStore::kMaxWalks + 2; w++) CHECKF(walk_intact(s, w, 20), "walk %u", w);
  // too small a block: not ready, nothing recorded
  WalkStore tiny;
  tiny.attach(mem.data(), 1000);
  CHECK(!tiny.ready() && tiny.open(1) == 0 && !tiny.append(1, 0, "x", 1));
}

// 20 minutes of walking at 50 Hz, the records the recorder writes per tick
// in a walk (measured on the virtual body: TELEMETRY in, FEET out, PLAN; a
// SET_BALANCE refresh at 5 Hz; a PLANNER now and then), into the size the
// platforms allocate.
void test_store_twenty_minutes() {
  std::vector<uint8_t> mem(record::kStoreBytes);
  WalkStore s;
  s.attach(mem.data(), (uint32_t)mem.size());
  s.open(0);
  uint8_t tel[2 + KILO_TELEMETRY_LEN] = {KILO_MSG_TELEMETRY, 0};
  uint8_t feet[2 + KILO_FEET_LEN] = {KILO_MSG_FEET, 0};
  uint8_t bal[2 + KILO_SET_BALANCE_LEN] = {KILO_MSG_SET_BALANCE, 0};
  kilo_walk_plan_t plan = {};
  kilo_walk_planner_t pl = {};
  const uint32_t ticks = 20 * 60 * 50;
  uint32_t used_at_walk = 0;
  for (uint32_t t = 0; t < ticks; t++) {
    uint32_t us = t * 20000;
    s.append(KILO_WALK_RECORD_RX, us, tel, sizeof tel);
    s.append(KILO_WALK_RECORD_TX, us, feet, sizeof feet);
    s.append(KILO_WALK_RECORD_PLAN, us, &plan, sizeof plan);
    if (t % 10 == 0) s.append(KILO_WALK_RECORD_TX, us, bal, sizeof bal);
    if (t % 50 == 0) s.append(KILO_WALK_RECORD_PLANNER, us, &pl, sizeof pl);
    if (t == 0) used_at_walk = s.used();
  }
  s.close();
  const WalkEntry* e = s.find(1);
  CHECK(e != nullptr && !(e->flags & KILO_WALK_FLAGS_TRUNCATED) && s.dropped_records() == 0);
  double per_tick = (double)s.used() / ticks;
  double minutes = (double)s.capacity() / per_tick / 50.0 / 60.0;
  std::printf("  20 min at 50 Hz: %u B (%.1f B a tick); %u B of records hold %.0f minutes\n", (unsigned)s.used(),
              per_tick, (unsigned)s.capacity(), minutes);
  CHECK(minutes >= 25 && used_at_walk > 0);
}

// ---- the recorder in the Mind ------------------------------------------------------------

struct Rig {
  FakeBody body;
  CaptureLog log;
  Mind mind;
  std::vector<uint8_t> mem;
  explicit Rig(const MindConfig& cfg, uint32_t store = 1u << 20)
      : mind(body.serial, body.clock, log, nullptr, cfg), mem(store) {
    mind.attach_recorder(mem.data(), (uint32_t)mem.size());
    mind.start();
    run_ms(100);
  }
  void pass() {
    body.step();
    mind.poll();
    body.react();
  }
  void run_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i += 4) pass();
  }
  bool ring_has(const char* t) const {
    const LogRing& r = const_cast<Mind&>(mind).log();
    for (uint32_t s = r.first_seq(); s < r.next_seq(); s++) {
      const LogEntry* e = r.get(s);
      if (e != nullptr && std::strstr(e->text, t) != nullptr) return true;
    }
    return false;
  }
};

const Rec* mark(const std::vector<Rec>& rs, const char* word) {
  size_t n = std::strlen(word);
  for (const Rec& r : rs)
    if (r.kind == KILO_WALK_RECORD_MARK && r.payload.size() > n && std::memcmp(r.payload.data(), word, n) == 0 &&
        r.payload[n] == ' ')
      return &r;
  return nullptr;
}

size_t frames(const std::vector<Rec>& rs, uint8_t kind, uint8_t type) {
  size_t n = 0;
  for (const Rec& r : rs) n += r.kind == kind && !r.payload.empty() && r.payload[0] == type;
  return n;
}

void test_recorder_records_a_walk() {
  MindConfig cfg;
  cfg.remote = true;
  cfg.mind_build = 0x12345678;
  Rig r(cfg);
  r.body.pitch = 3 * 16;   // a tilted IMU: measured - predicted has an offset
  r.run_ms(2000);          // the pre-roll fills (idle TELEMETRY, heartbeats)
  record::WalkStore& st = r.mind.recorder().store();
  CHECK(st.kept() == 0 && !r.mind.recorder().recording());
  r.body.event(KILO_EVENT_IR_KEY, 0x68);  // A4: trot
  r.run_ms(40);
  CHECK(r.mind.walker().active() && r.mind.recorder().recording() && st.recording_id() == 1);
  r.run_ms(3000);
  r.body.event(KILO_EVENT_IR_KEY, 0x62);  // B7: faster
  r.run_ms(1500);
  r.body.event(KILO_EVENT_IR_KEY, 0x02);  // B6: the planned stop, then stand
  for (int k = 0; k < 400 && r.mind.walker().active(); k++) r.run_ms(20);
  CHECKF(!r.mind.walker().active() && r.mind.walker().ended() != nullptr, "%s",
         r.mind.walker().ended() ? r.mind.walker().ended() : "still walking");
  CHECK(r.mind.recorder().recording());  // the post-roll
  r.run_ms((uint32_t)(record::WalkRecorder::kPostRollUs / 1000) + 100);
  CHECK(!r.mind.recorder().recording() && st.recording_id() == 0 && st.kept() == 1);
  const WalkEntry* e = st.find(1);
  CHECK(e != nullptr && (e->flags & KILO_WALK_FLAGS_STREAMED) && (e->flags & KILO_WALK_FLAGS_ENDED_OK) &&
        !(e->flags & (KILO_WALK_FLAGS_RECORDING | KILO_WALK_FLAGS_TRUNCATED)));
  std::string info(e->text, e->text_len);
  CHECKF(info.find("gait=trot dir=forward speed=") == 0 && info.find(" end=stopped") != std::string::npos, "%s",
         info.c_str());
  bool ok;
  std::vector<Rec> rs = parse(read_all(st, 1), &ok);
  CHECK(ok && rs.size() == e->records);
  // the first record is the walk's MARK at t 0; the times never go back
  CHECK(!rs.empty() && rs[0].kind == KILO_WALK_RECORD_MARK && rs[0].t_us == 0 &&
        text(rs[0]).find("walk id=1 format=1 t0_us_hi=") == 0 && text(rs[0]).find("mind_build=12345678") != std::string::npos);
  bool mono = true;
  for (size_t i = 1; i < rs.size(); i++) mono = mono && rs[i].t_us >= rs[i - 1].t_us;
  CHECK(mono);
  const Rec* setup = mark(rs, "setup");
  CHECK(setup != nullptr && text(*setup).find("pattern=trot dir=forward speed_asked=") != std::string::npos);
  // the pre-roll: about 2 s of frames before the setup
  CHECKF(setup != nullptr && setup->t_us >= 1900000 && setup->t_us <= 2000000, "setup at %u us",
         setup ? setup->t_us : 0);
  size_t pre_tel = 0;
  for (const Rec& x : rs) pre_tel += x.kind == KILO_WALK_RECORD_RX && x.payload[0] == KILO_MSG_TELEMETRY && setup && x.t_us < setup->t_us;
  CHECKF(pre_tel >= 95 && pre_tel <= 101, "%zu TELEMETRY in the pre-roll", pre_tel);
  const char* words[] = {"stream", "config", "model", "masses", "change", "timing", "stop", "result", "error", "end"};
  for (const char* w : words) CHECKF(mark(rs, w) != nullptr, "no MARK %s", w);
  CHECK(mark(rs, "config") && text(*mark(rs, "config")).find("margin_mm=") != std::string::npos);
  CHECK(mark(rs, "masses") && text(*mark(rs, "masses")).find("battery_g=") != std::string::npos);
  CHECK(mark(rs, "end") && text(*mark(rs, "end")) == "end ok=1 streamed=1 reason=stopped");
  const planner::StreamStats& ss = r.mind.walker().stream_stats();
  size_t plans = 0, sent = 0;
  for (const Rec& x : rs) {
    if (x.kind != KILO_WALK_RECORD_PLAN) continue;
    plans++;
    kilo_walk_plan_t p;
    std::memcpy(&p, x.payload.data(), sizeof p);
    sent += p.sent;
  }
  CHECKF(plans == ss.sent + ss.skipped && sent == ss.sent, "%zu plans %zu sent, walker %u + %u", plans, sent,
         (unsigned)ss.sent, (unsigned)ss.skipped);
  CHECK(frames(rs, KILO_WALK_RECORD_TX, KILO_MSG_FEET) == ss.sent && e->feet == ss.sent);
  CHECK(frames(rs, KILO_WALK_RECORD_RX, KILO_MSG_TELEMETRY) > 400);  // 50 Hz in the walk
  CHECK(mark(rs, "result") && text(*mark(rs, "result")).find("sent=" + std::to_string(ss.sent)) != std::string::npos);
  // the error tracker saw the tilt (the scripted body reports ref 0: the offset is the tilt less the prediction)
  CHECK(mark(rs, "error") && text(*mark(rs, "error")).find("offset_pitch_deg=") != std::string::npos);
  // the walk's summary is in the RAM log
  CHECK(r.ring_has("walk trot forward") && r.ring_has(" holds ") && r.ring_has("walk error: "));
  // the post-roll ended about 2 s after the end
  const Rec& last = rs.back();
  const Rec* end = mark(rs, "end");
  CHECKF(end && last.t_us - end->t_us >= 1900000, "post-roll %u us", end ? last.t_us - end->t_us : 0);
  std::printf("  a %u-tick walk: %u records, %u B (%.1f B a tick sent)\n", (unsigned)ss.sent, (unsigned)e->records,
              (unsigned)e->size, (double)e->size / ss.sent);
}

void test_recorder_setup_failure_and_next_walk() {
  MindConfig cfg;
  cfg.remote = true;
  Rig r(cfg);
  // a walk that never gets its SET_STREAM: refused by the body
  r.body.answer = [](const Sent& s) -> uint8_t {
    return s.type == KILO_MSG_SET_STREAM ? KILO_ACK_STATUS_BUSY : KILO_ACK_STATUS_OK;
  };
  r.body.event(KILO_EVENT_IR_KEY, 0x68);  // A4: trot
  r.run_ms(200);
  CHECK(!r.mind.walker().active());
  r.body.answer = [](const Sent&) -> uint8_t { return KILO_ACK_STATUS_OK; };
  r.run_ms(500);  // inside the first walk's post-roll
  r.body.event(KILO_EVENT_IR_KEY, 0x68);  // trot again: the first recording closes now
  r.run_ms(2000);
  record::WalkStore& st = r.mind.recorder().store();
  const WalkEntry* a = st.find(1);
  CHECK(a != nullptr && !(a->flags & (KILO_WALK_FLAGS_RECORDING | KILO_WALK_FLAGS_STREAMED | KILO_WALK_FLAGS_ENDED_OK)));
  CHECK(a && std::string(a->text, a->text_len).find("end=SET_STREAM BUSY") != std::string::npos);
  CHECK(st.recording_id() == 2 && (st.find(2)->flags & KILO_WALK_FLAGS_STREAMED));
  bool ok;
  std::vector<Rec> rs = parse(read_all(st, 1), &ok);
  CHECK(ok && mark(rs, "end") && text(*mark(rs, "end")) == "end ok=0 streamed=0 reason=SET_STREAM BUSY" &&
        mark(rs, "stream") == nullptr && mark(rs, "error") == nullptr);
}

// ---- the RAM log keeps what matters ------------------------------------------------------

void test_ring_keeps_first_presses_not_repeats() {
  MindConfig cfg;
  cfg.remote = true;
  Rig r(cfg);
  r.body.event(KILO_EVENT_IR_KEY, 0x02);  // B6 pressed
  r.run_ms(40);
  for (int i = 0; i < 20; i++) {  // ... and held: a repeat every ~110 ms
    r.body.event(KILO_EVENT_IR_KEY, 0x102);
    r.run_ms(108);
  }
  CHECK(r.ring_has("event IR_KEY 0x0002") && r.ring_has("remote B6 (0x02, stand): posture stand"));
  CHECK(!r.ring_has("event IR_KEY 0x0102") && !r.ring_has("repeat: ignored"));
  CHECK(r.log.has("event IR_KEY 0x0102") && r.log.has("remote B6 (0x02, stand) repeat: ignored"));  // the platform's log has them
  r.body.event(KILO_EVENT_IR_KEY, 0x45);  // another remote's key, first press: kept
  r.run_ms(20);
  CHECK(r.ring_has("remote ? (0x45): ignored"));
  r.body.pitch = 40 * 16;  // percepts: the platform's log only
  r.run_ms(1000);
  CHECK(r.log.has("percept tilted nose_down") && !r.ring_has("percept "));
  r.mind.log().set_watched(true);  // a laptop reads the log: all of it
  r.body.pitch = 0;
  r.body.event(KILO_EVENT_IR_KEY, 0x102);
  r.run_ms(1000);
  CHECK(r.ring_has("percept level") && r.ring_has("event IR_KEY 0x0102"));
}

}  // namespace

int main() {
  test_store_drops_oldest_whole_walks();
  test_store_walk_that_outgrows_it();
  test_store_index_limit();
  test_store_twenty_minutes();
  test_recorder_records_a_walk();
  test_recorder_setup_failure_and_next_walk();
  test_ring_keeps_first_presses_not_repeats();
  std::printf("  sizeof(WalkRecorder) %zu B, sizeof(Mind) %zu B, index %u B, pre-roll %zu B\n",
              sizeof(record::WalkRecorder), sizeof(Mind), (unsigned)WalkStore::kIndexBytes,
              (size_t)record::WalkRecorder::kPreRoll * 72);
  return report("record");
}
