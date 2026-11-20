// The flight recorder: the block store (seal, CRC, ring overwrite, time
// lookup), the recorder in the Mind on a scripted body (everything from
// boot, a walk run, a trick run and what stopped it, a refused trick, the
// session metrics), what the RAM log keeps of a 15-minute play session
// (and why the old 64-line ring lost it), and the card saver on a fake card:
// safe moments only, 4 KB writes, a slow write's back-off, a power cut in
// the middle of a save, the next boot's scan, the cap, the bench, reading
// back, deleting.
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include "flash/crc32.h"
#include "mind.h"
#include "record/flight.h"
#include "record/recorder.h"
#include "record/saver.h"
#include "tests/body_rig.h"
#include "tests/check.h"
#include "tests/fake_card.h"
#include "tricks/library.h"

using namespace kilo;
using record::FlightStore;

namespace {

struct Rec {
  uint8_t kind;
  uint64_t t_us;
  std::string payload;
};

struct Block {
  kilo_record_block_header_t h;
  std::vector<Rec> recs;
};

// Blocks back to back; false if a block is malformed or its CRC is wrong.
bool parse_blocks(const uint8_t* b, size_t n, std::vector<Block>* out) {
  size_t at = 0;
  while (at + KILO_RECORD_BLOCK_HEADER_SIZE <= n) {
    Block blk;
    std::memcpy(&blk.h, b + at, sizeof blk.h);
    const kilo_record_block_header_t& h = blk.h;
    if (h.magic != record::kBlockMagic || h.used < sizeof h || at + h.used > n) return false;
    if (h.crc32 != 0) {
      uint32_t c = crc32(b + at + sizeof h, h.used - (uint32_t)sizeof h);
      if (c == 0) c = 1;
      if (c != h.crc32) return false;
    }
    uint64_t t0 = ((uint64_t)h.t0_us_hi << 32) | h.t0_us_lo;
    size_t p = at + sizeof h;
    while (p + KILO_WALK_RECORD_HEADER_SIZE <= at + h.used) {
      kilo_walk_record_header_t rh;
      std::memcpy(&rh, b + p, sizeof rh);
      p += sizeof rh;
      if (p + rh.len > at + h.used) return false;
      blk.recs.push_back({rh.kind, t0 + rh.t_us, std::string((const char*)b + p, rh.len)});
      p += rh.len;
    }
    if (p != at + h.used || blk.recs.size() != h.records) return false;
    at += h.used;
    out->push_back(blk);
  }
  return at == n;
}

// Every record in RAM, in order.
std::vector<Rec> ram_records(const FlightStore& f) {
  std::vector<Rec> out;
  if (f.empty()) return out;
  for (uint32_t s = f.first_seq(); s <= f.last_seq(); s++) {
    std::vector<Block> bs;
    CHECK(parse_blocks(f.block(s), f.used(s), &bs) && bs.size() == 1);
    for (Block& b : bs) out.insert(out.end(), b.recs.begin(), b.recs.end());
  }
  return out;
}

const Rec* mark(const std::vector<Rec>& rs, const char* word, size_t from = 0) {
  size_t n = std::strlen(word);
  for (size_t i = from; i < rs.size(); i++) {
    const Rec& r = rs[i];
    if (r.kind == KILO_WALK_RECORD_MARK && r.payload.size() >= n && r.payload.compare(0, n, word) == 0 &&
        (r.payload.size() == n || r.payload[n] == ' '))
      return &r;
  }
  return nullptr;
}

size_t count_kind(const std::vector<Rec>& rs, uint8_t kind, int type = -1) {
  size_t n = 0;
  for (const Rec& r : rs) n += r.kind == kind && (type < 0 || (!r.payload.empty() && (uint8_t)r.payload[0] == type));
  return n;
}

// ---- the block store --------------------------------------------------------------------

void test_flight_store() {
  std::vector<uint8_t> mem(4 * record::kBlockBytes + 100);
  FlightStore f;
  f.attach(mem.data(), (uint32_t)mem.size());
  CHECK(f.ready() && f.empty() && f.capacity() == 4 * record::kBlockBytes);
  uint8_t p[200];
  for (int i = 0; i < 200; i++) p[i] = (uint8_t)i;
  uint64_t t = 5000000;
  // 300 records of 206 B: about 3.8 blocks
  for (int i = 0; i < 300; i++, t += 20000) CHECK(f.append(KILO_WALK_RECORD_RX, t, p, sizeof p));
  CHECK(f.blocks() == 4 && f.first_seq() == 0 && f.open() && f.dropped_blocks() == 0 && f.records() == 300);
  for (uint32_t s = 0; s < 3; s++) {
    std::vector<Block> bs;
    CHECK(f.sealed(s) && parse_blocks(f.block(s), f.used(s), &bs) && bs[0].h.crc32 != 0 && bs[0].h.seq == s);
  }
  CHECK(!f.sealed(3));
  // more: the oldest blocks go whole
  for (int i = 0; i < 300; i++, t += 20000) f.append(KILO_WALK_RECORD_TX, t, p, 50, p, 50);
  CHECK(f.blocks() == 4 && f.first_seq() > 0 && f.dropped_blocks() == f.first_seq() && !f.has(0) &&
        f.block(0) == nullptr);
  // the time lookup
  uint32_t mid = f.first_seq() + 1;
  CHECK(f.seq_at(f.t0(mid)) == mid && f.seq_at(f.t0(mid) + 1) == mid && f.seq_at(f.t0(mid) - 1) == mid - 1);
  CHECK(f.seq_at(0) == f.first_seq() && f.seq_at(UINT64_MAX) == f.last_seq());
  // a long silence opens a new block (t_us is 32 bits)
  uint32_t before = f.last_seq();
  f.append(KILO_WALK_RECORD_MARK, t + FlightStore::kMaxSpanUs + 1000, "late", 4);
  CHECK(f.last_seq() == before + 1 && f.t0(f.last_seq()) == t + FlightStore::kMaxSpanUs + 1000);
  // heartbeats: counted in the open block
  f.count_skipped(t + FlightStore::kMaxSpanUs + 2000);
  kilo_record_block_header_t h;
  std::memcpy(&h, f.block(f.last_seq()), sizeof h);
  CHECK(h.skipped == 1 && f.skipped() == 1);
  // the boot's number, stamped into the blocks in RAM (the CRC does not cover it)
  f.set_boot(7);
  for (uint32_t s = f.first_seq(); s <= f.last_seq(); s++) {
    std::memcpy(&h, f.block(s), sizeof h);
    CHECK(h.boot == 7);
  }
  std::vector<Block> bs;
  CHECK(parse_blocks(f.block(f.first_seq()), f.used(f.first_seq()), &bs));
  // too long a record, too small a store
  CHECK(!f.append(KILO_WALK_RECORD_MARK, t, p, 200, p, 100));
  FlightStore tiny;
  tiny.attach(mem.data(), record::kBlockBytes);
  CHECK(!tiny.ready() && !tiny.append(KILO_WALK_RECORD_RX, 0, p, 1));
}

// ---- the recorder in the Mind -----------------------------------------------------------

struct Rig {
  FakeBody body;
  CaptureLog log;
  Mind mind;
  std::vector<uint8_t> mem;
  std::vector<LogEntry> history;
  explicit Rig(const MindConfig& cfg, uint32_t store = 4u << 20)
      : mind(body.serial, body.clock, log, nullptr, cfg), mem(store), history(LogRing::kLines) {
    mind.attach_log(history.data(), (uint32_t)history.size());
    mind.attach_recorder(mem.data(), (uint32_t)mem.size());
  }
  void start() {
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
  std::vector<Rec> records() { return ram_records(mind.recorder().flight()); }
  std::string metric(uint32_t i) {
    char line[204];
    mind.metric_line(i, line, sizeof line);
    return line;
  }
};

void test_records_from_boot() {
  MindConfig cfg;
  cfg.remote = true;
  Rig r(cfg);
  r.start();
  r.run_ms(2000);
  std::vector<Rec> rs = r.records();
  size_t tel = count_kind(rs, KILO_WALK_RECORD_RX, KILO_MSG_TELEMETRY);
  CHECKF(tel >= 95 && tel <= 106, "%zu TELEMETRY in 2.1 s idle", tel);
  CHECK(count_kind(rs, KILO_WALK_RECORD_TX, KILO_MSG_HEARTBEAT) == 0 && r.mind.recorder().flight().skipped() > 5);
  CHECK(count_kind(rs, KILO_WALK_RECORD_TX, KILO_MSG_HELLO_REQ) == 1 && count_kind(rs, KILO_WALK_RECORD_RX, KILO_MSG_HELLO) == 1);
  bool logged = false;
  for (const Rec& x : rs) logged = logged || (x.kind == KILO_WALK_RECORD_LOG && x.payload.find("link connected") == 1);
  CHECK(logged && r.mind.recorder().runs_total() == 0);
  // a key no remote action takes: in the stream (EVENT, and the log line)
  r.body.event(KILO_EVENT_IR_KEY, 0x45);
  r.run_ms(20);
  rs = r.records();
  bool key = false;
  for (const Rec& x : rs) key = key || (x.kind == KILO_WALK_RECORD_LOG && x.payload.find("remote ? (0x45): ignored") != std::string::npos);
  CHECK(key && count_kind(rs, KILO_WALK_RECORD_RX, KILO_MSG_EVENT) >= 1);
}

void test_walk_run() {
  MindConfig cfg;
  cfg.remote = true;
  cfg.mind_build = 0x12345678;
  Rig r(cfg);
  r.start();
  r.body.pitch = 3 * 16;
  r.run_ms(2000);
  r.body.event(KILO_EVENT_IR_KEY, 0x68);  // A4: trot
  r.run_ms(40);
  record::Recorder& rec = r.mind.recorder();
  CHECK(r.mind.walker().active() && rec.in_run() && rec.recording());
  r.run_ms(3000);
  r.body.event(KILO_EVENT_IR_KEY, 0x62);  // B7: faster
  r.run_ms(1500);
  r.body.event(KILO_EVENT_IR_KEY, 0x02);  // B6: the planned stop, then stand
  for (int k = 0; k < 400 && r.mind.walker().active(); k++) r.run_ms(20);
  CHECK(!r.mind.walker().active() && !rec.in_run() && rec.recording());  // the post-roll
  r.run_ms((uint32_t)(record::Recorder::kPostRollUs / 1000) + 100);
  CHECK(!rec.recording() && rec.runs_kept() == 1);
  const record::Run* run = rec.find(1);
  CHECK(run != nullptr && run->kind == KILO_RUN_KIND_WALK && (run->flags & KILO_WALK_FLAGS_STREAMED) &&
        (run->flags & KILO_WALK_FLAGS_ENDED_OK) && !(run->flags & KILO_WALK_FLAGS_RECORDING));
  std::string text(run->text, run->text_len);
  CHECKF(text.find("gait=trot dir=forward speed=") == 0 && text.find(" end=stopped") != std::string::npos, "%s", text.c_str());
  uint32_t a, b;
  CHECK(rec.span(*run, &a, &b) && a <= b);
  std::vector<Rec> rs = r.records();
  const Rec* w = mark(rs, "walk");
  CHECK(w != nullptr && w->payload.find("walk id=1 format=2 t0_us_hi=") == 0 &&
        w->payload.find("mind_build=12345678") != std::string::npos);
  const char* words[] = {"setup", "stream", "config", "model", "masses", "change", "timing", "stop", "result", "error", "end"};
  for (const char* x : words) CHECKF(mark(rs, x) != nullptr, "no MARK %s", x);
  CHECK(mark(rs, "end")->payload == "end ok=1 streamed=1 reason=stopped");
  CHECK(count_kind(rs, KILO_WALK_RECORD_RUN) == 2);
  const planner::StreamStats& ss = r.mind.walker().stream_stats();
  CHECK(count_kind(rs, KILO_WALK_RECORD_PLAN) == ss.sent + ss.skipped && run->feet == ss.sent &&
        count_kind(rs, KILO_WALK_RECORD_TX, KILO_MSG_FEET) == ss.sent);
  // the keys that drove it are marked in the run, and the walk's summary is in the log
  CHECK(mark(rs, "key") != nullptr && mark(rs, "key")->payload.find("key B7 (0x62, faster)") == 0);
  CHECK(r.ring_has("walk trot forward") && r.ring_has("walk error: "));
  CHECKF(r.metric(2).find("walks: started=1 ok=1") == 0 && r.metric(2).find("stopped=1") != std::string::npos, "%s",
         r.metric(2).c_str());
  CHECKF(r.metric(1).find("keys: pressed=3 acted=3") == 0 && r.metric(1).find("A4=1") != std::string::npos, "%s",
         r.metric(1).c_str());
}

uint16_t trick_done_arg(const char* name, uint8_t reason) {
  const tricks::Trick* t = tricks::find_trick(name);
  return (uint16_t)(tricks::trick_id_for(*t) | (reason << 8));
}

void test_trick_runs() {
  MindConfig cfg;
  cfg.remote = true;
  cfg.rear_up_key = 1;
  Rig r(cfg);
  r.start();
  r.run_ms(500);
  record::Recorder& rec = r.mind.recorder();
  // 1. A2: rear_up plays and ends DONE
  r.body.event(KILO_EVENT_IR_KEY, 0x10);
  r.run_ms(200);
  CHECK(r.mind.player().playing() && rec.in_run());
  r.body.mode = KILO_MODE_TRICK;
  r.body.pitch = -20 * 16;
  r.run_ms(3000);
  r.body.mode = KILO_MODE_IDLE;
  r.body.pitch = 0;
  r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)trick_done_arg("rear_up", KILO_TRICK_DONE_REASON_DONE));
  r.run_ms(100);
  CHECK(!rec.in_run() && r.mind.player().result().state == tricks::PlayState::Done);
  r.run_ms(2200);
  const record::Run* run = rec.find(1);
  CHECK(run != nullptr && run->kind == KILO_RUN_KIND_TRICK && (run->flags & KILO_WALK_FLAGS_TRICK) &&
        (run->flags & KILO_WALK_FLAGS_STREAMED) && (run->flags & KILO_WALK_FLAGS_ENDED_OK) &&
        !(run->flags & KILO_WALK_FLAGS_RECORDING));
  CHECK(std::string(run->text, run->text_len) == "trick=rear_up stage=1 end=DONE");
  std::vector<Rec> rs = r.records();
  const Rec* t = mark(rs, "trick");
  CHECKF(t != nullptr && t->payload.find("trick id=1 format=2 trick=rear_up stage=1 tid=") == 0 &&
             t->payload.find(" battery_mv=8000 mode=") != std::string::npos && t->payload.find(" by=remote") != std::string::npos,
         "%s", t ? t->payload.c_str() : "none");
  const Rec* fr = mark(rs, "frames");
  const tricks::Trick* trick = tricks::find_trick("rear_up");
  CHECK(fr != nullptr && fr->payload.find("frames dur=" + std::to_string(trick->frames[0].dur) + ",") == 0 &&
        fr->payload.find(" loop=") != std::string::npos);
  const Rec* play = mark(rs, "play");
  const Rec* end = mark(rs, "end");
  CHECK(play != nullptr && play->payload == "play speed=64" && play->t_us == r.mind.player().played_at_us());
  CHECKF(end != nullptr && end->payload.find("end reason=DONE elapsed_ms=") == 0 && end->payload.find(" battery_mv=8000") != std::string::npos,
         "%s", end ? end->payload.c_str() : "none");
  CHECK(count_kind(rs, KILO_WALK_RECORD_TX, KILO_MSG_TRICK_PLAY) == 1);
  const size_t end_i = (size_t)(end - rs.data());
  size_t tel_in = 0;  // the measured pitch at every tick of the play
  for (const Rec& x : rs)
    if (x.kind == KILO_WALK_RECORD_RX && (uint8_t)x.payload[0] == KILO_MSG_TELEMETRY && x.t_us >= play->t_us && x.t_us <= end->t_us)
      tel_in++;
  CHECKF(tel_in >= 150, "%zu TELEMETRY during the play", tel_in);

  // 2. A2 again, then B6: stopped by B6, the key marked in the run
  r.body.event(KILO_EVENT_IR_KEY, 0x10);
  r.run_ms(300);
  CHECK(r.mind.player().playing());
  r.body.mode = KILO_MODE_TRICK;
  r.run_ms(100);
  r.body.event(KILO_EVENT_IR_KEY, 0x02);  // B6
  r.run_ms(100);
  CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1);
  r.body.mode = KILO_MODE_IDLE;
  r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)trick_done_arg("rear_up", KILO_TRICK_DONE_REASON_STOPPED));
  r.run_ms(4000);
  rs = r.records();
  const Rec* t2 = mark(rs, "trick", end_i + 1);
  CHECK(t2 != nullptr && t2->payload.find("trick id=2 ") == 0);
  size_t from = (size_t)(t2 - rs.data());
  const Rec* k2 = mark(rs, "key", from);
  const Rec* e2 = mark(rs, "end", from);
  CHECKF(k2 != nullptr && e2 != nullptr && k2->payload.find("key B6 (0x02, stand)") == 0 && k2->t_us <= e2->t_us &&
             e2->payload.find("end reason=STOPPED") == 0,
         "%s / %s", k2 ? k2->payload.c_str() : "-", e2 ? e2->payload.c_str() : "-");

  // 3. a low battery: A2 refused, a REFUSED run (the battery at rest: 7000 mV for 20 s)
  r.body.batt_mv = 7000;
  r.run_ms(21000);
  r.body.event(KILO_EVENT_IR_KEY, 0x10);
  r.run_ms(100);
  const record::Run* run3 = rec.find(3);
  CHECK(run3 != nullptr && std::string(run3->text, run3->text_len) == "trick=rear_up stage=1 end=REFUSED" &&
        !(run3->flags & KILO_WALK_FLAGS_STREAMED));
  rs = r.records();
  const Rec* e3 = mark(rs, "end", (size_t)(mark(rs, "trick id=3") - rs.data()));
  CHECK(e3 != nullptr && e3->payload.find("end reason=REFUSED") == 0 &&
        e3->payload.find("why=battery 7000 mV at rest (needs 7400)") != std::string::npos);
  std::string tm = r.metric(3), rf = r.metric(4);
  CHECKF(tm.find("tricks: started=3 DONE=1 STOPPED=1 REFUSED=1 | rear_up=3") == 0, "%s", tm.c_str());
  CHECKF(rf.find("refusals: total=1 battery_mV=1") == 0 || rf.find("refusals: total=1 battery") == 0, "%s", rf.c_str());
  CHECK(r.metric(5).find("battery_mv=7000 battery_min_mv=7000") != std::string::npos);
}

// ---- the log of a play session ----------------------------------------------------------

// What the Circle kernel adds around the Mind's own lines (kernel.cpp): the
// once-a-second stats lines (status: link, body, loop), and when the
// Ethernet cable goes in, Circle's and the net task's lines.
void stats_lines(Mind& m) {
  Log& s = m.log().status();
  logf(s, LogLevel::Info, "link ttyS1 (PL011, GPIO14/15): telemetry 50/s (total 4500) rx 4600 frames, tx 900 (450 heartbeats)");
  logf(s, LogLevel::Info, "body: mode IDLE epoch 4 tick 4500 rx_err 0 overruns 0/50");
  logf(s, LogLevel::Info, "loop 890/s: period max 1500 mean 1123 us, work max 300 mean 40 us, mind.poll max 280 mean 25 us; 0 clients");
}

// The rear-up's stages from the remote (KILO_REAR_UP=3: A2 plays rear_up_3, which holds stage 1's and
// stage 2's tops for 4 s each): without A2 in a hold he comes down from it; A2 in the hold goes on;
// each stage a line and a mark in the trick's run. The battery at rest decides the refusal: a sag
// right after a walk does not refuse it.
void test_rear_up_stages() {
  MindConfig cfg;
  cfg.remote = true;
  cfg.rear_up_key = 3;
  const tricks::Trick* t = tricks::find_trick("rear_up_3");
  CHECK(t != nullptr);
  auto frame_of = [&](const char* name) {
    for (uint8_t i = 0; i < t->n_guards; i++)
      if (std::strcmp(t->guards[i].name, name) == 0) return (int)t->guards[i].frame;
    return -1;
  };
  auto ends_at_ms = [&](int f) {  // the frame's hold ends this long after TRICK_PLAY
    uint32_t ticks = 0;
    for (int i = 0; i <= f; i++) ticks += t->frames[i].dur + t->frames[i].hold;
    return ticks * KILO_TICK_MS;
  };
  auto has_mark = [](const std::vector<Rec>& rs, const char* text) {
    for (const Rec& x : rs)
      if (x.kind == KILO_WALK_RECORD_MARK && x.payload.find(text) != std::string::npos) return true;
    return false;
  };
  int up = frame_of("up"), mid = frame_of("mid");
  CHECK(up > 0 && mid > up && t->frames[up].hold == 200 && t->frames[mid].hold == 200);
  for (int presses = 0; presses < 3; presses++) {
    Rig r(cfg);
    r.start();
    r.run_ms(500);
    r.body.batt_mv = 7300;  // the sag right after a walk: at rest it was 8000 a moment ago
    r.run_ms(200);
    r.body.event(KILO_EVENT_IR_KEY, 0x10);  // A2
    r.run_ms(200);
    CHECKF(r.mind.player().playing(), "presses %d: not playing (%s)", presses, r.mind.remote().last());
    r.body.batt_mv = 8000;
    r.body.mode = KILO_MODE_TRICK;
    uint64_t t0 = r.mind.player().played_at_us();
    auto at_ms = [&]() { return (uint32_t)((r.body.clock.t - t0) / 1000); };
    auto run = [&](uint32_t ms) {  // the body pitched as the trick's guards expect at each moment (the model's)
      for (uint32_t k = 0; k < ms; k += 20) {
        uint32_t now = at_ms(), acc = 0;
        for (uint8_t i = 0; i < t->n_guards; i++) {
          int f = t->guards[i].frame;
          acc = 0;
          for (int j = 0; j < f; j++) acc += t->frames[j].dur + t->frames[j].hold;
          if (now >= (acc + t->frames[f].dur / 2) * KILO_TICK_MS)
            r.body.pitch = (int16_t)(-(t->guards[i].lo_a16 + t->guards[i].hi_a16) / 2);
        }
        r.run_ms(20);
      }
    };
    // into stage 1's hold
    while (at_ms() < ends_at_ms(up) - 2000) run(20);
    CHECK(r.ring_has("remote: rear_up: stage 1 (of 3): A2 to go on") && r.body.count(KILO_MSG_TRICK_STOP) == 0);
    if (presses >= 1) {
      r.body.event(KILO_EVENT_IR_KEY, 0x10);
      r.run_ms(100);
      CHECK(r.ring_has("remote: rear_up: on to stage 2 (of 3)"));
    }
    while (at_ms() < ends_at_ms(up) + 200) run(20);
    if (presses == 0) {  // no A2: down from stage 1, before its hold ended
      CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1 && r.ring_has("remote: rear_up: held stage 1 without A2: coming down"));
      r.body.mode = KILO_MODE_IDLE;
      r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)trick_done_arg("rear_up_3", KILO_TRICK_DONE_REASON_STOPPED));
      r.run_ms(4000);
      std::vector<Rec> rs = r.records();
      CHECK(has_mark(rs, "rear_up: up to stage 1 (of 3)") && has_mark(rs, "rear_up: held stage 1 without A2"));
      continue;
    }
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 0);
    while (at_ms() < ends_at_ms(mid) - 2000) run(20);
    CHECK(r.ring_has("remote: rear_up: stage 2 (of 3): A2 to go on"));
    if (presses == 2) {
      r.body.event(KILO_EVENT_IR_KEY, 0x10);
      r.run_ms(100);
      CHECK(r.ring_has("remote: rear_up: on to stage 3 (of 3)"));
    }
    while (at_ms() < ends_at_ms(mid) + 200) run(20);
    if (presses == 1) {
      CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1 && r.ring_has("remote: rear_up: held stage 2 without A2: coming down"));
      continue;
    }
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 0);
    run(3000);
    CHECKF(r.ring_has("remote: rear_up: stage 3 (of 3)"), "guard %s", r.mind.player().result().guard ? r.mind.player().result().guard : "-");
    r.body.event(KILO_EVENT_IR_KEY, 0x02);  // B6 brings him down from any stage
    r.run_ms(200);
    CHECK(r.body.count(KILO_MSG_TRICK_STOP) == 1 && r.ring_has("remote: rear_up: coming down (stage 3 allowed)"));
    r.body.mode = KILO_MODE_IDLE;
    r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)trick_done_arg("rear_up_3", KILO_TRICK_DONE_REASON_STOPPED));
    r.run_ms(4000);
    std::vector<Rec> rs = r.records();
    for (const char* m : {"rear_up: up to stage 1 (of 3)", "rear_up: stage 1 (of 3)", "rear_up: on to stage 2 (of 3)",
                          "rear_up: stage 2 (of 3)", "rear_up: on to stage 3 (of 3)", "rear_up: stage 3 (of 3)",
                          "rear_up: coming down (stage 3 allowed)"})
      CHECKF(has_mark(rs, m), "no mark %s", m);
  }
}

void test_play_session_log() {
  MindConfig cfg;
  cfg.remote = true;
  cfg.rear_up_key = 1;
  cfg.stats_period_ms = 1000;
  Rig r(cfg, 8u << 20);
  r.start();
  LogRing& ring = r.mind.log();
  uint64_t next_stats = r.body.clock.t + 1000000;
  auto run_s = [&](double s) {
    uint64_t until = r.body.clock.t + (uint64_t)(s * 1e6);
    while (r.body.clock.t < until) {
      r.pass();
      if (r.body.clock.t >= next_stats) {
        next_stats += 1000000;
        stats_lines(r.mind);
      }
    }
  };
  auto hold = [&](uint16_t code, int repeats) {  // a key pressed and held: repeats every ~110 ms
    r.body.event(KILO_EVENT_IR_KEY, (int16_t)code);
    run_s(0.11);
    for (int i = 0; i < repeats; i++) {
      r.body.event(KILO_EVENT_IR_KEY, (int16_t)(code | 0x100));
      run_s(0.11);
    }
  };
  // 15 minutes without a laptop: 8 walks (trot, faster, stop), a few
  // postures, a trick, a hand near the sensor, keys held down
  run_s(20);
  for (int w = 0; w < 8; w++) {
    hold(0x68, 3);  // A4: trot
    run_s(15);
    hold(0x62, 2);  // B7: faster
    run_s(20);
    hold(0x02, 5);  // B6: stop and stand
    run_s(25);
    r.body.pitch = (int16_t)(w % 2 ? 40 * 16 : 0);  // picked up, put down: percepts
    run_s(20);
    r.body.pitch = 0;
    run_s(10);
  }
  hold(0x10, 1);  // A2: rear_up
  r.body.mode = KILO_MODE_TRICK;
  run_s(10);
  r.body.mode = KILO_MODE_IDLE;
  r.body.event(KILO_EVENT_TRICK_DONE, (int16_t)trick_done_arg("rear_up", KILO_TRICK_DONE_REASON_DONE));
  run_s(60);
  uint32_t session_lines = ring.next_seq();
  // the cable goes in: Circle's and the net task's lines
  const char* net[] = {"smsc951x: Link is up", "dhcp: Sending DHCPDISCOVER", "dhcp: Received DHCPOFFER",
                       "dhcp: Lease is 3600 s", "Ethernet link up, 912000 ms after the net task started",
                       "listening on 192.168.88.9:5300 (DHCP lease): kilo status --host 192.168.88.9",
                       "mDNS: kilo.local answers with this address",
                       "mDNS: announced kilo._kilo._tcp.local, port 5300, at 192.168.88.9"};
  for (const char* l : net) logf(ring, LogLevel::Info, "%s", l);
  // a laptop watches the log for a minute (the status lines go to it)
  ring.set_watched(true);
  run_s(60);
  ring.set_watched(false);

  // Count what the session logged, and what a 64-line ring (the old size) kept.
  std::map<std::string, int> kinds;
  auto kind_of = [](const char* t) -> std::string {
    const char* pre[] = {"remote ", "walk ", "trick ", "event ", "host: ", "behaviour", "body STATS", "HELLO", "link ",
                         "mDNS", "dhcp", "listening", "Ethernet", "smsc", "flight", "IR remote", "balance"};
    for (const char* p : pre)
      if (std::strncmp(t, p, std::strlen(p)) == 0) return p;
    return "other";
  };
  uint32_t total = ring.next_seq();
  for (uint32_t s = ring.first_seq(); s < total; s++) kinds[kind_of(ring.get(s)->text)]++;
  std::string hist;
  for (auto& kv : kinds) hist += " " + kv.first + "=" + std::to_string(kv.second);
  std::printf("  a 15-minute session: %u history lines (%u before the cable),%s\n", (unsigned)total,
              (unsigned)session_lines, hist.c_str());
  // the platform's log got the routine lines too: per-second stats, held-key repeats, percepts
  size_t status = 0;
  for (const std::string& l : r.log.lines)
    status += l.find("link ttyS1") == 0 || l.find("body: mode") == 0 || l.find("loop ") == 0 ||
              l.find("repeat: ignored") != std::string::npos || l.find("percept ") == 0 || l.find("body STATS") == 0;
  std::printf("  routine (status) lines the session made: %zu; none of them in the history\n", status);
  CHECK(status > 2000);
  // Nothing of the session is lost: every key acted on, every walk's
  // start and end, the trick, all in the history; no status line in it.
  CHECK(ring.first_seq() == 0 && total < LogRing::kLines);
  int remote_acted = 0, walks = 0, walk_ends = 0, trick = 0, status_in = 0;
  for (uint32_t s = 0; s < total; s++) {
    const char* t = ring.get(s)->text;
    remote_acted += std::strncmp(t, "remote ", 7) == 0 && std::strstr(t, "ignored") == nullptr;
    walks += std::strncmp(t, "walk trot forward", 17) == 0;
    walk_ends += std::strncmp(t, "walk ended: stopped", 19) == 0;
    trick += std::strstr(t, "trick rear_up: TRICK_DONE DONE") != nullptr;
    status_in += std::strncmp(t, "link ttyS1", 10) == 0 || std::strncmp(t, "loop ", 5) == 0 ||
                 std::strstr(t, "repeat: ignored") != nullptr || std::strncmp(t, "percept ", 8) == 0;
  }
  CHECKF(remote_acted == 25 && walks >= 8 && walk_ends == 8 && trick == 1 && status_in == 0,
         "remote %d walks %d ends %d trick %d status %d", remote_acted, walks, walk_ends, trick, status_in);
  // the old ring: 64 lines. Of the session, the last 64 lines before the
  // laptop read would have been all that was left: not one walk's start.
  uint32_t old_first = total > 64 ? total - 64 : 0;
  int old_walks = 0;
  for (uint32_t s = old_first; s < total; s++) old_walks += std::strncmp(ring.get(s)->text, "walk trot forward at", 20) == 0;
  std::printf("  a 64-line ring would have kept lines %u..%u: %d of the 8 walks' starts\n", (unsigned)old_first,
              (unsigned)(total - 1), old_walks);
  CHECK(old_walks < 8);
  // the flight recorder kept the status lines as well (all of them)
  size_t rec_status = 0, made = 0;
  for (const Rec& x : r.records())
    rec_status += x.kind == KILO_WALK_RECORD_LOG && (x.payload.find("link ttyS1") == 1 || x.payload.find("loop ") == 1);
  for (const std::string& l : r.log.lines) made += l.find("link ttyS1") == 0 || l.find("loop ") == 0;
  CHECKF(rec_status == made && r.mind.recorder().flight().dropped_blocks() == 0, "%zu status lines in the flight recorder of %zu",
         rec_status, made);
}

// ---- the card saver ---------------------------------------------------------------------

struct CardRig : Rig {
  FakeCard& card;
  record::Saver saver;
  CardRig(const MindConfig& cfg, FakeCard& c, bool on)
      : Rig(cfg), card(c), saver(body.clock, mind.log(), mind.recorder(), mind.log(), c, on) {
    c.set_clock(body.clock);  // the card's calls take the rig's time
    mind.attach_saver(saver);
  }
};

struct Sink : record::BenchSink {
  std::vector<kilo_host_bench_result_t> got;
  void bench_result(const kilo_host_bench_result_t& r) override { got.push_back(r); }
};

std::vector<std::string> names(const FakeCard& c, const char* ext) {
  std::vector<std::string> out;
  for (auto& kv : c.files)
    if (kv.first.size() > 4 && kv.first.compare(kv.first.size() - 4, 4, ext) == 0) out.push_back(kv.first);
  return out;
}

void walk_once(Rig& r) {
  r.body.event(KILO_EVENT_IR_KEY, 0x68);  // A4
  r.run_ms(3000);
  r.body.event(KILO_EVENT_IR_KEY, 0x02);  // B6
  for (int k = 0; k < 400 && r.mind.walker().active(); k++) r.run_ms(20);
}

void test_saver() {
  MindConfig cfg;
  cfg.remote = true;
  std::map<std::string, std::vector<uint8_t>> after_boot1;
  {
    // boot 1: the card is empty; saving on
    FakeCard card;
    CardRig r(cfg, card, true);
    r.start();
    r.run_ms(3000);
    record::Saver& s = r.saver;
    CHECK(s.scanned() && s.boot() == 1 && card.has("kilorec/SAVE_ON"));
    uint32_t before = card.calls;
    // nothing touches the card while walking
    r.body.event(KILO_EVENT_IR_KEY, 0x68);
    r.run_ms(200);
    uint32_t at_walk = card.calls;
    r.run_ms(3000);
    CHECKF(card.calls == at_walk, "%u card calls during the walk", card.calls - at_walk);
    r.body.event(KILO_EVENT_IR_KEY, 0x02);
    for (int k = 0; k < 400 && r.mind.walker().active(); k++) r.run_ms(20);
    uint32_t at_end = card.calls;
    r.run_ms(1900);  // the post-roll
    CHECK(card.calls == at_end);
    r.run_ms(6000);  // then quiet: the save
    CHECKF(card.calls > before && names(card, ".krc").size() >= 1 && card.has("kilorec/b00001.run") &&
               card.has("kilorec/b00001.log") && card.max_write <= record::Saver::kChunk,
           "calls %u, %zu krc, max write %u", card.calls - before, names(card, ".krc").size(), card.max_write);
    // the saved blocks parse, the walk's RUN marks are in them, the CRCs hold
    std::vector<Block> bs;
    size_t runs = 0;
    for (const std::string& n : names(card, ".krc")) {
      CHECK(parse_blocks(card.files[n].data(), card.files[n].size(), &bs));
    }
    for (const Block& b : bs) {
      CHECK(b.h.boot == 1 && b.h.crc32 != 0);
      runs += count_kind(b.recs, KILO_WALK_RECORD_RUN);
    }
    CHECK(runs == 2);
    std::string log(card.files["kilorec/b00001.log"].begin(), card.files["kilorec/b00001.log"].end());
    CHECKF(log.find(" I walk trot forward") != std::string::npos && log.find(" M walks: started=1") != std::string::npos,
           "%s", log.substr(0, 300).c_str());
    kilo_host_rec_status_t st = {};
    s.fill_status(st);
    CHECK(st.card_state == KILO_RECORD_CARD_STATE_IDLE && st.saved_seq > 0 && st.writes > 0 && st.card_files >= 1);

    // a slow write: a warning, the back-off, then on
    card.slow_once_us = 150000;
    walk_once(r);
    r.run_ms(6000);
    CHECK(s.slow_writes() == 1 && s.write_max_us() >= 150000 && r.ring_has("card: ") && r.ring_has("took 150 ms"));
    st = {};
    s.fill_status(st);
    CHECK(st.card_state == KILO_RECORD_CARD_STATE_BACKOFF);
    uint32_t calls = card.calls;
    r.run_ms(20000);
    CHECK(card.calls == calls);  // waiting out the 30 s
    r.run_ms(15000);
    CHECK(card.calls > calls);

    // the bench, the body limp
    r.body.flags = 0;  // no servo power
    r.run_ms(3000);
    Sink sink;
    const char* why = nullptr;
    CHECKF(s.start_bench(4, sink, true, &why), "%s", why ? why : "");
    for (int k = 0; k < 200 && s.bench_running(); k++) r.pass();
    CHECKF(sink.got.size() == 4 && sink.got[0].write_bytes == 512 && sink.got[1].write_bytes == 4096 &&
               sink.got[2].write_bytes == 16384 && sink.got[0].count == 4 && sink.got[0].min_us == card.write_us &&
               sink.got[3].write_bytes == 0 && sink.got[3].close_us > 0 && sink.got[3].remove_us > 0 && !card.has("kilorec/bench.tmp"),
           "%zu results", sink.got.size());
    CHECK(!s.start_bench(4, sink, false, &why) && std::strstr(why, "limp") != nullptr);
    r.body.flags = KILO_TELEMETRY_FLAGS_SERVOS_POWERED;

    // a walk, then the power goes in the middle of its save
    walk_once(r);
    for (int k = 0; k < 5000 && !(card.busy() && card.count_ext(".tmp") == 1); k++) r.pass();
    for (int k = 0; k < 2; k++) r.pass();
    CHECK(card.busy() && card.count_ext(".tmp") == 1);
    card.power_cut();
    after_boot1 = card.files;
  }
  {
    // boot 2: the half file is removed, boot 1's runs are listed, its blocks read back
    FakeCard card_;
    FakeCard* card = &card_;
    CardRig r(cfg, *card, false);
    card->files = after_boot1;
    size_t tmp_before = card->count_ext(".tmp") + card->count_ext(".rtm");
    CHECK(tmp_before == 1);
    r.start();
    r.run_ms(3000);
    record::Saver& s = r.saver;
    CHECKF(s.scanned() && s.boot() == 2 && s.save_on() && card->count_ext(".tmp") + card->count_ext(".rtm") == 0,
           "boot %u on %d", (unsigned)s.boot(), (int)s.save_on());  // SAVE_ON from the card
    CHECK(r.ring_has("a save cut short") && s.card_runs() >= 2);
    const record::CardRun& c0 = s.card_run(0);
    CHECK(c0.boot == 1 && c0.run.id == 1 && (c0.run.flags & KILO_WALK_FLAGS_EARLIER_BOOT) && c0.run.kind == KILO_RUN_KIND_WALK);
    // read back boot 1's first walk's span from the card
    uint64_t from = c0.run.start_us - 2000000, to = c0.run.end_us + 2000000;
    CHECK(s.read_blocks(1, from, to, 0xFFFFFFFFu, 7));
    std::vector<uint8_t> got;
    uint8_t buf[200];
    int32_t n = -2;
    for (int k = 0; k < 20000 && n != 0 && n != -1; k++) {
      r.pass();
      n = s.read(7, buf, sizeof buf);
      if (n > 0) got.insert(got.end(), buf, buf + n);
    }
    std::vector<Block> bs;
    CHECK(n == 0 && parse_blocks(got.data(), got.size(), &bs) && !bs.empty());
    bool begin = false, ended = false;
    for (const Block& b : bs)
      for (const Rec& x : b.recs) {
        if (x.kind == KILO_WALK_RECORD_RUN) {
          kilo_run_mark_t m;
          std::memcpy(&m, x.payload.data(), sizeof m);
          begin = begin || (m.id == 1 && m.event == KILO_RUN_EVENT_BEGIN);
          ended = ended || (m.id == 1 && m.event == KILO_RUN_EVENT_END);
        }
      }
    uint64_t t0 = ((uint64_t)bs.front().h.t0_us_hi << 32) | bs.front().h.t0_us_lo;
    CHECKF(begin && ended && t0 <= from + 1 && bs.back().recs.back().t_us >= c0.run.end_us, "begin %d end %d", begin, ended);
    // boot 1's log
    CHECK(s.read_log(1, 7));
    std::string log;
    n = -2;
    for (int k = 0; k < 2000 && n != 0 && n != -1; k++) {
      r.pass();
      n = s.read(7, buf, sizeof buf);
      if (n > 0) log.append((const char*)buf, (size_t)n);
    }
    CHECK(log.find("walk trot forward") != std::string::npos);
    // the cap: a small one, the oldest files go
    s.set_cap(40000);
    walk_once(r);
    r.run_ms(10000);
    CHECKF(s.card_bytes() <= 40000 + 70000 && s.files() >= 1, "%u B in %u files", (unsigned)s.card_bytes(), (unsigned)s.files());
    // delete every earlier boot's
    uint32_t files = 0;
    CHECK(s.request_delete(0xFFFFFFFFu, &files));
    r.run_ms(3000);
    bool any1 = false;
    for (auto& kv : card->files) any1 = any1 || kv.first.find("kilorec/b00001") == 0;
    CHECK(!any1 && s.card_runs() == 0 && !s.request_delete(2, &files));  // never this boot's
  }
}

void test_saver_failures() {
  MindConfig cfg;
  cfg.remote = true;
  FakeCard card_;
  FakeCard* card = &card_;
  CardRig r(cfg, *card, true);
  r.start();
  r.run_ms(3000);
  card->fail_writes = 100;  // the card refuses every write
  for (int i = 0; i < 6; i++) {
    walk_once(r);
    r.run_ms(4500);
    r.run_ms(700000);  // past the back-off (it doubles)
  }
  kilo_host_rec_status_t st = {};
  r.saver.fill_status(st);
  CHECKF(st.card_state == KILO_RECORD_CARD_STATE_FAILED && st.card_errors >= 5 && r.ring_has("saving stopped"),
         "state %u errors %u", st.card_state, (unsigned)st.card_errors);
  // no card at all: nothing happens, the recorder runs on
  FakeCard none_;
  FakeCard* none = &none_;
  none->present = false;
  CardRig n(cfg, *none, true);
  n.start();
  n.run_ms(3000);
  CHECK(!n.saver.scanned() && none->calls == 0 && n.mind.recorder().flight().records() > 100);
}

}  // namespace

// The body's first HELLO lost (on the Pi the SD card's mount stalled the loop 1.5 s right after
// the first HELLO_REQ, and the PL011 dropped the answer): the mind asks again once a second, so
// the HELLO comes and with it the body build and the eyes (caps EYES). The card is not touched
// before the HELLO, so its mount no longer falls on the first exchange.
void test_hello_lost_at_boot() {
  MindConfig cfg;
  cfg.remote = true;
  FakeCard card;
  CardRig r(cfg, card, true);
  r.body.eyes = true;
  r.body.drop_hellos = 1;
  r.start();
  r.run_ms(800);
  CHECK(!r.mind.link().state().has_hello && card.ready_calls == 0 && !r.mind.guard().eyes_driven());
  r.run_ms(500);  // 1.4 s: asked again at 1 s
  CHECK(r.mind.link().state().has_hello && r.mind.guard().eyes_driven());
  CHECK(r.mind.link().stats().hello_polls == 2 && r.body.count(KILO_MSG_SET_EYES) >= 1);
  r.run_ms(4000);  // then the card (2 s of quiet after the HELLO)
  CHECK(card.ready_calls > 0 && r.saver.scanned());
  CHECK(r.mind.link().stats().hello_polls == 2);  // and no more HELLO_REQ
  {  // no body answering at all: the card waits 3 s, not forever; HELLO_REQ at 1 Hz at most
    FakeCard card2;
    CardRig r2(cfg, card2, true);
    r2.body.drop_hellos = 1000;
    r2.start();
    r2.run_ms(2800);
    CHECK(card2.ready_calls == 0);
    r2.run_ms(3000);
    CHECK(card2.ready_calls > 0);
    CHECKF(r2.mind.link().stats().hello_polls <= 7, "%u HELLO_REQ in 5.9 s", (unsigned)r2.mind.link().stats().hello_polls);
  }
}

int main() {
  test_flight_store();
  test_records_from_boot();
  test_walk_run();
  test_trick_runs();
  test_rear_up_stages();
  test_play_session_log();
  test_saver();
  test_saver_failures();
  test_hello_lost_at_boot();
  std::printf("  sizeof(Recorder) %zu B, sizeof(Saver) %zu B, sizeof(Mind) %zu B, LogEntry %zu B\n",
              sizeof(record::Recorder), sizeof(record::Saver), sizeof(Mind), sizeof(LogEntry));
  return report("record");
}
