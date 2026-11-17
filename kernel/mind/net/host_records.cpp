// The flight recorder over the host channel (net/host_server.h): runs,
// ranges, the card, the metrics, the bench.
#include "net/host_server.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {

namespace {

constexpr uint32_t kEarlierBit = 20;  // an earlier boot's run id: boot << 20 | its id
constexpr uint32_t kRunIdMask = (1u << kEarlierBit) - 1;

void fill_run_info(kilo_host_walk_info_t& w, const record::Run& r, uint32_t id, uint32_t start_ms, uint64_t now) {
  w.found = 1;
  w.id = id;
  w.start_ms = start_ms;
  uint64_t end = r.end_us != 0 ? r.end_us : now;
  w.duration_ms = end > r.start_us ? (uint32_t)((end - r.start_us) / 1000u) : 0;
  w.feet = r.feet;
  w.flags = r.flags;
  uint32_t n = 0;
  while (n < KILO_HOST_WALK_TEXT_MAX && r.text[n] != '\0' && n < r.text_len) n++;
  mem_copy(w.text, r.text, n);
}

}  // namespace

bool HostServer::body_limp() const {
  const BodyState& b = link_.state();
  if (!b.has_telemetry || !link_.connected()) return true;  // no body: nothing can move
  return b.telemetry.mode == KILO_MODE_LIMP || !(b.telemetry.flags & KILO_TELEMETRY_FLAGS_SERVOS_POWERED);
}

void HostServer::walk_list(uint32_t i, uint8_t seq, const uint8_t* p, uint64_t now) {
  if (rec_ == nullptr)
    return result(i, seq, KILO_HOST_MSG_WALK_LIST, KILO_HOST_RESULT_CODE_UNSUPPORTED, "no recorder on this mind");
  uint32_t after;
  mem_copy(&after, p, sizeof after);
  kilo_host_walk_info_t w;
  mem_zero(&w, sizeof w);
  const record::FlightStore& f = rec_->flight();
  uint32_t card = saver_ != nullptr ? saver_->card_runs() : 0;
  w.kept = (uint16_t)(rec_->runs_kept() + card);
  w.capacity = f.capacity();
  w.used = f.used_bytes();
  w.dropped_walks = f.dropped_blocks();
  uint32_t len = KILO_HOST_WALK_INFO_MIN_LEN;
  // the open run, if any
  for (const record::Run* r = rec_->run_after(0); r != nullptr; r = rec_->run_after(r->id))
    if (r->flags & KILO_WALK_FLAGS_RECORDING) w.recording = r->id;
  const record::Run* r = after < (1u << kEarlierBit) ? rec_->run_after(after) : nullptr;
  if (r != nullptr) {
    fill_run_info(w, *r, r->id, r->start_us > start_us_ ? (uint32_t)((r->start_us - start_us_) / 1000u) : 0, now);
    uint32_t a, b;
    if (rec_->span(*r, &a, &b)) {
      uint32_t bytes = 0;
      for (uint32_t s = a; s <= b; s++) bytes += f.used(s);
      w.bytes = bytes;
      if (saver_ != nullptr && b < saver_->saved_seq()) w.flags |= KILO_WALK_FLAGS_SAVED;
    } else if (!(r->flags & KILO_WALK_FLAGS_RECORDING)) {
      bool saved = saver_ != nullptr && saver_->has_card_blocks(saver_->boot());
      w.flags |= saved ? KILO_WALK_FLAGS_SAVED : KILO_WALK_FLAGS_TRUNCATED;
    }
  } else if (saver_ != nullptr) {
    // the card's earlier boots, by (boot, id): their WALK_INFO ids grow with both
    const record::CardRun* best = nullptr;
    for (uint32_t k = 0; k < card; k++) {
      const record::CardRun& c = saver_->card_run(k);
      uint32_t id = (c.boot << kEarlierBit) | (c.run.id & kRunIdMask);
      if (id > after && (best == nullptr || id < ((best->boot << kEarlierBit) | (best->run.id & kRunIdMask)))) best = &c;
    }
    if (best != nullptr) {
      uint32_t id = (best->boot << kEarlierBit) | (best->run.id & kRunIdMask);
      fill_run_info(w, best->run, id, (uint32_t)(best->run.start_us / 1000u), best->run.end_us);
    }
  }
  if (w.found) {
    uint32_t n = 0;
    while (n < KILO_HOST_WALK_TEXT_MAX && w.text[n] != '\0') n++;
    len += n;
  }
  reply(i, seq, &w, len, KILO_HOST_MSG_WALK_INFO);
}

// RAM: the blocks of [from_us, to_us] from min_seq on.
bool HostServer::plan_ram(Client& c, uint64_t from_us, uint64_t to_us, uint32_t min_seq) {
  const record::FlightStore& f = rec_->flight();
  if (f.empty()) return false;
  uint32_t a = f.seq_at(from_us), b = f.seq_at(to_us);
  if (a < min_seq) a = min_seq;
  if (b < a || !f.has(a)) return false;
  c.fetch_ram = true;
  c.fetch_seq = a;
  c.fetch_last = b;
  c.fetch_boff = 0;
  c.fetch_last_used = f.used(b);
  return true;
}

bool HostServer::plan_fetch(uint32_t i, uint32_t id, uint32_t boot, uint64_t from_us, uint64_t to_us) {
  Client& c = clients_[i];
  stop_fetch(i);
  c.fetch_id = id;
  c.fetch_off = 0;
  c.fetch_total = 0;
  c.fetch_card = c.fetch_ram = false;
  c.fetch_from_us = from_us;
  c.fetch_to_us = to_us;
  const record::FlightStore& f = rec_->flight();
  bool this_boot = boot == 0 || (saver_ != nullptr && boot == saver_->boot());
  if (!this_boot) {
    if (saver_ == nullptr || !saver_->read_blocks(boot, from_us, to_us, 0xFFFFFFFFu, i)) return false;
    c.fetch_card = true;
  } else {
    // what RAM no longer has, from the card (if saved), then RAM
    bool older = f.empty() || from_us < f.t0(f.first_seq());
    if (older && saver_ != nullptr && f.first_seq() > 0 &&
        saver_->read_blocks(saver_->boot(), from_us, to_us, f.empty() ? 0xFFFFFFFFu : f.first_seq(), i))
      c.fetch_card = true;
    if (!c.fetch_card) {
      if (!plan_ram(c, from_us, to_us, 0)) return false;
      for (uint32_t s = c.fetch_seq; s < c.fetch_last; s++) c.fetch_total += f.used(s);
      c.fetch_total += c.fetch_last_used;
    } else {
      c.fetch_ram = true;  // decided when the card part ends
    }
  }
  c.fetch_on = true;
  return true;
}

void HostServer::stop_fetch(uint32_t i) {
  Client& c = clients_[i];
  if (c.fetch_card && saver_ != nullptr) saver_->read_cancel(i);
  c.fetch_on = c.fetch_card = c.fetch_ram = false;
}

void HostServer::walk_fetch(uint32_t i, uint8_t seq, const uint8_t* p) {
  const uint8_t type = KILO_HOST_MSG_WALK_FETCH;
  if (rec_ == nullptr) return result(i, seq, type, KILO_HOST_RESULT_CODE_UNSUPPORTED, "no recorder on this mind");
  kilo_host_walk_fetch_t q;
  mem_copy(&q, p, sizeof q);
  if (q.id == 0) {
    stop_fetch(i);
    return result(i, seq, type, KILO_HOST_RESULT_CODE_OK, "stopped");
  }
  char why[60];
  const uint64_t pre = record::Recorder::kPreRollUs, post = record::Recorder::kPostRollUs;
  if (q.id >> kEarlierBit) {  // an earlier boot's, from the card
    uint32_t boot = q.id >> kEarlierBit, rid = q.id & kRunIdMask;
    const record::CardRun* run = nullptr;
    for (uint32_t k = 0; saver_ != nullptr && k < saver_->card_runs(); k++)
      if (saver_->card_run(k).boot == boot && (saver_->card_run(k).run.id & kRunIdMask) == rid) run = &saver_->card_run(k);
    if (run == nullptr) {
      format(why, sizeof why, "no run %u of boot %u on the card", (unsigned)rid, (unsigned)boot);
      return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, why);
    }
    uint64_t from = run->run.start_us > pre ? run->run.start_us - pre : 0;
    if (!plan_fetch(i, q.id, boot, from, run->run.end_us + post))
      return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "its blocks are not on the card any more");
    return result(i, seq, type, KILO_HOST_RESULT_CODE_OK, "from the card");
  }
  const record::Run* r = rec_->find(q.id);
  if (r == nullptr) {
    format(why, sizeof why, "no run %u", (unsigned)q.id);
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, why);
  }
  if (r->flags & KILO_WALK_FLAGS_RECORDING) {
    format(why, sizeof why, "run %u is still being recorded", (unsigned)q.id);
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, why);
  }
  uint64_t from = r->start_us > pre ? r->start_us - pre : 0;
  if (!plan_fetch(i, q.id, 0, from, r->end_us + post)) {
    format(why, sizeof why, "run %u is no longer in RAM nor on the card", (unsigned)q.id);
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, why);
  }
  if (q.offset != 0) clients_[i].fetch_off = 0;  // resumed fetches start over (blocks, not offsets)
  result(i, seq, type, KILO_HOST_RESULT_CODE_OK);
}

void HostServer::rec_fetch(uint32_t i, uint8_t seq, const uint8_t* p, uint64_t now) {
  const uint8_t type = KILO_HOST_MSG_REC_FETCH;
  if (rec_ == nullptr) return result(i, seq, type, KILO_HOST_RESULT_CODE_UNSUPPORTED, "no recorder on this mind");
  kilo_host_rec_fetch_t q;
  mem_copy(&q, p, sizeof q);
  if (q.what > KILO_REC_FETCH_WHAT_LOG) return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "what: 0 or 1");
  if (q.what == KILO_REC_FETCH_WHAT_LOG) {
    stop_fetch(i);
    uint32_t boot = q.boot != 0 ? q.boot : (saver_ != nullptr ? saver_->boot() : 0);
    if (saver_ == nullptr || !saver_->read_log(boot, i))
      return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "no saved log for that boot on the card");
    Client& c = clients_[i];
    c.fetch_id = 0;
    c.fetch_off = c.fetch_total = 0;
    c.fetch_card = true;
    c.fetch_ram = false;
    c.fetch_on = true;
    return result(i, seq, type, KILO_HOST_RESULT_CODE_OK, "the saved log");
  }
  // from_ms, to_ms: the clock's ms modulo 2^32 (REC_STATUS.clock_ms): the most recent such time, for this boot
  auto unwrap = [&](uint32_t ms) -> uint64_t {
    uint64_t now_ms = now / 1000u;
    uint64_t t = (now_ms & ~0xFFFFFFFFull) | ms;
    if (t > now_ms && t >= (1ull << 32)) t -= 1ull << 32;
    return t * 1000u;
  };
  bool this_boot = q.boot == 0 || (saver_ != nullptr && q.boot == saver_->boot());
  uint64_t from = this_boot ? unwrap(q.from_ms) : (uint64_t)q.from_ms * 1000u;
  uint64_t to = q.to_ms == 0xFFFFFFFFu ? (this_boot ? now : ~0ull) : this_boot ? unwrap(q.to_ms) : (uint64_t)q.to_ms * 1000u;
  if (to < from) return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "to_ms before from_ms");
  if (!plan_fetch(i, 0, q.boot, from, to))
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "nothing recorded in that range");
  char what[60];
  Client& c = clients_[i];
  if (c.fetch_card) format(what, sizeof what, "from the card%s", c.fetch_ram ? ", then RAM" : "");
  else format(what, sizeof what, "blocks %u..%u, %u bytes", (unsigned)c.fetch_seq, (unsigned)c.fetch_last,
              (unsigned)c.fetch_total);
  result(i, seq, type, KILO_HOST_RESULT_CODE_OK, what);
}

// The fetching client's next chunks, as far as its send ring has room (a
// slow laptop waits, nothing is dropped), then END or GONE.
void HostServer::pump_fetch(uint32_t i) {
  Client& c = clients_[i];
  const record::FlightStore& f = rec_->flight();
  uint32_t chunks = rec_->recording() ? kFetchChunksRecording : kFetchChunks;
  for (uint32_t k = 0; k < chunks && c.fetch_on; k++) {
    if (room(i) < KILO_HOST_WALK_DATA_MAX_LEN + kHostOverhead) return;
    kilo_host_walk_data_t d;
    d.id = c.fetch_id;
    d.offset = c.fetch_off;
    d.total = c.fetch_total;
    int32_t n = 0;
    bool gone = false;
    if (c.fetch_card) {
      n = saver_->read(i, d.data, sizeof d.data);
      if (n == -2) return;  // not this pass
      if (n == 0 && c.fetch_ram) {  // the card part is over: RAM from the next block on
        c.fetch_card = false;
        uint32_t last = saver_->read_last_seq();
        uint32_t min_seq = last == 0xFFFFFFFFu ? 0 : last + 1;
        if (!plan_ram(c, c.fetch_from_us, c.fetch_to_us, min_seq)) c.fetch_ram = false;
        k--;
        continue;
      }
      if (n < 0) gone = true;
    } else if (c.fetch_ram) {
      if (c.fetch_seq > c.fetch_last) {
        n = 0;
      } else if (!f.has(c.fetch_seq)) {
        gone = true;
      } else {
        uint32_t used = c.fetch_seq == c.fetch_last ? c.fetch_last_used : f.used(c.fetch_seq);
        uint32_t m = used - c.fetch_boff < sizeof d.data ? used - c.fetch_boff : (uint32_t)sizeof d.data;
        mem_copy(d.data, f.block(c.fetch_seq) + c.fetch_boff, m);
        if (c.fetch_boff == 0 && c.fetch_seq == c.fetch_last && m >= 16) mem_copy(d.data + 12, &used, 4);  // its used as sent
        c.fetch_boff += m;
        if (c.fetch_boff >= used) {
          c.fetch_seq++;
          c.fetch_boff = 0;
        }
        n = (int32_t)m;
      }
    }
    d.status = gone ? KILO_WALK_DATA_STATUS_GONE : n == 0 ? KILO_WALK_DATA_STATUS_END : KILO_WALK_DATA_STATUS_DATA;
    if (d.status != KILO_WALK_DATA_STATUS_DATA) n = 0;
    uint32_t len = KILO_HOST_WALK_DATA_MIN_LEN + (uint32_t)n;
    if (!enqueue(i, KILO_HOST_MSG_WALK_DATA, c.out_seq, &d, len)) return;
    c.out_seq++;
    if (d.status != KILO_WALK_DATA_STATUS_DATA) {
      c.fetch_on = c.fetch_card = c.fetch_ram = false;
      if (!gone) stats_.walk_fetches++;
      else logf(log_, LogLevel::Warn, "host: client %d: a download lost its blocks (overwritten, or the card failed)", (int)i);
      return;
    }
    c.fetch_off += (uint32_t)n;
    stats_.walk_bytes_sent += (uint32_t)n;
  }
}

void HostServer::rec_status(uint32_t i, uint8_t seq, uint64_t now) {
  if (rec_ == nullptr)
    return result(i, seq, KILO_HOST_MSG_REC_STATUS_REQ, KILO_HOST_RESULT_CODE_UNSUPPORTED, "no recorder on this mind");
  const record::FlightStore& f = rec_->flight();
  kilo_host_rec_status_t s;
  mem_zero(&s, sizeof s);
  s.clock_ms = (uint32_t)(now / 1000u);
  s.capacity = f.capacity();
  s.used = f.used_bytes();
  s.block_bytes = record::kBlockBytes;
  s.blocks = f.blocks();
  s.first_seq = f.empty() ? f.next_seq() : f.first_seq();
  s.next_seq = f.empty() ? f.next_seq() : f.last_seq();
  s.oldest_ms = f.empty() ? 0 : (uint32_t)(f.t0(f.first_seq()) / 1000u);
  s.dropped_blocks = f.dropped_blocks();
  s.records = f.records();
  s.skipped = f.skipped();
  s.runs = rec_->runs_total();
  if (saver_ != nullptr) {
    s.boot = saver_->boot();
    saver_->fill_status(s);
  } else {
    s.unsaved_bytes = f.used_bytes();
  }
  reply(i, seq, &s, sizeof s, KILO_HOST_MSG_REC_STATUS);
}

void HostServer::metrics(uint32_t i, uint8_t seq, const uint8_t* p) {
  kilo_host_metrics_t m;
  mem_zero(&m, sizeof m);
  m.index = p[0];
  uint32_t n = metrics_ != nullptr ? metrics_->metric_lines() : 0;
  m.count = (uint8_t)n;
  uint32_t len = KILO_HOST_METRICS_MIN_LEN;
  if (m.index < n) {
    char line[204];
    metrics_->metric_line(m.index, line, sizeof line);
    uint32_t k = 0;
    while (line[k] != '\0' && k < sizeof m.text) k++;
    mem_copy(m.text, line, k);
    len += k;
  }
  reply(i, seq, &m, len, KILO_HOST_MSG_METRICS);
}

void HostServer::rec_bench(uint32_t i, uint8_t seq, const uint8_t* p) {
  const uint8_t type = KILO_HOST_MSG_REC_BENCH;
  if (saver_ == nullptr) return result(i, seq, type, KILO_HOST_RESULT_CODE_UNSUPPORTED, "no SD card on this mind");
  if (controller_ != (int32_t)i) return result(i, seq, type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER, "take control first");
  if (link_paused() || flashing() || updating())
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BUSY, "a flash, an update or the bridge has the mind");
  kilo_host_rec_bench_t q;
  mem_copy(&q, p, sizeof q);
  const char* why = nullptr;
  if (!saver_->start_bench(q.count, *this, body_limp(), &why))
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BUSY, why);
  bench_client_ = (int32_t)i;
  logf(log_, LogLevel::Info, "host: client %d: SD card bench, %u writes a size", (int)i, (unsigned)q.count);
  result(i, seq, type, KILO_HOST_RESULT_CODE_OK);
}

void HostServer::bench_result(const kilo_host_bench_result_t& r) {
  if (bench_client_ < 0) return;
  uint32_t i = (uint32_t)bench_client_;
  if (enqueue(i, KILO_HOST_MSG_BENCH_RESULT, clients_[i].out_seq, &r, sizeof r)) clients_[i].out_seq++;
  if (r.write_bytes == 0) bench_client_ = -1;
}

void HostServer::rec_save(uint32_t i, uint8_t seq, const uint8_t* p) {
  const uint8_t type = KILO_HOST_MSG_REC_SAVE;
  if (saver_ == nullptr) return result(i, seq, type, KILO_HOST_RESULT_CODE_UNSUPPORTED, "no SD card on this mind");
  if (controller_ != (int32_t)i) return result(i, seq, type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER, "take control first");
  if (p[0] > 2) return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "mode: 0 off, 1 on, 2 now");
  saver_->set_mode(p[0]);
  static const char* const kWhat[] = {"saving off", "saving on", "saving at the next safe moment"};
  logf(log_, LogLevel::Info, "host: client %d: %s", (int)i, kWhat[p[0]]);
  result(i, seq, type, KILO_HOST_RESULT_CODE_OK, kWhat[p[0]]);
}

void HostServer::rec_delete(uint32_t i, uint8_t seq, const uint8_t* p) {
  const uint8_t type = KILO_HOST_MSG_REC_DELETE;
  if (saver_ == nullptr) return result(i, seq, type, KILO_HOST_RESULT_CODE_UNSUPPORTED, "no SD card on this mind");
  if (controller_ != (int32_t)i) return result(i, seq, type, KILO_HOST_RESULT_CODE_NOT_CONTROLLER, "take control first");
  kilo_host_rec_delete_t q;
  mem_copy(&q, p, sizeof q);
  uint32_t files = 0;
  if (!saver_->request_delete(q.boot, &files))
    return result(i, seq, type, KILO_HOST_RESULT_CODE_BAD_ARG, "not this boot's (or the card is not read yet)");
  char what[60];
  format(what, sizeof what, "%u block files and their indexes go", (unsigned)files);
  logf(log_, LogLevel::Info, "host: client %d: deleting recordings on the card (%s)", (int)i, what);
  result(i, seq, type, KILO_HOST_RESULT_CODE_OK, what);
}

}  // namespace kilo
