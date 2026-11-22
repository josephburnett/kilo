#include "record/saver.h"
#include "util/fmt.h"
#include "util/log.h"
#include "util/mem.h"
#include "util/str.h"

namespace kilo {
namespace record {

namespace {

constexpr char kDir[] = "kilorec";
constexpr char kFlag[] = "kilorec/SAVE_ON";
constexpr char kBench[] = "kilorec/bench.tmp";
constexpr uint32_t kRunsMagic = 0x4E55524Bu;  // "KRUN"
constexpr uint32_t kRunsVersion = 1;
constexpr uint32_t kBenchSize[Saver::kBenchSizes] = {512, 4096, 16384};

struct RunsHeader {
  uint32_t magic, version, boot, count;
};

// "123" at s: the number and where it ended; false (end untouched) if no digit or over 32 bits.
bool number(const char* s, uint32_t* v, const char** end) {
  const char* p = parse_dec_u32(s, s + str_len(s), v);
  if (p == nullptr) return false;
  *end = p;
  return true;
}

}  // namespace

Saver::Saver(Clock& clock, Log& log, Recorder& recorder, LogRing& ring, RecordCard& card, bool save_on)
    : clock_(clock), log_(log), rec_(recorder), ring_(ring), card_(card), save_on_(save_on) {}

void Saver::file_name(char* out, uint32_t n, uint32_t boot, uint32_t seq, const char* ext) const {
  format(out, n, "%s/b%05u-s%07u.%s", kDir, (unsigned)boot, (unsigned)seq, ext);
}

void Saver::boot_name(char* out, uint32_t n, uint32_t boot, const char* ext) const {
  format(out, n, "%s/b%05u.%s", kDir, (unsigned)boot, ext);
}

uint8_t Saver::state() const {
  if (phase_ == Phase::NoCard || phase_ == Phase::Scan) return KILO_RECORD_CARD_STATE_NONE;
  if (phase_ == Phase::Failed) return KILO_RECORD_CARD_STATE_FAILED;
  if (clock_.now_us() < backoff_until_) return KILO_RECORD_CARD_STATE_BACKOFF;
  if (!save_on_) return KILO_RECORD_CARD_STATE_OFF;
  return job_ != Job::None ? KILO_RECORD_CARD_STATE_SAVING : KILO_RECORD_CARD_STATE_IDLE;
}

void Saver::fill_status(kilo_host_rec_status_t& s) const {
  const FlightStore& f = rec_.flight();
  s.card_state = state();
  s.save_on = save_on_ ? 1 : 0;
  s.saved_seq = saved_seq_;
  uint32_t unsaved = 0;
  for (uint32_t q = saved_seq_ > f.first_seq() ? saved_seq_ : f.first_seq(); !f.empty() && q < f.next_seq(); q++)
    unsaved += f.used(q);
  s.unsaved_bytes = unsaved;
  s.card_bytes = card_bytes_ > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)card_bytes_;
  s.card_files = (uint16_t)n_files_;
  uint32_t boots = 0, last = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n_files_; i++)
    if (files_[i].boot != boot_ && files_[i].boot != last) {
      boots++;
      last = files_[i].boot;
    }
  s.card_boots = (uint16_t)boots;
  s.writes = writes_;
  s.write_max_us = write_max_us_;
  s.slow_writes = slow_;
  s.card_errors = errors_;
}

bool Saver::has_card_blocks(uint32_t boot) const {
  for (uint32_t i = 0; i < n_files_; i++)
    if (files_[i].boot == boot) return true;
  return false;
}

// ---- files ------------------------------------------------------------------------------

void Saver::add_file(uint32_t boot, uint32_t seq, uint32_t size) {
  if (n_files_ == kMaxFiles) return;  // the cap removes the oldest first (Cap)
  uint32_t i = n_files_;
  while (i > 0 && (files_[i - 1].boot > boot || (files_[i - 1].boot == boot && files_[i - 1].seq > seq))) {
    files_[i] = files_[i - 1];
    i--;
  }
  files_[i] = {boot, seq, size};
  n_files_++;
  card_bytes_ += size;
}

void Saver::drop_file(uint32_t i) {
  card_bytes_ -= files_[i].size;
  for (uint32_t k = i; k + 1 < n_files_; k++) files_[k] = files_[k + 1];
  n_files_--;
}

int32_t Saver::oldest_file() const { return n_files_ > 0 ? 0 : -1; }

// ---- timing -----------------------------------------------------------------------------

void Saver::timed(uint64_t t0, const char* what) {
  uint64_t now = clock_.now_us();
  uint32_t dt = (uint32_t)(now - t0);
  if (dt > write_max_us_) write_max_us_ = dt;
  if (dt > kSlowUs) {
    slow_++;
    backoff_us_ = backoff_us_ == 0 ? kBackoffUs : (backoff_us_ * 2 > kBackoffMaxUs ? kBackoffMaxUs : backoff_us_ * 2);
    backoff_until_ = now + backoff_us_;
    logf(log_, LogLevel::Warn, "card: %s took %u ms (over %u): saving waits %u s", what, (unsigned)(dt / 1000u),
         (unsigned)(kSlowUs / 1000u), (unsigned)(backoff_us_ / 1000000u));
  }
}

void Saver::failure(const char* what) {
  errors_++;
  failures_++;
  if (file_open_) {
    card_.close();
    file_open_ = false;
  }
  job_ = Job::None;
  bench_.on = false;
  backoff_us_ = backoff_us_ == 0 ? kBackoffUs : (backoff_us_ * 2 > kBackoffMaxUs ? kBackoffMaxUs : backoff_us_ * 2);
  backoff_until_ = clock_.now_us() + backoff_us_;
  if (failures_ >= kMaxFailures) {
    phase_ = Phase::Failed;
    logf(log_, LogLevel::Error, "card: %s failed %u times in a row: saving stopped (kilo record save on retries)",
         what, (unsigned)failures_);
  } else {
    logf(log_, LogLevel::Warn, "card: %s failed: trying again in %u s", what, (unsigned)(backoff_us_ / 1000000u));
  }
}

bool Saver::timed_ok(RecordCard::Result r, uint64_t t0, const char* what) {
  timed(t0, what);
  if (r == RecordCard::Result::Failed) {
    failure(what);
    return false;
  }
  return true;  // failures_ counts on until a whole save is done
}

// ---- the pass ---------------------------------------------------------------------------

void Saver::poll(bool safe, bool limp) {
  safe_ = safe;
  if (!safe) {
    if (bench_.on) step_bench(false);  // the body moved: the bench stops
    return;
  }
  if (phase_ == Phase::NoCard) {
    if (!card_.ready()) return;
    phase_ = Phase::Scan;
    job_ = Job::ScanDir;
  }
  if (phase_ == Phase::Scan) return step_scan();
  if (phase_ == Phase::Failed) return;
  if (bench_.on) return step_bench(limp);
  if (rd_.on) return;  // the reader has the card
  if (clock_.now_us() < backoff_until_) return;
  if (job_ != Job::None) {
    if (job_ == Job::Delete) return step_delete();
    if (job_ == Job::Flag) {
      uint64_t t0 = clock_.now_us();
      bool on = save_on_;
      RecordCard::Result r = RecordCard::Result::Ok;
      if (on && !flag_file_) {
        r = card_.create(kFlag);
        if (r == RecordCard::Result::Ok) r = card_.close();
      } else if (!on && flag_file_) {
        r = card_.remove(kFlag);
        if (r == RecordCard::Result::Missing) r = RecordCard::Result::Ok;
      }
      job_ = Job::None;
      if (timed_ok(r, t0, "SAVE_ON")) {
        flag_file_ = on;
        flag_dirty_ = false;
      }
      return;
    }
    return step_save();
  }
  if (flag_dirty_) {
    job_ = Job::Flag;
    return;
  }
  if (delete_on_) {
    job_ = Job::Delete;
    return;
  }
  if (save_on_ && want_save(clock_.now_us())) begin_save();
}

void Saver::set_mode(uint8_t mode) {
  if (mode == 2) {
    save_now_ = true;
    return;
  }
  bool on = mode == 1;
  if (on && phase_ == Phase::Failed) {
    phase_ = Phase::Idle;
    failures_ = 0;
    backoff_us_ = 0;
    backoff_until_ = 0;
  }
  if (on != save_on_ || on != flag_file_) flag_dirty_ = true;
  save_on_ = on;
}

bool Saver::request_delete(uint32_t boot, uint32_t* files) {
  uint32_t n = 0;
  for (uint32_t i = 0; i < n_files_; i++)
    if (files_[i].boot != boot_ && (boot == 0xFFFFFFFFu || files_[i].boot == boot)) n++;
  *files = n;  // the boots' .run and .log go with them
  if (boot == boot_ || !scanned()) return false;
  delete_boot_ = boot;
  delete_on_ = true;
  return true;
}

// ---- the scan ---------------------------------------------------------------------------

void Saver::step_scan() {
  uint64_t t0 = clock_.now_us();
  switch (job_) {
    case Job::ScanDir: {
      RecordCard::Result r = card_.make_dir(kDir);
      if (!timed_ok(r, t0, "making kilorec/")) {
        phase_ = Phase::Failed;
        return;
      }
      r = card_.list_begin(kDir);
      if (r != RecordCard::Result::Ok) {
        failure("listing kilorec/");
        phase_ = Phase::Failed;
        return;
      }
      job_ = Job::ScanList;
      return;
    }
    case Job::ScanList: {
      char name[48];
      uint32_t size = 0;
      RecordCard::Result r = card_.list_next(name, sizeof name, &size);
      timed(t0, "listing kilorec/");
      if (r == RecordCard::Result::Ok) {
        uint32_t boot = 0, seq = 0;
        const char* p;
        if (str_eq(name, "SAVE_ON")) {
          flag_file_ = true;
        } else if (ends_with(name, ".tmp") || ends_with(name, ".rtm")) {
          if (n_clean_ < 16) format(clean_[n_clean_++], sizeof clean_[0], "%s/%s", kDir, name);
          if (ends_with(name, ".rtm") && name[0] == 'b' && number(name + 1, &boot, &p) && boot >= boot_) boot_ = boot;
        } else if (name[0] == 'b' && number(name + 1, &boot, &p)) {
          if (boot > boot_) boot_ = boot;  // boot_ holds the highest so far
          if (p[0] == '-' && p[1] == 's' && number(p + 2, &seq, &p) && str_eq(p, ".krc")) add_file(boot, seq, size);
          else if (str_eq(p, ".run") && n_run_boots_ < 64) run_boots_[n_run_boots_++] = boot;
          else if (str_eq(p, ".log") && n_log_boots_ < 64) log_boots_[n_log_boots_++] = boot;
        }
        return;
      }
      // the end of the listing: this boot's number
      boot_ = boot_ + 1;
      rec_.flight().set_boot(boot_);
      if (flag_file_) save_on_ = true;
      if (save_on_ != flag_file_) flag_dirty_ = true;
      logf(log_, LogLevel::Info, "card: kilorec/ has %u files (%u KB) of earlier boots; this is boot %u; saving %s",
           (unsigned)n_files_, (unsigned)(card_bytes_ >> 10), (unsigned)boot_,
           save_on_ ? (flag_file_ ? "on (SAVE_ON)" : "on") : "off (kilo record save on)");
      job_ = Job::ScanClean;
      return;
    }
    case Job::ScanClean: {
      if (n_clean_ == 0) {
        job_ = Job::ScanRuns;
        run_boot_i_ = 0;
        // newest boots first: they fill the card run table
        for (uint32_t i = 1; i < n_run_boots_; i++)
          for (uint32_t k = i; k > 0 && run_boots_[k - 1] < run_boots_[k]; k--) {
            uint32_t t = run_boots_[k];
            run_boots_[k] = run_boots_[k - 1];
            run_boots_[k - 1] = t;
          }
        return;
      }
      RecordCard::Result r = card_.remove(clean_[--n_clean_]);  // a power cut's half file
      timed(t0, "removing a half-written file");
      if (r == RecordCard::Result::Ok) logf(log_, LogLevel::Info, "card: removed %s (a save cut short)", clean_[n_clean_]);
      return;
    }
    case Job::ScanRuns: {
      if (run_boot_i_ >= n_run_boots_ || n_card_runs_ >= kMaxCardRuns) {
        if (file_open_) {
          card_.close();
          file_open_ = false;
        }
        phase_ = Phase::Idle;
        job_ = Job::None;
        if (n_card_runs_ > 0)
          logf(log_, LogLevel::Info, "card: %u runs of earlier boots listed (kilo runs --host)", (unsigned)n_card_runs_);
        return;
      }
      uint32_t boot = run_boots_[run_boot_i_];
      if (!file_open_) {
        char name[40];
        boot_name(name, sizeof name, boot, "run");
        RecordCard::Result r = card_.open(name);
        timed(t0, "opening a run index");
        if (r != RecordCard::Result::Ok) {
          run_boot_i_++;
          return;
        }
        file_open_ = true;
        scan_off_ = 0;
        RunsHeader h;
        if (card_.read((uint8_t*)&h, sizeof h) != (int32_t)sizeof h || h.magic != kRunsMagic ||
            h.version != kRunsVersion || h.boot != boot) {
          card_.close();
          file_open_ = false;
          run_boot_i_++;
        }
        return;
      }
      Run r;
      int32_t n = card_.read((uint8_t*)&r, sizeof r);
      timed(t0, "reading a run index");
      if (n != (int32_t)sizeof r) {
        card_.close();
        file_open_ = false;
        run_boot_i_++;
        return;
      }
      if (n_card_runs_ < kMaxCardRuns && r.end_us != 0) {
        CardRun& c = card_runs_[n_card_runs_++];
        c.boot = boot;
        c.run = r;
        c.run.flags = (uint8_t)((r.flags & ~KILO_WALK_FLAGS_RECORDING) | KILO_WALK_FLAGS_SAVED | KILO_WALK_FLAGS_EARLIER_BOOT);
      }
      return;
    }
    default:
      job_ = Job::ScanDir;
      return;
  }
}

// ---- saving -----------------------------------------------------------------------------

bool Saver::want_save(uint64_t now) const {
  if (rec_.recording()) return false;  // a run or its post-roll: after it
  const FlightStore& f = rec_.flight();
  uint32_t sealed_end = f.empty() ? 0 : (f.open() ? f.last_seq() : f.next_seq());
  uint32_t from = saved_seq_ > f.first_seq() ? saved_seq_ : f.first_seq();
  uint32_t waiting = sealed_end > from ? sealed_end - from : 0;
  bool open_data = f.open() && f.used(f.last_seq()) > kBlockHeader;
  bool log_new = ring_.next_seq() != log_seq_;
  bool run_new = rec_.last_run_end_us() > last_run_saved_us_;
  if (save_now_ || run_new) return waiting > 0 || open_data || log_new || rec_.runs_changed() != runs_saved_;
  if (waiting >= kIdleBlocks) return true;
  return now - last_save_us_ >= kIdleUs && (waiting > 0 || log_new);
}

void Saver::begin_save() {
  FlightStore& f = rec_.flight();
  bool run_new = rec_.last_run_end_us() > last_run_saved_us_;
  if ((save_now_ || run_new) && f.open() && f.used(f.last_seq()) > kBlockHeader) f.seal();  // the run's tail with it
  metrics_due_ = true;
  if (!f.empty() && saved_seq_ < f.first_seq()) {
    logf(log_, LogLevel::Warn, "card: blocks %u..%u were overwritten in RAM before they could be saved",
         (unsigned)saved_seq_, (unsigned)(f.first_seq() - 1));
    saved_seq_ = f.first_seq();
  }
  next_save_part();
}

// After the blocks of a file: more blocks, the run index, the log, the cap, done.
void Saver::next_save_part() {
  const FlightStore& f = rec_.flight();
  uint32_t sealed_end = f.empty() ? 0 : (f.open() ? f.last_seq() : f.next_seq());
  if (!f.empty() && saved_seq_ < sealed_end) {
    save_first_ = saved_seq_;
    save_end_ = sealed_end - saved_seq_ > kBlocksPerFile ? saved_seq_ + kBlocksPerFile : sealed_end;
    save_seq_ = save_first_;
    save_off_ = 0;
    save_size_ = 0;
    job_ = Job::BlocksCreate;
    return;
  }
  if (rec_.runs_changed() != runs_saved_ && rec_.runs_kept() > 0) {
    job_ = Job::RunsCreate;
    return;
  }
  if (ring_.next_seq() != log_seq_ || metrics_due_) {
    job_ = Job::LogOpen;
    return;
  }
  if (card_bytes_ > cap_bytes_ || n_files_ >= kMaxFiles) {
    job_ = Job::Cap;
    return;
  }
  job_ = Job::None;
  last_save_us_ = clock_.now_us();
  if (rec_.last_run_end_us() > last_run_saved_us_) run_saves_++;  // a run's end is on the card now
  last_run_saved_us_ = rec_.last_run_end_us();
  save_now_ = false;
  failures_ = 0;
  backoff_us_ = 0;
}

uint32_t Saver::fill_log(uint8_t* buf, uint32_t n) {
  uint32_t len = 0;
  if (log_seq_ < ring_.first_seq()) {
    int k = format((char*)buf, n, "0 W (%u lines of the log were overwritten before this save)\n",
                   (unsigned)(ring_.first_seq() - log_seq_));
    len += (uint32_t)k;
    log_seq_ = ring_.first_seq();
  }
  static const char kTag[] = "DIWE";
  while (log_seq_ < ring_.next_seq()) {
    const LogEntry* e = ring_.get(log_seq_);
    if (e == nullptr) break;
    if (len + e->len + 16 > n) return len;
    len += (uint32_t)format((char*)buf + len, n - len, "%u %c %s\n", (unsigned)e->t_ms,
                            kTag[(unsigned)e->level & 3u], e->text);
    log_seq_++;
  }
  if (metrics_due_ && metrics_ != nullptr) {
    uint32_t lines = metrics_->metric_lines();
    uint32_t t_ms = (uint32_t)(clock_.now_us() / 1000u);
    if (len + lines * 210u > n) return len;  // next write
    for (uint32_t i = 0; i < lines; i++) {
      char line[204];
      metrics_->metric_line(i, line, sizeof line);
      len += (uint32_t)format((char*)buf + len, n - len, "%u M %s\n", (unsigned)t_ms, line);
    }
  }
  metrics_due_ = false;
  return len;
}

void Saver::step_save() {
  FlightStore& f = rec_.flight();
  uint64_t t0 = clock_.now_us();
  char name[48], to[48];
  switch (job_) {
    case Job::BlocksCreate:
      file_name(name, sizeof name, boot_, save_first_, "tmp");
      if (!timed_ok(card_.create(name), t0, "creating a block file")) return;
      file_open_ = true;
      job_ = Job::BlocksWrite;
      return;
    case Job::BlocksWrite: {
      if (save_seq_ >= save_end_) {
        job_ = Job::BlocksSync;
        return;
      }
      if (!f.has(save_seq_) || !f.sealed(save_seq_)) {  // overwritten meanwhile: the file ends here
        save_end_ = save_seq_;
        job_ = Job::BlocksSync;
        return;
      }
      uint32_t used = f.used(save_seq_);
      uint32_t n = used - save_off_ > kChunk ? kChunk : used - save_off_;
      RecordCard::Result r = card_.write(f.block(save_seq_) + save_off_, n);
      writes_++;
      if (!timed_ok(r, t0, "writing blocks")) return;
      save_off_ += n;
      save_size_ += n;
      if (save_off_ >= used) {
        save_seq_++;
        save_off_ = 0;
      }
      return;
    }
    case Job::BlocksSync:
      writes_++;
      if (!timed_ok(card_.sync(), t0, "syncing a block file")) return;
      job_ = Job::BlocksClose;
      return;
    case Job::BlocksClose:
      writes_++;
      file_open_ = false;
      if (!timed_ok(card_.close(), t0, "closing a block file")) return;
      job_ = Job::BlocksRename;
      return;
    case Job::BlocksRename:
      file_name(name, sizeof name, boot_, save_first_, "tmp");
      if (save_end_ == save_first_) {  // nothing in it (every block overwritten)
        card_.remove(name);
        timed(t0, "removing an empty file");
        saved_seq_ = f.first_seq();
        next_save_part();
        return;
      }
      file_name(to, sizeof to, boot_, save_first_, "krc");
      writes_++;
      if (!timed_ok(card_.rename(name, to), t0, "renaming a block file")) return;
      add_file(boot_, save_first_, save_size_);
      saved_seq_ = save_end_;
      next_save_part();
      return;
    case Job::RunsCreate:
      boot_name(name, sizeof name, boot_, "rtm");
      if (!timed_ok(card_.create(name), t0, "creating the run index")) return;
      file_open_ = true;
      runs_saved_ = rec_.runs_changed();
      runs_off_ = 0;
      buf_len_ = 0;
      job_ = Job::RunsWrite;
      return;
    case Job::RunsWrite: {
      // the header, then whole Run entries, a chunk at a time
      uint32_t len = 0;
      if (runs_off_ == 0) {
        RunsHeader h = {kRunsMagic, kRunsVersion, boot_, rec_.runs_kept()};
        mem_copy(buf_, &h, sizeof h);
        len = sizeof h;
      }
      uint32_t after = 0, k = 0;
      const Run* r = rec_.run_after(0);
      while (r != nullptr && k < runs_off_) {  // skip what was written
        after = r->id;
        r = rec_.run_after(after);
        k++;
      }
      while (r != nullptr && len + sizeof(Run) <= kChunk) {
        mem_copy(buf_ + len, r, sizeof(Run));
        len += sizeof(Run);
        runs_off_++;
        r = rec_.run_after(r->id);
      }
      if (len == 0) {
        job_ = Job::RunsClose;
        return;
      }
      writes_++;
      if (!timed_ok(card_.write(buf_, len), t0, "writing the run index")) return;
      if (r == nullptr) job_ = Job::RunsClose;
      return;
    }
    case Job::RunsClose:
      writes_++;
      file_open_ = false;
      if (!timed_ok(card_.close(), t0, "closing the run index")) return;
      job_ = Job::RunsRemove;
      return;
    case Job::RunsRemove: {
      boot_name(name, sizeof name, boot_, "run");
      RecordCard::Result r = card_.remove(name);
      if (r == RecordCard::Result::Missing) r = RecordCard::Result::Ok;
      if (!timed_ok(r, t0, "removing the old run index")) return;
      job_ = Job::RunsRename;
      return;
    }
    case Job::RunsRename:
      boot_name(name, sizeof name, boot_, "rtm");
      boot_name(to, sizeof to, boot_, "run");
      writes_++;
      if (!timed_ok(card_.rename(name, to), t0, "renaming the run index")) return;
      next_save_part();
      return;
    case Job::LogOpen:
      boot_name(name, sizeof name, boot_, "log");
      if (!timed_ok(card_.append(name), t0, "opening the log")) return;
      file_open_ = true;
      job_ = Job::LogWrite;
      return;
    case Job::LogWrite: {
      uint32_t len = fill_log(buf_, kChunk);
      if (len == 0) {
        job_ = Job::LogClose;
        return;
      }
      writes_++;
      if (!timed_ok(card_.write(buf_, len), t0, "writing the log")) return;
      return;
    }
    case Job::LogClose:
      writes_++;
      file_open_ = false;
      if (!timed_ok(card_.close(), t0, "closing the log")) return;
      job_ = Job::None;
      next_save_part();
      return;
    case Job::Cap: {
      int32_t i = oldest_file();
      if (i < 0 || (card_bytes_ <= cap_bytes_ && n_files_ < kMaxFiles)) {
        next_save_part();
        if (job_ == Job::Cap) job_ = Job::None;
        return;
      }
      if (n_files_ == 1 && files_[i].boot == boot_) {  // never this boot's last file
        job_ = Job::None;
        return;
      }
      file_name(name, sizeof name, files_[i].boot, files_[i].seq, "krc");
      RecordCard::Result r = card_.remove(name);
      if (r == RecordCard::Result::Missing) r = RecordCard::Result::Ok;
      if (!timed_ok(r, t0, "removing the oldest recording")) return;
      drop_file((uint32_t)i);
      return;
    }
    default:
      job_ = Job::None;
      return;
  }
}

void Saver::step_delete() {
  uint64_t t0 = clock_.now_us();
  char name[48];
  bool all = delete_boot_ == 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n_files_; i++) {
    const File& fl = files_[i];
    if (fl.boot == boot_ || (!all && fl.boot != delete_boot_)) continue;
    file_name(name, sizeof name, fl.boot, fl.seq, "krc");
    RecordCard::Result r = card_.remove(name);
    if (r == RecordCard::Result::Missing) r = RecordCard::Result::Ok;
    if (!timed_ok(r, t0, "deleting a recording")) return;
    drop_file(i);
    return;
  }
  for (uint32_t i = 0; i < n_run_boots_ + n_log_boots_; i++) {  // then the boots' .run and .log
    bool run = i < n_run_boots_;
    uint32_t* list = run ? run_boots_ : log_boots_;
    uint32_t* n = run ? &n_run_boots_ : &n_log_boots_;
    uint32_t k = run ? i : i - n_run_boots_;
    uint32_t b = list[k];
    if (b == boot_ || (!all && b != delete_boot_)) continue;
    boot_name(name, sizeof name, b, run ? "run" : "log");
    RecordCard::Result r = card_.remove(name);
    if (r == RecordCard::Result::Missing) r = RecordCard::Result::Ok;
    if (!timed_ok(r, t0, "deleting a run index or log")) return;
    list[k] = list[--*n];
    return;
  }
  uint32_t w = 0;  // the card run table without them
  for (uint32_t i = 0; i < n_card_runs_; i++)
    if (!all && card_runs_[i].boot != delete_boot_) card_runs_[w++] = card_runs_[i];
  n_card_runs_ = w;
  delete_on_ = false;
  job_ = Job::None;
  logf(log_, LogLevel::Info, "card: deleted the recordings of %s", all ? "every earlier boot" : "one boot");
}

// ---- the bench --------------------------------------------------------------------------

bool Saver::start_bench(uint16_t count, BenchSink& sink, bool limp, const char** why) {
  if (count == 0 || count > 256) *why = "count: 1..256";
  else if (!scanned() || phase_ == Phase::Failed) *why = "no card (or it failed)";
  else if (!limp) *why = "the body is not limp: kilo posture rest, or switch the servos off";
  else if (job_ != Job::None || rd_.on || bench_.on) *why = "the card is busy (a save or a download)";
  else *why = nullptr;
  if (*why != nullptr) return false;
  bench_ = {};
  bench_.on = true;
  bench_.sink = &sink;
  bench_.count = count;
  bench_.min_us = 0xFFFFFFFFu;
  job_ = Job::BenchCreate;
  return true;
}

void Saver::bench_report(uint32_t size, uint8_t error) {
  kilo_host_bench_result_t r;
  mem_zero(&r, sizeof r);
  r.write_bytes = size;
  r.error = error;
  if (size != 0) {
    r.count = (uint16_t)bench_.done;
    r.min_us = bench_.done ? bench_.min_us : 0;
    r.max_us = bench_.max_us;
    r.mean_us = bench_.done ? (uint32_t)(bench_.sum_us / bench_.done) : 0;
    r.sync_us = bench_.sync_us;
    logf(log_, LogLevel::Info, "card bench: %u writes of %u B: min %u mean %u max %u us, sync %u us",
         (unsigned)r.count, (unsigned)size, (unsigned)r.min_us, (unsigned)r.mean_us, (unsigned)r.max_us,
         (unsigned)r.sync_us);
  } else {
    r.close_us = bench_.close_us;
    r.remove_us = bench_.sync_us;  // the remove's time (kept in sync_us at the end)
    logf(log_, error ? LogLevel::Warn : LogLevel::Info, "card bench: close %u us, delete %u us%s",
         (unsigned)r.close_us, (unsigned)r.remove_us, error ? ": stopped" : "");
  }
  if (bench_.sink != nullptr) bench_.sink->bench_result(r);
}

void Saver::step_bench(bool limp) {
  if (!limp && job_ != Job::BenchClose && job_ != Job::BenchRemove) {
    if (file_open_) {
      card_.close();
      file_open_ = false;
    }
    bench_report(0, 2);  // the body moved
    job_ = Job::BenchRemove;
    bench_.sink = nullptr;
    return;
  }
  uint64_t t0 = clock_.now_us();
  RecordCard::Result r;
  switch (job_) {
    case Job::BenchCreate:
      r = card_.create(kBench);
      timed(t0, "bench: create");
      if (r != RecordCard::Result::Ok) {
        bench_report(0, 1);
        bench_.on = false;
        job_ = Job::None;
        return;
      }
      file_open_ = true;
      job_ = Job::BenchWrite;
      return;
    case Job::BenchWrite: {
      uint32_t size = kBenchSize[bench_.size_i];
      static uint8_t pattern[16384];
      pattern[bench_.done & 0x3FFFu] = (uint8_t)bench_.done;
      r = card_.write(pattern, size);
      uint32_t dt = (uint32_t)(clock_.now_us() - t0);
      writes_++;
      if (dt > write_max_us_) write_max_us_ = dt;
      if (r != RecordCard::Result::Ok) {
        card_.close();
        file_open_ = false;
        bench_report(size, 1);
        bench_report(0, 1);
        job_ = Job::BenchRemove;
        bench_.sink = nullptr;
        return;
      }
      bench_.done++;
      bench_.sum_us += dt;
      if (dt < bench_.min_us) bench_.min_us = dt;
      if (dt > bench_.max_us) bench_.max_us = dt;
      if (bench_.done >= bench_.count) job_ = Job::BenchSync;
      return;
    }
    case Job::BenchSync:
      r = card_.sync();
      bench_.sync_us = (uint32_t)(clock_.now_us() - t0);
      bench_report(kBenchSize[bench_.size_i], r == RecordCard::Result::Ok ? 0 : 1);
      bench_.done = 0;
      bench_.sum_us = 0;
      bench_.max_us = 0;
      bench_.min_us = 0xFFFFFFFFu;
      if (++bench_.size_i >= kBenchSizes || r != RecordCard::Result::Ok) job_ = Job::BenchClose;
      else job_ = Job::BenchWrite;
      return;
    case Job::BenchClose:
      if (file_open_) card_.close();
      file_open_ = false;
      bench_.close_us = (uint32_t)(clock_.now_us() - t0);
      job_ = Job::BenchRemove;
      return;
    case Job::BenchRemove:
      card_.remove(kBench);
      bench_.sync_us = (uint32_t)(clock_.now_us() - t0);
      if (bench_.sink != nullptr) bench_report(0, 0);
      bench_.on = false;
      job_ = Job::None;
      return;
    default:
      bench_.on = false;
      job_ = Job::None;
      return;
  }
}

// ---- reading back -----------------------------------------------------------------------

bool Saver::read_blocks(uint32_t boot, uint64_t from_us, uint64_t to_us, uint32_t below_seq, uint32_t owner) {
  if (rd_.on || !scanned()) return false;
  uint32_t first = n_files_;
  for (uint32_t i = 0; i < n_files_; i++)
    if (files_[i].boot == boot && (boot != boot_ || files_[i].seq < below_seq)) {
      first = i;
      break;
    }
  if (first == n_files_) return false;
  rd_ = {};
  rd_.on = true;
  rd_.owner = owner;
  rd_.boot = boot;
  rd_.from_us = from_us;
  rd_.to_us = to_us;
  rd_.below_seq = boot == boot_ ? below_seq : 0xFFFFFFFFu;
  rd_.file = first;
  rd_.last_seq = 0xFFFFFFFFu;
  return true;
}

bool Saver::read_log(uint32_t boot, uint32_t owner) {
  if (rd_.on || !scanned()) return false;
  bool have = false;
  for (uint32_t i = 0; i < n_log_boots_; i++) have = have || log_boots_[i] == boot;
  if (!have && boot != boot_) return false;
  rd_ = {};
  rd_.on = true;
  rd_.log = true;
  rd_.owner = owner;
  rd_.boot = boot;
  return true;
}

void Saver::read_cancel(uint32_t owner) {
  if (!rd_.on || rd_.owner != owner) return;
  if (rd_.opened) card_.close();
  rd_.on = false;
}

bool Saver::rd_open(uint32_t file_seq) {
  char name[48];
  if (rd_.opened) card_.close();
  rd_.opened = false;
  uint64_t t0 = clock_.now_us();
  if (rd_.log) boot_name(name, sizeof name, rd_.boot, "log");
  else file_name(name, sizeof name, rd_.boot, file_seq, "krc");
  RecordCard::Result r = card_.open(name);
  timed(t0, "opening a recording");
  if (r != RecordCard::Result::Ok) return false;
  rd_.opened = true;
  rd_.open_seq = file_seq;
  return true;
}

bool Saver::rd_next_file() {
  for (uint32_t i = rd_.file + 1; i < n_files_; i++)
    if (files_[i].boot == rd_.boot) {
      rd_.file = i;
      rd_.off = 0;
      return true;
    }
  rd_.done_files = true;
  return false;
}

int32_t Saver::read(uint32_t owner, uint8_t* out, uint32_t n) {
  if (!rd_.on || rd_.owner != owner) return -1;
  if (!safe_ || file_open_ || job_ != Job::None) return -2;  // not safe, or a save or the bench has the card
  int32_t r = rd_step(out, n);
  if (r == 0 || r == -1) {
    if (rd_.opened) card_.close();
    rd_.opened = false;
    rd_.on = false;
  }
  return r;
}

int32_t Saver::rd_step(uint8_t* out, uint32_t n) {
  Reader& d = rd_;
  if (d.log) {
    if (!d.opened) return rd_open(0) ? -2 : -1;
    int32_t got = card_.read(out, n);
    return got < 0 ? -1 : got;
  }
  if (d.streaming) {
    if (!d.opened || d.open_seq != d.s_file_seq) return rd_open(d.s_file_seq) ? -2 : -1;
    if (card_.seek(d.s_off + d.s_pos) != RecordCard::Result::Ok) return -1;
    uint32_t want = d.s_used - d.s_pos < n ? d.s_used - d.s_pos : n;
    int32_t got = card_.read(out, want);
    if (got <= 0) return -1;
    d.s_pos += (uint32_t)got;
    if (d.s_pos >= d.s_used) {
      d.streaming = false;
      d.last_seq = d.s_seq;
    }
    return got;
  }
  // the next header decides about the pending block
  bool have_next = false;
  kilo_record_block_header_t h = {};
  uint32_t h_file_seq = 0, h_off = 0;
  if (!d.done_files) {
    uint32_t fs = files_[d.file].seq;
    if (!d.opened || d.open_seq != fs) return rd_open(fs) ? -2 : -1;
    uint64_t t0 = clock_.now_us();
    int32_t got = -1;
    if (card_.seek(d.off) == RecordCard::Result::Ok) got = card_.read((uint8_t*)&h, sizeof h);
    timed(t0, "reading a recording");
    if (got != (int32_t)sizeof h || h.magic != kBlockMagic || h.used < kBlockHeader || h.used > kBlockBytes) {
      rd_next_file();
      return -2;
    }
    if (h.seq >= d.below_seq) {
      d.done_files = true;  // the rest is in RAM
    } else {
      have_next = true;
      h_file_seq = fs;
      h_off = d.off;
      d.off += h.used;
    }
  }
  uint64_t ht0 = ((uint64_t)h.t0_us_hi << 32) | h.t0_us_lo;
  bool next_known = have_next || (d.done_files && h.magic == kBlockMagic);
  if (d.pend) {
    bool covers = d.pend_t0 <= d.to_us && (d.pend_t0 >= d.from_us || !next_known || ht0 > d.from_us);
    if (d.pend_t0 > d.to_us) {  // past the range: done
      d.pend = false;
      return 0;
    }
    d.pend = false;
    if (covers) {
      d.streaming = true;
      d.s_file_seq = d.pend_file_seq;
      d.s_off = d.pend_off;
      d.s_used = d.pend_used;
      d.s_pos = 0;
      d.s_seq = d.pend_seq;
    }
  }
  if (have_next) {
    d.pend = true;
    d.pend_seq = h.seq;
    d.pend_file_seq = h_file_seq;
    d.pend_off = h_off;
    d.pend_used = h.used;
    d.pend_t0 = ht0;
    return -2;
  }
  if (d.done_files && !d.streaming) return 0;
  return -2;
}

}  // namespace record
}  // namespace kilo
