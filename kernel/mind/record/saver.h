// saver: the flight recorder onto the SD card, so a power-off loses only
// what was not saved yet (kernel/README.md "The flight recorder"). Files in
// kilorec/ on the card's FAT partition:
//   bBBBBB-sSSSSSSS.krc  blocks of boot B from block S on, back to back
//                        (written as .tmp, renamed when whole: a power
//                        cut leaves a .tmp, which the next boot deletes)
//   bBBBBB.run           boot B's run index (Run structs; written as .rtm,
//                        the old one removed, then renamed)
//   bBBBBB.log           boot B's log history, "t_ms L text" a line, and the
//                        session metrics ("t_ms M ...") at each save, appended
//   SAVE_ON              saving is on (REC_SAVE 1 makes it, 0 removes it)
// Boot numbers: one more than the highest on the card, found when the card
// is first read (the scan); until then blocks carry boot 0.
//
// The card stalls the loop while it works (the SD driver busy-waits), and
// its write times on Kilo's card are not known, so:
// - nothing touches the card unless poll() is told it is safe (the Mind: no
//   walk or trick, nothing moving, no update or flash, quiet for a moment);
// - one card call a pass, writes of at most KILO_HOST_RECORD_WRITE_CHUNK;
// - every call is timed: the slowest is kept, and one slower than
//   KILO_HOST_RECORD_SLOW_WRITE_MS is a warning and a back-off (30 s,
//   doubling to 10 min); five failures in a row stop saving (FAILED);
// - saving is off unless the build or the card's SAVE_ON says on; the
//   bench (REC_BENCH, `kilo bench sd`) measures the card first.
// When: after a run's post-roll (the open block sealed, so the run's tail
// goes too), at REC_SAVE 2, and while idle when 8 sealed blocks wait or
// 5 minutes passed. A save writes the unsaved sealed blocks (64 at most a
// file), the run index if it changed, the new log lines and the metrics,
// then deletes the oldest files while the card holds more than
// KILO_HOST_RECORD_CARD_CAP_MB.
//
// Reading back (the host server's fetch of an earlier boot, or of this
// boot's blocks the RAM ring no longer has, or a saved log): one reader at a
// time, a card call a pass, only when it is safe and no save has a file open.
#ifndef KILO_MIND_RECORD_SAVER_H
#define KILO_MIND_RECORD_SAVER_H
#include <stdint.h>
#include "platform.h"
#include "proto/kilo_protocol.h"
#include "record/recorder.h"
#include "util/logring.h"

namespace kilo {
namespace record {

// The METRICS lines (the Mind), saved with the log.
class MetricsSource {
 public:
  virtual uint32_t metric_lines() = 0;
  virtual void metric_line(uint32_t i, char* out, uint32_t n) = 0;

 protected:
  ~MetricsSource() = default;
};

// Where REC_BENCH's results go (the host server, to the client that asked).
class BenchSink {
 public:
  virtual void bench_result(const kilo_host_bench_result_t& r) = 0;

 protected:
  ~BenchSink() = default;
};

// An earlier boot's run, from its .run file.
struct CardRun {
  uint32_t boot;
  Run run;  // run.id: that boot's id
};

class Saver {
 public:
  static constexpr uint32_t kChunk = KILO_HOST_RECORD_WRITE_CHUNK;
  static constexpr uint64_t kSlowUs = KILO_HOST_RECORD_SLOW_WRITE_MS * 1000ull;
  static constexpr uint64_t kCapBytes = (uint64_t)KILO_HOST_RECORD_CARD_CAP_MB << 20;
  static constexpr uint32_t kMaxFiles = 1024;
  static constexpr uint32_t kMaxCardRuns = 256;
  static constexpr uint32_t kBlocksPerFile = 64;
  static constexpr uint32_t kIdleBlocks = 8;          // sealed blocks waiting: save while idle
  static constexpr uint64_t kIdleUs = 300000000ull;   // ... or this long since the last save
  static constexpr uint64_t kBackoffUs = 30000000ull;
  static constexpr uint64_t kBackoffMaxUs = 600000000ull;
  static constexpr uint32_t kMaxFailures = 5;
  static constexpr uint32_t kBenchSizes = 3;          // 512 B, 4 KB, 16 KB a write

  Saver(Clock& clock, Log& log, Recorder& recorder, LogRing& ring, RecordCard& card, bool save_on);
  void set_metrics(MetricsSource* m) { metrics_ = m; }
  // The card's cap (the tests; KILO_HOST_RECORD_CARD_CAP_MB otherwise).
  void set_cap(uint64_t bytes) { cap_bytes_ = bytes; }

  // One pass: at most one card call, only when safe. limp: the body is
  // limp or off (the bench needs it).
  void poll(bool safe, bool limp);

  // REC_SAVE: 0 off, 1 on (SAVE_ON written at the next safe moment), 2 now.
  void set_mode(uint8_t mode);
  // REC_BENCH: false (and *why) if it cannot start now.
  bool start_bench(uint16_t count, BenchSink& sink, bool limp, const char** why);
  bool bench_running() const { return bench_.on; }
  // REC_DELETE (0xFFFFFFFF: every earlier boot): the files go at the next safe moments.
  bool request_delete(uint32_t boot, uint32_t* files);

  // ---- reading back ----
  // Start reading boot's blocks whose time overlaps [from_us, to_us] (for
  // this boot: only blocks below below_seq, the rest is in RAM), or its
  // log. False if there is nothing on the card for it, or a reader runs.
  bool read_blocks(uint32_t boot, uint64_t from_us, uint64_t to_us, uint32_t below_seq, uint32_t owner);
  bool read_log(uint32_t boot, uint32_t owner);
  // Up to n bytes into out: the count; 0 at the end (the reader closes);
  // -1 failed (closed); -2 nothing this pass (wait). Only the owner reads.
  int32_t read(uint32_t owner, uint8_t* out, uint32_t n);
  void read_cancel(uint32_t owner);
  bool reading() const { return rd_.on; }
  // The last block seq the reader sent (this boot: where RAM takes over).
  uint32_t read_last_seq() const { return rd_.last_seq; }

  // ---- the card's runs of earlier boots ----
  uint32_t card_runs() const { return n_card_runs_; }
  const CardRun& card_run(uint32_t i) const { return card_runs_[i]; }
  // This boot's blocks below this seq are on the card.
  uint32_t saved_seq() const { return saved_seq_; }
  bool has_card_blocks(uint32_t boot) const;

  void fill_status(kilo_host_rec_status_t& s) const;
  uint8_t state() const;
  bool scanned() const { return phase_ != Phase::Scan && phase_ != Phase::NoCard; }
  uint32_t boot() const { return boot_; }
  uint32_t writes() const { return writes_; }
  uint32_t write_max_us() const { return write_max_us_; }
  uint32_t slow_writes() const { return slow_; }
  uint32_t errors() const { return errors_; }
  uint64_t card_bytes() const { return card_bytes_; }
  uint32_t files() const { return n_files_; }
  bool save_on() const { return save_on_; }

 private:
  enum class Phase : uint8_t { NoCard, Scan, Idle, Failed };
  enum class Job : uint8_t {
    None, ScanDir, ScanList, ScanClean, ScanRuns,
    BlocksCreate, BlocksWrite, BlocksSync, BlocksClose, BlocksRename,
    RunsCreate, RunsWrite, RunsClose, RunsRemove, RunsRename,
    LogOpen, LogWrite, LogClose, Cap, Flag, Delete,
    BenchCreate, BenchWrite, BenchSync, BenchClose, BenchRemove,
  };
  struct File {
    uint32_t boot, seq, size;
  };

  bool timed_ok(RecordCard::Result r, uint64_t t0, const char* what);
  void timed(uint64_t t0, const char* what);
  void failure(const char* what);
  void step_scan();
  void step_save();
  void step_bench(bool limp);
  void step_delete();
  bool want_save(uint64_t now) const;
  void begin_save();
  void next_save_part();
  void add_file(uint32_t boot, uint32_t seq, uint32_t size);
  void drop_file(uint32_t i);
  int32_t oldest_file() const;
  void file_name(char* out, uint32_t n, uint32_t boot, uint32_t seq, const char* ext) const;
  void boot_name(char* out, uint32_t n, uint32_t boot, const char* ext) const;
  uint32_t fill_log(uint8_t* buf, uint32_t n);
  void bench_report(uint32_t size, uint8_t error);
  bool card_runs_load(const uint8_t* data, uint32_t n, uint32_t boot);

  Clock& clock_;
  Log& log_;
  Recorder& rec_;
  LogRing& ring_;
  RecordCard& card_;
  MetricsSource* metrics_ = nullptr;

  Phase phase_ = Phase::NoCard;
  Job job_ = Job::None;
  bool save_on_;
  bool flag_file_ = false;     // SAVE_ON is on the card
  bool flag_dirty_ = false;    // save_on_ changed: SAVE_ON to be written or removed
  bool save_now_ = false;
  uint32_t boot_ = 0;
  uint64_t last_save_us_ = 0;
  uint64_t last_run_saved_us_ = 0;
  uint64_t backoff_until_ = 0;
  uint64_t backoff_us_ = 0;
  uint32_t failures_ = 0;
  uint32_t writes_ = 0, write_max_us_ = 0, slow_ = 0, errors_ = 0;
  bool file_open_ = false;
  bool safe_ = false;          // the last poll()'s: reads wait for it too

  File files_[kMaxFiles] = {};
  uint32_t n_files_ = 0;
  uint64_t card_bytes_ = 0;
  uint64_t cap_bytes_ = kCapBytes;
  CardRun card_runs_[kMaxCardRuns] = {};
  uint32_t n_card_runs_ = 0;

  // the scan
  char clean_[16][40] = {};
  uint32_t n_clean_ = 0;
  uint32_t run_boots_[64] = {};
  uint32_t n_run_boots_ = 0, run_boot_i_ = 0;
  uint32_t log_boots_[64] = {};
  uint32_t n_log_boots_ = 0;
  uint32_t scan_off_ = 0;

  // a save
  uint32_t saved_seq_ = 0;
  uint32_t save_first_ = 0, save_end_ = 0;  // blocks [first, end) into this file
  uint32_t save_seq_ = 0, save_off_ = 0;   // where the write is
  uint32_t save_size_ = 0;
  uint32_t runs_saved_ = 0xFFFFFFFFu;       // rec_.runs_changed() at the last run index written
  uint32_t runs_off_ = 0;
  uint32_t log_seq_ = 0;                    // the next history line to append
  bool metrics_due_ = false;
  uint32_t delete_boot_ = 0;
  bool delete_on_ = false;
  uint8_t buf_[kChunk] = {};
  uint32_t buf_len_ = 0, buf_off_ = 0;

  // the bench
  struct Bench {
    bool on;
    BenchSink* sink;
    uint16_t count;
    uint32_t size_i, done;
    uint32_t min_us, max_us;
    uint64_t sum_us;
    uint32_t sync_us, close_us;
  } bench_ = {};

  // the reader
  struct Reader {
    bool on, log;
    uint32_t owner, boot;
    uint64_t from_us, to_us;
    uint32_t below_seq;
    uint32_t file;          // index into files_ order (by seq) of the boot
    uint32_t file_seq;      // the open file's first seq (0xFFFFFFFF: none open)
    uint32_t off;           // the next header's offset in it
    bool pend;              // a block waits for the next header's t0 to decide
    uint32_t pend_seq, pend_file_seq, pend_off, pend_used;
    uint64_t pend_t0;
    bool streaming;         // sending the chosen block
    uint32_t s_file_seq, s_off, s_used, s_pos, s_seq;
    bool opened;            // the card file of s_/off is open
    uint32_t open_seq;
    uint32_t last_seq;
    bool done_files;
  } rd_ = {};
  bool rd_next_file();
  int32_t rd_step(uint8_t* out, uint32_t n);
  bool rd_open(uint32_t file_seq);
};

}  // namespace record
}  // namespace kilo

#endif
