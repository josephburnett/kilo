#include "update/updater.h"
#include "proto/kilo_names.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {

namespace {

constexpr char kMarker[] = KILO_HOST_UPDATE_MARKER;
constexpr uint32_t kMarkerLen = sizeof kMarker - 1;

const char* result_name(KernelStore::Result r) {
  switch (r) {
    case KernelStore::Result::Ok: return "ok";
    case KernelStore::Result::Missing: return "missing";
    default: return "failed";
  }
}

}  // namespace

const char* update_error_name(uint8_t error) { return kilo_update_error_name(error); }

bool has_kernel_marker(const uint8_t* data, uint32_t len) {
  if (len < kMarkerLen) return false;
  for (uint32_t i = 0; i + kMarkerLen <= len; i++) {
    if (data[i] != (uint8_t)kMarker[0]) continue;
    uint32_t k = 1;
    while (k < kMarkerLen && data[i + k] == (uint8_t)kMarker[k]) k++;
    if (k == kMarkerLen) return true;
  }
  return false;
}

KernelUpdater::KernelUpdater(uint8_t* buffer, uint32_t capacity, KernelStore& store, Clock& clock, Log& log)
    : buf_(buffer), capacity_(capacity), store_(store), clock_(clock), log_(log) {}

uint8_t KernelUpdater::begin(uint32_t size, uint32_t crc32, uint8_t flags) {
  if (busy()) return 0xFF;  // the caller refuses first (HostServer: UPDATING)
  status_ = {};
  status_.size = size;
  want_crc_ = crc32;
  flags_ = flags;
  error_ = KILO_UPDATE_ERROR_NONE;
  crc_.reset();
  if (size == 0 || size > capacity_) {
    status_.size = 0;
    fail(KILO_UPDATE_ERROR_BAD_SIZE);
    return KILO_UPDATE_ERROR_BAD_SIZE;
  }
  set_phase(KILO_UPDATE_PHASE_RECEIVING);
  return KILO_UPDATE_ERROR_NONE;
}

uint8_t KernelUpdater::write(uint32_t offset, const uint8_t* data, uint32_t len) {
  if (!receiving()) return KILO_UPDATE_ERROR_BAD_OFFSET;
  if (error_ != KILO_UPDATE_ERROR_NONE) return error_;
  if (offset != status_.received || len > status_.size - status_.received) {
    error_ = KILO_UPDATE_ERROR_BAD_OFFSET;
    return error_;
  }
  mem_copy(buf_ + offset, data, len);
  crc_.update(data, len);
  status_.received += len;
  return KILO_UPDATE_ERROR_NONE;
}

uint8_t KernelUpdater::commit() {
  if (!receiving()) return KILO_UPDATE_ERROR_SHORT;  // the caller checks first
  set_phase(KILO_UPDATE_PHASE_CHECKING);
  status_.crc32 = crc_.value();
  uint8_t e = error_;
  if (e == KILO_UPDATE_ERROR_NONE && status_.received < status_.size) e = KILO_UPDATE_ERROR_SHORT;
  if (e == KILO_UPDATE_ERROR_NONE && status_.crc32 != want_crc_) e = KILO_UPDATE_ERROR_CRC_MISMATCH;
  bool force = (flags_ & KILO_UPDATE_FLAGS_FORCE) != 0;
  bool marker = e == KILO_UPDATE_ERROR_NONE && has_kernel_marker(buf_, status_.size);
  if (e == KILO_UPDATE_ERROR_NONE && !marker && !force) e = KILO_UPDATE_ERROR_NOT_A_KERNEL;
  if (e != KILO_UPDATE_ERROR_NONE) {
    logf(log_, LogLevel::Warn, "update: image refused: %s (%u of %u bytes, crc32 %08x, UPDATE_BEGIN said %08x)",
         update_error_name(e), (unsigned)status_.received, (unsigned)status_.size, (unsigned)status_.crc32,
         (unsigned)want_crc_);
    fail(e);
    return e;
  }
  logf(log_, LogLevel::Info, "update: image good: %u bytes, crc32 %08x, %s", (unsigned)status_.size,
       (unsigned)status_.crc32,
       marker ? "the mind kernel's marker found" : "no marker (FORCE: installed anyway)");
  return KILO_UPDATE_ERROR_NONE;
}

void KernelUpdater::drop() {
  if (!receiving()) return;
  status_ = {};
  set_phase(KILO_UPDATE_PHASE_IDLE);
}

void KernelUpdater::poll() {
  switch (status_.phase) {
    case KILO_UPDATE_PHASE_CHECKING:
      set_phase(KILO_UPDATE_PHASE_WRITING);
      step_write();
      return;
    case KILO_UPDATE_PHASE_WRITING:
      step_write();
      return;
    case KILO_UPDATE_PHASE_VERIFYING:
      step_verify();
      return;
    case KILO_UPDATE_PHASE_INSTALLING:
      install();
      return;
    default:
      return;
  }
}

// kernel7.new: created on the first pass, then a slice per pass, closed
// (flushed to the card) after the last.
void KernelUpdater::step_write() {
  if (!started_) {
    started_ = true;
    phase_us_ = clock_.now_us();
    KernelStore::Result r = store_.create(kKernelNewFile);
    if (r != KernelStore::Result::Ok) {
      logf(log_, LogLevel::Error, "update: cannot create %s (%s): the card is unchanged", kKernelNewFile,
           result_name(r));
      fail(KILO_UPDATE_ERROR_WRITE_FAILED);
      return;
    }
    open_ = true;
    logf(log_, LogLevel::Info, "update: writing %s (%u bytes; the link is paused)", kKernelNewFile,
         (unsigned)status_.size);
  }
  uint32_t left = status_.size - status_.done;
  uint32_t n = left < kSliceBytes ? left : kSliceBytes;
  if (n > 0) {
    if (store_.write(buf_ + status_.done, n) != KernelStore::Result::Ok) {
      logf(log_, LogLevel::Error, "update: writing %s failed at byte %u", kKernelNewFile, (unsigned)status_.done);
      fail(KILO_UPDATE_ERROR_WRITE_FAILED);
      return;
    }
    status_.done += n;
    changed();
  }
  if (status_.done < status_.size) return;
  open_ = false;
  if (store_.close() != KernelStore::Result::Ok) {
    logf(log_, LogLevel::Error, "update: closing %s failed (not on the card)", kKernelNewFile);
    fail(KILO_UPDATE_ERROR_WRITE_FAILED);
    return;
  }
  logf(log_, LogLevel::Info, "update: %s written: %u bytes in %u ms", kKernelNewFile, (unsigned)status_.size,
       (unsigned)ms_since(phase_us_));
  set_phase(KILO_UPDATE_PHASE_VERIFYING);
}

// kernel7.new read back, up to a slice per pass, to its end: its length and
// CRC-32 must be the image's.
void KernelUpdater::step_verify() {
  if (!started_) {
    started_ = true;
    phase_us_ = clock_.now_us();
    KernelStore::Result r = store_.open(kKernelNewFile);
    if (r != KernelStore::Result::Ok) {
      logf(log_, LogLevel::Error, "update: cannot open %s to read it back (%s)", kKernelNewFile, result_name(r));
      fail(KILO_UPDATE_ERROR_VERIFY_FAILED);
      return;
    }
    open_ = true;
    crc_.reset();
  }
  uint32_t budget = kSliceBytes;
  bool end = false;
  while (budget > 0) {
    int32_t got = store_.read(chunk_, budget < kReadChunk ? budget : kReadChunk);
    if (got < 0) {
      logf(log_, LogLevel::Error, "update: reading %s back failed at byte %u", kKernelNewFile,
           (unsigned)status_.done);
      fail(KILO_UPDATE_ERROR_VERIFY_FAILED);
      return;
    }
    if (got == 0) {
      end = true;
      break;
    }
    if ((uint32_t)got > status_.size - status_.done) {
      logf(log_, LogLevel::Error, "update: %s is longer than the image (%u bytes)", kKernelNewFile,
           (unsigned)status_.size);
      fail(KILO_UPDATE_ERROR_VERIFY_FAILED);
      return;
    }
    crc_.update(chunk_, (uint32_t)got);
    status_.done += (uint32_t)got;
    budget -= (uint32_t)got;
  }
  changed();
  if (!end) return;
  open_ = false;
  store_.close();
  status_.card_crc32 = crc_.value();
  if (status_.done != status_.size || status_.card_crc32 != status_.crc32) {
    logf(log_, LogLevel::Error, "update: %s read back is not the image: %u bytes, crc32 %08x (want %u, %08x)",
         kKernelNewFile, (unsigned)status_.done, (unsigned)status_.card_crc32, (unsigned)status_.size,
         (unsigned)status_.crc32);
    fail(KILO_UPDATE_ERROR_VERIFY_FAILED);
    return;
  }
  logf(log_, LogLevel::Info, "update: %s read back: %u bytes, crc32 %08x, in %u ms", kKernelNewFile,
       (unsigned)status_.done, (unsigned)status_.card_crc32, (unsigned)ms_since(phase_us_));
  set_phase(KILO_UPDATE_PHASE_INSTALLING);
}

// The renames, in one pass: only now does kernel7.img change. The previous kernel is kernel7.old only
// for the swap (put back if the last rename fails), removed once the new one is kernel7.img: no kernel
// is kept on the card (the commits are in git: docs/laptop-setup.md, a kernel that misbehaves).
void KernelUpdater::install() {
  uint64_t t = clock_.now_us();
  KernelStore::Result r = store_.remove(kKernelOldFile);
  if (r == KernelStore::Result::Failed) {
    logf(log_, LogLevel::Error, "update: cannot remove %s: %s unchanged", kKernelOldFile, kKernelFile);
    fail(KILO_UPDATE_ERROR_INSTALL_FAILED);
    return;
  }
  if (r == KernelStore::Result::Ok) logf(log_, LogLevel::Info, "update: removed a %s left on the card", kKernelOldFile);
  if (store_.remove(kKernelBadFile) == KernelStore::Result::Ok)
    logf(log_, LogLevel::Info, "update: removed a %s left on the card", kKernelBadFile);

  r = store_.rename(kKernelFile, kKernelOldFile);
  if (r == KernelStore::Result::Failed) {
    logf(log_, LogLevel::Error, "update: cannot rename %s to %s: %s unchanged", kKernelFile, kKernelOldFile,
         kKernelFile);
    fail(KILO_UPDATE_ERROR_INSTALL_FAILED);
    return;
  }
  bool moved = r == KernelStore::Result::Ok;
  logf(log_, LogLevel::Info, moved ? "update: %s -> %s (for the swap)" : "update: no %s on the card (no %s)",
       kKernelFile, kKernelOldFile);

  r = store_.rename(kKernelNewFile, kKernelFile);
  if (r != KernelStore::Result::Ok) {
    logf(log_, LogLevel::Error, "update: cannot rename %s to %s (%s)", kKernelNewFile, kKernelFile, result_name(r));
    if (moved) {
      if (store_.rename(kKernelOldFile, kKernelFile) != KernelStore::Result::Ok) {
        logf(log_, LogLevel::Error, "update: cannot put %s back as %s either: the card has NO %s; rewrite it "
             "in a card reader (kernel/README.md)", kKernelOldFile, kKernelFile, kKernelFile);
        store_.sync();
        fail(KILO_UPDATE_ERROR_CARD_BROKEN);
        return;
      }
      logf(log_, LogLevel::Warn, "update: put %s back as %s: the card boots the old kernel", kKernelOldFile,
           kKernelFile);
    }
    fail(KILO_UPDATE_ERROR_INSTALL_FAILED);
    return;
  }
  if (moved && store_.remove(kKernelOldFile) == KernelStore::Result::Failed)
    logf(log_, LogLevel::Warn, "update: cannot remove %s (the previous kernel): the next update removes it",
         kKernelOldFile);
  if (store_.sync() != KernelStore::Result::Ok)
    logf(log_, LogLevel::Warn, "update: syncing the card's directory reported a failure");
  logf(log_, LogLevel::Info, "update: installed: %s is the new kernel (%u bytes, crc32 %08x), in %u ms",
       kKernelFile, (unsigned)status_.size, (unsigned)status_.crc32, (unsigned)ms_since(t));
  set_phase(KILO_UPDATE_PHASE_DONE);
}

// FAILED with error. A kernel7.new already begun is removed: before the
// renames the card is left as it was.
void KernelUpdater::fail(uint8_t error) {
  bool on_card = status_.phase >= KILO_UPDATE_PHASE_WRITING && status_.phase <= KILO_UPDATE_PHASE_INSTALLING;
  if (open_) {
    open_ = false;
    store_.close();
  }
  if (on_card && error != KILO_UPDATE_ERROR_CARD_BROKEN) {
    KernelStore::Result r = store_.remove(kKernelNewFile);
    if (r == KernelStore::Result::Ok) logf(log_, LogLevel::Info, "update: removed %s", kKernelNewFile);
    else if (r == KernelStore::Result::Failed) logf(log_, LogLevel::Warn, "update: cannot remove %s", kKernelNewFile);
  }
  status_.error = error;
  set_phase(KILO_UPDATE_PHASE_FAILED);
  logf(log_, LogLevel::Warn, "update: FAILED: %s", update_error_name(error));
}

void KernelUpdater::set_phase(uint8_t phase) {
  status_.phase = phase;
  if (phase == KILO_UPDATE_PHASE_VERIFYING) status_.done = 0;
  started_ = false;
  changed();
}

}  // namespace kilo
