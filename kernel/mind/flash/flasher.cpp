#include "flash/flasher.h"
#include "link/names.h"
#include "util/log.h"
#include "util/mem.h"

namespace kilo {

namespace {

// RESET low: link.reset_pulse_ms, what mind/kilo/tools/gpio.py holds for
// `kilo flash` (docs/architecture.md Appendix F).
constexpr uint64_t kResetUs = KILO_RESET_PULSE_MS * 1000u;
// After release, before the first GET_SYNC: the 328P's start-up delay (the
// Uno fuses' SUT, 65 ms) and some margin. From then on a GET_SYNC waits in
// the 328P's two-byte RX FIFO until optiboot reads it, even while the Uno
// build still blinks its LED (3 flashes of 125 ms), so only one is sent at
// a time and the first is given until well after the blink: a third byte
// in the FIFO would desync optiboot, which then resets into the firmware.
constexpr uint64_t kBootWaitUs = 100000;
// The first GET_SYNC's wait for INSYNC OK: past start-up and the blink
// (65 + 375 ms after the reset edge) with margin.
constexpr uint64_t kFirstSyncReplyUs = 600000;
// Each later GET_SYNC's (optiboot answers at once when it listens; avrdude
// -c arduino tries 10 times).
constexpr uint64_t kSyncReplyUs = 200000;
// No new GET_SYNC this long after release: optiboot's watchdog starts the
// firmware about 1 s after its LED flashes (optiboot.c WATCHDOG_1S; every
// byte it receives restarts it), so a later one would reach the firmware.
constexpr uint64_t kSyncWindowUs = 1300000;
// In sync: drop late answers to earlier GET_SYNCs before the next command.
constexpr uint64_t kDrainUs = 20000;
// Any other command's answer. The slowest is PROG_PAGE: 133 bytes at
// 115200 (11.5 ms) and a page erase and write (2 x 4.5 ms at most, the
// 328P datasheet's tWD_FLASH). Well inside optiboot's 1 s watchdog.
constexpr uint64_t kReplyUs = 200000;
// After LEAVE_PROGMODE's answer: optiboot's 16 ms watchdog reset, then the
// firmware's start; bytes meanwhile are optiboot's last or line noise.
constexpr uint64_t kLeaveSettleUs = 50000;
// The new firmware's HELLO: `kilo flash --verify-timeout`'s default (reset,
// DMP init and the first HELLO take about 2.2 s on Kilo).
constexpr uint64_t kHelloUs = 8000000;

const char* phase_name(uint8_t p) {
  switch (p) {
    case KILO_FLASH_PHASE_RESETTING: return "resetting";
    case KILO_FLASH_PHASE_SYNCING: return "syncing";
    case KILO_FLASH_PHASE_PROGRAMMING: return "programming";
    case KILO_FLASH_PHASE_VERIFYING: return "verifying";
    case KILO_FLASH_PHASE_BOOTING: return "booting";
    case KILO_FLASH_PHASE_DONE: return "done";
    case KILO_FLASH_PHASE_FAILED: return "failed";
    default: return "?";
  }
}

}  // namespace

const char* flash_error_name(uint8_t e) {
  switch (e) {
    case KILO_FLASH_ERROR_NONE: return "NONE";
    case KILO_FLASH_ERROR_BAD_SIZE: return "BAD_SIZE";
    case KILO_FLASH_ERROR_BAD_BUILD: return "BAD_BUILD";
    case KILO_FLASH_ERROR_BAD_OFFSET: return "BAD_OFFSET";
    case KILO_FLASH_ERROR_SHORT: return "SHORT";
    case KILO_FLASH_ERROR_CRC_MISMATCH: return "CRC_MISMATCH";
    case KILO_FLASH_ERROR_SYNC_TIMEOUT: return "SYNC_TIMEOUT";
    case KILO_FLASH_ERROR_NO_REPLY: return "NO_REPLY";
    case KILO_FLASH_ERROR_BAD_REPLY: return "BAD_REPLY";
    case KILO_FLASH_ERROR_BAD_SIGNATURE: return "BAD_SIGNATURE";
    case KILO_FLASH_ERROR_VERIFY_MISMATCH: return "VERIFY_MISMATCH";
    case KILO_FLASH_ERROR_SERIAL: return "SERIAL";
    case KILO_FLASH_ERROR_BAUD: return "BAUD";
    case KILO_FLASH_ERROR_NO_HELLO: return "NO_HELLO";
    case KILO_FLASH_ERROR_WRONG_BUILD: return "WRONG_BUILD";
    case KILO_FLASH_ERROR_BAD_RESET_CAUSE: return "BAD_RESET_CAUSE";
    case KILO_FLASH_ERROR_CONTROL_LOST: return "CONTROL_LOST";
    default: return "?";
  }
}

Flasher::Flasher(Serial& serial, Clock& clock, Gpio& gpio, SerialLink& link, Log& log)
    : serial_(serial), clock_(clock), gpio_(gpio), link_(link), log_(log) {}

bool Flasher::start(const FlashImage& img, bool persist) {
  if (busy() || !img.ready()) return false;
  image_ = &img;
  persist_ = persist;
  persist_pending_ = false;
  const BodyState& b = link_.state();
  prev_build_ = b.has_hello ? b.hello.fw_build : 0u;
  status_ = {};
  status_.pages = (uint16_t)img.pages();
  status_.build = img.manifest().build;
  uint64_t now = clock_.now_us();
  start_us_ = now;
  logf(log_, LogLevel::Info, "flash: build %08x, %u bytes, %u pages; the body ran %08x", (unsigned)status_.build,
       (unsigned)img.manifest().size, (unsigned)status_.pages, (unsigned)prev_build_);
  // The link stops here (owns_uart()): no heartbeat, goal or FEET goes out,
  // and the body is about to be reset anyway, so no REST first.
  if (!serial_.set_baud(KILO_HOST_FLASH_BAUD)) {
    step_ = Step::BootWait;  // busy() and owns the UART, so fail() restores the link
    fail(KILO_FLASH_ERROR_BAUD);
    return true;
  }
  begin_attempt(now);
  return true;
}

void Flasher::begin_attempt(uint64_t now) {
  status_.attempt++;
  status_.page = 0;
  status_.stk_cmd = 0;
  sync_tries_ = 0;
  gpio_.reset_assert();
  step_ = Step::ResetHold;
  step_at_us_ = now;
  set_phase(KILO_FLASH_PHASE_RESETTING);
}

void Flasher::poll() {
  if (!busy()) return;
  uint64_t now = clock_.now_us();
  switch (step_) {
    case Step::Idle:
      return;
    case Step::ResetHold:
      if (now - step_at_us_ < kResetUs) return;
      gpio_.reset_release();
      discard_rx();  // what the old firmware sent before the reset
      release_us_ = now;
      step_ = Step::BootWait;
      step_at_us_ = now;
      return;
    case Step::BootWait:
      discard_rx();
      if (now - step_at_us_ < kBootWaitUs) return;
      set_phase(KILO_FLASH_PHASE_SYNCING);
      send_sync(now);
      return;
    case Step::Sync: {
      if (!pump_tx()) return;
      uint8_t buf[32];
      for (;;) {
        int32_t n = serial_.read(buf, sizeof buf);
        if (n < 0) return fail(KILO_FLASH_ERROR_SERIAL);
        if (n == 0) break;
        // Anything before INSYNC OK is noise (the firmware's last bytes, a
        // glitch at reset): look for the pair.
        for (int32_t k = 0; k < n; k++) {
          if (rx_len_ == 1 && buf[k] == stk::kOk) {
            step_ = Step::SyncDrain;
            step_at_us_ = now;
            return;
          }
          rx_len_ = buf[k] == stk::kInsync ? 1u : 0u;
        }
      }
      if (now - step_at_us_ < (sync_tries_ == 1 ? kFirstSyncReplyUs : kSyncReplyUs)) return;
      if (now - release_us_ + kSyncReplyUs > kSyncWindowUs) {
        status_.stk_cmd = stk::kGetSync;
        return attempt_failed(KILO_FLASH_ERROR_SYNC_TIMEOUT, now);
      }
      send_sync(now);
      return;
    }
    case Step::SyncDrain:
      discard_rx();
      if (now - step_at_us_ < kDrainUs) return;
      logf(log_, LogLevel::Info, "flash: optiboot in sync %u ms after release (attempt %u)",
           (unsigned)((now - release_us_) / 1000u), (unsigned)status_.attempt);
      program_us_ = now;
      exchange(Part::EnterProgmode, 0, now);
      return;
    case Step::Command: {
      if (!pump_tx()) return;
      int32_t r = read_reply();
      if (r < 0) return;  // failed (read_reply said why)
      if (r == 0) {
        if (now - step_at_us_ >= kReplyUs) {
          status_.stk_cmd = tx_[0];
          attempt_failed(KILO_FLASH_ERROR_NO_REPLY, now);
        }
        return;
      }
      next_part(now);
      return;
    }
    case Step::LeaveSettle:
      discard_rx();
      if (now - step_at_us_ < kLeaveSettleUs) return;
      restart_link();
      step_ = Step::Booting;
      step_at_us_ = now;
      set_phase(KILO_FLASH_PHASE_BOOTING);
      return;
    case Step::Booting: {
      const BodyState& b = link_.state();
      if (b.has_hello && b.hello_at_us >= step_at_us_) {
        status_.fw_build = b.hello.fw_build;
        status_.reset_cause = b.hello.reset_cause;
        bool same = prev_build_ == status_.build;
        if (b.hello.fw_build != status_.build) return fail(KILO_FLASH_ERROR_WRONG_BUILD);
        if (b.hello.reset_cause != KILO_RESET_CAUSE_FLASHED &&
            !(same && b.hello.reset_cause == KILO_RESET_CAUSE_WARM_UNEXPECTED))
          return fail(KILO_FLASH_ERROR_BAD_RESET_CAUSE);
        return finish_ok();
      }
      if (now - step_at_us_ >= kHelloUs) fail(KILO_FLASH_ERROR_NO_HELLO);
      return;
    }
  }
}

// ---- STK500 ----------------------------------------------------------------------------

void Flasher::send_sync(uint64_t now) {
  discard_rx();  // a late, partial answer to the last one
  sync_tries_++;
  tx_[0] = stk::kGetSync;
  tx_[1] = stk::kCrcEop;
  tx_len_ = 2;
  tx_done_ = 0;
  rx_len_ = 0;
  step_ = Step::Sync;
  step_at_us_ = now;
}

// Build and send one command of the sequence; its answer is INSYNC,
// expect_data bytes, OK.
void Flasher::exchange(Part part, uint32_t expect_data, uint64_t now) {
  part_ = part;
  uint32_t addr = page_ * KILO_HOST_FLASH_PAGE / 2u;  // optiboot takes word addresses
  uint32_t n = 0;
  switch (part) {
    case Part::EnterProgmode:
      tx_[n++] = stk::kEnterProgmode;
      break;
    case Part::ReadSign:
      tx_[n++] = stk::kReadSign;
      break;
    case Part::ProgAddress:
    case Part::VerifyAddress:
      tx_[n++] = stk::kLoadAddress;
      tx_[n++] = (uint8_t)(addr & 0xFFu);
      tx_[n++] = (uint8_t)(addr >> 8);
      break;
    case Part::ProgPage:
      tx_[n++] = stk::kProgPage;
      tx_[n++] = (uint8_t)(KILO_HOST_FLASH_PAGE >> 8);
      tx_[n++] = (uint8_t)(KILO_HOST_FLASH_PAGE & 0xFFu);
      tx_[n++] = 'F';
      mem_copy(tx_ + n, image_->page(page_), KILO_HOST_FLASH_PAGE);
      n += KILO_HOST_FLASH_PAGE;
      break;
    case Part::ReadPage:
      tx_[n++] = stk::kReadPage;
      tx_[n++] = (uint8_t)(KILO_HOST_FLASH_PAGE >> 8);
      tx_[n++] = (uint8_t)(KILO_HOST_FLASH_PAGE & 0xFFu);
      tx_[n++] = 'F';
      break;
    case Part::Leave:
      tx_[n++] = stk::kLeaveProgmode;
      break;
  }
  tx_[n++] = stk::kCrcEop;
  tx_len_ = n;
  tx_done_ = 0;
  rx_len_ = 0;
  rx_want_ = 2 + expect_data;
  step_ = Step::Command;
  step_at_us_ = now;
}

// The current command was answered INSYNC .. OK: check it, go on.
void Flasher::next_part(uint64_t now) {
  uint32_t pages = image_->pages();
  switch (part_) {
    case Part::EnterProgmode:
      return exchange(Part::ReadSign, 3, now);
    case Part::ReadSign:
      if (rx_[1] != stk::kSignature[0] || rx_[2] != stk::kSignature[1] || rx_[3] != stk::kSignature[2]) {
        logf(log_, LogLevel::Error, "flash: signature %02x %02x %02x, not the ATmega328P's 1e 95 0f",
             (unsigned)rx_[1], (unsigned)rx_[2], (unsigned)rx_[3]);
        status_.stk_cmd = stk::kReadSign;
        return fail(KILO_FLASH_ERROR_BAD_SIGNATURE);  // the wrong chip: a retry will not help
      }
      page_ = 0;
      set_phase(KILO_FLASH_PHASE_PROGRAMMING);
      return exchange(Part::ProgAddress, 0, now);
    case Part::ProgAddress:
      return exchange(Part::ProgPage, 0, now);
    case Part::ProgPage:
      page_++;
      status_.page = (uint16_t)page_;
      changed();
      if (page_ < pages) return exchange(Part::ProgAddress, 0, now);
      logf(log_, LogLevel::Info, "flash: programmed %u pages in %u ms", (unsigned)pages,
           (unsigned)((now - program_us_) / 1000u));
      page_ = 0;
      status_.page = 0;
      set_phase(KILO_FLASH_PHASE_VERIFYING);
      return exchange(Part::VerifyAddress, 0, now);
    case Part::VerifyAddress:
      return exchange(Part::ReadPage, KILO_HOST_FLASH_PAGE, now);
    case Part::ReadPage:
      for (uint32_t k = 0; k < KILO_HOST_FLASH_PAGE; k++) {
        if (rx_[1 + k] != image_->page(page_)[k]) {
          logf(log_, LogLevel::Error, "flash: verify: byte 0x%04x reads %02x, programmed %02x",
               (unsigned)(page_ * KILO_HOST_FLASH_PAGE + k), (unsigned)rx_[1 + k],
               (unsigned)image_->page(page_)[k]);
          status_.stk_cmd = stk::kReadPage;
          return attempt_failed(KILO_FLASH_ERROR_VERIFY_MISMATCH, now);
        }
      }
      page_++;
      status_.page = (uint16_t)page_;
      changed();
      if (page_ < pages) return exchange(Part::VerifyAddress, 0, now);
      logf(log_, LogLevel::Info, "flash: verified %u pages, %u ms in all", (unsigned)pages,
           (unsigned)((now - program_us_) / 1000u));
      return exchange(Part::Leave, 0, now);
    case Part::Leave:
      // Optiboot resets into the new firmware: the UART back at the link
      // baud now, and what arrives while it boots is dropped.
      if (!serial_.set_baud(KILO_BAUD)) return fail(KILO_FLASH_ERROR_BAUD);
      step_ = Step::LeaveSettle;
      step_at_us_ = now;
      return;
  }
}

// Write what is left of the command. True once all of it went out.
bool Flasher::pump_tx() {
  if (tx_done_ < tx_len_) {
    int32_t w = serial_.write(tx_ + tx_done_, tx_len_ - tx_done_);
    if (w < 0) {
      fail(KILO_FLASH_ERROR_SERIAL);
      return false;
    }
    tx_done_ += (uint32_t)w;
  }
  return tx_done_ == tx_len_;
}

// Read toward rx_want_ bytes. 1: complete and good (INSYNC first, OK last);
// 0: not yet; -1: failed (attempt_failed or fail was called).
int32_t Flasher::read_reply() {
  while (rx_len_ < rx_want_) {
    int32_t n = serial_.read(rx_ + rx_len_, rx_want_ - rx_len_);
    if (n < 0) {
      fail(KILO_FLASH_ERROR_SERIAL);
      return -1;
    }
    if (n == 0) return 0;
    rx_len_ += (uint32_t)n;
    if (rx_[0] != stk::kInsync) break;  // out of sync: no use waiting for the rest
  }
  if (rx_[0] != stk::kInsync || rx_[rx_want_ - 1] != stk::kOk) {
    logf(log_, LogLevel::Warn, "flash: command %02x answered %02x .. %02x, not 14 .. 10", (unsigned)tx_[0],
         (unsigned)rx_[0], (unsigned)(rx_len_ >= rx_want_ ? rx_[rx_want_ - 1] : 0u));
    status_.stk_cmd = tx_[0];
    attempt_failed(KILO_FLASH_ERROR_BAD_REPLY, clock_.now_us());
    return -1;
  }
  return 1;
}

void Flasher::discard_rx() {
  uint8_t buf[64];
  while (serial_.read(buf, sizeof buf) > 0) {
  }
}

// ---- ends -----------------------------------------------------------------------------

void Flasher::attempt_failed(uint8_t error, uint64_t now) {
  logf(log_, LogLevel::Warn, "flash: attempt %u failed in %s: %s (command %02x)", (unsigned)status_.attempt,
       phase_name(status_.phase), flash_error_name(error), (unsigned)status_.stk_cmd);
  if (status_.attempt >= kAttempts) return fail(error);
  // A desynced optiboot has already reset into the firmware (or will, 1 s
  // after the last byte): a fresh reset starts it over.
  status_.error = error;  // the last attempt's, until the next one ends
  begin_attempt(now);
}

void Flasher::fail(uint8_t error) {
  if (!busy()) return;
  bool had_uart = owns_uart();
  Step step = step_;
  status_.error = error;
  step_ = Step::Idle;
  if (had_uart) {
    if (step == Step::ResetHold) gpio_.reset_release();  // never leave the body held in reset
    serial_.set_baud(KILO_BAUD);
    restart_link();
  }
  set_phase(KILO_FLASH_PHASE_FAILED);
  logf(log_, LogLevel::Error, "flash: FAILED: %s after %u attempt(s), %u ms; the link is back",
       flash_error_name(error), (unsigned)status_.attempt, (unsigned)((clock_.now_us() - start_us_) / 1000u));
}

void Flasher::finish_ok() {
  status_.error = KILO_FLASH_ERROR_NONE;
  step_ = Step::Idle;
  if (persist_) persist_pending_ = true;
  set_phase(KILO_FLASH_PHASE_DONE);
  logf(log_, LogLevel::Info, "flash: done in %u ms: HELLO build %08x reset %s",
       (unsigned)((clock_.now_us() - start_us_) / 1000u), (unsigned)status_.fw_build,
       reset_cause_name(status_.reset_cause));
}

void Flasher::restart_link() {
  // A frame delimiter first: if the firmware was running while we spoke
  // STK500 (no bootloader, or optiboot gave up), its decoder holds those
  // bytes, and the link's HELLO_REQ would arrive glued to them and fail
  // its CRC.
  const uint8_t delimiter = 0x00;
  serial_.write(&delimiter, 1);
  // As after the bridge: a fresh session (HELLO_REQ; goals resent once the
  // body is heard). The firmware's boot HELLO arrives unsolicited anyway.
  link_.disconnect();
  link_.connect();
}

void Flasher::set_phase(uint8_t phase) {
  status_.phase = phase;
  changed();
  if (phase != KILO_FLASH_PHASE_DONE && phase != KILO_FLASH_PHASE_FAILED)
    logf(log_, LogLevel::Debug, "flash: %s (attempt %u)", phase_name(phase), (unsigned)status_.attempt);
}

}  // namespace kilo
