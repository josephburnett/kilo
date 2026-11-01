// flasher: program the body's ATmega328P through its optiboot bootloader
// over the body link's UART (docs/fast-boot.md §6, phase P4), as `kilo
// flash` does on Linux with avrdude -c arduino (mind/kilo/tools/flash.py).
// One cooperative, non-blocking state machine driven by poll() from the
// mind's main loop, like the link.
//
// The sequence (timings in flasher.cpp, with where they come from):
//  1. Take the UART: while owns_uart() the main loop must not poll() the
//     link or stream FEET (the walker); the body's watchdog covers the gap
//     until the reset. set_baud(host.flash_baud, 115200).
//  2. RESET: Gpio::reset_assert() for link.reset_pulse_ms (100 ms), then
//     reset_release() (the emulated open drain of mind/kilo/tools/gpio.py).
//  3. STK500v1 as optiboot speaks it (optiboot.c): GET_SYNC until it answers
//     INSYNC OK (it listens about 1 s after an external reset), then
//     ENTER_PROGMODE, READ_SIGN (must be the 328P's 1E 95 0F), per 128-byte
//     page LOAD_ADDRESS (word address) + PROG_PAGE, then per page
//     LOAD_ADDRESS + READ_PAGE compared with the image, LEAVE_PROGMODE.
//     Optiboot resets into the new firmware 16 ms later.
//  4. set_baud(link baud), restart the link (disconnect + connect, as after
//     the bridge), and wait for the new firmware's HELLO: fw_build must be
//     the image's build and reset_cause FLASHED (WARM_UNEXPECTED is also
//     taken when the body already ran this build: an identical image
//     leaves the body's reset record unchanged, docs/architecture.md §3a).
//     The link is the main loop's again from here (owns_uart() false).
//
// An attempt that fails in step 3 (no sync, no or a bad answer, a verify
// mismatch) is retried once from a fresh reset, as `kilo flash` retries
// avrdude: optiboot survives an interrupted flash. Any failure ends with
// the UART back at the link baud and the link restarted.
#ifndef KILO_MIND_FLASH_FLASHER_H
#define KILO_MIND_FLASH_FLASHER_H
#include <stdint.h>
#include "flash/image.h"
#include "link/serial_link.h"
#include "platform.h"

namespace kilo {

// STK500v1, the subset optiboot implements (avrdude's stk500.h names).
namespace stk {
constexpr uint8_t kOk = 0x10;
constexpr uint8_t kInsync = 0x14;
constexpr uint8_t kCrcEop = 0x20;  // ends every command
constexpr uint8_t kGetSync = 0x30;
constexpr uint8_t kEnterProgmode = 0x50;
constexpr uint8_t kLeaveProgmode = 0x51;
constexpr uint8_t kLoadAddress = 0x55;  // + address lo, hi: a word address
constexpr uint8_t kProgPage = 0x64;     // + length hi, lo, memtype 'F', data
constexpr uint8_t kReadPage = 0x74;     // + length hi, lo, memtype 'F'
constexpr uint8_t kReadSign = 0x75;
constexpr uint8_t kSignature[3] = {0x1E, 0x95, 0x0F};  // ATmega328P
}  // namespace stk

// KILO_FLASH_ERROR_* by name, for logs ("?" if unknown).
const char* flash_error_name(uint8_t error);

class Flasher {
 public:
  static constexpr uint8_t kAttempts = 2;

  // What FLASH_STATUS reports (KILO_FLASH_PHASE_*, KILO_FLASH_ERROR_*).
  struct Status {
    uint8_t phase;
    uint8_t error;
    uint8_t attempt;
    uint8_t stk_cmd;   // the STK500 command that failed
    uint16_t page;     // pages done in PROGRAMMING / VERIFYING
    uint16_t pages;
    uint32_t build;    // the image's
    uint32_t fw_build; // HELLO after the flash, 0 = none yet
    uint8_t reset_cause;
  };

  Flasher(Serial& serial, Clock& clock, Gpio& gpio, SerialLink& link, Log& log);

  // Flash img (ready(); it must stay unchanged until busy() is false).
  // persist: after DONE, persist_pending() asks the platform to write it to
  // SD. False if busy or the image is not ready.
  bool start(const FlashImage& img, bool persist);
  // Advance; never blocks. Call every pass of the main loop while busy()
  // (a HostServer it is attached to does).
  void poll();

  // From start() to DONE or FAILED.
  bool busy() const { return step_ != Step::Idle; }
  // The UART is the flasher's: do not poll() the link or send on it.
  bool owns_uart() const { return busy() && step_ != Step::Booting; }
  const Status& status() const { return status_; }
  // Bumped on every change of status(): a reporter sends when it moves.
  uint32_t changes() const { return changes_; }

  // DONE with persist: write image() to SD (body.bin, body.json), then
  // persist_done(). The core does no file I/O. Until then HostServer
  // refuses a new FLASH_BEGIN, so the image cannot change under the write.
  bool persist_pending() const { return persist_pending_; }
  void persist_done() { persist_pending_ = false; }
  const FlashImage* image() const { return image_; }

 private:
  enum class Step : uint8_t {
    Idle,
    ResetHold,   // RESET low
    BootWait,    // released: optiboot starting
    Sync,        // GET_SYNC sent, waiting
    SyncDrain,   // in sync: let late answers to earlier GET_SYNCs arrive and drop them
    Command,     // an STK500 exchange in flight (cmd_ says which part of the sequence)
    LeaveSettle, // LEAVE_PROGMODE answered: optiboot jumps to the firmware
    Booting,     // the link has the UART: waiting for the HELLO
  };
  enum class Part : uint8_t { EnterProgmode, ReadSign, ProgAddress, ProgPage, VerifyAddress, ReadPage, Leave };

  void begin_attempt(uint64_t now);
  void send_sync(uint64_t now);
  void exchange(Part part, uint32_t expect_data, uint64_t now);
  void next_part(uint64_t now);
  bool pump_tx();
  int32_t read_reply();
  void discard_rx();
  void attempt_failed(uint8_t error, uint64_t now);
  void fail(uint8_t error);
  void finish_ok();
  void restart_link();
  void set_phase(uint8_t phase);
  void changed() { changes_++; }

  Serial& serial_;
  Clock& clock_;
  Gpio& gpio_;
  SerialLink& link_;
  Log& log_;

  const FlashImage* image_ = nullptr;
  bool persist_ = false;
  bool persist_pending_ = false;
  uint32_t prev_build_ = 0;  // HELLO.fw_build before the flash, 0 = none

  Step step_ = Step::Idle;
  Part part_ = Part::EnterProgmode;
  Status status_ = {};
  uint32_t changes_ = 0;

  uint64_t start_us_ = 0;     // start()
  uint64_t step_at_us_ = 0;   // the current step began (reset, release, send, connect)
  uint64_t release_us_ = 0;   // RESET released in this attempt
  uint64_t program_us_ = 0;   // in sync: programming began

  uint8_t tx_[5 + KILO_HOST_FLASH_PAGE];  // the longest command: PROG_PAGE
  uint32_t tx_len_ = 0;
  uint32_t tx_done_ = 0;
  uint8_t rx_[2 + KILO_HOST_FLASH_PAGE];  // the longest answer: READ_PAGE's
  uint32_t rx_len_ = 0;
  uint32_t rx_want_ = 0;
  uint32_t page_ = 0;
  uint32_t sync_tries_ = 0;  // GET_SYNCs sent in this attempt
};

}  // namespace kilo

#endif
