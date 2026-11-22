// updater: the mind's own kernel image onto its SD card, from the laptop
// (the host channel's UPDATE_BEGIN, UPDATE_DATA, UPDATE_COMMIT; `kilo update
// --host`), so that the Pi boots it from the card at the next power-on with
// no network.
//
// The image is assembled in a RAM buffer the platform gives (its capacity is
// the largest image: 4 MB on the heap in the Circle kernel), checked at
// commit, then written through the platform's KernelStore (platform.h), a
// bounded slice per poll(), in this order:
//
//   CHECKING    all image_size bytes in, the CRC-32 (zlib's, flash/crc32.h)
//               UPDATE_BEGIN gave, and unless FORCE the marker
//               KILO_HOST_UPDATE_MARKER somewhere in the image: the mind
//               kernel embeds it (kernel/src/kernel.cpp), the netboot loader
//               and any other file do not, so a wrong file is not installed
//   WRITING     kernel7.new created, kSliceBytes per poll(), closed (on the
//               card)
//   VERIFYING   kernel7.new read back, kSliceBytes per poll(): its size and
//               CRC-32 must be the image's
//   INSTALLING  one poll(): kernel7.old and kernel7.bad removed if there
//               are any, kernel7.img renamed to kernel7.old (if there is
//               one), kernel7.new renamed to kernel7.img, kernel7.old
//               removed, sync: no previous kernel is kept on the card (the
//               owner, 2026-11-22: "This is all in Git"; a kernel that
//               misbehaves is replaced by rebuilding the wanted commit)
//   DONE        the platform reboots into it (HostServer: REST, then
//               reboot_requested())
//
// Any failure before the renames removes kernel7.new and leaves kernel7.img
// as it was (WRITE_FAILED, VERIFY_FAILED; INSTALL_FAILED if removing
// kernel7.old or the first rename fails). If the last rename fails, the old
// kernel is renamed back (INSTALL_FAILED); only if that fails too is the card
// left without a kernel7.img (CARD_BROKEN: rewrite it in a reader). Every
// step is logged, the slow ones with their time.
//
// The store's calls block (the SD card), so the platform's link must be
// paused while busy(): HostServer::link_paused() says so.
#ifndef KILO_MIND_UPDATE_UPDATER_H
#define KILO_MIND_UPDATE_UPDATER_H
#include <stdint.h>
#include "flash/crc32.h"
#include "platform.h"
#include "proto/kilo_protocol.h"

namespace kilo {

// File names on the SD card's FAT partition (config.txt boots kernel7.img).
constexpr const char* kKernelFile = "kernel7.img";
constexpr const char* kKernelNewFile = "kernel7.new";
constexpr const char* kKernelOldFile = "kernel7.old";  // the previous kernel, only during the swap
constexpr const char* kKernelBadFile = "kernel7.bad";  // what docs/laptop-setup.md once had the owner rename to

// KILO_UPDATE_ERROR_* by name, for logs and RESULT text ("?" if unknown).
const char* update_error_name(uint8_t error);

// Whether the marker KILO_HOST_UPDATE_MARKER is anywhere in data.
bool has_kernel_marker(const uint8_t* data, uint32_t len);

class KernelUpdater {
 public:
  // Store work per poll(): a slice the SD card writes or reads in some tens
  // of ms (the main loop waits that long; the link is paused anyway).
  static constexpr uint32_t kSliceBytes = 16384;
  static constexpr uint32_t kReadChunk = 2048;  // read back in this much per store call

  // What UPDATE_STATUS reports (KILO_UPDATE_PHASE_*, KILO_UPDATE_ERROR_*).
  struct Status {
    uint8_t phase;
    uint8_t error;
    uint32_t received;
    uint32_t size;
    uint32_t done;        // bytes written (WRITING) or read back (VERIFYING)
    uint32_t crc32;       // the received image's
    uint32_t card_crc32;  // kernel7.new's as read back, 0 until VERIFYING ends
  };

  // buffer: capacity bytes the updater owns from now on (the largest image).
  KernelUpdater(uint8_t* buffer, uint32_t capacity, KernelStore& store, Clock& clock, Log& log);

  uint32_t capacity() const { return capacity_; }

  // Start receiving an image of size bytes (UPDATE_BEGIN): RECEIVING, or
  // BAD_SIZE (0 or over capacity()) and FAILED. Drops what was received
  // before. Not while busy() (false: the caller refuses first).
  uint8_t begin(uint32_t size, uint32_t crc32, uint8_t flags);
  // The next len bytes at offset, which must be received() (in order) and
  // within size: else BAD_OFFSET, kept until commit() reports it.
  uint8_t write(uint32_t offset, const uint8_t* data, uint32_t len);
  // CHECKING: all bytes in, the CRC, the marker (unless FORCE). NONE: the
  // phase is CHECKING and busy(); poll() writes from the next pass. Else
  // FAILED with that error, and the image is dropped.
  uint8_t commit();
  // Forget a receiving image (the controller let go).
  void drop();
  // Advance one slice. Call every pass of the main loop while busy() (a
  // HostServer it is attached to does).
  void poll();

  bool receiving() const { return status_.phase == KILO_UPDATE_PHASE_RECEIVING; }
  // From commit() to DONE or FAILED: the store is in use, the link must pause.
  bool busy() const {
    return status_.phase >= KILO_UPDATE_PHASE_CHECKING && status_.phase <= KILO_UPDATE_PHASE_INSTALLING;
  }
  const Status& status() const { return status_; }
  // Bumped on every change of status(): a reporter sends when it moves.
  uint32_t changes() const { return changes_; }

 private:
  void step_write();
  void step_verify();
  void install();
  void fail(uint8_t error);
  void set_phase(uint8_t phase);
  void changed() { changes_++; }
  uint32_t ms_since(uint64_t t) const { return (uint32_t)((clock_.now_us() - t) / 1000u); }

  uint8_t* buf_;
  uint32_t capacity_;
  KernelStore& store_;
  Clock& clock_;
  Log& log_;

  Status status_ = {};
  uint32_t changes_ = 0;
  uint32_t want_crc_ = 0;   // UPDATE_BEGIN.crc32
  uint8_t flags_ = 0;
  uint8_t error_ = 0;       // a write() failed: commit() reports it
  Crc32 crc_;               // of what was received, then of what is read back
  bool open_ = false;       // the store has kernel7.new open
  bool started_ = false;    // the current phase has begun (created / opened)
  uint64_t phase_us_ = 0;   // the current phase began
  uint8_t chunk_[kReadChunk];
};

}  // namespace kilo

#endif
