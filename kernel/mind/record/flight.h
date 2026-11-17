// flight: the flight recorder's RAM store, a ring of fixed-size blocks
// (protocol/spec.toml "The flight recorder", kernel/README.md). Every record
// the recorder makes (record/recorder.h: link frames, log lines, a walk's
// plans and marks, a trick's marks, the RUN marks that delimit runs) is
// appended to the open block; a block that cannot take the next record is
// sealed (its CRC-32 computed) and the next slot opened, overwriting the
// oldest block when the ring is full. Nothing is allocated after attach().
//
// A block is self-contained: a kilo_record_block_header_t (its seq, the
// mind's clock at its start, its length, its CRC) and records whose t_us
// count from the block's t0. So a reader (the host server's fetch, the card
// saver, the laptop) can start at any block, a torn block is recognised by
// its CRC, and a range of time is a range of block numbers. A record never
// spans two blocks. A block also closes when its t_us would pass
// kMaxSpanUs (u32 microseconds), so a long silence costs nothing.
//
// Cost: an append is a header, a memcpy of at most 255 bytes and the
// block's running CRC-32 over them; sealing a block writes the CRC.
#ifndef KILO_MIND_RECORD_FLIGHT_H
#define KILO_MIND_RECORD_FLIGHT_H
#include <stdint.h>
#include "flash/crc32.h"
#include "proto/kilo_protocol.h"

namespace kilo {
namespace record {

constexpr uint32_t kBlockBytes = KILO_HOST_RECORD_BLOCK_BYTES;
constexpr uint32_t kBlockHeader = KILO_RECORD_BLOCK_HEADER_SIZE;
constexpr uint32_t kRecordHeader = KILO_WALK_RECORD_HEADER_SIZE;
constexpr uint32_t kRecordMax = kRecordHeader + 255;
constexpr uint32_t kBlockMagic = KILO_HOST_RECORD_BLOCK_MAGIC;

class FlightStore {
 public:
  static constexpr uint64_t kMaxSpanUs = 0x7FFFFFFFull;  // a block's records span at most this (35 min)

  // storage: whole blocks (bytes / kBlockBytes of them, at least 2). Clears.
  void attach(uint8_t* storage, uint32_t bytes);
  bool ready() const { return slots_ >= 2; }

  // One record at the mind's clock now_us: the header, then a and b (b may
  // be null). False if not ready or the payload is over 255 bytes.
  bool append(uint8_t kind, uint64_t now_us, const void* a, uint32_t na, const void* b = nullptr, uint32_t nb = 0);
  // A frame counted, not stored (HEARTBEAT): in the open block's skipped.
  void count_skipped(uint64_t now_us);
  // Seal the open block now (the card saver: a run's tail goes to the card
  // with it); the next record opens a new one. Nothing when none is open.
  void seal();
  // The boot's number on the card, into every block's header from now on
  // and into those in RAM (the CRC covers the records only).
  void set_boot(uint32_t boot);

  // Blocks: seqs first_seq() .. last_seq() are in RAM (none when empty());
  // the last is the open one when open() is true.
  bool empty() const { return count_ == 0; }
  uint32_t first_seq() const { return next_ - count_; }
  uint32_t last_seq() const { return next_ - 1; }
  uint32_t next_seq() const { return next_; }  // the seq the next block opened gets
  bool open() const { return open_; }
  bool has(uint32_t seq) const { return count_ != 0 && seq >= first_seq() && seq < next_; }
  // The block's bytes (header first) and its length so far; null if not in RAM.
  const uint8_t* block(uint32_t seq) const;
  uint32_t used(uint32_t seq) const;
  bool sealed(uint32_t seq) const { return has(seq) && !(open_ && seq == next_ - 1); }
  uint64_t t0(uint32_t seq) const;
  // The newest block in RAM that starts at or before t_us (the first if
  // none does). Only when !empty().
  uint32_t seq_at(uint64_t t_us) const;

  uint32_t capacity() const { return slots_ * kBlockBytes; }
  uint32_t blocks() const { return count_; }
  uint32_t used_bytes() const;
  uint32_t dropped_blocks() const { return dropped_; }
  uint32_t records() const { return records_; }
  uint32_t skipped() const { return skipped_; }
  uint32_t boot() const { return boot_; }

 private:
  uint8_t* slot(uint32_t seq) const { return mem_ + (seq % slots_) * kBlockBytes; }
  kilo_record_block_header_t* header(uint32_t seq) const;
  void open_block(uint64_t now_us);

  uint8_t* mem_ = nullptr;
  uint32_t slots_ = 0;
  uint32_t next_ = 0;   // seq of the next block to open
  uint32_t count_ = 0;  // blocks in RAM
  bool open_ = false;
  uint64_t t0_ = 0;     // the open block's
  Crc32 crc_;           // ... over its records so far
  uint32_t dropped_ = 0, records_ = 0, skipped_ = 0;
  uint32_t boot_ = 0;
};

}  // namespace record
}  // namespace kilo

#endif
