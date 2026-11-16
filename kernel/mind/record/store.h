// store: the walk recordings in RAM, a byte ring of whole walks and their
// index. The recorder (record/recorder.h) writes one walk at a time; the
// host server (net/host_server.h) lists them (WALK_LIST) and reads them out
// (WALK_FETCH) while the recorder goes on writing.
//
// The storage is one block the platform allocates once at start (the Circle
// kernel and kilo-mind-host: new[] of kStoreBytes, mind.h attach_recorder());
// the store carves its index from its start and keeps the rest as the ring
// of records. Nothing is allocated afterwards.
//
// Records are appended to the walk being recorded (open()...close()); each
// is a kilo_walk_record_header_t (kind, len, t_us) and len payload bytes,
// stored contiguously in ring order (a record may wrap the ring's end; a
// reader copies it out in two pieces). When a record does not fit:
// - the oldest walks that are not being recorded go, whole (dropped_walks);
// - if the walk being recorded is the only one left, the record is dropped
//   (dropped_records, the walk flagged TRUNCATED). kEndReserve bytes stay
//   free for essential records (the walk's end), so a walk that outgrows the
//   store keeps its start and its end.
// The index holds kMaxWalks walks; opening one more drops the oldest too.
// A reader that holds an id finds the walk gone (read() < 0) once dropped:
// never bytes of another walk.
#ifndef KILO_MIND_RECORD_STORE_H
#define KILO_MIND_RECORD_STORE_H
#include <stdint.h>
#include "proto/kilo_protocol.h"

namespace kilo {
namespace record {

struct WalkEntry {
  uint32_t id;          // 1.. in recording order
  uint32_t begin;       // where its first byte is in the ring
  uint32_t size;        // its bytes so far
  uint32_t records;
  uint32_t feet;        // FEET frames sent (the recorder counts them)
  uint64_t start_us;    // the mind's clock at the walk's setup
  uint64_t end_us;      // ... at its end; 0 until then
  uint8_t flags;        // KILO_WALK_FLAGS_*
  uint8_t text_len;
  char text[KILO_HOST_WALK_TEXT_MAX];  // WALK_INFO.text: "gait=.. dir=.. speed=.. end=.."
};

class WalkStore {
 public:
  static constexpr uint32_t kMaxWalks = 128;
  static constexpr uint32_t kIndexBytes = kMaxWalks * (uint32_t)sizeof(WalkEntry);
  static constexpr uint32_t kRecordMax = KILO_WALK_RECORD_HEADER_SIZE + 255;
  static constexpr uint32_t kEndReserve = 2048;  // free for essential records (a walk's end: a few MARK lines)

  // storage: bytes, at least kIndexBytes + a few KB; the ring is the rest
  // (rounded down to 4 bytes). Clears everything.
  void attach(uint8_t* storage, uint32_t bytes);
  bool ready() const { return ring_ != nullptr; }

  // Start a walk (a walk still open is closed first). Its id, 0 if not ready.
  uint32_t open(uint64_t start_us);
  // One record onto the open walk: the header, then a and b (b may be
  // null). essential: may use kEndReserve. False if dropped (no walk open, or
  // no room: counted).
  bool append(uint8_t kind, uint32_t t_us, const void* a, uint32_t na, const void* b = nullptr, uint32_t nb = 0,
              bool essential = false);
  // The open walk is complete: fetchable from now on.
  void close();
  // The walk being recorded, or null.
  WalkEntry* current() { return open_ ? &at(n_ - 1) : nullptr; }
  const WalkEntry* current() const { return open_ ? &at(n_ - 1) : nullptr; }

  // The oldest walk with an id above after (0: the oldest), or null.
  const WalkEntry* after(uint32_t after) const;
  const WalkEntry* find(uint32_t id) const;
  // Up to n bytes of walk id from offset into out: the count, 0 at its end,
  // -1 if the walk is not (or no longer) in the store. *total: its size.
  int32_t read(uint32_t id, uint32_t offset, uint8_t* out, uint32_t n, uint32_t* total) const;

  uint32_t capacity() const { return cap_; }
  uint32_t used() const { return used_; }
  uint32_t kept() const { return n_; }
  uint32_t recording_id() const { return open_ ? at(n_ - 1).id : 0; }
  uint32_t dropped_walks() const { return dropped_walks_; }
  uint32_t dropped_records() const { return dropped_records_; }

 private:
  WalkEntry& at(uint32_t i) { return index_[(head_ + i) % kMaxWalks]; }
  const WalkEntry& at(uint32_t i) const { return index_[(head_ + i) % kMaxWalks]; }
  uint32_t wrap(uint32_t pos) const { return pos >= cap_ ? pos - cap_ : pos; }
  void drop_oldest();
  void put(uint32_t pos, const void* src, uint32_t n);

  WalkEntry* index_ = nullptr;
  uint8_t* ring_ = nullptr;
  uint32_t cap_ = 0;
  uint32_t tail_ = 0;  // the oldest walk's first byte
  uint32_t used_ = 0;  // bytes from tail_ (every walk kept, the open one included)
  uint32_t head_ = 0;  // the oldest walk's index slot
  uint32_t n_ = 0;
  bool open_ = false;
  uint32_t next_id_ = 1;
  uint32_t dropped_walks_ = 0;
  uint32_t dropped_records_ = 0;
};

}  // namespace record
}  // namespace kilo

#endif
