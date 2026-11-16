#include "record/store.h"
#include "util/mem.h"

namespace kilo {
namespace record {

void WalkStore::attach(uint8_t* storage, uint32_t bytes) {
  index_ = nullptr;
  ring_ = nullptr;
  cap_ = tail_ = used_ = head_ = n_ = 0;
  open_ = false;
  if (storage == nullptr || bytes < kIndexBytes + 2 * kEndReserve + kRecordMax) return;
  // the index first, aligned for its u64 fields
  uintptr_t a = ((uintptr_t)storage + 7u) & ~(uintptr_t)7u;
  uint32_t skip = (uint32_t)(a - (uintptr_t)storage);
  index_ = reinterpret_cast<WalkEntry*>(storage + skip);
  mem_zero(index_, kIndexBytes);
  ring_ = storage + skip + kIndexBytes;
  cap_ = (bytes - skip - kIndexBytes) & ~3u;
}

void WalkStore::drop_oldest() {
  WalkEntry& e = at(0);
  tail_ = wrap(tail_ + e.size);
  used_ -= e.size;
  e.id = 0;  // a reader holding the id finds nothing
  head_ = (head_ + 1) % kMaxWalks;
  n_--;
  dropped_walks_++;
}

uint32_t WalkStore::open(uint64_t start_us) {
  if (!ready()) return 0;
  if (open_) close();
  if (n_ == kMaxWalks) drop_oldest();
  WalkEntry& e = index_[(head_ + n_) % kMaxWalks];
  mem_zero(&e, sizeof e);
  e.id = next_id_++;
  e.begin = wrap(tail_ + used_);
  e.start_us = start_us;
  e.flags = KILO_WALK_FLAGS_RECORDING;
  if (n_ == 0) tail_ = e.begin;
  n_++;
  open_ = true;
  return e.id;
}

void WalkStore::close() {
  if (!open_) return;
  at(n_ - 1).flags &= (uint8_t)~KILO_WALK_FLAGS_RECORDING;
  open_ = false;
}

void WalkStore::put(uint32_t pos, const void* src, uint32_t n) {
  const uint8_t* s = static_cast<const uint8_t*>(src);
  uint32_t first = cap_ - pos < n ? cap_ - pos : n;
  mem_copy(ring_ + pos, s, first);
  mem_copy(ring_, s + first, n - first);
}

bool WalkStore::append(uint8_t kind, uint32_t t_us, const void* a, uint32_t na, const void* b, uint32_t nb,
                       bool essential) {
  if (!open_) return false;
  WalkEntry& cur = at(n_ - 1);
  uint32_t len = na + nb;
  if (len > 255) {
    dropped_records_++;
    return false;
  }
  uint32_t need = KILO_WALK_RECORD_HEADER_SIZE + len;
  uint32_t keep = essential ? 0 : kEndReserve;
  while (cap_ - used_ < need + keep && n_ > 1) drop_oldest();  // never the open walk itself
  if (cap_ - used_ < need + keep) {
    dropped_records_++;
    cur.flags |= KILO_WALK_FLAGS_TRUNCATED;
    return false;
  }
  kilo_walk_record_header_t h;
  h.kind = kind;
  h.len = (uint8_t)len;
  h.t_us = t_us;
  uint32_t pos = wrap(tail_ + used_);
  put(pos, &h, sizeof h);
  pos = wrap(pos + (uint32_t)sizeof h);
  if (na != 0) put(pos, a, na);
  pos = wrap(pos + na);
  if (nb != 0) put(pos, b, nb);
  used_ += need;
  cur.size += need;
  cur.records++;
  return true;
}

const WalkEntry* WalkStore::after(uint32_t after) const {
  for (uint32_t i = 0; i < n_; i++)
    if (at(i).id > after) return &at(i);  // ids grow with the index order
  return nullptr;
}

const WalkEntry* WalkStore::find(uint32_t id) const {
  if (id == 0) return nullptr;
  for (uint32_t i = 0; i < n_; i++)
    if (at(i).id == id) return &at(i);
  return nullptr;
}

int32_t WalkStore::read(uint32_t id, uint32_t offset, uint8_t* out, uint32_t n, uint32_t* total) const {
  const WalkEntry* e = find(id);
  if (e == nullptr) return -1;
  *total = e->size;
  if (offset >= e->size) return 0;
  if (n > e->size - offset) n = e->size - offset;
  uint32_t pos = wrap(e->begin + offset);
  uint32_t first = cap_ - pos < n ? cap_ - pos : n;
  mem_copy(out, ring_ + pos, first);
  mem_copy(out + first, ring_, n - first);
  return (int32_t)n;
}

}  // namespace record
}  // namespace kilo
