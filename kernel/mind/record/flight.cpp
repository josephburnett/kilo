#include "record/flight.h"
#include "util/mem.h"

namespace kilo {
namespace record {

void FlightStore::attach(uint8_t* storage, uint32_t bytes) {
  mem_ = storage;
  slots_ = storage != nullptr ? bytes / kBlockBytes : 0;
  if (slots_ < 2) slots_ = 0;
  next_ = count_ = dropped_ = records_ = skipped_ = 0;
  open_ = false;
}

kilo_record_block_header_t* FlightStore::header(uint32_t seq) const {
  return reinterpret_cast<kilo_record_block_header_t*>(slot(seq));
}

const uint8_t* FlightStore::block(uint32_t seq) const { return has(seq) ? slot(seq) : nullptr; }

uint32_t FlightStore::used(uint32_t seq) const {
  if (!has(seq)) return 0;
  uint32_t u;
  mem_copy(&u, slot(seq) + 12, 4);  // header.used (the header is packed: copy, not cast)
  return u;
}

uint64_t FlightStore::t0(uint32_t seq) const {
  if (!has(seq)) return 0;
  kilo_record_block_header_t h;
  mem_copy(&h, slot(seq), sizeof h);
  return ((uint64_t)h.t0_us_hi << 32) | h.t0_us_lo;
}

uint32_t FlightStore::seq_at(uint64_t t_us) const {
  uint32_t lo = first_seq(), hi = last_seq();  // binary search: t0 grows with seq
  if (t0(lo) > t_us) return lo;
  while (lo < hi) {
    uint32_t mid = lo + (hi - lo + 1) / 2;
    if (t0(mid) <= t_us) lo = mid;
    else hi = mid - 1;
  }
  return lo;
}

uint32_t FlightStore::used_bytes() const {
  if (count_ == 0) return 0;
  uint32_t n = (count_ - 1) * kBlockBytes;  // sealed blocks count whole: their slot is taken
  return n + used(last_seq());
}

void FlightStore::open_block(uint64_t now_us) {
  if (count_ == slots_) {
    count_--;  // the oldest block goes
    dropped_++;
  }
  uint32_t seq = next_++;
  count_++;
  kilo_record_block_header_t h;
  mem_zero(&h, sizeof h);
  h.magic = kBlockMagic;
  h.boot = boot_;
  h.seq = seq;
  h.used = kBlockHeader;
  h.t0_us_lo = (uint32_t)now_us;
  h.t0_us_hi = (uint32_t)(now_us >> 32);
  mem_copy(slot(seq), &h, sizeof h);
  t0_ = now_us;
  crc_.reset();
  open_ = true;
}

void FlightStore::seal() {
  if (!open_) return;
  uint32_t seq = next_ - 1;
  uint8_t* b = slot(seq);
  kilo_record_block_header_t h;
  mem_copy(&h, b, sizeof h);
  h.crc32 = crc_.value();  // over b + kBlockHeader .. b + h.used, kept as the records came
  if (h.crc32 == 0) h.crc32 = 1;  // 0 means "open"; a 1-in-4e9 CRC of 0 reads as 1 (the laptop knows)
  mem_copy(b, &h, sizeof h);
  open_ = false;
}

void FlightStore::set_boot(uint32_t boot) {
  boot_ = boot;
  for (uint32_t s = first_seq(); count_ != 0 && s < next_; s++) mem_copy(slot(s) + 4, &boot_, 4);
}

bool FlightStore::append(uint8_t kind, uint64_t now_us, const void* a, uint32_t na, const void* b, uint32_t nb) {
  if (!ready() || na + nb > 255) return false;
  uint32_t need = kRecordHeader + na + nb;
  if (open_) {
    uint32_t u = used(next_ - 1);
    if (u + need > kBlockBytes || now_us < t0_ || now_us - t0_ > kMaxSpanUs) seal();
  }
  if (!open_) open_block(now_us);
  uint32_t seq = next_ - 1;
  uint8_t* blk = slot(seq);
  kilo_record_block_header_t h;
  mem_copy(&h, blk, sizeof h);
  kilo_walk_record_header_t r;
  r.kind = kind;
  r.len = (uint8_t)(na + nb);
  r.t_us = (uint32_t)(now_us - t0_);
  uint8_t* p = blk + h.used;
  mem_copy(p, &r, sizeof r);
  if (na != 0) mem_copy(p + sizeof r, a, na);
  if (nb != 0) mem_copy(p + sizeof r + na, b, nb);
  crc_.update(p, need);
  h.used += need;
  h.records++;
  mem_copy(blk, &h, sizeof h);
  records_++;
  return true;
}

void FlightStore::count_skipped(uint64_t now_us) {
  skipped_++;
  if (!ready()) return;
  if (!open_) open_block(now_us);
  uint8_t* blk = slot(next_ - 1);
  uint32_t n;
  mem_copy(&n, blk + 28, 4);  // header.skipped
  n++;
  mem_copy(blk + 28, &n, 4);
}

}  // namespace record
}  // namespace kilo
