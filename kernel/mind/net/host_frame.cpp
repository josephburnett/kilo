#include "net/host_frame.h"
#include "proto/crc16.h"
#include "util/mem.h"

namespace kilo {

uint32_t host_frame_encode(uint8_t type, uint8_t seq, const uint8_t* payload, uint32_t len, uint8_t* out) {
  if (len > KILO_HOST_MAX_PAYLOAD) return 0;
  uint32_t n = len + 4;  // type, seq, payload, crc
  out[0] = (uint8_t)(n & 0xFFu);
  out[1] = (uint8_t)(n >> 8);
  out[2] = type;
  out[3] = seq;
  mem_copy(out + 4, payload, len);
  uint16_t crc = kilo_crc16(out + 2, (uint16_t)(len + 2));
  out[4 + len] = (uint8_t)(crc & 0xFFu);
  out[5 + len] = (uint8_t)(crc >> 8);
  return len + kHostOverhead;
}

void HostRx::reset() {
  have_ = 0;
  need_ = 0;
  failed_ = false;
}

HostRxStatus HostRx::push(uint8_t byte) {
  if (failed_) return HostRxStatus::BadLength;
  buf_[have_++] = byte;
  if (have_ == 2) {
    need_ = (uint16_t)(buf_[0] | (buf_[1] << 8));
    if (need_ < 4 || need_ > KILO_HOST_MAX_FRAME - 2) {
      failed_ = true;
      return HostRxStatus::BadLength;
    }
    return HostRxStatus::None;
  }
  if (have_ < 2 || have_ < 2u + need_) return HostRxStatus::None;
  have_ = 0;
  uint16_t body = (uint16_t)(need_ - 2);  // type, seq, payload
  uint16_t rx = (uint16_t)(buf_[2 + body] | (buf_[3 + body] << 8));
  if (kilo_crc16(buf_ + 2, body) != rx) {
    failed_ = true;
    return HostRxStatus::BadCrc;
  }
  frame_.type = buf_[2];
  frame_.seq = buf_[3];
  frame_.len = (uint16_t)(body - 2);
  mem_copy(frame_.payload, buf_ + 4, frame_.len);
  return HostRxStatus::Frame;
}

}  // namespace kilo
