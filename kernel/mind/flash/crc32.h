// crc32: the body image's checksum (FLASH_BEGIN.crc32, body.json "crc32"):
// CRC-32 as zlib.crc32 computes it (IEEE 802.3: reflected polynomial
// 0xEDB88320, initial value and final xor 0xFFFFFFFF), so the laptop's
// Python and the mind agree without sharing code. Check value: "123456789"
// -> 0xCBF43926 (protocol/vectors/host_frames.json "crc32").
//
// Bitwise, without a table: 1 KB less in the kernel, and 32 KB take well
// under a millisecond on the Pi 2's Cortex-A7, once per flash.
#ifndef KILO_MIND_FLASH_CRC32_H
#define KILO_MIND_FLASH_CRC32_H
#include <stdint.h>

namespace kilo {

class Crc32 {
 public:
  void update(const uint8_t* data, uint32_t len);
  uint32_t value() const { return ~state_; }
  void reset() { state_ = 0xFFFFFFFFu; }

 private:
  uint32_t state_ = 0xFFFFFFFFu;
};

// The CRC-32 of len bytes (zlib.crc32(data)).
uint32_t crc32(const uint8_t* data, uint32_t len);

}  // namespace kilo

#endif
