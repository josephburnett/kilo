#include "flash/crc32.h"

namespace kilo {

void Crc32::update(const uint8_t* data, uint32_t len) {
  uint32_t c = state_;
  for (uint32_t i = 0; i < len; i++) {
    c ^= data[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  state_ = c;
}

uint32_t crc32(const uint8_t* data, uint32_t len) {
  Crc32 c;
  c.update(data, len);
  return c.value();
}

}  // namespace kilo
