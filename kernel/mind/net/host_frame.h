// host_frame: the laptop channel's framing (protocol/spec.toml [host]):
//
//     len u16 LE | type u8 | seq u8 | payload (0..KILO_HOST_MAX_PAYLOAD) | crc16 u16 LE
//
// len counts the bytes after itself; crc16 is the body link's CRC-16/CCITT-
// FALSE (proto/crc16.h) over type, seq, payload. TCP is reliable and ordered,
// so there is no COBS and no delimiter; a bad length or CRC means the stream
// is out of step and the connection is closed. Python counterpart:
// build_host_frame and HostStreamDecoder in protocol/py/kilo_link_codec.py.
#ifndef KILO_MIND_NET_HOST_FRAME_H
#define KILO_MIND_NET_HOST_FRAME_H
#include <stdint.h>
#include "proto/kilo_protocol.h"

namespace kilo {

constexpr uint32_t kHostOverhead = 6;  // len, type, seq, crc
static_assert(KILO_HOST_MAX_FRAME == KILO_HOST_MAX_PAYLOAD + kHostOverhead, "spec host.max_frame");

struct HostFrame {
  uint8_t type;
  uint8_t seq;
  uint16_t len;  // payload bytes
  uint8_t payload[KILO_HOST_MAX_PAYLOAD];
};

// Write one frame to out (room for len + kHostOverhead bytes). Returns the
// frame's size, 0 when len > KILO_HOST_MAX_PAYLOAD.
uint32_t host_frame_encode(uint8_t type, uint8_t seq, const uint8_t* payload, uint32_t len, uint8_t* out);

enum class HostRxStatus : uint8_t {
  None,       // need more bytes
  Frame,      // frame() holds a complete frame with a good CRC (type and length unchecked)
  BadLength,  // len outside 4..KILO_HOST_MAX_FRAME - 2: out of step
  BadCrc,     // out of step
};

// A byte-at-a-time receiver (no buffer shuffling). After BadLength or BadCrc
// it stays failed until reset(): the connection is to be closed.
class HostRx {
 public:
  void reset();
  HostRxStatus push(uint8_t byte);
  const HostFrame& frame() const { return frame_; }
  bool failed() const { return failed_; }

 private:
  uint32_t have_ = 0;  // bytes of the current frame so far, len included
  uint16_t need_ = 0;  // the frame's len field (bytes after it)
  uint8_t buf_[KILO_HOST_MAX_FRAME] = {};
  bool failed_ = false;
  HostFrame frame_ = {};
};

}  // namespace kilo

#endif
