#include "link/names.h"
#include "proto/kilo_frame.h"
#include "proto/kilo_names.h"

namespace kilo {

// The spec's names, from the generated proto/kilo_names.h.
const char* msg_name(uint8_t type) { return kilo_msg_name(type); }
const char* event_name(uint8_t code) { return kilo_event_name(code); }
const char* ack_status_name(uint8_t status) { return kilo_ack_status_name(status); }
const char* mode_name(uint8_t mode) { return kilo_mode_name(mode); }
const char* reset_cause_name(uint8_t cause) { return kilo_reset_cause_name(cause); }
const char* config_state_name(uint8_t state) { return kilo_config_state_name(state); }

const char* frame_status_name(uint8_t status) {
  switch (status) {
    case KILO_FRAME_OK: return "ok";
    case KILO_FRAME_BAD_COBS: return "bad_cobs";
    case KILO_FRAME_BAD_CRC: return "bad_crc";
    case KILO_FRAME_BAD_LENGTH: return "bad_length";
    case KILO_FRAME_BAD_PAYLOAD_LEN: return "bad_payload_len";
    case KILO_FRAME_UNKNOWN_TYPE: return "unknown_type";
    case KILO_FRAME_OVERFLOW: return "overflow";
    default: return "?";
  }
}

}  // namespace kilo
