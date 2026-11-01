// names: printable names for message types and the spec's enums, for log
// lines. Python counterpart: the name dicts in kilo/link/consts.py (NAMES,
// EVENTS, ACK_STATUS, MODES, RESET_CAUSES, CONFIG_STATES), which come from
// the generated module. The C header has the message names only (its
// message table); the enum names are listed in names.cpp and
// kernel/host/tests/test_names.cpp checks them against
// protocol/gen/kilo_protocol.py.
//
// Every function returns a static string; unknown values give "?".
#ifndef KILO_MIND_LINK_NAMES_H
#define KILO_MIND_LINK_NAMES_H
#include <stdint.h>

namespace kilo {

const char* msg_name(uint8_t type);
const char* event_name(uint8_t code);
const char* ack_status_name(uint8_t status);
const char* mode_name(uint8_t mode);
const char* reset_cause_name(uint8_t cause);
const char* config_state_name(uint8_t state);
// A kilo_frame_status, in kilo/link/codec.py's error kinds where they exist
// ("bad_cobs", "bad_crc", "bad_length", "unknown_type", "overflow"); the C
// codec also tells a bad payload length apart: "bad_payload_len" (Python
// reports it as "bad_length" too).
const char* frame_status_name(uint8_t status);

}  // namespace kilo

#endif
