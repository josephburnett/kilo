// The message table (name per id) is in the generated header behind this
// switch, meant for host tools; the Pi has the room for it.
#define KILO_PROTOCOL_FIELD_TABLE
#include "link/names.h"
#include "proto/kilo_frame.h"
#include "proto/kilo_protocol.h"

namespace kilo {

namespace {

// Entry i is the name of value i (the enums below are dense from 0 or 1).
const char* pick(const char* const* names, uint32_t count, uint32_t value) {
  return value < count && names[value] != nullptr ? names[value] : "?";
}

#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

const char* const kEvents[] = {
    nullptr,           "BOOT",         "FELL",          "RECOVERED",     "RECOVERY_FAILED",
    "LOW_BATTERY",     "BATTERY_OK",   "WATCHDOG",      "LINK_RESTORED", "IMU_FAULT",
    "IMU_OK",          "TRICK_DONE",   "GAIT_STOPPED",  "GOAL_REFUSED",  "CONFIG_MIGRATED",
    "CONFIG_DEFAULTED", "EEPROM_SAVED", "TX_DROPPED",   "IR_KEY",        "FEET_UNDERRUN",
};
static_assert(COUNT(kEvents) == KILO_EVENT_FEET_UNDERRUN + 1, "event names != spec");

const char* const kAckStatus[] = {
    "OK",           "BAD_LEN",  "BAD_ARG",          "BUSY",           "UNKNOWN_TYPE", "CRC_MISMATCH",
    "NO_TRICK",     "REFUSED_LOW_BATT", "REFUSED_FALLEN", "EEPROM_BUSY", "STALE",        "UNCALIBRATED",
};
static_assert(COUNT(kAckStatus) == KILO_ACK_STATUS_UNCALIBRATED + 1, "ack status names != spec");

const char* const kModes[] = {
    "BOOT", "LIMP", "IDLE", "GAIT", "TRICK", "FALLEN", "RECOVERING", "LOW_BATT", "CALIB", "LINK_LOST",
};
static_assert(COUNT(kModes) == KILO_MODE_LINK_LOST + 1, "mode names != spec");

const char* const kResetCauses[] = {"COLD", "FLASHED", "REQUESTED", "WARM_UNEXPECTED"};
static_assert(COUNT(kResetCauses) == KILO_RESET_CAUSE_WARM_UNEXPECTED + 1, "reset cause names != spec");

const char* const kConfigStates[] = {"OK", "MIGRATED", "DEFAULTED"};
static_assert(COUNT(kConfigStates) == KILO_CONFIG_STATE_DEFAULTED + 1, "config state names != spec");

}  // namespace

const char* msg_name(uint8_t type) {
  for (uint32_t i = 0; i < KILO_MSG_TABLE_LEN; i++)
    if (kilo_msg_table[i].id == type) return kilo_msg_table[i].name;
  return "?";
}

const char* event_name(uint8_t code) { return pick(kEvents, COUNT(kEvents), code); }
const char* ack_status_name(uint8_t status) { return pick(kAckStatus, COUNT(kAckStatus), status); }
const char* mode_name(uint8_t mode) { return pick(kModes, COUNT(kModes), mode); }
const char* reset_cause_name(uint8_t cause) { return pick(kResetCauses, COUNT(kResetCauses), cause); }
const char* config_state_name(uint8_t state) { return pick(kConfigStates, COUNT(kConfigStates), state); }

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
