#include "remote/keys.h"
#include "util/fmt.h"

namespace kilo {
namespace remote {

const Key kKeyTable[kKeys] = {
    {0xA2, "A7", "rest", Kind::Posture, "rest", "sleep"},
    {0x62, "B7", "forward", Kind::Speed, "faster", "faster"},
    {0xE2, "C7", "gyro toggle", Kind::Freeze, "", "freeze"},
    {0x22, "A6", "left", Kind::Turn, "left", "turn left"},
    {0x02, "B6", "stand", Kind::Posture, "stand", "stand"},
    {0xC2, "C6", "right", Kind::Turn, "right", "turn right"},
    {0xE0, "A5", "pause/limp", Kind::Behaviour, "", "behaviour"},
    {0xA8, "B5", "backward", Kind::Speed, "slower", "slower"},
    {0x90, "C5", "calibrate", Kind::Posture, "calib", "calibration posture"},
    {0x68, "A4", "stepping", Kind::Gait, "trot", "trot"},
    {0x98, "B4", "crawl", Kind::Gait, "crawl", "crawl"},
    {0xB0, "C4", "walk", Kind::Gait, "walk", "walk"},
    {0x30, "A3", "trot", Kind::Ignore, "", "none"},
    {0x18, "B3", "sit", Kind::Ignore, "", "none"},
    {0x7A, "C3", "stretch", Kind::Ignore, "", "none"},
    {0x10, "A2", "hi", Kind::Ignore, "", "none"},
    {0x38, "B2", "push-up", Kind::Trick, "rear_up", "rear up"},
    {0x5A, "C2", "pee", Kind::Ignore, "", "none"},
    {0x42, "A1", "wash face", Kind::Ignore, "", "none"},
    {0x4A, "B1", "butt-up", Kind::Ignore, "", "none"},
    {0x52, "C1", "last skill", Kind::Ignore, "", "none"},
};

namespace {
// B2 with the rear-up key off: a row 1-3 key that does nothing.
const Key kRearUpOff = {0x38, "B2", "push-up", Kind::Ignore, "", "none"};
bool g_rear_up_key = false;
}  // namespace

void set_rear_up_key(bool on) { g_rear_up_key = on; }
bool rear_up_key() { return g_rear_up_key; }

const Key* key_for(uint8_t command) {
  for (const Key& k : kKeyTable)
    if (k.command == command) return (k.kind == Kind::Trick && !g_rear_up_key) ? &kRearUpOff : &k;
  return nullptr;
}

const Key* action_for(uint16_t arg) {
  if (key_repeat(arg)) return nullptr;
  return key_for(key_command(arg));
}

void describe(uint16_t arg, char* buf, uint32_t cap) {
  uint8_t cmd = key_command(arg);
  const Key* k = key_for(cmd);
  const char* rep = key_repeat(arg) ? " repeat" : "";
  static const char kHex[] = "0123456789ABCDEF";  // ir.py prints 0x%02X; fmt has only %x
  char hex[3] = {kHex[cmd >> 4], kHex[cmd & 15], '\0'};
  if (k != nullptr) format(buf, cap, "%s (0x%s, %s)%s", k->grid, hex, k->label, rep);
  else format(buf, cap, "? (0x%s)%s", hex, rep);
}

}  // namespace remote
}  // namespace kilo
