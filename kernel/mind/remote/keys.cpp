#include "remote/keys.h"
#include "util/fmt.h"

namespace kilo {
namespace remote {

const Key kKeyTable[kKeys] = {
    {0xA2, "A7", "rest", Kind::Posture, "rest"},
    {0x62, "B7", "forward", Kind::Walk, "forward"},
    {0xE2, "C7", "gyro toggle", Kind::BalanceToggle, ""},
    {0x22, "A6", "left", Kind::Walk, "left"},
    {0x02, "B6", "stand", Kind::Posture, "stand"},
    {0xC2, "C6", "right", Kind::Walk, "right"},
    {0xE0, "A5", "pause/limp", Kind::Posture, "limp"},
    {0xA8, "B5", "backward", Kind::Walk, "backward"},
    {0x90, "C5", "calibrate", Kind::Speed, "slower"},
    {0x68, "A4", "stepping", Kind::Walk, "march"},
    {0x98, "B4", "crawl", Kind::GaitPattern, "crawl"},
    {0xB0, "C4", "walk", Kind::GaitPattern, "walk"},
    {0x30, "A3", "trot", Kind::GaitPattern, "trot"},
    {0x18, "B3", "sit", Kind::Posture, "sit"},
    {0x7A, "C3", "stretch", Kind::Trick, "stretch"},
    {0x10, "A2", "hi", Kind::Trick, "hi"},
    {0x38, "B2", "push-up", Kind::Trick, "push_up"},
    {0x5A, "C2", "pee", Kind::Trick, "pee"},
    {0x42, "A1", "wash face", Kind::Trick, "wash_face"},
    {0x4A, "B1", "butt-up", Kind::Trick, "butt_up"},
    {0x52, "C1", "last skill", Kind::Speed, "faster"},
};

const Key* key_for(uint8_t command) {
  for (const Key& k : kKeyTable)
    if (k.command == command) return &k;
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
  if (k != nullptr) format(buf, cap, "%s (0x%s, %s)%s", k->grid, hex, k->legacy, rep);
  else format(buf, cap, "? (0x%s)%s", hex, rep);
}

}  // namespace remote
}  // namespace kilo
