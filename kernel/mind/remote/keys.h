// keys: the Petoi IR remote's key table and the key map. A port of the
// table and ACTIONS of mind/kilo/ir.py. The body decodes the remote (NEC,
// address 0x00) and relays each key as EVENT IR_KEY, arg = the command byte
// | 0x100 for a held-key repeat.
//
// Keys are named by grid, as the owner reads the remote: columns A B C left
// to right, rows 1-7 bottom to top (B7 is the top middle). The legacy
// meaning (src/infrared.h) is kept for reference.
//
//   A7 0xA2 posture rest     B7 0x62 walk forward      C7 0xE2 balance on/off
//   A6 0x22 walk left        B6 0x02 posture stand     C6 0xC2 walk right
//   A5 0xE0 posture limp     B5 0xA8 walk backward     C5 0x90 slower
//   A4 0x68 walk march       B4 0x98 pattern crawl     C4 0xB0 pattern walk
//   A3 0x30 pattern trot     B3 0x18 posture sit       C3 0x7A trick stretch
//   A2 0x10 trick hi         B2 0x38 trick push_up     C2 0x5A trick pee
//   A1 0x42 trick wash_face  B1 0x4A trick butt_up     C1 0x52 faster
//
// There is no shutdown key (Python has none either).
#ifndef KILO_MIND_REMOTE_KEYS_H
#define KILO_MIND_REMOTE_KEYS_H
#include <stdint.h>

namespace kilo {
namespace remote {

constexpr uint16_t kRepeat = 0x100;

enum class Kind : uint8_t { Ignore, Posture, Trick, BalanceToggle, Walk, GaitPattern, Speed };

struct Key {
  uint8_t command;     // NEC command byte
  const char* grid;    // "B7"
  const char* legacy;  // what the legacy firmware did with it
  Kind kind;           // what the mind does (ir.py ACTIONS)
  const char* name;    // the posture, trick, direction, pattern, or "slower" / "faster"
};

constexpr int kKeys = 21;
extern const Key kKeyTable[kKeys];  // row by row from the top, as on the remote

const Key* key_for(uint8_t command);
// ir.py split(): the command byte and whether it is a held-key repeat.
inline uint8_t key_command(uint16_t arg) { return (uint8_t)(arg & 0xFF); }
inline bool key_repeat(uint16_t arg) { return (arg & kRepeat) != 0; }
// ir.py action_for(): the key's entry, or null for a repeat or a key of another remote.
const Key* action_for(uint16_t arg);
// "B3 (0x18, sit)", "B3 (0x18, sit) repeat", "? (0x45)" (ir.py describe(),
// which also prints the key's label; the grid names it here).
void describe(uint16_t arg, char* buf, uint32_t cap);

}  // namespace remote
}  // namespace kilo

#endif
