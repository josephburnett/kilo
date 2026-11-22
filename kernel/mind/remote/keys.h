// keys: the Petoi IR remote's key table and the key map. The body decodes the remote (NEC,
// address 0x00) and relays each key as EVENT IR_KEY, arg = the command byte
// | 0x100 for a held-key repeat.
//
// Keys are named by grid, as the owner reads the remote: columns A B C left
// to right, rows 1-7 bottom to top (B7 is the top middle). The owner's map
// of 2026-11-15 (remote.h says what each key does):
//
//   A7 0xA2 sleep (rest)      B7 0x62 faster            C7 0xE2 freeze
//   A6 0x22 turn left / steer B6 0x02 stand and stop    C6 0xC2 turn right / steer
//   A5 0xE0 behaviour         B5 0xA8 slower, then back C5 0x90 calibration posture
//   A4 0x68 trot              B4 0x98 crawl             C4 0xB0 walk
//   A2 0x10 rear up (trick rear_up, rear_up_2 or rear_up_3 by the stage;
//           again: down), only with the rear-up key on (below); B2 until
//           2026-11-17 (the owner: A2 matches the picture better)
//   B2 0x38 meow (Kind::Sound: one of the meow family, docs/sound.md;
//           the Mind plays it at once, past the remote's queue: it moves nothing)
//   A1 0x42 long trot (Kind::Gait "longtrot": the trot with a longer stride
//           built up a step at a time, three stages with B7 / B5; since
//           2026-11-21, docs/walking-and-balance.md §15)
//   B1 0x4A the righting experiment (Kind::Righting, docs/righting.md):
//           lying on a side or his back, he rights himself by a move planned
//           on the body model; refused (a log line) when he is not lying
//           there or the mind is built without KILO_RIGHTING=1; 2026-11-22.
//           The owner's experiment keys go along row 1 (A1 rear-up, B1
//           righting, C1 free), then row 2: never two on one key
//   A3 0x30, B3 0x18, C3 0x7A, C2 0x5A,
//   C1 0x52: nothing (logged as ignored)
//
// The rear-up key is OFF unless switched on (docs/rear-up-trick.md, the
// switch): until the owner has done the staged, hand-supported trials,
// A2 does nothing, like the other keys of rows 1-3. The switch is
// KILO_REAR_UP, which is also the stage (1, 2 or 3: how high Kilo goes;
// the owner tries them in order): make -C kernel KILO_REAR_UP=2
// (MindConfig::rear_up_key, which the Mind passes to set_rear_up_key());
// kilo-mind-host --rear-up[=N].
//
// Each key's legacy meaning (the original firmware's remote table) is kept
// for reference. There is no shutdown key.
#ifndef KILO_MIND_REMOTE_KEYS_H
#define KILO_MIND_REMOTE_KEYS_H
#include <stdint.h>

namespace kilo {
namespace remote {

constexpr uint16_t kRepeat = 0x100;

enum class Kind : uint8_t { Ignore, Posture, Gait, Speed, Turn, Freeze, Behaviour, Trick, Sound, Measure, Righting };

struct Key {
  uint8_t command;     // NEC command byte
  const char* grid;    // "B7"
  const char* legacy;  // what the legacy firmware did with it
  Kind kind;           // what the mind does
  const char* name;    // the posture, the gait pattern, the turn ("left"), "faster" / "slower", or the trick
  const char* label;   // what the key does, for the log ("stand", "faster", "none")
};

constexpr int kKeys = 21;
extern const Key kKeyTable[kKeys];  // row by row from the top, as on the remote

const Key* key_for(uint8_t command);
// The command byte and whether it is a held-key repeat.
inline uint8_t key_command(uint16_t arg) { return (uint8_t)(arg & 0xFF); }
inline bool key_repeat(uint16_t arg) { return (arg & kRepeat) != 0; }
// The key's entry (rows 1-3 too, Kind::Ignore but A1, B1, A2 and B2), or null
// for a held-key repeat or a key of another remote. With the rear-up key
// off, A2's entry is a Kind::Ignore one ("none").
const Key* action_for(uint16_t arg);
// The stage A2 starts, 1 rear_up
// (28 deg), 2 rear_up_2 (40 deg), 3 rear_up_3 (54 deg), or 0: A2 does
// nothing (the default). Anything above 3 is 0.
constexpr uint8_t kRearUpStages = 3;
void set_rear_up_key(uint8_t stage);
uint8_t rear_up_key();
// "B6 (0x02, stand)", "B6 (0x02, stand) repeat", "? (0x45)": a key for the log.
void describe(uint16_t arg, char* buf, uint32_t cap);

}  // namespace remote
}  // namespace kilo

#endif
