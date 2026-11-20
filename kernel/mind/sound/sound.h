// sound: the sounds built into the image (docs/sound.md). kernel/tools/
// pack_sounds.py packs every kernel/sounds/NAME.wav into a generated table
// (the kernel's build/gen/sounds.gen.cpp, kilo-mind-host's too): name,
// sample rate, length and the samples, stored as 8-bit mu-law (G.711), one
// byte a sample: half the size of 16-bit PCM, with about 13 bits of
// dynamic range near silence, more than the Pi's PWM output resolves (about
// 12.5 bits at 44.1 kHz). Decoding is a 256-entry table (kUlaw below).
//
// Names: a-z, 0-9 and _, at most kNameMax characters. A name ending in
// digits is a variant of a family: meow1, meow2 and meow3 are the family
// "meow" (family()). Asking for "meow" picks one of them; asking for
// "meow2" plays that one (player.h).
#ifndef KILO_MIND_SOUND_SOUND_H
#define KILO_MIND_SOUND_SOUND_H
#include <stdint.h>

namespace kilo {
namespace sound {

constexpr uint32_t kNameMax = 15;  // SOUND.name, SOUND_INFO.name

struct Sound {
  const char* name;     // "meow1"
  uint32_t rate;        // Hz, as stored (8000 .. 22050)
  uint32_t samples;     // the length: samples / rate seconds
  const uint8_t* data;  // samples mu-law bytes
};

// The table the build packed (sounds.gen.cpp); kSoundCount may be 0 (no
// kernel/sounds/*.wav, or a build with KILO_SOUND=0: then kSounds is
// unused).
extern const Sound kSounds[];
extern const uint32_t kSoundCount;

// The family of a name: the name without its trailing digits ("meow2" ->
// "meow"; "mew" -> "mew"). Into out (kNameMax + 1 bytes).
void family(const char* name, char* out);
// name ends in a digit: a variant asked for by its own name.
bool is_variant(const char* name);
// A name the pack tool would take (a-z 0-9 _, 1..kNameMax).
bool valid_name(const char* name);

// G.711 mu-law: the byte to a 16-bit sample (ITU-T G.711, as Sun's
// g711.c ulaw2linear: 0xFF and 0x7F are 0, 0x80 is +32124, 0x00 -32124).
struct UlawTable {
  int16_t v[256];
};
constexpr UlawTable make_ulaw() {
  UlawTable t = {};
  for (int i = 0; i < 256; i++) {
    int u = ~i & 0xFF;
    int mag = (((u & 0x0F) << 3) + 0x84) << ((u & 0x70) >> 4);
    t.v[i] = (int16_t)((u & 0x80) ? 0x84 - mag : mag - 0x84);
  }
  return t;
}
constexpr UlawTable kUlaw = make_ulaw();

}  // namespace sound
}  // namespace kilo

#endif
