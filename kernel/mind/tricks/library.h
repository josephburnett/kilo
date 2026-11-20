// library: the trick library compiled into the mind (library.gen.cpp, made
// by tricks/gen_library.py from mind/kilo/tricks/library/*.json). Python
// counterpart: kilo/tricks/library.py (names(), resolve(), load()).
#ifndef KILO_MIND_TRICKS_LIBRARY_H
#define KILO_MIND_TRICKS_LIBRARY_H
#include "tricks/library.gen.h"

namespace kilo {
namespace tricks {

// The trick named name: its Kilo name or its legacy token (library.resolve()).
// Null when the library has none.
const Trick* find_trick(const char* name);

// The rear-up stage a trick is (docs/rear-up-trick.md): rear_up 1,
// rear_up_2 2, rear_up_3 3; 0 for another trick.
unsigned rear_up_stage(const char* name);

}  // namespace tricks
}  // namespace kilo

#endif
