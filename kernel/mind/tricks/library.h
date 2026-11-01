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

}  // namespace tricks
}  // namespace kilo

#endif
