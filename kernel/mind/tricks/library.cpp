#include "tricks/library.h"
#include "util/str.h"

namespace kilo {
namespace tricks {

const Trick* find_trick(const char* name) {
  for (const Trick& t : kLibrary)
    if (str_eq(name, t.name)) return &t;
  for (const Trick& t : kLibrary)
    if (t.legacy[0] != '\0' && str_eq(name, t.legacy)) return &t;
  return nullptr;
}

}  // namespace tricks
}  // namespace kilo
