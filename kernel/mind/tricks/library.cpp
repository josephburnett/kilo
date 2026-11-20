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

unsigned rear_up_stage(const char* name) {
  if (str_eq(name, "rear_up")) return 1;
  if (str_eq(name, "rear_up_2")) return 2;
  if (str_eq(name, "rear_up_3")) return 3;
  return 0;
}

}  // namespace tricks
}  // namespace kilo
