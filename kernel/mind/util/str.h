// str: the string helpers the core needs, without libc's <string.h>.
#ifndef KILO_MIND_UTIL_STR_H
#define KILO_MIND_UTIL_STR_H

namespace kilo {

inline bool str_eq(const char* a, const char* b) {
  if (a == nullptr || b == nullptr) return a == b;
  while (*a != '\0' && *a == *b) {
    a++;
    b++;
  }
  return *a == *b;
}

}  // namespace kilo

#endif
