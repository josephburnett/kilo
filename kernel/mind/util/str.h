// str: the string helpers the core needs, without libc's <string.h>.
//
// The parsers read the range [p, end) and return where they stopped, or
// null when the text is not what they parse; a null-terminated string can
// pass end = p + str_len(p).
#ifndef KILO_MIND_UTIL_STR_H
#define KILO_MIND_UTIL_STR_H
#include <stdint.h>

namespace kilo {

inline uint32_t str_len(const char* s) {
  uint32_t n = 0;
  while (s[n] != '\0') n++;
  return n;
}

inline bool str_eq(const char* a, const char* b) {
  if (a == nullptr || b == nullptr) return a == b;
  while (*a != '\0' && *a == *b) {
    a++;
    b++;
  }
  return *a == *b;
}

// strcmp: < 0, 0 or > 0 as a sorts before, with or after b (by unsigned char).
inline int str_cmp(const char* a, const char* b) {
  while (*a != '\0' && *a == *b) {
    a++;
    b++;
  }
  return (unsigned char)*a - (unsigned char)*b;
}

inline char ascii_lower(char c) { return c >= 'A' && c <= 'Z' ? (char)(c - 'A' + 'a') : c; }

// Equal but for ASCII case (pattern names, DNS names).
inline bool str_ieq(const char* a, const char* b) {
  while (*a != '\0' && ascii_lower(*a) == ascii_lower(*b)) {
    a++;
    b++;
  }
  return ascii_lower(*a) == ascii_lower(*b);
}

inline bool ends_with(const char* s, const char* tail) {
  uint32_t a = str_len(s), b = str_len(tail);
  return a >= b && str_eq(s + a - b, tail);
}

// [p, end) starts with lit: the text after it, else null.
inline const char* skip_lit(const char* p, const char* end, const char* lit) {
  for (; *lit != '\0'; lit++, p++)
    if (p >= end || *p != *lit) return nullptr;
  return p;
}

// An unsigned decimal at p (at least one digit, as many as follow) that fits
// 32 bits: its end, else null.
inline const char* parse_dec_u32(const char* p, const char* end, uint32_t* v) {
  if (p >= end || *p < '0' || *p > '9') return nullptr;
  uint64_t x = 0;
  while (p < end && *p >= '0' && *p <= '9') {
    x = x * 10u + (uint32_t)(*p++ - '0');
    if (x > 0xFFFFFFFFu) return nullptr;
  }
  *v = (uint32_t)x;
  return p;
}

// Exactly 8 hex digits at p (either case): their end, else null.
inline const char* parse_hex32(const char* p, const char* end, uint32_t* v) {
  uint32_t x = 0;
  for (int i = 0; i < 8; i++, p++) {
    if (p >= end) return nullptr;
    char c = ascii_lower(*p);
    uint32_t d;
    if (c >= '0' && c <= '9') d = (uint32_t)(c - '0');
    else if (c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
    else return nullptr;
    x = x << 4 | d;
  }
  *v = x;
  return p;
}

}  // namespace kilo

#endif
