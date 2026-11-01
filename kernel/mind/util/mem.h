// mem: the only libc the core uses, through the compiler's builtins so that
// no libc header is needed. GCC inlines them or calls memcpy/memset,
// which Circle provides (circle/util.h), as does the host's libc.
#ifndef KILO_MIND_UTIL_MEM_H
#define KILO_MIND_UTIL_MEM_H
#include <stddef.h>

namespace kilo {

// n == 0 allows null pointers (memcpy itself does not: an empty payload is null)
inline void mem_copy(void* dst, const void* src, size_t n) {
  if (n != 0) __builtin_memcpy(dst, src, n);
}
inline void mem_zero(void* dst, size_t n) { __builtin_memset(dst, 0, n); }

}  // namespace kilo

#endif
