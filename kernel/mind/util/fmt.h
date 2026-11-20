// fmt: a small printf for log lines (the core has no libc printf).
//
// Conversions: %d (int), %u and %x (unsigned), %s (const char*; null
// prints "(null)"), %c, %%. Each may have a width and a '0' flag ("%08x",
// "%5d"); numbers pad with spaces, or with zeros after the sign under '0'.
// No length modifiers, precision, '-' or floats: cast wider values to int or
// unsigned (32 bits on every target the mind runs on). The printf format
// attribute makes GCC check the arguments as for printf. An unknown
// conversion prints '?'.
#ifndef KILO_MIND_UTIL_FMT_H
#define KILO_MIND_UTIL_FMT_H
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

namespace kilo {

static_assert(sizeof(int) == 4, "fmt: int is 32 bits");

// Format into buf (cap >= 1): always null-terminated, cut to fit.
// Returns the length written, without the null.
size_t vformat(char* buf, size_t cap, const char* fmt, va_list ap);
size_t format(char* buf, size_t cap, const char* fmt, ...) __attribute__((format(printf, 3, 4)));

// The fixed-point value n / 10^decimals (decimals 1 to 3) with that many
// decimals: (-84, 1) -> "-8.4", (850, 2) -> "8.50", (-5, 1) -> "-0.5". plus
// puts a "+" before a value >= 0. The caller rounds or truncates to n.
size_t format_fixed(char* buf, size_t cap, int32_t n, int decimals, bool plus = false);

}  // namespace kilo

#endif
