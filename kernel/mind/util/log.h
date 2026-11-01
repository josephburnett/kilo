// log: printf-style lines into the platform's Log sink, formatted by
// util/fmt.h (the same conversions; pass int and unsigned).
#ifndef KILO_MIND_UTIL_LOG_H
#define KILO_MIND_UTIL_LOG_H
#include "platform.h"

namespace kilo {

// Longest line kept, null included; longer lines are cut.
constexpr unsigned kLogLineMax = 160;

void logf(Log& log, LogLevel level, const char* fmt, ...) __attribute__((format(printf, 3, 4)));

}  // namespace kilo

#endif
