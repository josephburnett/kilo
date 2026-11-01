#include "util/log.h"
#include "util/fmt.h"

namespace kilo {

void logf(Log& log, LogLevel level, const char* fmt, ...) {
  char line[kLogLineMax];  // on the stack: no static buffer to share between callers
  va_list ap;
  va_start(ap, fmt);
  vformat(line, sizeof line, fmt, ap);
  va_end(ap);
  log.write(level, line);
}

}  // namespace kilo
