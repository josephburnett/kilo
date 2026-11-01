// util/fmt and util/log: the core's own printf, against the C library's
// for every case both define.
#include <climits>
#include <cstdio>
#include <cstring>
#include "tests/check.h"
#include "tests/fakes.h"
#include "util/fmt.h"
#include "util/log.h"

using namespace kilo;

// kilo::format and snprintf must agree on fmt with these arguments.
#define SAME(fmt, ...)                                                          \
  do {                                                                          \
    char a[64], b[64];                                                          \
    size_t n = format(a, sizeof a, fmt, __VA_ARGS__);                           \
    std::snprintf(b, sizeof b, fmt, __VA_ARGS__);                               \
    CHECKF(std::strcmp(a, b) == 0 && n == std::strlen(b), "%s: \"%s\" != \"%s\"", fmt, a, b); \
  } while (0)

int main() {
  SAME("%d", 0);
  SAME("%d", 42);
  SAME("%d", -42);
  SAME("%d", INT_MAX);
  SAME("%d", INT_MIN);
  SAME("%u", 0u);
  SAME("%u", UINT_MAX);
  SAME("%x", 0xBEEFu);
  SAME("%08x", 0x45524F43u);
  SAME("%x", 0u);
  SAME("%5d", 42);
  SAME("%5d", -42);
  SAME("%05d", -42);
  SAME("%03u", 7u);
  SAME("%2d", 12345);
  SAME("%s", "hello");
  SAME("%8s|", "ab");
  SAME("%c%c", 'o', 'k');
  SAME("%3c", 'x');
  SAME("100%% %s", "done");
  SAME("HELLO proto %u.%u build %08x reset %s epoch %u%s", 2u, 0u, 0xdeadbeefu, "COLD", 3u, "");

  char buf[64];
  // A null string, an unknown conversion, a trailing '%'. The formats are
  // not literals: GCC's format check would reject these at compile time.
  const char* volatile null_str = nullptr;  // volatile: GCC would see the null
  const char* unknown = "a%qb";
  const char* trailing = "end%";
  format(buf, sizeof buf, "%s", null_str);
  CHECK(std::strcmp(buf, "(null)") == 0);
  format(buf, sizeof buf, unknown, 1);
  CHECKF(std::strcmp(buf, "a?b") == 0, "%s", buf);
  format(buf, sizeof buf, trailing, 0);
  CHECKF(std::strcmp(buf, "end") == 0, "%s", buf);

  // cut to fit, always terminated
  char small[6];
  size_t n = format(small, sizeof small, "%s", "abcdefgh");
  CHECKF(n == 5 && std::strcmp(small, "abcde") == 0, "%zu %s", n, small);
  char one[1] = {'x'};
  CHECK(format(one, sizeof one, "%d", 123) == 0 && one[0] == '\0');
  char two[2];
  format(two, sizeof two, "%05d", -1);
  CHECK(std::strcmp(two, "-") == 0);

  // logf: one line per call, cut at kLogLineMax - 1
  CaptureLog log;
  logf(log, LogLevel::Warn, "x=%d", 5);
  CHECK(log.lines.size() == 1 && log.lines[0] == "x=5");
  char longs[400];
  std::memset(longs, 'a', sizeof longs - 1);
  longs[sizeof longs - 1] = '\0';
  logf(log, LogLevel::Info, "%s", longs);
  CHECKF(log.lines[1].size() == kLogLineMax - 1, "%zu", log.lines[1].size());
  return report("fmt");
}
