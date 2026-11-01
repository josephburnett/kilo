// Minimal test harness (as body/test/check.h): CHECK records a failure and
// keeps going, so one run reports every broken case; main() returns
// report(), nonzero on any failure.
#ifndef KILO_HOST_TESTS_CHECK_H
#define KILO_HOST_TESTS_CHECK_H
#include <cstdio>

static int g_fail = 0;
static int g_checks = 0;

#define CHECKF(cond, ...)                                                      \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fail++;                                                                \
      if (g_fail <= 30) {                                                      \
        std::printf("%s:%d: CHECK(%s) failed: ", __FILE__, __LINE__, #cond);   \
        std::printf(__VA_ARGS__);                                              \
        std::printf("\n");                                                     \
      }                                                                        \
    }                                                                          \
  } while (0)

#define CHECK(cond) CHECKF(cond, "%s", "")

static int report(const char* name) {
  std::printf("%-12s %6d checks, %d failed\n", name, g_checks, g_fail);
  return g_fail ? 1 : 0;
}

#endif
