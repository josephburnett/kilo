// slurp(): a repository file as a string, by its path from the repo root
// (REPO_ROOT, set by body/tools/host_tests.sh). Tests use it to read the
// legacy firmware and the generated tables they check the body against.
#ifndef KILO_TEST_SLURP_H
#define KILO_TEST_SLURP_H
#include <fstream>
#include <sstream>
#include <string>

#ifndef REPO_ROOT
#define REPO_ROOT "../.."
#endif

static inline std::string slurp(const char* rel) {
  std::ifstream f(std::string(REPO_ROOT) + "/" + rel);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

#endif
