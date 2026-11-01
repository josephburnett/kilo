// What the core lists by hand because the C header does not carry it: the
// enum names (link/names.cpp) and the query -> reply map (link/consts.cpp),
// checked against the generated Python module (flattened by
// flatten_vectors.py), and the message names against the frame vectors.
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include "link/consts.h"
#include "link/names.h"
#include "tests/check.h"

using namespace kilo;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: test_names vectors.txt\n");
    return 2;
  }
  std::ifstream in(argv[1]);
  std::string line;
  std::map<int, int> replies;
  int enums = 0, msgs = 0;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string what;
    ss >> what;
    if (what == "enum") {
      std::string which, name;
      int v;
      ss >> which >> v >> name;
      const char* got = which == "event"          ? event_name((uint8_t)v)
                        : which == "ack_status"   ? ack_status_name((uint8_t)v)
                        : which == "mode"         ? mode_name((uint8_t)v)
                        : which == "reset_cause"  ? reset_cause_name((uint8_t)v)
                        : which == "config_state" ? config_state_name((uint8_t)v)
                                                  : "(no such enum)";
      CHECKF(name == got, "%s %d: %s != %s", which.c_str(), v, got, name.c_str());
      enums++;
    } else if (what == "reply") {
      int q, r;
      ss >> q >> r;
      replies[q] = r;
    } else if (what == "frame") {
      std::string name, msg;
      int type;
      ss >> name >> msg >> type;
      CHECKF(msg == msg_name((uint8_t)type), "%d: %s != %s", type, msg_name((uint8_t)type), msg.c_str());
      msgs++;
    }
  }
  CHECK(replies.size() == 7);
  for (int t = 0; t < 256; t++) {
    auto it = replies.find(t);
    int want = it == replies.end() ? 0 : it->second;
    CHECKF(query_reply((uint8_t)t) == want, "query_reply(0x%02x) = 0x%02x, want 0x%02x", t, query_reply((uint8_t)t),
           want);
  }
  CHECK(std::string(event_name(200)) == "?" && std::string(msg_name(0x7F)) == "?");
  std::printf("  %d enum names, %d message names, %zu query replies\n", enums, msgs, replies.size());
  CHECK(enums > 40 && msgs > 40);
  return report("names");
}
