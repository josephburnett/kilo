// The HostServer tests' common steps: the throwaway key list as the
// server's, and a connection taken through the hello and the handshake.
#ifndef KILO_HOST_TESTS_HOST_RIG_H
#define KILO_HOST_TESTS_HOST_RIG_H
#include "net/host_server.h"
#include "tests/check.h"
#include "tests/fake_net.h"

namespace kilo {

// The tests' keys (fake_net.h test_keys) as who may talk to server.
inline void use_test_keys(HostServer& server, Random& random) {
  uint32_t n;
  const AuthorizedKey* keys = test_keys(&n);
  server.set_keys(keys, n, random);
}

// A connection on slot that becomes a client: the hello, the handshake (one
// X25519 per pass: three passes), the client's first record. pass() is one
// pass of the mind's loop.
template <class Pass>
FakeConn& connect_client(FakeListener& listener, int32_t slot, Pass pass) {
  FakeConn& c = listener.connect(slot);
  for (int k = 0; k < 10 && !c.established; k++) {
    pass();
    c.pump();
  }
  pass();
  CHECKF(c.established && c.open, "slot %d: no handshake", (int)slot);
  return c;
}

}  // namespace kilo

#endif
