// authorized_keys: the laptops' public keys the host channel accepts
// (kernel/README.md "Who may talk to Kilo: keys"). The Circle kernel has
// them built in: kernel/keys/gen_keys.py turns kernel/keys/authorized_keys
// into a generated table at build time. kilo-mind-host reads the same file
// format at run time (--authorized-keys FILE, kernel/host/posix.h).
#ifndef KILO_MIND_NET_AUTHORIZED_KEYS_H
#define KILO_MIND_NET_AUTHORIZED_KEYS_H
#include <stdint.h>

namespace kilo {

struct AuthorizedKey {
  static constexpr uint32_t kNameMax = 40;  // with the null; a longer comment is cut
  uint8_t key[32];                          // X25519 public key
  char name[kNameMax];                      // the file's comment: whose key (for the log)
};

// The index of key in keys[0..n), or -1. Constant time in the list: every
// entry is compared whole (crypto_verify32), and the match does not branch.
int32_t find_authorized_key(const AuthorizedKey* keys, uint32_t n, const uint8_t key[32]);

}  // namespace kilo

#endif
