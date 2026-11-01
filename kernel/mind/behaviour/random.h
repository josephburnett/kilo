// random: Python's random.Random, bit for bit: the Mersenne Twister
// (MT19937) seeded as random.Random(int) seeds it (init_by_array over the
// seed's 32-bit words), random() from two draws (53 bits), uniform() and
// gauss(). The behaviour layer draws its noise, chances and glances from
// it, so a seeded C++ runner makes the same choices as the Python one
// (kernel/host/tests/test_behaviour.cpp replays Python's runs).
#ifndef KILO_MIND_BEHAVIOUR_RANDOM_H
#define KILO_MIND_BEHAVIOUR_RANDOM_H
#include <stdint.h>

namespace kilo {
namespace behaviour {

class PyRandom {
 public:
  explicit PyRandom(uint64_t seed = 0) { seed_with(seed); }
  void seed_with(uint64_t seed);  // random.Random(seed) for 0 <= seed < 2^64
  double random();                // [0, 1), 53 bits
  double uniform(double a, double b) { return a + (b - a) * random(); }
  double gauss(double mu, double sigma);
  uint32_t genrand();

 private:
  void init_genrand(uint32_t s);
  static constexpr int kN = 624;
  uint32_t mt_[kN];
  int mti_ = kN + 1;
  bool has_gauss_next_ = false;
  double gauss_next_ = 0;
};

}  // namespace behaviour
}  // namespace kilo

#endif
