#include "behaviour/random.h"
#include "util/dmath.h"

namespace kilo {
namespace behaviour {

// CPython's Modules/_randommodule.c (the reference MT19937, Matsumoto & Nishimura).

void PyRandom::init_genrand(uint32_t s) {
  mt_[0] = s;
  for (int i = 1; i < kN; i++) mt_[i] = 1812433253u * (mt_[i - 1] ^ (mt_[i - 1] >> 30)) + (uint32_t)i;
  mti_ = kN;
}

void PyRandom::seed_with(uint64_t seed) {
  // random_seed(): the absolute value's 32-bit words, least significant first (at least one)
  uint32_t key[2] = {(uint32_t)seed, (uint32_t)(seed >> 32)};
  int len = key[1] != 0 ? 2 : 1;
  init_genrand(19650218u);
  int i = 1, j = 0;
  for (int k = kN > len ? kN : len; k; k--) {
    mt_[i] = (mt_[i] ^ ((mt_[i - 1] ^ (mt_[i - 1] >> 30)) * 1664525u)) + key[j] + (uint32_t)j;
    i++;
    j++;
    if (i >= kN) {
      mt_[0] = mt_[kN - 1];
      i = 1;
    }
    if (j >= len) j = 0;
  }
  for (int k = kN - 1; k; k--) {
    mt_[i] = (mt_[i] ^ ((mt_[i - 1] ^ (mt_[i - 1] >> 30)) * 1566083941u)) - (uint32_t)i;
    i++;
    if (i >= kN) {
      mt_[0] = mt_[kN - 1];
      i = 1;
    }
  }
  mt_[0] = 0x80000000u;
  has_gauss_next_ = false;
}

uint32_t PyRandom::genrand() {
  static const uint32_t mag01[2] = {0x0u, 0x9908b0dfu};
  uint32_t y;
  if (mti_ >= kN) {
    int kk;
    for (kk = 0; kk < kN - 397; kk++) {
      y = (mt_[kk] & 0x80000000u) | (mt_[kk + 1] & 0x7fffffffu);
      mt_[kk] = mt_[kk + 397] ^ (y >> 1) ^ mag01[y & 0x1u];
    }
    for (; kk < kN - 1; kk++) {
      y = (mt_[kk] & 0x80000000u) | (mt_[kk + 1] & 0x7fffffffu);
      mt_[kk] = mt_[kk + (397 - kN)] ^ (y >> 1) ^ mag01[y & 0x1u];
    }
    y = (mt_[kN - 1] & 0x80000000u) | (mt_[0] & 0x7fffffffu);
    mt_[kN - 1] = mt_[396] ^ (y >> 1) ^ mag01[y & 0x1u];
    mti_ = 0;
  }
  y = mt_[mti_++];
  y ^= (y >> 11);
  y ^= (y << 7) & 0x9d2c5680u;
  y ^= (y << 15) & 0xefc60000u;
  y ^= (y >> 18);
  return y;
}

double PyRandom::random() {
  uint32_t a = genrand() >> 5, b = genrand() >> 6;
  return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0);
}

double PyRandom::gauss(double mu, double sigma) {
  // random.py gauss(): Box-Muller, the second value kept for the next call
  double z;
  if (has_gauss_next_) {
    z = gauss_next_;
    has_gauss_next_ = false;
  } else {
    double x2pi = random() * 6.283185307179586;  // TWOPI
    double g2rad = d_sqrt(-2.0 * d_log(1.0 - random()));
    z = d_cos(x2pi) * g2rad;
    gauss_next_ = d_sin(x2pi) * g2rad;
    has_gauss_next_ = true;
  }
  return mu + z * sigma;
}

}  // namespace behaviour
}  // namespace kilo
