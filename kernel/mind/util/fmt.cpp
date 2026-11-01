#include "util/fmt.h"

namespace kilo {

namespace {

struct Out {
  char* buf;
  size_t cap;
  size_t n;
  void put(char c) {
    if (n + 1 < cap) buf[n++] = c;
  }
  void pad(char c, unsigned count) {
    for (unsigned i = 0; i < count; i++) put(c);
  }
};

void put_number(Out& o, unsigned mag, bool neg, unsigned base, unsigned width, bool zero) {
  char d[10];  // least significant first; 2^32 has 10 decimal digits
  unsigned k = 0;
  do {
    unsigned r = mag % base;
    d[k++] = (char)(r < 10 ? '0' + r : 'a' + r - 10);
    mag /= base;
  } while (mag != 0);
  unsigned len = k + (neg ? 1 : 0);
  unsigned fill = width > len ? width - len : 0;
  if (!zero) o.pad(' ', fill);
  if (neg) o.put('-');
  if (zero) o.pad('0', fill);
  while (k > 0) o.put(d[--k]);
}

}  // namespace

size_t vformat(char* buf, size_t cap, const char* fmt, va_list ap) {
  Out o{buf, cap, 0};
  for (const char* p = fmt; *p != '\0'; p++) {
    if (*p != '%') {
      o.put(*p);
      continue;
    }
    p++;
    bool zero = false;
    if (*p == '0') {
      zero = true;
      p++;
    }
    unsigned width = 0;
    while (*p >= '0' && *p <= '9') width = width * 10 + (unsigned)(*p++ - '0');
    switch (*p) {
      case 'd': {
        int v = va_arg(ap, int);
        // the magnitude in unsigned arithmetic: -INT_MIN does not fit an int
        unsigned mag = v < 0 ? 0u - (unsigned)v : (unsigned)v;
        put_number(o, mag, v < 0, 10, width, zero);
        break;
      }
      case 'u':
        put_number(o, va_arg(ap, unsigned), false, 10, width, zero);
        break;
      case 'x':
        put_number(o, va_arg(ap, unsigned), false, 16, width, zero);
        break;
      case 'c':
        o.pad(' ', width > 1 ? width - 1 : 0);
        o.put((char)va_arg(ap, int));
        break;
      case 's': {
        const char* s = va_arg(ap, const char*);
        if (s == nullptr) s = "(null)";
        unsigned len = 0;
        while (s[len] != '\0') len++;
        o.pad(' ', width > len ? width - len : 0);
        for (unsigned i = 0; i < len; i++) o.put(s[i]);
        break;
      }
      case '%':
        o.put('%');
        break;
      case '\0':  // a '%' that ends the string: stop at the null
        p--;
        break;
      default:
        o.put('?');
        break;
    }
  }
  if (cap > 0) buf[o.n] = '\0';
  return o.n;
}

size_t format(char* buf, size_t cap, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  size_t n = vformat(buf, cap, fmt, ap);
  va_end(ap);
  return n;
}

}  // namespace kilo
