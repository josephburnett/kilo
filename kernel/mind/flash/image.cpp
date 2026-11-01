#include "flash/image.h"
#include "util/fmt.h"
#include "util/mem.h"

namespace kilo {

// ---- body.json ----------------------------------------------------------------------
//
// Just enough JSON for one flat object of strings and numbers: the manifest
// is written by our own tools (Python's json module, format_manifest()), so
// escapes, nesting and floats are refused rather than handled.

namespace {

struct Cursor {
  const char* p;
  const char* end;

  void skip_ws() {
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) p++;
  }
  bool eat(char c) {
    skip_ws();
    if (p < end && *p == c) {
      p++;
      return true;
    }
    return false;
  }
  // A string without escapes: [*s, *s + *n). False if it is not one.
  bool string(const char** s, uint32_t* n) {
    if (!eat('"')) return false;
    const char* start = p;
    while (p < end && *p != '"') {
      if (*p == '\\' || (unsigned char)*p < 0x20) return false;
      p++;
    }
    if (p >= end) return false;
    *s = start;
    *n = (uint32_t)(p - start);
    p++;
    return true;
  }
  // An unsigned decimal integer that fits 32 bits.
  bool number(uint32_t* v) {
    skip_ws();
    if (p >= end || *p < '0' || *p > '9') return false;
    uint64_t x = 0;
    while (p < end && *p >= '0' && *p <= '9') {
      x = x * 10u + (uint32_t)(*p - '0');
      if (x > 0xFFFFFFFFu) return false;
      p++;
    }
    if (p < end && (*p == '.' || *p == 'e' || *p == 'E')) return false;
    *v = (uint32_t)x;
    return true;
  }
  bool word(const char* w) {
    skip_ws();
    const char* q = p;
    for (; *w != '\0'; w++, q++)
      if (q >= end || *q != *w) return false;
    p = q;
    return true;
  }
};

bool equals(const char* s, uint32_t n, const char* lit) {
  uint32_t i = 0;
  for (; i < n; i++)
    if (lit[i] == '\0' || lit[i] != s[i]) return false;
  return lit[i] == '\0';
}

// Exactly 8 hex digits (either case).
bool hex32(const char* s, uint32_t n, uint32_t* v) {
  if (n != 8) return false;
  uint32_t x = 0;
  for (uint32_t i = 0; i < n; i++) {
    char c = s[i];
    uint32_t d;
    if (c >= '0' && c <= '9') d = (uint32_t)(c - '0');
    else if (c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F') d = (uint32_t)(c - 'A' + 10);
    else return false;
    x = x << 4 | d;
  }
  *v = x;
  return true;
}

}  // namespace

const char* parse_manifest(const char* text, uint32_t len, Manifest& out) {
  Cursor c = {text, text + len};
  bool have_format = false, have_build = false, have_size = false, have_crc = false;
  uint32_t format = 0;
  Manifest m = {};
  if (!c.eat('{')) return "not a JSON object";
  if (!c.eat('}')) {
    for (;;) {
      const char* key;
      uint32_t key_len;
      if (!c.string(&key, &key_len)) return "a key is not a plain string";
      if (!c.eat(':')) return "no ':' after a key";
      c.skip_ws();
      const char* s;
      uint32_t n;
      uint32_t v;
      if (equals(key, key_len, "format")) {
        if (!c.number(&format)) return "format: not a number";
        have_format = true;
      } else if (equals(key, key_len, "size")) {
        if (!c.number(&m.size)) return "size: not a number";
        have_size = true;
      } else if (equals(key, key_len, "build")) {
        if (!c.string(&s, &n) || !hex32(s, n, &m.build)) return "build: not 8 hex digits in a string";
        have_build = true;
      } else if (equals(key, key_len, "crc32")) {
        if (!c.string(&s, &n) || !hex32(s, n, &m.crc32)) return "crc32: not 8 hex digits in a string";
        have_crc = true;
      } else if (c.p < c.end && *c.p == '"') {  // another key: skip its value
        if (!c.string(&s, &n)) return "a value is not a plain string";
      } else if (!c.number(&v) && !c.word("true") && !c.word("false") && !c.word("null")) {
        return "a value is not a string, number, true, false or null";
      }
      if (c.eat(',')) continue;
      if (c.eat('}')) break;
      return "no ',' or '}' after a value";
    }
  }
  c.skip_ws();
  if (c.p != c.end) return "text after the object";
  if (!have_format || format != kManifestFormat) return "format is not 1";
  if (!have_build || !have_size || !have_crc) return "build, size or crc32 missing";
  out = m;
  return nullptr;
}

uint32_t format_manifest(const Manifest& m, char* out, uint32_t cap) {
  char buf[96];
  size_t n = format(buf, sizeof buf, "{\"format\": %u, \"build\": \"%08x\", \"size\": %u, \"crc32\": \"%08x\"}\n",
                    (unsigned)kManifestFormat, (unsigned)m.build, (unsigned)m.size, (unsigned)m.crc32);
  if (n + 1 > cap) return 0;
  mem_copy(out, buf, n + 1);
  return (uint32_t)n;
}

// ---- the image ---------------------------------------------------------------------

void FlashImage::clear() {
  manifest_ = {};
  received_ = 0;
  crc_.reset();
  error_ = KILO_FLASH_ERROR_NONE;
  ready_ = false;
}

uint8_t FlashImage::begin(const Manifest& m) {
  clear();
  if (m.size == 0 || m.size > kMaxSize) return KILO_FLASH_ERROR_BAD_SIZE;
  if (m.build == 0) return KILO_FLASH_ERROR_BAD_BUILD;
  manifest_ = m;
  // Erased flash reads 0xFF: the last page's tail is programmed as that.
  __builtin_memset(data_, 0xFF, pages() * kPage);
  return KILO_FLASH_ERROR_NONE;
}

uint8_t FlashImage::write(uint32_t offset, const uint8_t* data, uint32_t len) {
  if (error_ != KILO_FLASH_ERROR_NONE) return error_;
  if (manifest_.size == 0 || offset != received_ || len > manifest_.size - received_) {
    error_ = KILO_FLASH_ERROR_BAD_OFFSET;
    return error_;
  }
  mem_copy(data_ + offset, data, len);
  crc_.update(data, len);
  received_ += len;
  return KILO_FLASH_ERROR_NONE;
}

uint8_t FlashImage::finish() {
  uint8_t e = error_;
  if (e == KILO_FLASH_ERROR_NONE && manifest_.size == 0) e = KILO_FLASH_ERROR_BAD_SIZE;
  if (e == KILO_FLASH_ERROR_NONE && received_ != manifest_.size) e = KILO_FLASH_ERROR_SHORT;
  if (e == KILO_FLASH_ERROR_NONE && crc_.value() != manifest_.crc32) e = KILO_FLASH_ERROR_CRC_MISMATCH;
  if (e != KILO_FLASH_ERROR_NONE) {
    clear();
    return e;
  }
  ready_ = true;
  return KILO_FLASH_ERROR_NONE;
}

uint8_t FlashImage::load_from(const Manifest& m, const uint8_t* data, uint32_t size) {
  uint8_t e = begin(m);
  if (e != KILO_FLASH_ERROR_NONE) return e;
  if (size != m.size) {
    clear();
    return size < m.size ? KILO_FLASH_ERROR_SHORT : KILO_FLASH_ERROR_BAD_SIZE;
  }
  write(0, data, size);
  return finish();
}

bool FlashImage::needs_flash(const kilo_hello_t* hello) const {
  return ready_ && (hello == nullptr || hello->fw_build != manifest_.build);
}

}  // namespace kilo
