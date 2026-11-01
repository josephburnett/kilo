// image: the body firmware image the mind flashes (docs/fast-boot.md §6),
// held whole in RAM: body.bin (raw flash bytes from address 0, what
// avr-objcopy -O binary makes of the build) and its manifest (build hash,
// size, CRC-32).
//
// Two ways in, one image:
// - the laptop channel: begin() with FLASH_BEGIN's manifest, write() each
//   FLASH_DATA in order, finish() at FLASH_COMMIT (net/host_server.h);
// - the SD card: the platform reads body.json and body.bin from the FAT
//   partition, parse_manifest() and load_from(); needs_flash() against the
//   body's HELLO says whether to flash at boot (kernel/mind/README.md
//   "Flashing" has the file names and format).
//
// Errors are FLASH_STATUS.error codes (KILO_FLASH_ERROR_*): 0 is none.
#ifndef KILO_MIND_FLASH_IMAGE_H
#define KILO_MIND_FLASH_IMAGE_H
#include <stdint.h>
#include "flash/crc32.h"
#include "proto/kilo_protocol.h"

namespace kilo {

struct Manifest {
  uint32_t build;  // -DKILO_BUILD: the body's HELLO.fw_build once it runs this image
  uint32_t size;   // bytes of body.bin
  uint32_t crc32;  // zlib.crc32 of body.bin
};

// The manifest file's format version ("format" in body.json).
constexpr uint32_t kManifestFormat = 1;
// File names on the SD card's FAT partition, next to kernel7.img.
constexpr const char* kImageFile = "body.bin";
constexpr const char* kManifestFile = "body.json";

// Parse body.json: one flat JSON object, as the laptop writes it
// (mind/kilo/tools/body_image.py):
//
//   {"format": 1, "build": "3d441ec0", "size": 31234, "crc32": "cbf43926", ...}
//
// build and crc32 are 8 hex digits in a string, size a decimal number,
// format must be 1; other keys (the laptop adds "commit") are skipped.
// Returns null, or why the text is not a manifest (a static string).
const char* parse_manifest(const char* text, uint32_t len, Manifest& out);

// Write a manifest as body.json text (null-terminated; what the platform
// writes to SD after a FLASH_BEGIN with PERSIST). Returns the length, 0 if
// cap is too small (96 bytes always do).
uint32_t format_manifest(const Manifest& m, char* out, uint32_t cap);

class FlashImage {
 public:
  static constexpr uint32_t kMaxSize = KILO_HOST_FLASH_MAX_IMAGE;
  static constexpr uint32_t kPage = KILO_HOST_FLASH_PAGE;

  // Start receiving an image of m.size bytes. BAD_SIZE (0 or over
  // kMaxSize) or BAD_BUILD (0); either way what was held before is gone.
  uint8_t begin(const Manifest& m);
  // The next len bytes at offset, which must be received() (in order) and
  // within size: else BAD_OFFSET, and the image stays failed until begin().
  uint8_t write(uint32_t offset, const uint8_t* data, uint32_t len);
  // All bytes in and the CRC good: ready(). Else SHORT, CRC_MISMATCH or the
  // write's error, and the image is cleared.
  uint8_t finish();

  // The whole image at once (from SD): begin, write, finish.
  uint8_t load_from(const Manifest& m, const uint8_t* data, uint32_t size);

  void clear();
  bool ready() const { return ready_; }
  const Manifest& manifest() const { return manifest_; }
  uint32_t received() const { return received_; }
  // The CRC-32 of what was received (the manifest's, once ready()).
  uint32_t crc() const { return crc_.value(); }

  // Pages to program: size rounded up to kPage. Page i's kPage bytes; past
  // size they read 0xFF (erased flash), as avrdude pads a last page.
  uint32_t pages() const { return (manifest_.size + kPage - 1) / kPage; }
  const uint8_t* page(uint32_t i) const { return data_ + i * kPage; }

  // Whether the body should be flashed with this image: ready() and the
  // body's HELLO reports another build. hello null (no HELLO from the body
  // at all, e.g. an empty or broken firmware) counts as another build.
  bool needs_flash(const kilo_hello_t* hello) const;

 private:
  uint8_t data_[kMaxSize] = {};
  Manifest manifest_ = {};
  uint32_t received_ = 0;
  Crc32 crc_;
  uint8_t error_ = 0;  // a write failed: finish() reports it
  bool ready_ = false;
};

static_assert(KILO_HOST_FLASH_MAX_IMAGE % KILO_HOST_FLASH_PAGE == 0, "whole pages");

}  // namespace kilo

#endif
