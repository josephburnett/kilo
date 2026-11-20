// cardflash: the body firmware from the SD card, once, at boot (the card
// route of docs/laptop-setup.md, `kilo card PATH`; kernel/README.md
// "Flashing the body"). The laptop puts body.bin and body.json next to
// kernel7.img; at the next boot the mind compares the manifest with what
// the body reports in its HELLO, flashes it with the same Flasher the
// laptop's FLASH_* use, and writes what happened to body-result.txt on the
// card, so a laptop that reads the card later (`kilo card PATH --status`)
// can tell without a network.
//
// When: once per boot, after the body's first HELLO and its first TELEMETRY
// (the battery), or kHelloWaitUs after the link came up with no HELLO
// (no firmware, a broken one, or the body is off), and only while the card
// is free (poll(card_free): no kernel update writing it, no bridge or
// laptop flash holding the UART).
//
// The rules, in this order (decide_card_flash(), each logged):
//   1. no body.json: nothing (no image on the card is not an error)
//   2. body.json or body.bin fails its checks (not a manifest, no body.bin,
//      size or CRC-32 not the manifest's: a half-copied file): REFUSED,
//      checked again at every boot (copying it whole again is the cure)
//   3. the body's HELLO reports the manifest's build: SAME, no flash
//   4. body-result.txt says a flash of this very image (build, size and
//      CRC-32) FAILED: not retried. Only a changed image (or `kilo card`,
//      which removes body-result.txt when it writes the image) tries again.
//      This is what keeps a bad image from flashing at every boot.
//   5. the battery reads under kMinBatteryMv (6600 mV): REFUSED,
//      checked again at the next boot (charge it first). On USB power, or
//      with no TELEMETRY (no firmware), there is no reading and no refusal.
//   6. otherwise FLASH: then FLASHED (the HELLO after the reset reports the
//      image's build) or FAILED (the flasher's error).
// A flash attempt is never repeated within a boot, and nothing here reboots
// anything, so there is no loop.
//
// What is written (body-result.txt, via body-result.new and a rename): for
// FLASHED, FAILED and REFUSED, and for SAME unless the file already says
// FLASHED or SAME for this image (so a good flash's record, with its log,
// stays until the next image). See format_card_result() for the layout.
//
// The sounds (CardFlashHost::card_sound): "trill" after a good flash,
// "hiss" after a failed one or a refusal. Nothing while the flash runs: the
// main loop pauses everything on the link meanwhile, the sound player too.
#ifndef KILO_MIND_FLASH_CARDFLASH_H
#define KILO_MIND_FLASH_CARDFLASH_H
#include <stdint.h>
#include "flash/flasher.h"
#include "flash/image.h"
#include "link/serial_link.h"
#include "platform.h"
#include "util/logring.h"

namespace kilo {

// The result file, next to body.json on the card's FAT partition.
constexpr const char* kResultFile = "body-result.txt";
constexpr const char* kResultNewFile = "body-result.new";
// Its first line: the format.
constexpr const char* kResultMagic = "kilo-body-result 1";

enum class CardOutcome : uint8_t { None, Flashed, Failed, Refused, Same };
// "flashed", "failed", "refused", "same", "none".
const char* card_outcome_name(CardOutcome o);

// What body-result.txt says (the lines the mind reads back).
struct CardRecord {
  CardOutcome outcome;
  Manifest image;  // the card's image it is about (zeros when body.json was unreadable)
};
// The "outcome:" and "image:" lines of a result file; false if text is not one.
bool parse_card_record(const char* text, uint32_t len, CardRecord& out);

// What the decision looks at.
struct CardFacts {
  bool has_manifest;         // body.json is on the card
  const char* image_bad;     // why body.json/body.bin fail their checks; null: the image is good
  Manifest manifest;         // body.json's (when it parsed)
  bool has_hello;            // the body's HELLO
  uint32_t body_build;       // ... its fw_build
  bool has_battery;          // a TELEMETRY with a battery reading (not on USB power)
  uint16_t batt_mv;
  bool has_record;           // body-result.txt parsed
  CardRecord record;
};

enum class CardAction : uint8_t { Nothing, Refuse, Same, NotRetried, Flash };

// The rules above. *why: a static string for the log and the result file.
CardAction decide_card_flash(const CardFacts& f, const char** why);

// Who hears of the outcome: the sounds, and a walk to stop before the flash.
class CardFlashHost {
 public:
  virtual void card_sound(const char* name, const char* why) = 0;
  // Right before the flasher takes the UART (a walk the remote started in the first seconds ends first).
  virtual void card_before_flash() {}

 protected:
  ~CardFlashHost() = default;
};

class CardFlash {
 public:
  static constexpr uint64_t kHelloWaitUs = 3000000;     // no HELLO this long after begin(): decide without one
  static constexpr uint64_t kTelemetryWaitUs = 1000000; // after the HELLO, wait this long for a TELEMETRY
  static constexpr uint16_t kMinBatteryMv = 6600;  // a flash wants margin: above the body's LOW trip (6200 mV)
  static constexpr uint32_t kLogLines = 48;             // log lines kept in the result file, at most
  static constexpr uint32_t kResultMax = 9216;
  static constexpr uint32_t kManifestMax = 1024;
  static constexpr uint32_t kRecordReadMax = 1024;      // the result file's head: its key lines

  // card: the FAT partition (the kernel updater's store: the same files
  // next to kernel7.img). image: a 32 KB image of its own (not the laptop's).
  CardFlash(KernelStore& card, FlashImage& image, Flasher& flasher, SerialLink& link, Clock& clock, Log& log);
  // The log lines from the check to the end go into the result file.
  void set_ring(const LogRing* ring) { ring_ = ring; }
  void set_host(CardFlashHost* host) { host_ = host; }
  void set_mind_build(uint32_t build) { mind_build_ = build; }

  // The link has come up (link.connect()): the wait for the HELLO starts.
  void begin();
  // Every pass of the main loop after the Mind's poll(). card_free: the
  // card and the UART are nobody else's (see above). The card is read in
  // one pass (body.json, body.bin, body-result.txt: tens of ms) and written
  // in one (the result).
  void poll(bool card_free);

  bool done() const { return step_ == Step::Done; }
  // The flasher runs the card's image (so its end is this one's to report).
  bool flashing() const { return step_ == Step::Flashing; }
  CardOutcome outcome() const { return outcome_; }
  CardAction action() const { return action_; }
  // The result file as last written (or formatted), for the tests.
  const char* result_text() const { return text_; }

 private:
  enum class Step : uint8_t { Idle, Waiting, Flashing, Done };

  void decide();
  void load(CardFacts& f);
  int32_t read_file(const char* name, char* buf, uint32_t cap);  // bytes, -1 missing, -2 too big, -3 failed
  const char* load_image(const Manifest& m);
  void finish_flash();
  void record(CardOutcome outcome, const char* why);
  bool write_result();
  void say(const char* sound, const char* why);
  uint32_t ms_since_begin() const { return (uint32_t)((clock_.now_us() - begin_us_) / 1000u); }

  KernelStore& card_;
  FlashImage& image_;
  Flasher& flasher_;
  SerialLink& link_;
  Clock& clock_;
  Log& log_;
  const LogRing* ring_ = nullptr;
  CardFlashHost* host_ = nullptr;
  uint32_t mind_build_ = 0;

  Step step_ = Step::Idle;
  CardAction action_ = CardAction::Nothing;
  CardOutcome outcome_ = CardOutcome::None;
  uint64_t begin_us_ = 0;
  uint64_t hello_seen_us_ = 0;  // this check first saw a HELLO (the link asks again while no TELEMETRY comes)
  uint64_t flash_us_ = 0;       // the flasher started
  uint32_t first_seq_ = 0;  // the log ring's first line of this check
  CardFacts facts_ = {};
  char why_[160] = {};
  char text_[kResultMax] = {};
};

}  // namespace kilo

#endif
