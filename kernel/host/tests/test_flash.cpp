// Body flashing (docs/fast-boot.md P4): the CRC-32 against zlib's vectors
// (protocol/vectors/host_frames.json "crc32", flattened), body.json parsing
// and writing, FlashImage, the STK500 flasher against a scripted optiboot
// on a fake UART and clock (sync after the LED blink, pages, verify, the
// HELLO after; no bootloader, a desync, a bad signature, a verify mismatch
// retried, a wrong build), the image channel of HostServer: FLASH_*, and
// UPDATE_* (the kernel updater against an in-memory card,
// tests/fake_store.h) with the exclusion between the two, and the boot-time
// flash from the SD card (CardFlash: its rules, the result file, several
// boots on one card).
#include <cstdlib>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "flash/cardflash.h"
#include "flash/crc32.h"
#include "flash/flasher.h"
#include "flash/image.h"
#include "link/serial_link.h"
#include "net/host_frame.h"
#include "net/host_server.h"
#include "tests/check.h"
#include "tests/fake_net.h"
#include "tests/fake_store.h"
#include "tests/fakes.h"
#include "tests/host_rig.h"
#include "update/updater.h"
#include "util/logring.h"

using namespace kilo;

namespace {

std::vector<uint8_t> pattern(uint32_t n, uint32_t seed) {
  std::vector<uint8_t> v(n);
  uint32_t x = seed;
  for (uint32_t i = 0; i < n; i++) {
    x = x * 1103515245u + 12345u;
    v[i] = (uint8_t)(x >> 16);
  }
  return v;
}

// ---- a scripted optiboot on the body's UART ----------------------------------------
//
// The body as the flasher sees it: the firmware until RESET is pulled, then
// optiboot (after its start-up delay and LED blink) speaking STK500v1 as
// optiboot.c does, a 1 s watchdog, LEAVE_PROGMODE or a desync starting the
// firmware, and the firmware's boot HELLO a little later.

class FakeGpio final : public Gpio {
 public:
  std::vector<char> edges;
  bool low = false;
  void reset_assert() override {
    edges.push_back('L');
    low = true;
  }
  void reset_release() override {
    edges.push_back('H');
    low = false;
  }
};

class FakeOptiboot final : public Serial {
 public:
  enum class Mode { App, Reset, Boot };

  // What this body does.
  bool bootloader = true;            // false: RESET restarts the firmware, nothing listens
  uint64_t listen_after_us = 440000; // start-up delay + 3 LED flashes (the Uno build)
  uint8_t sig[3] = {0x1E, 0x95, 0x0F};
  int corrupt_reads = 0;             // READ_PAGEs that answer a flipped byte
  bool drop_answers = false;         // optiboot hears but never answers
  uint32_t build = 0x46414B45;       // the firmware's HELLO.fw_build ("FAKE")
  uint32_t new_build = 0x3D441EC0;   // after a programmed flash
  uint64_t app_boot_us = 300000;     // firmware start to its boot HELLO
  uint32_t write_max = 0;            // > 0: write() takes at most this many bytes

  // What happened.
  Mode mode = Mode::App;
  uint32_t baud = KILO_BAUD;
  uint8_t flash[32768];
  int resets = 0, syncs = 0, pages_written = 0, pages_read = 0, leaves = 0, desyncs = 0;
  std::vector<uint32_t> bauds;       // every set_baud

  FakeOptiboot(FakeClock& clock, FakeGpio& gpio) : clock_(clock), gpio_(gpio) {
    std::memset(flash, 0xFF, sizeof flash);
    kilo_rx_init(&app_rx_);
  }

  int32_t read(uint8_t* buf, uint32_t n) override {
    step();
    size_t k = std::min<size_t>(n, out_.size());
    for (size_t i = 0; i < k; i++) {
      buf[i] = out_.front();
      out_.pop_front();
    }
    return (int32_t)k;
  }
  int32_t write(const uint8_t* buf, uint32_t n) override {
    step();
    if (write_max > 0 && n > write_max) n = write_max;
    if (mode == Mode::Boot && baud == KILO_HOST_FLASH_BAUD) {
      // The 328P's RX FIFO holds two bytes (and the shift register a third)
      // while optiboot is not reading yet; the rest are lost.
      for (uint32_t i = 0; i < n; i++)
        if (listening() || in_.size() < 3) in_.push_back(buf[i]);
      last_rx_us_ = clock_.t;
      parse();
    }
    // The firmware answers HELLO_REQ; the rest of what the link sends is dropped.
    if (mode == Mode::App && baud == KILO_BAUD) {
      for (uint32_t i = 0; i < n; i++) {
        kilo_frame_t f;
        if (kilo_rx_push(&app_rx_, buf[i], &f) == KILO_FRAME_OK && f.type == KILO_MSG_HELLO_REQ && !hello_due_)
          send_hello();
      }
    }
    return (int32_t)n;
  }
  uint32_t tx_free() override { return 4096; }
  bool set_baud(uint32_t b) override {
    baud = b;
    bauds.push_back(b);
    return true;
  }

  // Advance with the clock: RESET edges, optiboot's watchdog, the firmware's boot HELLO.
  void step() {
    uint64_t now = clock_.t;
    // Every RESET edge since the last step, in order (a pulse may fall between two calls).
    for (; edges_seen_ < gpio_.edges.size(); edges_seen_++) {
      if (gpio_.edges[edges_seen_] == 'L') {
        mode = Mode::Reset;
        resets++;
        in_.clear();
        hello_due_ = false;
      } else if (mode == Mode::Reset) {
        if (bootloader) {
          mode = Mode::Boot;
          boot_at_ = now;
          last_rx_us_ = now;
        } else {
          start_app(now, false);
        }
      }
    }
    if (mode == Mode::Boot) {
      parse();
      uint64_t since = last_rx_us_ > boot_at_ + listen_after_us ? last_rx_us_ : boot_at_ + listen_after_us;
      if (now > since + 1000000) start_app(now, false);  // WATCHDOG_1S
    }
    if (hello_due_ && now >= hello_at_) {
      hello_due_ = false;
      send_hello();
    }
  }

  void send_hello() {
    {
      kilo_hello_t h;
      std::memset(&h, 0, sizeof h);
      h.proto_major = KILO_PROTO_MAJOR;
      h.proto_minor = KILO_PROTO_MINOR;
      h.fw_build = build;
      h.reset_cause = cause_;
      h.max_payload = KILO_MAX_PAYLOAD;
      h.joint_count = KILO_JOINT_COUNT;
      h.epoch = 1;
      uint8_t wire[KILO_MAX_WIRE];
      uint8_t n = kilo_frame_encode(KILO_MSG_HELLO, 0, (const uint8_t*)&h, sizeof h, wire);
      out_.insert(out_.end(), wire, wire + n);
    }
  }

 private:
  bool listening() const { return clock_.t >= boot_at_ + listen_after_us; }

  void start_app(uint64_t now, bool flashed) {
    mode = Mode::App;
    in_.clear();
    uint32_t was = build;
    if (flashed) build = new_build;
    // body core/reset.cpp: another build than the record's is FLASHED, else WARM_UNEXPECTED.
    cause_ = build != was ? KILO_RESET_CAUSE_FLASHED : KILO_RESET_CAUSE_WARM_UNEXPECTED;
    hello_due_ = true;
    hello_at_ = now + app_boot_us;
  }

  void answer(std::initializer_list<uint8_t> bytes) {
    if (!drop_answers) out_.insert(out_.end(), bytes.begin(), bytes.end());
  }

  // optiboot.c's loop: a command, its arguments, CRC_EOP (else the
  // watchdog resets into the firmware), INSYNC, data, OK.
  void parse() {
    if (!listening()) return;
    for (;;) {
      if (in_.empty() || mode != Mode::Boot) return;
      uint8_t cmd = in_[0];
      size_t need = 2;
      if (cmd == stk::kLoadAddress) need = 4;
      else if (cmd == stk::kProgPage) need = in_.size() >= 3 ? 5u + (in_[1] << 8 | in_[2]) : 1000000u;
      else if (cmd == stk::kReadPage) need = 5;
      if (in_.size() < need) return;
      if (in_[need - 1] != stk::kCrcEop) {
        desyncs++;
        start_app(clock_.t, pages_written > 0);
        return;
      }
      std::vector<uint8_t> c(in_.begin(), in_.begin() + (long)need);
      in_.erase(in_.begin(), in_.begin() + (long)need);
      switch (cmd) {
        case stk::kGetSync:
          syncs++;
          answer({stk::kInsync, stk::kOk});
          break;
        case stk::kLoadAddress:
          addr_ = (uint32_t)(c[1] | c[2] << 8) * 2u;
          answer({stk::kInsync, stk::kOk});
          break;
        case stk::kProgPage: {
          uint32_t len = (uint32_t)(c[1] << 8 | c[2]);
          CHECKF(c[3] == 'F' && len == 128 && addr_ % 128 == 0, "PROG_PAGE %u at %u", len, addr_);
          std::memcpy(flash + addr_, c.data() + 4, len);
          pages_written++;
          answer({stk::kInsync, stk::kOk});
          break;
        }
        case stk::kReadPage: {
          uint32_t len = (uint32_t)(c[1] << 8 | c[2]);
          if (drop_answers) break;
          out_.push_back(stk::kInsync);
          for (uint32_t k = 0; k < len; k++) out_.push_back(flash[addr_ + k]);
          if (corrupt_reads > 0) {
            corrupt_reads--;
            out_[out_.size() - len + 5] ^= 0x01;
          }
          out_.push_back(stk::kOk);
          pages_read++;
          break;
        }
        case stk::kReadSign:
          answer({stk::kInsync, sig[0], sig[1], sig[2], stk::kOk});
          break;
        case stk::kLeaveProgmode:
          leaves++;
          answer({stk::kInsync, stk::kOk});
          start_app(clock_.t + 16000, true);  // WATCHDOG_16MS
          return;
        default:  // ENTER_PROGMODE and the rest: verifySpace, INSYNC OK
          answer({stk::kInsync, stk::kOk});
          break;
      }
    }
  }

  FakeClock& clock_;
  FakeGpio& gpio_;
  size_t edges_seen_ = 0;
  kilo_rx_t app_rx_;
  std::deque<uint8_t> in_;
  std::deque<uint8_t> out_;
  uint64_t boot_at_ = 0;
  uint64_t last_rx_us_ = 0;
  uint32_t addr_ = 0;
  bool hello_due_ = false;
  uint64_t hello_at_ = 0;
  uint8_t cause_ = 0;
};

struct Rig {
  FakeClock clock;
  FakeGpio gpio;
  FakeOptiboot body{clock, gpio};
  CaptureLog log;
  SerialLink link{body, clock, log};
  Flasher flasher{body, clock, gpio, link, log};
  FlashImage image;

  Rig() { link.connect(); }
  // The main loop: the link only while the flasher does not own the UART.
  void pass() {
    link.mark_alive();
    flasher.poll();
    if (!flasher.owns_uart()) link.poll();
  }
  // Run until the flasher is done (or ms pass). Returns the elapsed ms.
  uint32_t run(uint32_t max_ms = 30000) {
    uint32_t t = 0;
    for (; t < max_ms && flasher.busy(); t++) {
      clock.advance_ms(1);
      pass();
    }
    return t;
  }
  void load(uint32_t size, uint32_t build = 0x3D441EC0) {
    std::vector<uint8_t> img = pattern(size, size);
    CHECK(image.load_from(Manifest{build, size, crc32(img.data(), size)}, img.data(), size) == KILO_FLASH_ERROR_NONE);
    body.new_build = build;
  }
};

// ---- tests ---------------------------------------------------------------------------

int g_crc_vectors = 0;

void test_crc32_vectors(const char* path) {
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string what, name, data, crc;
    ss >> what >> name >> data >> crc;
    if (what != "crc32") continue;
    std::vector<uint8_t> d = unhex(data);
    uint32_t want = (uint32_t)std::strtoul(crc.c_str(), nullptr, 16);
    CHECKF(crc32(d.data(), (uint32_t)d.size()) == want, "crc32 %s", name.c_str());
    // In pieces, as FLASH_DATA arrives.
    Crc32 c;
    for (size_t i = 0; i < d.size(); i += 7) c.update(d.data() + i, (uint32_t)std::min<size_t>(7, d.size() - i));
    CHECKF(c.value() == want, "crc32 %s in pieces", name.c_str());
    g_crc_vectors++;
  }
  CHECKF(g_crc_vectors >= 3, "%d crc32 vectors", g_crc_vectors);
}

void test_manifest() {
  Manifest m = {};
  const char* py = "{\n  \"format\": 1,\n  \"build\": \"3d441ec0\",\n  \"size\": 31234,\n  \"crc32\": \"CBF43926\",\n"
                   "  \"commit\": \"3d441ec0aa\",\n  \"note\": null,\n  \"n\": 5, \"ok\": true\n}\n";
  CHECKF(parse_manifest(py, (uint32_t)std::strlen(py), m) == nullptr, "python's json.dumps(indent=2)");
  CHECK(m.build == 0x3D441EC0u && m.size == 31234 && m.crc32 == 0xCBF43926u);

  char buf[96];
  uint32_t n = format_manifest(Manifest{0xDEADBEEF, 7, 0x0000000A}, buf, sizeof buf);
  CHECKF(std::string(buf, n) == "{\"format\": 1, \"build\": \"deadbeef\", \"size\": 7, \"crc32\": \"0000000a\"}\n", "%s", buf);
  Manifest back = {};
  CHECK(parse_manifest(buf, n, back) == nullptr && back.build == 0xDEADBEEF && back.size == 7 && back.crc32 == 10);
  CHECK(format_manifest(back, buf, 20) == 0);

  struct Bad {
    const char* text;
    const char* why;
  } bad[] = {
      {"", "not a JSON object"},
      {"[1]", "not a JSON object"},
      {"{\"format\": 2, \"build\": \"3d441ec0\", \"size\": 1, \"crc32\": \"00000000\"}", "format is not 1"},
      {"{\"format\": 1, \"build\": \"3d441ec\", \"size\": 1, \"crc32\": \"00000000\"}", "build: not 8 hex digits"},
      {"{\"format\": 1, \"build\": 1023, \"size\": 1, \"crc32\": \"00000000\"}", "build: not 8 hex digits"},
      {"{\"format\": 1, \"build\": \"3d441ec0\", \"crc32\": \"00000000\"}", "missing"},
      {"{\"format\": 1, \"build\": \"3d441ec0\", \"size\": 1.5, \"crc32\": \"00000000\"}", "size: not a number"},
      {"{\"format\": 1, \"build\": \"3d441ec0\", \"size\": 99999999999, \"crc32\": \"0\"}", "size: not a number"},
      {"{\"format\": 1, \"x\": {\"y\": 1}, \"build\": \"3d441ec0\", \"size\": 1, \"crc32\": \"00000000\"}", "not a string"},
      {"{\"format\": 1, \"build\": \"3d441ec0\", \"size\": 1, \"crc32\": \"00000000\"} x", "text after"},
      {"{\"format\": 1 \"build\": \"3d441ec0\"}", "no ','"},
      {"{\"for\\\"mat\": 1}", "plain string"},
  };
  for (const Bad& b : bad) {
    const char* why = parse_manifest(b.text, (uint32_t)std::strlen(b.text), m);
    CHECKF(why != nullptr && std::strstr(why, b.why) != nullptr, "%s -> %s", b.text, why ? why : "ok");
  }
}

void test_image() {
  static FlashImage img;
  std::vector<uint8_t> d = pattern(1000, 1);
  uint32_t crc = crc32(d.data(), 1000);
  CHECK(img.begin(Manifest{1, 0, 0}) == KILO_FLASH_ERROR_BAD_SIZE);
  CHECK(img.begin(Manifest{1, FlashImage::kMaxSize + 1, 0}) == KILO_FLASH_ERROR_BAD_SIZE);
  CHECK(img.begin(Manifest{0, 1000, crc}) == KILO_FLASH_ERROR_BAD_BUILD);

  // In order, in chunks: ready, pages padded with 0xFF.
  CHECK(img.begin(Manifest{0x1234, 1000, crc}) == KILO_FLASH_ERROR_NONE);
  for (uint32_t o = 0; o < 1000; o += 200) CHECK(img.write(o, d.data() + o, 200) == KILO_FLASH_ERROR_NONE);
  CHECK(img.received() == 1000 && !img.ready());
  CHECK(img.finish() == KILO_FLASH_ERROR_NONE && img.ready());
  CHECK(img.pages() == 8 && img.page(7)[1000 - 7 * 128 - 1] == d[999] && img.page(7)[127] == 0xFF);
  kilo_hello_t h = {};
  h.fw_build = 0x1234;
  CHECK(!img.needs_flash(&h));
  h.fw_build = 0x1235;
  CHECK(img.needs_flash(&h) && img.needs_flash(nullptr));

  // Out of order: BAD_OFFSET sticks until finish(), which clears the image.
  CHECK(img.begin(Manifest{0x1234, 1000, crc}) == KILO_FLASH_ERROR_NONE);
  CHECK(img.write(0, d.data(), 200) == KILO_FLASH_ERROR_NONE);
  CHECK(img.write(400, d.data() + 400, 200) == KILO_FLASH_ERROR_BAD_OFFSET);
  CHECK(img.write(200, d.data() + 200, 200) == KILO_FLASH_ERROR_BAD_OFFSET);
  CHECK(img.finish() == KILO_FLASH_ERROR_BAD_OFFSET && !img.ready() && !img.needs_flash(nullptr));
  // Past the size.
  CHECK(img.begin(Manifest{0x1234, 100, 0}) == KILO_FLASH_ERROR_NONE);
  CHECK(img.write(0, d.data(), 101) == KILO_FLASH_ERROR_BAD_OFFSET);
  // Short, and a wrong CRC.
  CHECK(img.begin(Manifest{0x1234, 1000, crc}) == KILO_FLASH_ERROR_NONE);
  CHECK(img.write(0, d.data(), 999) == KILO_FLASH_ERROR_NONE);
  CHECK(img.finish() == KILO_FLASH_ERROR_SHORT);
  CHECK(img.load_from(Manifest{0x1234, 1000, crc ^ 1}, d.data(), 1000) == KILO_FLASH_ERROR_CRC_MISMATCH);
  CHECK(img.load_from(Manifest{0x1234, 1000, crc}, d.data(), 999) == KILO_FLASH_ERROR_SHORT);
  CHECK(img.load_from(Manifest{0x1234, 1000, crc}, d.data(), 1000) == KILO_FLASH_ERROR_NONE && img.ready());
  // The largest image: whole pages, no padding.
  std::vector<uint8_t> big = pattern(FlashImage::kMaxSize, 2);
  CHECK(img.load_from(Manifest{9, FlashImage::kMaxSize, crc32(big.data(), FlashImage::kMaxSize)}, big.data(),
                      FlashImage::kMaxSize) == KILO_FLASH_ERROR_NONE);
  CHECK(img.pages() == FlashImage::kMaxSize / 128);
}

void test_flash_ok() {
  static Rig r;
  r.load(31234);
  CHECK(!r.flasher.start(FlashImage(), false));  // not ready
  CHECK(r.flasher.start(r.image, true));
  CHECK(r.flasher.owns_uart() && r.body.baud == KILO_HOST_FLASH_BAUD);
  CHECK(!r.flasher.start(r.image, true));  // busy
  uint32_t ms = r.run();
  const Flasher::Status& s = r.flasher.status();
  CHECKF(s.phase == KILO_FLASH_PHASE_DONE && s.error == 0, "phase %u error %s", s.phase, flash_error_name(s.error));
  CHECK(s.attempt == 1 && s.page == 245 && s.pages == 245 && s.fw_build == 0x3D441EC0 &&
        s.reset_cause == KILO_RESET_CAUSE_FLASHED);
  CHECK(r.gpio.edges == std::vector<char>({'L', 'H'}));
  CHECK(r.body.resets == 1 && r.body.syncs == 1 && r.body.pages_written == 245 && r.body.pages_read == 245 &&
        r.body.leaves == 1 && r.body.desyncs == 0);
  CHECK(std::memcmp(r.body.flash, r.image.page(0), 245 * 128) == 0);
  CHECK(r.body.baud == KILO_BAUD);
  CHECK(r.flasher.persist_pending());
  r.flasher.persist_done();
  CHECK(!r.flasher.persist_pending() && !r.flasher.busy() && !r.flasher.owns_uart());
  CHECK(r.log.has("flash: done in"));
  // Reset 100 ms, sync after the 440 ms blink, pages in ~1 ms passes, then the HELLO 300 ms after LEAVE.
  CHECKF(ms > 840 && ms < 3000, "%u ms", ms);
  CHECK(r.link.state().has_hello && r.link.state().hello.fw_build == 0x3D441EC0);
}

void test_same_build_again() {
  // Reflashing the build the body runs: its reset record does not change,
  // so the HELLO says WARM_UNEXPECTED, which is taken.
  static Rig r;
  r.load(500, 0x46414B45);  // the fake firmware's own build
  // The fake reports its build at boot; reset once without the bootloader to get that HELLO.
  r.body.bootloader = false;
  r.gpio.reset_assert();
  r.clock.advance_ms(10);
  r.gpio.reset_release();
  for (int i = 0; i < 400; i++) {
    r.clock.advance_ms(1);
    r.pass();
  }
  CHECK(r.link.state().has_hello && r.link.state().hello.fw_build == 0x46414B45);
  r.body.bootloader = true;
  CHECK(r.flasher.start(r.image, false));
  r.run();
  const Flasher::Status& s = r.flasher.status();
  CHECKF(s.phase == KILO_FLASH_PHASE_DONE && s.reset_cause == KILO_RESET_CAUSE_WARM_UNEXPECTED, "phase %u %s",
         s.phase, flash_error_name(s.error));
  CHECK(!r.flasher.persist_pending());
}

void test_no_bootloader() {
  // RESET restarts the firmware and nothing answers GET_SYNC: two attempts,
  // SYNC_TIMEOUT, the UART back at the link baud and the link reconnected,
  // which then hears the (unchanged) firmware.
  static Rig r;
  r.load(1000);
  r.body.bootloader = false;
  CHECK(r.flasher.start(r.image, true));
  r.run();
  const Flasher::Status& s = r.flasher.status();
  CHECKF(s.phase == KILO_FLASH_PHASE_FAILED && s.error == KILO_FLASH_ERROR_SYNC_TIMEOUT, "%s",
         flash_error_name(s.error));
  CHECK(s.attempt == 2 && s.stk_cmd == stk::kGetSync && r.body.resets == 2);
  CHECK(r.gpio.edges == std::vector<char>({'L', 'H', 'L', 'H'}) && !r.gpio.low);
  CHECK(r.body.baud == KILO_BAUD && !r.flasher.persist_pending() && !r.flasher.owns_uart());
  for (int i = 0; i < 500; i++) {
    r.clock.advance_ms(1);
    r.pass();
  }
  CHECK(r.link.connected() && r.link.state().has_hello && r.link.state().hello.fw_build == 0x46414B45);
}

void test_silent_bootloader() {
  // Optiboot hears but never answers: GET_SYNC times out the same way.
  static Rig r;
  r.load(1000);
  r.body.drop_answers = true;
  r.flasher.start(r.image, false);
  r.run();
  CHECK(r.flasher.status().error == KILO_FLASH_ERROR_SYNC_TIMEOUT && r.flasher.status().attempt == 2);
}

void test_bad_signature() {
  static Rig r;
  r.load(1000);
  r.body.sig[1] = 0x94;  // an ATmega168
  r.flasher.start(r.image, false);
  r.run();
  const Flasher::Status& s = r.flasher.status();
  CHECK(s.phase == KILO_FLASH_PHASE_FAILED && s.error == KILO_FLASH_ERROR_BAD_SIGNATURE);
  CHECK(s.attempt == 1 && s.stk_cmd == stk::kReadSign && r.body.pages_written == 0);  // not retried: the wrong chip
  CHECK(r.body.baud == KILO_BAUD && !r.gpio.low);
}

void test_verify_mismatch_retried() {
  // One READ_PAGE reads a flipped bit: the attempt fails, a fresh reset
  // programs and verifies again, and the flash ends DONE.
  static Rig r;
  r.load(3000);
  r.body.corrupt_reads = 1;
  r.flasher.start(r.image, false);
  r.run();
  const Flasher::Status& s = r.flasher.status();
  CHECKF(s.phase == KILO_FLASH_PHASE_DONE && s.attempt == 2, "phase %u attempt %u %s", s.phase, s.attempt,
         flash_error_name(s.error));
  CHECK(r.body.resets == 2 && r.body.pages_written == 2 * 24 && r.log.has("VERIFY_MISMATCH"));

  static Rig r2;
  r2.load(3000);
  r2.body.corrupt_reads = 2;  // every attempt
  r2.flasher.start(r2.image, false);
  r2.run();
  CHECK(r2.flasher.status().phase == KILO_FLASH_PHASE_FAILED &&
        r2.flasher.status().error == KILO_FLASH_ERROR_VERIFY_MISMATCH && r2.flasher.status().stk_cmd == stk::kReadPage);
}

void test_wrong_build_and_no_hello() {
  static Rig r;
  r.load(1000, 0x11111111);
  r.body.new_build = 0x22222222;  // the firmware reports another build than the manifest's
  r.flasher.start(r.image, false);
  r.run();
  CHECK(r.flasher.status().phase == KILO_FLASH_PHASE_FAILED &&
        r.flasher.status().error == KILO_FLASH_ERROR_WRONG_BUILD && r.flasher.status().fw_build == 0x22222222);

  static Rig r2;
  r2.load(1000);
  r2.body.app_boot_us = 60000000;  // never within 8 s
  r2.flasher.start(r2.image, false);
  uint32_t ms = r2.run();
  CHECK(r2.flasher.status().error == KILO_FLASH_ERROR_NO_HELLO);
  CHECKF(ms > 8000 && ms < 10000, "%u ms", ms);
}

void test_short_writes() {
  // A short write mid-PROG_PAGE is finished on later passes (Serial::write
  // may take part of a command), not resent.
  static Rig r;
  r.load(2000);
  r.body.write_max = 17;
  r.flasher.start(r.image, false);
  r.run();
  CHECK(r.flasher.status().phase == KILO_FLASH_PHASE_DONE && r.body.desyncs == 0 && r.body.pages_written == 16);
}

// ---- the flash channel of HostServer -----------------------------------------------------

struct HostRig {
  FakeClock clock;
  FakeGpio gpio;
  FakeOptiboot body{clock, gpio};
  CaptureLog capture;
  LogRing ring{clock, &capture};
  SerialLink link{body, clock, ring};
  Flasher flasher{body, clock, gpio, link, ring};
  FlashImage image;
  FakeListener listener;
  FakeRandom random;
  HostServer server{listener, link, body, clock, ring, ring, 1};
  FakeStore store;
  std::vector<uint8_t> kernel_buf = std::vector<uint8_t>(1u << 20);
  KernelUpdater updater{kernel_buf.data(), (uint32_t)kernel_buf.size(), store, clock, ring};
  uint8_t seq = 0;
  std::vector<HostFrame> stash;  // what command() read besides its RESULT

  HostRig() {
    link.connect();
    server.attach_flasher(flasher, image);
    server.attach_updater(updater);
    use_test_keys(server, random);
  }
  void pass() {
    link.mark_alive();
    server.poll();
    if (!server.link_paused()) link.poll();
  }
  void run_ms(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t++) {
      clock.advance_ms(1);
      pass();
    }
  }
  FakeConn& connect(int32_t slot) {
    return connect_client(listener, slot, [this] { pass(); });
  }
  // A command's RESULT code (-1: none).
  int command(FakeConn& c, uint8_t type, const void* p = nullptr, uint32_t len = 0) {
    uint8_t s = seq++;
    c.send(type, s, p, len);
    pass();
    int code = -1;
    for (const HostFrame& f : c.take()) {
      if (f.type == KILO_HOST_MSG_RESULT && f.seq == s) code = f.payload[1];
      else stash.push_back(f);
    }
    return code;
  }
  void send_image(FakeConn& c, const std::vector<uint8_t>& d, uint32_t build, uint32_t crc, uint8_t flags) {
    kilo_host_flash_begin_t b = {build, (uint32_t)d.size(), crc, flags};
    CHECK(command(c, KILO_HOST_MSG_FLASH_BEGIN, &b, sizeof b) == KILO_HOST_RESULT_CODE_OK);
    for (uint32_t o = 0; o < d.size(); o += 200) {
      uint8_t p[204];
      uint32_t n = std::min<uint32_t>(200, (uint32_t)d.size() - o);
      std::memcpy(p, &o, 4);
      std::memcpy(p + 4, d.data() + o, n);
      c.send(KILO_HOST_MSG_FLASH_DATA, 0, p, 4 + n);
      if (o % 1000 == 0) pass();
    }
    pass();
  }
  void send_kernel(FakeConn& c, const std::vector<uint8_t>& d, uint32_t crc, uint8_t flags) {
    kilo_host_update_begin_t b = {(uint32_t)d.size(), crc, flags};
    CHECK(command(c, KILO_HOST_MSG_UPDATE_BEGIN, &b, sizeof b) == KILO_HOST_RESULT_CODE_OK);
    for (uint32_t o = 0; o < d.size(); o += 200) {
      uint8_t p[204];
      uint32_t n = std::min<uint32_t>(200, (uint32_t)d.size() - o);
      std::memcpy(p, &o, 4);
      std::memcpy(p + 4, d.data() + o, n);
      c.send(KILO_HOST_MSG_UPDATE_DATA, 0, p, 4 + n);
      if (o % 800 == 0) pass();  // 4 frames, within the 1 KB a pass reads per client
    }
    pass();
  }
};

std::vector<kilo_host_update_status_t> update_statuses(const std::vector<HostFrame>& fs) {
  std::vector<kilo_host_update_status_t> out;
  for (const HostFrame& f : fs) {
    if (f.type != KILO_HOST_MSG_UPDATE_STATUS) continue;
    kilo_host_update_status_t st;
    std::memcpy(&st, f.payload, sizeof st);
    out.push_back(st);
  }
  return out;
}

void test_host_channel() {
  static HostRig r;
  FakeConn& a = r.connect(0);
  FakeConn& obs = r.connect(1);
  std::vector<uint8_t> d = pattern(5000, 7);
  uint32_t crc = crc32(d.data(), 5000);
  kilo_host_flash_begin_t b = {0x3D441EC0, 5000, crc, 0};

  // Control first; a bad manifest.
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &b, sizeof b) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);
  uint8_t take[2] = {1, 0};
  CHECK(r.command(a, KILO_HOST_MSG_CONTROL, take, 2) == KILO_HOST_RESULT_CODE_OK);
  kilo_host_flash_begin_t big = {1, FlashImage::kMaxSize + 1, 0, 0};
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &big, sizeof big) == KILO_HOST_RESULT_CODE_IMAGE_BAD);
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_COMMIT) == KILO_HOST_RESULT_CODE_BAD_ARG);  // nothing begun
  CHECK(r.command(obs, KILO_HOST_MSG_FLASH_COMMIT) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);

  // A CRC that does not match: FLASH_STATUS FAILED CRC_MISMATCH, then RESULT IMAGE_BAD; nothing flashed.
  r.send_image(a, d, 0x3D441EC0, crc ^ 0x80, 0);
  uint8_t s = r.seq++;
  a.send(KILO_HOST_MSG_FLASH_COMMIT, s);
  r.pass();
  std::vector<HostFrame> fs = a.take();
  CHECK(fs.size() == 2 && fs[0].type == KILO_HOST_MSG_FLASH_STATUS && fs[1].type == KILO_HOST_MSG_RESULT);
  if (fs.size() == 2) {
    kilo_host_flash_status_t st;
    std::memcpy(&st, fs[0].payload, sizeof st);
    CHECK(st.phase == KILO_FLASH_PHASE_FAILED && st.error == KILO_FLASH_ERROR_CRC_MISMATCH && st.received == 5000);
    CHECK(fs[1].seq == s && fs[1].payload[1] == KILO_HOST_RESULT_CODE_IMAGE_BAD);
  }
  CHECK(!r.server.flashing() && r.gpio.edges.empty());

  // Out of order: BAD_OFFSET at COMMIT.
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &b, sizeof b) == KILO_HOST_RESULT_CODE_OK);
  uint8_t p[8] = {100, 0, 0, 0, 1, 2, 3, 4};
  a.send(KILO_HOST_MSG_FLASH_DATA, 0, p, 8);
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_COMMIT) == KILO_HOST_RESULT_CODE_IMAGE_BAD);

  // Losing control while receiving drops the image.
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &b, sizeof b) == KILO_HOST_RESULT_CODE_OK);
  uint8_t give[2] = {0, 0};
  CHECK(r.command(a, KILO_HOST_MSG_CONTROL, give, 2) == KILO_HOST_RESULT_CODE_OK);
  CHECK(r.capture.has("body image dropped (the controller let go)"));
  CHECK(r.command(a, KILO_HOST_MSG_CONTROL, take, 2) == KILO_HOST_RESULT_CODE_OK);

  // The good image, PERSIST: RESULT OK, the link paused, refusals while flashing, statuses to DONE.
  r.send_image(a, d, 0x3D441EC0, crc, KILO_FLASH_FLAGS_PERSIST);
  r.stash.clear();
  CHECK(r.server.stats().flash_bytes_rx >= 5000);
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_COMMIT) == KILO_HOST_RESULT_CODE_OK);
  CHECK(r.server.flashing() && r.server.link_paused());
  kilo_host_posture_t q = {KILO_POSTURE_STAND, 0, 0};
  CHECK(r.command(a, KILO_HOST_MSG_POSTURE, &q, sizeof q) == KILO_HOST_RESULT_CODE_FLASHING);
  CHECK(r.command(a, KILO_HOST_MSG_BRIDGE_OPEN) == KILO_HOST_RESULT_CODE_FLASHING);
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &b, sizeof b) == KILO_HOST_RESULT_CODE_FLASHING);
  r.seq++;
  obs.send(KILO_HOST_MSG_STATUS_REQ, 0);
  r.pass();
  std::vector<HostFrame> so = obs.take();
  CHECK(so.size() == 1 && so[0].type == KILO_HOST_MSG_STATUS && (so[0].payload[9] & KILO_HOST_STATE_FLASHING));
  std::vector<kilo_host_flash_status_t> sts;
  for (const HostFrame& f : r.stash) {
    if (f.type != KILO_HOST_MSG_FLASH_STATUS) continue;
    kilo_host_flash_status_t st;
    std::memcpy(&st, f.payload, sizeof st);
    sts.push_back(st);
  }
  for (int t = 0; t < 10000 && r.server.flashing(); t++) {
    r.run_ms(1);
    for (const HostFrame& f : a.take()) {
      if (f.type != KILO_HOST_MSG_FLASH_STATUS) continue;
      kilo_host_flash_status_t st;
      std::memcpy(&st, f.payload, sizeof st);
      sts.push_back(st);
    }
  }
  r.run_ms(5);
  for (const HostFrame& f : a.take())
    if (f.type == KILO_HOST_MSG_FLASH_STATUS) {
      kilo_host_flash_status_t st;
      std::memcpy(&st, f.payload, sizeof st);
      sts.push_back(st);
    }
  CHECK(!sts.empty() && obs.take().empty());  // only the flash's client gets them
  if (!sts.empty()) {
    const kilo_host_flash_status_t& last = sts.back();
    CHECKF(last.phase == KILO_FLASH_PHASE_DONE && last.fw_build == 0x3D441EC0 && last.pages == 40 &&
               last.received == 5000 && last.reset_cause == KILO_RESET_CAUSE_FLASHED,
           "phase %u error %u", last.phase, last.error);
    CHECK(sts.front().phase == KILO_FLASH_PHASE_RESETTING);
    // Every phase in order, every page reported.
    uint8_t phase = 0;
    int prog = 0, ver = 0;
    for (const auto& st : sts) {
      CHECK(st.phase >= phase);
      phase = st.phase;
      prog += st.phase == KILO_FLASH_PHASE_PROGRAMMING;
      ver += st.phase == KILO_FLASH_PHASE_VERIFYING;
    }
    CHECKF(prog >= 40 && ver >= 40, "%d programming, %d verifying statuses", prog, ver);
  }
  CHECK(!r.server.link_paused() && r.flasher.persist_pending());
  CHECK(r.command(a, KILO_HOST_MSG_POSTURE, &q, sizeof q) == KILO_HOST_RESULT_CODE_OK);
  // Until the platform has written the image to SD, it may not be replaced.
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &b, sizeof b) == KILO_HOST_RESULT_CODE_FLASHING);
  r.flasher.persist_done();
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &b, sizeof b) == KILO_HOST_RESULT_CODE_OK);
}

// UPDATE_* on HostServer: control, the checks' refusals (status first),
// the exclusion with a body flash both ways, the refusals while writing,
// and DONE: the card's files, REST, reboot_requested().
void test_update_channel() {
  static HostRig r;
  FakeConn& a = r.connect(0);
  FakeConn& obs = r.connect(1);
  r.store.files["kernel7.img"] = {'O', 'L', 'D'};
  std::vector<uint8_t> k = pattern(200000, 3);
  std::memcpy(k.data() + 150000, KILO_HOST_UPDATE_MARKER, std::strlen(KILO_HOST_UPDATE_MARKER));
  uint32_t kcrc = crc32(k.data(), (uint32_t)k.size());
  kilo_host_update_begin_t ub = {(uint32_t)k.size(), kcrc, 0};

  // Observers may not; the controller may.
  CHECK(r.command(obs, KILO_HOST_MSG_UPDATE_BEGIN, &ub, sizeof ub) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);
  CHECK(r.command(obs, KILO_HOST_MSG_UPDATE_COMMIT) == KILO_HOST_RESULT_CODE_NOT_CONTROLLER);
  uint8_t take[2] = {1, 0};
  CHECK(r.command(a, KILO_HOST_MSG_CONTROL, take, 2) == KILO_HOST_RESULT_CODE_OK);
  CHECK(r.command(a, KILO_HOST_MSG_UPDATE_COMMIT) == KILO_HOST_RESULT_CODE_BAD_ARG);  // nothing begun
  kilo_host_update_begin_t big = {(uint32_t)r.kernel_buf.size() + 1, 0, 0};
  CHECK(r.command(a, KILO_HOST_MSG_UPDATE_BEGIN, &big, sizeof big) == KILO_HOST_RESULT_CODE_IMAGE_BAD);

  // No marker: UPDATE_STATUS FAILED NOT_A_KERNEL, then RESULT IMAGE_BAD; the card untouched.
  std::vector<uint8_t> loader = pattern(5000, 9);
  r.send_kernel(a, loader, crc32(loader.data(), 5000), 0);
  uint8_t s = r.seq++;
  a.send(KILO_HOST_MSG_UPDATE_COMMIT, s);
  r.pass();
  std::vector<HostFrame> fs = a.take();
  CHECK(fs.size() == 2 && fs[0].type == KILO_HOST_MSG_UPDATE_STATUS && fs[1].type == KILO_HOST_MSG_RESULT);
  if (fs.size() == 2) {
    std::vector<kilo_host_update_status_t> st = update_statuses(fs);
    CHECK(st.size() == 1 && st[0].phase == KILO_UPDATE_PHASE_FAILED && st[0].error == KILO_UPDATE_ERROR_NOT_A_KERNEL &&
          st[0].received == 5000);
    CHECK(fs[1].seq == s && fs[1].payload[1] == KILO_HOST_RESULT_CODE_IMAGE_BAD);
  }
  CHECK(r.store.ops.empty() && !r.server.updating());

  // A body flash runs: UPDATE_BEGIN is refused (FLASHING) until it ends.
  std::vector<uint8_t> body = pattern(3000, 7);
  r.send_image(a, body, 0x3D441EC0, crc32(body.data(), 3000), 0);
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_COMMIT) == KILO_HOST_RESULT_CODE_OK && r.server.flashing());
  CHECK(r.command(a, KILO_HOST_MSG_UPDATE_BEGIN, &ub, sizeof ub) == KILO_HOST_RESULT_CODE_FLASHING);
  for (int t = 0; t < 20000 && r.server.flashing(); t++) r.run_ms(1);
  CHECK(!r.server.flashing() && r.flasher.status().phase == KILO_FLASH_PHASE_DONE);
  a.take();
  r.stash.clear();

  // The kernel: RESULT OK, CHECKING behind it, then the link paused and the refusals while it writes.
  r.send_kernel(a, k, kcrc, 0);
  CHECK(r.server.stats().update_bytes_rx >= k.size());
  CHECK(r.command(a, KILO_HOST_MSG_UPDATE_COMMIT) == KILO_HOST_RESULT_CODE_OK);
  CHECK(r.server.updating() && r.server.link_paused());
  kilo_host_flash_begin_t fb = {0x3D441EC0, 3000, crc32(body.data(), 3000), 0};
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &fb, sizeof fb) == KILO_HOST_RESULT_CODE_UPDATING);
  kilo_host_posture_t q = {KILO_POSTURE_STAND, 0, 0};
  CHECK(r.command(a, KILO_HOST_MSG_POSTURE, &q, sizeof q) == KILO_HOST_RESULT_CODE_UPDATING);
  CHECK(r.command(a, KILO_HOST_MSG_REBOOT) == KILO_HOST_RESULT_CODE_UPDATING);
  CHECK(r.command(a, KILO_HOST_MSG_BRIDGE_OPEN) == KILO_HOST_RESULT_CODE_UPDATING);
  CHECK(r.command(a, KILO_HOST_MSG_UPDATE_BEGIN, &ub, sizeof ub) == KILO_HOST_RESULT_CODE_UPDATING);
  CHECK(!r.server.reboot_requested());
  std::vector<kilo_host_update_status_t> sts = update_statuses(r.stash);
  for (int t = 0; t < 100 && r.server.updating(); t++) {
    r.run_ms(1);
    std::vector<kilo_host_update_status_t> more = update_statuses(a.take());
    sts.insert(sts.end(), more.begin(), more.end());
  }
  r.run_ms(1);
  std::vector<kilo_host_update_status_t> more = update_statuses(a.take());
  sts.insert(sts.end(), more.begin(), more.end());
  CHECK(update_statuses(obs.take()).empty());  // only the update's client gets them
  CHECK(!sts.empty());
  if (!sts.empty()) {
    CHECK(sts.front().phase == KILO_UPDATE_PHASE_CHECKING);
    const kilo_host_update_status_t& last = sts.back();
    CHECKF(last.phase == KILO_UPDATE_PHASE_DONE && last.received == k.size() && last.image_size == k.size() &&
               last.crc32 == kcrc && last.card_crc32 == kcrc,
           "phase %u error %u", last.phase, last.error);
    uint8_t phase = 0;
    int writing = 0, verifying = 0;
    for (const auto& st : sts) {
      CHECK(st.phase >= phase);
      phase = st.phase;
      writing += st.phase == KILO_UPDATE_PHASE_WRITING;
      verifying += st.phase == KILO_UPDATE_PHASE_VERIFYING;
    }
    CHECKF(writing >= 12 && verifying >= 12, "%d writing, %d verifying statuses", writing, verifying);
  }
  CHECK(r.store.files["kernel7.img"] == k && !r.store.has("kernel7.old") && !r.store.has("kernel7.new"));
  CHECK(r.server.reboot_requested() && !r.server.link_paused());
  CHECK(r.capture.has("kernel updated on the SD card: rebooting into it (REST latched)"));
  // Rebooting: no new flash or update.
  CHECK(r.command(a, KILO_HOST_MSG_FLASH_BEGIN, &fb, sizeof fb) == KILO_HOST_RESULT_CODE_UPDATING);
  CHECK(r.command(a, KILO_HOST_MSG_UPDATE_BEGIN, &ub, sizeof ub) == KILO_HOST_RESULT_CODE_UPDATING);
}

// Losing control (or the client) while receiving drops the kernel image.
void test_update_dropped() {
  static HostRig r;
  FakeConn& a = r.connect(0);
  uint8_t take[2] = {1, 0};
  CHECK(r.command(a, KILO_HOST_MSG_CONTROL, take, 2) == KILO_HOST_RESULT_CODE_OK);
  std::vector<uint8_t> k = pattern(1000, 3);
  r.send_kernel(a, k, crc32(k.data(), 1000), KILO_UPDATE_FLAGS_FORCE);
  CHECK(r.updater.receiving());
  uint8_t give[2] = {0, 0};
  CHECK(r.command(a, KILO_HOST_MSG_CONTROL, give, 2) == KILO_HOST_RESULT_CODE_OK);
  CHECK(!r.updater.receiving() && r.capture.has("kernel image dropped (the controller let go)"));
  CHECK(r.command(a, KILO_HOST_MSG_CONTROL, take, 2) == KILO_HOST_RESULT_CODE_OK);
  CHECK(r.command(a, KILO_HOST_MSG_UPDATE_COMMIT) == KILO_HOST_RESULT_CODE_BAD_ARG);
  // FORCE: an image without the marker goes on the card.
  r.store.files["kernel7.img"] = {'O', 'L', 'D'};
  r.send_kernel(a, k, crc32(k.data(), 1000), KILO_UPDATE_FLAGS_FORCE);
  CHECK(r.command(a, KILO_HOST_MSG_UPDATE_COMMIT) == KILO_HOST_RESULT_CODE_OK);
  r.run_ms(10);
  CHECK(r.store.files["kernel7.img"] == k && r.server.reboot_requested());
}

void test_no_flasher() {
  FakeClock clock;
  FakeSerial serial;
  CaptureLog capture;
  LogRing ring{clock, &capture};
  SerialLink link{serial, clock, ring};
  FakeListener listener;
  FakeRandom random;
  static HostServer server{listener, link, serial, clock, ring, ring, 1};
  use_test_keys(server, random);
  FakeConn& c = connect_client(listener, 0, [] { server.poll(); });
  uint8_t take[2] = {1, 0};
  c.send(KILO_HOST_MSG_CONTROL, 0, take, 2);
  kilo_host_flash_begin_t b = {1, 10, 0, 0};
  c.send(KILO_HOST_MSG_FLASH_BEGIN, 1, &b, sizeof b);
  server.poll();
  std::vector<HostFrame> fs = c.take();
  CHECK(fs.size() == 2 && fs[1].payload[1] == KILO_HOST_RESULT_CODE_UNSUPPORTED);
  // No kernel store either (kilo-mind-host without --card): UPDATE_* UNSUPPORTED, saying so.
  kilo_host_update_begin_t ub = {10, 0, 0};
  c.send(KILO_HOST_MSG_UPDATE_BEGIN, 2, &ub, sizeof ub);
  c.send(KILO_HOST_MSG_UPDATE_COMMIT, 3);
  server.poll();
  fs = c.take();
  CHECK(fs.size() == 2 && fs[0].payload[1] == KILO_HOST_RESULT_CODE_UNSUPPORTED &&
        fs[1].payload[1] == KILO_HOST_RESULT_CODE_UNSUPPORTED);
  if (!fs.empty()) CHECK(std::string((const char*)fs[0].payload + 2, fs[0].len - 2).find("--card") != std::string::npos);
}

// ---- the boot-time flash from the SD card (flash/cardflash.h) --------------------------------

std::string manifest_json(const Manifest& m) {
  char buf[96];
  uint32_t n = format_manifest(m, buf, sizeof buf);
  return std::string(buf, n);
}

void put(FakeStore& card, const char* name, const std::string& text) {
  card.files[name] = std::vector<uint8_t>(text.begin(), text.end());
}
void put(FakeStore& card, const char* name, const std::vector<uint8_t>& data) { card.files[name] = data; }

struct Sounds final : CardFlashHost {
  std::vector<std::string> said;
  int before = 0;
  void card_sound(const char* name, const char*) override { said.push_back(name); }
  void card_before_flash() override { before++; }
};

// One boot: a CardFlash on the rig's link and the card, run until it is done (or ms pass).
struct Boot {
  Rig& r;
  FakeStore& card;
  LogRing ring;
  FlashImage image;
  Sounds host;
  CardFlash cf;
  Boot(Rig& rig, FakeStore& c)
      : r(rig), card(c), ring(rig.clock, &rig.log), cf(c, image, rig.flasher, rig.link, rig.clock, ring) {
    cf.set_ring(&ring);
    cf.set_host(&host);
    cf.set_mind_build(0x12345678);
    cf.begin();
  }
  uint32_t run(uint32_t max_ms = 40000, bool card_free = true) {
    uint32_t t = 0;
    for (; t < max_ms && !cf.done(); t++) {
      r.clock.advance_ms(1);
      r.pass();
      cf.poll(card_free);
    }
    return t;
  }
};

// An image on the card: body.bin and body.json.
Manifest card_image(FakeStore& card, uint32_t size, uint32_t build, uint32_t seed) {
  std::vector<uint8_t> img = pattern(size, seed);
  Manifest m{build, size, crc32(img.data(), size)};
  put(card, kImageFile, img);
  put(card, kManifestFile, manifest_json(m));
  return m;
}

bool contains(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }

void test_card_record() {
  const char* text =
      "kilo-body-result 1\noutcome: failed\nimage: build 3d441ec0 size 31234 crc32 cbf43926\nwhy: x\nlog:\n"
      "  1.000 I outcome: flashed\n";
  CardRecord r = {};
  CHECK(parse_card_record(text, (uint32_t)std::strlen(text), r));
  CHECK(r.outcome == CardOutcome::Failed && r.image.build == 0x3D441EC0 && r.image.size == 31234 &&
        r.image.crc32 == 0xCBF43926);
  const char* none = "kilo-body-result 1\r\noutcome: refused\r\nimage: none\r\n";  // an editor's line ends
  CHECK(parse_card_record(none, (uint32_t)std::strlen(none), r) && r.outcome == CardOutcome::Refused &&
        r.image.build == 0 && r.image.size == 0);
  const char* bad[] = {"",
                       "kilo-body-result 2\noutcome: failed\nimage: none\n",
                       "kilo-body-result 1\noutcome: maybe\nimage: none\n",
                       "kilo-body-result 1\noutcome: failed\n",
                       "kilo-body-result 1\noutcome: failed\nimage: build 3d441ec size 1 crc32 00000000\n",
                       "kilo-body-result 1\nlog:\noutcome: failed\nimage: none\n"};
  for (const char* b : bad) CHECKF(!parse_card_record(b, (uint32_t)std::strlen(b), r), "%s", b);
}

void test_card_decide() {
  CardFacts f = {};
  const char* why = nullptr;
  CHECK(decide_card_flash(f, &why) == CardAction::Nothing);
  f.has_manifest = true;
  f.manifest = Manifest{0xAAAA0001, 1000, 0x11111111};
  f.image_bad = "body.bin is 500 bytes, body.json says 1000: half copied?";
  CHECK(decide_card_flash(f, &why) == CardAction::Refuse && contains(why, "half copied"));
  f.image_bad = nullptr;
  // match
  f.has_hello = true;
  f.body_build = 0xAAAA0001;
  CHECK(decide_card_flash(f, &why) == CardAction::Same);
  // mismatch, no battery reading (USB power, or no TELEMETRY): flash
  f.body_build = 0xBBBB0002;
  CHECK(decide_card_flash(f, &why) == CardAction::Flash);
  // no HELLO at all (no firmware): flash
  f.has_hello = false;
  CHECK(decide_card_flash(f, &why) == CardAction::Flash && contains(why, "no HELLO"));
  f.has_hello = true;
  // the battery: under the mark is refused, at the mark is not
  f.has_battery = true;
  f.batt_mv = CardFlash::kMinBatteryMv - 1;
  CHECK(decide_card_flash(f, &why) == CardAction::Refuse && contains(why, "battery"));
  f.batt_mv = CardFlash::kMinBatteryMv;
  CHECK(decide_card_flash(f, &why) == CardAction::Flash);
  // a failed flash of this very image: not retried; the body running it anyway wins
  f.has_record = true;
  f.record = CardRecord{CardOutcome::Failed, f.manifest};
  CHECK(decide_card_flash(f, &why) == CardAction::NotRetried);
  f.body_build = 0xAAAA0001;
  CHECK(decide_card_flash(f, &why) == CardAction::Same);
  f.body_build = 0xBBBB0002;
  // a changed image (the same build, another CRC or size) is tried
  f.record.image.crc32 ^= 1;
  CHECK(decide_card_flash(f, &why) == CardAction::Flash);
  f.record.image = f.manifest;
  f.record.image.size++;
  CHECK(decide_card_flash(f, &why) == CardAction::Flash);
  // a refusal, a good flash or "same" on record does not hold a flash back
  f.record.image = f.manifest;
  for (CardOutcome o : {CardOutcome::Refused, CardOutcome::Flashed, CardOutcome::Same}) {
    f.record.outcome = o;
    CHECK(decide_card_flash(f, &why) == CardAction::Flash);
  }
  // the image's checks come first
  f.record.outcome = CardOutcome::Failed;
  f.image_bad = "x";
  CHECK(decide_card_flash(f, &why) == CardAction::Refuse);
}

void test_card_flash_boots() {
  static Rig r;
  FakeStore card;
  put(card, "kernel7.img", std::string("kernel"));
  Manifest m = card_image(card, 31234, 0x3D441EC0, 7);
  r.body.new_build = m.build;

  // Boot 1: the body runs FAKE, the card has 3d441ec0: flashed.
  {
    Boot b(r, card);
    // Not before the HELLO and the TELEMETRY wait, nor while the card is not free.
    b.run(1500, false);
    CHECK(!b.cf.done() && !b.cf.flashing() && r.gpio.edges.empty());
    uint32_t ms = b.run();
    CHECKF(b.cf.done() && b.cf.outcome() == CardOutcome::Flashed, "outcome %s after %u ms",
           card_outcome_name(b.cf.outcome()), ms);
    CHECK(r.gpio.edges == std::vector<char>({'L', 'H'}) && b.host.before == 1);
    CHECK(b.host.said == std::vector<std::string>({"trill"}));
    CHECK(r.link.state().hello.fw_build == m.build);
    std::string t = card.text(kResultFile);
    CHECKF(t.rfind("kilo-body-result 1\noutcome: flashed\nimage: build 3d441ec0 size 31234 crc32 ", 0) == 0, "%s",
           t.c_str());
    CHECK(contains(t, "body before: build 46414b45\n"));
    CHECK(contains(t, "body after: build 3d441ec0 reset FLASHED\n"));
    CHECK(contains(t, "error: none\n") && contains(t, "mind: build 12345678"));
    CHECK(contains(t, "battery: no reading\n"));
    CHECK(contains(t, "next boot: nothing to do"));
    // The log lines from the check on (the flasher's own go to the rig's log here; on the Pi, to the same ring).
    CHECK(contains(t, "\nlog:\n") && contains(t, " I boot flash check: the body runs build 46414b45"));
    CHECK(!card.has(kResultNewFile));
  }
  // Boot 2: the body runs the card's build: no flash, and the record of the flash stays.
  {
    std::string before = card.text(kResultFile);
    card.ops.clear();
    Boot b(r, card);
    b.run();
    CHECK(b.cf.done() && b.cf.action() == CardAction::Same && b.cf.outcome() == CardOutcome::Same);
    CHECK(r.gpio.edges.size() == 2 && b.host.said.empty());
    CHECK(card.text(kResultFile) == before);
    for (const std::string& o : card.ops) CHECKF(o.rfind("create", 0) != 0, "%s", o.c_str());
  }
  // A "same" with no record of it is written (a card made with the build the body already ran).
  {
    card.files.erase(kResultFile);
    Boot b(r, card);
    b.run();
    CHECK(b.cf.outcome() == CardOutcome::Same && contains(card.text(kResultFile), "outcome: same\n"));
  }
}

void test_card_flash_refused() {
  static Rig r;
  FakeStore card;
  // Half copied: body.bin shorter than body.json says.
  card_image(card, 4000, 0x3D441EC0, 9);
  card.files[kImageFile].resize(2500);
  {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.done() && b.cf.outcome() == CardOutcome::Refused && r.gpio.edges.empty());
    CHECK(b.host.said == std::vector<std::string>({"hiss"}));
    std::string t = card.text(kResultFile);
    CHECKF(contains(t, "outcome: refused\n") && contains(t, "half copied") &&
               contains(t, "next boot: checked again: copy body.bin and body.json whole"),
           "%s", t.c_str());
    CHECK(r.log.has("NOT flashing"));
  }
  // Damaged: the right size, a flipped bit.
  card_image(card, 4000, 0x3D441EC0, 9);
  card.files[kImageFile][100] ^= 0x40;
  {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.outcome() == CardOutcome::Refused && r.gpio.edges.empty());
    CHECK(contains(card.text(kResultFile), "CRC_MISMATCH"));
  }
  // Longer than the manifest says; a manifest that is not one; no body.bin.
  card_image(card, 4000, 0x3D441EC0, 9);
  card.files[kImageFile].push_back(0);
  {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.outcome() == CardOutcome::Refused && contains(card.text(kResultFile), "do not belong"));
  }
  put(card, kManifestFile, std::string("{\"format\": 1, \"build\": \"3d44"));
  {
    Boot b(r, card);
    b.run();
    std::string t = card.text(kResultFile);
    CHECKF(b.cf.outcome() == CardOutcome::Refused && contains(t, "image: none\n") && contains(t, "not a manifest"),
           "%s", t.c_str());
  }
  card_image(card, 4000, 0x3D441EC0, 9);
  card.files.erase(kImageFile);
  {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.outcome() == CardOutcome::Refused && contains(card.text(kResultFile), "is missing"));
  }
  // No body.json at all: nothing, nothing written.
  card.files.clear();
  {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.done() && b.cf.action() == CardAction::Nothing && b.cf.outcome() == CardOutcome::None);
    CHECK(card.files.empty() && b.host.said.empty() && r.gpio.edges.empty());
  }
}

void test_card_flash_failed_not_retried() {
  static Rig r;
  r.body.bootloader = false;  // RESET restarts the firmware: SYNC_TIMEOUT
  FakeStore card;
  card_image(card, 3000, 0x3D441EC0, 11);
  // Boot 1: tried, failed, recorded.
  {
    Boot b(r, card);
    b.run();
    CHECKF(b.cf.done() && b.cf.outcome() == CardOutcome::Failed, "%s", card_outcome_name(b.cf.outcome()));
    CHECK(r.gpio.edges.size() == 4);  // two attempts
    CHECK(b.host.said == std::vector<std::string>({"hiss"}));
    std::string t = card.text(kResultFile);
    CHECKF(contains(t, "outcome: failed\n") && contains(t, "error: SYNC_TIMEOUT\n") &&
               contains(t, "body after: build 46414b45") && contains(t, "next boot: not retried"),
           "%s", t.c_str());
  }
  // Boots 2 and 3: the same image is not tried again; the file stays as it was.
  std::string failed = card.text(kResultFile);
  for (int boot = 0; boot < 2; boot++) {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.done() && b.cf.action() == CardAction::NotRetried && b.host.said.empty());
    CHECK(r.gpio.edges.size() == 4 && card.text(kResultFile) == failed);
  }
  CHECK(r.log.has("it is tried again only once body.bin or body.json change"));
  // A changed image: tried again (with a bootloader this time: flashed).
  r.body.bootloader = true;
  Manifest m2 = card_image(card, 3000, 0x3D441EC1, 12);
  r.body.new_build = m2.build;
  {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.outcome() == CardOutcome::Flashed && r.gpio.edges.size() == 6);
    CHECK(contains(card.text(kResultFile), "outcome: flashed\n"));
  }
  // The failed image again with its record: not retried; once `kilo card` has removed the record: tried.
  r.body.bootloader = false;
  card_image(card, 3000, 0x3D441EC0, 11);
  put(card, kResultFile, failed);
  {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.action() == CardAction::NotRetried);
  }
  card.files.erase(kResultFile);
  {
    Boot b(r, card);
    b.run();
    CHECK(b.cf.action() == CardAction::Flash && b.cf.outcome() == CardOutcome::Failed && r.gpio.edges.size() == 10);
  }
}

void test_card_result_write_fails() {
  // A write-protected card: the flash still happens; the log says the result is not on the card.
  static Rig r;
  FakeStore card;
  Manifest m = card_image(card, 1000, 0x3D441EC0, 13);
  r.body.new_build = m.build;
  card.fail_create = true;
  Boot b(r, card);
  b.run();
  CHECK(b.cf.outcome() == CardOutcome::Flashed && !card.has(kResultFile));
  CHECK(r.log.has("body-result.txt is NOT on the card"));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: test_flash VECTORS.txt\n");
    return 2;
  }
  test_crc32_vectors(argv[1]);
  test_manifest();
  test_image();
  test_flash_ok();
  test_same_build_again();
  test_no_bootloader();
  test_silent_bootloader();
  test_bad_signature();
  test_verify_mismatch_retried();
  test_wrong_build_and_no_hello();
  test_short_writes();
  test_host_channel();
  test_update_channel();
  test_update_dropped();
  test_no_flasher();
  test_card_record();
  test_card_decide();
  test_card_flash_boots();
  test_card_flash_refused();
  test_card_flash_failed_not_retried();
  test_card_result_write_fails();
  return report("flash");
}
