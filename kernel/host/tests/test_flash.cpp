// Body flashing (docs/fast-boot.md P4): the CRC-32 against zlib's vectors
// (protocol/vectors/host_frames.json "crc32", flattened), body.json parsing
// and writing, FlashImage, the STK500 flasher against a scripted optiboot
// on a fake UART and clock (sync after the LED blink, pages, verify, the
// HELLO after; no bootloader, a desync, a bad signature, a verify mismatch
// retried, a wrong build), and the flash channel of HostServer.
#include <cstdlib>
#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "flash/crc32.h"
#include "flash/flasher.h"
#include "flash/image.h"
#include "link/serial_link.h"
#include "net/host_frame.h"
#include "net/host_server.h"
#include "tests/check.h"
#include "tests/fakes.h"
#include "util/logring.h"

using namespace kilo;

namespace {

std::vector<uint8_t> unhex(const std::string& s) {
  std::vector<uint8_t> out;
  if (s == "-") return out;
  for (size_t i = 0; i + 1 < s.size(); i += 2) out.push_back((uint8_t)std::strtoul(s.substr(i, 2).c_str(), nullptr, 16));
  return out;
}

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

class FakeConn final : public Conn {
 public:
  std::vector<uint8_t> in;
  size_t pos = 0;
  std::vector<uint8_t> out;
  bool open = false;
  HostRx rx;

  int32_t read(uint8_t* buf, uint32_t n) override {
    if (!open) return -1;
    size_t k = std::min<size_t>(n, in.size() - pos);
    if (k) std::memcpy(buf, in.data() + pos, k);
    pos += k;
    return (int32_t)k;
  }
  int32_t write(const uint8_t* buf, uint32_t n) override {
    if (!open) return -1;
    out.insert(out.end(), buf, buf + n);
    return (int32_t)n;
  }
  void close() override { open = false; }
  bool is_open() const override { return open; }

  void send(uint8_t type, uint8_t seq, const void* payload = nullptr, uint32_t len = 0) {
    uint8_t f[KILO_HOST_MAX_FRAME];
    uint32_t n = host_frame_encode(type, seq, (const uint8_t*)payload, len, f);
    in.insert(in.end(), f, f + n);
  }
  std::vector<HostFrame> take() {
    std::vector<HostFrame> got;
    for (uint8_t b : out)
      if (rx.push(b) == HostRxStatus::Frame) got.push_back(rx.frame());
    out.clear();
    return got;
  }
};

class FakeListener final : public Listener {
 public:
  FakeConn conns[4];
  std::deque<int32_t> pending;
  int32_t accept() override {
    if (pending.empty()) return -1;
    int32_t s = pending.front();
    pending.pop_front();
    conns[s].open = true;
    return s;
  }
  Conn* conn(uint32_t slot) override { return &conns[slot]; }
  uint32_t slots() const override { return 4; }
};

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
  HostServer server{listener, link, body, clock, ring, ring, 1};
  uint8_t seq = 0;
  std::vector<HostFrame> stash;  // what command() read besides its RESULT

  HostRig() {
    link.connect();
    server.attach_flasher(flasher, image);
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
    listener.pending.push_back(slot);
    pass();
    return listener.conns[slot];
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
};

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

void test_no_flasher() {
  FakeClock clock;
  FakeSerial serial;
  CaptureLog capture;
  LogRing ring{clock, &capture};
  SerialLink link{serial, clock, ring};
  FakeListener listener;
  static HostServer server{listener, link, serial, clock, ring, ring, 1};
  listener.pending.push_back(0);
  server.poll();
  FakeConn& c = listener.conns[0];
  uint8_t take[2] = {1, 0};
  c.send(KILO_HOST_MSG_CONTROL, 0, take, 2);
  kilo_host_flash_begin_t b = {1, 10, 0, 0};
  c.send(KILO_HOST_MSG_FLASH_BEGIN, 1, &b, sizeof b);
  server.poll();
  std::vector<HostFrame> fs = c.take();
  CHECK(fs.size() == 2 && fs[1].payload[1] == KILO_HOST_RESULT_CODE_UNSUPPORTED);
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
  test_no_flasher();
  return report("flash");
}
