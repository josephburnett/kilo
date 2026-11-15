// The kernel updater (update/updater.h) against an in-memory card
// (tests/fake_store.h): a good image installed (kernel7.old kept, slices
// bounded), the checks at commit (bad size, out of order, short, bad CRC,
// no marker, FORCE), and every store failure: before the renames the card's
// kernel7.img is untouched and kernel7.new removed; a failed last rename
// puts the old kernel back, and only if that fails too is the card broken.
// The host channel's UPDATE_* are tested in test_flash.cpp (with a flasher,
// for the exclusion).
#include <cstring>
#include <string>
#include <vector>
#include "flash/crc32.h"
#include "tests/check.h"
#include "tests/fake_store.h"
#include "tests/fakes.h"
#include "update/updater.h"

using namespace kilo;

namespace {

constexpr uint32_t kCapacity = 1u << 20;

// A fake kernel: a pattern with the marker at `at` (none if at < 0).
std::vector<uint8_t> kernel_image(uint32_t n, int64_t at) {
  std::vector<uint8_t> v(n);
  uint32_t x = 12345;
  for (uint32_t i = 0; i < n; i++) {
    x = x * 1103515245u + 12345u;
    v[i] = (uint8_t)(x >> 16);
    if (v[i] == 'K') v[i] = 'k';  // no accidental marker start
  }
  if (at >= 0) std::memcpy(v.data() + at, KILO_HOST_UPDATE_MARKER, std::strlen(KILO_HOST_UPDATE_MARKER));
  return v;
}

struct Rig {
  FakeClock clock;
  CaptureLog log;
  FakeStore store;
  std::vector<uint8_t> buf = std::vector<uint8_t>(kCapacity);
  KernelUpdater up{buf.data(), kCapacity, store, clock, log};
  std::vector<uint8_t> phases;  // every phase poll() went through
  uint32_t passes = 0;

  Rig() {
    store.files["kernel7.img"] = {'O', 'L', 'D'};
    store.files["kernel7.old"] = {'S', 'T', 'A', 'L', 'E'};
    store.files["config.txt"] = {'k'};
  }
  void send(const std::vector<uint8_t>& d, uint32_t chunk = 200) {
    for (uint32_t o = 0; o < d.size(); o += chunk) {
      uint32_t n = std::min<uint32_t>(chunk, (uint32_t)d.size() - o);
      CHECK(up.write(o, d.data() + o, n) == KILO_UPDATE_ERROR_NONE);
    }
  }
  uint8_t install(const std::vector<uint8_t>& d, uint8_t flags = 0) {
    CHECK(up.begin((uint32_t)d.size(), crc32(d.data(), (uint32_t)d.size()), flags) == KILO_UPDATE_ERROR_NONE);
    send(d);
    uint8_t e = up.commit();
    if (e != KILO_UPDATE_ERROR_NONE) return e;
    run();
    return up.status().error;
  }
  void run() {
    for (int t = 0; t < 1000 && up.busy(); t++) {
      clock.advance_ms(1);
      up.poll();
      passes++;
      if (phases.empty() || phases.back() != up.status().phase) phases.push_back(up.status().phase);
    }
  }
  std::vector<uint8_t> file(const char* name) { return store.files[name]; }
};

void test_good_image() {
  Rig r;
  std::vector<uint8_t> d = kernel_image(100000, 70000);
  CHECK(r.install(d) == KILO_UPDATE_ERROR_NONE);
  CHECKF(r.up.status().phase == KILO_UPDATE_PHASE_DONE, "phase %u error %u", r.up.status().phase,
         r.up.status().error);
  CHECK(r.file("kernel7.img") == d);
  CHECK(r.store.text("kernel7.old") == "OLD");  // the previous kernel kept, the stale one gone
  CHECK(!r.store.has("kernel7.new") && r.store.text("config.txt") == "k");
  const KernelUpdater::Status& s = r.up.status();
  CHECK(s.received == 100000 && s.size == 100000 && s.crc32 == crc32(d.data(), 100000) && s.card_crc32 == s.crc32);
  // Phases in order, the writes in slices: 7 passes to write 100000 bytes, 7 to read them back.
  std::vector<uint8_t> want = {KILO_UPDATE_PHASE_WRITING, KILO_UPDATE_PHASE_VERIFYING, KILO_UPDATE_PHASE_INSTALLING,
                               KILO_UPDATE_PHASE_DONE};
  CHECK(r.phases == want);
  CHECKF(r.store.max_write <= KernelUpdater::kSliceBytes && r.store.writes == 7, "%u writes, max %u",
         r.store.writes, r.store.max_write);
  CHECKF(r.passes >= 14 && r.passes <= 17, "%u passes", r.passes);
  // The renames only after the read-back, in this order.
  std::vector<std::string> tail(r.store.ops.end() - 4, r.store.ops.end());
  std::vector<std::string> want_ops = {"remove kernel7.old", "rename kernel7.img kernel7.old",
                                       "rename kernel7.new kernel7.img", "sync"};
  CHECK(tail == want_ops);
  CHECK(r.log.has("update: installed: kernel7.img is the new kernel"));
  CHECK(!r.up.busy());
}

void test_no_old_kernel() {
  Rig r;
  r.store.files.erase("kernel7.img");
  r.store.files.erase("kernel7.old");
  std::vector<uint8_t> d = kernel_image(5000, 0);
  CHECK(r.install(d) == KILO_UPDATE_ERROR_NONE && r.up.status().phase == KILO_UPDATE_PHASE_DONE);
  CHECK(r.file("kernel7.img") == d && !r.store.has("kernel7.old"));
  CHECK(r.log.has("no kernel7.img on the card to keep"));
}

void test_checks() {
  {  // sizes
    Rig r;
    CHECK(r.up.begin(0, 0, 0) == KILO_UPDATE_ERROR_BAD_SIZE && r.up.status().phase == KILO_UPDATE_PHASE_FAILED);
    CHECK(r.up.begin(kCapacity + 1, 0, 0) == KILO_UPDATE_ERROR_BAD_SIZE);
    CHECK(r.up.begin(kCapacity, 0, 0) == KILO_UPDATE_ERROR_NONE && r.up.receiving());
  }
  {  // bad CRC: refused at commit, the card never touched
    Rig r;
    std::vector<uint8_t> d = kernel_image(3000, 10);
    CHECK(r.up.begin(3000, crc32(d.data(), 3000) ^ 1, 0) == KILO_UPDATE_ERROR_NONE);
    r.send(d);
    CHECK(r.up.commit() == KILO_UPDATE_ERROR_CRC_MISMATCH && r.up.status().phase == KILO_UPDATE_PHASE_FAILED);
    r.run();
    CHECK(r.store.ops.empty() && r.store.text("kernel7.img") == "OLD");
  }
  {  // short
    Rig r;
    std::vector<uint8_t> d = kernel_image(3000, 10);
    CHECK(r.up.begin(3001, crc32(d.data(), 3000), 0) == KILO_UPDATE_ERROR_NONE);
    r.send(d);
    CHECK(r.up.commit() == KILO_UPDATE_ERROR_SHORT && r.store.ops.empty());
  }
  {  // out of order, and past the end
    Rig r;
    std::vector<uint8_t> d = kernel_image(3000, 10);
    CHECK(r.up.begin(3000, crc32(d.data(), 3000), 0) == KILO_UPDATE_ERROR_NONE);
    CHECK(r.up.write(1, d.data(), 10) == KILO_UPDATE_ERROR_BAD_OFFSET);
    CHECK(r.up.write(0, d.data(), 10) == KILO_UPDATE_ERROR_BAD_OFFSET);  // sticks
    CHECK(r.up.commit() == KILO_UPDATE_ERROR_BAD_OFFSET);
    CHECK(r.up.begin(10, 0, 0) == KILO_UPDATE_ERROR_NONE);
    CHECK(r.up.write(0, d.data(), 11) == KILO_UPDATE_ERROR_BAD_OFFSET);
  }
  {  // no marker: NOT_A_KERNEL (the netboot loader, a wrong file); the marker cut off at the end too
    Rig r;
    std::vector<uint8_t> d = kernel_image(3000, -1);
    CHECK(r.install(d) == KILO_UPDATE_ERROR_NOT_A_KERNEL && r.store.ops.empty());
    std::vector<uint8_t> cut = kernel_image(3000, 3000 - (int64_t)std::strlen(KILO_HOST_UPDATE_MARKER));
    cut.pop_back();
    CHECK(!has_kernel_marker(cut.data(), (uint32_t)cut.size()));
    CHECK(has_kernel_marker((const uint8_t*)"xxKILO-MIND-KERNEL-1", 20));
    CHECK(!has_kernel_marker((const uint8_t*)"KILO-MIND-KERNEL-", 17));
  }
  {  // FORCE: installed without the marker
    Rig r;
    std::vector<uint8_t> d = kernel_image(3000, -1);
    CHECK(r.install(d, KILO_UPDATE_FLAGS_FORCE) == KILO_UPDATE_ERROR_NONE);
    CHECK(r.up.status().phase == KILO_UPDATE_PHASE_DONE && r.file("kernel7.img") == d);
    CHECK(r.log.has("no marker (FORCE: installed anyway)"));
  }
  {  // drop: back to idle
    Rig r;
    CHECK(r.up.begin(10, 0, 0) == KILO_UPDATE_ERROR_NONE);
    r.up.drop();
    CHECK(r.up.status().phase == KILO_UPDATE_PHASE_IDLE && !r.up.receiving());
  }
}

// Every store failure before the renames: kernel7.img and kernel7.old as
// they were, kernel7.new gone, no rename asked.
void check_card_unchanged(Rig& r, uint8_t error, const char* what) {
  CHECKF(r.up.status().phase == KILO_UPDATE_PHASE_FAILED && r.up.status().error == error, "%s: phase %u error %u",
         what, r.up.status().phase, r.up.status().error);
  CHECKF(r.store.text("kernel7.img") == "OLD" && r.store.text("kernel7.old") == "STALE", "%s: files changed", what);
  CHECKF(!r.store.has("kernel7.new") && !r.store.renamed(), "%s: kernel7.new left or renamed", what);
  CHECK(!r.up.busy());
}

void test_store_failures() {
  std::vector<uint8_t> d = kernel_image(60000, 100);
  {
    Rig r;
    r.store.fail_create = true;
    r.install(d);
    check_card_unchanged(r, KILO_UPDATE_ERROR_WRITE_FAILED, "create");
  }
  {
    Rig r;
    r.store.fail_write_after = 40000;
    r.install(d);
    check_card_unchanged(r, KILO_UPDATE_ERROR_WRITE_FAILED, "write");
    CHECK(r.log.has("writing kernel7.new failed at byte 32768"));
  }
  {
    Rig r;
    r.store.corrupt_read = true;
    r.install(d);
    check_card_unchanged(r, KILO_UPDATE_ERROR_VERIFY_FAILED, "read back");
    CHECK(r.up.status().card_crc32 != r.up.status().crc32 && r.up.status().card_crc32 != 0);
  }
  {
    Rig r;
    r.store.fail_remove_old = true;
    r.install(d);
    CHECK(r.up.status().error == KILO_UPDATE_ERROR_INSTALL_FAILED);
    CHECK(r.store.text("kernel7.img") == "OLD" && !r.store.has("kernel7.new") && !r.store.renamed());
  }
  {  // the first rename fails: kernel7.img stays (kernel7.old was already removed)
    Rig r;
    r.store.fail_rename_from = "kernel7.img";
    r.install(d);
    CHECK(r.up.status().error == KILO_UPDATE_ERROR_INSTALL_FAILED);
    CHECK(r.store.text("kernel7.img") == "OLD" && !r.store.has("kernel7.new"));
  }
  {  // the last rename fails: the old kernel put back
    Rig r;
    r.store.fail_rename_from = "kernel7.new";
    r.install(d);
    CHECK(r.up.status().error == KILO_UPDATE_ERROR_INSTALL_FAILED);
    CHECK(r.store.text("kernel7.img") == "OLD" && !r.store.has("kernel7.new") && !r.store.has("kernel7.old"));
    CHECK(r.log.has("put kernel7.old back as kernel7.img"));
  }
  {  // and putting it back fails too: CARD_BROKEN, kernel7.new left for a person to rename
    Rig r;
    r.store.fail_rename_from = "kernel7.new";
    r.store.fail_rollback = true;
    r.install(d);
    CHECK(r.up.status().error == KILO_UPDATE_ERROR_CARD_BROKEN);
    CHECK(!r.store.has("kernel7.img") && r.store.text("kernel7.old") == "OLD" && r.file("kernel7.new") == d);
    CHECK(r.log.has("the card has NO kernel7.img"));
  }
  {  // after a failure the next update works
    Rig r;
    r.store.fail_create = true;
    r.install(d);
    r.store.fail_create = false;
    CHECK(r.install(d) == KILO_UPDATE_ERROR_NONE && r.file("kernel7.img") == d);
  }
}

void test_names() {
  CHECK(std::strcmp(update_error_name(KILO_UPDATE_ERROR_NOT_A_KERNEL), "NOT_A_KERNEL") == 0);
  CHECK(std::strcmp(update_error_name(KILO_UPDATE_ERROR_CARD_BROKEN), "CARD_BROKEN") == 0);
  CHECK(std::strcmp(update_error_name(200), "?") == 0);
}

}  // namespace

int main() {
  test_good_image();
  test_no_old_kernel();
  test_checks();
  test_store_failures();
  test_names();
  return report("update");
}
