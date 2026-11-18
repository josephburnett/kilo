// A RecordCard in memory for the saver's tests: files by path, each call
// optionally slow (the fake clock advances by write_us per write), refusals
// on demand, and power_cut(): what a FAT card keeps when the power goes in
// the middle of a write (the open file has what was written up to the last
// sync, or nothing if it was never synced; renames and removes done stay).
#ifndef KILO_HOST_TESTS_FAKE_CARD_H
#define KILO_HOST_TESTS_FAKE_CARD_H
#include <map>
#include <string>
#include <vector>
#include "platform.h"
#include "tests/fakes.h"

namespace kilo {

class FakeCard final : public RecordCard {
 public:
  void set_clock(FakeClock& clock) { clock_ = &clock; }

  bool present = true;
  uint64_t write_us = 2000;     // what a write takes
  uint64_t sync_us = 5000;
  uint64_t slow_once_us = 0;    // the next write takes this long instead
  int fail_writes = 0;          // the next n writes fail
  uint32_t calls = 0, writes = 0, max_write = 0;
  uint64_t max_call_us = 0;
  std::map<std::string, std::vector<uint8_t>> files;   // what is on the card
  std::map<std::string, size_t> synced;                // the open file's bytes that survive a cut

  uint32_t ready_calls = 0;     // ready(): the first one mounts a real card
  bool ready() override {
    ready_calls++;
    return present;
  }
  Result make_dir(const char* p) override {
    tick(100);
    dirs_.insert({p, true});
    return Result::Ok;
  }
  Result create(const char* p) override {
    tick(1000);
    if (!open_.empty()) return Result::Failed;
    open_ = p;
    files[p].clear();
    synced[p] = 0;
    pos_ = 0;
    reading_ = false;
    return Result::Ok;
  }
  Result append(const char* p) override {
    tick(1000);
    if (!open_.empty()) return Result::Failed;
    open_ = p;
    synced[p] = files[p].size();
    pos_ = files[p].size();
    reading_ = false;
    return Result::Ok;
  }
  Result open(const char* p) override {
    tick(500);
    if (!open_.empty()) return Result::Failed;
    if (files.count(p) == 0) return Result::Missing;
    open_ = p;
    pos_ = 0;
    reading_ = true;
    return Result::Ok;
  }
  Result seek(uint32_t off) override {
    if (open_.empty()) return Result::Failed;
    pos_ = off;
    return Result::Ok;
  }
  Result write(const uint8_t* d, uint32_t n) override {
    writes++;
    if (n > max_write) max_write = n;
    tick(slow_once_us != 0 ? slow_once_us : write_us);
    slow_once_us = 0;
    if (open_.empty() || reading_) return Result::Failed;
    if (fail_writes > 0) {
      fail_writes--;
      return Result::Failed;
    }
    std::vector<uint8_t>& f = files[open_];
    f.insert(f.end(), d, d + n);
    pos_ = f.size();
    return Result::Ok;
  }
  int32_t read(uint8_t* buf, uint32_t n) override {
    tick(300);
    if (open_.empty() || !reading_) return -1;
    const std::vector<uint8_t>& f = files[open_];
    if (pos_ >= f.size()) return 0;
    uint32_t k = (uint32_t)(f.size() - pos_ < n ? f.size() - pos_ : n);
    std::copy(f.begin() + (long)pos_, f.begin() + (long)pos_ + k, buf);
    pos_ += k;
    return (int32_t)k;
  }
  Result sync() override {
    tick(sync_us);
    if (open_.empty()) return Result::Failed;
    synced[open_] = files[open_].size();
    return Result::Ok;
  }
  Result close() override {
    tick(2000);
    if (open_.empty()) return Result::Failed;
    if (!reading_) synced[open_] = files[open_].size();
    open_.clear();
    return Result::Ok;
  }
  Result remove(const char* p) override {
    tick(1500);
    return files.erase(p) ? Result::Ok : Result::Missing;
  }
  Result rename(const char* a, const char* b) override {
    tick(1500);
    if (files.count(a) == 0) return Result::Missing;
    if (files.count(b) != 0) return Result::Failed;
    files[b] = files[a];
    files.erase(a);
    return Result::Ok;
  }
  Result list_begin(const char* dir) override {
    tick(200);
    list_.clear();
    std::string pre = std::string(dir) + "/";
    for (auto& kv : files)
      if (kv.first.compare(0, pre.size(), pre) == 0) list_.push_back({kv.first.substr(pre.size()), kv.second.size()});
    li_ = 0;
    return Result::Ok;
  }
  Result list_next(char* name, uint32_t n, uint32_t* size) override {
    tick(100);
    if (li_ >= list_.size()) return Result::Missing;
    snprintf(name, n, "%s", list_[li_].first.c_str());
    *size = (uint32_t)list_[li_].second;
    li_++;
    return Result::Ok;
  }

  // The power goes: the open file keeps what was synced; the handle is gone.
  void power_cut() {
    if (!open_.empty() && !reading_) files[open_].resize(synced[open_]);
    open_.clear();
  }
  bool has(const std::string& p) const { return files.count(p) != 0; }
  size_t count_ext(const char* ext) const {
    size_t n = 0;
    for (auto& kv : files)
      if (kv.first.size() > 4 && kv.first.compare(kv.first.size() - 4, 4, ext) == 0) n++;
    return n;
  }
  bool busy() const { return !open_.empty(); }

 private:
  void tick(uint64_t us) {
    calls++;
    if (us > max_call_us) max_call_us = us;
    if (clock_ != nullptr) clock_->t += us;
  }
  FakeClock* clock_ = nullptr;
  std::string open_;
  size_t pos_ = 0;
  bool reading_ = false;
  std::map<std::string, bool> dirs_;
  std::vector<std::pair<std::string, size_t>> list_;
  size_t li_ = 0;
};

}  // namespace kilo

#endif
