// A KernelStore (platform.h) in memory, for the kernel updater's tests: the
// card's files by name, one open at a time, and failures on demand.
#ifndef KILO_HOST_TESTS_FAKE_STORE_H
#define KILO_HOST_TESTS_FAKE_STORE_H
#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include "platform.h"

namespace kilo {

class FakeStore final : public KernelStore {
 public:
  std::map<std::string, std::vector<uint8_t>> files;
  std::vector<std::string> ops;  // "create kernel7.new", "rename kernel7.img kernel7.old", ...
  // Failures to inject.
  bool fail_create = false;
  int64_t fail_write_after = -1;  // write() fails once this many bytes are written
  bool corrupt_read = false;      // flip a bit of the first byte read back
  bool fail_remove_old = false;
  std::string fail_rename_from;   // rename(from, ...) fails for this from
  bool fail_rollback = false;     // rename(kernel7.old, kernel7.img) fails too
  // What was asked.
  uint32_t writes = 0;
  uint32_t reads = 0;
  uint32_t max_write = 0;

  Result create(const char* name) override {
    ops.push_back(std::string("create ") + name);
    if (fail_create || !open_.empty()) return Result::Failed;
    open_ = name;
    writing_ = true;
    files[open_].clear();
    pos_ = 0;
    return Result::Ok;
  }
  Result open(const char* name) override {
    ops.push_back(std::string("open ") + name);
    if (!open_.empty()) return Result::Failed;
    if (!files.count(name)) return Result::Missing;
    open_ = name;
    writing_ = false;
    pos_ = 0;
    return Result::Ok;
  }
  Result write(const uint8_t* data, uint32_t len) override {
    if (open_.empty() || !writing_) return Result::Failed;
    writes++;
    if (len > max_write) max_write = len;
    std::vector<uint8_t>& f = files[open_];
    if (fail_write_after >= 0 && (int64_t)(f.size() + len) > fail_write_after) return Result::Failed;
    f.insert(f.end(), data, data + len);
    return Result::Ok;
  }
  int32_t read(uint8_t* buf, uint32_t len) override {
    if (open_.empty() || writing_) return -1;
    reads++;
    const std::vector<uint8_t>& f = files[open_];
    uint32_t n = (uint32_t)std::min<size_t>(len, f.size() - pos_);
    for (uint32_t k = 0; k < n; k++) buf[k] = f[pos_ + k];
    if (corrupt_read && pos_ == 0 && n > 0) buf[0] ^= 0x01;
    pos_ += n;
    return (int32_t)n;
  }
  Result close() override {
    if (open_.empty()) return Result::Failed;
    ops.push_back("close " + open_);
    open_.clear();
    return Result::Ok;
  }
  Result remove(const char* name) override {
    ops.push_back(std::string("remove ") + name);
    if (fail_remove_old && std::string(name) == "kernel7.old") return Result::Failed;
    return files.erase(name) ? Result::Ok : Result::Missing;
  }
  Result rename(const char* from, const char* to) override {
    ops.push_back(std::string("rename ") + from + " " + to);
    if (fail_rename_from == from) return Result::Failed;
    if (fail_rollback && std::string(from) == "kernel7.old" && std::string(to) == "kernel7.img") return Result::Failed;
    if (!files.count(from)) return Result::Missing;
    if (files.count(to)) return Result::Failed;
    files[to] = files[from];
    files.erase(from);
    return Result::Ok;
  }
  Result sync() override {
    ops.push_back("sync");
    return Result::Ok;
  }

  bool has(const std::string& name) const { return files.count(name) != 0; }
  std::string text(const std::string& name) const {
    auto it = files.find(name);
    return it == files.end() ? "<none>" : std::string(it->second.begin(), it->second.end());
  }
  bool renamed() const {
    for (const std::string& o : ops)
      if (o.rfind("rename", 0) == 0) return true;
    return false;
  }

 private:
  std::string open_;
  bool writing_ = false;
  size_t pos_ = 0;
};

}  // namespace kilo

#endif
