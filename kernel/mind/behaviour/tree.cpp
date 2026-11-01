#include "behaviour/tree.h"

namespace kilo {
namespace behaviour {

const char* status_name(Status s) {
  switch (s) {
    case Status::Running: return "running";
    case Status::Success: return "success";
    default: return "failure";
  }
}

// ---- context --------------------------------------------------------------------------

void Context::begin_tick() {
  now = clock != nullptr ? clock(clock_arg) : now;
  ticks++;
  path_n = 0;
  depth = 0;
  running_n = -1;
}

void Context::end_tick() {
  if (running_n >= 0) {
    for (int i = 0; i < running_n; i++) path[i] = running[i];
    path_n = running_n;
  }
}

void Context::leaf(Status s) {
  int n = depth < kDepth ? depth : kDepth;
  for (int i = 0; i < n; i++) path[i] = stack[i];
  path_n = n;
  if (s == Status::Running) {
    for (int i = 0; i < n; i++) running[i] = stack[i];
    running_n = n;
  }
}

Status Node::tick(Context& ctx) {
  ctx.push(name_);
  Status st = do_tick(ctx);
  ctx.pop();
  has_status_ = true;
  status_ = st;
  return st;
}

// ---- composites -----------------------------------------------------------------------

Status Sequence::do_tick(Context& ctx) {
  int running = running_;
  running_ = -1;
  if (!memory_) idx_ = 0;
  while (idx_ < n_) {
    Status st = children_[idx_]->tick(ctx);
    if (idx_ == running && st != Status::Running) running = -1;  // it finished by itself
    if (st == Status::Success) {
      idx_++;
      continue;
    }
    if (running != -1 && running != idx_) children_[running]->halt();  // a guard before it failed or took over
    if (st == Status::Running) {
      running_ = idx_;
      return Status::Running;
    }
    idx_ = 0;
    return Status::Failure;
  }
  idx_ = 0;
  return Status::Success;
}

void Sequence::halt() {
  if (running_ != -1) children_[running_]->halt();
  running_ = -1;
  idx_ = 0;
  Node::halt();
}

Status Selector::do_tick(Context& ctx) {
  for (int i = 0; i < n_; i++) {
    Status st = children_[i]->tick(ctx);
    if (st == Status::Failure) {
      if (running_ == i) running_ = -1;
      continue;
    }
    if (running_ != -1 && running_ != i) children_[running_]->halt();  // preempted
    running_ = st == Status::Running ? i : -1;
    return st;
  }
  running_ = -1;
  return Status::Failure;
}

void Selector::halt() {
  if (running_ != -1) children_[running_]->halt();
  running_ = -1;
  Node::halt();
}

Status Parallel::do_tick(Context& ctx) {
  for (int i = 0; i < n_; i++) {
    if (done_[i]) continue;
    Status st = children_[i]->tick(ctx);
    if (st != Status::Running) {
      done_[i] = true;
      n_done_++;
    }
    if (policy_ == Policy::All && st == Status::Failure) return finish(Status::Failure);
    if (policy_ == Policy::Any && st == Status::Success) return finish(Status::Success);
  }
  if (n_done_ == n_) return finish(policy_ == Policy::All ? Status::Success : Status::Failure);
  return Status::Running;
}

Status Parallel::finish(Status st) {
  for (int i = 0; i < n_; i++)
    if (!done_[i]) children_[i]->halt();
  for (int i = 0; i < n_; i++) done_[i] = false;
  n_done_ = 0;
  return st;
}

void Parallel::halt() {
  finish(Status::Failure);
  Node::halt();
}

// ---- decorators -------------------------------------------------------------------------

Status Inverter::do_tick(Context& ctx) {
  Status st = child_->tick(ctx);
  if (st == Status::Success) return Status::Failure;
  if (st == Status::Failure) return Status::Success;
  return st;
}

Status ForceSuccess::do_tick(Context& ctx) {
  return child_->tick(ctx) == Status::Running ? Status::Running : Status::Success;
}

Status Cooldown::do_tick(Context& ctx) {
  last_now_ = ctx.now;
  if (ctx.now < ready_at_) return Status::Failure;
  Status st = child_->tick(ctx);
  running_ = st == Status::Running;
  if (st != Status::Running) ready_at_ = ctx.now + seconds_;
  return st;
}

void Cooldown::halt() {
  if (running_) {  // an abandoned run still counts as a use
    ready_at_ = last_now_ + seconds_;
    running_ = false;
  }
  Decorator::halt();
}

Status Timeout::do_tick(Context& ctx) {
  if (!started_) {
    started_ = true;
    started_at_ = ctx.now;
  }
  if (ctx.now - started_at_ >= seconds_) {
    child_->halt();
    started_ = false;
    return Status::Failure;
  }
  Status st = child_->tick(ctx);
  if (st != Status::Running) started_ = false;
  return st;
}

Status Chance::do_tick(Context& ctx) {
  if (!active_) {
    if (ctx.rng->random() >= p_) return Status::Failure;
    active_ = true;
  }
  Status st = child_->tick(ctx);
  if (st != Status::Running) active_ = false;
  return st;
}

Status Repeat::do_tick(Context& ctx) {
  Status st = child_->tick(ctx);
  if (st == Status::Failure) {
    count_ = 0;
    return Status::Failure;
  }
  if (st == Status::Success) {
    count_++;
    if (times_ != 0 && count_ >= times_) {
      count_ = 0;
      return Status::Success;
    }
  }
  return Status::Running;
}

// ---- leaves ----------------------------------------------------------------------------

Status Action::do_tick(Context& ctx) {
  Status st = run(ctx);
  ctx.leaf(st);
  return st;
}

Status AsyncAction::do_tick(Context& ctx) {
  if (!started_) {
    if (!start(ctx)) {
      ctx.leaf(Status::Failure);
      return Status::Failure;
    }
    started_ = true;
    ctx.leaf(Status::Running);
    return Status::Running;
  }
  Status st = poll(ctx);
  if (st != Status::Running) started_ = false;
  ctx.leaf(st);
  return st;
}

void AsyncAction::halt() {
  if (started_) cancel();
  started_ = false;
  Node::halt();
}

Status Condition::do_tick(Context& ctx) {
  Status st = test(ctx) ? Status::Success : Status::Failure;
  ctx.leaf(st);
  return st;
}

Status Wait::do_tick(Context& ctx) {
  if (!has_until_) {
    has_until_ = true;
    until_ = ctx.now + seconds_;
  }
  if (ctx.now >= until_) {
    has_until_ = false;
    ctx.leaf(Status::Success);
    return Status::Success;
  }
  ctx.leaf(Status::Running);
  return Status::Running;
}

Status tick_root(Node& root, Context& ctx) {
  ctx.begin_tick();
  Status st = root.tick(ctx);
  ctx.end_tick();
  return st;
}

void format_g(char* buf, uint32_t cap, double v) {
  char tmp[32];
  uint32_t n = 0;
  if (v < 0) {
    tmp[n++] = '-';
    v = -v;
  }
  // 6 significant digits, fixed notation (the names' values are 0.001 .. 99999)
  int e = 0;
  if (v != 0) {
    while (v >= 10.0) {
      v /= 10.0;
      e++;
    }
    while (v < 1.0) {
      v *= 10.0;
      e--;
    }
  }
  int64_t digits = (int64_t)(v * 100000.0 + 0.5);  // d.ddddd as 6 digits
  if (digits >= 1000000) {
    digits /= 10;
    e++;
  }
  char d[6];
  for (int i = 5; i >= 0; i--) {
    d[i] = (char)('0' + digits % 10);
    digits /= 10;
  }
  int last = 5;  // the last significant digit kept
  while (last > 0 && d[last] == '0') last--;
  if (e < 0) {
    tmp[n++] = '0';
    tmp[n++] = '.';
    for (int i = -1; i > e; i--) tmp[n++] = '0';
    for (int i = 0; i <= last; i++) tmp[n++] = d[i];
  } else {
    for (int i = 0; i <= e || i <= last; i++) {
      if (i == e + 1) tmp[n++] = '.';
      tmp[n++] = i < 6 ? d[i] : '0';
    }
  }
  uint32_t k = 0;
  for (; k < n && k + 1 < cap; k++) buf[k] = tmp[k];
  buf[k] = '\0';
}

}  // namespace behaviour
}  // namespace kilo
