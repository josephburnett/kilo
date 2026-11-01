// tree: a minimal behaviour-tree engine. A port of kilo/behaviour/tree.py
// with the same semantics: tick(ctx) returns RUNNING, SUCCESS or FAILURE;
// halt() resets a node (and cancels what it runs) when a parent abandons it.
//
// - Sequence: with memory by default (resumes at the running child); fails
//   on the first failure. memory=false re-checks leading children (guards)
//   every tick and halts the running child when one fails.
// - Selector: memory-less; re-checks from the first child each tick, so a
//   higher one preempts (and halts) a lower running one.
// - Parallel: "all" succeeds when all do and fails on the first failure;
//   "any" succeeds on the first success and fails when all fail.
// - Decorators: Inverter, ForceSuccess, Cooldown, Timeout, Chance (rolled
//   once per run on the context's RNG), Repeat (n times or forever).
// - Leaves: Action (a function: SUCCESS or FAILURE), AsyncAction (Python's
//   coroutine action: RUNNING from its first tick until what it started is
//   done; halt() cancels it), Condition, Wait.
//
// The Context is the blackboard and records each tick's active path: the
// node names down to the running leaf (or the last leaf ticked).
//
// No heap: nodes are members of their owner (behaviour/library.h), children
// are arrays of pointers it holds; names are static strings or the owner's
// buffers. Python's per-node exceptions become a false return and an entry
// in ctx's error count (Context::error).
#ifndef KILO_MIND_BEHAVIOUR_TREE_H
#define KILO_MIND_BEHAVIOUR_TREE_H
#include <stdint.h>
#include "behaviour/drives.h"
#include "behaviour/random.h"

namespace kilo {
namespace behaviour {

enum class Status : uint8_t { Running, Success, Failure };
const char* status_name(Status s);  // "running", "success", "failure"

class BodyIface;
struct Personality;

// What the runner reads from the body each tick (a BodyState.snapshot()'s
// keys the behaviours and the runner use).
struct Snapshot {
  bool has_epoch = false;
  int epoch = 0;
  uint32_t reboots = 0;
  bool connected = true;
  bool alive = true;
  int mode = -1;  // KILO_MODE_*, -1 none
  bool fallen = false;
  bool moving = false;
  bool has_battery = false;
  double battery_pct = 0;
  bool has_dist = false;
  double dist_mm = 0;
  bool has_approach = false;
  double approach_mm_s = 0;
  // the last EVENTs (snapshot()["events"], newest last)
  struct Event {
    uint8_t code;
    int16_t arg;
    uint16_t tick;
    uint8_t epoch;
  };
  static constexpr int kEvents = 8;
  Event events[kEvents] = {};
  int n_events = 0;
  uint32_t event_count = 0;  // EVENTs received so far (the newest is number event_count)
};

// What the behaviours see as percepts (ctx.percepts: runner.py's list).
struct BehaviourPercepts {
  bool face = false, pet = false, near = false, approaching = false;
  bool clear = false, tilted = false, tap = false, picked_up = false;
  double startle = 0;  // 0: none
  bool has_stimulus_pan = false, has_stimulus_tilt = false, has_face_pan = false, has_face_tilt = false;
  double stimulus_pan = 0, stimulus_tilt = 0, face_pan = 0, face_tilt = 0;
  bool shutdown = false, battery_critical = false;
  uint16_t events = 0;  // percept_events: a bit per perception::Percept produced this tick
};

// ctx.data: the scratch the behaviours share.
struct Scratch {
  bool has_posture = false;
  const char* posture = nullptr;  // the cached posture goal (name, balance)
  int8_t posture_balance = -1;    // -1 None, 0, 1
  bool has_balance = false;
  bool balance = false;
  bool has_head = false;
  double head_pan = 0, head_tilt = 0;
  bool has_settle_until = false;
  double settle_until = 0;
  bool winding_down = false;
  bool shutdown_requested = false;
  bool poweroff = false;
  bool low_battery = false;
  void forget_goals() {
    has_posture = false;
    has_balance = false;
  }
};

struct Context {
  static constexpr int kDepth = 16;
  BodyIface* body = nullptr;
  const Personality* cfg = nullptr;
  PyRandom* rng = nullptr;
  double (*clock)(void* arg) = nullptr;  // tree.py Context.clock
  void* clock_arg = nullptr;
  Snapshot state;
  BehaviourPercepts percepts;
  Drives drives;
  Scratch data;
  double now = 0;
  uint32_t ticks = 0;
  // errors (ctx.errors): how many since the runner last took them, and the last one
  uint32_t errors = 0;
  const char* error_node = nullptr;
  const char* error_text = nullptr;
  // the per-tick trace
  const char* path[kDepth];
  int path_n = 0;
  const char* stack[kDepth];
  int depth = 0;
  const char* running[kDepth];
  int running_n = -1;  // -1: nothing ran

  void begin_tick();
  // The active path; valid until the next tick.
  void end_tick();
  void push(const char* name) {
    if (depth < kDepth) stack[depth] = name;
    depth++;
  }
  void pop() { depth--; }
  void leaf(Status s);
  void error(const char* node, const char* text) {
    errors++;
    error_node = node;
    error_text = text;
  }
};

class Node {
 public:
  explicit Node(const char* name) : name_(name) {}
  Status tick(Context& ctx);
  virtual void halt() { has_status_ = false; }
  const char* name() const { return name_; }
  void set_name(const char* name) { name_ = name; }
  bool has_status() const { return has_status_; }
  Status status() const { return status_; }

 protected:
  ~Node() = default;
  // Not pure: a pure virtual would need __cxa_pure_virtual, which the
  // freestanding core does not link (the same for the other hooks below).
  virtual Status do_tick(Context&) { return Status::Failure; }

 private:
  const char* name_;
  bool has_status_ = false;
  Status status_ = Status::Failure;
};

// ---- composites ------------------------------------------------------------------------

class Sequence : public Node {
 public:
  Sequence(const char* name, Node* const* children, int n, bool memory = true)
      : Node(name), children_(children), n_(n), memory_(memory) {}
  void halt() override;
  int index() const { return idx_; }

 protected:
  Status do_tick(Context& ctx) override;

 private:
  Node* const* children_;
  int n_;
  bool memory_;
  int idx_ = 0;
  int running_ = -1;
};

class Selector : public Node {
 public:
  Selector(const char* name, Node* const* children, int n) : Node(name), children_(children), n_(n) {}
  void halt() override;

 protected:
  Status do_tick(Context& ctx) override;

 private:
  Node* const* children_;
  int n_;
  int running_ = -1;
};

class Parallel : public Node {
 public:
  static constexpr int kMax = 8;
  enum class Policy : uint8_t { All, Any };
  Parallel(const char* name, Node* const* children, int n, Policy policy = Policy::All)
      : Node(name), children_(children), n_(n < kMax ? n : kMax), policy_(policy) {}
  void halt() override;

 protected:
  Status do_tick(Context& ctx) override;

 private:
  Status finish(Status st);
  Node* const* children_;
  int n_;
  Policy policy_;
  bool done_[kMax] = {};
  int n_done_ = 0;
};

// ---- decorators --------------------------------------------------------------------------

class Decorator : public Node {
 public:
  Decorator(const char* name, Node* child) : Node(name), child_(child) {}
  void halt() override {
    child_->halt();
    Node::halt();
  }
  Node* child() const { return child_; }

 protected:
  Node* child_;
};

class Inverter : public Decorator {
 public:
  explicit Inverter(Node* child, const char* name = "Inverter") : Decorator(name, child) {}

 protected:
  Status do_tick(Context& ctx) override;
};

class ForceSuccess : public Decorator {
 public:
  explicit ForceSuccess(Node* child, const char* name = "ForceSuccess") : Decorator(name, child) {}

 protected:
  Status do_tick(Context& ctx) override;
};

// After the child finishes (either way, or is abandoned while running),
// FAILURE without ticking it for `seconds`.
class Cooldown : public Decorator {
 public:
  Cooldown(double seconds, Node* child, const char* name) : Decorator(name, child), seconds_(seconds) {}
  void halt() override;

 protected:
  Status do_tick(Context& ctx) override;

 private:
  double seconds_;
  double ready_at_ = kNegInfinity;
  double last_now_ = kNegInfinity;
  bool running_ = false;
  static constexpr double kNegInfinity = -__builtin_inf();
};

// FAILURE (and halt the child) if it is still RUNNING `seconds` after it started.
class Timeout : public Decorator {
 public:
  Timeout(double seconds, Node* child, const char* name) : Decorator(name, child), seconds_(seconds) {}
  void halt() override {
    started_ = false;
    Decorator::halt();
  }

 protected:
  Status do_tick(Context& ctx) override;

 private:
  double seconds_;
  bool started_ = false;
  double started_at_ = 0;
};

// Roll ctx.rng once per run: with probability p run the child, else FAILURE.
class Chance : public Decorator {
 public:
  Chance(double p, Node* child, const char* name) : Decorator(name, child), p_(p) {}
  void halt() override {
    active_ = false;
    Decorator::halt();
  }

 protected:
  Status do_tick(Context& ctx) override;

 private:
  double p_;
  bool active_ = false;
};

// Re-run the child after each SUCCESS; times 0 = forever (RUNNING). A child
// FAILURE fails it; with times, SUCCESS after that many.
class Repeat : public Decorator {
 public:
  Repeat(Node* child, uint32_t times, const char* name) : Decorator(name, child), times_(times) {}
  void halt() override {
    count_ = 0;
    Decorator::halt();
  }

 protected:
  Status do_tick(Context& ctx) override;

 private:
  uint32_t times_;
  uint32_t count_ = 0;
};

// ---- leaves ----------------------------------------------------------------------------------

// A function of the context: SUCCESS or FAILURE (Python: a status, a bool or None).
class Action : public Node {
 public:
  explicit Action(const char* name) : Node(name) {}

 protected:
  virtual Status run(Context&) { return Status::Success; }
  Status do_tick(Context& ctx) override;
};

// tree.py Action with a coroutine: start() begins it (false: it failed at
// once), poll() reports it, cancel() abandons it. RUNNING from the first
// tick (a new task is never done yet) until poll() says otherwise.
class AsyncAction : public Node {
 public:
  explicit AsyncAction(const char* name) : Node(name) {}
  void halt() override;

 protected:
  virtual bool start(Context&) { return false; }
  virtual Status poll(Context&) { return Status::Failure; }
  virtual void cancel() {}
  Status do_tick(Context& ctx) override;

 private:
  bool started_ = false;
};

class Condition : public Node {
 public:
  explicit Condition(const char* name) : Node(name) {}

 protected:
  virtual bool test(Context&) { return false; }
  Status do_tick(Context& ctx) override;
};

// RUNNING for `seconds` from its first tick, then SUCCESS.
class Wait : public Node {
 public:
  Wait(double seconds, const char* name) : Node(name), seconds_(seconds) {}
  void halt() override {
    has_until_ = false;
    Node::halt();
  }
  bool waiting() const { return has_until_; }

 protected:
  Status do_tick(Context& ctx) override;

 private:
  double seconds_;
  bool has_until_ = false;
  double until_ = 0;
};

// tree.py tick_root: stamp the time, tick, record the active path.
Status tick_root(Node& root, Context& ctx);

// "%g" of a duration or probability for node names (Wait(1.5s), Cooldown(30s),
// Chance(0.5)): up to 6 significant digits, no trailing zeros.
void format_g(char* buf, uint32_t cap, double v);

}  // namespace behaviour
}  // namespace kilo

#endif
