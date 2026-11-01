# The mind core on the Linux host: the kilo-mind-host program, the unit
# tests, and a freestanding build that proves the core needs nothing from
# libc or libstdc++ beyond memcpy/memset/memcmp and from libm beyond the
# planner's float functions (what Circle gives).
#
#   make              build/kilo-mind-host, build/walk-demo
#   make check        unit tests (sanitizers), the planner against the Python vectors
#   make bench        the planner's cost per tick (no sanitizers)
#   make freestanding the core built and linked with -ffreestanding -nostdlib
#
# kernel/host/test.sh runs all three and the integration run against vbody.

KERNEL := ..
REPO := ../..
include $(KERNEL)/mind/sources.mk

CXX ?= g++
CC ?= gcc
PYTHON ?= python3
B := build

WARN := -Wall -Wextra -Werror
# -ffp-contract=off: no fused multiply-add, so the planner's prediction of
# a slot's end matches what next() then computes (as on the Cortex-A7, which
# has FMA). -fno-math-errno: sqrtf is an instruction, not a libm call.
FPFLAGS := -ffp-contract=off -fno-math-errno
CXXFLAGS := -std=c++17 $(WARN) -O2 -fno-exceptions -fno-rtti $(FPFLAGS) -I$(KERNEL)/mind -I.
CFLAGS := -std=c99 $(WARN) -O2
SAN := -fsanitize=address,undefined -fno-sanitize-recover=all -g

CORE_CXX := $(filter %.cpp,$(MIND_SRCS))
CORE_C := $(filter %.c,$(MIND_SRCS))
HOST_SRCS := main.cpp posix.cpp baud.cpp
TESTS := test_fmt test_frames test_link test_names test_planner test_walker
PLANNER_VECTORS := tests/planner_vectors.txt

# The laptop interface (docs/fast-boot.md P2): POSIX sockets for --listen, its
# unit tests, and the build id STATUS reports (the commit's first 8 hex digits,
# as kernel/Makefile's KILO_BUILD; build/mind-build-id changes only with it).
HOST_SRCS += net_posix.cpp
TESTS += test_host
MIND_BUILD := $(shell git rev-parse --short=8 HEAD 2>/dev/null || echo 0)
$(shell mkdir -p $(B) && (echo $(MIND_BUILD) | cmp -s - $(B)/mind-build-id || echo $(MIND_BUILD) > $(B)/mind-build-id))
$(B)/obj/host/main.o: CXXFLAGS += -DKILO_MIND_BUILD=0x$(MIND_BUILD)
$(B)/obj/host/main.o: $(B)/mind-build-id

# Body flashing (docs/fast-boot.md P4): the image, CRC-32, manifest and the
# STK500 flasher against a scripted optiboot, and the host channel's flash
# messages (the core sources are in mind/sources.mk).
TESTS += test_flash

.PHONY: all check bench freestanding clean
all: $(B)/kilo-mind-host $(B)/walk-demo

# ---- the program ----------------------------------------------------------
CORE_OBJS := $(CORE_CXX:%.cpp=$(B)/obj/%.o) $(CORE_C:%.c=$(B)/obj/%.o)
HOST_OBJS := $(HOST_SRCS:%.cpp=$(B)/obj/host/%.o)

$(B)/obj/%.o: $(KERNEL)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -c $< -o $@
$(B)/obj/%.o: $(KERNEL)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@
$(B)/obj/host/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -c $< -o $@

$(B)/kilo-mind-host: $(CORE_OBJS) $(HOST_OBJS)
	$(CXX) $^ -o $@

# The walk against the virtual body (kernel/host/test.sh).
$(B)/walk-demo: $(CORE_OBJS) $(B)/obj/host/walk_demo.o $(B)/obj/host/posix.o $(B)/obj/host/baud.o
	$(CXX) $^ -o $@

# ---- unit tests (sanitizers, own objects) ----------------------------------
SAN_OBJS := $(CORE_CXX:%.cpp=$(B)/san/%.o) $(CORE_C:%.c=$(B)/san/%.o)

$(B)/san/%.o: $(KERNEL)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(SAN) -O1 -MMD -c $< -o $@
$(B)/san/%.o: $(KERNEL)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(SAN) -O1 -MMD -c $< -o $@
$(B)/%: tests/%.cpp $(SAN_OBJS)
	$(CXX) $(CXXFLAGS) $(SAN) -O1 -MMD $< $(SAN_OBJS) -o $@

# The JSON vectors flattened to lines a C++ test reads without a JSON parser.
$(B)/vectors.txt: tests/flatten_vectors.py $(wildcard $(REPO)/protocol/vectors/*.json) $(REPO)/protocol/gen/kilo_protocol.py
	@mkdir -p $(B)
	$(PYTHON) tests/flatten_vectors.py $(REPO) > $@

# The planner in double precision throughout (-DKILO_PLANNER_DOUBLE), for
# the vector test's twin: it shows how much of a deviation from Python is
# the port and how much is float.
PLANNER_CXX := $(filter mind/planner/%,$(CORE_CXX))
DBL_OBJS := $(filter-out $(PLANNER_CXX:%.cpp=$(B)/san/%.o),$(SAN_OBJS)) $(PLANNER_CXX:%.cpp=$(B)/dbl/%.o)
$(B)/dbl/%.o: $(KERNEL)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -DKILO_PLANNER_DOUBLE $(SAN) -O1 -MMD -c $< -o $@
$(B)/test_planner_vectors: tests/test_planner_vectors.cpp $(SAN_OBJS)
	$(CXX) $(CXXFLAGS) $(SAN) -O1 -MMD $< $(SAN_OBJS) -o $@
$(B)/test_planner_vectors_double: tests/test_planner_vectors.cpp $(DBL_OBJS)
	$(CXX) $(CXXFLAGS) -DKILO_PLANNER_DOUBLE $(SAN) -O1 -MMD $< $(DBL_OBJS) -o $@

check: $(TESTS:%=$(B)/%) $(B)/vectors.txt $(B)/test_planner_vectors $(B)/test_planner_vectors_double
	@fail=0; for t in $(TESTS); do $(B)/$$t $(B)/vectors.txt || fail=1; done; \
	$(B)/test_planner_vectors $(PLANNER_VECTORS) || fail=1; \
	$(B)/test_planner_vectors_double $(PLANNER_VECTORS) || fail=1; exit $$fail

# ---- the planner's cost per tick (optimised as the kernel is, no sanitizers) --------
BENCH_DBL_OBJS := $(PLANNER_CXX:%.cpp=$(B)/bdbl/%.o) $(filter-out $(PLANNER_CXX:%.cpp=$(B)/obj/%.o),$(CORE_OBJS))
$(B)/bdbl/%.o: $(KERNEL)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -DKILO_PLANNER_DOUBLE -MMD -c $< -o $@
$(B)/bench_planner: tests/bench_planner.cpp $(CORE_OBJS)
	$(CXX) $(CXXFLAGS) $< $(CORE_OBJS) -o $@
$(B)/bench_planner_double: tests/bench_planner.cpp $(BENCH_DBL_OBJS)
	$(CXX) $(CXXFLAGS) -DKILO_PLANNER_DOUBLE $< $(BENCH_DBL_OBJS) -o $@
bench: $(B)/bench_planner $(B)/bench_planner_double
	$(B)/bench_planner
	$(B)/bench_planner_double

# ---- freestanding check -----------------------------------------------------
# Only the compiler's own headers (stdint.h, stddef.h, stdarg.h), no libc's;
# then a static link with -nostdlib against stubs of the allowed functions:
# any other undefined symbol (printf, sqrt, operator new, __cxa_*) fails it.
FREE := -ffreestanding -nostdinc -isystem $(shell $(CXX) -print-file-name=include) \
        -fno-stack-protector -fno-pie -fno-asynchronous-unwind-tables -O2 $(WARN) $(FPFLAGS)
FREE_OBJS := $(CORE_CXX:%.cpp=$(B)/free/%.o) $(CORE_C:%.c=$(B)/free/%.o)
# memcpy memset memcmp: circle/util.h. The rest: the toolchain's libm, which
# Circle links (STDLIB_SUPPORT 1), the planner's float functions (planner/fmath.h;
# GCC turns a sinf and a cosf of the same angle into sincosf).
ALLOWED := memcpy memset memcmp sinf cosf sincosf tanf atanf atan2f

$(B)/free/%.o: $(KERNEL)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 $(FREE) -fno-exceptions -fno-rtti -fno-threadsafe-statics -I$(KERNEL)/mind -c $< -o $@
$(B)/free/%.o: $(KERNEL)/%.c
	@mkdir -p $(dir $@)
	$(CC) -std=c99 $(FREE) -c $< -o $@
$(B)/free/stubs.o: tests/freestanding_stubs.c
	@mkdir -p $(dir $@)
	$(CC) -std=c99 $(FREE) -c $< -o $@

freestanding: $(FREE_OBJS) $(B)/free/stubs.o
	ld -r -o $(B)/free/core.o $(FREE_OBJS)
	@undef=$$(nm -u $(B)/free/core.o | awk '{print $$2}' | sort -u | tr '\n' ' '); \
	echo "core's undefined symbols: $${undef:-none}"; \
	for s in $$undef; do case " $(ALLOWED) " in *" $$s "*) ;; *) echo "not allowed: $$s"; exit 1;; esac; done
	$(CC) -nostdlib -static -no-pie -Wl,-e,kilo_freestanding_entry $(B)/free/core.o $(B)/free/stubs.o \
	    -o $(B)/free/core.elf
	@echo "freestanding link ok: $(B)/free/core.elf"

clean:
	rm -rf $(B)

-include $(shell find $(B) -name '*.d' 2>/dev/null)

# ---- P3 (docs/fast-boot.md): tricks, perception, behaviour, remote and the Mind --------
# Their unit tests join `make check` (TESTS is read when check runs).
TESTS += test_tricks test_perception test_behaviour
# Perception and behaviour compute in double as Python does (util/dmath.h):
# the drives' arousal decay needs exp, their optional gaussian noise log, sin
# and cos (GCC fuses the pair into sincos). Circle links libm, which has them.
ALLOWED += exp log sin cos sincos
