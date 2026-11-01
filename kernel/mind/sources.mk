# The portable mind core, paths relative to kernel/. Included by the Circle
# kernel's Makefile and by kernel/host/Makefile; both add -I mind.
MIND_SRCS = \
	mind/util/fmt.cpp \
	mind/util/log.cpp \
	mind/link/body_state.cpp \
	mind/link/consts.cpp \
	mind/link/goals.cpp \
	mind/link/names.cpp \
	mind/link/serial_link.cpp \
	mind/planner/errors.cpp \
	mind/planner/model.cpp \
	mind/planner/patterns.cpp \
	mind/planner/plan.cpp \
	mind/planner/walker.cpp \
	mind/proto/crc16.c \
	mind/proto/kilo_frame.c

# The laptop interface (docs/fast-boot.md §5, P2): RAM log ring and host server.
MIND_SRCS += \
	mind/util/logring.cpp \
	mind/net/host_frame.cpp \
	mind/net/host_server.cpp

# Body flashing (docs/fast-boot.md §6, P4): the image, its CRC-32 and
# manifest, and the STK500v1 flasher.
MIND_SRCS += \
	mind/flash/crc32.cpp \
	mind/flash/image.cpp \
	mind/flash/flasher.cpp

# ---- P3 (docs/fast-boot.md): tricks, perception, behaviour, remote and the Mind --------
MIND_SRCS += \
	mind/tricks/model.cpp \
	mind/tricks/library.cpp \
	mind/tricks/library.gen.cpp \
	mind/tricks/player.cpp \
	mind/perception/ultrasonic.cpp \
	mind/perception/imu.cpp \
	mind/perception/percepts.cpp \
	mind/behaviour/random.cpp \
	mind/behaviour/drives.cpp \
	mind/behaviour/personality.gen.cpp \
	mind/behaviour/tree.cpp \
	mind/behaviour/selector.cpp \
	mind/behaviour/library.cpp \
	mind/behaviour/runner.cpp
