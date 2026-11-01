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
	mind/proto/crc16.c \
	mind/proto/kilo_frame.c

# The laptop interface (docs/fast-boot.md §5, P2): RAM log ring and host server.
MIND_SRCS += \
	mind/util/logring.cpp \
	mind/net/host_frame.cpp \
	mind/net/host_server.cpp
