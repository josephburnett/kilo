# Kilo mind kernel: a Circle bare-metal image for the Raspberry Pi 2 B.
#
#   make -C kernel               Circle libraries, build/kernel7.img, build/netboot/kernel7.img
#   make -C kernel boot          download the Raspberry Pi firmware into circle/boot (network)
#   make -C kernel clean         remove build/
#   make -C kernel circle-clean  remove the objects Circle built inside the submodule

TOOLCHAIN ?= $(HOME)/opt/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi

CIRCLEHOME := circle
# "override" so a stale circle/Config.mk cannot win over these.
override PREFIX := $(TOOLCHAIN)/bin/arm-none-eabi-
override RASPPI := 2
override AARCH  := 32
# Circle's default: link libgcc and the toolchain's libm, nothing else.
# No libc, no libstdc++, no exceptions or RTTI (see README).
override STDLIB_SUPPORT := 1

BUILD  := build
TARGET := $(BUILD)/kernel7
# Rules.mk's "clean" runs "rm -f *.o ... $(EXTRACLEAN)" in this directory.
EXTRACLEAN := -r $(BUILD)

CIRCLE_LIBS := \
	$(CIRCLEHOME)/lib/usb/libusb.a \
	$(CIRCLEHOME)/lib/input/libinput.a \
	$(CIRCLEHOME)/lib/fs/fat/libfatfs.a \
	$(CIRCLEHOME)/lib/fs/libfs.a \
	$(CIRCLEHOME)/lib/net/libnet.a \
	$(CIRCLEHOME)/lib/sched/libsched.a \
	$(CIRCLEHOME)/lib/libcircle.a
CIRCLE_LIBDIRS := lib lib/usb lib/input lib/fs lib/fs/fat lib/sched lib/net

# The portable mind core lists its sources in mind/sources.mk as MIND_SRCS,
# paths relative to this directory (.cpp or .c).
-include mind/sources.mk
MIND_OBJS := $(patsubst %.c,$(BUILD)/%.o,$(patsubst %.cpp,$(BUILD)/%.o,$(MIND_SRCS)))

OBJS := $(BUILD)/src/main.o $(BUILD)/src/kernel.o $(MIND_OBJS)
LIBS := $(CIRCLE_LIBS)

NETBOOT_SRC  := $(CIRCLEHOME)/sample/38-bootloader
NETBOOT_OBJS := $(BUILD)/netboot/main.o $(BUILD)/netboot/kernel.o \
		$(BUILD)/netboot/httpbootserver.o $(BUILD)/netboot/tftpbootserver.o
NETBOOT_GEN  := $(BUILD)/netboot/index.h $(BUILD)/netboot/style.h $(BUILD)/netboot/favicon.h

.DEFAULT_GOAL := all
.PHONY: all toolchain circle-config circle circle-clean boot

all: $(TARGET).img $(BUILD)/netboot/kernel7.img

# Dependencies come from -MMD below, not from Rules.mk's .d rules.
CHECK_DEPS := 0
include $(CIRCLEHOME)/Rules.mk

INCLUDE += -I mind

# Build id: the first 8 hex digits of the commit, as for the body
# (-DKILO_BUILD=0x<hash>, mind/kilo/tools/flash.py). build/build-id changes
# only when the hash does, so kernel.o is rebuilt only then.
KILO_BUILD := $(shell git rev-parse --short=8 HEAD 2>/dev/null || echo 0)
$(shell mkdir -p $(BUILD) && (echo $(KILO_BUILD) | cmp -s - $(BUILD)/build-id || echo $(KILO_BUILD) > $(BUILD)/build-id))
$(BUILD)/src/kernel.o: CPPFLAGS += -DKILO_BUILD=0x$(KILO_BUILD)
$(BUILD)/src/kernel.o: $(BUILD)/build-id

# Relink when the object list changes (a mind source added or removed).
$(shell mkdir -p $(BUILD) && (echo $(OBJS) | cmp -s - $(BUILD)/objs || echo $(OBJS) > $(BUILD)/objs))
$(TARGET).img: $(BUILD)/objs

toolchain:
	@test -x $(PREFIX)gcc || { echo "no toolchain at $(TOOLCHAIN) (see kernel/README.md)"; exit 1; }

# Circle's Config.mk is git-ignored inside the submodule. Generated here,
# rewritten only when its content changes.
define CONFIG_MK
RASPPI = $(RASPPI)
AARCH = $(AARCH)
PREFIX = $(PREFIX)
STDLIB_SUPPORT = $(STDLIB_SUPPORT)
endef
export CONFIG_MK
circle-config: toolchain
	@printf '%s\n' "$$CONFIG_MK" | cmp -s - $(CIRCLEHOME)/Config.mk || printf '%s\n' "$$CONFIG_MK" > $(CIRCLEHOME)/Config.mk

# The libraries the kernel and the loader link. Each directory has its own
# up-to-date check, so this is cheap when nothing changed.
circle: circle-config
	@$(MAKE) --no-print-directory -s -C $(CIRCLEHOME)/tools
	@for d in $(CIRCLE_LIBDIRS); do $(MAKE) --no-print-directory -s -C $(CIRCLEHOME)/$$d || exit 1; done

$(CIRCLE_LIBS): circle
	@:

circle-clean:
	@cd $(CIRCLEHOME) && ./makeall --nosample clean >/dev/null

boot: circle-config
	$(MAKE) -C $(CIRCLEHOME)/boot

$(OBJS) $(NETBOOT_OBJS): | circle-config

$(BUILD)/%.o: %.cpp
	@mkdir -p $(@D)
	@echo "  CPP   $@"
	@$(CPP) $(CPPFLAGS) -MMD -MP -c -o $@ $<

$(BUILD)/%.o: %.c
	@mkdir -p $(@D)
	@echo "  CC    $@"
	@$(CC) $(CFLAGS) $(C_STANDARD) -MMD -MP -c -o $@ $<

# Netboot loader: our kernel.cpp and main.cpp (no CSerialDevice) with the
# sample's HTTP and TFTP servers, compiled from the submodule unchanged.
$(BUILD)/netboot/%.o: netboot/%.cpp
	@mkdir -p $(@D)
	@echo "  CPP   $@"
	@$(CPP) $(CPPFLAGS) -I $(NETBOOT_SRC) -MMD -MP -c -o $@ $<

$(BUILD)/netboot/%.o: $(NETBOOT_SRC)/%.cpp
	@mkdir -p $(@D)
	@echo "  CPP   $@"
	@$(CPP) $(CPPFLAGS) -I $(BUILD)/netboot -I $(NETBOOT_SRC) -MMD -MP -c -o $@ $<

$(BUILD)/netboot/httpbootserver.o: $(NETBOOT_GEN)

$(BUILD)/netboot/%.h: $(NETBOOT_SRC)/%.html | circle
	@mkdir -p $(@D)
	@$(CIRCLEHOME)/tools/converttool $< > $@
$(BUILD)/netboot/%.h: $(NETBOOT_SRC)/%.css | circle
	@mkdir -p $(@D)
	@$(CIRCLEHOME)/tools/converttool $< > $@
$(BUILD)/netboot/%.h: $(NETBOOT_SRC)/%.ico | circle
	@mkdir -p $(@D)
	@$(CIRCLEHOME)/tools/converttool -b $< > $@

$(BUILD)/netboot/kernel7.img: $(NETBOOT_OBJS) $(LIBS) $(CIRCLEHOME)/circle.ld
	@echo "  LD    $(BUILD)/netboot/kernel7.elf"
	@$(LD) -o $(BUILD)/netboot/kernel7.elf -Map $(BUILD)/netboot/kernel7.map $(LDFLAGS) \
		-T $(CIRCLEHOME)/circle.ld $(CRTBEGIN) $(NETBOOT_OBJS) \
		--start-group $(LIBS) $(EXTRALIBS) --end-group $(CRTEND)
	@$(OBJCOPY) $(BUILD)/netboot/kernel7.elf -O binary $@
	@echo -n "  WC    $@ => "
	@wc -c < $@

-include $(OBJS:.o=.d) $(NETBOOT_OBJS:.o=.d)
