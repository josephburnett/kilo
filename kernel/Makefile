# Kilo mind kernel: a Circle bare-metal image for the Raspberry Pi 2 B.
#
#   make -C kernel               Circle libraries, build/kernel7.img, build/netboot/kernel7.img
#   make -C kernel boot          download the Raspberry Pi firmware into circle/boot (network)
#   make -C kernel card          stage both cards' files in build/card/{netboot,kernel}/ for copying by hand
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

# The SD card (body.bin, body.json): Circle's FatFs addon (long names, read
# and write) over its SD driver addon. lib/fs/fat, Circle's own FAT driver,
# reads 8.3 names only; lib/fs has the partition manager the SD driver uses.
CIRCLE_LIBS := \
	$(CIRCLEHOME)/addon/fatfs/libfatfs.a \
	$(CIRCLEHOME)/addon/SDCard/libsdcard.a \
	$(CIRCLEHOME)/lib/usb/libusb.a \
	$(CIRCLEHOME)/lib/input/libinput.a \
	$(CIRCLEHOME)/lib/fs/libfs.a \
	$(CIRCLEHOME)/lib/net/libnet.a \
	$(CIRCLEHOME)/lib/sched/libsched.a \
	$(CIRCLEHOME)/lib/libcircle.a
CIRCLE_LIBDIRS := lib lib/usb lib/input lib/fs lib/sched lib/net addon/fatfs addon/SDCard

# The portable mind core lists its sources in mind/sources.mk as MIND_SRCS,
# paths relative to this directory (.cpp or .c).
-include mind/sources.mk
MIND_OBJS := $(patsubst %.c,$(BUILD)/%.o,$(patsubst %.cpp,$(BUILD)/%.o,$(MIND_SRCS)))

OBJS := $(BUILD)/src/main.o $(BUILD)/src/kernel.o $(BUILD)/src/seams.o $(BUILD)/src/bootprobe.o \
	$(BUILD)/src/net.o $(BUILD)/src/plannerbench.o $(BUILD)/src/bootcard.o $(BUILD)/src/power.o $(MIND_OBJS)

# Build options (README "Body link", "Boot probe", "Planner bench", "mDNS",
# "The Mind's settings"):
#   KILO_BODY_SERIAL=pl011  the PL011 on GPIO14/15 (default): the Pi on the NyBoard's socket, its
#                           header UART and GPIO4 reset; USB (Ethernet) is scanned after the link is up
#   KILO_BODY_SERIAL=auto   a USB serial adapter if one enumerates, else the PL011: the fallback for
#                           Kilo's USB adapter cable (USB is scanned before the link)
#   KILO_BOOT_PROBE=1|0     GPIO21 (header pin 40) marks the boot for a scope (default 1)
#   KILO_PLANNER_BENCH=1|0  time the walking planner once at boot, no body involved (default 1)
#   KILO_MDNS=1|0           answer mDNS queries for kilo.local and the _kilo._tcp service (default 1)
#   KILO_BEHAVIOUR=0|1      run the behaviour runner from boot (default 0; the remote's A5 starts it too)
#   KILO_REAR_UP=0|1|2|3    the remote's A1 starts the rear_up trick at that stage: 1 rear_up (28 deg),
#                           2 rear_up_2 (40 deg), 3 rear_up_3 (54 deg) (default 0: A1 does nothing
#                           until the staged trials of docs/rear-up-trick.md are done)
#   KILO_INTENT=0|1         stand, sit and rest are intents (docs/intent.md): the remote's B6 and A7, the
#                           behaviour's postures and the guard's sit go through the state estimate and
#                           the moves planned from the body model (default 0: postures change as before,
#                           until the first floor try of docs/intent.md is done; the state estimate itself
#                           always runs and only logs)
#   KILO_RECORD_SAVE=0|1    save the flight recorder to the SD card from the start (default 0: off until
#                           `kilo bench sd --host` has measured the card and `kilo record save on --host`)
#   KILO_LIGHT_SLEEP=1|0    light sleep: the servos off 3 s after the remote's A7 (or 2 min lying idle in REST),
#                           the Pi's idle loop after 30 s more of nothing (default 1; 0: A7 is plain rest,
#                           the awake baseline of docs/power.md)
#   KILO_SOUND=1|0          Kilo's sounds on the 3.5 mm jack (docs/sound.md): kernel/sounds/*.wav packed into the
#                           image, Circle's PWM sound device (default 1; 0: no sound code, no sounds, no libsound)
#   KILO_SOUND_IDLE=stop|run  the sound device between sounds: stopped, started for each sound with a ramp
#                           (default), or running at silence from boot (src/sound.h)
KILO_BODY_SERIAL ?= pl011
KILO_BOOT_PROBE ?= 1
KILO_PLANNER_BENCH ?= 1
KILO_MDNS ?= 1
KILO_BEHAVIOUR ?= 0
KILO_REAR_UP ?= 0
KILO_INTENT ?= 0
KILO_RECORD_SAVE ?= 0
KILO_LIGHT_SLEEP ?= 1
KILO_SOUND ?= 1
KILO_SOUND_IDLE ?= stop
ifeq ($(filter 0 1 2 3,$(KILO_REAR_UP)),)
$(error KILO_REAR_UP must be 0, 1, 2 or 3)
endif
ifeq ($(filter 0 1,$(KILO_INTENT)),)
$(error KILO_INTENT must be 0 or 1)
endif
ifeq ($(filter 0 1,$(KILO_SOUND)),)
$(error KILO_SOUND must be 0 or 1)
endif
ifeq ($(filter stop run,$(KILO_SOUND_IDLE)),)
$(error KILO_SOUND_IDLE must be stop or run)
endif
ifeq ($(KILO_BODY_SERIAL),pl011)
KILO_DEFS := -DKILO_BODY_SERIAL_PL011
else ifneq ($(KILO_BODY_SERIAL),auto)
$(error KILO_BODY_SERIAL must be auto or pl011)
endif
KILO_DEFS += -DKILO_BOOT_PROBE=$(KILO_BOOT_PROBE) -DKILO_PLANNER_BENCH=$(KILO_PLANNER_BENCH) -DKILO_MDNS=$(KILO_MDNS) \
	-DKILO_BEHAVIOUR=$(KILO_BEHAVIOUR) -DKILO_REAR_UP=$(KILO_REAR_UP) -DKILO_INTENT=$(KILO_INTENT) -DKILO_RECORD_SAVE=$(KILO_RECORD_SAVE) -DKILO_LIGHT_SLEEP=$(KILO_LIGHT_SLEEP) \
	-DKILO_SOUND=$(KILO_SOUND) -DKILO_SOUND_KEEP_RUNNING=$(if $(filter run,$(KILO_SOUND_IDLE)),1,0)

# Kilo's sounds (docs/sound.md): the PWM sound device (src/sound.cpp, Circle's lib/sound) and the
# table tools/pack_sounds.py packs from sounds/*.wav into build/gen/sounds.gen.cpp, every build (it
# rewrites the table only when it changed, prints the size report, and stops the build when a file
# cannot be used or the sounds are over the budget). KILO_SOUND=0: none of it.
ifeq ($(KILO_SOUND),1)
CIRCLE_LIBS := $(CIRCLEHOME)/lib/sound/libsound.a $(CIRCLE_LIBS)
CIRCLE_LIBDIRS += lib/sound
OBJS += $(BUILD)/src/sound.o $(BUILD)/gen/sounds.gen.o
endif
LIBS := $(CIRCLE_LIBS)

NETBOOT_SRC  := $(CIRCLEHOME)/sample/38-bootloader
NETBOOT_OBJS := $(BUILD)/netboot/main.o $(BUILD)/netboot/kernel.o \
		$(BUILD)/netboot/httpbootserver.o $(BUILD)/netboot/tftpbootserver.o
NETBOOT_GEN  := $(BUILD)/netboot/index.h $(BUILD)/netboot/style.h $(BUILD)/netboot/favicon.h

.DEFAULT_GOAL := all
.PHONY: all toolchain circle-config circle circle-clean boot card

all: $(TARGET).img $(BUILD)/netboot/kernel7.img

# Dependencies come from -MMD below, not from Rules.mk's .d rules.
CHECK_DEPS := 0
include $(CIRCLEHOME)/Rules.mk

INCLUDE += -I mind

# The whole mind core as on the host (kernel/host/Makefile FPFLAGS), so the
# maths the host tests check is the maths that runs here: no fused
# multiply-add (the planner's slot-end prediction must equal what it then
# computes; perception and behaviour compute in double as Python does, and
# their golden tests compare against Python's rounding), sqrt as an
# instruction.
FPFLAGS := -ffp-contract=off -fno-math-errno
$(BUILD)/mind/%.o: CPPFLAGS += $(FPFLAGS)

# Build id (build-id.mk): the kernel logs it and serves it as
# STATUS.mind_build (kernel/host's KILO_MIND_BUILD, same hash).
BUILD_ID_FILE := $(BUILD)/build-id
include build-id.mk
KILO_BUILD := $(BUILD_ID)
$(BUILD)/src/kernel.o: CPPFLAGS += -DKILO_BUILD=0x$(KILO_BUILD)
$(BUILD)/src/kernel.o: $(BUILD)/build-id

# Who may talk to Kilo (README "Who may talk to Kilo: keys"): the laptops'
# public keys, compiled into kernel.o from KILO_AUTHORIZED_KEYS (default
# keys/authorized_keys, committed; a path relative to kernel/, or absolute:
# a test build uses a throwaway list). keys/gen_keys.py runs every build,
# rewrites the header only when the list changed, and stops the build, saying
# what to do, while the list has no key.
KILO_AUTHORIZED_KEYS ?= keys/authorized_keys
KEYS_SHOWN := $(if $(filter keys/authorized_keys,$(KILO_AUTHORIZED_KEYS)),kernel/keys/authorized_keys,$(KILO_AUTHORIZED_KEYS))
KEYS_H := $(BUILD)/gen/authorized_keys.h
.PHONY: kilo-keys
kilo-keys:
	@:
$(KEYS_H): kilo-keys
	@python3 keys/gen_keys.py "$(KILO_AUTHORIZED_KEYS)" $@ --shown-as "$(KEYS_SHOWN)"
$(BUILD)/src/kernel.o: $(KEYS_H)
$(BUILD)/src/kernel.o: CPPFLAGS += -I $(BUILD)/gen

# KILO_SOUNDS_DIR: where the .wav files are (default sounds/, i.e. kernel/sounds; a path relative to
# kernel/, or absolute: a test build with other sounds).
KILO_SOUNDS_DIR ?= sounds
SOUNDS_SHOWN := $(if $(filter sounds,$(KILO_SOUNDS_DIR)),kernel/sounds,$(KILO_SOUNDS_DIR))
SOUNDS_GEN := $(BUILD)/gen/sounds.gen.cpp
.PHONY: kilo-sounds
kilo-sounds:
	@:
$(SOUNDS_GEN): kilo-sounds
	@mkdir -p $(@D)
	@python3 tools/pack_sounds.py "$(KILO_SOUNDS_DIR)" $@ --shown-as "$(SOUNDS_SHOWN)"
$(BUILD)/gen/sounds.gen.o: $(SOUNDS_GEN) mind/sound/sound.h
	@echo "  CPP   $@"
	@$(CPP) $(CPPFLAGS) -c -o $@ $<

# The options reach src/ only; build/options changes when they do, so
# switching them rebuilds what reads them.
$(shell mkdir -p $(BUILD) && (echo $(KILO_DEFS) | cmp -s - $(BUILD)/options || echo $(KILO_DEFS) > $(BUILD)/options))
$(BUILD)/src/%.o: CPPFLAGS += $(KILO_DEFS)
$(filter $(BUILD)/src/%,$(OBJS)): $(BUILD)/options

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
	@for d in addon/fatfs addon/SDCard; do $(MAKE) --no-print-directory -s -C $(CIRCLEHOME)/$$d clean >/dev/null; done

boot: circle-config
	$(MAKE) -C $(CIRCLEHOME)/boot

# The two cards' contents as plain folders, for a computer that has the card
# reader but not this checkout (rtb has no reader): copy a folder's files onto
# an empty FAT card. tools/make-sd.sh does the same with its checks when the
# card is mounted here.
card: all
	@for mode in netboot kernel; do \
		d=$(BUILD)/card/$$mode; rm -rf $$d; mkdir -p $$d; \
		tools/make-sd.sh --force-dir $$d --$$mode >/dev/null || exit 1; \
		echo "  CARD  $$d: $$(ls $$d | tr '\n' ' ')"; \
	done

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

# (not the sounds table's: its .d would be remade, and make restarted, every build: its one header is above)
-include $(filter-out $(BUILD)/gen/sounds.gen.d,$(OBJS:.o=.d)) $(NETBOOT_OBJS:.o=.d)
