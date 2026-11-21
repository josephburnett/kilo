//
// options.h: the build switches (kernel/Makefile "Build options", which
// states each one's default and passes all of them as -D). Included first
// by every file that tests one, so a switch missing from the command line
// stops the build instead of reading as 0 in an #if.
//
//   KILO_PLANNER_BENCH  time the walking planner once at boot
//   KILO_BEHAVIOUR      the behaviour runner from boot
//   KILO_REAR_UP        the remote's A2 rear-up key and its stage, 0 off
//                       (docs/rear-up-trick.md, the switch)
//   KILO_RECORD_SAVE    the flight recorder's card saver from the start
//                       (mind/record/saver.h; else off until `kilo record
//                       save on --host`, remembered as kilorec/SAVE_ON)
//   KILO_LIGHT_SLEEP    light sleep: the servos off 3 s after A7, the Pi's
//                       idle loop after 30 s of nothing (mind/power/
//                       sleep.h, docs/power.md; 0: A7 is plain rest)
//   KILO_SOUND          the sounds on the 3.5 mm jack (src/sound.h, docs/
//                       sound.md; 0: every play request is a counted no-op)
//   KILO_SOUND_KEEP_RUNNING  the sound device runs between sounds
//   KILO_MDNS           mDNS answers for kilo.local (src/net.h CKiloMdns)
//   KILO_BOOT_PROBE     GPIO21 marks the boot for a scope (src/bootprobe.h)
//
// KILO_BUILD (the commit's hash, kernel.o only) and KILO_BODY_SERIAL_PL011
// (defined or not) are not checked here.
//
#ifndef KILO_SRC_OPTIONS_H
#define KILO_SRC_OPTIONS_H

#if !defined(KILO_PLANNER_BENCH) || !defined(KILO_BEHAVIOUR) || !defined(KILO_REAR_UP) || \
	!defined(KILO_RECORD_SAVE) || !defined(KILO_LIGHT_SLEEP) || !defined(KILO_SOUND) || \
	!defined(KILO_SOUND_KEEP_RUNNING) || !defined(KILO_MDNS) || !defined(KILO_BOOT_PROBE)
#error "a KILO_ build switch is missing: kernel/Makefile defines them all (KILO_DEFS)"
#endif

#if KILO_REAR_UP < 0 || KILO_REAR_UP > 3
#error "KILO_REAR_UP must be 0, 1, 2 or 3"
#endif

#endif
