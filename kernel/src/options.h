//
// options.h: the build switches (kernel/Makefile "Build options", which
// states each one's default and passes all of them as -D). Included first
// by every file that tests one, so a switch missing from the command line
// stops the build instead of reading as 0 in an #if.
//
//   KILO_PLANNER_BENCH  time the walking planner once at boot
//   KILO_BEHAVIOUR      the behaviour runner from boot
//   KILO_REAR_UP        the remote's A1 rear-up key and its stage, 0 off
//                       (docs/rear-up-trick.md, the switch)
//   KILO_INTENT         stand, sit and rest as intents: the state estimate
//                       and the moves planned from the body model
//                       (mind/intent/intent.h, docs/intent.md; 0: postures
//                       change as before, the estimate only logs)
//   KILO_RIGHTING       the righting experiment: B1 rights him from a side
//                       or his back by moves planned on the body model
//                       (docs/righting.md; 0: B1 does nothing, he lies
//                       still after a fall: the body has no get-up)
//   KILO_RECORD_SAVE    the flight recorder's card saver from the start
//                       (mind/record/saver.h; else off until `kilo record
//                       save on --host`, remembered as kilorec/SAVE_ON)
//   KILO_LIGHT_SLEEP    light sleep: the servos off 3 s after A7, the Pi's
//                       idle loop after 30 s of nothing (mind/power/
//                       sleep.h, docs/power.md; 0: A7 is plain rest)
//   KILO_SOUND          the sounds on the 3.5 mm jack (src/sound.h, docs/
//                       sound.md; 0: every play request is a counted no-op)
//   KILO_MDNS           mDNS answers for kilo.local (src/net.h CKiloMdns)
//   KILO_BOOT_PROBE     GPIO21 marks the boot for a scope (src/bootprobe.h)
//
// KILO_BUILD (the commit's hash, kernel.o only) is not checked here.
//
#ifndef KILO_SRC_OPTIONS_H
#define KILO_SRC_OPTIONS_H

#if !defined(KILO_PLANNER_BENCH) || !defined(KILO_BEHAVIOUR) || !defined(KILO_REAR_UP) || \
	!defined(KILO_INTENT) || !defined(KILO_RIGHTING) || \
	!defined(KILO_RECORD_SAVE) || !defined(KILO_LIGHT_SLEEP) || !defined(KILO_SOUND) || \
	!defined(KILO_MDNS) || !defined(KILO_BOOT_PROBE)
#error "a KILO_ build switch is missing: kernel/Makefile defines them all (KILO_DEFS)"
#endif

#if KILO_REAR_UP < 0 || KILO_REAR_UP > 3
#error "KILO_REAR_UP must be 0, 1, 2 or 3"
#endif

#if KILO_INTENT < 0 || KILO_INTENT > 1
#error "KILO_INTENT must be 0 or 1"
#endif

#if KILO_RIGHTING < 0 || KILO_RIGHTING > 1
#error "KILO_RIGHTING must be 0 or 1"
#endif

#endif
