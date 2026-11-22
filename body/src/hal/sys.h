// sys: reset record in .noinit RAM, hardware watchdog, stack painting.
#ifndef KILO_HAL_SYS_H
#define KILO_HAL_SYS_H
#include <stdint.h>
#include "../core/reset.h"

// Survives resets that keep power: the C runtime neither zeroes nor paints
// it. It sits at a FIXED address, the top 10 bytes of RAM (0x8F6-0x8FF),
// with the stack starting just below it (__stack = 0x8F5); both are set by
// the linker flags in body/tools/build_flags.sh and checked by
// size_check.sh. It must not move between builds: the new firmware reads
// the record the old one wrote. (Until 2026-11-22 it was in .noinit, right
// after .bss, so any build whose .data + .bss changed size read the old
// record a few bytes off and called a flash a cold boot; docs/architecture.md §3a.)
#define SYS_RESET_RECORD_ADDR 0x8F6
extern ResetRecord sys_reset_record;

// Hardware watchdog: 250 ms, armed only after the IMU bring-up (which takes
// ~300 ms of legitimately blocking I2C). If the loop ever hangs, the AVR
// resets, the PCA9685 keeps the pose, and the next boot is a warm start.
void sys_wdt_arm(void);
void sys_wdt_kick(void);
// REBOOT: mark the record, then let the watchdog reset us in 15 ms.
void sys_reboot(void) __attribute__((noreturn));

// The stack grows down from the top of RAM towards the globals. At reset,
// before anything runs, all free RAM is painted with 0xC5; the deepest the
// stack has ever reached is where the paint stops (STATS stack_free_min).
uint16_t sys_stack_free_min(void);

#endif
