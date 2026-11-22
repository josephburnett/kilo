#include "sys.h"
#include <avr/io.h>
#include <avr/wdt.h>

// On the AVR the record has no definition here: the linker puts the symbol
// at SYS_RESET_RECORD_ADDR (body/tools/build_flags.sh), so a build without
// those flags does not link. On the host (the tests) it is a plain global.
#ifdef __AVR__
static_assert(SYS_RESET_RECORD_ADDR + sizeof(ResetRecord) == RAMEND + 1,
              "the reset record fills the top of RAM, above the stack");
#else
ResetRecord sys_reset_record;
#endif

// The two .initN hooks below are AVR start-up code (naked, AVR assembly);
// the host tests (body/test/hal) compile the rest of this file.
#ifdef __AVR__
// Runs in .init3, before main() and before the C runtime clears RAM. After
// a watchdog reset the watchdog stays enabled (at its shortest timeout on
// some parts), so turn it off first thing. MCUSR is already 0 here: optiboot
// cleared it (which is why core/reset.h exists).
extern "C" void sys_wdt_off_early(void) __attribute__((naked, used, section(".init3")));
extern "C" void sys_wdt_off_early(void) {
  MCUSR = 0;
  wdt_disable();
}

// Runs in .init1: paint from the end of the globals (_end) to the top of
// the stack (__stack, just below the reset record, which is not painted). Plain asm because there is no stack yet
// to speak of and nothing in C is set up.
extern "C" void sys_paint_stack(void) __attribute__((naked, used, section(".init1")));
extern "C" void sys_paint_stack(void) {
  __asm__ volatile(
      "  ldi r30, lo8(_end)\n"
      "  ldi r31, hi8(_end)\n"
      "  ldi r24, 0xc5\n"
      "  ldi r25, hi8(__stack)\n"
      "  rjmp 2f\n"
      "1:\n"
      "  st Z+, r24\n"
      "2:\n"
      "  cpi r30, lo8(__stack)\n"
      "  cpc r31, r25\n"
      "  brlo 1b\n"
      "  breq 1b\n" ::);
}
#endif

extern uint8_t _end;
extern uint8_t __stack;

uint16_t sys_stack_free_min(void) {
  const uint8_t* p = &_end;
  while (p <= &__stack && *p == 0xC5) p++;
  return (uint16_t)(p - &_end);
}

void sys_wdt_arm(void) { wdt_enable(WDTO_250MS); }
void sys_wdt_kick(void) { wdt_reset(); }

void sys_reboot(void) {
  sys_reset_record.reason = RESET_REASON_REQUESTED;
  wdt_enable(WDTO_15MS);
  for (;;) {
  }
}
