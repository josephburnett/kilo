#include "eyes.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../features.h"

#if KILO_EYES
// D7 = PD7. PORTD is shared with the buzzer ISR's D5 toggle: only sbi/cbi
// here (hal/gpio.h).
void eyes_init(void) { DDRD |= _BV(7); }

// One byte, MSB first, interrupts already off. Cycles after the sbi: a 0
// goes low after 5 (312 ns), a 1 after 12 (750 ns); a bit is 22 (1.375 us).
#ifndef __AVR__
// The host HAL tests (body/test/hal/test_eyes.cpp): the bytes as sent, and
// whether interrupts were off for each.
uint8_t eyes_test_bytes[18], eyes_test_n, eyes_test_irq_on;
static void ws_byte(uint8_t b) {
  if (eyes_test_n < sizeof eyes_test_bytes) eyes_test_bytes[eyes_test_n++] = b;
  if (SREG & 0x80) eyes_test_irq_on++;
}
#else
static void ws_byte(uint8_t b) {
  uint8_t n = 8;
  asm volatile(
      "1: sbi %[port], 7\n\t"
      "nop\n\t"
      "nop\n\t"
      "sbrs %[b], 7\n\t"
      "cbi %[port], 7\n\t"
      "rjmp .+0\n\t"
      "rjmp .+0\n\t"
      "rjmp .+0\n\t"
      "cbi %[port], 7\n\t"
      "lsl %[b]\n\t"
      "rjmp .+0\n\t"
      "nop\n\t"
      "nop\n\t"
      "dec %[n]\n\t"
      "brne 1b\n\t"
      : [b] "+r"(b), [n] "+r"(n)
      : [port] "I"(_SFR_IO_ADDR(PORTD)));
}
#endif

void eyes_show(uint8_t mask, uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 0; i < 6; i++, mask >>= 1) {
    uint8_t on = mask & 1u;
    uint8_t sreg = SREG;
    cli();
    ws_byte(on ? g : 0);  // WS2812 order: green, red, blue
    ws_byte(on ? r : 0);
    ws_byte(on ? b : 0);
    SREG = sreg;
  }
}
#endif
