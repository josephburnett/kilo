// hal/eyes.cpp on fake registers: D7 an output, six LEDs of green, red,
// blue (WS2812 order) with the mask's LEDs lit and the others off, every
// byte sent with interrupts off, and SREG restored after each LED. (The bit
// timing itself is the AVR asm; on the host each byte is recorded.)
#include "../check.h"
#include "fake_avr.h"
#include "../../src/hal/eyes.cpp"

int main() {
  DDRD = 0;
  eyes_init();
  CHECK(DDRD & _BV(7));
  SREG = 0x80;  // interrupts on, as in the firmware
  eyes_test_n = eyes_test_irq_on = 0;
  eyes_show(0x3F, 1, 2, 3);
  CHECK(eyes_test_n == 18 && eyes_test_irq_on == 0 && (SREG & 0x80));
  for (int i = 0; i < 6; i++)
    CHECK(eyes_test_bytes[3 * i] == 2 && eyes_test_bytes[3 * i + 1] == 1 && eyes_test_bytes[3 * i + 2] == 3);
  eyes_test_n = 0;
  eyes_show(0x07, 9, 8, 7);  // the right eye only
  CHECK(eyes_test_n == 18 && eyes_test_bytes[0] == 8 && eyes_test_bytes[8] == 7 && eyes_test_bytes[9] == 0 &&
        eyes_test_bytes[17] == 0);
  return report("hal_eyes");
}
