// eyes: the six WS2812 LEDs of the RGB ultrasonic module, data on D7
// (legacy RgbUltrasonic(6, 7): the sonar on D6, the LEDs on D7, Grove
// socket G4; the legacy src/rgbUltrasonic/RGBLed.cpp, in git history, drove them the same way).
//
// SET_EYES (core/link.cpp keeps the last one in link_eyes) is shown from
// the tick (app/main.cpp) when a new one came and no echo is being timed.
// Each LED's 24 bits go out with interrupts off (about 33 us: 22 cycles a
// bit at 16 MHz), interrupts back on between LEDs, so nothing waits more
// than one LED: the UART holds 2 bytes and a byte takes 40 us at 250000
// baud, Timer0 keeps its overflow flag, the IR's edges are stamped at most
// 33 us late. A gap between LEDs (an ISR) shorter than the LEDs' reset time
// (50 us, 280 us on the newer WS2812B) keeps the frame going; a longer one
// latches early and the next LED's bits land on the first LED again, which
// with one colour on all six (the mind's 0x3F) shows the same colour.
#ifndef KILO_HAL_EYES_H
#define KILO_HAL_EYES_H
#include <stdint.h>

void eyes_init(void);
// LEDs whose bit is set in mask show r, g, b; the others are off.
void eyes_show(uint8_t mask, uint8_t r, uint8_t g, uint8_t b);

#endif
