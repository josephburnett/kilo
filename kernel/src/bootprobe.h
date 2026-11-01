//
// bootprobe.h: a GPIO for timing the boot with a scope (docs/fast-boot.md P6).
//
// GPIO21, header pin 40 (pin 39 next to it is ground): outside the pins 1-10
// the NyBoard's socket mates, and used by nothing else here. It goes high
// at the top of main(), the first code of ours that runs, and low when the
// first HELLO arrives from the body. Against the 5 V rail: power to the
// rising edge is the GPU firmware plus Circle's start-up; the high time is
// our initialisation (USB enumeration included) plus the body's boot.
//
// Build with KILO_BOOT_PROBE=0 (make -C kernel KILO_BOOT_PROBE=0) to leave
// the pin alone.
//
#ifndef KILO_SRC_BOOTPROBE_H
#define KILO_SRC_BOOTPROBE_H

#ifndef KILO_BOOT_PROBE
#define KILO_BOOT_PROBE 1
#endif

#define KILO_BOOT_PROBE_PIN	21

void BootProbeStart (void);	// pin high (main())
void BootProbeHello (void);	// pin low (first HELLO); later calls do nothing

#endif
