//
// bootprobe.cpp: see bootprobe.h.
//
#include "bootprobe.h"
#include <circle/gpiopin.h>

#if KILO_BOOT_PROBE

// Built before main() by the default constructor, which touches no
// hardware; BootProbeStart() sets the pin up.
static CGPIOPin s_Probe;
static bool s_bHelloSeen = false;

void BootProbeStart (void)
{
	s_Probe.AssignPin (KILO_BOOT_PROBE_PIN);
	s_Probe.SetMode (GPIOModeInput, FALSE);	// Write() needs a GPIO mode
	s_Probe.Write (HIGH);			// latch high, then drive: one clean edge
	s_Probe.SetMode (GPIOModeOutput, FALSE);
}

void BootProbeHello (void)
{
	if (!s_bHelloSeen)
	{
		s_bHelloSeen = true;
		s_Probe.Write (LOW);
	}
}

#else

void BootProbeStart (void) {}
void BootProbeHello (void) {}

#endif
