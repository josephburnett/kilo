//
// main.cpp: entry point, as in Circle's samples.
//
#include "kernel.h"
#include "bootprobe.h"
#include <circle/startup.h>

int main (void)
{
	BootProbeStart ();	// first: the scope's "kernel running" edge

	CKernel Kernel;
	if (!Kernel.Initialize ())
	{
		halt ();
		return EXIT_HALT;
	}

	TShutdownMode ShutdownMode = Kernel.Run ();

	switch (ShutdownMode)
	{
	case ShutdownReboot:
		reboot ();
		return EXIT_REBOOT;

	case ShutdownHalt:
	default:
		halt ();
		return EXIT_HALT;
	}
}
