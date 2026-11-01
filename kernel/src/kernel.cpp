//
// kernel.cpp: the Kilo mind kernel (P0: hello world, blinks the ACT LED).
//
// Logs go to the HDMI screen if the firmware gives us a frame buffer, else
// nowhere. Kilo has no monitor; the LED is the observable result.
//
#include "kernel.h"

#ifndef KILO_BUILD
#define KILO_BUILD 0
#endif

static const char FromKernel[] = "kilo";

CKernel::CKernel (void)
:	m_Screen (m_Options.GetWidth (), m_Options.GetHeight ()),
	m_Timer (&m_Interrupt),
	m_Logger (m_Options.GetLogLevel (), &m_Timer)
{
	m_ActLED.On ();		// first sign of life, before anything can fail
}

boolean CKernel::Initialize (void)
{
	CDevice *pLog = &m_Null;
	if (m_Screen.Initialize ())
	{
		pLog = &m_Screen;
	}

	return    m_Logger.Initialize (pLog)
	       && m_Interrupt.Initialize ()
	       && m_Timer.Initialize ();
}

TShutdownMode CKernel::Run (void)
{
	m_Logger.Write (FromKernel, LogNotice, "Kilo mind, build %08x", (unsigned) KILO_BUILD);

	for (;;)		// 2 Hz: 250 ms on, 250 ms off
	{
		m_ActLED.On ();
		m_Timer.MsDelay (250);
		m_ActLED.Off ();
		m_Timer.MsDelay (250);
	}

	return ShutdownHalt;
}
