//
// kernel.h: the Kilo mind kernel (P0: hello world, blinks the ACT LED).
//
#ifndef KILO_KERNEL_H
#define KILO_KERNEL_H

#include <circle/actled.h>
#include <circle/devicenameservice.h>
#include <circle/exceptionhandler.h>
#include <circle/interrupt.h>
#include <circle/koptions.h>
#include <circle/logger.h>
#include <circle/nulldevice.h>
#include <circle/screen.h>
#include <circle/timer.h>
#include <circle/types.h>

enum TShutdownMode
{
	ShutdownNone,
	ShutdownHalt,
	ShutdownReboot
};

class CKernel
{
public:
	CKernel (void);

	boolean Initialize (void);
	TShutdownMode Run (void);

private:
	// Construction order matters (Circle's samples use the same order).
	// No CSerialDevice: GPIO14/15 (PL011) belong to the body link.
	CActLED			m_ActLED;
	CKernelOptions		m_Options;
	CDeviceNameService	m_DeviceNameService;
	CScreenDevice		m_Screen;
	CNullDevice		m_Null;
	CExceptionHandler	m_ExceptionHandler;
	CInterruptSystem	m_Interrupt;
	CTimer			m_Timer;
	CLogger			m_Logger;
};

#endif
