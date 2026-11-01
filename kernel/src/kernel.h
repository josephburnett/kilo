//
// kernel.h: the Kilo mind kernel (P1: the body link).
//
#ifndef KILO_KERNEL_H
#define KILO_KERNEL_H

#include "seams.h"
#include "link/serial_link.h"
#include <circle/actled.h>
#include <circle/bcmrandom.h>
#include <circle/devicenameservice.h>
#include <circle/exceptionhandler.h>
#include <circle/interrupt.h>
#include <circle/koptions.h>
#include <circle/logger.h>
#include <circle/nulldevice.h>
#include <circle/screen.h>
#include <circle/sched/scheduler.h>
#include <circle/timer.h>
#include <circle/types.h>
#include <circle/usb/usbhcidevice.h>

enum TShutdownMode
{
	ShutdownNone,
	ShutdownHalt,
	ShutdownReboot
};

// What the main loop learns from the link between two stats lines.
struct CLinkWatch : kilo::LinkListener
{
	void on_hello (const kilo_hello_t &rHello, bool bSolicited) override;
	void on_telemetry (const kilo_telemetry_t &rTelemetry) override;

	CircleClock *m_pClock = 0;
	kilo::Log *m_pLog = 0;
	uint64_t m_nBootUs = 0;			// clock at the top of the kernel
	bool m_bFirstHello = false;
	unsigned m_nTelemetry = 0;		// in the current second
	unsigned m_nOverruns = 0;		// TELEMETRY with the OVERRUN flag, current second
};

class CKernel
{
public:
	CKernel (void);

	boolean Initialize (void);
	TShutdownMode Run (void);

private:
	kilo::Serial *ChooseSerial (void);
	void LogStats (void);
	void UpdateLED (uint64_t nNowUs);

private:
	// Construction order matters (Circle's samples use the same order).
	// No CSerialDevice member: the PL011 on GPIO14/15 is built only when it
	// is the body link (ChooseSerial).
	CActLED			m_ActLED;
	CKernelOptions		m_Options;
	CDeviceNameService	m_DeviceNameService;
	CScreenDevice		m_Screen;
	CNullDevice		m_Null;
	CExceptionHandler	m_ExceptionHandler;
	CInterruptSystem	m_Interrupt;
	CTimer			m_Timer;
	CLogger			m_Logger;
	CScheduler		m_Scheduler;
	CUSBHCIDevice		m_USBHCI;
	CBcmRandomNumberGenerator m_Random;

	CircleClock		m_Clock;
	CircleLog		m_Log;
	CircleResetGpio		m_ResetGpio;	// GPIO4 released at once; P4 pulses it
	Pl011Serial		*m_pPl011;	// the body link's device: one of these two
	UsbSerial		*m_pUsbSerial;
	kilo::Serial		*m_pSerial;
	const char		*m_pSerialName;
	kilo::SerialLink	*m_pLink;	// built once the Serial is chosen
	CLinkWatch		m_Watch;
};

#endif
