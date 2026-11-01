//
// kernel.h: the Kilo mind kernel: the body link (P1), the laptop interface
// over Ethernet (P2), the walking planner (P3) and flashing the body (P4).
//
#ifndef KILO_KERNEL_H
#define KILO_KERNEL_H

#include "seams.h"
#include "bootcard.h"
#include "net.h"
#include "plannerbench.h"
#include "flash/flasher.h"
#include "flash/image.h"
#include "link/serial_link.h"
#include "net/host_server.h"
#include "planner/walker.h"
#include "util/logring.h"
#include <circle/actled.h>
#include <circle/bcmrandom.h>
#include <circle/devicenameservice.h>
#include <circle/exceptionhandler.h>
#include <circle/interrupt.h>
#include <circle/koptions.h>
#include <circle/logger.h>
#include <circle/net/netsubsystem.h>
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

// What the main loop learns from the link between two stats lines. It is
// the link's one listener, so it also feeds the walker.
struct CLinkWatch : kilo::LinkListener
{
	void on_hello (const kilo_hello_t &rHello, bool bSolicited) override;
	void on_telemetry (const kilo_telemetry_t &rTelemetry) override;
	void on_event (const kilo_event_t &rEvent) override;

	CircleClock *m_pClock = 0;
	kilo::Log *m_pLog = 0;
	kilo::planner::Walker *m_pWalker = 0;
	uint64_t m_nBootUs = 0;			// clock at the top of the kernel
	bool m_bFirstHello = false;		// the boot-time flash check waits for it
	unsigned m_nTelemetry = 0;		// in the current second
	unsigned m_nOverruns = 0;		// TELEMETRY with the OVERRUN flag, current second
};

// Durations within one stats second, in microseconds.
struct CTiming
{
	unsigned m_nCount = 0;
	unsigned m_nMax = 0;
	u64 m_nSum = 0;

	void Add (unsigned nUs)
	{
		m_nCount++;
		m_nSum += nUs;
		if (nUs > m_nMax)
		{
			m_nMax = nUs;
		}
	}
	unsigned Mean (void) const	{ return m_nCount != 0 ? (unsigned) (m_nSum / m_nCount) : 0; }
	void Reset (void)		{ m_nCount = 0; m_nMax = 0; m_nSum = 0; }
};

class CKernel
{
public:
	CKernel (void);

	boolean Initialize (void);
	TShutdownMode Run (void);

private:
	kilo::Serial *ChooseSerial (void);
	void BootFlash (uint64_t nNowUs);
	void WatchFlash (void);
	void DrainCircleLog (void);
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
	CNetSubSystem		m_Net;		// DHCP as "kilo"; the net task starts it
	CBcmRandomNumberGenerator m_Random;

	CircleClock		m_Clock;
	CircleLog		m_CircleLog;	// into CLogger: the screen, or nowhere
	kilo::LogRing		*m_pRing;	// the last lines, for `kilo log --host`
	KernelLog		*m_pLog;	// every core line: the ring, then CLogger
	CircleResetGpio		m_ResetGpio;	// GPIO4 released at once; P4 pulses it
	Pl011Serial		*m_pPl011;	// the body link's device: one of these two
	UsbSerial		*m_pUsbSerial;
	kilo::Serial		*m_pSerial;
	const char		*m_pSerialName;
	kilo::SerialLink	*m_pLink;	// built once the Serial is chosen
	CLinkWatch		m_Watch;

	CircleListener		*m_pListener;	// the network seams, served by the net task
	CKiloNet		*m_pNetTask;
	kilo::HostServer	*m_pHost;
	kilo::planner::Walker	*m_pWalker;
	CPlannerBench		*m_pBench;	// 0 once done, or built without it

	// Flashing the body (P4): the laptop's FLASH_* through the host server,
	// and the image on the SD card at boot.
	kilo::Flasher		*m_pFlasher;	// polled by the host server
	CBootCard		*m_pBootCard;	// body.bin, body.json on the FAT partition
	kilo::FlashImage	*m_pHostImage;	// 32 KB: FLASH_BEGIN..COMMIT fills it
	kilo::FlashImage	*m_pCardImage;	// 32 KB: the SD card's, at boot
	enum TBootFlash
	{
		BootFlashWait,			// for the first HELLO, or BootHelloWaitUs
		BootFlashRunning,		// flashing the SD card's image
		BootFlashDone
	};
	TBootFlash		m_BootFlash;
	uint64_t		m_nLinkUpUs;	// link.connect() at boot
	boolean			m_bGoAhead;	// the boot flash check is over: the net task may scan USB
	boolean			m_bWasFlashing;

	CTiming			m_Period;	// pass to pass: the 1 ms sleep and the other tasks
	CTiming			m_Work;		// one pass's own work
	CTiming			m_WalkerPoll;	// Walker::poll()
};

#endif
