//
// kernel.cpp: the Kilo mind kernel (docs/fast-boot.md): the body link (P1),
// the laptop interface over Ethernet (P2) and the walking planner (P3).
//
// It exchanges HELLO with the body, keeps the link fed (HEARTBEAT, HELLO_REQ
// while the body is lost), serves `kilo --host` on TCP port 5300 from a net
// task in the background, polls the walker, logs statistics once a second
// and shows the link's state on the ACT LED.
//
// Logs go into a RAM ring that `kilo log --host` reads, and to the HDMI
// screen if the firmware gives us a frame buffer, else nowhere (Kilo has no
// monitor). The LED:
//
//   steady on      starting up (the USB scan takes about a second or more)
//   0.5 Hz blink   no body: no HELLO yet, or no TELEMETRY for 300 ms
//   2 Hz blink     the body is alive: TELEMETRY flowing
//   5 Hz blink     the body speaks another protocol major: not driven (flash it)
//
#include "kernel.h"
#include "bootprobe.h"
#include "link/names.h"
#include "util/fmt.h"
#include "util/log.h"

#ifndef KILO_BUILD
#define KILO_BUILD 0
#endif

#ifndef KILO_PLANNER_BENCH
#define KILO_PLANNER_BENCH 1
#endif

using namespace kilo;
using namespace kilo::planner;

// The main loop's period. The link wants poll() at least every ~10 ms (the
// heartbeat is due after 100 ms of silence, a command is retried after
// 50 ms) and the walker wakes on a 20 ms grid; 1 ms keeps the RX ring short
// and the walker's wake within 1 ms of its deadline, and gives the net task
// its turn in every sleep.
static const unsigned LoopPeriodUs = 1000;

// After the Serial failed: how often the link tries again.
static const uint64_t ReconnectUs = 1000000;

// No TELEMETRY this long: the body is lost (the link's own rule).
static const uint32_t BodyLostUs = KILO_BODY_LOST_MS * 1000u;

// The planner bench starts this long after the first laptop client
// connects, so that a `kilo log --host` tail sees its lines live: the RAM
// ring holds about 20 s of log, and nobody reads the screen.
static const uint64_t BenchDelayUs = 1000000;

// Circle's own log events copied into the ring per pass, at most.
static const unsigned CircleEventsPerPass = 4;

void CLinkWatch::on_hello (const kilo_hello_t &rHello, bool bSolicited)
{
	if (m_bFirstHello)
	{
		return;
	}
	m_bFirstHello = true;
	BootProbeHello ();
	uint64_t nUs = m_pClock->now_us () - m_nBootUs;
	logf (*m_pLog, LogLevel::Info, "first HELLO %u.%03u s after the kernel started (%s)",
	      (unsigned) (nUs / 1000000u), (unsigned) (nUs / 1000u % 1000u),
	      bSolicited ? "answering HELLO_REQ" : "unsolicited: the body booted");
}

void CLinkWatch::on_telemetry (const kilo_telemetry_t &rTelemetry)
{
	m_nTelemetry++;
	if (rTelemetry.flags & KILO_TELEMETRY_FLAGS_OVERRUN)
	{
		m_nOverruns++;
	}
	m_pWalker->on_telemetry (rTelemetry);
}

void CLinkWatch::on_event (const kilo_event_t &rEvent)
{
	m_pWalker->on_event (rEvent);
}

CKernel::CKernel (void)
:	m_Screen (m_Options.GetWidth (), m_Options.GetHeight ()),
	m_Timer (&m_Interrupt),
	m_Logger (m_Options.GetLogLevel (), &m_Timer),
	m_USBHCI (&m_Interrupt, &m_Timer),	// no plug-and-play: one scan
	m_Net (0, 0, 0, 0, "kilo"),		// DHCP, hostname kilo (net.cpp: the fallback)
	m_CircleLog (&m_Logger, LogLevel::Info),
	m_pRing (0),
	m_pLog (0),
	m_pPl011 (0),
	m_pUsbSerial (0),
	m_pSerial (0),
	m_pSerialName ("none"),
	m_pLink (0),
	m_pListener (0),
	m_pNetTask (0),
	m_pHost (0),
	m_pWalker (0),
	m_pBench (0)
{
	m_ActLED.On ();		// first sign of life, before anything can fail
	m_Watch.m_pClock = &m_Clock;
	m_Watch.m_nBootUs = m_Clock.now_us ();
}

boolean CKernel::Initialize (void)
{
	CDevice *pLog = &m_Null;
	if (m_Screen.Initialize ())
	{
		pLog = &m_Screen;
	}

	if (   !m_Logger.Initialize (pLog)
	    || !m_Interrupt.Initialize ()
	    || !m_Timer.Initialize ())
	{
		return FALSE;
	}

	// The ring has no sink of its own: KernelLog sends each core line on to
	// CLogger, and DrainCircleLog copies Circle's own lines into the ring only.
	m_pRing = new LogRing (m_Clock);
	m_pLog = new KernelLog (m_pRing, &m_CircleLog);
	m_Watch.m_pLog = m_pLog;

	logf (*m_pLog, LogLevel::Info, "Kilo mind, build %08x", (unsigned) KILO_BUILD);

	m_pSerial = ChooseSerial ();
	if (m_pSerial == 0)
	{
		return FALSE;
	}

	m_pLink = new SerialLink (*m_pSerial, m_Clock, *m_pLog);
	m_pWalker = new Walker (*m_pLink, m_Clock, *m_pLog);
	m_Watch.m_pWalker = m_pWalker;
	m_pLink->set_listener (&m_Watch);
	// The clock reads nearly the same at every boot, so the core's clock-based
	// seed would repeat (seq, type) pairs inside the body's 500 ms de-dup
	// window after a quick reboot of the mind. The hardware RNG does not.
	m_pLink->seed_seq ((uint8_t) m_Random.GetNumber ());
	m_pLink->connect ();

	// The laptop interface. Nothing on the network is up yet: the net task
	// brings it up once the main loop runs (docs/fast-boot.md §3 rule 1).
	m_pListener = new CircleListener;
	m_pHost = new HostServer (*m_pListener, *m_pLink, *m_pSerial, m_Clock, *m_pLog, *m_pRing,
				  (uint32_t) KILO_BUILD);

#if KILO_PLANNER_BENCH
	m_pBench = new CPlannerBench (m_pLog);
#endif

	// Last: it first runs at the main loop's first sleep. With the PL011 the
	// USB host was not scanned yet (ChooseSerial), and the LAN9514 is a USB
	// device, so the task scans first.
#ifdef KILO_BODY_SERIAL_PL011
	CUSBHCIDevice *pUSB = &m_USBHCI;
#else
	CUSBHCIDevice *pUSB = 0;
#endif
	m_pNetTask = new CKiloNet (&m_Net, pUSB, &m_Watch.m_bFirstHello, m_pLog, m_pListener);

	return TRUE;
}

// The body link's device. Rule: a USB serial adapter (utty1, the CH343 on
// Kilo's cable today) if one enumerated, else the PL011 on GPIO14/15. Built
// with KILO_BODY_SERIAL=pl011, the USB scan is skipped and the PL011 is
// used at once (the production wiring: no USB before the link, plan rule 1).
kilo::Serial *CKernel::ChooseSerial (void)
{
#ifndef KILO_BODY_SERIAL_PL011
	// This scan also finds the LAN9514 (Ethernet) for the net task.
	if (!m_USBHCI.Initialize ())
	{
		logf (*m_pLog, LogLevel::Error, "USB host failed: trying the PL011 (no Ethernet)");
	}
	else
	{
		CUSBSerialDevice *pDevice =
			(CUSBSerialDevice *) m_DeviceNameService.GetDevice ("utty1", FALSE);
		if (pDevice != 0)
		{
			m_pUsbSerial = new UsbSerial (pDevice);
			if (!m_pUsbSerial->set_baud (KILO_BAUD))
			{
				// Not fatal: the CDC default may still be right; the stats show it.
				logf (*m_pLog, LogLevel::Warn, "utty1: cannot set %u baud", (unsigned) KILO_BAUD);
			}
			m_pSerialName = "utty1 (USB serial)";
			logf (*m_pLog, LogLevel::Info, "body link on %s, %u baud", m_pSerialName,
			      (unsigned) KILO_BAUD);

			return m_pUsbSerial;
		}

		logf (*m_pLog, LogLevel::Info, "no USB serial adapter (utty1)");
	}
#endif

	m_pPl011 = new Pl011Serial (&m_Interrupt);
	if (!m_pPl011->Initialize (KILO_BAUD))
	{
		logf (*m_pLog, LogLevel::Error, "PL011 failed");

		return 0;
	}
	m_pSerialName = "ttyS1 (PL011, GPIO14/15)";
	logf (*m_pLog, LogLevel::Info, "body link on %s, %u baud", m_pSerialName, (unsigned) KILO_BAUD);

	return m_pPl011;
}

TShutdownMode CKernel::Run (void)
{
	uint64_t nStart = m_Clock.now_us ();
	uint64_t nNextStats = nStart + 1000000;
	uint64_t nNextReconnect = 0;
	uint64_t nBenchAt = 0;			// 0: no laptop client yet
	unsigned nLastPass = CTimer::GetClockTicks ();
	boolean bWasBridged = FALSE;

	for (;;)
	{
		// CTimer::GetClockTicks: 1 MHz, 32 bits; differences wrap correctly.
		unsigned nPass = CTimer::GetClockTicks ();
		m_Period.Add (nPass - nLastPass);
		nLastPass = nPass;

		// This loop is the mind: it is alive while it runs.
		m_pLink->mark_alive ();
		m_pHost->poll ();			// accept, answer, stream; never blocks
		boolean bBridged = m_pHost->bridge_open ();
		if (!bBridged)
		{
			if (bWasBridged && m_pWalker->active ())
			{
				// A walk the bridge cut off: the body stopped streaming
				// (FEET underrun) while the laptop had the UART. End it
				// now that the link may send again (abort() can send
				// SET_STREAM, which must not go out during the bridge).
				// Nothing starts a walk on the kernel yet (P3's Mind will).
				m_pWalker->abort ();
			}

			m_pLink->poll ();		// the raw bridge pauses the link

			unsigned t0 = CTimer::GetClockTicks ();
			m_pWalker->poll ();
			m_WalkerPoll.Add (CTimer::GetClockTicks () - t0);
		}
		bWasBridged = bBridged;

		uint64_t nNow = m_Clock.now_us ();

		if (!m_pLink->connected ())
		{
			// The Serial failed (a USB adapter gone). The PL011 does not
			// fail; a USB device that went away does not come back without
			// plug-and-play, so this mostly logs once a second.
			if (nNow >= nNextReconnect)
			{
				nNextReconnect = nNow + ReconnectUs;
				m_pLink->connect ();
			}
		}

		// The planner bench: one call per pass, never during a walk or bridge.
		if (nBenchAt == 0 && m_pHost->clients () > 0)
		{
			nBenchAt = nNow + BenchDelayUs;
		}
		if (   m_pBench != 0
		    && nBenchAt != 0
		    && nNow >= nBenchAt
		    && !bBridged
		    && !m_pWalker->active ())
		{
			if (!m_pBench->Step ())
			{
				delete m_pBench;
				m_pBench = 0;
			}
		}

		DrainCircleLog ();

		if (nNow >= nNextStats)
		{
			nNextStats += 1000000;
			LogStats ();
		}

		UpdateLED (nNow);

		m_Work.Add (CTimer::GetClockTicks () - nPass);

		// Sleep, not CTimer::usDelay: a delay spins with nothing else running,
		// a sleep hands the CPU to the scheduler, which runs the other tasks
		// (Circle's net task, DHCP, PHY, and ours, net.h) round robin until
		// the wake tick. The scheduler is cooperative, so a task that does
		// not yield within a few ms delays the link (it tolerates ~10 ms);
		// the loop stats line shows the longest pass-to-pass period.
		m_Scheduler.usSleep (LoopPeriodUs);
	}

	return ShutdownHalt;
}

// Circle's own log lines (USB, Ethernet, DHCP, TCP) into the RAM ring, so
// `kilo log --host` shows them. Our own lines go to CLogger with
// WriteNoAlloc, which makes no event, so nothing is copied twice. CLogger
// keeps the last 50 events; Debug and Trace are left out.
void CKernel::DrainCircleLog (void)
{
	for (unsigned n = 0; n < CircleEventsPerPass; n++)
	{
		TLogSeverity Severity;
		char Source[LOG_MAX_SOURCE];
		char Message[LOG_MAX_MESSAGE];
		time_t Time;
		unsigned nHundredth;
		int nTimeZone;
		if (!m_Logger.ReadEvent (&Severity, Source, Message, &Time, &nHundredth, &nTimeZone))
		{
			return;
		}
		if (Severity > LogNotice)
		{
			continue;
		}

		LogLevel Level =   Severity <= LogError ? LogLevel::Error
				 : Severity == LogWarning ? LogLevel::Warn : LogLevel::Info;
		char Line[kLogLineMax];
		format (Line, sizeof Line, "%s: %s", Source, Message);
		m_pRing->write (Level, Line);
	}
}

// uint32_t is unsigned long on arm-none-eabi; the core's %u takes unsigned.
static inline unsigned U (uint32_t n) { return (unsigned) n; }

void CKernel::LogStats (void)
{
	const LinkStats &s = m_pLink->stats ();
	const BodyState &b = m_pLink->state ();
	logf (*m_pLog, LogLevel::Info,
	      "link %s: telemetry %u/s (total %u) rx %u frames, tx %u (%u heartbeats) "
	      "decode errors %u (crc %u cobs %u overflow %u) tx deferred %u short %u",
	      m_pSerialName, m_Watch.m_nTelemetry, U (b.telemetry_count), U (s.frames_rx),
	      U (s.frames_tx), U (s.heartbeats), U (s.decode_error_total ()),
	      U (s.decode_errors[KILO_FRAME_BAD_CRC]), U (s.decode_errors[KILO_FRAME_BAD_COBS]),
	      U (s.decode_errors[KILO_FRAME_OVERFLOW]), U (s.tx_deferred), U (s.tx_short));
	if (m_pPl011 != 0)
	{
		const unsigned *e = m_pPl011->m_nLineErrors;
		if (e[1] + e[2] + e[3] + e[4] + e[0] != 0)
		{
			logf (*m_pLog, LogLevel::Warn, "PL011 line errors: break %u overrun %u framing %u parity %u",
			      e[1], e[2], e[3], e[4]);
		}
	}
	if (b.has_telemetry)
	{
		logf (*m_pLog, LogLevel::Info, "body: mode %s epoch %u tick %u rx_err %u overruns %u/%u",
		      mode_name (b.telemetry.mode), b.telemetry.epoch, U (b.tick), b.telemetry.rx_err,
		      m_Watch.m_nOverruns, m_Watch.m_nTelemetry);
	}

	// The loop's timing, for the first boot to measure with no scope.
	// period: pass to pass (1000 us asked); work: a pass without the sleep;
	// walker.poll: the planner's share (a walk's wake runs it every 20 ms).
	logf (*m_pLog, LogLevel::Info,
	      "loop %u/s: period max %u mean %u us, work max %u mean %u us, walker.poll max %u mean %u us%s; "
	      "%u clients",
	      m_Period.m_nCount, m_Period.m_nMax, m_Period.Mean (), m_Work.m_nMax, m_Work.Mean (),
	      m_WalkerPoll.m_nMax, m_WalkerPoll.Mean (), m_pBench != 0 ? " (bench)" : "",
	      U (m_pHost->clients ()));

	m_Watch.m_nTelemetry = 0;
	m_Watch.m_nOverruns = 0;
	m_Period.Reset ();
	m_Work.Reset ();
	m_WalkerPoll.Reset ();
}

void CKernel::UpdateLED (uint64_t nNowUs)
{
	const BodyState &b = m_pLink->state ();

	// Half period of the blink, in ms: 5 Hz on a protocol mismatch, 2 Hz
	// while the body is alive, 0.5 Hz while there is no body. The kernel
	// keeps running on a mismatch (the heartbeat still goes out; the link
	// refuses goals and FEET), unlike kilo-mind-host, which exits 2.
	unsigned nHalfMs;
	if (b.proto_ok == 0)
	{
		nHalfMs = 100;
	}
	else if (b.alive (nNowUs, BodyLostUs))
	{
		nHalfMs = 250;
	}
	else
	{
		nHalfMs = 1000;
	}

	if ((nNowUs / 1000 / nHalfMs) % 2 == 0)
	{
		m_ActLED.On ();
	}
	else
	{
		m_ActLED.Off ();
	}
}
