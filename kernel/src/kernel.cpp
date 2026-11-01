//
// kernel.cpp: the Kilo mind kernel (docs/fast-boot.md): the whole mind core
// as one kilo::Mind (mind/mind.h), as kernel/host/main.cpp runs it on
// Linux: the body link (P1), the laptop interface over Ethernet (P2), the
// walker, the trick player, perception, the IR remote and the behaviour
// runner (P3, the runner off unless built with KILO_BEHAVIOUR=1), and
// flashing the body from the laptop or the SD card (P4).
//
// Initialize: the screen (or nothing), interrupts, timer, then the Mind
// (so every line is in its ring), the body link's Serial, the flasher,
// mind.start() (the link connects: HELLO_REQ), the planner bench and last
// the net task. Then one pass every 1 ms:
//
//   mind.poll()        stamp liveness, serve the laptop (and poll the
//                      flasher), then unless the bridge or the flasher has
//                      the UART: the link, the walker, the trick player,
//                      the remote, the behaviour runner
//   the end of a flash: log it; --persist: write the image to SD
//   the boot flash check: once, after the first HELLO (or 3 s)
//   link.connect() again once a second if the Serial failed
//   a planner bench step: once, after the first laptop client
//   Circle's log events into the ring
//   the stats lines once a second, the ACT LED
//   CScheduler::usSleep(1000): the net task and Circle's run here
//
// Logs go into the Mind's RAM ring, which `kilo log --host` reads, and to
// the HDMI screen if the firmware gives us a frame buffer, else nowhere
// (Kilo has no monitor). The LED:
//
//   steady on      starting up (the USB scan takes about a second or more)
//   0.5 Hz blink   no body: no HELLO yet, or no TELEMETRY for 300 ms
//   2 Hz blink     the body is alive: TELEMETRY flowing
//   5 Hz blink     the body speaks another protocol major: not driven (flash it)
//   double flash   flashing the body (two 100 ms flashes a second)
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

// The behaviour runner (kilo run --behaviour): off, as the Pi's daemon ran.
// make -C kernel KILO_BEHAVIOUR=1 turns it on.
#ifndef KILO_BEHAVIOUR
#define KILO_BEHAVIOUR 0
#endif

using namespace kilo;

// The Pi's daemon ran `kilo run --remote --balance 0.99` (docs/setup.md,
// ~/.config/kilo/env KILO_RUN_ARGS): the IR remote live, and balance on
// from the start with this Kp.
static const bool RemoteOn = true;
static const double BalanceKp = 0.99;

// STATS_REQ to the body this often, the answer logged ("body STATS: ..."),
// as kilo-mind-host does.
static const uint32_t BodyStatsPeriodMs = 1000;

// The main loop's period. The link wants poll() at least every ~10 ms (the
// heartbeat is due after 100 ms of silence, a command is retried after
// 50 ms), the walker wakes on a 20 ms grid and the behaviour runner on a
// 100 ms one; 1 ms keeps the RX ring short and the walker's wake within
// 1 ms of its deadline, and gives the net task its turn in every sleep.
static const unsigned LoopPeriodUs = 1000;

// After the Serial failed: how often the link tries again.
static const uint64_t ReconnectUs = 1000000;

// No TELEMETRY this long: the body is lost (the link's own rule).
static const uint32_t BodyLostUs = KILO_BODY_LOST_MS * 1000u;

// The planner bench starts this long after the first laptop client
// connects, so that a `kilo log --host` tail sees its lines live: the RAM
// ring holds about 15 s of log (four stats lines a second), and nobody
// reads the screen.
static const uint64_t BenchDelayUs = 1000000;

// Circle's own log events copied into the ring per pass, at most.
static const unsigned CircleEventsPerPass = 4;

// The boot-time flash check waits this long after mind.start() for the
// body's first HELLO; with none, the SD card's image counts as another
// build (an empty or broken firmware sends no HELLO).
static const uint64_t BootHelloWaitUs = 3000000;

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
}

CKernel::CKernel (void)
:	m_Screen (m_Options.GetWidth (), m_Options.GetHeight ()),
	m_Timer (&m_Interrupt),
	m_Logger (m_Options.GetLogLevel (), &m_Timer),
	m_USBHCI (&m_Interrupt, &m_Timer),	// no plug-and-play: one scan
	m_Net (0, 0, 0, 0, "kilo"),		// DHCP, hostname kilo (net.cpp: the fallback)
	m_CircleLog (&m_Logger, LogLevel::Info),
	m_KernelLog (&m_CircleLog),
	m_pPl011 (0),
	m_pUsbSerial (0),
	m_pSerialName ("none"),
	m_pMind (0),
	m_pLog (0),
	m_pListener (0),
	m_pNetTask (0),
	m_pBench (0),
	m_pFlasher (0),
	m_pBootCard (0),
	m_pHostImage (0),
	m_pCardImage (0),
	m_BootFlash (BootFlashWait),
	m_nLinkUpUs (0),
	m_bGoAhead (FALSE),
	m_bWasFlashing (FALSE),
	m_bPoweroffLogged (FALSE)
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

	// The mind core, as `kilo run --remote --balance 0.99` ran it on the
	// Pi. The laptop interface's network seams first: the Mind's host
	// server serves them, the net task (last, below) fills them. Nothing
	// on the network is up yet: the net task brings it up once the main
	// loop runs (docs/fast-boot.md §3 rule 1).
	m_pListener = new CircleListener;
	MindConfig Config;
	Config.mind_build = (uint32_t) KILO_BUILD;
	Config.remote = RemoteOn;
	Config.behaviour = KILO_BEHAVIOUR != 0;
	Config.balance_kp = BalanceKp;
	// The behaviour RNG's seed: 32 bits, so that the logged seed (the
	// runner logs it as %u) replays the same choices.
	Config.seed = m_Random.GetNumber ();
	Config.stats_period_ms = BodyStatsPeriodMs;
	m_pMind = new Mind (m_Serial, m_Clock, m_KernelLog, m_pListener, Config);
	m_pLog = &m_pMind->log ();
	m_Watch.m_pLog = m_pLog;
	m_pMind->set_tap (&m_Watch);

	logf (*m_pLog, LogLevel::Info, "Kilo mind, build %08x%s", (unsigned) KILO_BUILD,
	      KILO_BEHAVIOUR ? " (behaviour runner on)" : "");

	kilo::Serial *pSerial = ChooseSerial ();
	if (pSerial == 0)
	{
		return FALSE;
	}
	m_Serial.Set (pSerial);

	// Flashing the body: the flasher takes the link's Serial (115200 for
	// optiboot, then back) and GPIO4 (the body's RESET). The Mind's host
	// server fills the laptop's image from FLASH_*, starts the flasher and
	// polls it, for the boot-time flash from SD too (BootFlash), and while
	// it has the UART the Mind pauses everything on the link. Two images of
	// 32 KB on the heap (static, their initialisers would put 64 KB into the
	// kernel image): a laptop sending one while the boot check reads the
	// card cannot have it overwritten. The card is touched only once the
	// link has its HELLO (BootFlash): reading it spins.
	m_pHostImage = new FlashImage;
	m_pCardImage = new FlashImage;
	m_pFlasher = new Flasher (m_Serial, m_Clock, m_ResetGpio, m_pMind->link (), *m_pLog);
	m_pMind->host ().attach_flasher (*m_pFlasher, *m_pHostImage);
	m_pBootCard = new CBootCard (&m_Interrupt, &m_Timer, m_pLog);

	// The clock reads nearly the same at every boot, so the core's
	// clock-based seed would repeat (seq, type) pairs inside the body's
	// 500 ms de-dup window after a quick reboot of the mind. The hardware
	// RNG does not.
	m_pMind->link ().seed_seq ((uint8_t) m_Random.GetNumber ());
	m_pMind->start ();		// balance on, then link.connect(): HELLO_REQ
	m_nLinkUpUs = m_Clock.now_us ();

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
	m_pNetTask = new CKiloNet (&m_Net, pUSB, &m_bGoAhead, m_pLog, m_pListener, (u32) KILO_BUILD);

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
	SerialLink &Link = m_pMind->link ();
	uint64_t nStart = m_Clock.now_us ();
	uint64_t nNextStats = nStart + 1000000;
	uint64_t nNextReconnect = 0;
	uint64_t nBenchAt = 0;			// 0: no laptop client yet
	unsigned nLastPass = CTimer::GetClockTicks ();

	for (;;)
	{
		// CTimer::GetClockTicks: 1 MHz, 32 bits; differences wrap correctly.
		unsigned nPass = CTimer::GetClockTicks ();
		m_Period.Add (nPass - nLastPass);
		nLastPass = nPass;

		// The mind: this loop is alive while it runs, and Mind::poll()
		// stamps the link's liveness for the body's watchdog. While the raw
		// bridge is open or the flasher has the UART, it serves the laptop
		// only; a walk such a pause cut off is aborted when it ends.
		m_pMind->poll ();
		m_MindPoll.Add (CTimer::GetClockTicks () - nPass);

		uint64_t nNow = m_Clock.now_us ();

		WatchFlash ();			// first: it names a boot flash that just ended
		BootFlash (nNow);

		if (!Link.connected ())
		{
			// The Serial failed (a USB adapter gone). The PL011 does not
			// fail; a USB device that went away does not come back without
			// plug-and-play, so this mostly logs once a second.
			if (nNow >= nNextReconnect)
			{
				nNextReconnect = nNow + ReconnectUs;
				Link.connect ();
			}
		}

		// The planner bench: one call per pass, never while the body moves
		// (a walk, a trick) or the UART is taken (bridge, flash).
		if (nBenchAt == 0 && m_pMind->host ().clients () > 0)
		{
			nBenchAt = nNow + BenchDelayUs;
		}
		if (   m_pBench != 0
		    && nBenchAt != 0
		    && nNow >= nBenchAt
		    && !m_pMind->host ().link_paused ()
		    && !m_pFlasher->busy ()
		    && !m_pMind->walker ().active ()
		    && !m_pMind->player ().busy ())
		{
			if (!m_pBench->Step ())
			{
				delete m_pBench;
				m_pBench = 0;
			}
		}

		// The behaviour runner's wind-down ends in a poweroff the daemon
		// would run; the kernel cannot switch the Pi off.
		if (m_pMind->poweroff_requested () && !m_bPoweroffLogged)
		{
			m_bPoweroffLogged = TRUE;
			logf (*m_pLog, LogLevel::Warn, "behaviour runner asked for poweroff: the kernel cannot switch "
			      "the Pi off; the runner has stopped, the link stays up");
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

// The boot-time flash from the SD card (docs/fast-boot.md §6 path B,
// `kilo flash --sd`): once the body's first HELLO arrived (or none came
// BootHelloWaitUs after link.connect()), read body.json and body.bin and
// flash them if the body runs another build. Then the go-ahead for the net
// task's USB scan (KILO_BODY_SERIAL=pl011), which stalls the main loop and
// so must not cut into a flash. A missing or bad file is no error.
//
// The card is read only here, after the HELLO, so neither the link nor the
// boot probe (GPIO21 falls at the first HELLO) waits for it; the read
// itself stalls the loop (SD driver busy-waits), logged in ms.
void CKernel::BootFlash (uint64_t nNowUs)
{
	switch (m_BootFlash)
	{
	case BootFlashWait:
		if (   !m_Watch.m_bFirstHello
		    && nNowUs - m_nLinkUpUs < BootHelloWaitUs)
		{
			return;
		}
		if (m_pMind->host ().link_paused () || m_pFlasher->busy ())
		{
			return;			// a bridge or a laptop's flash first
		}
		break;

	case BootFlashRunning:
		if (m_pFlasher->busy ())
		{
			return;			// WatchFlash logs the end
		}
		m_BootFlash = BootFlashDone;
		m_bGoAhead = TRUE;
		return;

	case BootFlashDone:
		return;
	}

	m_BootFlash = BootFlashDone;
	const BodyState &b = m_pMind->link ().state ();
	const kilo_hello_t *pHello = b.has_hello ? &b.hello : 0;
	if (pHello == 0)
	{
		logf (*m_pLog, LogLevel::Warn, "boot flash check: no HELLO %u ms after the link came up "
		      "(no firmware, a broken one, or the body is off)", (unsigned) (BootHelloWaitUs / 1000));
	}

	if (!m_pBootCard->LoadBodyImage (*m_pCardImage))
	{
		m_bGoAhead = TRUE;
		return;
	}

	uint32_t nCard = m_pCardImage->manifest ().build;
	if (!m_pCardImage->needs_flash (pHello))
	{
		logf (*m_pLog, LogLevel::Info, "boot flash check: the body runs build %08x, the card's: no flash",
		      (unsigned) nCard);
		m_pCardImage->clear ();
		m_bGoAhead = TRUE;
		return;
	}

	if (pHello != 0)
	{
		logf (*m_pLog, LogLevel::Info, "boot flash check: the body runs build %08x, the card has %08x: flashing",
		      (unsigned) pHello->fw_build, (unsigned) nCard);
	}
	else
	{
		logf (*m_pLog, LogLevel::Info, "boot flash check: no HELLO, the card has %08x: flashing",
		      (unsigned) nCard);
	}

	// A walk the remote started in the first seconds ends before the
	// flasher takes the UART (abort() may send SET_STREAM, which must go
	// out first). A trick is not stopped: the flash resets the body, and
	// the player gives up on it by its own timeout.
	m_pMind->walker ().abort ();
	if (!m_pFlasher->start (*m_pCardImage, false))	// the host server polls it
	{
		logf (*m_pLog, LogLevel::Warn, "boot flash check: the flasher is busy: not flashing");
		m_bGoAhead = TRUE;
		return;
	}
	m_BootFlash = BootFlashRunning;
}

// The end of a flash, whoever started it, and --persist: after a DONE from
// a FLASH_BEGIN with PERSIST, write the image to the SD card (body.bin, then
// body.json), then persist_done() whatever the outcome, so the host server
// takes FLASH_BEGIN again (it refuses one while a persist is pending).
void CKernel::WatchFlash (void)
{
	boolean bFlashing = m_pFlasher->busy ();
	if (m_bWasFlashing && !bFlashing)
	{
		const Flasher::Status &s = m_pFlasher->status ();
		const char *pFrom = m_BootFlash == BootFlashRunning ? "the SD card (boot)" : "the laptop";
		if (s.phase == KILO_FLASH_PHASE_DONE)
		{
			logf (*m_pLog, LogLevel::Info, "body flashed from %s: build %08x, HELLO build %08x reset %s",
			      pFrom, (unsigned) s.build, (unsigned) s.fw_build, reset_cause_name (s.reset_cause));
		}
		else
		{
			logf (*m_pLog, LogLevel::Error, "body flash from %s FAILED: %s (STK500 command %02x, attempt %u); "
			      "the link is back", pFrom, flash_error_name (s.error), (unsigned) s.stk_cmd,
			      (unsigned) s.attempt);
		}
	}
	m_bWasFlashing = bFlashing;

	if (m_pFlasher->persist_pending ())
	{
		const FlashImage *pImage = m_pFlasher->image ();
		if (pImage == 0 || !m_pBootCard->PersistBodyImage (*pImage))
		{
			logf (*m_pLog, LogLevel::Warn, "--persist: the image is NOT on the SD card; the body runs it anyway");
		}
		m_pFlasher->persist_done ();
	}
}

// Circle's own log lines (USB, Ethernet, DHCP, TCP) into the RAM ring, so
// `kilo log --host` shows them. The ring passes every line on to
// KernelLog, which is muted meanwhile, so CLogger does not print them a
// second time. Our own lines go to CLogger with WriteNoAlloc, which makes
// no event, so nothing is copied twice. CLogger keeps the last 50 events;
// Debug and Trace are left out.
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
		m_KernelLog.Mute (TRUE);		// CLogger printed it already: the ring only
		m_pLog->write (Level, Line);
		m_KernelLog.Mute (FALSE);
	}
}

// uint32_t is unsigned long on arm-none-eabi; the core's %u takes unsigned.
static inline unsigned U (uint32_t n) { return (unsigned) n; }

void CKernel::LogStats (void)
{
	const LinkStats &s = m_pMind->link ().stats ();
	const BodyState &b = m_pMind->link ().state ();
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
		if (e[1] + e[2] + e[3] + e[4] + e[0] + m_pPl011->m_nDrainTimeouts != 0)
		{
			logf (*m_pLog, LogLevel::Warn, "PL011 line errors: break %u overrun %u framing %u parity %u; "
			      "baud switches with bytes still queued %u", e[1], e[2], e[3], e[4], m_pPl011->m_nDrainTimeouts);
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
	// mind.poll: the mind core's share (the link, the laptop, the walker,
	// whose wake every 20 ms of a walk runs the planner, the trick player,
	// the remote, the behaviour runner's 10 Hz tick).
	logf (*m_pLog, LogLevel::Info,
	      "loop %u/s: period max %u mean %u us, work max %u mean %u us, mind.poll max %u mean %u us%s; "
	      "%u clients",
	      m_Period.m_nCount, m_Period.m_nMax, m_Period.Mean (), m_Work.m_nMax, m_Work.Mean (),
	      m_MindPoll.m_nMax, m_MindPoll.Mean (), m_pBench != 0 ? " (bench)" : "",
	      U (m_pMind->host ().clients ()));

	m_Watch.m_nTelemetry = 0;
	m_Watch.m_nOverruns = 0;
	m_Period.Reset ();
	m_Work.Reset ();
	m_MindPoll.Reset ();
}

void CKernel::UpdateLED (uint64_t nNowUs)
{
	const BodyState &b = m_pMind->link ().state ();

	// Flashing: two 100 ms flashes a second, a pattern no link state uses.
	if (m_pFlasher->busy ())
	{
		unsigned nMs = (unsigned) (nNowUs / 1000 % 1000);
		if (nMs < 100 || (nMs >= 200 && nMs < 300))
		{
			m_ActLED.On ();
		}
		else
		{
			m_ActLED.Off ();
		}

		return;
	}

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
