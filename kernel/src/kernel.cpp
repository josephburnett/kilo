//
// kernel.cpp: the Kilo mind kernel (docs/fast-boot.md): the whole mind core
// as one kilo::Mind (mind/mind.h), as kernel/host/main.cpp runs it on
// Linux: the body link (P1), the laptop interface over Ethernet (P2), the
// walker, the trick player, perception, the IR remote and the behaviour
// runner (P3, the runner off unless built with KILO_BEHAVIOUR=1),
// flashing the body from the laptop or the SD card (P4), writing a new
// kernel onto the SD card from the laptop (`kilo update --host`), and the
// flight recorder (every link frame, log line, walk and trick in RAM from
// boot, `kilo runs --host`, `kilo record fetch --host`; saved to the SD card
// at safe moments when switched on, `kilo record save on --host`).
//
// Initialize: the screen (or nothing), interrupts, timer, then the Mind
// (so every line is in its ring), the body link's Serial, the flasher, the
// kernel updater, the walk recorder's store, mind.start() (the link connects: HELLO_REQ), the planner bench and last
// the net task. Then one pass every 1 ms:
//
//   mind.poll()        stamp liveness, serve the laptop (and poll the
//                      flasher and the kernel updater: a slice of SD card
//                      writes), then unless the bridge or the flasher has
//                      the UART or the updater the card: the link, the
//                      walker, the trick player, the remote, the behaviour
//                      runner
//   the end of a flash: log it; --persist: write the image to SD
//   the boot flash check: once, after the first HELLO (or 3 s): the SD
//                      card's body firmware if the body runs another build
//                      (mind/flash/cardflash.h), the outcome in
//                      body-result.txt on the card
//   link.connect() again once a second if the Serial failed
//   a planner bench step: once, after the first laptop client
//   Circle's log events into the ring
//   the stats lines once a second, the ACT LED
//   a laptop's REBOOT, or a kernel update installed: 200 ms on (the
//                      answer leaves, REST goes out), then Run() returns
//                      ShutdownReboot (main.cpp: reboot()): the firmware
//                      boots the card's kernel7.img
//   CScheduler::usSleep(1000): the net task and Circle's run here; in
//                      light sleep (A7, then 30 s of nothing: mind/power/
//                      sleep.h) LightSleep(10 ms) instead: the other tasks'
//                      turns and wfi between them
//
// Logs go into the Mind's RAM ring, which `kilo log --host` reads, and to
// the HDMI screen if the firmware gives us a frame buffer, else nowhere
// (Kilo has no monitor). The LED:
//
//   steady on      starting up
//   0.5 Hz blink   no body: no HELLO yet, or no TELEMETRY for 300 ms
//   2 Hz blink     the body is alive: TELEMETRY flowing
//   5 Hz blink     the body speaks another protocol major: not driven (flash it)
//   double flash   flashing the body, or writing a kernel update to the card
//                  (two 100 ms flashes a second)
//   50 ms flash    light sleep (one every 5 s)
//
#include "kernel.h"
#include <circle/synchronize.h>		// WaitForInterrupt (LightSleep)
#include "authorized_keys.h"		// generated from kernel/keys/authorized_keys (Makefile)
#include "net/noise.h"
#include "bootprobe.h"
#include "link/names.h"
#include "util/fmt.h"
#include "util/log.h"

using namespace kilo;

// This image's identity for `kilo update --host`: the kernel updater
// (mind/update/updater.h) installs an image only if this string is in it
// (unless forced), so the netboot loader, which lacks it, or a wrong file is
// not written over kernel7.img by mistake. used: kept by the compiler; and
// the start-up log line refers to it, so a build with Circle's GC_SECTIONS=1
// (--gc-sections; off by default) keeps it too.
extern "C" const char KiloMindKernelMarker[] __attribute__ ((used)) = KILO_HOST_UPDATE_MARKER;

// The largest kernel image `kilo update --host` may send (the updater's
// buffer, on the heap): about 8 times this kernel.
static const uint32_t KernelImageMax = KILO_HOST_UPDATE_MAX_IMAGE;

// How Kilo runs: the IR remote live, and balance on from the start with
// this Kp.
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

// After a laptop's REBOOT the loop runs on this long before Run() returns
// ShutdownReboot: the net task sends the RESULT OK (and the link the REST)
// in the passes' sleeps. Bounded: a client that stopped reading does not
// hold the reboot.
static const uint64_t RebootFlushUs = 200000;

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
}

CKernel::CKernel (void)
:	m_Screen (m_Options.GetWidth (), m_Options.GetHeight ()),
	m_Timer (&m_Interrupt),
	m_Logger (m_Options.GetLogLevel (), &m_Timer),
	m_USBHCI (&m_Interrupt, &m_Timer),	// no plug-and-play: one scan
	m_Net (0, 0, 0, 0, "kilo"),		// DHCP, hostname kilo (net.cpp: the fallback)
	m_KeyRandom (&m_Random),
	m_CircleLog (&m_Logger, LogLevel::Info),
	m_KernelLog (&m_CircleLog),
	m_pPl011 (0),
	m_pSerialName ("none"),
	m_pMind (0),
	m_pLog (0),
	m_pListener (0),
	m_pNetTask (0),
	m_pBench (0),
	m_pPower (0),
#if KILO_SOUND
	m_pSound (0),
#endif
	m_pFlasher (0),
	m_pBootCard (0),
	m_pHostImage (0),
	m_pCardImage (0),
	m_pCardFlash (0),
	m_pUpdater (0),
	m_bGoAhead (FALSE),
	m_bWasFlashing (FALSE),
	m_bPoweroffLogged (FALSE),
	m_nWakeups (0),
	m_bSlept (FALSE)
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

	// The mind core: the remote on, balance on at Kp 0.99 (above). The laptop interface's network seams first: the Mind's host
	// server serves them, the net task (last, below) fills them. Nothing
	// on the network is up yet: the net task brings it up once the main
	// loop runs (docs/fast-boot.md §3 rule 1).
	m_pListener = new CircleListener;
	MindConfig Config;
	Config.mind_build = (uint32_t) KILO_BUILD;
	Config.remote = RemoteOn;
	Config.behaviour = KILO_BEHAVIOUR != 0;
	Config.rear_up_key = (uint8_t) KILO_REAR_UP;
	Config.intent = KILO_INTENT != 0;
	Config.righting = KILO_RIGHTING != 0;
	Config.balance_kp = BalanceKp;
	// The behaviour RNG's seed: 32 bits, so that the logged seed (the
	// runner logs it as %u) replays the same choices.
	Config.seed = m_Random.GetNumber ();
	Config.stats_period_ms = BodyStatsPeriodMs;
	Config.record_save = KILO_RECORD_SAVE != 0;
	Config.light_sleep = KILO_LIGHT_SLEEP != 0;
	// Who may talk to it over the network: the laptops' public keys built
	// in from kernel/keys/authorized_keys (build/gen/authorized_keys.h, the
	// Makefile's KILO_AUTHORIZED_KEYS), the ephemeral keys from the hardware RNG.
	Config.authorized_keys = kAuthorizedKeys;
	Config.authorized_key_count = kAuthorizedKeyCount;
	Config.random = &m_KeyRandom;
#if KILO_SOUND
	// The jack: Circle's PWM sound device (the PWM clock and GPIO40/45 set
	// up now; the DMA starts with the first sound), the sounds the build packed.
	m_pSound = new CKiloSound (&m_Interrupt);
	m_pSound->Initialize ();
	Config.sounds = kilo::sound::kSounds;
	Config.sound_count = kilo::sound::kSoundCount;
	Config.sound_output = m_pSound;
#endif
	m_pMind = new Mind (m_Serial, m_Clock, m_KernelLog, m_pListener, Config);
	m_pMind->attach_log (new LogEntry[LogRing::kLines], LogRing::kLines);	// 4096 lines of history, 700 KB
	m_pLog = &m_pMind->log ();
	m_Watch.m_pLog = m_pLog;
	m_pMind->set_tap (&m_Watch);

	static const char *const RearUp[] = {"", " (rear-up key A1 on: rear_up, stage 1)",
					     " (rear-up key A1 on: rear_up_2, stage 2)",
					     " (rear-up key A1 on: rear_up_3, stage 3)"};
	logf (*m_pLog, LogLevel::Info, "Kilo mind, build %08x%s%s%s (%s)", (unsigned) KILO_BUILD,
	      KILO_BEHAVIOUR ? " (behaviour runner on)" : "", RearUp[KILO_REAR_UP],
	      KILO_INTENT ? " (intents on: stand, sit and rest are moves from the model)" : "",
	      KiloMindKernelMarker);
	for (unsigned i = 0; i < kAuthorizedKeyCount; i++)
	{
		char Fingerprint[17];
		kilo::noise::fingerprint (kAuthorizedKeys[i].key, Fingerprint);
		logf (*m_pLog, LogLevel::Info, "host: authorized key %u of %u: %s %s", i + 1, kAuthorizedKeyCount,
		      Fingerprint, kAuthorizedKeys[i].name);
	}

#if KILO_SOUND
	logf (*m_pLog, LogLevel::Info,
	      "sound: PWM on the 3.5 mm jack, %u Hz, range %d, chunks of %u frames, idle stopped",
	      CKiloSound::SampleRate, m_pSound->GetRangeMax () + 1, CKiloSound::ChunkFrames);
#else
	logf (*m_pLog, LogLevel::Info, "sound: not built in (KILO_SOUND=0)");
#endif

	// Light sleep's hardware: the Ethernet PHY (once the net task has it)
	// and the SoC's clock and temperature, logged once here.
	m_pPower = new CKiloPower (m_pLog);
	m_pMind->sleep ().set_hardware (m_pPower);
	kilo::power::Soc Soc;
	if (m_pPower->soc (&Soc))
	{
		logf (*m_pLog, LogLevel::Info, "power: ARM %u MHz, SoC %d C, throttled 0x%x; light sleep %s",
		      (unsigned) (Soc.arm_hz / 1000000), (int) (Soc.temp_mc / 1000), (unsigned) Soc.throttled,
		      KILO_LIGHT_SLEEP ? "on (A7: servos off 3 s after, the Pi after 30 s of nothing)" : "off (KILO_LIGHT_SLEEP=0)");
	}

	kilo::Serial *pSerial = OpenSerial ();
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
	// The boot check reads and writes the card through the same KernelStore
	// as the updater (one file at a time; never while an update writes).
	m_CardFlashHost.m_pMind = m_pMind;
	m_pCardFlash = new CardFlash (*m_pBootCard, *m_pCardImage, *m_pFlasher, m_pMind->link (), m_Clock, *m_pLog);
	m_pCardFlash->set_ring (&m_pMind->log ());
	m_pCardFlash->set_host (&m_CardFlashHost);
	m_pCardFlash->set_mind_build ((uint32_t) KILO_BUILD);

	// Updating this kernel on the card (`kilo update --host`): the host
	// server fills the updater from UPDATE_* and polls it, and while it
	// writes (a 16 KB slice of SD card I/O per pass, the loop stalled for
	// each) the Mind pauses the link as for a flash. The card is mounted at
	// the first write. On DONE the host server asks for the reboot below.
	m_pUpdater = new KernelUpdater (new u8[KernelImageMax], KernelImageMax, *m_pBootCard, m_Clock, *m_pLog);
	m_pMind->host ().attach_updater (*m_pUpdater);

	// The flight recorder (mind/record/recorder.h): from here on every link
	// frame, log line, walk and trick, in 64 MB of RAM (hours: the oldest
	// 16 KB blocks go first), for `kilo runs --host` and `kilo record fetch
	// --host`. Its saver writes it to the card's kilorec/ at safe moments
	// (no walk or trick, nothing moving; a 4 KB write a pass, each timed),
	// when saving is on (KILO_RECORD_SAVE, or the card's SAVE_ON).
	m_pMind->attach_recorder (new u8[record::kStoreBytes], record::kStoreBytes);
	m_pMind->attach_saver (*new record::Saver (m_Clock, *m_pLog, m_pMind->recorder (), m_pMind->log (),
						   *new CRecordCard (m_pBootCard), KILO_RECORD_SAVE != 0));

	// The clock reads nearly the same at every boot, so the core's
	// clock-based seed would repeat (seq, type) pairs inside the body's
	// 500 ms de-dup window after a quick reboot of the mind. The hardware
	// RNG does not.
	m_pMind->link ().seed_seq ((uint8_t) m_Random.GetNumber ());
	m_pMind->start ();		// balance on, then link.connect(): HELLO_REQ
	m_pCardFlash->begin ();

#if KILO_PLANNER_BENCH
	m_pBench = new CPlannerBench (m_pLog);
#endif

	// Last: it first runs at the main loop's first sleep. The USB host was
	// not scanned yet, and the LAN9514 is a USB device, so the task scans
	// first.
	m_pNetTask = new CKiloNet (&m_Net, &m_USBHCI, &m_bGoAhead, m_pLog, m_pListener, (u32) KILO_BUILD);

	return TRUE;
}

// The body link's device: the PL011 on GPIO14/15, opened at once (the Pi
// on the NyBoard's socket, no USB before the link; the net task scans USB
// later).
kilo::Serial *CKernel::OpenSerial (void)
{
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
	uint64_t nRebootAt = 0;			// 0: no REBOOT asked
	unsigned nLastPass = CTimer::GetClockTicks ();

	// From here only warnings and errors reach the screen. A line on the
	// screen holds interrupts off for milliseconds (CScreenDevice's lock,
	// more when it scrolls), longer than the PL011's 16-byte FIFO lasts at
	// 250000 baud (0.64 ms): on the Pi the stats lines cost one overrun,
	// one lost frame, every second (docs/fast-boot-first-boot-results.md).
	// The ring keeps every line for `kilo log --host`.
	m_CircleLog.set_min (LogLevel::Warn);

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
		BootFlash ();

		if (!Link.connected ())
		{
			// The Serial failed. The PL011 does not fail, so this mostly
			// logs once a second.
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

		// The behaviour runner's wind-down ends in a poweroff request; the
		// kernel cannot switch the Pi off, so it only logs it.
		if (m_pMind->poweroff_requested () && !m_bPoweroffLogged)
		{
			m_bPoweroffLogged = TRUE;
			logf (*m_pLog, LogLevel::Warn, "behaviour runner asked for poweroff: the kernel cannot switch "
			      "the Pi off; the runner has stopped, the link stays up");
		}

		// A laptop's REBOOT, or a kernel update installed (HostServer):
		// the answer is queued; give the net task RebootFlushUs of passes to
		// send it, then restart. The firmware boots the SD card again: its
		// kernel7.img (on the netboot card: the loader, which waits for the
		// next kernel; after an update: the new kernel).
		if (m_pMind->reboot_requested ())
		{
			if (nRebootAt == 0)
			{
				nRebootAt = nNow + RebootFlushUs;
			}
			else if (nNow >= nRebootAt)
			{
				logf (*m_pLog, LogLevel::Warn, "rebooting (%s)",
				      m_pUpdater->status ().phase == KILO_UPDATE_PHASE_DONE
				      ? "a new kernel7.img on the card: kilo update" : "REBOOT from the laptop");

				return ShutdownReboot;
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
		// Asleep (A7, then 30 s of nothing: mind/power/sleep.h) the pass
		// waits in wfi instead (LightSleep).
		unsigned nSleepUs = m_pMind->sleep ().loop_period_us ();
		if (nSleepUs == 0)
		{
			m_Scheduler.usSleep (LoopPeriodUs);
		}
		else
		{
			LightSleep (nSleepUs);
		}
	}

	return ShutdownHalt;
}

// One pass's wait in light sleep. CScheduler::usSleep would spin in its
// Yield() while every task sleeps (the scheduler has no idle wfi), so the
// main task stays ready instead: each round gives the other tasks their turn
// (the net task, Circle's PHY and DHCP tasks: those whose wake time has come
// run, then Yield() returns here), then waits in wfi for the next interrupt:
// the timer's 100 Hz tick at the latest, or a UART byte from the body, or
// USB. The core's clock stops in wfi; the ARM's draw then is its leakage.
void CKernel::LightSleep (unsigned nUs)
{
	m_bSlept = TRUE;
	unsigned nStart = CTimer::GetClockTicks ();
	for (;;)
	{
		m_Scheduler.Yield ();
		if (CTimer::GetClockTicks () - nStart >= nUs)
		{
			return;
		}
		WaitForInterrupt ();
		m_nWakeups++;
	}
}

// The boot-time flash from the SD card (mind/flash/cardflash.h; the card
// route, `kilo card PATH`): once the body's first HELLO arrived (or none came
// 3 s after link.connect()), read body.json and body.bin, flash them if the
// body runs another build, and write the outcome to body-result.txt. Then
// the go-ahead for the net task's USB scan, which stalls the main loop and so must not
// cut into a flash.
//
// The card is read only after the HELLO, so neither the link nor the boot
// probe (GPIO21 falls at the first HELLO) waits for it; the read itself
// stalls the loop (the SD driver busy-waits), logged in ms. Never while the
// bridge or a laptop's flash has the UART, or a kernel update the card.
void CKernel::BootFlash (void)
{
	if (m_pCardFlash->done ())
	{
		m_bGoAhead = TRUE;
		return;
	}
	m_pCardFlash->poll (!m_pMind->host ().link_paused () && !m_pUpdater->busy ());
	if (m_pCardFlash->done ())
	{
		m_bGoAhead = TRUE;
	}
}

void CCardFlashHost::card_signal (kilo::signals::Signal Signal, const char *pWhy)
{
	m_pMind->signal (Signal, pWhy);	// no one is connected: the owner sees (and hears) how it went
}

// A walk the remote started in the first seconds ends before the flasher
// takes the UART (abort() may send SET_STREAM, which must go out first). A
// trick is not stopped: the flash resets the body, and the player gives up
// on it by its own timeout.
void CCardFlashHost::card_before_flash (void)
{
	m_pMind->walker ().abort ();
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
		const char *pFrom = m_pCardFlash->flashing () ? "the SD card (boot)" : "the laptop";
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
	Log &Status = m_pMind->log ().status ();	// in the RAM log only while a laptop reads it
	logf (Status, LogLevel::Info,
	      "link %s: telemetry %u/s (total %u) rx %u frames, tx %u (%u heartbeats) "
	      "decode errors %u (crc %u cobs %u overflow %u) tx deferred %u short %u",
	      m_pSerialName, m_Watch.m_nTelemetry, U (b.telemetry_count), U (s.frames_rx),
	      U (s.frames_tx), U (s.heartbeats), U (s.decode_error_total ()),
	      U (s.decode_errors[KILO_FRAME_BAD_CRC]), U (s.decode_errors[KILO_FRAME_BAD_COBS]),
	      U (s.decode_errors[KILO_FRAME_OVERFLOW]), U (s.tx_deferred), U (s.tx_short));
	if (m_pPl011 != 0)
	{
		const unsigned *e = m_pPl011->m_nLineErrors;
		unsigned nErrors = e[1] + e[2] + e[3] + e[4] + e[0] + m_pPl011->m_nDrainTimeouts;
		if (nErrors != 0)
		{
			// A warning (the screen, Run()) only when the count grew: a
			// line on the screen can itself cost an overrun.
			static unsigned s_nLast = 0;
			LogLevel Level = nErrors != s_nLast ? LogLevel::Warn : LogLevel::Info;
			s_nLast = nErrors;
			logf (Status, Level, "PL011 line errors: break %u overrun %u framing %u parity %u; "
			      "baud switches with bytes still queued %u", e[1], e[2], e[3], e[4], m_pPl011->m_nDrainTimeouts);
		}
	}
	if (b.has_telemetry)
	{
		logf (Status, LogLevel::Info, "body: mode %s epoch %u tick %u rx_err %u overruns %u/%u",
		      mode_name (b.telemetry.mode), b.telemetry.epoch, U (b.tick), b.telemetry.rx_err,
		      m_Watch.m_nOverruns, m_Watch.m_nTelemetry);
	}

	// The loop's timing, for the first boot to measure with no scope.
	// period: pass to pass (1000 us asked); work: a pass without the sleep;
	// mind.poll: the mind core's share (the link, the laptop, the walker,
	// whose wake every 20 ms of a walk runs the planner, the trick player,
	// the remote, the behaviour runner's 10 Hz tick).
	logf (Status, LogLevel::Info,
	      "loop %u/s: period max %u mean %u us, work max %u mean %u us, mind.poll max %u mean %u us%s; "
	      "%u clients%s",
	      m_Period.m_nCount, m_Period.m_nMax, m_Period.Mean (), m_Work.m_nMax, m_Work.Mean (),
	      m_MindPoll.m_nMax, m_MindPoll.Mean (), m_pBench != 0 ? " (bench)" : "",
	      U (m_pMind->host ().clients ()), m_bSlept ? " (light sleep)" : "");
	if (m_bSlept)
	{
		// how often the core woke from wfi: the interrupts that keep it up
		logf (Status, LogLevel::Info, "light sleep: %u wfi wakeups/s", m_nWakeups);
	}

	// the session metrics (kilo metrics --host): the worst second so far
	// (a second of light sleep has 10 ms periods by design: not counted)
	record::Metrics &Metrics = m_pMind->metrics ();
	if (m_MindPoll.m_nMax > Metrics.loop_max_us)
	{
		Metrics.loop_max_us = m_MindPoll.m_nMax;
	}
	if (m_Period.m_nMax > Metrics.loop_period_max_us && !m_bSlept)
	{
		Metrics.loop_period_max_us = m_Period.m_nMax;
	}
	m_bSlept = FALSE;
	m_nWakeups = 0;

	m_Watch.m_nTelemetry = 0;
	m_Watch.m_nOverruns = 0;
	m_Period.Reset ();
	m_Work.Reset ();
	m_MindPoll.Reset ();
}

void CKernel::UpdateLED (uint64_t nNowUs)
{
	const BodyState &b = m_pMind->link ().state ();

	// Flashing, or writing a kernel update: two 100 ms flashes a second, a
	// pattern no link state uses (while the card is written the loop stalls
	// a slice at a time, so the flashes stretch).
	if (m_pFlasher->busy () || m_pUpdater->busy ())
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

	// Light sleep: off, but a 50 ms flash every 5 s, so that asleep is not
	// mistaken for dead.
	if (m_pMind->sleep ().asleep ())
	{
		if (nNowUs / 1000 % 5000 < 50)
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
