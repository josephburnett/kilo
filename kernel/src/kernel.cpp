//
// kernel.cpp: the Kilo mind kernel, phase P1 of docs/fast-boot.md: the body
// link. It exchanges HELLO with the body, keeps the link fed (HEARTBEAT,
// HELLO_REQ while the body is lost), logs link statistics once a second and
// shows the link's state on the ACT LED.
//
// Logs go to the HDMI screen if the firmware gives us a frame buffer, else
// nowhere (Kilo has no monitor; the laptop's log tail is P2). The LED is the
// observable result:
//
//   steady on      starting up (the USB scan takes about a second or more)
//   0.5 Hz blink   no body: no HELLO yet, or no TELEMETRY for 300 ms
//   2 Hz blink     the body is alive: TELEMETRY flowing
//   5 Hz blink     the body speaks another protocol major: not driven (flash it)
//
#include "kernel.h"
#include "bootprobe.h"
#include "link/names.h"
#include "util/log.h"

#ifndef KILO_BUILD
#define KILO_BUILD 0
#endif

using namespace kilo;

static const char FromKernel[] = "kilo";

// The main loop's period. The link wants poll() at least every ~10 ms (the
// heartbeat is due after 100 ms of silence, a command is retried after
// 50 ms); 1 ms keeps the RX ring short and leaves room for P2's net task.
static const unsigned LoopPeriodUs = 1000;

// After the Serial failed: how often the link tries again.
static const uint64_t ReconnectUs = 1000000;

// No TELEMETRY this long: the body is lost (the link's own rule).
static const uint32_t BodyLostUs = KILO_BODY_LOST_MS * 1000u;

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
	m_USBHCI (&m_Interrupt, &m_Timer),	// no plug-and-play: one scan at boot
	m_Log (&m_Logger, LogLevel::Info),
	m_pPl011 (0),
	m_pUsbSerial (0),
	m_pSerial (0),
	m_pSerialName ("none"),
	m_pLink (0)
{
	m_ActLED.On ();		// first sign of life, before anything can fail
	m_Watch.m_pClock = &m_Clock;
	m_Watch.m_pLog = &m_Log;
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

	m_Logger.Write (FromKernel, LogNotice, "Kilo mind, build %08x", (unsigned) KILO_BUILD);

	m_pSerial = ChooseSerial ();
	if (m_pSerial == 0)
	{
		return FALSE;
	}

	m_pLink = new SerialLink (*m_pSerial, m_Clock, m_Log);
	m_pLink->set_listener (&m_Watch);
	// The clock reads nearly the same at every boot, so the core's clock-based
	// seed would repeat (seq, type) pairs inside the body's 500 ms de-dup
	// window after a quick reboot of the mind. The hardware RNG does not.
	m_pLink->seed_seq ((uint8_t) m_Random.GetNumber ());
	m_pLink->connect ();

	return TRUE;
}

// The body link's device. Rule: a USB serial adapter (utty1, the CH343 on
// Kilo's cable today) if one enumerated, else the PL011 on GPIO14/15. Built
// with KILO_BODY_SERIAL=pl011, the USB scan is skipped and the PL011 is
// used at once (the production wiring: no USB before the link, plan rule 1).
kilo::Serial *CKernel::ChooseSerial (void)
{
#ifndef KILO_BODY_SERIAL_PL011
	if (!m_USBHCI.Initialize ())
	{
		m_Logger.Write (FromKernel, LogError, "USB host failed: trying the PL011");
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
				m_Logger.Write (FromKernel, LogWarning, "utty1: cannot set %u baud",
						(unsigned) KILO_BAUD);
			}
			m_pSerialName = "utty1 (USB serial)";
			m_Logger.Write (FromKernel, LogNotice, "body link on %s, %u baud",
					m_pSerialName, (unsigned) KILO_BAUD);

			return m_pUsbSerial;
		}

		m_Logger.Write (FromKernel, LogNotice, "no USB serial adapter (utty1)");
	}
#endif

	m_pPl011 = new Pl011Serial (&m_Interrupt);
	if (!m_pPl011->Initialize (KILO_BAUD))
	{
		m_Logger.Write (FromKernel, LogError, "PL011 failed");

		return 0;
	}
	m_pSerialName = "ttyS1 (PL011, GPIO14/15)";
	m_Logger.Write (FromKernel, LogNotice, "body link on %s, %u baud",
			m_pSerialName, (unsigned) KILO_BAUD);

	return m_pPl011;
}

TShutdownMode CKernel::Run (void)
{
	uint64_t nNextStats = m_Clock.now_us () + 1000000;
	uint64_t nNextReconnect = 0;

	for (;;)
	{
		// This loop is the whole mind in P1: it is alive while it runs.
		m_pLink->mark_alive ();
		m_pLink->poll ();

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

		if (nNow >= nNextStats)
		{
			nNextStats += 1000000;
			LogStats ();
		}

		UpdateLED (nNow);

		// Sleep, not CTimer::usDelay: a delay spins with nothing else running,
		// a sleep hands the CPU to the scheduler. Alone (P1) the scheduler
		// spins until the wake tick, the same cost; from P2 the net task runs
		// in this gap. The scheduler is cooperative, so a task that does not
		// yield within a few ms delays the link (it tolerates ~10 ms).
		m_Scheduler.usSleep (LoopPeriodUs);
	}

	return ShutdownHalt;
}

// uint32_t is unsigned long on arm-none-eabi; the core's %u takes unsigned.
static inline unsigned U (uint32_t n) { return (unsigned) n; }

void CKernel::LogStats (void)
{
	const LinkStats &s = m_pLink->stats ();
	const BodyState &b = m_pLink->state ();
	logf (m_Log, LogLevel::Info,
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
			logf (m_Log, LogLevel::Warn, "PL011 line errors: break %u overrun %u framing %u parity %u",
			      e[1], e[2], e[3], e[4]);
		}
	}
	if (b.has_telemetry)
	{
		logf (m_Log, LogLevel::Info, "body: mode %s epoch %u tick %u rx_err %u overruns %u/%u",
		      mode_name (b.telemetry.mode), b.telemetry.epoch, U (b.tick), b.telemetry.rx_err,
		      m_Watch.m_nOverruns, m_Watch.m_nTelemetry);
	}
	m_Watch.m_nTelemetry = 0;
	m_Watch.m_nOverruns = 0;
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
