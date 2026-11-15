//
// plannerbench.cpp: see plannerbench.h.
//
#include "plannerbench.h"
#include "third_party/monocypher/monocypher.h"
#include "util/log.h"
#include <circle/timer.h>
#include <circle/util.h>

using namespace kilo;
using namespace kilo::planner;

// The walk each pattern gets, as one repetition of bench_planner.cpp: its
// default speed, then 40 mm/s, a turn to the left, a change of pattern, and
// the planned stop.
static const char *const Patterns[] = {"trot", "walk", "crawl"};
static const unsigned PatternCount = sizeof Patterns / sizeof Patterns[0];
static const unsigned SpeedAt = 400, TurnAt = 800, ChangeAt = 1100, StopAt = 1400, MaxTicks = 2400;

CPlannerBench::CPlannerBench (kilo::Log *pLog)
:	m_pLog (pLog),
	m_nPattern (0),
	m_bStarted (FALSE),
	m_nTick (0),
	m_nStartUs (0),
	m_nSumUs (0),
	m_nMaxUs (0),
	m_nNoiseStep (0),
	m_nInitiatorUs {0, 0},
	m_nStepUs {0, 0, 0},
	m_nSealMaxUs (0),
	m_nOpenMaxUs (0),
	m_nSealSumUs (0),
	m_nOpenSumUs (0),
	m_bNoiseOk (TRUE)
{
}

boolean CPlannerBench::Step (void)
{
	if (m_nPattern >= PatternCount)
	{
		return NoiseStep ();
	}

	const Pattern *pPattern = find_pattern (Patterns[m_nPattern]);
	if (pPattern == 0)
	{
		m_nPattern++;
		return TRUE;
	}

	if (!m_bStarted)
	{
		// The first slot's whole lean is solved here: the costliest call.
		unsigned t0 = CTimer::GetClockTicks ();
		m_Planner.start (*pPattern, Direction::Forward, -1, 1000);
		m_nStartUs = CTimer::GetClockTicks () - t0;
		m_bStarted = TRUE;
		m_nTick = 0;
		m_nSumUs = 0;
		m_nMaxUs = 0;

		return TRUE;
	}

	if (m_nTick == SpeedAt)
	{
		m_Planner.set_speed (40);
	}
	if (m_nTick == TurnAt)
	{
		m_Planner.set_direction (Direction::Left);
	}
	if (m_nTick == ChangeAt)
	{
		const Pattern *pOther = find_pattern (m_nPattern == 0 ? "walk" : "trot");
		if (pOther != 0)
		{
			m_Planner.set_pattern (*pOther);
		}
	}
	if (m_nTick == StopAt)
	{
		m_Planner.stop ();
	}

	if (m_Planner.done () || m_nTick >= MaxTicks)
	{
		Report ();
		m_nPattern++;
		m_bStarted = FALSE;

		return TRUE;	// the Noise phase follows
	}

	unsigned t0 = CTimer::GetClockTicks ();
	m_Planner.next ();
	unsigned nUs = CTimer::GetClockTicks () - t0;
	m_nSumUs += nUs;
	if (nUs > m_nMaxUs)
	{
		m_nMaxUs = nUs;
	}
	m_nTick++;

	return TRUE;
}

// One X25519 operation (or one record sealed and opened) per call, as the
// host server spreads its handshake over passes.
boolean CPlannerBench::NoiseStep (void)
{
	using namespace kilo::noise;
	static const u8 Prologue[] = "kilo noise bench";
	unsigned nStep = m_nNoiseStep++;
	unsigned t0 = CTimer::GetClockTicks ();
	if (nStep == 0)
	{
		u8 SPriv[32], SPub[32], EI[32];
		for (unsigned i = 0; i < 32; i++)
		{
			SPriv[i] = (u8) (i * 7 + 1);
			EI[i] = (u8) (i * 5 + 3);
		}
		crypto_x25519_public_key (SPub, SPriv);	// (not timed: the laptop's)
		t0 = CTimer::GetClockTicks ();
		m_Initiator.init_initiator (Prologue, sizeof Prologue, SPriv, SPub, EI);
		m_Initiator.write_message1 (0, 0, m_Msg);
		m_nInitiatorUs[0] = CTimer::GetClockTicks () - t0;
		u8 ER[32];
		for (unsigned i = 0; i < 32; i++)
		{
			ER[i] = (u8) (i * 3 + 9);
		}
		m_Responder.init_responder (Prologue, sizeof Prologue, SPub, ER);
		m_bNoiseOk = m_Responder.read_message1 (m_Msg, HandshakeKN::kMsg1Len, 0);
		return TRUE;
	}
	if (nStep <= 3)
	{
		if (nStep == 1)
		{
			m_Responder.responder_keygen ();
		}
		else if (nStep == 2)
		{
			m_bNoiseOk = m_bNoiseOk && m_Responder.responder_ee ();
		}
		else
		{
			m_bNoiseOk = m_bNoiseOk && m_Responder.responder_se (0, 0, m_Msg, m_RSend, m_RRecv);
		}
		m_nStepUs[nStep - 1] = CTimer::GetClockTicks () - t0;
		return TRUE;
	}
	if (nStep == 4)
	{
		m_bNoiseOk = m_bNoiseOk && m_Initiator.read_message2 (m_Msg, HandshakeKN::kMsg2Len, 0, m_ISend, m_IRecv);
		m_nInitiatorUs[1] = CTimer::GetClockTicks () - t0;
		memset (m_Record, 0x5A, sizeof m_Record);
		return TRUE;
	}
	if (nStep < 5 + NoiseRecords)
	{
		m_bNoiseOk = m_bNoiseOk && m_RSend.encrypt (0, 0, m_Record, KILO_HOST_RECORD_MAX, m_Record);
		unsigned t1 = CTimer::GetClockTicks ();
		m_bNoiseOk = m_bNoiseOk && m_IRecv.decrypt (0, 0, m_Record, sizeof m_Record, m_Record);
		unsigned t2 = CTimer::GetClockTicks ();
		unsigned nSeal = t1 - t0, nOpen = t2 - t1;
		m_nSealSumUs += nSeal;
		m_nOpenSumUs += nOpen;
		m_nSealMaxUs = nSeal > m_nSealMaxUs ? nSeal : m_nSealMaxUs;
		m_nOpenMaxUs = nOpen > m_nOpenMaxUs ? nOpen : m_nOpenMaxUs;
		return TRUE;
	}
	logf (*m_pLog, m_bNoiseOk ? LogLevel::Info : LogLevel::Error,
	      "noise bench: X25519 keygen %u us, ee %u us, se %u us (the mind's handshake, a pass each); "
	      "the laptop's side %u + %u us; a %u-byte record: seal mean %u max %u us, open mean %u max %u us%s",
	      m_nStepUs[0], m_nStepUs[1], m_nStepUs[2], m_nInitiatorUs[0], m_nInitiatorUs[1],
	      (unsigned) KILO_HOST_RECORD_MAX, (unsigned) (m_nSealSumUs / NoiseRecords), m_nSealMaxUs,
	      (unsigned) (m_nOpenSumUs / NoiseRecords), m_nOpenMaxUs, m_bNoiseOk ? "" : ": FAILED (a step refused)");
	m_Initiator.wipe ();
	m_Responder.wipe ();
	return FALSE;
}

void CPlannerBench::Report (void)
{
	unsigned nMean100 = m_nTick > 0 ? (unsigned) (m_nSumUs * 100 / m_nTick) : 0;
	logf (*m_pLog, LogLevel::Info,
	      "planner bench %s: start %u us; next() x%u mean %u.%02u us max %u us "
	      "(a tick is 20000 us; Python on the Pi: max 4500 us)",
	      Patterns[m_nPattern], m_nStartUs, m_nTick, nMean100 / 100, nMean100 % 100, m_nMaxUs);
}
