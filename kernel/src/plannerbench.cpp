//
// plannerbench.cpp: see plannerbench.h.
//
#include "plannerbench.h"
#include "util/log.h"
#include <circle/timer.h>

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
	m_nMaxUs (0)
{
}

boolean CPlannerBench::Step (void)
{
	if (m_nPattern >= PatternCount)
	{
		return FALSE;
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

		return m_nPattern < PatternCount;
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

void CPlannerBench::Report (void)
{
	unsigned nMean100 = m_nTick > 0 ? (unsigned) (m_nSumUs * 100 / m_nTick) : 0;
	logf (*m_pLog, LogLevel::Info,
	      "planner bench %s: start %u us; next() x%u mean %u.%02u us max %u us "
	      "(a tick is 20000 us; Python on the Pi: max 4500 us)",
	      Patterns[m_nPattern], m_nStartUs, m_nTick, nMean100 / 100, nMean100 % 100, m_nMaxUs);
}
