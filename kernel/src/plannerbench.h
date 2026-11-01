//
// plannerbench.h: the walking planner's cost on the Pi 2 B, measured at boot
// with no body involved (docs/fast-boot.md §7: "check the 20 ms budget early
// in P3"). A Planner of its own, not the walker's: nothing is sent to the
// body. One planner call per main-loop pass, so the loop keeps its 1 ms
// cadence; one log line per pattern. The host's counterpart is
// `make -C kernel/host bench` (kernel/host/tests/bench_planner.cpp).
//
// Off with `make -C kernel KILO_PLANNER_BENCH=0`.
//
#ifndef KILO_SRC_PLANNERBENCH_H
#define KILO_SRC_PLANNERBENCH_H

#include "planner/plan.h"
#include "platform.h"
#include <circle/types.h>

class CPlannerBench
{
public:
	CPlannerBench (kilo::Log *pLog);

	// One step of the bench: a pattern's start(), or one next(). FALSE once
	// every pattern has run (then delete it: the planner is a few KB).
	boolean Step (void);

private:
	void Report (void);

	kilo::Log *m_pLog;
	kilo::planner::Planner m_Planner;
	unsigned m_nPattern;		// index into the pattern list
	boolean m_bStarted;		// the current pattern's start() ran
	unsigned m_nTick;		// next() calls so far for this pattern
	unsigned m_nStartUs;		// start()'s cost
	u64 m_nSumUs;
	unsigned m_nMaxUs;
};

#endif
