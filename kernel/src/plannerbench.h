//
// plannerbench.h: the walking engine's cost on the Pi 2 B, measured at boot
// with no body involved (docs/fast-boot.md §7: "check the 20 ms budget early
// in P3"). A Solver of its own (engine/solve.h), not the walker's: nothing is sent to the
// body. One solver call per main-loop pass, so the loop keeps its 1 ms
// cadence; one log line per gait. The host's counterpart is
// `make -C kernel/host bench` (kernel/host/tests/bench_engine.cpp).
//
// Then the host channel's Noise handshake (kernel/README.md "Who may talk to
// Kilo: keys") between two fixed test keys, one X25519 per pass as the host
// server does it, and the cost of sealing and opening a full record; one line:
//   noise bench: X25519 keygen N us, ee N us, se N us (the mind's three passes); ...
// The host's counterpart: kernel/host/tests/bench_noise.cpp.
//
// Off with `make -C kernel KILO_PLANNER_BENCH=0`.
//
#ifndef KILO_SRC_PLANNERBENCH_H
#define KILO_SRC_PLANNERBENCH_H

#include "net/noise.h"
#include "proto/kilo_protocol.h"
#include "engine/solve.h"
#include "platform.h"
#include <circle/types.h>

class CPlannerBench
{
public:
	CPlannerBench (kilo::Log *pLog);

	// One step of the bench: a pattern's start(), or one next(). FALSE once
	// every gait has run (then delete it: the solver is a few KB).
	boolean Step (void);

private:
	boolean NoiseStep (void);
	void Report (void);

	kilo::Log *m_pLog;
	kilo::engine::Solver m_Planner;
	unsigned m_nPattern;		// index into the pattern list
	boolean m_bStarted;		// the current pattern's start() ran
	unsigned m_nTick;		// next() calls so far for this pattern
	unsigned m_nStartUs;		// start()'s cost
	u64 m_nSumUs;
	unsigned m_nMaxUs;

	// The Noise phase, after the patterns.
	static const unsigned NoiseRecords = 20;
	unsigned m_nNoiseStep;
	kilo::noise::HandshakeKN m_Initiator, m_Responder;
	kilo::noise::CipherState m_ISend, m_IRecv, m_RSend, m_RRecv;
	u8 m_Msg[64];
	u8 m_Record[KILO_HOST_RECORD_MAX + kilo::noise::kTagLen];
	unsigned m_nInitiatorUs[2];	// message 1, message 2
	unsigned m_nStepUs[3];		// keygen, ee, se
	unsigned m_nSealMaxUs, m_nOpenMaxUs;
	u64 m_nSealSumUs, m_nOpenSumUs;
	boolean m_bNoiseOk;
};

#endif
