//
// sound.cpp: Kilo's sounds on the PWM sound device (sound.h).
//
#include "sound.h"
#include "proto/kilo_protocol.h"
#include <circle/synchronize.h>
#include <circle/timer.h>

CKiloSound::CKiloSound (CInterruptSystem *pInterrupt, boolean bKeepRunning)
:	CPWMSoundBaseDevice (pInterrupt, SampleRate, 2 * ChunkFrames),
	m_Renderer (SampleRate),
	m_bKeepRunning (bKeepRunning),
	m_bAsleep (FALSE),
	m_bFailed (FALSE),
	m_nPeriodUs (ChunkFrames * 1000000u / SampleRate),
	m_bFirst (TRUE),
	m_nLastAt (0),
	m_nChunks (0),
	m_nLate (0),
	m_nIntervalMax (0),
	m_nCallbackMax (0),
	m_nCallbackSum (0),
	m_nStarts (0),
	m_nFailures (0)
{
}

boolean CKiloSound::Initialize (void)
{
	m_Renderer.set_max ((uint32_t) GetRangeMax ());
	if (m_bKeepRunning)
	{
		m_Renderer.power (true);
		Run ();
	}

	return !m_bFailed;
}

void CKiloSound::Run (void)
{
	if (m_bFailed || IsActive () || m_Renderer.settled_off ())
	{
		return;
	}
	m_bFirst = TRUE;
	if (!Start ())
	{
		m_bFailed = TRUE;	// the DMA channel or its buffers: it will not come
		m_nFailures++;
		EnterCritical ();
		m_Renderer.power (false);
		LeaveCritical ();
		return;
	}
	m_nStarts++;
}

bool CKiloSound::play (const kilo::sound::Sound &rSound, uint32_t nGainQ16)
{
	if (m_bFailed || m_bAsleep)
	{
		return false;
	}
	EnterCritical ();
	m_Renderer.power (true);
	m_Renderer.start (rSound, nGainQ16);
	LeaveCritical ();
	Run ();			// stopped: Start() (stopping: poll() starts it once the DMA is idle)

	return !m_bFailed;
}

bool CKiloSound::playing (void)
{
	EnterCritical ();
	bool bSounding = m_Renderer.sounding ();
	LeaveCritical ();

	return bSounding;
}

void CKiloSound::stop (void)
{
	EnterCritical ();
	m_Renderer.stop ();
	LeaveCritical ();
}

void CKiloSound::suspend (bool bAsleep)
{
	m_bAsleep = bAsleep;
	EnterCritical ();
	if (bAsleep)
	{
		m_Renderer.power (false);	// the level down, then GetChunk returns 0: the DMA stops
	}
	else if (m_bKeepRunning)
	{
		m_Renderer.power (true);
	}
	LeaveCritical ();
	if (!bAsleep)
	{
		Run ();
	}
}

void CKiloSound::poll (void)
{
	EnterCritical ();
	if (!m_bKeepRunning && m_Renderer.powered () && !m_Renderer.sounding ())
	{
		m_Renderer.power (false);	// "stop": the sound is over, the level goes down
	}
	LeaveCritical ();
	Run ();				// a sound asked for while the DMA was stopping
}

void CKiloSound::stats (kilo::sound::OutputStats &rOut)
{
	rOut = kilo::sound::OutputStats ();
	EnterCritical ();
	bool bSounding = m_Renderer.sounding ();
	rOut.chunks = m_nChunks;
	rOut.late = m_nLate;
	rOut.interval_max_us = m_nIntervalMax;
	rOut.callback_max_us = m_nCallbackMax;
	rOut.callback_mean_us = m_nChunks != 0 ? (uint32_t) (m_nCallbackSum / m_nChunks) : 0;
	LeaveCritical ();
	rOut.starts = m_nStarts;
	rOut.failures = m_nFailures;
	rOut.state =   m_bFailed ? KILO_SOUND_STATE_FAILED
		     : m_bAsleep ? KILO_SOUND_STATE_ASLEEP
		     : bSounding ? KILO_SOUND_STATE_PLAYING : KILO_SOUND_STATE_IDLE;
}

// In the DMA's interrupt (and once from Start(), for the second buffer).
unsigned CKiloSound::GetChunk (u32 *pBuffer, unsigned nChunkSize)
{
	unsigned nStart = CTimer::GetClockTicks ();
	if (!m_bFirst)
	{
		unsigned nInterval = nStart - m_nLastAt;
		if (nInterval > m_nIntervalMax)
		{
			m_nIntervalMax = nInterval;
		}
		if (nInterval > m_nPeriodUs + m_nPeriodUs / 2)
		{
			m_nLate++;
		}
	}
	m_bFirst = FALSE;
	m_nLastAt = nStart;

	unsigned nWords = m_Renderer.render (pBuffer, nChunkSize / 2);

	unsigned nUs = CTimer::GetClockTicks () - nStart;
	m_nChunks++;
	m_nCallbackSum += nUs;
	if (nUs > m_nCallbackMax)
	{
		m_nCallbackMax = nUs;
	}

	return nWords;		// 0: settled off, the DMA stops after the buffer playing now
}
