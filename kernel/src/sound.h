//
// sound.h: Kilo's sounds on the Pi 2 B's 3.5 mm jack (docs/sound.md): the
// mind core's sound output (mind/sound/render.h SoundOutput) on Circle's
// PWM sound device, which drives the jack's two channels (GPIO40 and 45)
// with PWM, fed by DMA from two buffers; the DMA's interrupt asks for the
// next buffer (GetChunk), which the core's Renderer fills.
//
// The device: 44.1 kHz (the PWM carrier is the sample rate in M/S mode,
// Circle's default: above hearing, and the jack's filter takes most of
// it), range 250 MHz / 44100 = 5669 steps (about 12.5 bits), chunks of
// ChunkFrames stereo frames (11.6 ms). Mono: the same value on both
// channels.
//
// The interrupt: GetChunk runs with interrupts off (Circle does not nest
// them), so it holds off the PL011's receive interrupt, whose FIFO lasts
// about 0.5 ms at 250000 baud past its trigger level. It does only the
// Renderer's integer loop over the chunk (no logging, no allocation, no
// floating point) and times itself with the 1 MHz system timer: the
// longest and the mean go to the metrics line (callback_max_us), and a
// chunk asked for more than 1.5 chunk periods after the one before counts
// as late (the DMA may have replayed a buffer: a click).
//
// Idle: the device runs only while a sound plays: Start() for a sound, the
// DC level ramped up from 0 (kRampMs), the sound, the level ramped down,
// then GetChunk returns 0 and the DMA stops. No interrupts and no PWM
// carrier while idle (no idle hiss from the jack); each sound costs two
// 80 ms ramps. Light sleep stops a sound that is playing (the level ramped
// down first).
//
// The main loop's calls into the Renderer are wrapped in EnterCritical /
// LeaveCritical: a few instructions with interrupts off, so they never
// interleave with GetChunk.
//
#ifndef KILO_SRC_SOUND_H
#define KILO_SRC_SOUND_H
#include "sound/render.h"	// the core's seam (mind/sound/render.h)
#include <circle/interrupt.h>
#include <circle/sound/pwmsoundbasedevice.h>
#include <circle/types.h>

class CKiloSound final : public CPWMSoundBaseDevice, public kilo::sound::SoundOutput
{
public:
	static const unsigned SampleRate = kilo::sound::kOutRate;
	static const unsigned ChunkFrames = 512;		// 11.6 ms at 44.1 kHz

	CKiloSound (CInterruptSystem *pInterrupt);

	// The range. The device starts with the first sound.
	void Initialize (void);

	// kilo::sound::SoundOutput
	bool play (const kilo::sound::Sound &rSound, uint32_t nGainQ16) override;
	bool playing (void) override;
	void stop (void) override;
	void suspend (bool bAsleep) override;
	void poll (void) override;
	void stats (kilo::sound::OutputStats &rOut) override;

protected:
	unsigned GetChunk (u32 *pBuffer, unsigned nChunkSize) override;

private:
	void Run (void);	// start the DMA if the Renderer has something to send

	kilo::sound::Renderer m_Renderer;
	boolean m_bAsleep;
	boolean m_bFailed;
	unsigned m_nPeriodUs;	// one chunk

	// Written in the interrupt, read under EnterCritical.
	volatile boolean m_bFirst;		// the next chunk is the first after Start()
	volatile unsigned m_nLastAt;
	volatile unsigned m_nChunks;
	volatile unsigned m_nLate;
	volatile unsigned m_nIntervalMax;
	volatile unsigned m_nCallbackMax;
	volatile u64 m_nCallbackSum;

	unsigned m_nStarts;
	unsigned m_nFailures;
};

#endif
