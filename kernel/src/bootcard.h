//
// bootcard.h: the body image on the SD card's FAT partition (docs/fast-boot.md
// §6, P4): body.bin and body.json next to kernel7.img, read at boot (path B,
// `kilo flash --sd`) and written after `kilo flash --host --persist`.
//
// Circle's FatFs addon (addon/fatfs, ChaN's FatFs: long names, read and
// write) over its SD driver (addon/SDCard; on the Pi 2 B the SDHOST
// controller, PIO). Circle's own lib/fs/fat reads 8.3 names only, and
// "body.json" is not one.
//
// Every call spins: the SD driver busy-waits (Circle's NO_BUSY_WAIT is off),
// so the main loop stops for the duration. Each call logs how long it took.
// Not for interrupt context or a second core.
//
#ifndef KILO_SRC_BOOTCARD_H
#define KILO_SRC_BOOTCARD_H

#include "flash/image.h"
#include "platform.h"
#include <SDCard/emmc.h>
#include <fatfs/ff.h>
#include <circle/interrupt.h>
#include <circle/timer.h>
#include <circle/types.h>

class CBootCard
{
public:
	CBootCard (CInterruptSystem *pInterrupt, CTimer *pTimer, kilo::Log *pLog);

	// The SD card and its FAT partition, once: later calls return the first
	// one's result without touching the card again.
	boolean Mount (void);

	// body.json and body.bin into rImage (parse_manifest, load_from: size
	// and CRC checked). FALSE if either is missing or bad, logged at Info:
	// no image on the card is not an error.
	boolean LoadBodyImage (kilo::FlashImage &rImage);

	// rImage (ready) as body.bin and body.json: the old body.json is removed
	// first and the new one written last (as `kilo flash --sd` does,
	// kilo/tools/body_image.py), so a power cut never pairs a manifest with
	// half an image (and the CRC is checked at boot anyway). FALSE on any
	// failure, logged.
	boolean PersistBodyImage (const kilo::FlashImage &rImage);

private:
	// The whole file into pBuffer: its size, or ReadMissing, ReadTooBig,
	// ReadError.
	int ReadFile (const char *pName, void *pBuffer, unsigned nSize);
	boolean WriteFile (const char *pName, const void *pData, unsigned nSize);
	static const int ReadMissing = -1;
	static const int ReadTooBig = -2;
	static const int ReadError = -3;

	// "SD:/name"
	void Path (char *pOut, unsigned nSize, const char *pName) const;

private:
	CInterruptSystem *m_pInterrupt;
	CTimer *m_pTimer;
	kilo::Log *m_pLog;
	CEMMCDevice *m_pEMMC;		// built at the first Mount()
	boolean m_bTried;
	boolean m_bMounted;
	FATFS m_FileSystem;
	FIL m_File;
};

#endif
