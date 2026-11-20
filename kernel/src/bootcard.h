//
// bootcard.h: the SD card's FAT partition. The core's KernelStore
// (mind/platform.h) for two users that never overlap: the kernel updater
// (mind/update/updater.h, `kilo update --host`: kernel7.new, kernel7.img,
// kernel7.old) and the boot-time flash check (mind/flash/cardflash.h, the
// card route: body.json, body.bin, body-result.txt). And the body image
// written after `kilo flash --host --persist` (body.bin, body.json).
//
// Circle's FatFs addon (addon/fatfs, ChaN's FatFs: long names, read and
// write) over its SD driver (addon/SDCard; on the Pi 2 B the SDHOST
// controller, PIO). Circle's own lib/fs/fat reads 8.3 names only, and
// "body.json" is not one.
//
// Every call spins: the SD driver busy-waits (Circle's NO_BUSY_WAIT is off),
// so the main loop stops for the duration. --persist logs how long it took;
// the updater and the boot check log their own steps. Not for interrupt
// context or a second core.
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

class CBootCard : public kilo::KernelStore
{
public:
	CBootCard (CInterruptSystem *pInterrupt, CTimer *pTimer, kilo::Log *pLog);

	// The SD card and its FAT partition, once: later calls return the first
	// one's result without touching the card again.
	boolean Mount (void);

	// rImage (ready) as body.bin and body.json: the old body.json is removed
	// first and the new one written last (as `kilo flash --sd` does,
	// kilo/tools/body_image.py), so a power cut never pairs a manifest with
	// half an image (and the CRC is checked at boot anyway). FALSE on any
	// failure, logged.
	boolean PersistBodyImage (const kilo::FlashImage &rImage);

	// kilo::KernelStore, for the kernel updater and the boot flash check:
	// its own file object, so --persist (one pass) cannot disturb a file the
	// updater keeps open across passes. The first call mounts the card.
	// Failures are logged with FatFs's error and the file's name.
	Result create (const char *pName) override;
	Result open (const char *pName) override;
	Result write (const uint8_t *pData, uint32_t nLen) override;
	int32_t read (uint8_t *pBuffer, uint32_t nLen) override;
	Result close (void) override;
	Result remove (const char *pName) override;
	Result rename (const char *pFrom, const char *pTo) override;
	Result sync (void) override;

private:
	boolean WriteFile (const char *pName, const void *pData, unsigned nSize);

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
	FIL m_KernelFile;		// the KernelStore's open file
	boolean m_bKernelOpen;
	boolean m_bKernelWriting;
	char m_OpenName[32];		// the KernelStore's open file, for the log

	friend class CRecordCard;
};

// The flight recorder's saver (mind/record/saver.h) on the same FAT
// partition: kilo::RecordCard with a file and a directory object of its
// own, so neither the updater nor the body image calls disturb it. The
// first ready() mounts the card (CBootCard::Mount: once, at the boot flash
// check normally). Every call spins as the SD driver does; the saver makes
// at most one a pass and times it.
class CRecordCard : public kilo::RecordCard
{
public:
	CRecordCard (CBootCard *pCard);

	bool ready (void) override;
	Result make_dir (const char *pPath) override;
	Result create (const char *pPath) override;
	Result append (const char *pPath) override;
	Result open (const char *pPath) override;
	Result seek (uint32_t nOffset) override;
	Result write (const uint8_t *pData, uint32_t nLen) override;
	int32_t read (uint8_t *pBuffer, uint32_t nLen) override;
	Result sync (void) override;
	Result close (void) override;
	Result remove (const char *pPath) override;
	Result rename (const char *pFrom, const char *pTo) override;
	Result list_begin (const char *pDir) override;
	Result list_next (char *pName, uint32_t nSize, uint32_t *pSize) override;

private:
	Result Open (const char *pPath, BYTE uchMode);

	CBootCard *m_pCard;
	FIL m_File;
	boolean m_bOpen;
	DIR m_Dir;
	boolean m_bListing;
};

#endif
