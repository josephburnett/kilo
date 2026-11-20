//
// bootcard.cpp: the SD card's FAT partition: --persist's body.bin and
// body.json, and the store of the kernel updater and the boot-time flash
// check (see bootcard.h).
//
#include "bootcard.h"
#include "flash/flasher.h"
#include "util/fmt.h"
#include "util/log.h"
#include <circle/util.h>

using namespace kilo;

// FatFs's volume string for the SD card (addon/fatfs/ffconf.h FF_VOLUME_STRS).
static const char Drive[] = "SD:";

static inline unsigned Elapsed (unsigned nStart)
{
	return (CTimer::GetClockTicks () - nStart) / 1000;	// ms
}

CBootCard::CBootCard (CInterruptSystem *pInterrupt, CTimer *pTimer, kilo::Log *pLog)
:	m_pInterrupt (pInterrupt),
	m_pTimer (pTimer),
	m_pLog (pLog),
	m_pEMMC (0),
	m_bTried (FALSE),
	m_bMounted (FALSE),
	m_bKernelOpen (FALSE),
	m_bKernelWriting (FALSE)
{
	memset (&m_FileSystem, 0, sizeof m_FileSystem);
	memset (&m_File, 0, sizeof m_File);
	memset (&m_KernelFile, 0, sizeof m_KernelFile);
	m_OpenName[0] = '\0';
}

boolean CBootCard::Mount (void)
{
	if (m_bTried)
	{
		return m_bMounted;
	}
	m_bTried = TRUE;

	// No ACT LED for disk access (the third argument): the LED is the
	// kernel's status display.
	unsigned nStart = CTimer::GetClockTicks ();
	m_pEMMC = new CEMMCDevice (m_pInterrupt, m_pTimer, 0);
	if (!m_pEMMC->Initialize ())
	{
		logf (*m_pLog, LogLevel::Warn, "SD: the card did not initialise (%u ms): no body image from SD, "
		      "no result file, no --persist", Elapsed (nStart));
		return FALSE;
	}
	unsigned nCardMs = Elapsed (nStart);

	// 1: mount now (read the boot sector), so a bad partition shows here.
	FRESULT Result = f_mount (&m_FileSystem, Drive, 1);
	if (Result != FR_OK)
	{
		logf (*m_pLog, LogLevel::Warn, "SD: no FAT partition (FatFs error %u, %u ms)", (unsigned) Result,
		      Elapsed (nStart));
		return FALSE;
	}

	m_bMounted = TRUE;
	logf (*m_pLog, LogLevel::Info, "SD: FAT partition mounted in %u ms (card %u ms; the main loop waited)",
	      Elapsed (nStart), nCardMs);

	return TRUE;
}

boolean CBootCard::PersistBodyImage (const FlashImage &rImage)
{
	if (!rImage.ready ())
	{
		return FALSE;
	}
	if (!Mount ())
	{
		logf (*m_pLog, LogLevel::Warn, "SD: --persist: no card to write to");
		return FALSE;
	}

	unsigned nStart = CTimer::GetClockTicks ();
	const Manifest &m = rImage.manifest ();

	char Name[32];
	Path (Name, sizeof Name, kManifestFile);
	FRESULT Result = f_unlink (Name);
	if (Result != FR_OK && Result != FR_NO_FILE)
	{
		logf (*m_pLog, LogLevel::Warn, "SD: --persist: cannot remove the old %s (FatFs error %u)", kManifestFile,
		      (unsigned) Result);
		return FALSE;
	}

	// The image is whole pages in RAM, contiguous from page 0.
	if (!WriteFile (kImageFile, rImage.page (0), m.size))
	{
		return FALSE;
	}

	char Text[128];
	uint32_t nLen = format_manifest (m, Text, sizeof Text);
	if (nLen == 0 || !WriteFile (kManifestFile, Text, nLen))
	{
		return FALSE;
	}

	logf (*m_pLog, LogLevel::Info, "SD: --persist: wrote %s (%u bytes) and %s for build %08x in %u ms "
	      "(the main loop waited)", kImageFile, (unsigned) m.size, kManifestFile, (unsigned) m.build,
	      Elapsed (nStart));

	return TRUE;
}

boolean CBootCard::WriteFile (const char *pName, const void *pData, unsigned nSize)
{
	char Name[32];
	Path (Name, sizeof Name, pName);

	FRESULT Result = f_open (&m_File, Name, FA_WRITE | FA_CREATE_ALWAYS);
	if (Result != FR_OK)
	{
		logf (*m_pLog, LogLevel::Warn, "SD: --persist: cannot create %s (FatFs error %u%s)", pName,
		      (unsigned) Result, Result == FR_WRITE_PROTECTED ? ": write protected" : "");
		return FALSE;
	}

	UINT nWritten = 0;
	Result = f_write (&m_File, pData, nSize, &nWritten);
	FRESULT CloseResult = f_close (&m_File);	// flushes the data, the FAT and the directory entry
	if (Result != FR_OK || nWritten != nSize || CloseResult != FR_OK)
	{
		logf (*m_pLog, LogLevel::Warn, "SD: --persist: writing %s failed (FatFs error %u/%u, %u of %u bytes)",
		      pName, (unsigned) Result, (unsigned) CloseResult, (unsigned) nWritten, nSize);
		return FALSE;
	}

	return TRUE;
}

void CBootCard::Path (char *pOut, unsigned nSize, const char *pName) const
{
	format (pOut, nSize, "%s/%s", Drive, pName);
}

// ---- the store of the kernel updater and the boot flash check (kilo::KernelStore) ------
//
// FatFs writes the file's data at f_write (multi-sector, straight from the
// buffer when aligned) and the FAT and directory entry at f_close; f_unlink
// and f_rename update the directory and sync the volume before they
// return, so sync() has nothing left to do.

static KernelStore::Result Failed (Log *pLog, const char *pWhat, const char *pName, FRESULT Result)
{
	logf (*pLog, LogLevel::Warn, "SD: %s %s (FatFs error %u%s)", pWhat, pName, (unsigned) Result,
	      Result == FR_WRITE_PROTECTED ? ": write protected" : "");
	return KernelStore::Result::Failed;
}

KernelStore::Result CBootCard::create (const char *pName)
{
	if (m_bKernelOpen || !Mount ())
	{
		return Result::Failed;
	}
	char Name[32];
	Path (Name, sizeof Name, pName);
	FRESULT Result = f_open (&m_KernelFile, Name, FA_WRITE | FA_CREATE_ALWAYS);
	if (Result != FR_OK)
	{
		return Failed (m_pLog, "cannot create", pName, Result);
	}
	m_bKernelOpen = TRUE;
	m_bKernelWriting = TRUE;
	format (m_OpenName, sizeof m_OpenName, "%s", pName);

	return Result::Ok;
}

KernelStore::Result CBootCard::open (const char *pName)
{
	if (m_bKernelOpen || !Mount ())
	{
		return Result::Failed;
	}
	char Name[32];
	Path (Name, sizeof Name, pName);
	FRESULT Result = f_open (&m_KernelFile, Name, FA_READ | FA_OPEN_EXISTING);
	if (Result == FR_NO_FILE || Result == FR_NO_PATH)
	{
		return Result::Missing;
	}
	if (Result != FR_OK)
	{
		return Failed (m_pLog, "cannot open", pName, Result);
	}
	m_bKernelOpen = TRUE;
	m_bKernelWriting = FALSE;
	format (m_OpenName, sizeof m_OpenName, "%s", pName);

	return Result::Ok;
}

KernelStore::Result CBootCard::write (const uint8_t *pData, uint32_t nLen)
{
	if (!m_bKernelOpen || !m_bKernelWriting)
	{
		return Result::Failed;
	}
	UINT nWritten = 0;
	FRESULT Result = f_write (&m_KernelFile, pData, (UINT) nLen, &nWritten);
	if (Result != FR_OK || nWritten != nLen)
	{
		logf (*m_pLog, LogLevel::Warn, "SD: writing %s failed (FatFs error %u, %u of %u bytes%s)",
		      m_OpenName, (unsigned) Result, (unsigned) nWritten, (unsigned) nLen,
		      Result == FR_OK ? ": the card is full" : "");
		return Result::Failed;
	}

	return Result::Ok;
}

int32_t CBootCard::read (uint8_t *pBuffer, uint32_t nLen)
{
	if (!m_bKernelOpen || m_bKernelWriting)
	{
		return -1;
	}
	UINT nRead = 0;
	FRESULT Result = f_read (&m_KernelFile, pBuffer, (UINT) nLen, &nRead);
	if (Result != FR_OK)
	{
		logf (*m_pLog, LogLevel::Warn, "SD: reading %s failed (FatFs error %u)", m_OpenName, (unsigned) Result);
		return -1;
	}

	return (int32_t) nRead;
}

KernelStore::Result CBootCard::close (void)
{
	if (!m_bKernelOpen)
	{
		return Result::Failed;
	}
	m_bKernelOpen = FALSE;
	FRESULT Result = f_close (&m_KernelFile);	// after writing: the cached sector, the FAT, the directory entry
	if (Result != FR_OK)
	{
		return Failed (m_pLog, "cannot close", m_OpenName, Result);
	}

	return Result::Ok;
}

KernelStore::Result CBootCard::remove (const char *pName)
{
	if (!Mount ())
	{
		return Result::Failed;
	}
	char Name[32];
	Path (Name, sizeof Name, pName);
	FRESULT Result = f_unlink (Name);
	if (Result == FR_NO_FILE)
	{
		return Result::Missing;
	}
	if (Result != FR_OK)
	{
		return Failed (m_pLog, "cannot remove", pName, Result);
	}

	return Result::Ok;
}

KernelStore::Result CBootCard::rename (const char *pFrom, const char *pTo)
{
	if (!Mount ())
	{
		return Result::Failed;
	}
	char From[32], To[32];
	Path (From, sizeof From, pFrom);
	// f_rename's new name has no drive: it stays on the old one's volume.
	format (To, sizeof To, "%s", pTo);
	FRESULT Result = f_rename (From, To);
	if (Result == FR_NO_FILE)
	{
		return Result::Missing;
	}
	if (Result != FR_OK)
	{
		return Failed (m_pLog, "cannot rename", pFrom, Result);
	}

	return Result::Ok;
}

KernelStore::Result CBootCard::sync (void)
{
	return m_bMounted ? Result::Ok : Result::Failed;
}

// ---- the flight recorder's card (kilo::RecordCard) -------------------------------------
//
// FatFs: f_write puts the data on the card (multi-sector when it can);
// f_sync writes the FAT and the directory entry (the size), so a power cut
// after it keeps the file to that point; f_rename and f_unlink update the
// directory and sync the volume before they return.

CRecordCard::CRecordCard (CBootCard *pCard)
:	m_pCard (pCard),
	m_bOpen (FALSE),
	m_bListing (FALSE)
{
	memset (&m_File, 0, sizeof m_File);
	memset (&m_Dir, 0, sizeof m_Dir);
}

bool CRecordCard::ready (void)
{
	return m_pCard->Mount ();
}

static RecordCard::Result Done (FRESULT Result)
{
	if (Result == FR_OK)
	{
		return RecordCard::Result::Ok;
	}
	return Result == FR_NO_FILE || Result == FR_NO_PATH ? RecordCard::Result::Missing : RecordCard::Result::Failed;
}

RecordCard::Result CRecordCard::make_dir (const char *pPath)
{
	char Name[80];
	m_pCard->Path (Name, sizeof Name, pPath);
	FRESULT Result = f_mkdir (Name);
	return Result == FR_EXIST ? Result::Ok : Done (Result);
}

RecordCard::Result CRecordCard::Open (const char *pPath, BYTE uchMode)
{
	if (m_bOpen || !m_pCard->Mount ())
	{
		return Result::Failed;
	}
	char Name[80];
	m_pCard->Path (Name, sizeof Name, pPath);
	FRESULT Result = f_open (&m_File, Name, uchMode);
	if (Result != FR_OK)
	{
		return Done (Result);
	}
	m_bOpen = TRUE;
	return Result::Ok;
}

RecordCard::Result CRecordCard::create (const char *pPath)	{ return Open (pPath, FA_WRITE | FA_CREATE_ALWAYS); }
RecordCard::Result CRecordCard::append (const char *pPath)	{ return Open (pPath, FA_WRITE | FA_OPEN_APPEND); }
RecordCard::Result CRecordCard::open (const char *pPath)	{ return Open (pPath, FA_READ | FA_OPEN_EXISTING); }

RecordCard::Result CRecordCard::seek (uint32_t nOffset)
{
	return m_bOpen ? Done (f_lseek (&m_File, (FSIZE_t) nOffset)) : Result::Failed;
}

RecordCard::Result CRecordCard::write (const uint8_t *pData, uint32_t nLen)
{
	if (!m_bOpen)
	{
		return Result::Failed;
	}
	UINT nWritten = 0;
	FRESULT Result = f_write (&m_File, pData, (UINT) nLen, &nWritten);
	return Result == FR_OK && nWritten == nLen ? Result::Ok : Result::Failed;	// short: the card is full
}

int32_t CRecordCard::read (uint8_t *pBuffer, uint32_t nLen)
{
	if (!m_bOpen)
	{
		return -1;
	}
	UINT nRead = 0;
	return f_read (&m_File, pBuffer, (UINT) nLen, &nRead) == FR_OK ? (int32_t) nRead : -1;
}

RecordCard::Result CRecordCard::sync (void)
{
	return m_bOpen ? Done (f_sync (&m_File)) : Result::Failed;
}

RecordCard::Result CRecordCard::close (void)
{
	if (!m_bOpen)
	{
		return Result::Failed;
	}
	m_bOpen = FALSE;
	return Done (f_close (&m_File));
}

RecordCard::Result CRecordCard::remove (const char *pPath)
{
	if (!m_pCard->Mount ())
	{
		return Result::Failed;
	}
	char Name[80];
	m_pCard->Path (Name, sizeof Name, pPath);
	return Done (f_unlink (Name));
}

RecordCard::Result CRecordCard::rename (const char *pFrom, const char *pTo)
{
	if (!m_pCard->Mount ())
	{
		return Result::Failed;
	}
	char From[80];
	m_pCard->Path (From, sizeof From, pFrom);
	// f_rename's new name has no drive: it stays on the old one's volume.
	return Done (f_rename (From, pTo));
}

RecordCard::Result CRecordCard::list_begin (const char *pDir)
{
	if (!m_pCard->Mount ())
	{
		return Result::Failed;
	}
	if (m_bListing)
	{
		f_closedir (&m_Dir);
		m_bListing = FALSE;
	}
	char Name[80];
	m_pCard->Path (Name, sizeof Name, pDir);
	FRESULT Result = f_opendir (&m_Dir, Name);
	m_bListing = Result == FR_OK;
	return Done (Result);
}

RecordCard::Result CRecordCard::list_next (char *pName, uint32_t nSize, uint32_t *pSize)
{
	if (!m_bListing)
	{
		return Result::Missing;
	}
	FILINFO Info;
	FRESULT Result = f_readdir (&m_Dir, &Info);
	if (Result != FR_OK || Info.fname[0] == '\0')
	{
		f_closedir (&m_Dir);
		m_bListing = FALSE;
		return Result == FR_OK ? Result::Missing : Result::Failed;
	}
	format (pName, nSize, "%s", Info.fname);
	*pSize = (uint32_t) Info.fsize;
	return Result::Ok;
}
