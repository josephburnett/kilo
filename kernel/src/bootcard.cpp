//
// bootcard.cpp: body.bin and body.json on the SD card (see bootcard.h).
//
#include "bootcard.h"
#include "flash/flasher.h"
#include "util/fmt.h"
#include "util/log.h"
#include <circle/util.h>

using namespace kilo;

// FatFs's volume string for the SD card (addon/fatfs/ffconf.h FF_VOLUME_STRS).
static const char Drive[] = "SD:";

// body.json is one short line (about 100 bytes with "commit").
static const unsigned ManifestMax = 1024;

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
	m_bMounted (FALSE)
{
	memset (&m_FileSystem, 0, sizeof m_FileSystem);
	memset (&m_File, 0, sizeof m_File);
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
		      "no --persist", Elapsed (nStart));
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

boolean CBootCard::LoadBodyImage (FlashImage &rImage)
{
	if (!Mount ())
	{
		return FALSE;
	}

	unsigned nStart = CTimer::GetClockTicks ();

	char Text[ManifestMax + 1];
	int nLen = ReadFile (kManifestFile, Text, ManifestMax);
	if (nLen == ReadMissing)
	{
		logf (*m_pLog, LogLevel::Info, "SD: no %s: nothing to flash at boot", kManifestFile);
		return FALSE;
	}
	if (nLen < 0)
	{
		logf (*m_pLog, LogLevel::Info, "SD: %s %s: not flashing at boot", kManifestFile,
		      nLen == ReadTooBig ? "is too big for a manifest" : "cannot be read");
		return FALSE;
	}
	Text[nLen] = '\0';

	Manifest m;
	const char *pWhy = parse_manifest (Text, (uint32_t) nLen, m);
	if (pWhy != 0)
	{
		logf (*m_pLog, LogLevel::Info, "SD: %s is not a manifest (%s): not flashing at boot", kManifestFile, pWhy);
		return FALSE;
	}

	// Read into a heap buffer, then load_from() copies and checks it.
	u8 *pData = new u8[FlashImage::kMaxSize];
	int nSize = ReadFile (kImageFile, pData, FlashImage::kMaxSize);
	if (nSize < 0)
	{
		delete [] pData;
		logf (*m_pLog, LogLevel::Info, "SD: %s for build %08x %s: not flashing at boot", kImageFile,
		      (unsigned) m.build,
		      nSize == ReadMissing ? "is missing" : nSize == ReadTooBig ? "is over 32256 bytes" : "cannot be read");
		return FALSE;
	}

	uint8_t nError = rImage.load_from (m, pData, (uint32_t) nSize);
	delete [] pData;
	if (nError != KILO_FLASH_ERROR_NONE)
	{
		logf (*m_pLog, LogLevel::Info, "SD: %s (%u bytes) does not match %s (build %08x, %u bytes, crc32 %08x): "
		      "%s; not flashing at boot", kImageFile, (unsigned) nSize, kManifestFile, (unsigned) m.build,
		      (unsigned) m.size, (unsigned) m.crc32, flash_error_name (nError));
		return FALSE;
	}

	logf (*m_pLog, LogLevel::Info, "SD: body image build %08x, %u bytes, crc32 %08x good (read in %u ms)",
	      (unsigned) m.build, (unsigned) m.size, (unsigned) m.crc32, Elapsed (nStart));

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

int CBootCard::ReadFile (const char *pName, void *pBuffer, unsigned nSize)
{
	char Name[32];
	Path (Name, sizeof Name, pName);

	FRESULT Result = f_open (&m_File, Name, FA_READ | FA_OPEN_EXISTING);
	if (Result == FR_NO_FILE || Result == FR_NO_PATH)
	{
		return ReadMissing;
	}
	if (Result != FR_OK)
	{
		logf (*m_pLog, LogLevel::Warn, "SD: cannot open %s (FatFs error %u)", pName, (unsigned) Result);
		return ReadError;
	}

	int nResult;
	FSIZE_t nFileSize = f_size (&m_File);
	if (nFileSize > nSize)
	{
		nResult = ReadTooBig;
	}
	else
	{
		UINT nRead = 0;
		Result = f_read (&m_File, pBuffer, (UINT) nFileSize, &nRead);
		if (Result != FR_OK || nRead != nFileSize)
		{
			logf (*m_pLog, LogLevel::Warn, "SD: cannot read %s (FatFs error %u, %u of %u bytes)", pName,
			      (unsigned) Result, (unsigned) nRead, (unsigned) nFileSize);
			nResult = ReadError;
		}
		else
		{
			nResult = (int) nRead;
		}
	}

	f_close (&m_File);

	return nResult;
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
