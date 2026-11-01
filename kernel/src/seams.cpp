//
// seams.cpp: the mind core's seams on Circle (see seams.h).
//
#include "seams.h"
#include <circle/timer.h>
#include <assert.h>

// ---- PL011 ------------------------------------------------------------------

Pl011Serial::Pl011Serial (CInterruptSystem *pInterrupt)
:	m_nLineErrors {0, 0, 0, 0, 0},
	m_Device (pInterrupt),
	m_nBaud (0)
{
}

boolean Pl011Serial::Initialize (unsigned nBaud)
{
	// Circle's divisor: UARTCLK / (16 * baud) = 48 MHz / 4 MHz = 12, remainder
	// 0, so IBRD 12, FBRD 0: exactly 250000 (protocol/spec.toml). UARTCLK is
	// the firmware's measured rate rounded to 1 MHz (CMachineInfo).
	if (!m_Device.Initialize (nBaud, 8, 1, CSerialDevice::ParityNone))
	{
		return FALSE;
	}

	// The default translates NL to CR NL on output: in a binary frame that
	// inserts bytes and breaks the CRC.
	m_Device.SetOptions (0);
	m_nBaud = nBaud;

	return TRUE;
}

int32_t Pl011Serial::read (uint8_t *pBuffer, uint32_t nCount)
{
	// A negative result is a line error (break, overrun, framing, parity),
	// reported once and cleared, with the received bytes still in the ring:
	// counted, then read again. The link sees the damage as a bad frame.
	for (unsigned i = 0; i < 2; i++)
	{
		int nResult = m_Device.Read (pBuffer, nCount);
		if (nResult >= 0)
		{
			return nResult;
		}

		unsigned nError = (unsigned) -nResult;
		m_nLineErrors[nError < 5 ? nError : 0]++;
	}

	return 0;
}

int32_t Pl011Serial::write (const uint8_t *pBuffer, uint32_t nCount)
{
	// Into the 2 KiB TX ring; the interrupt handler feeds the FIFO. Never
	// blocks: what does not fit is not taken (the link checks tx_free()
	// first, so that does not happen).
	return m_Device.Write (pBuffer, nCount);
}

uint32_t Pl011Serial::tx_free (void)
{
	return m_Device.TxFree ();
}

bool Pl011Serial::set_baud (uint32_t nBaud)
{
	return nBaud == m_nBaud;
}

// ---- USB serial -------------------------------------------------------------

UsbSerial::UsbSerial (CUSBSerialDevice *pDevice)
:	m_pDevice (pDevice)
{
	assert (m_pDevice != 0);
	m_pDevice->SetOptions (0);		// no NL -> CR NL
}

int32_t UsbSerial::read (uint8_t *pBuffer, uint32_t nCount)
{
	// 0 while the IN request is in flight; -1 only when it cannot be queued.
	int nResult = m_pDevice->Read (pBuffer, nCount);

	return nResult < 0 ? -1 : nResult;
}

int32_t UsbSerial::write (const uint8_t *pBuffer, uint32_t nCount)
{
	// Blocks until the bulk OUT transfer completes: a frame is <= 66 bytes,
	// two full-speed packets. The adapter NAKs while its own buffer is full,
	// and Circle retries without a timeout, so a wedged adapter would stall
	// here (not seen; the offered load is a fifth of the line rate).
	int nResult = m_pDevice->Write (pBuffer, nCount);

	return nResult < 0 ? -1 : nResult;
}

uint32_t UsbSerial::tx_free (void)
{
	// Write() takes a whole frame or fails; nothing queues on this side.
	return 512;
}

bool UsbSerial::set_baud (uint32_t nBaud)
{
	return    m_pDevice->SetBaudRate (nBaud)
	       && m_pDevice->SetLineProperties (USBSerialDataBits8, USBSerialParityNone,
						USBSerialStopBits1);
}

// ---- clock, log, GPIO4 ------------------------------------------------------

uint64_t CircleClock::now_us (void)
{
	// CLOCKHZ is 1 MHz. On the Pi 2 Circle reads the ARM physical counter,
	// which the firmware's prescaler runs at 1 MHz; one mrrc reads all
	// 64 bits, so there is no 32-bit wrap (71 minutes) to handle here.
	return CTimer::GetClockTicks64 ();
}

void CircleLog::write (kilo::LogLevel Level, const char *pLine)
{
	if (Level < m_Min)
	{
		return;
	}

	// The RAM log ring for the laptop's log tail (docs/fast-boot.md §5, P2)
	// takes the line here, before CLogger.

	static const TLogSeverity Severity[] = {LogDebug, LogNotice, LogWarning, LogError};

	// No CString, no heap: the line is already formatted.
	m_pLogger->WriteNoAlloc ("kilo", Severity[(unsigned) Level & 3], pLine);
}

CircleResetGpio::CircleResetGpio (void)
:	m_Pin (4, GPIOModeInput)		// released: input, pull off
{
}

void CircleResetGpio::reset_assert (void)
{
	// Latch low first, then switch to output: the pin never drives the
	// latch's old level, which may be high.
	m_Pin.Write (LOW);
	m_Pin.SetMode (GPIOModeOutput, FALSE);
}

void CircleResetGpio::reset_release (void)
{
	m_Pin.SetMode (GPIOModeInput, TRUE);	// TRUE: pull off
}
