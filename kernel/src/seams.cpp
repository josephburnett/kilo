//
// seams.cpp: the mind core's seams on Circle (see seams.h).
//
#include "seams.h"
#include <circle/bcm2835.h>
#include <circle/bcmpropertytags.h>
#include <circle/machineinfo.h>
#include <circle/memio.h>
#include <circle/synchronize.h>
#include <circle/timer.h>
#include <assert.h>

// PL011 registers (circle/bcm2835.h has the addresses; lib/serial.cpp keeps
// these bits to itself).
#define UART_FR_BUSY		(1 << 3)
#define UART_CR_UARTEN		(1 << 0)

// ---- PL011 ------------------------------------------------------------------

Pl011Serial::Pl011Serial (CInterruptSystem *pInterrupt)
:	m_nLineErrors {0, 0, 0, 0, 0},
	m_nDrainTimeouts (0),
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

// CSerialDevice has no call for a new rate, and its Initialize() must not
// run twice: a second call adds "ttyS1" to the device name service again
// (a heap entry each time) and counts the interrupt user twice, so the
// destructor would never disconnect the IRQ. Destroying and re-creating the
// device is worse: the destructor disconnects the IRQ and turns GPIO14/15
// into inputs (TXD floats: the 328P may see a break or garbage) and the
// rings go with it. So the rate is changed the way CSerialDevice::SetParity()
// changes the line settings: wait until nothing is being sent, disable the
// UART, write the divisor, write LCRH back (the PL011 latches IBRD/FBRD on a
// write to LCRH), enable. Interrupt mask, FIFOs, pins and rings are untouched.
bool Pl011Serial::set_baud (uint32_t nBaud)
{
	if (nBaud == m_nBaud)
	{
		return true;
	}
	if (m_nBaud == 0 || nBaud < 300 || nBaud > 4000000)
	{
		return false;			// not initialised, or out of the PL011's range
	}

	// Circle's divisor (lib/serial.cpp): integer part, and the 6-bit
	// fraction rounded. 115200 from 48 MHz: IBRD 26, FBRD 3, 115176 baud
	// (-0.02 %); 250000: IBRD 12, FBRD 0, exact.
	unsigned nClockRate = CMachineInfo::Get ()->GetClockRate (CLOCK_ID_UART);
	unsigned nBaud16 = nBaud * 16;
	unsigned nIntDiv = nClockRate / nBaud16;
	unsigned nFractDiv2 = (nClockRate % nBaud16) * 8 / nBaud;
	unsigned nFractDiv = nFractDiv2 / 2 + nFractDiv2 % 2;
	if (nIntDiv < 1 || nIntDiv > 0xFFFF || nFractDiv > 0x3F)
	{
		return false;
	}

	// Drain: the TX ring (the interrupt handler empties it into the FIFO)
	// and the shift register. Bounded: the flasher calls this from the main
	// loop. Bytes still queued after DrainUs go out at the new rate.
	unsigned nStart = CTimer::GetClockTicks ();
	while (m_Device.IsTransmitting ())
	{
		if (CTimer::GetClockTicks () - nStart >= DrainUs)
		{
			m_nDrainTimeouts++;
			break;
		}
	}

	// IRQs off for the few register writes, so the TX interrupt handler does
	// not feed a disabled UART. A byte arriving meanwhile may be lost or
	// garbled; the flasher drops what comes in around a switch anyway.
	EnterCritical (IRQ_LEVEL);
	PeripheralEntry ();

	u32 nCR = read32 (ARM_UART0_CR);
	write32 (ARM_UART0_CR, nCR & ~UART_CR_UARTEN);
	for (unsigned i = 0; i < 100000 && (read32 (ARM_UART0_FR) & UART_FR_BUSY); i++)
	{
		// the last character's stop bit (drained above, so at most one)
	}
	write32 (ARM_UART0_IBRD, nIntDiv);
	write32 (ARM_UART0_FBRD, nFractDiv);
	write32 (ARM_UART0_LCRH, read32 (ARM_UART0_LCRH));
	write32 (ARM_UART0_CR, nCR);

	PeripheralExit ();
	LeaveCritical ();

	m_nBaud = nBaud;

	return true;
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

	// The Mind's ring has already taken the line (`kilo log --host`).

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
