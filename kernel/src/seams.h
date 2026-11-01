//
// seams.h: the mind core's seams (mind/platform.h) on Circle.
//
// Two Serial back ends for the body link, chosen in CKernel::Initialize
// (see kernel/README.md, "Body link"):
//   Pl011Serial  the SoC UART on GPIO14/15 (ttyS1), interrupt driven
//   UsbSerial    a USB serial adapter (utty1): the CH343 on Kilo's cable,
//                CDC-ACM, driven by Circle's CUSBSerialCDCDevice
//
#ifndef KILO_SRC_SEAMS_H
#define KILO_SRC_SEAMS_H

#include "platform.h"		// the core's seams (mind/platform.h, -I mind)
#include <circle/gpiopin.h>
#include <circle/interrupt.h>
#include <circle/logger.h>
#include <circle/serial.h>
#include <circle/types.h>
#include <circle/usb/usbserial.h>

// The PL011. Circle's CSerialDevice, with the free space of its TX ring made
// visible (AvailableForWrite() is protected).
class Pl011Serial final : public kilo::Serial
{
public:
	// Constructing it muxes GPIO14/15 to the UART, so it is built only when
	// chosen (CKernel news it): with the USB back end the header pins stay
	// untouched.
	Pl011Serial (CInterruptSystem *pInterrupt);

	// 8N1 at nBaud, interrupt driven, no NL -> CR NL translation.
	boolean Initialize (unsigned nBaud);

	int32_t read (uint8_t *pBuffer, uint32_t nCount) override;
	int32_t write (const uint8_t *pBuffer, uint32_t nCount) override;
	uint32_t tx_free (void) override;
	// A new line rate (the flasher's 115200 and back to 250000): waits up to
	// DrainUs for the TX ring and the shift register to empty, then
	// reprograms the divisor in place (seams.cpp). The interrupt, the GPIO
	// pins and both rings stay as they are.
	bool set_baud (uint32_t nBaud) override;

	// The longest set_baud() waits for queued bytes to go out: 2 KB at
	// 250000 baud is 82 ms.
	static const unsigned DrainUs = 100000;

	// Receive errors reported by the driver, per SERIAL_ERROR_* (1..4:
	// break, overrun, framing, parity). Overrun also counts a full RX ring.
	unsigned m_nLineErrors[5];
	// set_baud() went ahead after DrainUs with bytes still queued.
	unsigned m_nDrainTimeouts;

private:
	class CPL011 : public CSerialDevice
	{
	public:
		CPL011 (CInterruptSystem *pInterrupt) : CSerialDevice (pInterrupt) {}
		unsigned TxFree (void) { return AvailableForWrite (); }
	};


	CPL011 m_Device;
	unsigned m_nBaud;
};

// A USB serial device found by name (utty1). Circle's USB serial driver
// writes synchronously (a bulk transfer that returns when the adapter has
// taken the bytes) and reads one USB packet at a time (an IN request is
// queued when its buffer is empty; Read() returns 0 until it completes).
class UsbSerial final : public kilo::Serial
{
public:
	UsbSerial (CUSBSerialDevice *pDevice);

	int32_t read (uint8_t *pBuffer, uint32_t nCount) override;
	int32_t write (const uint8_t *pBuffer, uint32_t nCount) override;
	// No TX queue to measure: Write() returns once the transfer is done.
	uint32_t tx_free (void) override;
	bool set_baud (uint32_t nBaud) override;	// CDC SET_LINE_CODING, 8N1

private:
	CUSBSerialDevice *m_pDevice;
};

// The system timer: 1 MHz, 64 bits (CTimer::GetClockTicks64), so no wrap.
class CircleClock final : public kilo::Clock
{
public:
	uint64_t now_us (void) override;
};

// The core's log lines into CLogger (the screen if there is one, else a null
// device), at or above a minimum level.
class CircleLog final : public kilo::Log
{
public:
	CircleLog (CLogger *pLogger, kilo::LogLevel Min) : m_pLogger (pLogger), m_Min (Min) {}
	void write (kilo::LogLevel Level, const char *pLine) override;

private:
	CLogger *m_pLogger;
	kilo::LogLevel m_Min;
};

// The kernel's platform Log, behind the Mind's RAM ring (mind.log(): the
// ring takes every core line first, for `kilo log --host`, then hands it
// here). It passes lines on to CircleLog, except while muted:
// CKernel::DrainCircleLog mutes it to copy Circle's own log events into the
// ring alone, since CLogger already printed them.
class KernelLog final : public kilo::Log
{
public:
	KernelLog (kilo::Log *pNext) : m_pNext (pNext), m_bMuted (FALSE) {}
	void write (kilo::LogLevel Level, const char *pLine) override
	{
		if (!m_bMuted)
		{
			m_pNext->write (Level, pLine);
		}
	}
	void Mute (boolean bMuted)	{ m_bMuted = bMuted; }

private:
	kilo::Log *m_pNext;
	boolean m_bMuted;
};

// The body link's Serial for the Mind, which is built before the device is
// chosen (so the lines of the USB scan and the choice are in its ring):
// every call goes to the device set with Set(). Nothing calls it before
// then (the link opens in Mind::start()); if something did, it would read
// nothing and fail its writes.
class SerialSlot final : public kilo::Serial
{
public:
	SerialSlot (void) : m_pSerial (0) {}
	void Set (kilo::Serial *pSerial)	{ m_pSerial = pSerial; }

	int32_t read (uint8_t *pBuffer, uint32_t nCount) override
	{
		return m_pSerial != 0 ? m_pSerial->read (pBuffer, nCount) : 0;
	}
	int32_t write (const uint8_t *pBuffer, uint32_t nCount) override
	{
		return m_pSerial != 0 ? m_pSerial->write (pBuffer, nCount) : -1;
	}
	uint32_t tx_free (void) override
	{
		return m_pSerial != 0 ? m_pSerial->tx_free () : 0;
	}
	bool set_baud (uint32_t nBaud) override
	{
		return m_pSerial != 0 && m_pSerial->set_baud (nBaud);
	}

private:
	kilo::Serial *m_pSerial;
};

// GPIO4, the body's RESET, as an emulated open drain (mind/kilo/tools/gpio.py):
// released = input with no pull (the NyBoard's pull-up holds RESET high),
// asserted = output low. Never driven high.
class CircleResetGpio final : public kilo::Gpio
{
public:
	CircleResetGpio (void);			// released
	void reset_assert (void) override;
	void reset_release (void) override;

private:
	CGPIOPin m_Pin;
};

#endif
