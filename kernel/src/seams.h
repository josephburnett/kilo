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
	// Only the rate given to Initialize(): CSerialDevice cannot be
	// initialised twice. Switching to 115200 for flashing is P4's.
	bool set_baud (uint32_t nBaud) override;

	// Receive errors reported by the driver, per SERIAL_ERROR_* (1..4:
	// break, overrun, framing, parity). Overrun also counts a full RX ring.
	unsigned m_nLineErrors[5];

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
