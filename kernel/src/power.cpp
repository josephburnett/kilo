//
// power.cpp: light sleep's hardware on the Pi 2 B (power.h).
//
#include "power.h"
#include "util/log.h"
#include <circle/bcmpropertytags.h>
#include <circle/machineinfo.h>
#include <circle/netdevice.h>
#include <circle/timer.h>
#include <circle/usb/smsc951x.h>
#include <circle/usb/usb.h>
#include <circle/usb/usbhostcontroller.h>

// The LAN9514's registers and vendor requests (lib/usb/smsc951x.cpp).
static const u8 RequestWriteRegister = 0xA0;
static const u8 RequestReadRegister = 0xA1;
static const u32 RegMiiAddr = 0x114;
static const u32 MiiBusy = 0x01;
static const u32 MiiWrite = 0x02;
static const u32 PhyInternal = 0x01;
static const u32 RegMiiData = 0x118;

// The PHY's Mode Control/Status register and its energy-detect power-down
// (SMSC LAN83C185 / LAN8700, the LAN9514's PHY; Linux drivers/net/phy/smsc.c
// MII_LAN83C185_CTRL_STATUS, MII_LAN83C185_EDPWRDOWN).
static const u8 PhyModeControl = 17;
static const u16 PhyEdPowerDown = 1u << 13;

// PhyWait: the MII busy bit, at most this long.
static const unsigned MiiWaitUs = 20000;

CKiloPower::CKiloPower (kilo::Log *pLog)
:	m_pLog (pLog),
	m_bModelOK (FALSE),
	m_bLogged (FALSE)
{
	TMachineModel Model = CMachineInfo::Get ()->GetMachineModel ();
	m_bModelOK = Model == MachineModel2B || Model == MachineModelBPlus;
}

CUSBFunction *CKiloPower::Lan (void)
{
	if (!m_bModelOK)
	{
		return 0;
	}

	CNetDevice *pNetDevice = CNetDevice::GetNetDevice (NetDeviceTypeEthernet);
	if (pNetDevice == 0)
	{
		return 0;
	}

	// On a Pi 2 B the one Ethernet device is the LAN9514's. CNetDevice is
	// a private base of CSMSC951xDevice; the C-style cast is the one cast
	// that may convert through it (it adjusts the pointer as static_cast
	// would), and CUSBFunction is its public base.
	CSMSC951xDevice *pSmsc = (CSMSC951xDevice *) pNetDevice;

	return pSmsc;
}

bool CKiloPower::net_device (void)
{
	return Lan () != 0;
}

bool CKiloPower::net_link (void)
{
	CNetDevice *pNetDevice = CNetDevice::GetNetDevice (NetDeviceTypeEthernet);

	return pNetDevice != 0 && pNetDevice->IsLinkUp ();
}

boolean CKiloPower::ReadReg (CUSBFunction *pLan, u32 nIndex, u32 *pValue)
{
	return pLan->GetHost ()->ControlMessage (pLan->GetEndpoint0 (), REQUEST_IN | REQUEST_VENDOR,
						 RequestReadRegister, 0, (u16) nIndex, pValue,
						 sizeof *pValue) == (int) sizeof *pValue;
}

boolean CKiloPower::WriteReg (CUSBFunction *pLan, u32 nIndex, u32 nValue)
{
	return pLan->GetHost ()->ControlMessage (pLan->GetEndpoint0 (), REQUEST_OUT | REQUEST_VENDOR,
						 RequestWriteRegister, 0, (u16) nIndex, &nValue,
						 sizeof nValue) >= 0;
}

boolean CKiloPower::PhyWait (CUSBFunction *pLan)
{
	unsigned nStart = CTimer::GetClockTicks ();
	u32 nValue;
	do
	{
		if (CTimer::GetClockTicks () - nStart >= MiiWaitUs || !ReadReg (pLan, RegMiiAddr, &nValue))
		{
			return FALSE;
		}
	}
	while (nValue & MiiBusy);

	return TRUE;
}

boolean CKiloPower::PhyRead (CUSBFunction *pLan, u8 uchReg, u16 *pValue)
{
	u32 nAddr = (PhyInternal << 11) | ((u32) uchReg << 6);
	u32 nValue;
	if (   !PhyWait (pLan)
	    || !WriteReg (pLan, RegMiiAddr, nAddr | MiiBusy)
	    || !PhyWait (pLan)
	    || !ReadReg (pLan, RegMiiData, &nValue))
	{
		return FALSE;
	}
	*pValue = (u16) (nValue & 0xFFFF);

	return TRUE;
}

boolean CKiloPower::PhyWrite (CUSBFunction *pLan, u8 uchReg, u16 usValue)
{
	u32 nAddr = (PhyInternal << 11) | ((u32) uchReg << 6);

	return    PhyWait (pLan)
	       && WriteReg (pLan, RegMiiData, usValue)
	       && WriteReg (pLan, RegMiiAddr, nAddr | MiiWrite | MiiBusy)
	       && PhyWait (pLan);
}

bool CKiloPower::net_save (bool bOn)
{
	CUSBFunction *pLan = Lan ();
	if (pLan == 0)
	{
		return false;
	}

	unsigned nStart = CTimer::GetClockTicks ();
	u16 usMode;
	if (!PhyRead (pLan, PhyModeControl, &usMode))
	{
		return false;
	}
	u16 usNew = bOn ? (u16) (usMode | PhyEdPowerDown) : (u16) (usMode & ~PhyEdPowerDown);
	u16 usCheck = 0;
	if (   (usNew != usMode && !PhyWrite (pLan, PhyModeControl, usNew))
	    || !PhyRead (pLan, PhyModeControl, &usCheck)
	    || (usCheck & PhyEdPowerDown) != (usNew & PhyEdPowerDown))
	{
		return false;
	}

	// The first time: what the PHY said and how long the USB transfers took
	// (the loop waited for them).
	if (!m_bLogged)
	{
		m_bLogged = TRUE;
		kilo::logf (*m_pLog, kilo::LogLevel::Info, "power: Ethernet PHY register 17 %04x -> %04x (%u us)",
			    (unsigned) usMode, (unsigned) usCheck, CTimer::GetClockTicks () - nStart);
	}

	return true;
}

bool CKiloPower::soc (kilo::power::Soc *pOut)
{
	CBcmPropertyTags Tags;
	TPropertyTagClockRate Clock;
	Clock.nClockId = CLOCK_ID_ARM;
	if (!Tags.GetTag (PROPTAG_GET_CLOCK_RATE, &Clock, sizeof Clock, 4))
	{
		return false;
	}
	pOut->arm_hz = Clock.nRate;

	TPropertyTagTemperature Temperature;
	Temperature.nTemperatureId = TEMPERATURE_ID;
	pOut->temp_mc = Tags.GetTag (PROPTAG_GET_TEMPERATURE, &Temperature, sizeof Temperature, 4)
			? (int32_t) Temperature.nValue : 0;

	// Bit 0 under-voltage now, 16 since boot; 1/17 frequency capped; 2/18
	// throttled (as vcgencmd get_throttled). Older firmware: not answered.
	TPropertyTagSimple Throttled;
	Throttled.nValue = 0xFFFF;
	pOut->throttled = Tags.GetTag (PROPTAG_GET_THROTTLED, &Throttled, sizeof Throttled, 4)
			  ? Throttled.nValue : 0;

	return true;
}
