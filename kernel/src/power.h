//
// power.h: light sleep's hardware on the Pi 2 B (mind/power/sleep.h,
// docs/power.md): the Ethernet PHY's power-down and the SoC's clock,
// temperature and throttling.
//
// The Ethernet: the LAN9514 (USB hub and 10/100 Ethernet in one chip) has
// a PHY like SMSC's LAN83C185, whose register 17 (Mode Control/Status) has
// EDPWRDOWN (bit 13): the energy-detect power-down. With it on and no
// signal on the cable the PHY powers down all but its energy detector and
// wakes by itself when a signal comes (then autonegotiates). Circle's
// CSMSC951xDevice keeps its PHY access private, so this does the same
// thing through the device's control endpoint: the LAN9514's vendor
// requests (0xA0 write register, 0xA1 read register) on MII_ADDR (0x114)
// and MII_DATA (0x118), PHY 1, as lib/usb/smsc951x.cpp does. Circle's own
// PHY task reads registers 1 and 31 every 2 s; both use blocking control
// transfers that spin (no yield: NO_BUSY_WAIT is off), so the two never
// interleave on the cooperative scheduler.
//
// What is not possible here (docs/power.md "What light sleep cannot
// switch off"): the LAN9514 as a whole (USB suspend: Circle cannot resume
// a suspended device, and this kernel runs without USB plug-and-play), the
// USB host controller's power domain (the LAN9514 would not come back
// without a new USB scan), HDMI / composite (the firmware's TV service has
// no property-mailbox call), and a CPU clock below the firmware's minimum:
// Circle without CCPUThrottle already runs the ARM at it (600 MHz).
//
#ifndef KILO_SRC_POWER_H
#define KILO_SRC_POWER_H
#include "power/sleep.h"	// the core's seam (mind/power/sleep.h)
#include "platform.h"
#include <circle/usb/usbfunction.h>
#include <circle/types.h>

class CKiloPower final : public kilo::power::Hardware
{
public:
	CKiloPower (kilo::Log *pLog);

	// kilo::power::Hardware
	bool net_device (void) override;
	bool net_link (void) override;
	bool net_save (bool bOn) override;
	bool soc (kilo::power::Soc *pOut) override;

private:
	CUSBFunction *Lan (void);	// the LAN9514's Ethernet function, or 0
	boolean ReadReg (CUSBFunction *pLan, u32 nIndex, u32 *pValue);
	boolean WriteReg (CUSBFunction *pLan, u32 nIndex, u32 nValue);
	boolean PhyWait (CUSBFunction *pLan);
	boolean PhyRead (CUSBFunction *pLan, u8 uchReg, u16 *pValue);
	boolean PhyWrite (CUSBFunction *pLan, u8 uchReg, u16 usValue);

	kilo::Log *m_pLog;
	boolean m_bModelOK;		// a Pi 2 B (or B+): the Ethernet is a LAN9514
	boolean m_bLogged;
};

#endif
