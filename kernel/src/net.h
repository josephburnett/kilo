//
// net.h: the laptop interface's network on Circle (docs/fast-boot.md §5,
// phase P2): Ethernet bring-up in a background task, and the core's network
// seams (mind/platform_net.h) served from per-connection rings.
//
// The control loop never waits for the network (docs/fast-boot.md §3 rule
// 1). CKernel starts CKiloNet last, after the body link; it first runs when
// the main loop sleeps (CScheduler::usSleep) and then shares the CPU with it
// cooperatively. Every call it makes into Circle's TCP/IP stack is
// non-blocking (see net.cpp), so it hands the CPU back within one pass.
//
//   main loop (1 ms)                       CKiloNet task (1 ms)
//   HostServer::poll()                     Ethernet up, DHCP or the fallback
//     CircleListener::accept()  <- slot <-   CSocket::Accept (only when ready)
//     CircleConn::read()        <- RX ring <-  CSocket::Receive (MSG_DONTWAIT)
//     CircleConn::write()       -> TX ring ->  CSocket::Send (MSG_DONTWAIT, when TX ready)
//     CircleConn::close()       -> closing ->  delete the CSocket, slot free
//
// The two sides share the slots without locks: one core, a cooperative
// scheduler, and neither side yields in the middle of a ring update.
//
#ifndef KILO_SRC_NET_H
#define KILO_SRC_NET_H

#include "platform_net.h"	// the core's network seams (mind/platform_net.h)
#include "proto/kilo_protocol.h"
#include "seams.h"
#include <circle/net/netsubsystem.h>
#include <circle/net/socket.h>
#include <circle/netdevice.h>
#include <circle/sched/task.h>
#include <circle/types.h>
#include <circle/usb/usbhcidevice.h>

// Announce kilo.local and the laptop service over mDNS (CKiloMdns below).
// make -C kernel KILO_MDNS=0 leaves it out.
#ifndef KILO_MDNS
#define KILO_MDNS 1
#endif

// A byte ring of fixed size, one producer and one consumer.
template <unsigned N>
struct CByteRing
{
	u8 m_Data[N];
	unsigned m_nHead;	// first byte queued
	unsigned m_nLen;	// bytes queued

	void Reset (void)			{ m_nHead = 0; m_nLen = 0; }
	unsigned Free (void) const		{ return N - m_nLen; }
	// Copy in as much as fits: the count taken.
	unsigned Put (const u8 *pData, unsigned nCount)
	{
		unsigned n = nCount < Free () ? nCount : Free ();
		for (unsigned i = 0; i < n; i++)
		{
			m_Data[(m_nHead + m_nLen + i) % N] = pData[i];
		}
		m_nLen += n;
		return n;
	}

	// Copy out up to nCount bytes: the count.
	unsigned Get (u8 *pData, unsigned nCount)
	{
		unsigned n = nCount < m_nLen ? nCount : m_nLen;
		for (unsigned i = 0; i < n; i++)
		{
			pData[i] = m_Data[(m_nHead + i) % N];
		}
		Drop (n);
		return n;
	}

	// The queued bytes up to the end of the buffer, for Send() without a copy.
	// Put() meanwhile writes only past them, so the pointer stays valid.
	const u8 *Contiguous (unsigned *pCount) const
	{
		unsigned nToEnd = N - m_nHead;
		*pCount = m_nLen < nToEnd ? m_nLen : nToEnd;
		return &m_Data[m_nHead];
	}

	void Drop (unsigned nCount)
	{
		m_nHead = (m_nHead + nCount) % N;
		m_nLen -= nCount;
	}
};

// One connection slot. The main loop sees it as a kilo::Conn; the net task
// moves its bytes.
class CircleConn final : public kilo::Conn
{
public:
	// Ring sizes (kernel/README.md, "The net task"). RX: Circle's Receive()
	// hands over one whole TCP segment (up to FRAME_BUFFER_SIZE) and drops
	// what does not fit the buffer it is given, so the task receives only
	// when a full segment fits: 2 segments and a margin. The laptop sends
	// requests of at most KILO_HOST_MAX_FRAME bytes; HostServer reads up to
	// 1 KB a pass. TX: HostServer keeps its own 4 KB ring per client and
	// flushes what this one takes; the task empties this one every ~1 ms into
	// Circle's TCP queue (up to 64 KB more before Circle stops taking).
	// Streams are about 10 kB/s at most (TELEM at 50 Hz and log lines), so
	// 4 KB is 400 ms of output.
	static const unsigned RxBytes = 4096;
	static const unsigned TxBytes = 4096;

	enum TState
	{
		StateFree,	// no socket
		StatePending,	// accepted by the task, not yet by HostServer
		StateOpen,	// HostServer's
		StateClosing	// HostServer closed it: the task deletes the socket
	};

	CircleConn (void);

	// kilo::Conn, for the main loop
	int32_t read (uint8_t *pBuffer, uint32_t nCount) override;
	int32_t write (const uint8_t *pBuffer, uint32_t nCount) override;
	void close (void) override;
	bool is_open (void) const override	{ return m_State == StateOpen; }

private:
	friend class CKiloNet;
	friend class CircleListener;

	TState m_State;
	CSocket *m_pSocket;		// owned by the net task
	boolean m_bFailed;		// the peer closed or the connection failed
	u8 m_ForeignIP[4];
	u16 m_nForeignPort;
	CByteRing<RxBytes> m_Rx;	// socket -> HostServer
	CByteRing<TxBytes> m_Tx;	// HostServer -> socket
};

// The listening side for HostServer: KILO_HOST_MAX_CLIENTS slots.
class CircleListener final : public kilo::Listener
{
public:
	static const unsigned Slots = KILO_HOST_MAX_CLIENTS;

	int32_t accept (void) override;
	kilo::Conn *conn (uint32_t nSlot) override	{ return &m_Conn[nSlot]; }
	uint32_t slots (void) const override		{ return Slots; }

private:
	friend class CKiloNet;

	CircleConn m_Conn[Slots];
};

// The net task: Ethernet bring-up, the address, the listening socket, and
// the bytes between the slots' rings and their sockets.
class CKiloNet : public CTask
{
public:
	// pUSB: the USB host to initialise first (its scan finds the LAN9514), or
	// 0 when the kernel already did (the USB serial scan). pGoAhead: the scan
	// waits for it (the main loop sets it once the body's first HELLO and
	// the boot-time flash from SD are over) or UsbWaitUs, so that the scan,
	// which stalls the main loop, neither delays the HELLO nor cuts into a
	// flash (optiboot gives up after 1 s of silence). pLog: the kernel's
	// log. nBuild: the mind's build, in the mDNS TXT record.
	CKiloNet (CNetSubSystem *pNet, CUSBHCIDevice *pUSB, const boolean *pGoAhead,
		  kilo::Log *pLog, CircleListener *pListener, u32 nBuild);

	void Run (void) override;

	// For the stats line, read by the main loop.
	boolean HasAddress (void) const		{ return m_bListening; }
	const u8 *GetAddress (void) const	{ return m_IP; }
	boolean IsFallback (void) const		{ return m_bFallback; }
	unsigned GetRefused (void) const	{ return m_nRefused; }

	// How long the USB scan waits for the main loop's go-ahead: the longest
	// boot is no HELLO for 3 s, the SD card, and a flash that fails twice or
	// succeeds and waits up to 8 s for the new firmware's HELLO.
	static const unsigned UsbWaitUs = 30000000;

private:
	void BringUp (void);
	void WatchAddress (u64 nNowUs);
	void OpenListener (void);
	void AcceptAll (void);
	void Pump (CircleConn *pConn);

private:
	CNetSubSystem *m_pNet;
	CUSBHCIDevice *m_pUSB;
	const boolean *m_pGoAhead;
	kilo::Log *m_pLog;
	CircleListener *m_pListener;
	CircleClock m_Clock;

	CSocket *m_pListen;		// on m_IP, KILO_HOST_PORT
	boolean m_bListening;
	u8 m_IP[4];			// the address m_pListen was made for
	boolean m_bLinkUp;
	u64 m_nLinkUpUs;		// when the Ethernet link came up (0: not yet)
	boolean m_bFallback;		// the link-local address is in use
	boolean m_bNoLinkLogged;
	u64 m_nStartUs;
	unsigned m_nRefused;		// connections closed because every slot was busy
	u32 m_nBuild;
	CTask *m_pMdns;			// started with the first address (KILO_MDNS)

	u8 m_RxBuffer[FRAME_BUFFER_SIZE];	// one TCP segment
};

// mDNS (KILO_MDNS=1, the default): kilo.local answers with the kernel's
// address, and the laptop service is announced as "kilo" of type _kilo._tcp
// on KILO_HOST_PORT, so the owner needs no lease table. Circle's responder
// (CmDNSDaemon) answers A queries for the DHCP hostname ("kilo"; "kilo-2"
// if another host answers for it at start) with the address of the moment;
// its publisher (CmDNSPublisher) announces the service unsolicited 3 times
// at start and every 110 s, and does not answer service queries itself.
// This task starts both once the net task has an address (the responder
// probes the name from it, about 1 s of sleeping), and announces again when
// the address changes (the link-local fallback, then a lease).
class CKiloMdns : public CTask
{
public:
	CKiloMdns (CNetSubSystem *pNet, kilo::Log *pLog, u32 nBuild);

	void Run (void) override;

	static const unsigned PeriodMs = 500;	// how often the address is checked

private:
	CNetSubSystem *m_pNet;
	kilo::Log *m_pLog;
	char m_Text[24];			// TXT "build=xxxxxxxx"
};

#endif
