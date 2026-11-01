//
// net.cpp: Ethernet bring-up and the network seams on Circle (see net.h).
//
// Circle's sockets block by default. This task never lets them:
//   - Accept() is called only when GetStatus() on the listening socket says a
//     connection is already established, so it returns at once;
//   - Receive() and Send() take MSG_DONTWAIT; Send() is called only while
//     GetStatus().bTxReady (Circle's TCP queue holds less than 64 KB), since
//     a non-blocking Send() queues without limit (heap) when the peer stops
//     reading;
//   - the address and the link are read from CNetConfig / CNetDeviceLayer.
// The one long call is the USB scan (the default KILO_BODY_SERIAL=pl011
// build; with auto the kernel scanned before the link): Circle's
// USB enumeration spins in CTimer::MsDelay, so it stalls the main loop once,
// about a second (est.), after the body's first HELLO and the boot-time
// flash check (CKiloNet::BringUp).
//
// Addressing (protocol/spec.toml [host]): Circle asks DHCP as "kilo"; when
// no lease came KILO_HOST_DHCP_TIMEOUT_MS after the Ethernet link came up,
// the task writes the fixed link-local address into CNetConfig. Circle
// chooses DHCP or a static address at construction and cannot stop its DHCP
// client afterwards, so the client keeps trying in the background; a lease
// that comes later replaces the fallback, and the task then listens on the
// new address (connections on the old one die). A listening socket is bound
// to the address it was made with (Circle's TCP checksum takes the own
// address when the connection is created), hence a new one on every change.
//
#include "net.h"
#include "util/log.h"
#include "util/fmt.h"
#include <circle/net/in.h>
#include <circle/net/netconfig.h>
#include <circle/net/ipaddress.h>
#include <circle/net/mdnsdaemon.h>
#include <circle/net/mdnspublisher.h>
#include <circle/sched/scheduler.h>
#include <circle/string.h>
#include <circle/timer.h>
#include <circle/util.h>
#include <assert.h>

using namespace kilo;

static const unsigned NetPeriodUs = 1000;		// the task's pass
static const uint64_t DhcpTimeoutUs = KILO_HOST_DHCP_TIMEOUT_MS * 1000ull;
static const uint64_t NoLinkWarnUs = 10000000;		// "is the cable in?"
static const unsigned SendChunk = 1024;			// below any MSS: one segment, no yield inside Send
static const unsigned AcceptsPerPass = CircleListener::Slots;

// ---- the seams, for the main loop ---------------------------------------------

CircleConn::CircleConn (void)
:	m_State (StateFree),
	m_pSocket (0),
	m_bFailed (FALSE),
	m_nForeignPort (0)
{
	memset (m_ForeignIP, 0, sizeof m_ForeignIP);
	m_Rx.Reset ();
	m_Tx.Reset ();
}

int32_t CircleConn::read (uint8_t *pBuffer, uint32_t nCount)
{
	if (m_State != StateOpen)
	{
		return -1;
	}

	unsigned n = m_Rx.Get (pBuffer, nCount);
	if (n > 0)
	{
		return (int32_t) n;
	}

	// What arrived before the peer closed is read first.
	return m_bFailed ? -1 : 0;
}

int32_t CircleConn::write (const uint8_t *pBuffer, uint32_t nCount)
{
	if (m_State != StateOpen || m_bFailed)
	{
		return -1;
	}

	return (int32_t) m_Tx.Put (pBuffer, nCount);
}

void CircleConn::close (void)
{
	if (m_State == StateOpen)
	{
		m_State = StateClosing;		// the task deletes the socket, then frees the slot
	}
}

int32_t CircleListener::accept (void)
{
	for (unsigned i = 0; i < Slots; i++)
	{
		if (m_Conn[i].m_State == CircleConn::StatePending)
		{
			m_Conn[i].m_State = CircleConn::StateOpen;

			return (int32_t) i;
		}
	}

	return -1;
}

// ---- the task -----------------------------------------------------------------

CKiloNet::CKiloNet (CNetSubSystem *pNet, CUSBHCIDevice *pUSB, const boolean *pGoAhead,
		    kilo::Log *pLog, CircleListener *pListener, u32 nBuild)
:	CTask (0x10000),		// 64 KB: the USB scan runs on this stack
	m_pNet (pNet),
	m_pUSB (pUSB),
	m_pGoAhead (pGoAhead),
	m_pLog (pLog),
	m_pListener (pListener),
	m_pListen (0),
	m_bListening (FALSE),
	m_bLinkUp (FALSE),
	m_nLinkUpUs (0),
	m_bFallback (FALSE),
	m_bNoLinkLogged (FALSE),
	m_nStartUs (0),
	m_nRefused (0),
	m_nBuild (nBuild),
	m_pMdns (0)
{
	memset (m_IP, 0, sizeof m_IP);
	SetName ("kilonet");
}

void CKiloNet::Run (void)
{
	m_nStartUs = m_Clock.now_us ();

	BringUp ();

	for (;;)
	{
		WatchAddress (m_Clock.now_us ());
		AcceptAll ();
		for (unsigned i = 0; i < CircleListener::Slots; i++)
		{
			Pump (&m_pListener->m_Conn[i]);
		}

		CScheduler::Get ()->usSleep (NetPeriodUs);
	}
}

void CKiloNet::BringUp (void)
{
	if (m_pUSB != 0)
	{
		// The scan spins (no yield) for about a second (est.): let the link
		// get its HELLO and a boot-time flash run first.
		while (   !(m_pGoAhead != 0 && *m_pGoAhead)
		       && m_Clock.now_us () - m_nStartUs < UsbWaitUs)
		{
			CScheduler::Get ()->MsSleep (10);
		}

		uint64_t t0 = m_Clock.now_us ();
		boolean bOK = m_pUSB->Initialize ();
		unsigned nMs = (unsigned) ((m_Clock.now_us () - t0) / 1000);
		logf (*m_pLog, bOK ? LogLevel::Info : LogLevel::Error,
		      "USB scan for Ethernet: %s, %u ms (the main loop waited)", bOK ? "done" : "FAILED", nMs);
		if (!bOK)
		{
			return;		// no Ethernet: the task idles; the mind runs on
		}
	}

	// FALSE: do not wait for the PHY (Circle would spin up to 4 s); the
	// device and the DHCP client are taken up in CNetTask's Process().
	if (!m_pNet->Initialize (FALSE))
	{
		logf (*m_pLog, LogLevel::Error, "network: Circle's TCP/IP stack failed to start");
		return;
	}

	logf (*m_pLog, LogLevel::Info, "network: asking DHCP as %s; %u.%u.%u.%u if no lease %u ms after the link is up",
	      m_pNet->GetHostname (), 169u, 254u, 75u, 73u, (unsigned) KILO_HOST_DHCP_TIMEOUT_MS);
}

void CKiloNet::WatchAddress (u64 nNowUs)
{
	boolean bLinkUp = m_pNet->GetNetDeviceLayer ()->IsRunning ();
	if (bLinkUp != m_bLinkUp)
	{
		m_bLinkUp = bLinkUp;
		if (bLinkUp)
		{
			m_nLinkUpUs = nNowUs;
			logf (*m_pLog, LogLevel::Info, "Ethernet link up, %u ms after the net task started",
			      (unsigned) ((nNowUs - m_nStartUs) / 1000));
		}
		else
		{
			logf (*m_pLog, LogLevel::Warn, "Ethernet link down");
		}
	}
	if (!m_bLinkUp && !m_bNoLinkLogged && nNowUs - m_nStartUs > NoLinkWarnUs)
	{
		m_bNoLinkLogged = TRUE;
		logf (*m_pLog, LogLevel::Warn, "no Ethernet link after %u s: is the cable in?",
		      (unsigned) (NoLinkWarnUs / 1000000));
	}

	CNetConfig *pConfig = m_pNet->GetConfig ();
	if (   pConfig->GetIPAddress ()->IsNull ()
	    && m_bLinkUp
	    && nNowUs - m_nLinkUpUs >= DhcpTimeoutUs)
	{
		static const u8 Fallback[4] = KILO_HOST_FALLBACK_IP_INIT;
		u8 Mask[4];
		u32 nBits = KILO_HOST_FALLBACK_PREFIX;
		for (unsigned i = 0; i < 4; i++, nBits = nBits > 8 ? nBits - 8 : 0)
		{
			Mask[i] = nBits >= 8 ? 0xFF : (u8) (0xFF << (8 - nBits));
		}
		pConfig->SetIPAddress (Fallback);
		pConfig->SetNetMask (Mask);
		m_bFallback = TRUE;
		logf (*m_pLog, LogLevel::Warn, "no DHCP lease %u ms after the link came up: link-local %u.%u.%u.%u/%u "
		      "(DHCP keeps trying)", (unsigned) KILO_HOST_DHCP_TIMEOUT_MS, Fallback[0], Fallback[1],
		      Fallback[2], Fallback[3], (unsigned) KILO_HOST_FALLBACK_PREFIX);
	}

	if (m_bFallback && m_pNet->IsRunning ())	// IsRunning: the DHCP lease is bound
	{
		m_bFallback = FALSE;
		logf (*m_pLog, LogLevel::Info, "DHCP lease arrived after the link-local fallback");
	}

	u8 IP[4];
	pConfig->GetIPAddress ()->CopyTo (IP);
	if (memcmp (IP, m_IP, sizeof IP) != 0)
	{
		memcpy (m_IP, IP, sizeof m_IP);
		OpenListener ();
	}
}

void CKiloNet::OpenListener (void)
{
	if (m_pListen != 0)
	{
		delete m_pListen;	// its backlog connections carry the old address
		m_pListen = 0;
		m_bListening = FALSE;
	}

	const u8 *a = m_IP;
	if ((a[0] | a[1] | a[2] | a[3]) == 0)
	{
		logf (*m_pLog, LogLevel::Warn, "network: no address (lease lost?)");
		return;
	}

	m_pListen = new CSocket (m_pNet, IPPROTO_TCP);
	if (   m_pListen->Bind (KILO_HOST_PORT) < 0
	    || m_pListen->Listen (CircleListener::Slots) < 0)
	{
		delete m_pListen;
		m_pListen = 0;
		logf (*m_pLog, LogLevel::Error, "network: cannot listen on port %u", (unsigned) KILO_HOST_PORT);
		return;
	}

	m_bListening = TRUE;
	logf (*m_pLog, LogLevel::Info, "listening on %u.%u.%u.%u:%u (%s): kilo status --host %u.%u.%u.%u",
	      a[0], a[1], a[2], a[3], (unsigned) KILO_HOST_PORT,
	      m_bFallback ? "link-local fallback" : "DHCP lease", a[0], a[1], a[2], a[3]);

#if KILO_MDNS
	if (m_pMdns == 0)
	{
		m_pMdns = new CKiloMdns (m_pNet, m_pLog, m_nBuild);	// first runs at this task's next sleep
	}
#endif
}

void CKiloNet::AcceptAll (void)
{
	if (!m_bListening)
	{
		return;
	}

	for (unsigned n = 0; n < AcceptsPerPass; n++)
	{
		// Ready: a backlog connection is established, so Accept() takes it
		// without waiting (it would otherwise block this task until one is).
		if (!m_pListen->GetStatus ().bRxReady)
		{
			return;
		}

		CIPAddress ForeignIP;
		u16 nForeignPort = 0;
		CSocket *pSocket = m_pListen->Accept (&ForeignIP, &nForeignPort);
		if (pSocket == 0)
		{
			return;
		}

		CircleConn *pConn = 0;
		unsigned nSlot;
		for (nSlot = 0; nSlot < CircleListener::Slots; nSlot++)
		{
			if (m_pListener->m_Conn[nSlot].m_State == CircleConn::StateFree)
			{
				pConn = &m_pListener->m_Conn[nSlot];
				break;
			}
		}

		u8 IP[4];
		ForeignIP.CopyTo (IP);
		if (pConn == 0)
		{
			delete pSocket;		// closes it
			m_nRefused++;
			logf (*m_pLog, LogLevel::Warn, "host: refused %u.%u.%u.%u:%u: all %u slots busy",
			      IP[0], IP[1], IP[2], IP[3], (unsigned) nForeignPort, CircleListener::Slots);
			continue;
		}

		pConn->m_pSocket = pSocket;
		pConn->m_bFailed = FALSE;
		pConn->m_Rx.Reset ();
		pConn->m_Tx.Reset ();
		memcpy (pConn->m_ForeignIP, IP, sizeof IP);
		pConn->m_nForeignPort = nForeignPort;
		pConn->m_State = CircleConn::StatePending;	// HostServer's accept() opens it
		logf (*m_pLog, LogLevel::Info, "host: connection from %u.%u.%u.%u:%u (slot %u)",
		      IP[0], IP[1], IP[2], IP[3], (unsigned) nForeignPort, nSlot);
	}
}

void CKiloNet::Pump (CircleConn *pConn)
{
	switch (pConn->m_State)
	{
	case CircleConn::StateFree:
		return;

	case CircleConn::StateClosing:
		delete pConn->m_pSocket;	// terminates the connection (FIN)
		pConn->m_pSocket = 0;
		pConn->m_Rx.Reset ();
		pConn->m_Tx.Reset ();
		pConn->m_bFailed = FALSE;
		pConn->m_State = CircleConn::StateFree;
		return;

	case CircleConn::StatePending:
	case CircleConn::StateOpen:
		break;
	}

	if (pConn->m_bFailed)
	{
		return;			// until HostServer reads the end and closes the slot
	}

	CSocket *pSocket = pConn->m_pSocket;
	assert (pSocket != 0);
	CSocket::TStatus Status = pSocket->GetStatus ();
	if (!Status.bConnected)
	{
		pConn->m_bFailed = TRUE;	// reset, or closed by both ends
		return;
	}

	// RX: Receive() hands over a whole segment and drops what does not fit
	// the buffer, so only when a whole one fits the ring.
	if (Status.bRxReady && pConn->m_Rx.Free () >= sizeof m_RxBuffer)
	{
		int nResult = pSocket->Receive (m_RxBuffer, sizeof m_RxBuffer, MSG_DONTWAIT);
		if (nResult > 0)
		{
			pConn->m_Rx.Put (m_RxBuffer, (unsigned) nResult);
		}
		else if (nResult < 0)
		{
			pConn->m_bFailed = TRUE;	// the peer closed (FIN after its data) or an error
			return;
		}
	}

	// TX: only while Circle's TCP queue is below its limit; with
	// MSG_DONTWAIT it would otherwise queue without bound.
	if (pConn->m_Tx.m_nLen > 0 && Status.bTxReady)
	{
		unsigned nCount;
		const u8 *pData = pConn->m_Tx.Contiguous (&nCount);
		if (nCount > SendChunk)
		{
			nCount = SendChunk;
		}

		// Send() copies the bytes into a buffer, then yields once: the main
		// loop may append to the ring meanwhile (past these bytes) or close
		// the slot (the next pass frees it).
		int nResult = pSocket->Send (pData, nCount, MSG_DONTWAIT);
		if (nResult < 0)
		{
			pConn->m_bFailed = TRUE;
			return;
		}
		pConn->m_Tx.Drop ((unsigned) nResult);
	}
}

#if KILO_MDNS

// ---- mDNS -----------------------------------------------------------------------

CKiloMdns::CKiloMdns (CNetSubSystem *pNet, kilo::Log *pLog, u32 nBuild)
:	m_pNet (pNet),
	m_pLog (pLog)
{
	format (m_Text, sizeof m_Text, "build=%08x", (unsigned) nBuild);
	SetName ("kilomdns");
}

void CKiloMdns::Run (void)
{
	// The publisher's constructor starts Circle's responder and sleeps until
	// it has probed the name (3 queries 250 ms apart): this task waits, the
	// main loop and the net task run.
	uint64_t t0 = CTimer::GetClockTicks64 ();
	CmDNSPublisher *pPublisher = new CmDNSPublisher (m_pNet);
	CString Hostname (CmDNSDaemon::Get ()->GetHostname ());
	logf (*m_pLog, LogLevel::Info, "mDNS: %s.local answers with this address (%u ms to claim the name)",
	      (const char *) Hostname, (unsigned) ((CTimer::GetClockTicks64 () - t0) / 1000));

	static const char ServiceName[] = "kilo";
	const char *Text[] = {m_Text, nullptr};
	boolean bPublished = FALSE;
	u8 Last[4] = {0, 0, 0, 0};
	for (;;)
	{
		u8 IP[4];
		m_pNet->GetConfig ()->GetIPAddress ()->CopyTo (IP);
		if (   (IP[0] | IP[1] | IP[2] | IP[3]) != 0
		    && memcmp (IP, Last, sizeof IP) != 0)
		{
			// A new address: withdraw the service (a goodbye with TTL 0) and
			// announce it again; the A record carries the address of the moment.
			if (bPublished)
			{
				pPublisher->UnpublishService (ServiceName);
				bPublished = FALSE;
			}
			if (pPublisher->PublishService (ServiceName, "_kilo._tcp", KILO_HOST_PORT, Text))
			{
				bPublished = TRUE;
				memcpy (Last, IP, sizeof Last);
				logf (*m_pLog, LogLevel::Info, "mDNS: announced %s._kilo._tcp.local, port %u, at "
				      "%u.%u.%u.%u: kilo status --host %s.local", ServiceName, (unsigned) KILO_HOST_PORT,
				      IP[0], IP[1], IP[2], IP[3], (const char *) Hostname);
			}
			// else the publisher has not started yet: again next pass
		}

		CScheduler::Get ()->MsSleep (PeriodMs);
	}
}

#endif	// KILO_MDNS
