/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <sstream>
#include <errno.h>

#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/rprimitive/rstring.h>
#include <rdkcore/common/threads.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "TcpManager"

#include "repository.h"
#include "tcpmanager.h"


#define TCPMANAGER_DEBUG
// #define TcpManager_listener_DEBUG

#ifdef TCPMANAGER_DEBUG
	#define DOUT(a) { RDK_DEBUG_STREAM(a) }
#else
	#define DOUT(a)
#endif

namespace RDK2 { namespace RepositoryNS {

using namespace std;
using namespace Common;
using namespace Network;
using RDK2::RNetObjects::RYellowPages;

TcpManager::TcpManager(Repository* repository) : repository(repository), exiting(false)
{
	pthread_mutex_init( &mutex, NULL );
}

bool TcpManager::init()
{
	Session* session = repository->createSession("tcpPortBinder",
		"TCP socket binding session", "/");
	bool done = false;

	while (!done) {
		try {
			session->start();
			session->lock(PROPERTY_YELLOWPAGES, WHERE("Getting my address"));
			RYellowPages* yp = session->getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);
			if (yp->getSocketAddress(Network::TCP_IP, session->getRepositoryName(), myAddr)) {
				session->setIntB(PROPERTY_TCP_LISTENING_PORT, myAddr.getPort());
				done = true;
			}
			else {
				RDK_ERROR_PRINTF("I cannot find my address in the yellow pages");
			}
			session->unlock(PROPERTY_YELLOWPAGES);

			if (!done) {
				session->end();
				RDK_ERROR_PRINTF("Something wrong in loading TCP listening port");
				delete session;
				return false;
			}
			session->end();
		}
		catch (const SessionException& e) {
			session->terminate();
			delete session;
			RDK_ERROR_PRINTF(e.what());
			return false;
		}
	}

	delete session;

	done = false;
	while ( !done ) {
		try {
			serverSocket.bind( myAddr.getPort() );
			done = true;
		} catch ( Network::NetException& netexc ) {
			RDK_ERROR_STREAM( "Failure in binding TCP Server: " << netexc.fullwhat() );
			sleep( 4 );
		}
	}

	return true;
}

bool TcpManager::start()
{
	if (!createThread(&tcpListenerThreadId,
		(void*(*)(void*))tcpListenerThreadFn, this,
		"TCP listener thread")) return false;
	if (!createThread(&tcpAutomaticPeerCreatorThreadId,
		(void*(*)(void*))tcpAutomaticPeerCreatorThreadFn,
		this, "TCP automatic peer creator thread")) return false;
	return true;
}

bool TcpManager::areYouInterested(const RDK2::Object* obj)
{
	const RNetMessage* message;
	if (((message = dynamic_cast<const RNetMessage*>(obj)) != 0)
	&& (Network::TCP_IP == message->getNetProtocol())) {
		istringstream iss(message->getAddressees());
		string tmp;
		while (iss >> tmp) {
			if (peerMap.find(tmp) == peerMap.end()) return true;
			else if (peerMap[tmp]->getStatus() != CONNECTED) return true;
		}
	}
	return false;
}

void TcpManager::tcpAutomaticPeerCreator()
{
	tcpAutomaticPeerCreatorSession = repository->createSession("tcpAutoPeerCreator",
		"TCP automatic peer creator session", "/");
	Session* session = tcpAutomaticPeerCreatorSession;

	SESSION_TRY_START(session)
	session->queueSubscribe(PROPERTY_OUTBOX);
	session->queueSetInterests(PROPERTY_OUTBOX, this);
	session->listen(PROPERTY_OUTBOX);
	SESSION_END_CATCH_TERMINATE(session)

	set<string> peersCreatedNow;

	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)

		vector<const RDK2::Object*> q = session->queueFreeze(PROPERTY_OUTBOX);
		for (size_t i = 0; i < q.size(); i++) {
			const RNetMessage* msg = dynamic_cast<const RNetMessage*>(q[i]);
			if (msg) {
				istringstream iss(msg->getAddressees());
				string tmp;
				while (iss >> tmp) {
					if (tmp == session->getRepositoryName()) {
						session->queuePush(PROPERTY_INBOX, msg->clone());
					}
					else if (peerMap.find(tmp) == peerMap.end()) {
						RDK_DEBUG_PRINTF("We have a TCP message for '%s', we will try to connect",
							tmp.c_str());
						session->tryConnectTo(Network::TCP_IP, tmp);
						// at this point, a peer manager should have been created
						if (peerMap.find(tmp) != peerMap.end()) {
							peersCreatedNow.insert(tmp);
							RDK_DEBUG_PRINTF("Putting the first message in sender queue");
							RNetMessage* msgCloned =
								new RNetMessage(tmp, msg->getSender(), msg->getType(),
								msg->getNetProtocol(), msg->getPayload()->clone());
							// the message arrives directly in the sender outbox
							// and only in his outbox
							session->queuePush(PROPERTY_OUTBOX, msgCloned);
							// in this way the message is not lost FIXME queue size!
						}
						else {
							RDK_ERROR_PRINTF("tryTcpConnectWith failed in creating the "
								"peer manager, messages for this peer are lost");
						}
					}
					else if (find(peersCreatedNow.begin(), peersCreatedNow.end(), tmp)
					!= peersCreatedNow.end()) {
						// there is a peer manager for this message, but we
						// created it now, so the message have to be put in his
						// outbox
						RNetMessage* msgCloned =
							new RNetMessage(tmp, msg->getSender(), msg->getType(),
							msg->getNetProtocol(), msg->getPayload()->clone());
						// the message arrives directly in the sender outbox
						// and only in his outbox
						session->queuePush(PROPERTY_OUTBOX, msgCloned);
					}
				}
			}
		}

		SESSION_END_CATCH_TERMINATE(session)

		peersCreatedNow.clear();
	}

	SESSION_TRY_START(session)
	session->queueUnsubscribe(PROPERTY_OUTBOX);
	SESSION_END_CATCH_TERMINATE(session)
	delete session;
}

TcpManager::~TcpManager()
{
	exiting = true;
	serverSocket.unbind();
	pthread_join(tcpListenerThreadId, 0);

	tcpAutomaticPeerCreatorSession->wakeUp();
	pthread_join(tcpAutomaticPeerCreatorThreadId, 0);

	pthread_mutex_lock( &mutex );

	std::map<std::string, PeerManager*>::iterator iter;
	PeerManager* temp;
	while ( !peerMap.empty() ) {
		iter = peerMap.begin();
		temp = iter->second;
		peerMap.erase( iter );
		delete temp;
	}
	
	invalidSet.clear();
	RDK_INFO_PRINTF("TCP manager closed");

	pthread_mutex_unlock( &mutex );
	pthread_mutex_destroy( &mutex );
}

bool TcpManager::tryConnectTo( const string& peerName, time_t timeout )
{
	RDK_DEBUG_PRINTF("Trying connection with %s", peerName.c_str());
	if ( exiting ) {
		DOUT( "PeerSet::connect invoked on " << peerName << "but endOfWorkEnabled" )
		return false;
	}

	if ( peerName == repository->getRepositoryName() ) {
		RDK_ERROR_STREAM( "NONONONONO, AN OPERATOR IS USING DRUGS!!!!! I CANNOT CONNECT TO MYSELF, "
			"USE SOME SHARED DATA INSTEAD OF PISS ME OFF!!!!" );
		return false;
	}

	Network::InetAddress addr;
	if ( !getAddress( peerName, addr ) ) {
		DOUT( "Asked a connection for " << peerName << " but no address present" );
		return false;
	}

	pthread_mutex_lock( &mutex );
	
	if ( peerMap.find( peerName ) == peerMap.end() ) {
		DOUT( "Connect: Creating a peer manager for " << peerName );
		PeerManager* temp = new PeerManager( peerName, *this, NULL, timeout );
		peerMap.insert( make_pair( peerName, temp ) );
		invalidSet.erase( peerName  );
	} else {
		DOUT( "Asked a connection for " << peerName << " but manager already present" );
	}
	
	pthread_mutex_unlock( &mutex );

	return true;
}

Network::ConnectionStatus TcpManager::getConnectionStatusWith(const string& peerName)
{
	map<string, PeerManager*>::iterator it = peerMap.find(peerName);
	if (it == peerMap.end()) return NOT_CONNECTED;
	else return it->second->getStatus();
}

bool TcpManager::getAddress( const string& peerName, Network::InetAddress& address )
{
	bool addressok = false;
	repository->lock(PROPERTY_YELLOWPAGES, WHERE("Getting my address"));
	RYellowPages* yp = repository->getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);
	addressok = yp->getSocketAddress(Network::TCP_IP, peerName, address);
	repository->unlock(PROPERTY_YELLOWPAGES);
	return addressok;
}

void TcpManager::tcpListener()
{
#ifdef TcpManager_listener_DEBUG
	#warning TcpManager_listener_DEBUG enabled ( A lot of debug )
	#define LDOUT(a) { RDK_DEBUG_STREAM("PeerSet::accept: "<<a) }
#else
	#define LDOUT(a)
#endif
	//InetAddress myAddr;

	Session* session = repository->createSession("tcpListener", "TCP listener session", "/");

	RDK_INFO_STREAM( "TCP server ready on port " << myAddr.getPort() );

	unsigned short myUdpPort = 0;
	SESSION_TRY_START(session)
	myUdpPort = (unsigned short) session->getInt(PROPERTY_UDP_PORT);
	SESSION_END_CATCH_TERMINATE(session)

	while (!exiting) {
		TCPSocket* newSocket = NULL;
		while ( !newSocket && !exiting ) {
			try {
				DOUT( "Waiting for incoming TCP connection on port " <<
					serverSocket.getAddress().getPort() );
				newSocket = serverSocket.accept();
			} catch ( Network::NetException& netexc ) {
				if (!exiting) { RDK_ERROR_STREAM( "Error on accept: " << netexc.fullwhat() ); }
				bool done = false;
				while ( !done && !exiting ) {
					try {
						serverSocket.rebind();
						done = true;
					} catch ( Network::NetException& netexc ) {
						RDK_ERROR_STREAM( "Failure in rebinding TCP Server: " << netexc.fullwhat() );
						sleep( 4 );
					}
				}
			}
		}
	
		if ( exiting ) break;

		LDOUT( "Connection accepted from " << newSocket->getAddress().getIPAddress() << ":" <<
			newSocket->getAddress().getPort() );	
	
		// Take or not take decision
		pthread_mutex_lock( &mutex );
		try {
			string peerName;
			unsigned short peerUdpPort;
			// Good received name
			if ( ( *newSocket >> peerName >> peerUdpPort ) ) {
				DOUT( "Received name from peer: " << peerName <<
					" (his udp port is " << peerUdpPort << ")");
				// In case of lesser priority
				if ( peerName > repository->getRepositoryName() ) {
					// Sending connection accept
					if (*newSocket << repository->getRepositoryName() << " " << myUdpPort << endl) {
						std::map<std::string, PeerManager*>::iterator iter = peerMap.find( peerName );
						// Setting new connection
						if ( iter != peerMap.end() ) {
							iter->second->setNewConnection( newSocket );
						}
						else {
							// Creating new manager
							peerMap.insert( make_pair( peerName,
								new PeerManager( peerName, *this, newSocket ) ) );
						}
						Network::InetAddress udpAddress = newSocket->getAddress();
						udpAddress.setPort(peerUdpPort);
						addUdpPort(peerName, udpAddress, session);
					}
					else {
						RDK_ERROR_STREAM( "Connection arrived, but handshake failed with "
							<< peerName );
					}
				}
				// In case of greater priority
				else if ( peerName < repository->getRepositoryName() ) {
					// Connection present refusing
					if ( peerMap.find( peerName ) != peerMap.end() ) {
						DOUT( "Incoming connection from " << peerName << " refused" );
						delete newSocket;
					}
					// Connection non present refusing
					else if ( *newSocket << repository->getRepositoryName()
							<< " " << myUdpPort << endl) {
						peerMap.insert( make_pair( peerName,
							new PeerManager( peerName, *this, newSocket ) ) );
						Network::InetAddress udpAddress = newSocket->getAddress();
						udpAddress.setPort(peerUdpPort);
						addUdpPort(peerName, udpAddress, session);
					}
					else {
						RDK_ERROR_STREAM( "Connection arrived, but handshake failed with "
							<< peerName );
					}
				}
				else {
					RDK_ERROR_STREAM( "DUPE ERROR, ARRIVED A MESSAGE FROM MYSELF!!! "
						"OPERATOR STOP TO DRINK BEFORE WORK!!!!" );
					delete newSocket;
				}
			}
			else {
				RDK_ERROR_STREAM( "Connection arrived, but was lost before the "
					"arrive of the peerName" );
				delete newSocket;
			}
		}
		catch ( Network::NetException& netexc ) {
			RDK_ERROR_STREAM( "Non recoverable exception during handshake "
				"(destroying connection): " <<
				netexc.fullwhat() );
			delete newSocket;
		}
	
		pthread_mutex_unlock( &mutex );

	}
	serverSocket.unbind();
	delete session;
#undef LDOUT
}

void TcpManager::addUdpPort(const string& peerName, const Network::InetAddress& udpAddress, Session* session)
{
	bool done = false;
	//udpset.insert( peerName );
	while (!done) {
		try {
			session->start();
			session->lock(PROPERTY_YELLOWPAGES, HERE);
			RYellowPages* pages = session->getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);
			if (pages) {
				pages->newHost(Network::UDP_IP, peerName, udpAddress);
				done = true;
				RDK_INFO_STREAM("Connection accepted from " << peerName << " (UDP port " <<
					udpAddress.getPort() << ") ");
			}
			else {
				RDK_ERROR_STREAM("Why the devil this yellowpages is 0?");
			}
			session->unlock(PROPERTY_YELLOWPAGES);
			session->end();
		}
		catch (const SessionException& exc) {
			session->terminate();
			RDK_ERROR_STREAM("Error on loading ypages " << exc.what());
		}
		if (!done) sleep(4);
	}
}

}} // namespaces
