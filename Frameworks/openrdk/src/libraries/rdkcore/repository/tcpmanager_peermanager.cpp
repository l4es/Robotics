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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "TcpManager"

#include <sstream>
#include <iostream>
#include <cstring>

#include "tcpmanager.h"
#include "repository.h"

#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/rprimitive/rstring.h>
#include <rdkcore/common/threads.h>
#include <rdkcore/serialization_binary/binarywriter.h>
#include <rdkcore/serialization_binary/binaryreader.h>

#include <rdkcore/object/objectmanager.h>
#include <errno.h>

//#define TCPMANAGER_DEBUG
/*#define PeerManager_PeerManager_DEBUG
#define PeerManager_sender_DEBUG
#define PeerManager_receiver_DEBUG*/
//#define PeerManager__PeerManager_DEBUG
//#define PeerManager_areYouInterested_DEBUG
//#define PeerManager_connect_DEBUG

#ifdef TCPMANAGER_DEBUG
	#define DOUT(a) { RDK_DEBUG_STREAM(a) }
#else
	#define DOUT(a)
#endif

namespace RDK2 { namespace RepositoryNS {

using namespace std;
using namespace Network;
using namespace Common;
using namespace RDK2::Serialization::Binary;

TcpManager::PeerManager::PeerManager( const std::string& peerName, TcpManager& manager,
	Network::TCPSocket* socket, time_t giveUpTime ) :
	peerName( peerName ), manager(manager),
	socket(socket), receiverThreadId(0), exiting(false),
	giveUpTime(giveUpTime), status( Network::TRYING ), newSocketFlag(false),
	newSocket(NULL) {
#ifdef PeerManager_PeerManager_DEBUG
	#warning PeerManager_PeerManager_DEBUG enabled ( A lot of debug )
	#define LDOUT(a) { RDK_DEBUG_STREAM("PeerManager::PeerManager: "<<a) }
#else
	#define LDOUT(a)
#endif
	pthread_mutex_init( &mutex, NULL );

	senderSession = manager.repository->createSession("tcpSender[" + peerName + "]",
		"TCP sender session for '" + peerName + "'", "/");
	senderSession->start();
	senderSession->queueSubscribe(PROPERTY_OUTBOX);
	senderSession->queueSetInterests(PROPERTY_OUTBOX, this);
	senderSession->listen(PROPERTY_OUTBOX);
	senderSession->end();

	createThread(&receiverThreadId, (void*(*)(void*))receiverThreadFn, this, "TCP receiver thread for " + peerName);

#undef LDOUT
}

TcpManager::PeerManager::~PeerManager() {
#ifdef PeerManager__PeerManager_DEBUG

	#warning PeerManager__PeerManager_DEBUG enabled ( A lot of debug )
	#define LDOUT(a) { RDK_DEBUG_STREAM("PeerManager::~PeerManager: "<<a) }
#else
	#define LDOUT(a)// 	RDK_DEBUG_STREAM( "Scanning udpset: " << peerName );
// 	for ( std::set<std::string>::const_iterator iter = udpset.begin(); iter != udpset.end(); iter++ )
// 		RDK_DEBUG_STREAM( "client: " << *iter );

#endif
	exiting = true;
	if ( socket ) socket->disconnect();
	int joinresult = 0;
	if ( receiverThreadId )
		joinresult = pthread_join( receiverThreadId, NULL );
	joinresult = joinresult;
	DOUT( "Collected receiver thread for agent " << peerName <<
	      "( join for sender returned \"" << strerror( joinresult ) << "\" )");

	if ( socket ) delete socket;
	LDOUT( "Deleted data of PeerManager for " << peerName );

	SESSION_TRY_START(senderSession)
	senderSession->queueUnsubscribe(PROPERTY_OUTBOX);
	SESSION_END_CATCH_TERMINATE(senderSession)

	delete senderSession;
#undef LDOUT
}

bool TcpManager::PeerManager::areYouInterested( const RDK2::Object* object ) {
#ifdef PeerManager_areYouInterested_DEBUG
	#warning PeerManager_areYouInterested_DEBUG enabled ( A lot of debug )
	#define LDOUT(a) { RDK_DEBUG_STREAM("PeerManager::areYouInterested: "<<a) }
#else
	#define LDOUT(a)
#endif
	const RNetMessage* message;

	#ifdef PeerManager_areYouInterested_DEBUG
	if ( ( message = dynamic_cast<const RNetMessage*>( object ) ) == NULL ) {
		LDOUT( "Message discarded for " << peerName << " ( Not an RNetMessage )" );
		return false;
	} else if ( Network::TCP_IP != message->getNetProtocol() ) {
		LDOUT( "Message discarded for " << peerName << " ( Not for TCP_IP )" );
		return false;
	} else {
		string tmp;
		istringstream iss( message->getAddressees() );
		while ( iss >> tmp ) {
			if ( tmp == peerName ) {
				LDOUT( "Message accepted for " << peerName );
				return true;
			}
		}
	}
	LDOUT( "Message discarded for " << peerName << " ( addressees = \"" <<
	       message->getAddressees() << "\" )");
	#else
	if ( ( ( message = dynamic_cast<const RNetMessage*>( object ) ) != NULL ) &&
		( Network::TCP_IP == message->getNetProtocol() ) ) {
		istringstream iss( message->getAddressees() );
		string tmp;
		while ( iss >> tmp ) {
			if ( tmp == peerName ) {
				return true;
			}
		}
	}
	#endif
	return false;
#undef LDOUT
}

void TcpManager::PeerManager::sender( ) {
#ifdef PeerManager_sender_DEBUG
	#warning PeerManager_sender_DEBUG enabled ( A lot of debug )
	#define LDOUT(a) { RDK_DEBUG_STREAM("PeerManager::sender: "<<a) }
#else
	#define LDOUT(a)
#endif
	LDOUT( "Launched sender thread for agent " << peerName )

	// the session has been created by the PeerManager constructor and will be deleted by the desctructor
	// in this way, outbox messages for this peer will never get lost
	Session* session = senderSession;

	// each time the sender is started, we have to send the TCP subscriptions for this peer
	SESSION_TRY_START(session)
	RemoteSubscriptionRegister* reg = &(manager.repository->remoteSubscriptionRegister);
	vector<RRemoteSubscription> subToRenew = reg->queryForPropertiesToReceive("", "", Network::TCP_IP,
		RRemoteSubscription::WHAT_ANY, RRemoteSubscription::WHEN_ANY);
	for (vector<RRemoteSubscription>::iterator it = subToRenew.begin(); it != subToRenew.end(); ++it) {
		if (it->completeUrl.getHost() == peerName) {
			LDOUT("Remotely refreshing TCP subscription for property '"
				<< it->completeUrl.c_str() << "' to '" << it->completeUrl.getHost().c_str() << "'");
			session->queuePush(PROPERTY_OUTBOX,
				new RNetMessage(it->completeUrl.getHost(), session->getRepositoryName(),
				RNetMessage::PROPERTY_SUBSCRIPTION, Network::UDP_IP, it->clone()));
		}
	}
	SESSION_END_CATCH_TERMINATE(session)

	while (session->wait(), !stopSender) {
		while (!stopSender && getStatus() != CONNECTED) {
			LDOUT("Still not connected with '" << peerName.c_str() << "'");
			sleep(1);
		}
		if (stopSender) break;
		try {
			BinaryWriter bw(true);
			session->start();
			const vector<const RDK2::Object*> toSend = session->queueFreeze(PROPERTY_OUTBOX);
			for (vector<const RDK2::Object*>::const_iterator it = toSend.begin();
			it != toSend.end(); ++it) {
				const RNetMessage* msg = dynamic_cast<const RNetMessage*>(*it);
				if (!msg) {
					RDK_ERROR_PRINTF("There wasn't a RNetMessage in the outbox! (class '%s')",
						(*it)->getClassName().c_str());
				}
				else {
					try {
						string s = bw.serialize(true, msg);
						//for (size_t i = 0; i < s.size(); i++) cout << (int) s[i] << " ";
						*socket << s << endl;
						LDOUT("Message sent successfully (" << s.size() << " bytes)");
					}
					catch (const WritingException& e) {
						RDK_ERROR_PRINTF(e.what());
						stopSender = true;
						// FIXME here messages are lost!!!! queueKeep?
					}
				}
/*				else if (!msg->save(*socket) || !(*socket<<endl)) {
					RDK_ERROR_PRINTF("Error to send");
					stopSender = true;
					// FIXME here messages are lost!!!! queueKeep?
				}
				else {
					LDOUT("Message sent successfully");
				}*/
			}
			session->end();
		}
		catch (const SessionException& e) {
			session->terminate();
			RDK_ERROR_PRINTF(e.what());
		}
	}
	socket->disconnect();

#undef LDOUT
}

void TcpManager::PeerManager::receiver() {
#ifdef PeerManager_receiver_DEBUG
	#warning PeerManager_receiver_DEBUG enabled ( A lot of debug )
	#define LDOUT(a) { RDK_DEBUG_STREAM("PeerManager::receiver: "<<a) }
#else
	#define LDOUT(a)
#endif
	LDOUT( "Launched receiving thread for agent " << peerName );

	Session* session = manager.repository->createSession("tcpReceiver[" + peerName + "]",
		"TCP receiver session for '" + peerName + "'", "/");

	bool connected = ( socket != NULL );
	bool emergencySelfDestruct = false;
	
	if ( peerName == manager.repository->getRepositoryName() ) {
		RDK_ERROR_STREAM(
			"OPERATOR YOU ARE TRYING TO MAKE ME CONNECT MYSELF!!! STOP DRINK YOU DRUNK" );
		emergencySelfDestruct = true;
	} else if ( !connected ) {
		LDOUT( "Creating TCP Socket for " << peerName );
		socket = new TCPSocket();
		connected = connect( false );
		if ( connected ) {
			LDOUT( "Connect returned true" );
		} else {
			LDOUT( "Connect returned false (" << strerror(errno) << ")");
		}
	} else {
		status = Network::CONNECTED;
	}
	
	// Create the sender thread
	stopSender = false;
	if ( connected && !exiting && !emergencySelfDestruct ) {
		while ( !exiting && pthread_create( &senderThreadId,
						      NULL,
						      (void*(*)(void*))senderThreadFn,
						      this ) != 0 ) {
			RDK_ERROR_STREAM(
				"Failed to launch sender thread for " << peerName << " retry in 4s" );
			sleep( 4 );
		}
	}

	// Receive
	if ( connected && !exiting && !emergencySelfDestruct ) {
		// Receiving cicle
		bool unableToContinue = false;
		while ( !exiting && !unableToContinue ) {
			bool error = false;
			BinaryReader br;
			try {
				session->start();
				int sck = socket->getSocket();
				char c;
				::recv(sck, &c, 1, 0);
				LDOUT("Read character '" << c << "' (" << (int) c << ")");
				if (c == '\n') {
					LDOUT("Got EOF from stream");
					error = true;	// EOF
				}
				else {
					uint32_t l;
					::recv(sck, &l, 4, 0);
					LDOUT("Read len " << htonl(l));
					if (l <= 0) {
						error = true;
					}
					else {
						char* buffer = new char[htonl(l)+5+1];
						buffer[0] = c;
						memcpy(buffer+1, &l, 4);
						l = htonl(l);
						uint readBytes = 0;
						while (readBytes < l) {
							/* :WORKAROUND:29/12/09 16:25:20:lm:
							 * The two lines below prevent ragent hanging when the remote peer
							 * reset its connection (due to exiting called) 
							 * In this situation, the ragent stop receiving bytes and
							 * it hangs waiting for more bytes*/
							if (exiting)
								readBytes = l;
							int curBytes = ::recv(sck, buffer+5+readBytes, l-readBytes, 0);
							readBytes += curBytes;
							LDOUT("Read " << readBytes << " bytes");
						}
						::recv(sck, &c, 1, 0);	// reads EOF ('\n')
						string msg;
						//for (size_t i = 0; i < l+5; i++) cout << (int) buffer[i] << " ";
						msg.append(buffer, l+5);
						LDOUT("Received " << readBytes << " bytes, wanted " << l
							<< " (message size: " << msg.size() << ")");
						RNetMessage* message = dynamic_cast<RNetMessage*>(br.deserialize(msg));
						session->queuePush(PROPERTY_INBOX, message);
					}
				}
				session->end();
			}
			catch ( Network::NetException& netexc ) {
				session->terminate();
				RDK_ERROR_STREAM("Failure in receiving from " << peerName << ": " << netexc.fullwhat());
				error = true;
			}
			catch (const ReadingException& e) {
				session->terminate();
				RDK_ERROR_PRINTF("Error in retrieving the object (%s)", e.what());
				error = true;
			}
			// Connection reset management
			if ( error ) {
				stopSender = true;
				socket->disconnect();
				if (senderSession) senderSession->wakeUp();
				int joinresult = 0;
				if ( senderThreadId )
					joinresult = pthread_join( senderThreadId, NULL );
				joinresult = joinresult;
				senderThreadId = 0;
				DOUT( "Collected sender thread for agent " << peerName << "( join for sender returned \"" << 
					strerror( joinresult ) << "\" )");
				
				status = Network::CONNECTION_RESET;
				giveUpTime = 0;
				session->start();
				if ( connect( false ) ) {
					stopSender = false;
					while (!exiting 
					&& pthread_create(&senderThreadId, NULL, (void*(*)(void*))senderThreadFn, this) != 0 ) {
						RDK_ERROR_STREAM( "Failed to launch sender thread for " << peerName <<
							" retry in 4s" );
						sleep( 4 );
						// FIXME AC: questo loop non c'ha senso! 
					}
				} else {
					unableToContinue = true;
				}
				session->end();
			}
		}
	}

	if ( connected ) {
		stopSender = true;
		socket->disconnect();
		if (senderSession) senderSession->wakeUp();
		int joinresult = 0;
		if ( senderThreadId )
			joinresult = pthread_join( senderThreadId, NULL );
		joinresult = joinresult;
		DOUT( "Collected sender thread for agent " << peerName << "( join for sender returned \"" <<
			strerror( joinresult ) << "\" )");
	}
	
	if ( !exiting ) {
		DOUT( "Connection destroyed with agent " << peerName );
/*		if ( peerSet )
			peerSet->remove( peerName, true );*/ //FIXME
	}

	delete session;
	// If exiting I dont need to call this, because the destructor is called :P
	// ( Only the destructor can set the exiting flag )
#undef LDOUT
}

bool TcpManager::PeerManager::connect( bool giveUpOnRefuse ) {
#ifdef PeerManager_connect_DEBUG
	#warning PeerManager_connect_DEBUG enabled ( A lot of debug )
	#define LDOUT(a) { RDK_DEBUG_STREAM("PeerManager::connect: "<<a) }
#else
	#define LDOUT(a)
#endif
	LDOUT("Trying to connect ( exiting = " << exiting << " giveUpTime = " << (int)giveUpTime << " )" )
	bool connected = false;
	bool refused = false;
	
	InetAddress addr;
	if ( !manager.getAddress( peerName, addr ) ) {
		RDK_DEBUG_STREAM( "Cannot reconnect with " << peerName << " closing connection" )
		return false;
	}
	
	if ( giveUpTime )
		giveUpTime = giveUpTime + ::time(NULL);

	// Try to create the connection for giveUpTime seconds
	while ( !connected &&
		!exiting &&
		( !refused || !giveUpOnRefuse ) &&
		( !giveUpTime || giveUpTime > ::time(NULL) ) ) {
		try {
			socket->connect( addr );
			LDOUT( "Connection active, exchanging names" );
			string response;
			if (	( *socket << manager.repository->getRepositoryName() << ' ' << 0 << endl ) &&
				( *socket >> response ) &&
				response == peerName ) {
				connected = true;
				DOUT( "Connection established with " << peerName );
			} else {
				if ( giveUpOnRefuse )
					RDK_ERROR_STREAM( "Connection refused from " << peerName << "( returned \"" << response
						<< "\" ), give up" )
				else
					RDK_ERROR_STREAM( "Connection refused from " << peerName << "( returned \"" << response
						<< "\"), but not give up" )
				refused = true;
			}
		} catch ( Network::NetException& netexc ) {
			RDK_ERROR_STREAM("Error connecting to " << peerName << ": " << netexc.fullwhat());
			refused = true;
		}
		if ( !connected ) {
			pthread_mutex_lock( &mutex );
			if ( newSocketFlag ) {
				LDOUT( "New Socket taken" );
				try {
					delete socket;
				} catch ( Network::NetException& netexc ) {
					RDK_ERROR_STREAM("Error deleting socket for " << peerName << ": " << netexc.fullwhat());
				}
				socket = newSocket;
				newSocket = NULL;
				newSocketFlag = false;
				connected = true;
			}
			pthread_mutex_unlock( &mutex );
			sleep( 4 );	// FIXME was at the beginning of this "if"
		}
	}
	if ( connected ) {
		LDOUT( "Connection succesfull")
		status = Network::CONNECTED;
	} else if ( exiting ) {
		LDOUT( "Connection halted ( EndOfWork )" )
	} else {
		LDOUT( "Connection give up ( timeout or refused )" )
	}
	return connected;
#undef LDOUT
}

bool TcpManager::PeerManager::setNewConnection( Network::TCPSocket* socket ) {
	if ( !socket ) return false;

	pthread_mutex_lock( &mutex );
	newSocket = socket;
	newSocketFlag = true;
	pthread_mutex_unlock( &mutex );
	return true;
}

}} // namespaces
