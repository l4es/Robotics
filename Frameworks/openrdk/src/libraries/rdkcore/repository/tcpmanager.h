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

#ifndef RDK2_REPOSITORY_TCPMANAGER
#define RDK2_REPOSITORY_TCPMANAGER

#include <map>
#include <set>
#include <rdkcore/network/tcpsocket.h>
#include <rdkcore/network/tcpserversocket.h>
#include <rdkcore/posixconstructs/posixmutex.h>
//#include <rdkcore/posixqueues/pipequeue.h>
#include <rdkcore/posixqueues/interests.h>
#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/object/object.h>
#include <rdkcore/network/nettypes.h>

namespace RDK2 { namespace RepositoryNS {

class Repository;
class Session;
using namespace RDK2::Meta;
using RDK2::RNetObjects::RNetMessage;

class TcpManager : public PosixQueues::MyInterests<RDK2::Object> {
public:
	TcpManager(Repository* repository);
	~TcpManager();

	bool init();
	bool start();

	bool tryConnectTo(const string& peerName, time_t timeout = (time_t) 0);
	Network::ConnectionStatus getConnectionStatusWith(const string& peerName);

private:
	Repository* const repository;

	volatile bool exiting;

	static void* tcpListenerThreadFn(TcpManager* me) { me->tcpListener(); return 0; }
	void tcpListener();
	pthread_t tcpListenerThreadId;

	static void* tcpAutomaticPeerCreatorThreadFn(TcpManager* me)
		{ me->tcpAutomaticPeerCreator(); return 0; }
	void tcpAutomaticPeerCreator();
	pthread_t tcpAutomaticPeerCreatorThreadId;
	Session* tcpAutomaticPeerCreatorSession;
	bool areYouInterested(const RDK2::Object* obj);

	bool getAddress( const string& peerName, Network::InetAddress& address );

	class PeerManager;

	Network::InetAddress myAddr;

	std::map<std::string, PeerManager*> peerMap;
	pthread_mutex_t mutex;
	Network::TCPServerSocket serverSocket;
	std::set<std::string> invalidSet;

	void addUdpPort(const string& peerName,
		const Network::InetAddress& udpAddress, Session* session);

	class PeerManager : public PosixQueues::MyInterests<RDK2::Object> {
	public:
		PeerManager( const std::string& peerName, TcpManager& tcpManager,
			Network::TCPSocket* socket = NULL, time_t giveUpTime = (time_t)0 );
		virtual ~PeerManager();
		
		virtual bool areYouInterested( const RDK2::Object* object );
		
		inline Network::ConnectionStatus getStatus() { return status; }

	// Send receive thread
	private:
		static void* senderThreadFn( PeerManager* me ) { me->sender(); return NULL; }
		void sender( );

		static void* receiverThreadFn( PeerManager* me ) { me->receiver(); return NULL; }
		void receiver( );

	private:
		friend class TcpManager;
		bool setNewConnection( Network::TCPSocket* socket );
		bool connect( bool giveUpOnRefuse = true );

	private:
		const std::string	peerName;	///< Peer name
		TcpManager&		manager;	///< TCPManager pointer
		Network::TCPSocket*	socket;		///< TCP Socket

		pthread_t		receiverThreadId;	///< Receiver thread
		pthread_t		senderThreadId;	///< Sender thread
		
		volatile bool		exiting;	///< Volatile int, to terminate the threads
		volatile bool		stopSender;
	
		Session* senderSession;

		time_t			giveUpTime;	///< Time in seconds to give up the connection
		
		Network::ConnectionStatus	status;	///< Status of the connection
		
		pthread_mutex_t		mutex;	///< Mutex to set a new connection
		
		bool	newSocketFlag;		///< Flag to get a new connection
		Network::TCPSocket* newSocket;	///< The new connection to use
	};
};

}} // namespaces

#endif
