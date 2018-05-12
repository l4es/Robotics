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

#ifndef RDK2_REPOSITORY_UDPMANAGER
#define RDK2_REPOSITORY_UDPMANAGER

#include <pthread.h>
#include <rdkcore/network/udpsocket.h>
#include <rdkcore/rnetobjects/rnetmessage.h>

namespace RDK2 { namespace RepositoryNS {

class Repository;
class Session;
using namespace RDK2::RNetObjects;

class UdpManager {
public:
	UdpManager(Repository* repository);
	~UdpManager();

	bool init();
	bool start();

private:
	Repository* const repository;

	volatile bool exiting;

	Network::UDPSocket udpSocket;

	static void* receiverThreadFn(UdpManager* me) { me->receiver(); return 0; }
	void receiver();
	pthread_t receiverThreadId;

	static void* senderThreadFn(UdpManager* me) { me->sender(); return 0; }
	void sender();
	pthread_t senderThreadId;
	Session* senderSession;
	
	Session* receiverSession;
	
	class UdpSenderInterests : public PosixQueues::MyInterests<RDK2::Object> {
	public:
		bool areYouInterested(const RDK2::Object* object) {
			const RNetObjects::RNetMessage* message;
			if ((message = dynamic_cast<const RNetObjects::RNetMessage*>(object)) &&
			(Network::UDP_IP == message->getNetProtocol()))
				return true;
			else
				return false;
		}
	};
};

}}

#endif
