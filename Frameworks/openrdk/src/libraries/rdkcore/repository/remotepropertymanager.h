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

#ifndef RDK2_REPOSITORY_REMOTEPROPERTYMANAGER
#define RDK2_REPOSITORY_REMOTEPROPERTYMANAGER

#include <pthread.h>
#include <rdkcore/rnetobjects/rnetmessage.h>

namespace RDK2 { namespace RepositoryNS {

class Repository;
class Session;
using RDK2::RNetObjects::RNetMessage;

class RemotePropertyManager {
public:
	RemotePropertyManager(Repository* repository);
	~RemotePropertyManager();

	bool init();
	bool start();

protected:
	Repository* const repository;

	volatile bool exiting;

	static void* receiverThreadFn(RemotePropertyManager* me) { me->receiver(); return 0; }
	void receiver();
	pthread_t receiverThreadId;
	Session* receiverSession;

	static void* periodicSenderThreadFn(RemotePropertyManager* me) { me->periodicSender(); return 0; }
	void periodicSender();
	pthread_t periodicSenderThreadId;
	Session* periodicSenderSession;

	static void* periodicUdpRenewalThreadFn(RemotePropertyManager* me) { me->periodicUdpRenewal(); return 0; }
	void periodicUdpRenewal();
	pthread_t periodicUdpRenewalThreadId;
	Session* periodicUdpRenewalSession;

	void manageUpdateMessage(const RNetMessage* msg) throw (SessionException);
	void manageSubscriptionMessage(const RNetMessage* msg) throw (SessionException);

	class RemotePropertyManagerInterests : public PosixQueues::MyInterests<RDK2::Object> {
	public:
		bool areYouInterested(const RDK2::Object* object) {
			const RNetObjects::RNetMessage* message;
			if ((message = dynamic_cast<const RNetObjects::RNetMessage*>(object))
			&& (RNetObjects::RNetMessage::PROPERTY_UPDATE == message->getType()
			|| RNetObjects::RNetMessage::PROPERTY_SUBSCRIPTION == message->getType()))
				return true;
			else
				return false;
		}
	};
};

}}

#endif
