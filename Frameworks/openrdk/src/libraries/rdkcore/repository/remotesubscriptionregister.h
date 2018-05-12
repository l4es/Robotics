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

#ifndef RDK2_REPOSITORY_REMOTESUBSCRIPTIONREGISTER
#define RDK2_REPOSITORY_REMOTESUBSCRIPTIONREGISTER

#include <rdkcore/posixconstructs/posixmutex.h>
#include <rdkcore/repository_struct/rremotesubscription.h>

#include <map>

namespace RDK2 { namespace RepositoryNS {

class Repository;
class Session;
using namespace std;

class RemoteSubscriptionRegister {
public:
	RemoteSubscriptionRegister(Repository* repository) : counter(1), repository(repository) { }
	~RemoteSubscriptionRegister() { }

	/// Query for reading the register
	vector<RRemoteSubscription> queryForPropertiesToSend(Url completeUrl, string askingHost, Network::NetProtocol,
		RRemoteSubscription::What, RRemoteSubscription::When);
	vector<RRemoteSubscription> queryForPropertiesToReceive(Url completeUrl, string askingHost, Network::NetProtocol,
		RRemoteSubscription::What, RRemoteSubscription::When);

	/// Deletes a subscription for a property to send
	/// @param id the id of the subscription to delete
	void deleteSubscriptionToSend(uint id);

	/// @note register now owns the subscription object
	bool addPropertyToSendSubscription(RRemoteSubscription* sub);
	void addPropertyToReceiveSubscription(Session* askingSession, RRemoteSubscription* sub);
	void completePropertyToReceiveSubscription(RRemoteSubscription* sub);

	/// Deletes all subscription asked by the session
	/// @param session the session whose subscriptions have to be deleted
	void deleteAllSubscriptionsOfSession(Session* session);

	void refreshLastSendingTimestamp(uint id);

private:
	static bool isIncompleteSub(const RRemoteSubscription* sub);
	bool completeSub(RRemoteSubscription* sub);
	void refreshTimestampL(uint id);
	PosixConstructs::PosixMutex myMutex;
		list<RRemoteSubscription*> propertiesToSend;
		multimap<Session*, RRemoteSubscription*> propertiesToReceive;
	volatile uint counter;	// counter is put in remotesubscription by the add functions as an id
	Repository* repository;
};

}} // namespaces

#endif
