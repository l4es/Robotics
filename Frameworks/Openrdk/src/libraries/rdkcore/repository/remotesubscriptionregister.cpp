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

#include <rdkcore/repository_struct/rremotesubscription.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RemoteSubscriptionRegister"

#include "remotesubscriptionregister.h"
#include "repository.h"
#include "session.h"

// #define DEBUG_THIS
#ifdef DEBUG_THIS
#warning a lot of debug
#define DOUT(a, args...) RDK_DEBUG_PRINTF(a, ## args)
#else
#define DOUT(a, args...)
#endif

namespace RDK2 { namespace RepositoryNS {

using RDK2::RNetObjects::RNetMessage;

bool RemoteSubscriptionRegister::isIncompleteSub(const RRemoteSubscription* sub)
{
	return (sub->socketType == Network::NETPROTOCOL_UNKNOWN
		|| sub->what == RRemoteSubscription::WHAT_UNKNOWN || sub->when == RRemoteSubscription::WHEN_UNKNOWN
		|| (sub->when == RRemoteSubscription::PERIODIC && sub->maxPeriod == 0.)
		|| (sub->when == RRemoteSubscription::ON_CHANGE && sub->minPeriod == 0.));
}

bool RemoteSubscriptionRegister::completeSub(RRemoteSubscription* sub)
{
	bool wasIncomplete = isIncompleteSub(sub);
	bool isVector = false;
	Property* p = repository->getLocalProperty(sub->completeUrl);
	p->lock(HERE);
	string propertyClassName = p->getObjectClassName();
	isVector = (propertyClassName.find("Vector") != string::npos);
	RDK_DEBUG_PRINTF("PROPERTY CLASS = '%s'", propertyClassName.c_str());
	p->unlock();
	RDK_DEBUG_PRINTF("COMPLETING SUBSCRIPTION: '%s'", sub->getStringForVisualization().c_str());
	if (sub->socketType == Network::NETPROTOCOL_UNKNOWN) {
		if (propertyClassName == "RImage" || propertyClassName == "RMapImage") sub->socketType = Network::TCP_IP;
		else sub->socketType = Network::UDP_IP;
	}
	if (sub->what == RRemoteSubscription::WHAT_UNKNOWN) {
		if (propertyClassName == "RImage" || propertyClassName == "RMapImage") sub->what = RRemoteSubscription::STORAGE_VALUE;
		if (sub->socketType == Network::UDP_IP) sub->what = RRemoteSubscription::STORAGE_VALUE;
		else sub->what = RRemoteSubscription::STORAGE_DIFFS;
	}
	if (sub->when == RRemoteSubscription::WHEN_UNKNOWN) {
		if (sub->socketType == Network::UDP_IP && sub->what == RRemoteSubscription::STORAGE_VALUE)
			sub->when = RRemoteSubscription::PERIODIC;
		else if (sub->socketType == Network::UDP_IP && sub->what == RRemoteSubscription::STORAGE_DIFFS)
			sub->when = RRemoteSubscription::ON_CHANGE;
		else if (sub->socketType == Network::TCP_IP && sub->what == RRemoteSubscription::STORAGE_VALUE)
			sub->when = RRemoteSubscription::ON_CHANGE;
		else if (sub->socketType == Network::TCP_IP && sub->what == RRemoteSubscription::STORAGE_DIFFS)
			sub->when = RRemoteSubscription::PERIODIC;
	}
	if (sub->when == RRemoteSubscription::PERIODIC && sub->maxPeriod == 0.) {
		if (propertyClassName == "RMapImage" || propertyClassName == "RImage") sub->maxPeriod = 2000.;	// XXX
		else sub->maxPeriod = 500.;
	}
	else if (sub->when == RRemoteSubscription::ON_CHANGE && (sub->maxPeriod == 0. || sub->minPeriod == 0.)) {
		if (propertyClassName == "RMapImage" || propertyClassName == "RImage") sub->maxPeriod = 2000.;	// XXX
		else sub->maxPeriod = 500.;
	}
	if (propertyClassName == QUEUE_CLASSNAME) {
		sub->when = RRemoteSubscription::ON_CHANGE;
		sub->what = RRemoteSubscription::QUEUE;
		sub->socketType = Network::TCP_IP;
	}
	if (propertyClassName == "RImage" || propertyClassName == "RMapImage") {
		sub->socketType = Network::TCP_IP;	// FIXME it's a hack
		sub->maxPeriod = 500.;
		sub->what = RRemoteSubscription::STORAGE_VALUE;
		sub->when = RRemoteSubscription::PERIODIC;
	}
	if (isVector) {
		sub->socketType = Network::TCP_IP;
		sub->maxPeriod = 1000.;
		sub->what = RRemoteSubscription::STORAGE_VALUE;
		sub->when = RRemoteSubscription::PERIODIC;
	}
	RDK_DEBUG_PRINTF("COMPLETED SUBSCRIPTION: '%s'", sub->getStringForVisualization().c_str());
	return wasIncomplete;
}

bool RemoteSubscriptionRegister::addPropertyToSendSubscription(RRemoteSubscription* sub)
{
	bool reallyAdded = false;
	if (sub->completeUrl.isComplete()) {
		if (isIncompleteSub(sub)) {
			DOUT("Received an incomplete subscription: [%s]", sub->getStringForVisualization().c_str());
			completeSub(sub);
			DOUT("...completed as follows: [%s]", sub->getStringForVisualization().c_str());
			repository->queuePush(PROPERTY_OUTBOX,
				new RNetMessage(sub->askingHost, repository->getRepositoryName(),
				RNetMessage::PROPERTY_SUBSCRIPTION, sub->socketType, sub->clone()));
		}
		myMutex.lock(HERE);
		// check if that subscription is already on the database
		vector<RRemoteSubscription> q = queryForPropertiesToSend(sub->completeUrl, sub->askingHost, sub->socketType,
			sub->what, sub->when);
		if (q.size() == 0) {
			// if the subscription was not in the database, we add it
			sub->id = counter++;
			gettimeofday(&sub->timestamp, 0);
			DOUT("Adding a new subscription for property to send: [%s]",
				sub->getStringForVisualization().c_str());
			propertiesToSend.push_back(sub);
			reallyAdded = true;
		}
		else {
			// if the subscription was already in the database, we refresh its timestamp (if it is UDP)
			for (vector<RRemoteSubscription>::iterator it = q.begin(); it != q.end(); ++it) {
				if (it->socketType == Network::UDP_IP) {
					DOUT("Refreshing a udp subscription: [%s]",
						(*it)->getStringForVisualization().c_str());
					refreshTimestampL(it->id);
				}
			}
			delete sub;
		}
		myMutex.unlock();
	}
	else {
		RDK_ERROR_PRINTF("Malformed subscription, cannot add [%s]", sub->getStringForVisualization().c_str());
		delete sub;
	}
	return reallyAdded;
}

void RemoteSubscriptionRegister::completePropertyToReceiveSubscription(RRemoteSubscription* sub)
{
	if (sub->completeUrl.isComplete()) {
		myMutex.lock(HERE);
		for (multimap<Session*, RRemoteSubscription*>::iterator it = propertiesToReceive.begin();
		it != propertiesToReceive.end(); ++it) {
			if (it->second->id == sub->id) {
				DOUT("Updating an incomplete subscription (from [%s] to [%s])",
					it->second->getStringForVisualization().c_str(),
					sub->getStringForVisualization().c_str());
				delete it->second;
				it->second = sub;
				break;
			}
		}
		myMutex.unlock();
	}
	else {
		RDK_ERROR_PRINTF("Malformed subscription, cannot add [%s]", sub->getStringForVisualization().c_str());
		delete sub;
	}
}

void RemoteSubscriptionRegister::refreshTimestampL(uint id)
{
	for (list<RRemoteSubscription*>::iterator it = propertiesToSend.begin(); it != propertiesToSend.end(); ++it) {
		if ((*it)->id == id) {
			gettimeofday(&(*it)->timestamp, 0);
			return;
		}
	}
}

void RemoteSubscriptionRegister::refreshLastSendingTimestamp(uint id)
{
	for (list<RRemoteSubscription*>::iterator it = propertiesToSend.begin(); it != propertiesToSend.end(); ++it) {
		if ((*it)->id == id) {
			(*it)->lastSendingTimestamp.setToNow();
			return;
		}
	}
}

void RemoteSubscriptionRegister::addPropertyToReceiveSubscription(Session* askingSession, RRemoteSubscription* sub)
{
	if (!askingSession) {
		delete sub;
		return;
	}

	RDK_DEBUG_PRINTF("Remote subscribing property '%s' by session %x (%s)", sub->completeUrl.c_str(),
		askingSession, askingSession->getSessionName().c_str());

	if (sub->completeUrl.isComplete()) {
		myMutex.lock(HERE);
		vector<RRemoteSubscription> q =
			queryForPropertiesToReceive(sub->completeUrl, sub->askingHost, sub->socketType, sub->what, sub->when);
		if (q.empty()) {
			sub->id = counter++;
			DOUT("New property to receive (subscription: [%s])", sub->getStringForVisualization().c_str());
			propertiesToReceive.insert(make_pair(askingSession, sub));
		}
		else delete sub;
		myMutex.unlock();
	}
	else {
		RDK_ERROR_PRINTF("Malformed subscription, cannot add [%s]", sub->getStringForVisualization().c_str());
	}
}

void RemoteSubscriptionRegister::deleteAllSubscriptionsOfSession(Session* session)
{
	myMutex.lock(HERE);
	bool somethingDeleted = false;
	do {
		somethingDeleted = false;
		for (multimap<Session*, RRemoteSubscription*>::iterator it = propertiesToReceive.begin();
		it != propertiesToReceive.end(); ++it) {
			if (it->first == session) {
				DOUT("Deleting subscription [%s] of session %x", it->second->getStringForVisualization(),
					it->first);
				delete it->second;
				propertiesToReceive.erase(it);
				somethingDeleted = true;
				break;
			}
		}
	} while (somethingDeleted);
	myMutex.unlock();
}

void RemoteSubscriptionRegister::deleteSubscriptionToSend(uint id)
{
	myMutex.lock(HERE);
	list<RRemoteSubscription*>& subs = propertiesToSend;
	for (list<RRemoteSubscription*>::iterator it = subs.begin(); it != subs.end(); ++it) {
		if (id == (*it)->id) {
			delete *it;
			subs.erase(it);
			break;
		}
	}
	myMutex.unlock();
}

vector<RRemoteSubscription> RemoteSubscriptionRegister::queryForPropertiesToSend(Url completeUrl,
	string askingHost, Network::NetProtocol socketType,
	RRemoteSubscription::What what, RRemoteSubscription::When when)
{
	vector<RRemoteSubscription> res;
	list<RRemoteSubscription*>& subs = propertiesToSend;
	for (list<RRemoteSubscription*>::iterator it = subs.begin(); it != subs.end(); ++it) {
		RRemoteSubscription* s = *it;
		bool select = true;
		if (askingHost != "" && askingHost != s->askingHost) select = false;
		if (socketType != Network::NETPROTOCOL_ANY && socketType != s->socketType) select = false;
		if (what != RRemoteSubscription::WHAT_ANY && what != s->what) select = false;
		if (when != RRemoteSubscription::WHEN_ANY && when != s->when) select = false;
		if (completeUrl != "" && completeUrl != s->completeUrl) select = false;
		if (select) res.push_back(*s);	// the default copy constructor does what it has to do
	}
	return res;
}

vector<RRemoteSubscription> RemoteSubscriptionRegister::queryForPropertiesToReceive(Url completeUrl,
	string askingHost, Network::NetProtocol socketType,
	RRemoteSubscription::What what, RRemoteSubscription::When when)
{
	vector<RRemoteSubscription> res;
	multimap<Session*, RRemoteSubscription*>& subs = propertiesToReceive;
	DOUT("Looking for %s, %s, %d %d %d", completeUrl.c_str(), askingHost.c_str(), socketType, what, when);
	for (multimap<Session*, RRemoteSubscription*>::iterator it = subs.begin(); it != subs.end(); ++it) {
		RRemoteSubscription* s = it->second;
		DOUT("Subscription %s", s->getStringForVisualization().c_str());
		bool select = true;
		if (askingHost != "" && askingHost != s->askingHost) select = false;
		if (socketType != Network::NETPROTOCOL_ANY && socketType != s->socketType) select = false;
		if (what != RRemoteSubscription::WHAT_ANY && what != s->what) select = false;
		if (when != RRemoteSubscription::WHEN_ANY && when != s->when) select = false;
		if (completeUrl != "" && completeUrl != s->completeUrl) select = false;
		if (select) res.push_back(*s);	// the default copy constructor does what it has to do
	}
	return res;
}

}} // namespaces
