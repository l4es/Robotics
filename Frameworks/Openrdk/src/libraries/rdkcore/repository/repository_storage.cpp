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


#include "repository.h"
#include "events.h"

#include <rdkcore/repository_struct/rpropertyupdate.h>
#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/object/objectdiff.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Repository"

#include <stack>

namespace RDK2 { namespace RepositoryNS {

using RDK2::RNetObjects::RNetMessage;
using namespace RDK2::Meta;

void Repository::createStorage(cstr className, CUrl url, cstr description, RDK2::Object* defaultObject) throw (InvalidOperation)
{
	if (className == QUEUE_CLASSNAME)
		throw InvalidOperation("You cannot use createStorage() to create a queue property");
	addGenericProperty(className, url, description, defaultObject);
}

void Repository::valueChanged(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	vector<ObjectDiff*> noDiff;
	valueChanged(url, noDiff);
}

void Repository::valueChanged(CUrl url, const vector<ObjectDiff*>& diffs) throw (NoSuchProperty, InvalidOperation)
{
	if (isLocalProperty(url)) {
		for (size_t i = 0; i < diffs.size(); i++) {
			pushDiffInDiffQueue(url, (ObjectDiff*) diffs[i]->clone());
		}
	}

	if (isLocalProperty(url)) {
		if (diffs.size() == 0) wakeUpListeningSessions(url, "EventPropertyUpdateValue");
		else wakeUpListeningSessions(url, "EventPropertyUpdateDiff");
	}

	if (!isLocalProperty(url)) {
		if (url.getProtocol() == "file") {
			RDK_DEBUG_PRINTF("*** found a 'file' protocol valuechanged");
		}
		else if (url.getProtocol() == "rdk") {
			sendStorageValueToHost(url, url.getHost(), Network::TCP_IP /*FIXME*/);
		}
	}

	// send to remote hosts
	Url completeUrl = "rdk://" + getRepositoryName() + "/" + url;

	// send object to all remote repository that subscribed VALUE+ON_CHANGE
	sendStorageValuesToRemoteValueSubscribers(completeUrl, "",
		Network::NETPROTOCOL_ANY, RRemoteSubscription::STORAGE_VALUE, RRemoteSubscription::ON_CHANGE);

	if (diffs.size() == 0) {
		// send object to all remote repository that subscribed DIFFS+ON_CHANGE
		sendStorageValuesToRemoteValueSubscribers(completeUrl, "",
			Network::NETPROTOCOL_ANY, RRemoteSubscription::STORAGE_DIFFS, RRemoteSubscription::ON_CHANGE);
	}
	else {
		//RDK_DEBUG_PRINTF("Sending %u diffs for property '%s'", diffs.size(), url.c_str());
		// send diffs to all remote repository that subscribed DIFFS+ON_CHANGE
		RemoteSubscriptionRegister* reg = &remoteSubscriptionRegister;
		vector<RRemoteSubscription> psendSub =
			reg->queryForPropertiesToSend(completeUrl, "", Network::NETPROTOCOL_ANY,
			RRemoteSubscription::STORAGE_DIFFS, RRemoteSubscription::ON_CHANGE);
		for (vector<RRemoteSubscription>::iterator it = psendSub.begin(); it != psendSub.end(); ++it) {
			for (size_t i = 0; i < diffs.size(); i++) {
				sendStorageDiffToHost(it->completeUrl, it->askingHost, it->socketType,
					(ObjectDiff*) diffs[i]->clone());
			}
		}
	}
}

void Repository::wakeUpListeningSessions(CUrl url, const string& eventType) throw (NoSuchProperty)
{
	stack<Url> s;
	s.push(url);

	set<Url> visited;
	while (!s.empty()) {
		// take the top element of the stack
		Url u = s.top(); s.pop();

		// make the current url visited
		visited.insert(u);

		// lock the property
		lock(u, HERE);
		
		// get the property
		Property* p = getProperty(u, 0);

		// wake up sessions listening to this property
		if (p->listeningSessions.size() > 0)
		for (set<Session*>::iterator its = p->listeningSessions.begin(); its != p->listeningSessions.end(); ++its) {
			if (eventType == "EventPropertyUpdateDiff") (*its)->pushEvent(new EventPropertyUpdateDiff(u));
			else if (eventType == "EventPropertyUpdateValue") (*its)->pushEvent(new EventPropertyUpdateValue(u));
			else {
				RDK_ERROR_PRINTF("Unknown event '%s'", eventType.c_str());
				//(*its)->pushEvent(new EventPropertyUpdate(u));
			}
			// NOTE: wakeUp is called by pushEvent
		}

		// put reversed linked properties in the stack
		for (set<Url>::iterator itl = p->linkedBy.begin(); itl != p->linkedBy.end(); ++itl) {
			if (visited.find(*itl) == visited.end()) s.push(*itl);
		}

		// put linked property in the stack	
		if (p->linkTo != "" && visited.find(p->linkTo) == visited.end()) s.push(p->linkTo);
	
		// release the property
		unlock(u);
	}
}

// FIXME dentro session
void Repository::setObject(CUrl url, RDK2::Object* object) throw (NoSuchProperty, InvalidOperation, WrongType)
{
	lock(url, HERE);
	Property* p = getProperty(url);
	if (!p->isStorage()) {
		// FIXME
		throw InvalidOperation("Property '" + url + "' is not a storage");
	}
	p->setObjectL(object);
	unlock(url);
}

bool Repository::pushDiffInDiffQueue(CUrl url, ObjectDiff* diff) throw (NoSuchProperty, InvalidOperation)
{
	bool b = false;
	// FIXME unlock prima del trock
	lock(url, HERE);
	Property* p = getProperty(url);
	if (!p->isStorage()) throw InvalidOperation("Property '" + url + "' is not a storage");
	b = p->diffsQueue.push(diff, HERE);
	p->diffsQueue.gc(HERE);
	unlock(url);
	return b;
}

}} // namespaces
