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
#include "session.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Session"

namespace RDK2 { namespace RepositoryNS {

void Session::storageSubscribeDiffs(cstr aurl) 
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url url = getAbsoluteUrl(aurl);
	map<Url, QueueSubscription>::iterator it = diffsSubscriptions.find(url);
	if (it == diffsSubscriptions.end()) {
		repository->lock(url, HERE);
		Property* p = repository->getProperty(url);
		RObjectPipeQueue* queue = &p->diffsQueue;
		//diffsSubscriptions[url].queue = queue;
		RConsumerObjectPipeQueue* consumerQueue =
			queue->createConsumer(url + "(DIFFS)[" + sessionName + "]", HERE);
		queue->setSignalSemaphore(consumerQueue, &sessionSemaphore);
		diffsSubscriptions[url].consumerQueue = consumerQueue;
		diffsSubscriptions[url].mode = QUEUE_ALL;
		repository->unlock(url);
	}
}

void Session::storageUnsubscribeDiffs(cstr aurl) 
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url url = getAbsoluteUrl(aurl);
	map<Url, QueueSubscription>::iterator it = diffsSubscriptions.find(url);
	if (it != diffsSubscriptions.end()) {
		repository->lock(url, HERE);
		Property* p = repository->getProperty(url);
		RObjectPipeQueue* queue = &p->diffsQueue;
		queue->deleteConsumer(it->second.consumerQueue, HERE);
		repository->unlock(url);
	}
}

RConsumerObjectPipeQueue* Session::getDiffsConsumerQueue(cstr url)
{
	map<Url, QueueSubscription>::iterator it = diffsSubscriptions.find(url);
	if (it != diffsSubscriptions.end()) {
		return it->second.consumerQueue;
	} else {
		RDK_DEBUG_PRINTF("Subscribing diffs queue '%s'", url.c_str());
		storageSubscribeDiffs(url);
		return getDiffsConsumerQueue(url);	// FIXME are we sure this always succeed?
	}
}

const vector<const RDK2::Object*> Session::diffsFreeze(cstr url)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	return getDiffsConsumerQueue(getAbsoluteUrl(url))->freeze();
}

}} // namespace
