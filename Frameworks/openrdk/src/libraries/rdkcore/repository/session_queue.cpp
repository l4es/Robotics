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

#include <algorithm>
#include <sstream>
#include "repository.h"
#include "session.h"

#include <rdkcore/profiling/profiling.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Session"

namespace RDK2 { namespace RepositoryNS {

using namespace std;
using namespace RDK2::Profiling;

void Session::queueCreate(CUrl url, cstr description)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	CUrl aurl = getAbsoluteUrl(url);
	repository->addGenericProperty(QUEUE_CLASSNAME, aurl, description);
	ownedPropertiesMutex.lock(HERE);
	ownedProperties.insert(aurl);
	ownedPropertiesMutex.unlock();
	Property* p = lockAndGetProperty(url);
	p->setObject(new RObjectPipeQueue(aurl));
	//p->setPersistent(false);	// FIXME can be not persistent only
		// when the links are always persistent
	unlockProperty(p);
}

bool Session::queuePush(CUrl url, RDK2::Object* object) throw (NoSuchProperty, InvalidOperation)
{
	Property* p = lockAndGetProperty(url);

	bool res = false;
	if (p->getObjectClassName() == "") p->setObjectClassName(QUEUE_CLASSNAME);	// XXX are we sure?
	if (!p->isQueue()) throw InvalidOperation("Property '" + url + "' is not a queue");
	RObjectPipeQueue* queue = dynamic_cast<RObjectPipeQueue*>(getProperty(url)->object);
	if (queue) {
		res = queue->push(object, HERE);
		queue->gc(HERE);
	}
	if (url.isComplete() && url.getHost() != getRepositoryName()) {
		RDK_DEBUG_PRINTF("Sending object to the remote queue '%s'", p->url.c_str());
		repository->sendObjectsToRemoteQueueSubscribers(getAbsoluteUrl(url), object);
	}
	// NOTE: sessions listening to this queue are awaken by the queue itself

	unlockProperty(p);
	return res;
}

void Session::queueSubscribe(CUrl url, QueueSubscriptionMode qsm)
	throw (NoSuchProperty, InvalidOperation)
{
	// this will subscribe the queue if it is not
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, true, qsm);
	// this is needed if the subscription needs to be changed
	it->second.mode = qsm;
}

void Session::queueUnsubscribe(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, false);
	if (it != queueSubscriptions.end()) {
		Property* p = lockAndGetProperty(url);
		RObjectPipeQueue* queue = p->getObjectAsL<RObjectPipeQueue>();
		queue->deleteConsumer(it->second.consumerQueue, HERE);
		unlockProperty(p);
		queueSubscriptions.erase(it);
	}
}

void Session::queueSetInterests(CUrl url, PosixQueues::MyInterests<RDK2::Object>* i)
	throw (NoSuchProperty, InvalidOperation)
{
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, true, QUEUE_ALL);
	Property* p = lockAndGetProperty(url);
	RObjectPipeQueue* queue = p->getObjectAsL<RObjectPipeQueue>();
	queue->setInterests(it->second.consumerQueue, i);
	unlockProperty(p);
}

vector<const RDK2::Object*> Session::queueFreeze(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, true, QUEUE_ALL);
	RConsumerObjectPipeQueue* cq = it->second.consumerQueue;
	vector<const RDK2::Object*> r = cq->freeze();
/*	RDK_DEBUG_PRINTF("Freezing something in '%s' (%s, %d, %p)", url.c_str(), sessionName.c_str(),
		r.size(), cq);*/
	// FIXME non tiene conto degli eventuali keep
	if (r.size() && it->second.isListened) {
		for (size_t i = 0; i < r.size() - 1; i++) {
			sessionSemaphore.wait();
		}
	}
	ostringstream oss;
	oss << r.size();
	Profiler::addCustomLine("QUEUE_FREEZE", (sessionName + ":" + url).c_str(), oss.str().c_str());
	return r;
}

vector<const RDK2::Object*> Session::queueContent(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, true, QUEUE_ALL);
	return it->second.consumerQueue->content();
}

void Session::queueKeep(CUrl url, const RDK2::Object* object)
	throw (NoSuchProperty, InvalidOperation)
{
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, true, QUEUE_ALL);
	RConsumerObjectPipeQueue* cop = it->second.consumerQueue;
	vector<const RDK2::Object*> content = cop->content();

	if (content.end() == find(content.begin(), content.end(), object)) {
		throw InvalidOperation("Keep()ing a pointer not present in queue");
	}
	else {
		cop->keep(object);
	}
}

const RDK2::Object* Session::queueLast(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, true, QUEUE_LAST_OBJECT);
	return it->second.consumerQueue->last();
}

size_t Session::queueSize(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	Property* p = lockAndGetProperty(url);

	size_t queueSize = 0;
	if (p->getObjectClassName() == "") p->setObjectClassName(QUEUE_CLASSNAME);	// XXX are we sure?
	if (!p->isQueue()) throw InvalidOperation("Property '" + url + "' is not a queue");
	RObjectPipeQueue* queue = dynamic_cast<RObjectPipeQueue*>(getProperty(url)->object);
	if (queue) {
		queueSize = queue->size();
	}

	unlockProperty(p);
	return queueSize;
}

/** HERE STARTS SOME UTILITY FUNCTIONS **/

// protected
map<Url, Session::QueueSubscription>::iterator
Session::queueGetSubscription(CUrl url, bool alsoCreate, QueueSubscriptionMode qsm)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url aurl = getAbsoluteUrl(url);
	map<Url, QueueSubscription>::iterator it = queueSubscriptions.find(aurl);
	if (it == queueSubscriptions.end() && alsoCreate) {
		// the queue is not subscribed yet by this session
		Property* p = lockAndGetProperty(aurl);
		RObjectPipeQueue* queue = p->getObjectAsL<RObjectPipeQueue>();
		RConsumerObjectPipeQueue* consumerQueue =
			queue->createConsumer(aurl + "[" + sessionName + "]", HERE);
		queue->setSignalSemaphore(consumerQueue, 0);
		unlockProperty(p);
		QueueSubscription qs;
		qs.consumerQueue = consumerQueue;
		qs.mode = qsm;
		qs.isListened = false;
		pair<map<Url, QueueSubscription>::iterator, bool> insret =
			queueSubscriptions.insert(make_pair(aurl, qs));
/*		RDK_DEBUG_PRINTF("Created subscription of queue '%s' for "
			"session '%s' %p", aurl.c_str(), sessionName.c_str(), this);*/
		return insret.first;
	}
	return it;
}

// protected
void Session::listenToQueue(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	// listen implies subscribe
	// if the queue isn't subscribed, this will subscribe it (with QUEUE_ALL option)
	// it this is already subscribed, the subscription will not changed
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, true, QUEUE_ALL);
	Property* p = lockAndGetProperty(url);
	RObjectPipeQueue* queue = p->getObjectAsL<RObjectPipeQueue>();
	queue->setSignalSemaphore(it->second.consumerQueue, &sessionSemaphore);
	it->second.isListened = true;
	unlockProperty(p);
}

// protected
void Session::unlistenToQueue(CUrl url)
	throw (NoSuchProperty, InvalidOperation)
{
	map<Url, QueueSubscription>::iterator it = queueGetSubscription(url, false);
	if (it != queueSubscriptions.end()) {
		Property* p = lockAndGetProperty(url);
		RObjectPipeQueue* queue = p->getObjectAsL<RObjectPipeQueue>();
		queue->setSignalSemaphore(it->second.consumerQueue, 0);
		it->second.isListened = false;
		unlockProperty(p);
	}
}

}} // namespace
