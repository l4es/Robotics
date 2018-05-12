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
#define LOGGING_MODULE "Session"

#include "repository.h"
#include "session.h"

namespace RDK2 { namespace RepositoryNS {

void Session::listen(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	bool isQueue = false;
	Property* p = lockAndGetProperty(url, false);
		p->listeningSessions.insert(this);
		isQueue = p->isQueue();
	unlockProperty(p);
	if (isQueue) listenToQueue(url);
}

void Session::unlisten(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	bool isQueue = false;
	Property* p = lockAndGetProperty(url, false);
		p->listeningSessions.erase(this);
		isQueue = p->isQueue();
	unlockProperty(p);
	if (isQueue) unlistenToQueue(url);
}

void Session::listenToTreeChanges(cstr prefix) throw (InvalidOperation)
{
	checkStarted();
	repository->sessionsListeningToTreeMutex.lock(HERE);
/*		RDK_ERROR_PRINTF("Adding '%s' '%s'", prefix.c_str(), sessionName.c_str());*/
		repository->sessionsListeningToTree.insert(make_pair(prefix, this));
	repository->sessionsListeningToTreeMutex.unlock();
}

// FIXME
// void Session:unlistenToTreeChanges(CUrl prefix)
// {
// 	repository->propertiesMutex
// }

void Session::listenToTimer(double ms) throw (InvalidOperation)
{
	repository->sessionsListeningToTimerMutex.lock(HERE);
		repository->sessionsListeningToTimer[this].interval = ms;
		repository->sessionsListeningToTimer[this].timer.start();
	repository->sessionsListeningToTimerMutex.unlock();
}

// void Session::unlistenToTimer()
// {
// 	repository->sessionsListeningToTimerMutex.lock(HERE);	
// 	for (map<Session*, TimerListener>::iterator it = sessionsListeningToTimer.begin();
// 	it != sessionsListeningToTimer.end(); ++it) {
// 		if (it->first == this) {
// 			sessionsListeningToTimer.erase(it);
// 			break;
// 		}
// 	}
// 	repository->sessionsListeningToTimerMutex.unlock();
// }

void Session::wait() throw()
{
	bool isEnabled = true;
	do {
		sessionSemaphore.wait();

		isEnabled = true;
		try { isEnabled = getBool("enabled"); } catch (...) { }	// FIXME "enabled"
		
		if (exiting) isEnabled = true;
		isEnabled = true;	// FIXME	
		if (!isEnabled) {
			// empty the event queue
			currentEvents = consumerEventQueue->freeze();
			RDK_DEBUG_PRINTF("Emptying events queue of session '%s' because module is disabled", sessionName.c_str());
			// empty all subscripted queues
			for (map<Url, QueueSubscription>::iterator it = queueSubscriptions.begin();
			it != queueSubscriptions.end(); ++it) {
				RDK_DEBUG_PRINTF("Emptying queue '%s:%s' because module is disabled", sessionName.c_str(), it->first.c_str());
				it->second.consumerQueue->freeze();
			}
			
			// empty all subscripted diff queues
			for (map<Url, QueueSubscription>::iterator it = diffsSubscriptions.begin();
			it != diffsSubscriptions.end(); ++it) {
				RDK_DEBUG_PRINTF("Emptying diff queue of '%s:%s' because module is disabled", sessionName.c_str(), it->first.c_str());
				it->second.consumerQueue->freeze();
			}
				
			// FIXME nella sottoscrizione il modulo pu� indicare che non vuole che la coda venga
			// svuotata in questo caso
			// FIXME fare tanti wait per evitare di essere svegliati inutilmente
		}
	} while (!isEnabled);

	currentEvents = consumerEventQueue->freeze();

	if (currentEvents.size() > 0) {
		for (size_t i = 0; i < currentEvents.size()-1; i++) sessionSemaphore.wait();
	}
}

void Session::dontWait() throw ()
{
	// FIXME manage "enabled" stuff
	currentEvents = consumerEventQueue->freeze();
	if (currentEvents.size() > 0) {
		for (size_t i = 0; i < currentEvents.size()-1; i++) sessionSemaphore.wait();
	}
}

void Session::pushEvent(Event* event) throw ()
{
	eventQueue->push(event, HERE);
	eventQueue->gc(HERE);
	wakeUp();
}

vector<const Event*> Session::events() throw (InvalidOperation)
{
	checkStarted();
	return currentEvents;
}

bool Session::wokenByEvent(cstr eventName) throw (InvalidOperation)
{
	checkStarted();
	for (size_t i = 0; i < currentEvents.size(); i++) {
		if (currentEvents[i]->type == eventName) return true;
	}
	return false;
}

void Session::wakeUp(int times) throw ()
{
	for (int i = 0; i < times; i++) sessionSemaphore.signal();
}

void Session::registerTimerEventHandler(const Session::SessionEventHandler& handler) throw ()
{
	checkStarted();
	timerEventHandlers.push_back(handler);
}

/**
 * \warning Do not call this method within an EventHandler!
 */

bool Session::unregisterTimerEventHandler(const Session::SessionEventHandler& handler) throw ()
{
	checkStarted();
	bool found=false;
	for (list<SessionEventHandler>::iterator it = timerEventHandlers.begin();
				!found && it != timerEventHandlers.end();
				++it)
	{
		if (it->observer == handler.observer)
		{
			timerEventHandlers.erase(it);
			found = true;
		}
	}
	return found;
}

void Session::registerPropertyUpdateEventHandler(const Session::SessionEventHandler& handler, const Url& prefix) throw ()
{
	checkStarted();
	propertyUpdateEventHandlers.push_back(make_pair(prefix.contextualize(urlContext), handler));
}

/**
 * \warning Do not call this method within an EventHandler!
 */

void Session::unregisterPropertyUpdateEventHandler(const Session::SessionEventHandler& handler, const Url& prefix) throw ()
{
	checkStarted();
	bool found=false;
	for (list<pair<Url, SessionEventHandler> >::iterator it = propertyUpdateEventHandlers.begin();
				!found && it != propertyUpdateEventHandlers.end();
				++it)
	{
		if (it->first == prefix.contextualize(urlContext) && it->second.observer == handler.observer)
		{
			propertyUpdateEventHandlers.erase(it);
			found = true;
		}
	}
}

void Session::registerPropertyTreeEventHandler(const Session::SessionEventHandler& handler, const Url& prefix) throw ()
{
	checkStarted();
	propertyTreeEventHandlers.push_back(make_pair(prefix.contextualize(urlContext), handler));
}

void Session::processEvents() throw (SessionException)
{
	checkStarted();
	for (size_t i = 0; i < currentEvents.size(); i++) {
		const Event& e = *currentEvents[i];
		if (e.instanceOf("EventTimer")) {
			for (list<SessionEventHandler>::iterator it = timerEventHandlers.begin();
			it != timerEventHandlers.end(); ++it) {
				(*(it->fnptr))(it->observer, currentEvents[i]);
			}
		}
		else if (e.instanceOf("EventProperty")) {
			const EventProperty* ep = dynamic_cast<const EventProperty*>(currentEvents[i]);
			if (ep) {
				Url purl = ep->propertyUrl;
				//RDK_INFO_STREAM(purl);
				purl = purl.decontextualize("rdk://" + getRepositoryName());
				//RDK_INFO_STREAM(purl);
				if (e.instanceOf("EventPropertyUpdate")) {
					for (list<pair<Url, SessionEventHandler> >::iterator it = propertyUpdateEventHandlers.begin();
					it != propertyUpdateEventHandlers.end(); ++it) {
						if (purl.substr(0, it->first.size()) == it->first) {
							(*(it->second.fnptr))(it->second.observer, currentEvents[i]);
						}
					}
				}
				else if (e.instanceOf("EventPropertyTree")) {
					for (list<pair<Url, SessionEventHandler> >::iterator it = propertyTreeEventHandlers.begin();
					it != propertyTreeEventHandlers.end(); ++it) {
						if (purl.substr(0, it->first.size()) == it->first) {
							(*(it->second.fnptr))(it->second.observer, currentEvents[i]);
						}
					}
				}
			}
			else {
				RDK_ERROR_PRINTF("Cannot dynamic cast an object of class '%s' to class 'EventProperty'", e.type.c_str());
			}
		}
	}
}

}} // namespace
