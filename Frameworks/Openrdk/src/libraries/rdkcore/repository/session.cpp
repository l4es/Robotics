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

#include <rdkcore/robot/robotmodule.h>	// FIXME
#include <rdkcore/profiling/profiling.h>
#include <rdkcore/object/objectdiff.h>
#include <rdkcore/repository_struct/rpropertydef.h>
#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Session"

#include <rdkcore/object/objectmanager.h>

//#define DEBUG_THIS
#ifdef DEBUG_THIS
#warning a lot of debug
#define DOUT(a, args...) RDK_DEBUG_PRINTF(a, ## args)
#else
#define DOUT(a, args...)
#endif

namespace RDK2 { namespace RepositoryNS {

using namespace RDK2::Profiling;

Session::Session(cstr sessionName, cstr description, cstr urlContext, Repository* repository) :
	threadIdInitialized(false), urlContext(urlContext), repository(repository),
	sessionName(sessionName), description(description),
	started(false), exiting(false)
{
	eventQueue = new PosixQueues::PipeQueue<Event>(string()
		+ "Event queue for session '" + sessionName + "'");
	consumerEventQueue = eventQueue->createConsumer(string() + "Consumer event queue for session '"
		+ sessionName + "'", HERE);
	//RDK_DEBUG_PRINTF("Session '%s': creating", this->sessionName.c_str());
}

Session::~Session()
{
	DOUT("Deleting session %x (%s)", this, sessionName.c_str());
	// delete the session from the listening sessions of each property
	repository->propertiesMutex.lock(HERE);
	for (map<Url, Property*>::iterator it = repository->localProperties.begin();
	it != repository->localProperties.end(); ++it) {
		set<Session*>::iterator fs = it->second->listeningSessions.find(this);
		if (fs != it->second->listeningSessions.end()) {
			DOUT("Session %x was listening to property %s", this, it->first.c_str());
			it->second->listeningSessions.erase(fs);
		}
	}
	for (map<Url, Property*>::iterator it = repository->remoteProperties.begin();
	it != repository->remoteProperties.end(); ++it) {
		set<Session*>::iterator fs = it->second->listeningSessions.find(this);
		if (fs != it->second->listeningSessions.end()) {
			DOUT("Session %x was listening to remote property %s", this, it->first.c_str());
			it->second->listeningSessions.erase(fs);
		}
	}
	repository->propertiesMutex.unlock();

	// delete the session from the sessions listening to tree changes
	// XXX non ho capito se dopo l'erase in una multimap gli iteratori rimangono validi
	bool somethingDeleted;
	repository->sessionsListeningToTreeMutex.lock(HERE);
	do {
		somethingDeleted = false;
		for (multimap<string, Session*>::iterator it = repository->sessionsListeningToTree.begin();
		it != repository->sessionsListeningToTree.end(); ++it) {
			if (it->second == this) {
				repository->sessionsListeningToTree.erase(it);
				somethingDeleted = true;
				break;
			}
		}
	} while (somethingDeleted);
	repository->sessionsListeningToTreeMutex.unlock();

	// delete the session from the sessions listening to a timer
	// XXX vedi sopra
	repository->sessionsListeningToTimerMutex.lock(HERE);
	do {
		somethingDeleted = false;
		for (map<Session*, Repository::SessionTimer>::iterator it
		= repository->sessionsListeningToTimer.begin();
		it != repository->sessionsListeningToTimer.end(); ++it) {
			if (it->first == this) {
				repository->sessionsListeningToTimer.erase(it);
				somethingDeleted = true;
				break;
			}
		}
	} while (somethingDeleted);
	repository->sessionsListeningToTimerMutex.unlock();

	// delete all remote subscriptions of this session
	RemoteSubscriptionRegister* subreg = &repository->remoteSubscriptionRegister;
	subreg->deleteAllSubscriptionsOfSession(this);

	// delete all owned properties
	ownedPropertiesMutex.lock(HERE);
	for (set<Url>::iterator it = ownedProperties.begin(); it != ownedProperties.end(); ++it) {
		repository->deleteProperty(getAbsoluteUrl(*it));
	}
	ownedProperties.clear();
	ownedPropertiesMutex.unlock();

	eventQueue->deleteConsumer(consumerEventQueue, HERE);
	delete eventQueue;
}

bool Session::isLocalProperty(CUrl url) const throw ()
{
		if (!url.isComplete()) return false;
		else return url.getHost() == repository->getRepositoryName();
}

void Session::start() throw (InvalidOperation)
{
	if (!threadIdInitialized) {
		threadId = pthread_self();
		//threadIdInitialized = true;	// FIXME occhio, questa va decommentata, ma ora non funziona
	}
	else if (!pthread_equal(threadId, pthread_self())) {
		throw InvalidOperation("Session '" + sessionName + "' has been started the first time by another thread");
	}

	if (started) throw InvalidOperation("Session '" + sessionName + "' already started");

	lockedProperties.clear();
	started = true;
	sessionActivityTimer.start();
	sessionCurrentScheduleInterval = sessionScheduleTimer.getSeconds();
	sessionScheduleTimer.start();
	ostringstream oss;
	oss << sessionName << "\tschedule:" << sessionCurrentScheduleInterval;
	Profiler::start(oss.str().c_str());
}

void Session::profilingStartEvent(cstr eventName) throw (InvalidOperation)
{
	checkStarted();
	Profiler::start((sessionName + ":" + eventName).c_str());
}

void Session::profilingEndEvent(cstr eventName) throw (InvalidOperation)
{
	checkStarted();
	Profiler::end((sessionName + ":" + eventName).c_str());
}

void Session::end() throw (InvalidOperation)
{
	checkStarted();

	if (!lockedProperties.empty()) {
		RDK_ERROR_PRINTF("STUPID USER! The following properties have not been unlocked before the session end (I do it for you):");

		for (set<string>::iterator it = lockedProperties.begin();
			it != lockedProperties.end(); ++it)
		{
			RDK_ERROR_PRINTF("   '%s'", it->c_str());
			unlock(*it);
		}
	}

	lockedProperties.clear();

	for (map<Url, vector<ObjectDiff*> >::iterator it = pendingDiffs.begin();
			it != pendingDiffs.end(); ++it)
		{
			for (vector<ObjectDiff*>::iterator it2 = it->second.begin();
				it2 != it->second.end(); ++it2)
			delete (*it2);

		it->second.clear();
	}

	double curAct = sessionActivityTimer.getSeconds();
	try {
		setInt(PROPERTY_MODULE_ACTIVITY_COUNTER, getInt(PROPERTY_MODULE_ACTIVITY_COUNTER) + 1);
		sessionActivityCache.push_back(curAct);
		if (sessionActivityCache.size() > 10) sessionActivityCache.erase(sessionActivityCache.begin());
		sessionScheduleCache.push_back(sessionCurrentScheduleInterval);
		if (sessionScheduleCache.size() > 10) sessionScheduleCache.erase(sessionScheduleCache.begin());
		setDouble(PROPERTY_MODULE_LAST_ITERATION_TIMER, curAct);
		double average = 0.;	// I don't want to use another var
		for (size_t i = 0; i < sessionActivityCache.size(); i++) average += sessionActivityCache[i];
		if (sessionActivityCache.size() > 0) {
			setDouble(PROPERTY_MODULE_ITERATION_DURATION_MEAN, average / sessionActivityCache.size());
		}
		average = 0.;
		for (size_t i = 0; i < sessionScheduleCache.size(); i++) average += sessionScheduleCache[i];
		if (sessionScheduleCache.size() > 0) {
			setDouble(PROPERTY_MODULE_SCHEDULE_INTERVAL_MEAN, average / sessionActivityCache.size());
		}
	}
	catch (const SessionException&) { }

	ostringstream poss;
	poss << sessionName << "\tduration:" << curAct;
	Profiler::end(poss.str().c_str());
	
	try {
		double minScheduleInterval = this->getDouble(PROPERTY_MODULE_SCHEDULE_MIN_INTERVAL);
		usleep(minScheduleInterval * 1e6);
	}
	catch (const SessionException&) { }

	started = false;
}

void Session::terminate() throw()
{
	if (!started) {
		RDK_ERROR_PRINTF("Calling terminate on a non-started session");
		started = true;
	}

	if (!lockedProperties.empty())
		for (set<string>::iterator it = lockedProperties.begin();
				it != lockedProperties.end(); ++it)
		{
			RDK_DEBUG_PRINTF("Unlocking '%s'", it->c_str());
			try {
				unlock(*it);
			} catch (const SessionException& e) {
				RDK_ERROR_STREAM("It seems that something is going wrong in unlocking '" << *it << "'");
			}
		}

	lockedProperties.clear();
	for (map<Url, vector<ObjectDiff*> >::iterator it = pendingDiffs.begin();
			it != pendingDiffs.end(); ++it)
	{
		for (vector<ObjectDiff*>::iterator it2 = it->second.begin();
			it2 != it->second.end(); ++it2)
				delete (*it2);

		it->second.clear();
	}

	try {
		setInt(PROPERTY_MODULE_ACTIVITY_COUNTER,
			getInt(PROPERTY_MODULE_ACTIVITY_COUNTER) + 1);

		setDouble(PROPERTY_MODULE_LAST_ITERATION_TIMER,
			sessionActivityTimer.getSeconds());
	} catch (const SessionException&) { }

	started = false;
	Profiler::end(sessionName.c_str());
}

void Session::createStorage(cstr className, cstr url,
	cstr description, RDK2::Object* defaultValue)
	throw (InvalidOperation)
{
	checkStarted();
	repository->createStorage(className, getAbsoluteUrl(url),
		description, defaultValue);
	ownedPropertiesMutex.lock(HERE);
	ownedProperties.insert(getAbsoluteUrl(url));
	ownedPropertiesMutex.unlock();
}

void Session::setReadonly(cstr url, bool readonly)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->setReadonly(readonly);
	unlockProperty(p);
}

void Session::setPersistent(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->def.setPersistent();
	unlockProperty(p);
}

void Session::setVolatile(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->def.setVolatile();
	unlockProperty(p);
}

void Session::setKeepThis(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->def.setKeepThis();
	unlockProperty(p);
}

void Session::setEditable(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->def.setEditable();
	unlockProperty(p);
}

void Session::setNotEditable(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->def.setNotEditable();
	unlockProperty(p);
}

void Session::setExternalFileName(CUrl url, cstr fileName) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->def.setExternalFileName(fileName);
	unlockProperty(p);
}

void Session::setExternalFileBinary(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->def.setExternalFileBinary();
	unlockProperty(p);
}

void Session::setExternalFileXml(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Property* p = lockAndGetProperty(url);
	p->def.setExternalFileXml();
	unlockProperty(p);
}

void Session::lock(cstr url, Stack s)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url aurl = getAbsoluteUrl(url);
	Profiler::lock((sessionName + ":" + aurl).c_str());
	repository->lock(aurl, s);
	lockedProperties.insert(aurl);
}

void Session::unlock(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url aurl = getAbsoluteUrl(url);
	repository->unlock(aurl);
	Profiler::unlock((sessionName + ":" + aurl).c_str());

	// strustr said that v.erase(v.end()) is ok,
	// but at least my std implementation doesn't know it
	set<string>::iterator it = lockedProperties.find(aurl);

	if (it != lockedProperties.end())
		lockedProperties.erase(lockedProperties.find(aurl));
}

const RDK2::Object* Session::getObjectLC(cstr url)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	return getProperty(getAbsoluteUrl(url))->getObjectLC();
}

RDK2::Object* Session::getObjectClone(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url real = getAbsoluteUrl(url);
	Property* p = lockAndGetProperty(url);
	RDK2::Object* ob = p->getObjectL();
	if(ob) {
		RDK2::Object* clone = ob->clone();
		repository->unlock(real);
		Profiler::unlock((sessionName + ":" + url).c_str());
		return clone;
	} else {
		repository->unlock(real);
		Profiler::unlock((sessionName + ":" + url).c_str());
		return 0;
	}
}

bool Session::isSet(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	bool r = false;
	checkStarted();
	Property* p = lockAndGetProperty(url);
	r = (p->object != 0);
	unlockProperty(p);
	return r;
}

RDK2::Object* Session::getObjectL(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url aurl = getAbsoluteUrl(url);
	if (lockedProperties.find(aurl) == lockedProperties.end())
		throw InvalidOperation("You must lock property '" + aurl + "' before asking its object");
	return getProperty(aurl)->getObjectL();
}

string Session::getPropertyClassName(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	string classname;
	Property* p = getProperty(url, false);
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	classname = p->getObjectClassName();
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
	return classname;
}

string Session::getPropertyDescription(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	string desc;
	Property* p = getProperty(url, false);
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	desc = p->description;
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
	return desc;
}

vector<Url> Session::getPropertyNamesStartingWith(cstr prefix) throw ()
{
	vector<Url> ret;
	repository->propertiesMutex.lock(HERE);
	for (map<Url, Property*>::iterator it = repository->localProperties.begin();
		it != repository->localProperties.end(); ++it)
			if (RDK2::TextUtils::startsWith(it->first, prefix.c_str()))
				ret.push_back(it->first);

	for (map<Url, Property*>::iterator it = repository->remoteProperties.begin();
		it != repository->remoteProperties.end(); ++it)
			if (RDK2::TextUtils::startsWith(it->first, prefix.c_str()))
				ret.push_back(it->first);

	/* Ruby:
		ret << repository.localProperties.find {|name,ob| name.starts prefix}
	*/

	repository->propertiesMutex.unlock();
	return ret;
}

vector<PropertyEnumItem> Session::getPropertyEnumItems(cstr url)
	throw (NoSuchProperty, InvalidOperation)
{
	vector<PropertyEnumItem> enumItems;
	Property* p = getProperty(url);
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	enumItems = p->enumItems;
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
	return enumItems;
}

string Session::getPropertyLinkTo(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	string linkto;
	Property* p = getProperty(url, false);
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	linkto = p->isLink() ? p->linkTo : "";
	//RDK_DEBUG_PRINTF("Asking if link to property %s: %d '%s'", url.c_str(), p->isLink(), p->linkTo.c_str());
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
	return linkto;
}

RDK2::Object* Session::getObjectOrCreateItL(cstr url)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url aurl = getAbsoluteUrl(url);
	if (lockedProperties.find(aurl) == lockedProperties.end())
		throw InvalidOperation("You must lock property '"
			+ aurl + "' before asking its object");
	return repository->getObjectOrCreateItL(aurl);
}

void Session::deleteProperty(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	repository->deleteProperty(getAbsoluteUrl(url));
	ownedPropertiesMutex.lock(HERE);
	ownedProperties.erase(ownedProperties.find(getAbsoluteUrl(url)));
	ownedPropertiesMutex.unlock();
}

void Session::linkCreate(cstr first, cstr second) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();

	//printf("Creating link from '%s' to '%s'\n", first.c_str(), second.c_str());

	string oldLinkTo;

	Property* p = lockAndGetProperty(first, false);
	delete p->object;	// delete 0 is safe
	p->object = 0;
	p->isLinkB = (second != "");
	oldLinkTo = p->linkTo;
	p->linkTo = second;
	p->def.setLinkTo(second);
	unlockProperty(p);

	if (oldLinkTo != "") {
		p = lockAndGetProperty(oldLinkTo, false);
		p->linkedBy.erase(first);
		unlockProperty(p);
	}

	if (second != "") {
		p = lockAndGetProperty(second, false);
		//printf("Inserting '%s' as reverse link of '%s'\n", first.c_str(), second.c_str());
		p->linkedBy.insert(first);
		unlockProperty(p);
	}
}

void Session::linkRemove(cstr first) throw (NoSuchProperty, InvalidOperation)
{
	linkCreate(first, "");
}

Property* Session::getProperty(cstr url, bool followLinks)
	throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	Url aurl = getAbsoluteUrl(url);
	if (!aurl.isAbsolute() && !aurl.isComplete())
		throw InvalidOperation("Url has to be absolute or complete ('" + aurl + "')");
	Property* p = 0;
	repository->propertiesMutex.lock(HERE);
	for (int redirCount = 0; redirCount < (followLinks ? MAX_LINK_LEVELS : 1); redirCount++) {
		if (repository->isLocalProperty(aurl)) {
			if (aurl.isComplete()) aurl = aurl.getPath();
			map<Url, Property*>::iterator it = repository->localProperties.find(aurl);
			if (it == repository->localProperties.end()) {
				repository->propertiesMutex.unlock();
				throw NoSuchProperty("Property '" + (string) aurl + "' does not exists");
			}
			p = it->second;
			if (p->deleted) {
				repository->propertiesMutex.unlock();
				throw NoSuchProperty("Property '" + (string) aurl + "' has been deleted");
			}
			if (p->isLink()) aurl = p->linkTo;
			else {
				repository->propertiesMutex.unlock();
				return p;
			}
		}
		else {
			repository->propertiesMutex.unlock();
			Property* p = repository->getRemoteProperty(aurl, this);
			return p;
		}
	}
	repository->propertiesMutex.unlock();
	if (followLinks) throw InvalidOperation("Too many link levels");
	else {
		if (p) return p;
		else throw InvalidOperation("Something weird has happened");
	}
}

RPropertyDef* Session::getRPropertyDef(cstr url)
	throw (NoSuchProperty, InvalidOperation)
{
	Property* p = getProperty(url, false);
	RPropertyDef* pd;
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	pd = new RPropertyDef("rdk://" + getRepositoryName() + "/" + url, *p);
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
	return pd;
}

PropertyDef Session::getPropertyDef(cstr url)
	throw (NoSuchProperty, InvalidOperation)
{
	PropertyDef r;
	Property* p = lockAndGetProperty(url, false);
	r = p->getPropertyDef();
	unlockProperty(p);
	return r;
}

PropertyDef Session::getDefaultPropertyDef(cstr url)
	throw (NoSuchProperty, InvalidOperation)
{
	PropertyDef r;
	Property* p = lockAndGetProperty(url, false);
	r = p->getDefaultPropertyDef();
	unlockProperty(p);
	return r;
}

bool Session::isPersistent(cstr url) throw (InvalidOperation, NoSuchProperty)
{
	return getProperty(url)->isPersistent();
}

bool Session::isReadonly(cstr url) throw (InvalidOperation, NoSuchProperty)
{
	return getProperty(url)->isReadonly();
}

void Session::setPropertyOptions(cstr url, PropertyOptions options)
	throw (NoSuchProperty, InvalidOperation)
{
	Property* p = getProperty(url);
	Profiler::lock((sessionName + ":" + url).c_str());
	p->lock(HERE);
	p->options = options;
	if (options & NOT_PERSISTENT) p->def.setVolatile();
	p->unlock();
	Profiler::unlock((sessionName + ":" + url).c_str());
}

void Session::setObject(cstr url, RDK2::Object* object)
	throw (NoSuchProperty, InvalidOperation, WrongType)
{
	checkStarted();
	Url aurl = getAbsoluteUrl(url);
	// FIXME usare la cache delle property
	repository->setObject(aurl, object);
	valueChanged(aurl);
}

void Session::declareDiffL(cstr url, ObjectDiff* diff)
	throw (NoSuchProperty, InvalidOperation)
{
	// XXX strictly this is not "L" because it does not need a lock/unlock, though it will be called in a lock/unlock
	// the diff will be actually pushed in the diffQueue when valueChanged is called
	checkStarted();
	pendingDiffs[getAbsoluteUrl(url)].push_back(diff);
}

void Session::valueChanged(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();

	lock(url, HERE);
	getObjectL(url)->updateTimestamp();
	unlock(url);

	Url aurl = getAbsoluteUrl(url);
	vector<ObjectDiff*>& diffs = pendingDiffs[aurl];
	//RDK_DEBUG_PRINTF("Value changed: %u diffs for property %s", diffs.size(), aurl.c_str());
	repository->valueChanged(aurl, diffs);
	for (size_t i = 0; i < diffs.size(); i++) delete diffs[i];
	diffs.clear();
}

void Session::remotePropertyOptions(
	cstr url, Network::NetProtocol socketType,
	RRemoteSubscription::What what,
	RRemoteSubscription::When when, double period)
		throw (NoSuchProperty, InvalidOperation)
{
	checkStarted();
	repository->remotePropertyOptions(
		getAbsoluteUrl(url), socketType, what, when, period);
}

void Session::storageSubscribeValue(cstr url) throw (NoSuchProperty, InvalidOperation)
{
	RDK_DEBUG_PRINTF("Storage subscribing %s", url.c_str());
	getProperty(url);	// this will automatically "subscribe" the value if the property is remote
}

void Session::storageUnsubscribeValue(cstr) throw (NoSuchProperty, InvalidOperation)
{
	// FIXME: vuoto?
}

string Session::textConfigLineForProperty(CUrl relativeUrl)
{
	// FIXME DC: che schifo
	string ret;
	Profiler::lock((sessionName + ":" + relativeUrl).c_str());
	lock(relativeUrl, HERE);
	string linkTo = getPropertyLinkTo(relativeUrl);
	RPropertyDef* rpd = getRPropertyDef(relativeUrl);
	if (linkTo != "") {
		// it is a link
		ret = relativeUrl + "=@" + linkTo;
	}
	else if (rpd->savedExternally) {
		ret = relativeUrl + "=$" + rpd->externalFilename;
	}
	else {
		// it is neither a link nor it has to be saved externally
		const RDK2::Object* obj = getObjectLC(relativeUrl);
		if (!obj) {
			ret = relativeUrl + "=";
		}
		else if (obj->hasStringRepresentation()) {
			ret = relativeUrl + "=" + obj->getStringRepresentation();
		}
		else {
			ret = "#ERROR, the object " + relativeUrl + " does not have a string representation";
		}
	}
	unlock(relativeUrl);
	Profiler::unlock((sessionName + ":" + relativeUrl).c_str());
	return ret;
}

bool Session::propertyExists(cstr url) throw (InvalidOperation)
{
	Url aurl = getAbsoluteUrl(url);
	if (!aurl.isAbsolute() && !aurl.isComplete()) {
		throw InvalidOperation("Url has to be absolute or complete ('" + aurl + "')");
	}

	try {
		// throws NoSuchProperty (also if it has been deleted)
		repository->getProperty(aurl, this);
	}
	catch (const NoSuchProperty& e) {
		return false;
	}
	catch (const SessionException& e) {
		throw(e);
	}
	return true;
}


}} // namespace
