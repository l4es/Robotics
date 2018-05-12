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

#include <fstream>

#include <rdkcore/serialization_xml/xmlreader.h>
#include <rdkcore/serialization_binary/binaryreader.h>
#include <rdkcore/rprimitive/rint.h>
#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/object/objectmanager.h>
#include <rdkcore/filesystem/filesystem.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RepositoryNS"

#include "repository.h"

namespace RDK2 { namespace RepositoryNS {

using namespace std;
using RDK2::RNetObjects::RYellowPages;
using namespace RDK2::Serialization::Xml;
using namespace RDK2::Serialization::Binary;
using namespace RDK2::Filesystem;

Repository::Repository(cstr repositoryName) :
	repositoryName(repositoryName), exitTimerThread(false), 
	timerThreadStarted(false), udpManager(0), tcpManager(0),
	remotePropertyManager(0),  remoteSubscriptionRegister(this)
{
	if (repositoryName == "") {
		RDK_ERROR_PRINTF("Cannot create an unnamed repository!");
		exit(-1);	// XXX
	}
	addGenericProperty(QUEUE_CLASSNAME, PROPERTY_OUTBOX, "Outbox");
	getLocalProperty(PROPERTY_OUTBOX)->setObject(new RObjectPipeQueue(PROPERTY_OUTBOX));
	getLocalProperty(PROPERTY_OUTBOX)->setReadonly(true);
	addGenericProperty(QUEUE_CLASSNAME, PROPERTY_INBOX, "Inbox");
	getLocalProperty(PROPERTY_INBOX)->setObject(new RObjectPipeQueue(PROPERTY_INBOX));
	getLocalProperty(PROPERTY_INBOX)->setReadonly(true);
	
	createStorage("RInt", PROPERTY_UDP_PORT, "My UDP port");
	getLocalProperty(PROPERTY_UDP_PORT)->setReadonly(true);
	createStorage("RInt", PROPERTY_TCP_LISTENING_PORT, "My listening TCP port");
	getLocalProperty(PROPERTY_TCP_LISTENING_PORT)->setReadonly(true);
	createStorage("RString", PROPERTY_YELLOWPAGES_FILENAME, "Yellow pages filename", 0);
	getLocalProperty(PROPERTY_YELLOWPAGES_FILENAME)->setReadonly(true);
	getLocalProperty(PROPERTY_YELLOWPAGES_FILENAME)->setPersistent(false);
	createStorage("RYellowPages", PROPERTY_YELLOWPAGES, "Yellow pages");
	getLocalProperty(PROPERTY_YELLOWPAGES)->setObject(new RYellowPages());
	createStorage("RDouble", PROPERTY_UDP_RENEWAL_INTERVAL, "UDP Renewal interval", 
		new RDouble(1000., RDouble::MS));
	createStorage("RDouble", PROPERTY_UDP_RENEWAL_TIMEOUT, "UDP Renewal timeout", 
		new RDouble(2000., RDouble::MS));
}

Repository::~Repository()
{
	endRepositoryThreads();
	propertiesMutex.lock(HERE);	// ;-)
	for (map<Url, Property*>::iterator it = localProperties.begin(); 
	it != localProperties.end(); ++it) delete it->second;
	for (map<Url, Property*>::iterator it = remoteProperties.begin();
	it != remoteProperties.end(); ++it) delete it->second;
	propertiesMutex.unlock();
}

void Repository::endRepositoryThreads()
{
	if (timerThreadStarted) {
		exitTimerThread = true;
		RDK_DEBUG_PRINTF("Waiting for timer thread to exit");
		pthread_join(timerThreadId, 0);
		timerThreadStarted = false;
	}

	#define DELETE_AND_SET_ZERO(x) if(x) { delete x; x=0; }
	
	DELETE_AND_SET_ZERO(remotePropertyManager);
	DELETE_AND_SET_ZERO(tcpManager);
	DELETE_AND_SET_ZERO(udpManager);
}

bool Repository::loadYellowPages(cstr ypFilename)
{
	bool b = false;
	if (ypFilename != "") {
		RDK_INFO_PRINTF("Loading yellow pages from file '%s'", ypFilename.c_str());
		try {
			lock(PROPERTY_YELLOWPAGES, WHERE("Loading yellow pages from file"));
			try {
				RYellowPages* yp = getObjectAsL<RYellowPages>(PROPERTY_YELLOWPAGES);
				b = yp->loadFromFile(ypFilename);	// NOTE: error is managed by the function itself
			}
			catch (const SessionException& e) {
				RDK_ERROR_PRINTF(e.what());
			}
			unlock(PROPERTY_YELLOWPAGES);
		}
		catch (const SessionException& e) {
			RDK_ERROR_PRINTF(e.what());
		}
	}
	if (b) setObject(PROPERTY_YELLOWPAGES_FILENAME, new RString(ypFilename));
	return b;
}

vector<string> getOptionsFromTextConfig(cstr val, string& newVal)
{
	newVal = val;
	size_t a = val.find_first_of("[");
	size_t b = val.find_last_of("]");
	if (a == string::npos) return vector<string>();
	else {
		if (b == string::npos) {
			RDK_ERROR_PRINTF("Malformed text configuration");
			return vector<string>();
		}
		else {
			string z = val.substr(a+1, b-a-1);
			vector<string> v = tokenize(z, ",");
			for (size_t i = 0; i < v.size(); i++) v[i] = trim(v[i]);
			return v;
		}
	}
}

void Repository::setPropertyValueFromTextConfig(cstr key, cstr val, CUrl urlContext, Session* session)
	throw (NoSuchProperty, InvalidOperation)
{
	Url url = Url(key).contextualize(urlContext);
	try {
		if (val[0] == '@') {
			// it is a link
			session->linkCreate(url, val.substr(1));
		}
		else {
			session->lock(url, HERE);
			try {
				RDK2::Object* obj = session->getObjectOrCreateItL(url);
				if (obj && !obj->loadFromStringRepresentation(val)) {
					RDK_ERROR_PRINTF("Cannot load property '%s' value from '%s'",
						url.c_str(), val.c_str());
				}
			}
			catch (const SessionException& e) {
				RDK_ERROR_PRINTF(e.what());
			}
			session->unlock(url);
		}
	}
	catch (const SessionException& e) {
		RDK_ERROR_PRINTF(e.what());
	}
}

void Repository::startRepositoryThreads()
{
	udpManager = new UdpManager(this);
	tcpManager = new TcpManager(this);
	remotePropertyManager = new RemotePropertyManager(this);

	udpManager->init();
	tcpManager->init();
	remotePropertyManager->init();

	udpManager->start();
	tcpManager->start();
	remotePropertyManager->start();

	pthread_create(&timerThreadId, 0, (void*(*)(void*)) timerThreadExec, this);
	timerThreadStarted = true;
}

Session* Repository::createSession(cstr name, cstr description, cstr urlContext)
{
	return new Session(name, description, urlContext, this);
}

void Repository::addGenericProperty(cstr className, CUrl url, cstr description, RDK2::Object* defaultObject)
	throw (InvalidOperation)
{
	//RDK_DEBUG_PRINTF("Adding property " + url);
	if (!url.isAbsolute()) throw InvalidOperation("Url has to be absolute ('" + url + "')");

	// XXX url.valid()
	if (url.find_first_of(" \n") != string::npos)
		throw InvalidOperation("Url cannot contain spaces or newlines (" + url + ")");

	if (defaultObject && defaultObject->getClassName() != className)
		throw InvalidOperation("defaultValue class (" + 
			defaultObject->getClassName() + ") does not match " +
			"property class (" + className + ")");


	propertiesMutex.lock(HERE);
	map<Url, Property*>::iterator it = localProperties.find(url);
	if (it != localProperties.end() && !it->second->deleted) {
		propertiesMutex.unlock();
		throw InvalidOperation("Property '" + url + "' is already in the repository");
	}

	if (it != localProperties.end() && it->second->deleted) {
		// FIXME che ne facciamo della property eliminata? delete �troppo pericoloso?
		Property* p = new Property(url, className, defaultObject);
		p->description = description;
		localProperties[url] = p; /// XXX aggiornare quella precedente (cosi' c'e' memory leak)
	}
	else {
		Property* p = new Property(url, className, defaultObject);
		p->description = description;
		p->def.setDescription(description);
		localProperties[url] = p;
	}

	for (multimap<string, Session*>::iterator it = sessionsListeningToTree.begin();
	it != sessionsListeningToTree.end(); ++it) {
		if (url.startsWith(it->first)) {
			it->second->pushEvent(new EventPropertyTreeAdded(url));
		}
	}

	propertiesMutex.unlock();
}

void Repository::deleteProperty(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	if (!url.isAbsolute()) throw InvalidOperation("Url has to be absolute ('" + url + "')");

	propertiesMutex.lock(HERE);
	map<Url, Property*>::iterator it = localProperties.find(url);
	if (it == localProperties.end()) {
		propertiesMutex.unlock();
		throw NoSuchProperty("Property '" + url + "' is not in the repository");
	}

	// we do not erase any property, we just mark it as "deleted"
	it->second->deleted = true;

	for (multimap<string, Session*>::iterator it = sessionsListeningToTree.begin();
	it != sessionsListeningToTree.end(); ++it) {
		if (url.startsWith(it->first)) {
			it->second->pushEvent(new EventPropertyTreeDeleted(url));
		}
	}

	propertiesMutex.unlock();
}

const RDK2::Object* Repository::getObjectLC(CUrl url) throw (NoSuchProperty)
{
	return getProperty(url)->getObjectLC();
}

RDK2::Object* Repository::getObjectL(CUrl url) throw (NoSuchProperty)
{
	return getProperty(url)->getObjectL();
}

RDK2::Object* Repository::getObjectOrCreateItL(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	Property* p = getProperty(url);
	RDK2::Object* obj = p->getObjectL();
	if (obj) return obj;
	else {
		//RDK_DEBUG_PRINTF("Creating an object of class '%s'", p->getObjectClassName().c_str());
		if (p->getObjectClassName() == "") 
			throw InvalidOperation("Trying to create an object for an unclassed property");
		else {
			obj = RDK2::Meta::forName(p->getObjectClassName());
			p->setObjectL(obj);
			return obj;
		}
	}
}

void Repository::lock(CUrl url, Stack w) throw (NoSuchProperty, InvalidOperation)
{
	getProperty(url)->lock(w);
}

void Repository::unlock(CUrl url) throw (NoSuchProperty, InvalidOperation)
{
	getProperty(url)->unlock();
}

map<Url, string> Repository::getLocalPropertyNames()
{
	propertiesMutex.lock(HERE);
	map<Url, string> ret;
	for (map<Url, Property*>::iterator it = localProperties.begin();
	it != localProperties.end(); ++it) {
		if (!it->second->deleted) ret.insert(make_pair(it->first, it->second->getObjectClassName()));
	}
	propertiesMutex.unlock();
	return ret;
}

map<Url, string> Repository::getRemotePropertyNames(cstr host)
{
	propertiesMutex.lock(HERE);
	map<Url, string> ret;
	for (map<Url, Property*>::iterator it = remoteProperties.begin();
	it != remoteProperties.end(); ++it) {
		if (!it->second->deleted) {
			if (host != "" && it->first.getHost() == host) {
				ret.insert(make_pair(it->first, it->second->getObjectClassName()));
			}
		}
	}
	propertiesMutex.unlock();
	return ret;
}

vector<Url> Repository::getLocalPropertiesStartingWith(string prefix)
{
	propertiesMutex.lock(HERE);
	vector<Url> v;
	for (map<Url, Property*>::iterator it = localProperties.begin();
	it != localProperties.end(); ++it) {
		if( !it->second->deleted && startsWith(it->first, prefix.c_str()) )
			v.push_back(it->first);
	}
	propertiesMutex.unlock();
	return v;
}

/// Timer thread

uint gcd(uint a, uint b)
{
	if (a * b == 0) return 0;
	while (true) {
		a = a % b;
		if (a == 0) return b;
		b = b % a;
		if (b == 0) return a;
	}
}

void Repository::timerThread()
{
	RDK_INFO_PRINTF("Starting timer thread");
	uint waitMs = 1000;
	while (usleep(waitMs * 1000), !exitTimerThread) {
		sessionsListeningToTimerMutex.lock(HERE);
		waitMs = 1000;
		for (map<Session*, SessionTimer>::iterator it = sessionsListeningToTimer.begin();
		it != sessionsListeningToTimer.end(); ++it) {
			//RDK_DEBUG_STREAM("timer is now " << it->second.timer.getMs() << ", I wait until " <<
			//	it->second.interval);
			if (it->second.timer.getMs() >= it->second.interval) {
				it->first->pushEvent(new EventTimer());
				//it->first->wakeUp();	// NOTE: wakeUp is called by pushEvent
				it->second.timer.start();
			}
			//RDK_DEBUG_PRINTF("waitMs was %d", waitMs);
			waitMs = gcd(waitMs, (uint) it->second.interval);
			//RDK_DEBUG_PRINTF("gcd with %d becomes %d", (uint) it->second.interval, waitMs);
			if (waitMs < 10) waitMs = 10;
		}
		sessionsListeningToTimerMutex.unlock();
	}
}

void Repository::saveAllDefaultPropertyDefs()
{
	propertiesMutex.lock(HERE);
	for (map<Url, Property*>::iterator it = localProperties.begin();
	it != localProperties.end(); ++it) {
		it->second->saveDefaultPropertyDef();
	}
	propertiesMutex.unlock();
}

}} // namespaces
