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

#ifndef RDK2_REPOSITORY_REPOSITORY
#define RDK2_REPOSITORY_REPOSITORY

#include <string>
#include <vector>
#include <map>
#include <set>
#include <pthread.h>

#include <rdkcore/posixconstructs/posixsem.h>
#include <rdkcore/posixconstructs/posixmutex.h>
#include <rdkcore/posixqueues/queueinterface.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/object/object.h>
#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/textutils/textutils.h>

#include <rdkcore/repository_struct/property.h>
#include <rdkcore/repository_struct/url.h>
#include <rdkcore/repository_struct/rremotesubscription.h>
#include <rdkcore/repository_struct/rpropertydef.h>

#include "ApplicationProtocolHandler.h"
#include "session.h"
#include "udpmanager.h"
#include "remotepropertymanager.h"
#include "tcpmanager.h"
#include "remotesubscriptionregister.h"

#define MAX_LINK_LEVELS 5

#define PROPERTY_NETWORK_PREFIX       "/network"
#define PROPERTY_YELLOWPAGES          PROPERTY_NETWORK_PREFIX "/yellowPages"
#define PROPERTY_YELLOWPAGES_FILENAME PROPERTY_NETWORK_PREFIX "/yellowPagesFilename"
#define PROPERTY_UDP_PORT             PROPERTY_NETWORK_PREFIX "/udpPort"
#define PROPERTY_TCP_LISTENING_PORT   PROPERTY_NETWORK_PREFIX "/tcpListeningPort"
#define PROPERTY_INBOX                PROPERTY_NETWORK_PREFIX "/inbox"
#define PROPERTY_OUTBOX               PROPERTY_NETWORK_PREFIX "/outbox"
#define PROPERTY_NETWORK_REMOTESUBSCRIPTIONS_PREFIX PROPERTY_NETWORK_PREFIX "/remoteSubscriptions"
#define PROPERTY_UDP_RENEWAL_INTERVAL PROPERTY_NETWORK_REMOTESUBSCRIPTIONS_PREFIX "/udpRenewalInterval"
#define PROPERTY_UDP_RENEWAL_TIMEOUT  PROPERTY_NETWORK_REMOTESUBSCRIPTIONS_PREFIX "/udpRenewalTimeout"

namespace RDK2 { namespace RepositoryNS {

using namespace std;
using namespace RDK2::TextUtils;

class Repository {
public:
	/// Constructor
	/// @param repositoryName repository name, globally unique (it is the "host" part of the url)
	Repository(cstr repositoryName);

	/// Destructor
	virtual ~Repository();

	/// Each repository has a unique name, i.e. the one used in the constructor
	inline string getRepositoryName() { return repositoryName; }

	/// Globally, each property of this repository begins with its base url, i.e. "rdk://" + repositoryName
	inline Url getBaseUrl() { return Url("rdk://" + repositoryName); }

	Url normalizeUrl(const Url& url) throw ();

	/// Creation of a new session.
	/// @param urlContext context for the session (absolute url, starting with "/" and without host name)
	/// @param descName descriptive name
	Session* createSession(cstr name, cstr descName, cstr urlContext);

	/// Loads the yellow pages
	/// @param ypFilename the file name
	bool loadYellowPages(cstr ypFilename);

	/// Starts the comunication threads (UDP, TCP, and Remote property manager)
	void startRepositoryThreads();
	void endRepositoryThreads();

	// FIXME solo le url
	map<Url, string> getLocalPropertyNames();	///< @return a map: url (absolute), class name
	map<Url, string> getRemotePropertyNames(cstr host);	///< @return a map: url (complete), class name
	vector<Url> getLocalPropertiesStartingWith(string prefix);

protected:
	/// Take the address of a property. @param url absolute or complete url 
	/// of the property, can be remote
	Property* getProperty(CUrl url) throw (NoSuchProperty, InvalidOperation);	// FIXME DEPRECATED
	Property* getProperty(CUrl url, Session*) throw (NoSuchProperty);
	Property* getLocalProperty(CUrl url) throw (NoSuchProperty);
	Property* getRemoteProperty(CUrl url, Session* askingSession) throw ();

	bool isLocalProperty(CUrl) throw ();
	void addGenericProperty(cstr className, CUrl url, cstr description,
		RDK2::Object* defaultObject = 0) throw (InvalidOperation);

	void deleteProperty(CUrl url) throw (NoSuchProperty, InvalidOperation);

	/// @name Storage locking
	void lock(CUrl url, Stack w) throw (NoSuchProperty, InvalidOperation);
	void unlock(CUrl url) throw (NoSuchProperty, InvalidOperation);

	/// @name Storage creation
	/// Create a storage property. @param url *absolute* url of the property,
	/// i.e. starting with "/" and WITHOUT host name
	void createStorage(cstr className, CUrl url, cstr description,
		RDK2::Object* defaultObject = 0) throw (InvalidOperation);

	/// @name Storage value
	const RDK2::Object* getObjectLC(CUrl url) throw (NoSuchProperty);
	RDK2::Object* getObjectL(CUrl url) throw (NoSuchProperty);
	template <typename T>
	const T* getObjectAsLC(CUrl url) throw (NoSuchProperty, ValueNotSet, WrongType);
	template <typename T>
	T* getObjectAsL(CUrl url) throw (NoSuchProperty, ValueNotSet, WrongType);
	RDK2::Object* getObjectOrCreateItL(CUrl url) throw (NoSuchProperty, InvalidOperation);

	/// @name Storage modifying
	void setObject(CUrl, RDK2::Object* object) throw (NoSuchProperty, InvalidOperation, WrongType);
	bool pushDiffInDiffQueue(CUrl, ObjectDiff* diff) throw (NoSuchProperty, InvalidOperation);

	/// @name Events
	void valueChanged(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void valueChanged(CUrl url, const vector<ObjectDiff*>& diffs)
		throw (NoSuchProperty, InvalidOperation);
	void wakeUpListeningSessions(CUrl url, const string& eventType) throw (NoSuchProperty);

	/// @name Queue creation
// 	void queueCreate(CUrl absoluteUrl, cstr description) throw (NoSuchProperty, InvalidOperation);

public://XXX
	/// @name Queue push
	bool queuePush(CUrl Curl, RDK2::Object* object) throw (NoSuchProperty, InvalidOperation);

public:
	void saveAllDefaultPropertyDefs();

protected:
	string repositoryName;

	/// Repository properties
	PosixConstructs::PosixMutex propertiesMutex;
		map<Url, Property*> localProperties;    // urls here are absolute, i.e. /<path>, without <host> and without "rdk://"
		map<Url, Property*> remoteProperties;   // urls here are complete

	/// Sessions listening to tree changes
	PosixConstructs::PosixMutex sessionsListeningToTreeMutex;
		multimap<string, Session*> sessionsListeningToTree;

	/// Timer thread stuff
	struct SessionTimer {
		TimerR timer;
		double interval;
	};
	volatile bool exitTimerThread;
	PosixConstructs::PosixMutex sessionsListeningToTimerMutex;
		map<Session*, SessionTimer> sessionsListeningToTimer;
	pthread_t timerThreadId;
	bool	 timerThreadStarted;
	static void* timerThreadExec(Repository* repository) { repository->timerThread(); return 0; }
	void timerThread();

	/// Comunication threads
	UdpManager* udpManager;
	TcpManager* tcpManager;
	RemotePropertyManager* remotePropertyManager;

public:
	RemoteSubscriptionRegister remoteSubscriptionRegister;

protected:
	void remotePropertyOptions(CUrl url, Network::NetProtocol socketType = Network::TCP_IP,
		RRemoteSubscription::What what = RRemoteSubscription::STORAGE_DIFFS,
		RRemoteSubscription::When when = RRemoteSubscription::PERIODIC, double minPeriod = 50., double maxPeriod = 500.)
		throw (NoSuchProperty, InvalidOperation);

public:
	/// @name Utilities
	// XXX da mettere in Session
	void setPropertyValueFromTextConfig(cstr key, cstr val, CUrl urlContext, Session* session)
		throw (NoSuchProperty, InvalidOperation);

	/// @name Remote stuff utilities
	// XXX mettere in remotesubscriptionregister?
	void sendStorageValueToHost(CUrl completeUrl, cstr askingHost, Network::NetProtocol socketType, RDK2::Object* objToSend = 0)
		throw();
	void sendStorageValuesToRemoteValueSubscribers(CUrl completeUrl, cstr askingHost,
		Network::NetProtocol socketType, RRemoteSubscription::What what, RRemoteSubscription::When when) throw();
	void sendStorageDiffToHost(CUrl completeUrl, cstr askingHost, Network::NetProtocol socketType, ObjectDiff* diffToSend)
		throw();
	void sendObjectsToRemoteQueueSubscribers(CUrl completeUrl, const RDK2::Object* objToSend) throw();
	void setRemotePropertyDef(CUrl url, const RPropertyDef* def);

public:
	void registerApplicationProtocolHandler(const string& protocol, ApplicationProtocolHandler* handler);

private:
	ApplicationProtocolHandler* getApplicationProtocolHandler(const string& protocol);
	PosixConstructs::PosixMutex applicationProtocolHandlersMutex;
	map<string, ApplicationProtocolHandler*> applicationProtocolHandlers;

friend class Session;
friend class RemotePropertyManager;
friend class TcpManager;
friend class RemoteSubscriptionRegister;
};

template <typename T>
const T* Repository::getObjectAsLC(CUrl url) throw (NoSuchProperty, WrongType, ValueNotSet)
{
	return getProperty(url)->getObjectAsLC<T>();
}

template <typename T>
T* Repository::getObjectAsL(CUrl url) throw (NoSuchProperty, WrongType, ValueNotSet)
{
	return getProperty(url)->getObjectAsL<T>();
}

}} // namespaces

#endif
