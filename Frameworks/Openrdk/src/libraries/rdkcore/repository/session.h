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

/**
 * @file
 *
 * @brief This file contains the Session class and some useful macros
 */

#ifndef RDK2_REPOSITORY_SESSION
#define RDK2_REPOSITORY_SESSION

#include <set>

#include <rdkcore/object/object.h>
#include <rdkcore/network/inetaddress.h>
#include <rdkcore/posixconstructs/posixsem.h>
#include <rdkcore/profiling/profiling.h>
#include <rdkcore/rprimitive/rint.h>
#include <rdkcore/rprimitive/rstring.h>
#include <rdkcore/rprimitive/rdouble.h>
#include <rdkcore/rprimitive/rbool.h>
#include <rdkcore/container/container.h>
#include <rdkcore/rgeometry/rpoint2od.h>
#include <rdkcore/rgeometry/rpoint2d.h>
#include <rdkcore/rgeometry/rpoint3d.h>
#include <rdkcore/rgeometry/rpoint3od.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/rmaps/rmapimage.h>
#include <rdkcore/posixconstructs/stack.h>
#include <rdkcore/textutils/textutils.h>

#include <rdkcore/repository_struct/rremotesubscription.h>
#include <rdkcore/repository_struct/url.h>
#include <rdkcore/repository_struct/property.h>
#include <rdkcore/repository_struct/robjectqueue.h>

#include "events.h"

#include "tcpmanager.h"		// for Network::ConnectionStatus


namespace RDK2 { namespace RepositoryNS {

class RPropertyDef;
class Repository;
class Property;
struct PropertyEnumItem;
using namespace RDK2::Geometry;
using namespace RDK2::RGeometry;
using RDK2::RGraphics::RImage;
using RDK2::RMaps::RMapImage;
using RDK2::Containers::RObjectVector;
using namespace RDK2::RPrimitive;
using namespace PosixConstructs;
using namespace PosixQueues;
using namespace std;
using namespace RDK2::TextUtils;
using namespace RDK2::Profiling;
using namespace Network;

enum QueueSubscriptionMode {
	QUEUE_LAST_OBJECT, ///< I just need the last
	QUEUE_ALL          ///< Please send every object
};

/**
 * The Session is The Way you have to access the repository.
 *
 * Depending on the type of data you want to access, you will need to
 * call different methods of this class.
 *
 * Note that all methods ending with "L" need the property to be
 * locked in advance.
 */
class Session {
protected:
	Session(cstr sessionName, cstr description, cstr urlContext, Repository* repository);

public:
	 ~Session();

	/// @name Management
	//@{
	/**
	 * @brief Start the session.
	 *
	 * Start the session. You should do this before accessing any
	 * property.
	 */
	void start() throw (InvalidOperation);
	/**
	 * @brief End the session and complain for forgot locks
	 *
	 * End the session, and complain if any property has been
	 * left lock()-ed.
	 */
	void end() throw (InvalidOperation);
	/**
	 * @brief End the session not complaining for forgot locks
	 *
	 * End the session, not complaining if any property has been
	 * left lock()-ed.
	 *
	 * This method should be called to end the session when there
	 * has been an unexpected error.
	 */
	void terminate() throw ();

	inline void setUrlContext(const string& url) { urlContext = url; }
	inline Url getUrlContext() const { return urlContext; }

	string getRepositoryName() const;

	string getSessionName() const;

	string getDescription() const;

	//@}

	/// @name Links
	//@{
	void linkCreate(cstr first, cstr second) throw (NoSuchProperty, InvalidOperation);
	void linkRemove(cstr first) throw(NoSuchProperty, InvalidOperation);
	//@}

	/// @name UtilityFunctions
	template<typename T>
	void copyObject(const Url& url, T& dest) {
		Property* p = lockAndGetProperty(url, true);
		dest = *(p->getObjectAsL<T>());
		unlockProperty(p);
	}

	struct PropertyUnlocker {
		PropertyUnlocker(Session* session, Property* p) : session(session), p(p) { }
		~PropertyUnlocker() { session->unlockProperty(p); }
		Session* session;
		Property* p;
	};

	template<typename T>
	T& getObjectCopyAs(const Url& url) {
		Property* p = lockAndGetProperty(url, true);
		PropertyUnlocker unlocker(this, p);
		return *(p->getObjectAsL<T>());
	}

	/// @name Storage creation
	//@{
	/**
	 * @brief Create a property with "special" type
	 */
	void createStorage(cstr className, cstr relativecstr, cstr description,
		RDK2::Object* defaultValue = 0)
		throw (InvalidOperation);
	/**
	 * @brief Set the "read only" attribute on properties
	 */
	void setReadonly(cstr url, bool readonly) throw (NoSuchProperty, InvalidOperation);
	/**
	 * @brief Set the "persistent" attribute on properties.
	 *
	 * "Persistent" means that the content will be saved when
	 * RAgent exits, and will be restored at next restart.
	 */
	bool isReadonly(cstr url) throw (NoSuchProperty, InvalidOperation);
	bool isPersistent(cstr url) throw (NoSuchProperty, InvalidOperation);
	//@}

	/// @name Property deletion
	//@{
	void deleteProperty(cstr url) throw (NoSuchProperty, InvalidOperation);
	//@}

	/// @name Storage locking
	//@{
	/**
	 * @brief Lock the selected property.
	 *
	 * Don't forget to unlock() it!
	 *
	 * The typical value for w is HERE.
	 */
	void lock(cstr, Stack w) throw (NoSuchProperty, InvalidOperation);
	/**
	 * @brief Unlock the selected property
	 */
	void unlock(cstr) throw (NoSuchProperty, InvalidOperation);
	//@}

protected:
	Property* lockAndGetProperty(CUrl url, bool followLinks = true)
		throw (InvalidOperation, NoSuchProperty);
	void unlockProperty(Property* p);

public:
	/// @name Storage values
	//@{
	const RDK2::Object* getObjectLC(cstr) throw (NoSuchProperty, InvalidOperation);
	RDK2::Object* getObjectL(cstr) throw (NoSuchProperty, InvalidOperation);
	template<typename T> const T* getObjectAsLC(cstr url) throw (NoSuchProperty, InvalidOperation, WrongType, ValueNotSet);
	template<typename T> T* getObjectAsL(cstr url) throw (NoSuchProperty, InvalidOperation, WrongType, ValueNotSet);
	RDK2::Object* getObjectOrCreateItL(cstr url) throw (NoSuchProperty, InvalidOperation);

	// new functions
	inline const RDK2::Object* getLockedObjectC(const Url& url) throw (NoSuchProperty, InvalidOperation)
	{ lock(url, HERE); return getObjectLC(url); }
	template<typename T> inline const T* getLockedObjectAsC(const Url& url) throw (NoSuchProperty, InvalidOperation, WrongType, ValueNotSet)
	{ lock(url, HERE); return getObjectAsLC<T>(url); }
	inline RDK2::Object* getLockedObject(const Url& url) throw (NoSuchProperty, InvalidOperation)
	{ lock(url, HERE); return getObjectL(url); }
	template<typename T> inline T* getLockedObjectAs(const Url& url) throw (NoSuchProperty, InvalidOperation, WrongType, ValueNotSet)
	{ lock(url, HERE); return getObjectAsL<T>(url); }

	RDK2::Object* getObjectClone(cstr) throw (NoSuchProperty, InvalidOperation);
	template <typename T>
	T* getObjectCloneAs(cstr url) throw (NoSuchProperty, InvalidOperation, ValueNotSet, WrongType);

	// new functions
	inline RDK2::Object* getClonedObject(const Url& url) throw (NoSuchProperty, InvalidOperation)
	{ return getObjectClone(url); }
	template<typename T> inline T* getClonedObjectAs(const Url& url) throw (NoSuchProperty, InvalidOperation, WrongType, ValueNotSet)
	{ return getObjectCloneAs<T>(url); }

	bool haveSameObject(cstr, cstr) throw (NoSuchProperty, InvalidOperation);
	bool isSet(cstr) throw (NoSuchProperty, InvalidOperation);
	//@}

	/// @name Properties' properties ;-)
	//@{
	bool propertyExists(cstr) throw (InvalidOperation);
	string getPropertyClassName(cstr) throw (NoSuchProperty, InvalidOperation);
	string getPropertyDescription(cstr) throw (NoSuchProperty, InvalidOperation);
	string getPropertyLinkTo(cstr) throw (NoSuchProperty, InvalidOperation);
	vector<PropertyEnumItem> getPropertyEnumItems(cstr) throw (NoSuchProperty, InvalidOperation);
	// obsolete functions
	RPropertyDef* getRPropertyDef(cstr) throw (NoSuchProperty, InvalidOperation);	// FIXME
	void setPropertyOptions(cstr url, PropertyOptions options)
		throw (NoSuchProperty, InvalidOperation);
	// new functions
	PropertyDef getPropertyDef(cstr) throw (NoSuchProperty, InvalidOperation);
	PropertyDef getDefaultPropertyDef(cstr) throw (NoSuchProperty, InvalidOperation);
	void setPersistent(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void setVolatile(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void setKeepThis(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void setEditable(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void setNotEditable(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void setExternalFileName(CUrl url, cstr fileName) throw (NoSuchProperty, InvalidOperation);
	void setExternalFileBinary(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void setExternalFileXml(CUrl url) throw (NoSuchProperty, InvalidOperation);
	//@}

	/// @name Storage modifying
	//@{
	void setObject(cstr, RDK2::Object* value) throw (NoSuchProperty, InvalidOperation, WrongType);
	void declareDiffL(cstr, ObjectDiff* diff) throw (NoSuchProperty, InvalidOperation);
	const std::vector<const RDK2::Object*> diffsFreeze(cstr) throw (NoSuchProperty, InvalidOperation);
	//@}

	/// @name Storage subscription
	//@{
	void storageSubscribeValue(cstr) throw (NoSuchProperty, InvalidOperation);
	void storageUnsubscribeValue(cstr) throw (NoSuchProperty, InvalidOperation);
	void storageSubscribeDiffs(cstr) throw (NoSuchProperty, InvalidOperation);
	void storageUnsubscribeDiffs(cstr) throw (NoSuchProperty, InvalidOperation);
	//@}

	/// @name Remote property options
	//@{
	void remotePropertyOptions(cstr url, Network::NetProtocol socketType = Network::TCP_IP,
		RRemoteSubscription::What what = RRemoteSubscription::STORAGE_DIFFS,
		RRemoteSubscription::When when = RRemoteSubscription::PERIODIC, double period = 500.)
		throw (NoSuchProperty, InvalidOperation);
	//@}

	/// @name Queues
	//@{
	void queueCreate(CUrl url, cstr description) throw (NoSuchProperty, InvalidOperation);
	inline void createQueue(CUrl url, cstr description) throw (NoSuchProperty, InvalidOperation)
	{ queueCreate(url, description); }	// XXX it is an alias, for I always make a mistake
	void queueSubscribe(CUrl url, QueueSubscriptionMode qs = QUEUE_ALL)
		throw (NoSuchProperty, InvalidOperation);
	void queueUnsubscribe(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void queueSetInterests(CUrl url, PosixQueues::MyInterests<RDK2::Object>*i)
		throw (NoSuchProperty, InvalidOperation);
	vector<const RDK2::Object*> queueFreeze(CUrl url) throw (NoSuchProperty, InvalidOperation);
	template<typename T>
	vector<const T*> queueFreezeAs(CUrl url) throw (NoSuchProperty, InvalidOperation);
	vector<const RDK2::Object*> queueContent(CUrl url) throw (NoSuchProperty, InvalidOperation);
	template<typename T>
	vector<const T*> queueContentAs(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void queueKeep(CUrl url, const RDK2::Object* obj) throw (NoSuchProperty, InvalidOperation);
	const RDK2::Object* queueLast(CUrl url) throw (NoSuchProperty, InvalidOperation);
	template<typename T>
	const T* queueLastAs(CUrl url) throw (NoSuchProperty, InvalidOperation, WrongType);
	bool queuePush(CUrl url, RDK2::Object* obj) throw (NoSuchProperty, InvalidOperation);
	size_t queueSize(CUrl url) throw  (NoSuchProperty, InvalidOperation);
	//@}

	/// @name Events
	//@{
	/// NOTE: valueChanged *cannot* be called inside locked region.
	/// It assumes that url is unlocked and performes side-effect on it
	/// It update timestamp of the object.
	void valueChanged(CUrl url) throw (NoSuchProperty, InvalidOperation);
	/// NOTE: listen implies subscribe
	void listen(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void unlisten(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void listenToTreeChanges(cstr prefix) throw (InvalidOperation);
	void listenToTimer(double ms) throw (InvalidOperation);
	void wait() throw();
	/// you need to call this function if you don't wait()
	void dontWait() throw();
	/// you can safely call this function from another thread
	void pushEvent(Event* event) throw();
	vector<const Event*> events() throw (InvalidOperation);
	/// you can safely call this function from another thread
	void wakeUp(int times = 1) throw ();
	bool wokenByEvent(cstr eventName) throw (InvalidOperation);
	PosixConstructs::PosixSemaphore* getMySemaphore() { return &sessionSemaphore; }
	//@}

	/// @name Profiling
	//@{
	void profilingStartEvent(cstr name) throw (InvalidOperation);
	void profilingEndEvent(cstr name) throw (InvalidOperation);
	//@}

protected:
	/// @name Utilities for get/set/create functions
	#define TRY_TO_BOOL(a) try { a; return true; } catch (SessionException&) { return false; }
	template<typename SimpleType, typename RType>
	SimpleType getValue(CUrl url) throw (InvalidOperation, NoSuchProperty, ValueNotSet, WrongType);

public:
	/// @name Bools
	//@{
	void createBool(CUrl url, cstr description, PropertyOptions options = NORMAL)
		throw (InvalidOperation);
	void createBool(CUrl url, cstr description, bool defaultValue, PropertyOptions options = NORMAL)
		throw (InvalidOperation);
	void setBool(CUrl url, bool i) throw (InvalidOperation, NoSuchProperty, WrongType);
	bool getBool(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getBoolB(CUrl url, bool& b) { TRY_TO_BOOL(b = getBool(url)); }
	inline bool setBoolB(CUrl url, bool b) { TRY_TO_BOOL(setBool(url, b)); }
	//@}

	/// @name Ints
	//@{
	void createInt(CUrl url, cstr description, PropertyOptions options = NORMAL)
		throw (InvalidOperation);
	void createInt(CUrl url, cstr description, int defaultValue, PropertyOptions options = NORMAL)
		throw (InvalidOperation);
	void setInt(CUrl url, int i) throw (InvalidOperation, NoSuchProperty, WrongType);
	int getInt(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getIntB(CUrl url, int& i) { TRY_TO_BOOL(i = getInt(url)); }
	inline bool setIntB(CUrl url, int i) { TRY_TO_BOOL(setInt(url, i)); }
	//@}

	/// @name Strings
	//@{
	void createString(CUrl url, cstr description, PropertyOptions options = NORMAL)
		throw (InvalidOperation);
	void createString(CUrl url, cstr description, cstr defaultValue, PropertyOptions options = NORMAL)
		throw (InvalidOperation);
	void setString(CUrl url, const string& s) throw (InvalidOperation, NoSuchProperty, WrongType);
	string getString(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getStringB(CUrl url, string& s) { TRY_TO_BOOL(s = getString(url)); }
	inline bool setStringB(CUrl url, const string& s) { TRY_TO_BOOL(setString(url, s)); }
	//@}

	/// @name Doubles
	//@{
	void createDouble(CUrl url, cstr desc, RDouble::Unit unit, PropertyOptions options = NORMAL)
		throw (InvalidOperation);
	void createDouble(CUrl url, cstr desc, RDouble::Unit unit, double defValue,
		PropertyOptions options = NORMAL) throw (InvalidOperation);
	void setDouble(CUrl url, double d) throw (InvalidOperation, NoSuchProperty, WrongType);
	double getDouble(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getDoubleB(CUrl url, double& d) { TRY_TO_BOOL(d = getDouble(url)); }
	inline bool setDoubleB(CUrl url, double d) { TRY_TO_BOOL(setDouble(url, d)); }
	//@}

	#define ENUM_CREATE(v) vector<PropertyEnumItem> v
	#define ENUM_ITEM(v, value, name, desc) v.push_back(PropertyEnumItem(value, name, desc))
	/// @name Enums
	//@{
	void createEnum(CUrl url, cstr desc, const vector<PropertyEnumItem>& entries, uint defVal,
		PropertyOptions options = NORMAL) throw (InvalidOperation);
	void setEnum(CUrl url, uint v) throw (InvalidOperation, NoSuchProperty, WrongType);
	inline int getEnum(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet)
		{ return getInt(url); }
	string getEnumString(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getEnumB(CUrl url, uint& d) { TRY_TO_BOOL(d = getInt(url)); }
	inline bool setEnumB(CUrl url, uint d) { TRY_TO_BOOL(setInt(url, d)); }
	bool isEnum(CUrl url) throw (InvalidOperation, NoSuchProperty);
	//@}

	/// @name (2D) Poses
	//@{
	void createPose(CUrl url, cstr description) throw (InvalidOperation);
	void createPose(CUrl url, cstr description, const Point2od& defaultValue) throw (InvalidOperation);
	void setPose(CUrl url, const Point2od& p) throw (InvalidOperation, NoSuchProperty, WrongType);
	Point2od getPose(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getPoseB(CUrl url, Point2od& p) { TRY_TO_BOOL(p = getPose(url)); }
	inline bool setPoseB(CUrl url, const Point2od& p) { TRY_TO_BOOL(setPose(url, p)); }
	//@}

	/// @name (2D) Point
	//@{
	void createPoint(CUrl url, cstr description) throw (InvalidOperation);
	void createPoint(CUrl url, cstr description, const Point2d& defaultValue) throw (InvalidOperation);
	void setPoint(CUrl url, const Point2d& p) throw (InvalidOperation, NoSuchProperty, WrongType);
	Point2d getPoint(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getPointB(CUrl url, Point2d& p) { TRY_TO_BOOL(p = getPoint(url)); }
	inline bool setPointB(CUrl url, const Point2d& p) { TRY_TO_BOOL(setPoint(url, p)); }
	//@}

	/// @name 3D Poses
	//@{
	void createPose3(CUrl url, cstr description) throw (InvalidOperation);
	void createPose3(CUrl url, cstr description, const Point3od& defaultValue) throw (InvalidOperation);
	void setPose3(CUrl url, const Point3od& p) throw (InvalidOperation, NoSuchProperty, WrongType);
	Point3od getPose3(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getPose3B(CUrl url, Point3od& p) { TRY_TO_BOOL(p = getPose3(url)); }
	inline bool setPose3B(CUrl url, const Point3od& p) { TRY_TO_BOOL(setPose3(url, p)); }
	//@}

	/// @name 3D Point
	//@{
	void createPoint3(CUrl url, cstr description) throw (InvalidOperation);
	void createPoint3(CUrl url, cstr description, const Point3d& defaultValue) throw (InvalidOperation);
	void setPoint3(CUrl url, const Point3d& p) throw (InvalidOperation, NoSuchProperty, WrongType);
	Point3d getPoint3(CUrl url) throw (InvalidOperation, NoSuchProperty, WrongType, ValueNotSet);
	inline bool getPoint3B(CUrl url, Point3d& p) { TRY_TO_BOOL(p = getPoint3(url)); }
	inline bool setPoint3B(CUrl url, const Point3d& p) { TRY_TO_BOOL(setPoint3(url, p)); }
	//@}

	/// @name BigObjects
	//@{
	void createImage(CUrl url, cstr description, int width, int height, RImage::Type type)
		throw (InvalidOperation, WrongType)
		{ createStorage("RImage", url, description); setObject(url, new RImage(width, height, type)); }
	void createEmptyImage(CUrl url, cstr description) throw (InvalidOperation, WrongType)
		{ createStorage("RImage", url, description); }
	void createMap(CUrl url, cstr description, double x, double y, double theta, double realWidth,
		int bitmapWidth, int bitmapHeight) throw (InvalidOperation, WrongType)
		{ createStorage("RMapImage", url, description);
		  setObject(url, new RMapImage(x, y, theta, realWidth,
		  new RImage(bitmapWidth, bitmapHeight, RImage::C8))); }
	void createEmptyMap(cstr url, cstr description) throw (InvalidOperation, WrongType)
		{ createStorage("RMapImage", url, description); }
	template<typename T>
	void createVector(CUrl url, cstr description) throw (InvalidOperation, WrongType)
		{ RDK2::Containers::Vector<T>* v = new RDK2::Containers::Vector<T>();
		  createStorage(v->getClassName(), url, description); setObject(url, v); }
	//@}

	/// @name Network utilities
	//@{
	void tryConnectTo(Network::NetProtocol protocol, const string& peerName);
	void tryDisconnectFrom(Network::NetProtocol protocol, const string& peerName);
	ConnectionStatus getConnectionStatusWith(Network::NetProtocol type, const string& peerName);
	bool getInetAddressOf(const string& name, Network::NetProtocol socketType, InetAddress& address) throw (SessionException);
	//@}

	Repository* getRepository() { return repository; }

	/// Get absolute url from (possibly relative)
	Url getAbsoluteUrl(CUrl url);

	bool isLocalProperty(CUrl url) const throw ();

	/// @name Text config file utilities
	string textConfigLineForProperty(CUrl relativeUrl);

	vector<Url> getPropertyNamesStartingWith(cstr prefix) throw ();

protected:
	// Once started from a thread, the session cannot be started by another one
	bool threadIdInitialized;
	pthread_t threadId;

	TimerT sessionActivityTimer;
	TimerR sessionScheduleTimer;
	double sessionCurrentScheduleInterval;
	vector<double> sessionActivityCache;
	vector<double> sessionScheduleCache;

	Url urlContext;
	Repository* const repository;
	string sessionName;
	string description;

	bool started;
	void checkStarted() throw (InvalidOperation);

	// This is the semaphore for the session->wait()
	PosixConstructs::PosixSemaphore sessionSemaphore;

	PosixQueues::PipeQueue<Event>* eventQueue;
	PosixQueues::PipeQueueConsumer<Event>* consumerEventQueue;
	// the following vector is accessed only by the thread of this session
	vector<const Event*> currentEvents;

	// questi vengono usati solo dal thread della session, non hanno bisogno di mutex
	map<Url, vector<ObjectDiff*> > pendingDiffs;
	set<string> lockedProperties;

	Property* getProperty(cstr url, bool followLinks = true) throw (NoSuchProperty, InvalidOperation);

public:
	volatile bool exiting;		// ??? FIXME

protected:
	PosixConstructs::PosixMutex ownedPropertiesMutex;
		// note: here names are absolute
		set<Url> ownedProperties;

protected:
	struct QueueSubscription {
		QueueSubscription() : consumerQueue(0), mode(QUEUE_ALL), isListened(false) { }
		//RObjectPipeQueue* queue;
		RConsumerObjectPipeQueue* consumerQueue;
		QueueSubscriptionMode mode;
		bool isListened;	// XXX occhio! informazione duplicata (si pu� prendere
							// dalla master queue con hasSignalSemaphore())
	};
	PosixConstructs::PosixMutex queueSubscriptionsMutex;
		map<Url, QueueSubscription> queueSubscriptions;
		map<Url, QueueSubscription> diffsSubscriptions;
	// if alsoCreate is false, qsm has no meaning
	map<Url, QueueSubscription>::iterator
		queueGetSubscription(CUrl url, bool alsoCreate, QueueSubscriptionMode qsm = QUEUE_ALL)
		throw (NoSuchProperty, InvalidOperation);

	RConsumerObjectPipeQueue* getDiffsConsumerQueue(cstr url);

	void listenToQueue(CUrl url) throw (NoSuchProperty, InvalidOperation);
	void unlistenToQueue(CUrl url) throw (NoSuchProperty, InvalidOperation);

/// Events management
//@{
public:
	class SessionEventObserver { };		/// event observer interface
	typedef bool (*SessionEventHandlerFnPtr) (SessionEventObserver*, const Event*);
	struct SessionEventHandler {
		SessionEventHandler(SessionEventObserver* observer, SessionEventHandlerFnPtr fnptr) :
			observer(observer), fnptr(fnptr) { }
		SessionEventObserver* observer;
		SessionEventHandlerFnPtr fnptr;
	};
protected:
	list<SessionEventHandler> timerEventHandlers;
	list<pair<Url, SessionEventHandler> > propertyUpdateEventHandlers;
	list<pair<Url, SessionEventHandler> > propertyTreeEventHandlers;
public:
	void registerTimerEventHandler(const SessionEventHandler& handler) throw ();
	bool unregisterTimerEventHandler(const SessionEventHandler& handler) throw ();
	void registerPropertyUpdateEventHandler(const SessionEventHandler& handler, const Url& prefix = "") throw ();
	void unregisterPropertyUpdateEventHandler(const SessionEventHandler& handler, const Url& prefix = "") throw ();
	void registerPropertyTreeEventHandler(const SessionEventHandler& handler, const Url& prefix = "") throw ();
	void processEvents() throw (SessionException);
//@}

friend class Repository;
}; // class Session

#include "session.hpp"

}} // namespaces

/// Open a try {} block and start the session in there.
#define SESSION_TRY_START(s) try { s->start();
/// Just open a try block
#define SESSION_TRY(s) try {
/**
 * @brief Close the (try) block and open a catch {} block
 *
 * The session exception is named e.
 */
#define SESSION_CATCH(s) } catch (const RDK2::SessionException& e) {
/**
 * @brief Rethrow a catch-ed session, adding information
 *
 * The session will have appended information about where it's been
 * re-throw-ed.
 */
#define SESSION_RETHROW(s) 	char buf[1024]; \
	  sprintf(buf, "%s (From %s:%d)", e.what(), __FILE__, __LINE__); \
	  throw RDK2::SessionException(buf); }
/// SESSION_CATCH() and SESSION_RETHROW()
#define SESSION_CATCH_RETHROW(s) SESSION_CATCH(s) SESSION_RETHROW(s)

/// SESSION_CATCH() and go on silently
#define SESSION_CATCH_NOMSG(s) SESSION_CATCH(s) }

/// Calls end() on the session
#define SESSION_END(s) s->end();
#define SESSION_CONTINUE(s) { SESSION_END(s); continue; }
#define SESSION_CATCH_TERMINATE(s) } catch (const RDK2::SessionException& e) { s->terminate(); \
	RDK_ERROR_STREAM(e.what()); }
#define SESSION_CATCH_TERMINATE_NOMSG(s) } catch (const RDK2::SessionException& e) { s->terminate(); }
/// The most common way to close a session
#define SESSION_END_CATCH_TERMINATE(s) SESSION_END(s) SESSION_CATCH_TERMINATE(s)
/// The most common way to close a session, if you want no error messages
#define SESSION_END_CATCH_TERMINATE_NOMSG(s) SESSION_END(s) SESSION_CATCH_TERMINATE_NOMSG(s)

#endif
