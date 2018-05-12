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

#include <rdkcore/textutils/textutils.h>
#include "repository.h"
#include "url.h"

namespace RDK2 { namespace RepositoryNS {
/**
	Common usage:
	
	@code
	Session * s = ..
	
	try {
		
		s->sessionStart();
		...
		...
		...
		s->sessionEnd();
		
	} catch(SessionException e) {
		RDK_ERROR_STREAM(e.what());
		s->sessionTerminate();
	} catch(...) {
		RDK_ERROR_STREAM("Unexpected exception");
		s->sessionTerminate();
	}
	@endcode
	
	or:
	
	@code
	while(running) {
		s->wait();
		s->sessionStart();
		...
		s->sessionEnd();
		
		sleep(1000);
		
		s->sessionStart();
		...
		s->sessionEnd();
	}
	@endcode
*/

struct SessionInterfaceCore {
		
	/// @name Session management
	/// Session management
	//@{
		/// Starts a session. Every other function must be called while the session
		/// is started (or it will throw an InvalidException), with the only exception
		/// of wait().
		///
		/// @throw   InvalidOperation   Previous session was not ended properly.  
		virtual void sessionStart()
			throw(InvalidOperation) = 0;
		
		/// Check that lock are released, all is safe, etc.
		/// Use sessionTerminate() to clear locks if sessionEnd() throws an exception.
		///
		/// @throw   SyncError          something bad
		/// @throw   InvalidOperation   session was not started
		virtual void sessionEnd()
			throw(InvalidOperation, SyncError) = 0;
		
		/// Terminates the session (unlocks
		/// This function does not throw.
		virtual void sessionTerminate() throw() = 0;

		/// @return base url of this session
		virtual Url getUrlContext() = 0;
	//@}
		
	/// @name Params
	/// Parameters reading from file and parsing from command line arguments
	//@{
		virtual void parseConfigFile(Url baseUrl, cstr filename)
			throw(InvalidOperation, SyncError, ParseError) = 0;
			
		/// Parses command line arguments into properties.
		///
		/// Example:
		///
		/// @code
		/// int main(argc, argv) {
		/// 	...
		/// 	Session& s = ...
		/// 	s.addString("config/ypconfig", "yellow pages configuration", "yp.config");
		/// 	s.addInt("config/maxvictims", "maximum number of found victims", 1);
		/// 	s.parseArguments("config/", argc, argv);
		/// 	...
		/// }
		/// @endcode
		/// 
		/// @verbatim
		/// $ main --ypconfig yp2.config --maxvictims 3
		/// @endverbatim
		///
		/// @throw   ParseError      malformed argument list
		/// @throw   NoSuchProperty  some properties not existent
		/// @throw   WrongType       parsing error for some arguments
		virtual void parseArguments(Url baseUrl, int argc, char** argv)
			throw(ParseError, NoSuchProperty, WrongType) = 0;
		
	//@}
		
	/// @name Link
	/// Link creation and destruction
	//@{
		/// Create a link from property first to property second
		/// when the user access property first, he will actually access property second
		/// (there is a limited amount of subsequent link redirection)
		///
		/// If second is "unknown", nothing will happen for the moment.
		/// On subsequent operations (ex: getXXX()), it will throw.
		///
		/// @throw NoSuchProperty    if first does not exist
		virtual void linkCreate(Url first,  Url second) throw(NoSuchProperty) = 0;
	
		/// Removes a link. If link is not set, nothing happens.
		///
		/// @throw NoSuchProperty    if first does not exist
		virtual void linkRemove(Url first) throw(NoSuchProperty) = 0;
	//@}
		
		

	/// @name Storage creation
	/// Storage creation functions
	//@{
			
		/// This will create a storage property
		///
		/// @params repository owns defaultValue
		///
		/// @throw InvalidOperation  if resource already present
		/// @throw InvalidOperation  if url is not relative (or contains "..").
		virtual void createStorage(
			cstr className,
			Url relativeUrl,
			cstr description,
			RDK2::Object* defaultValue) throw(InvalidOperation) = 0;
		
		/// If read-only, it can't be changed remotely.
		/// default is false for all kinds of property
		///
		/// @throw NoSuchProperty    if url does not exist
		/// @throw InvalidOperation  if url is not a Storage or it is remote
		/// @throw InvalidOperation  if resource was not created by this session.
		virtual void setReadonly(Url url, bool readonly) 
			throw(NoSuchProperty, InvalidOperation) = 0;
		
		/// If isPersistent, it is saved to config file
		/// default is true for all kinds of property
		///
		/// @throw NoSuchProperty    if url does not exist 
		/// @throw InvalidOperation  if url is not a Storage or it is remote
		/// @throw InvalidOperation  if resource was not created by this session.
		virtual void setPersistent(Url url, bool isPersistent)
			throw(NoSuchProperty, InvalidOperation) = 0;

	//@}

	/// @name Storage locking
	/// Lock/getValue/unlock combo for storage properties
	//@{

		/// Lock Storage, to edit content
		/// use shortcuts below if datatype is rprimitive
		///
		/// @throw NoSuchProperty    if url does not exist
		/// @throw InvalidOperation  if url is a Queue
		/// @throw SyncError         if already locked
		virtual void lock(Url, Where w = Where())
			throw (NoSuchProperty, InvalidOperation, SyncError) = 0;
		
		/// @throw NoSuchProperty   if url does not exist
		/// @throw InvalidOperation if url is not storage
		/// @throw SyncError        if not locked
		virtual void unlock(Url)
			throw (NoSuchProperty, InvalidOperation, SyncError) = 0;
	
	//@}

	/// @name Storage value
	/// Function to retrieve the value (object) of a property
	//@{
		/// You can change the object, but not delete
		/// You CAN change remote properties, but changes are
		/// not propagated to the source. You should use push_update instead.
		/// If object has only default value, the default value is cloned
		/// and set as the value
		///
		/// @throw NoSuchProperty   if url does not exist
		/// @throw InvalidOperation if url is not Storage
		/// @throw ValueNotSet      if url does not have value
		/// @throw SyncError        if not locked
		virtual RDK2::Object* getValueL(Url)
			throw (NoSuchProperty, InvalidOperation, SyncError) = 0;

		virtual const RDK2::Object* getValueLC(Url)
			throw (NoSuchProperty, InvalidOperation, SyncError) = 0;
	//@}

	/// @name Storage modifying
	/// Functions for pushing updates to the storage
	//@{
		
		/// No copy is made, ob belongs to the session. You can't use ob anymore.
		/// 
		/// @throw NoSuchProperty   if url does not exist
		/// @throw InvalidOperation if url is not storage
		virtual void pushUpdate(Url, RDK2::Object*value)
			throw (NoSuchProperty, InvalidOperation) = 0;

		virtual void pushUpdate(Url, RDK2::Meta::ObjectUpdate*update)
			throw (NoSuchProperty, InvalidOperation) = 0;

		/// This will ask the object to create a RDK2::Meta::ObjectUpdate based on the Change.
		virtual void pushUpdate(Url, RObjectChange*change)
			throw (NoSuchProperty, InvalidOperation) = 0;

		/// Similar to queueFreeze().
		virtual std::vector<const RDK2::Meta::ObjectUpdate*> updatesFreeze(Url)
			throw (NoSuchProperty, InvalidOperation, SyncError) = 0;
	//@}
	
	/// @name Storage subscription
	/// Subscriptions for storage (subscription = 'I will take the value/updates of this property')
	//@{
		
		/// Subscribe/Unsubscribe for storage values.
		///
		/// @throw NoSuchProperty   if url does not exist
		/// @throw InvalidOperation if url is not storage
		virtual void subscribeStorageValue(Url)
			throw (NoSuchProperty, InvalidOperation) = 0;
		virtual void unsubscribeStorageValue(Url)
			throw (NoSuchProperty, InvalidOperation) = 0;
		
		/// Subscribe/Unsubscribe for storage updates.
		///
		/// @throw NoSuchProperty   if url does not exist
		/// @throw InvalidOperation if url is not storage
		virtual void subscribeStorageUpdates(Url)
			throw (NoSuchProperty, InvalidOperation) = 0;
		virtual void unsubscribeStorageUpdates(Url)
			throw (NoSuchProperty, InvalidOperation) = 0;
	//@}
		
	/// @name Queue creation
	/// Queue creation
	//@{	
		/// Create a Queue
		///
		/// @throw InvalidOperation  if resource already present
		/// @throw InvalidOperation  if url is not relative (or contains "..").
		virtual void queueCreate(Url, cstr description)
			throw(InvalidOperation) = 0;
	//@}
			
		
	/// @name Queue subscription
	/// Queue subscription.
	//@{		
		/// Subscriptions types for queue
		Enum QueueSubscription {
			QUEUE_LAST_OBJECT, ///< I just need the last
			QUEUE_ALL          ///< Please send every object
		};
		
		/// Subscribe for queue updates.
		/// Note: se la Queue �rediretta si riferisce alla rediretta?
		///
		/// @throw NoSuchProperty   if url does not exist
		/// @throw InvalidOperation if url is not Queue
		virtual void subscribeQueue(Url, QueueSubscription qs = QUEUE_ALL)
				throw (NoSuchProperty, InvalidOperation) = 0;
				
		virtual void unsubscribeQueue(Url)
				throw (NoSuchProperty, InvalidOperation) = 0;
		
		/// Set filter on objects. The caller continues to own the pointer i.
		///
		/// @throw NoSuchProperty   if url does not exist
		/// @throw InvalidOperation if url is not Queue
		virtual void queueSetInterests(Url, PosixQueues::MyInterests<RDK2::Object>*i)
			throw (NoSuchProperty, InvalidOperation) = 0;
	//@}
		
	/// @name Queue reading
	/// Queue reading
	//@{
		
		/// Locks the queue for reading.
		///
		/// @throw  NoSuchProperty   if url does not exist
		/// @throw  InvalidOperation if url is not Queue
		/// @throw  InvalidOperation if you don't have subscribed the Queue
		virtual std::vector<const RDK2::Object*> queueFreeze(Url)
			throw (NoSuchProperty, InvalidOperation, SyncError) = 0;
	
		/// Keeps the object for next time.
		///
		/// @throw  NoSuchProperty   if url does not exist
		/// @throw  InvalidOperation if url is not Queue
		virtual void queueKeep(Url, const RDK2::Object*)
			throw (NoSuchProperty, InvalidOperation, SyncError) = 0;
		
		/// Ritorna l'ultimo oggetto nella coda
		///
		/// Se c'�mai stato un oggetto nella coda, allora
		/// l'ultimo oggetto �sempre conservato in modo che sia
		/// accessibile da queueLast()..
		///
		/// Il puntatore resta valido fino al prossimo queueLast().
		/// Si noti che questa modalit�di accesso �perpendicolare al queueFreeze.
		///
		/// @throw  NoSuchProperty   if url does not exist
		/// @throw  InvalidOperation if url is not Queue
		virtual const RDK2::Object* queueLast(Url);

	// @}
		
	/// @name Events
	/// Events
	//@{
		/// Listen to value, update, queue
		virtual void listen(Url) throw(NoSuchProperty) = 0;
		
		/// The "one and only" wait on the "one and only" semaphore
		/// @return events count
		virtual std::vector<const Event*> wait() throw() = 0;
		
		/// Similar to queueContent
		virtual std::vector<const Event*> events() = 0;
		
		/// This is needed to make a thread capable of waiting on
		/// other things in addition to the session. With this function the 
		/// caller can use the session semaphore for other things.
		/// The pointer is property of the Session.
		virtual PosixConstructs::PosixSemaphore* getMySemaphore() = 0;
	//@}
	
	}; // class SessionInterfaceCore

	/** Shortcut, template e roba simile che utilizzano altre funzioni di base.
	Tutte le funzioni qui non sono virtual perch�possono essere implementate
	basandosi su quelle di SessionInterface. Sono messe qui per chiarezza.*/
	struct SessionInterface: public SessionInterfaceCore {
		
		/// If value is not set, it will return the default value.
		///
		/// @throw NoSuchProperty   if url does not exist
		/// @throw SyncError        if not locked
		/// @throw ValueNotSet      if url does not have value nor defaultValue
		virtual const RDK2::Object* getValueLC(Url) 
			throw (NoSuchProperty, InvalidOperation, SyncError) = 0;
	

	/// @name Ints
	//// Interi /////
	//@{
		void createInt(cstr name, cstr description, int value) 
			throw(InvalidOperation);
		
		/// @throw NoSuchProperty    if url does not exist
		/// @throw InvalidOperation  if url is not storage
		/// @throw WrongType         if url is not an RInt
		void setInt(Url, int) 
			throw(NoSuchProperty, InvalidOperation, WrongType);
		
		/// @throw NoSuchProperty    if url does not exist
		/// @throw InvalidOperation  if url is not storage
		/// @throw WrongType         if url is not an RInt
		int getInt(Url) 
			throw(NoSuchProperty, InvalidOperation, WrongType);
		
		/// Wraps @getIntL(Url)
		///
		/// @return   false   if getIntL(Url) returns exception.
		bool getIntB(Url, int&);
		
		/// @return   false   if setIntL(Url,int) returns exception.
		bool setIntB(Url, int);
		
		// ...
		void setValue(Url, RDK2::Object*);	// no copy is made
	//@}
	//// Fine interi /////
	
	//// double /////
	//@{
	//@}
	//// Fine double /////
	
	//// stringhe /////
	//@{
	//@}
	//// Fine stringhe /////
	
	//// pose /////
	//@{
	//@}
	//// Fine pose /////
	
	//// altro? /////
	// ...
	//// Fine altro /////

	//// code /////
	//@{
		/// Returns number of objects available for reading.
			///
			/// @throw  NoSuchProperty   if url does not exist
			/// @throw  InvalidOperation if url is not Queue
			virtual size_t queueSize(Url)
				throws(NoSuchProperty, InvalidOperation, SyncError) = 0;
		
			/// Ritorna l'i-esimo oggetto nella coda
			///
			/// @throw  NoSuchProperty   if url does not exist
			/// @throw  InvalidOperation if url is not Queue
			virtual const RDK2::Object* queueContent(Url, size_t i) 
				throw(NoSuchProperty, InvalidOperation, SyncError) = 0;
	//@}
	
		
	};


}};

