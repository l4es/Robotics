/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007  Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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

#ifndef H_QUEUEINTERFACE
#define H_QUEUEINTERFACE

#include <vector>
#include <rdkcore/posixconstructs/posixsem.h>
#include "interests.h"

void queueNoNo(const char*);

namespace PosixQueues {

template<class X>
class ConsumerQueueInterface;

template<class X>
class MasterQueueInterface {
	public:
	typedef int Id;

	/// Producer interface for queue

	/** Adds a pointer to the queue. The caller loses propriety of the pointer.
		Can be called from any contest (thread safe). Does not block in any case
		even if one of the queue is locked. */
	virtual bool push_back(X*) =0;
	

	/** No pushing while push_lock - it is useful for changing interests */ 	
	virtual void push_lock(const char*why, const char*file, int line) = 0;
	virtual void push_unlock() = 0;

	
	/// Consumer interface for queue.
	virtual Id getNewId(
		MyInterests<X>*interests=0, 
		PosixConstructs::PosixSemaphore* semaphore=0) = 0;
	
	virtual ConsumerQueueInterface<X> * getQueue(Id) = 0;
	/** Deletes a queue. No need to call push_lock(). */
	virtual void  deleteQueue(Id) = 0;

	virtual ~MasterQueueInterface() {}
};
	

template<class X>
class ConsumerQueueInterface {
	public:
	
	//MasterQueueInterface<X> * getMasterQueue();
	/// Basic interface
	
	/** Waits for new data to come, using a local semaphore. */
	virtual void wait() = 0;

	virtual void lock(const char*why, const char*file, int line) = 0;
		/// These methods should be called only when lock acquired. If
		/// lock acquired these methods do not block.
		virtual int size() = 0;
		virtual const X* first() = 0;
		virtual void discardFirst() = 0;
		void discardAll() { while(size()) discardFirst(); }
		
	virtual void unlock() = 0;

	
	/// Advanced interface
	
	/** This is needed to make a thread capable of waiting on
	multiple queues. The caller continues to own the pointer. */
	virtual void setSignalSemaphore(PosixConstructs::PosixSemaphore* semaphore) = 0;
	/** Revert to original policy */
	virtual void useLocalSemaphore() = 0;

	/** Set filter on objects. The caller continues to own the pointer. */
	virtual void setInterests(MyInterests<X>*i) = 0;

	virtual ~ConsumerQueueInterface() {}
	
	/** Utilities */
	template<class Y> const Y* firstAs() {
		if(!size()) {
			queueNoNo("Warning, cannot firstAs if size is 0");
			return 0;
		 }
		return dynamic_cast<const Y*>(first());
	}
	
	
	/** Thread-safe function to clone content with bit-a-bit copy*/
	template<class Y>
	std::vector<Y> cloneContentL() {
		std::vector<Y> v;
		LOCK(this, "cloning content");
		v.reserve(size());
			while(size()) {
				const Y * xp = firstAs<Y>();
				if(xp) v.push_back(*xp);
				discardFirst();
			}
		//q->dischargeSemaphore();
		unlock();
		return v;
	}
#if 0
	template<class Y>
	std::vector<std::auto_ptr<Y> > cloneContentWithCopyConstructor() {
		std::vector<std::auto_ptr<Y> > v;
		LOCK(this, "cloning content");
		v.reserve(size());
			while(size()) {
				const Y * xp = firstAs<Y>();
				if(xp) {
					Y * clone = new Y(xp); 
					std::auto_ptr<Y> p(clone);
					v.push_back(p);
				}
				discardFirst();
			}
		//q->dischargeSemaphore();
		unlock();
		return v;
	}
#endif
};


} // end namespace

#endif
