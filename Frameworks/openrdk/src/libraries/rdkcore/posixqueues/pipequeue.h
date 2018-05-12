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

#ifndef H_RDK2_QUEUES
#define H_RDK2_QUEUES

#include "interests.h"

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/posixconstructs/posixmutex.h>
#include <rdkcore/posixconstructs/posixsem.h>
#include <rdkcore/posixconstructs/stack.h>

#include <iostream>
#include <vector>
#include <map>
#include <deque>
#include <algorithm>

#include <unistd.h>

#define PIPEQUEUE_NOT_BLOCKING 1

namespace PosixQueues {

	using namespace PosixConstructs;
	using namespace RDK2::TextUtils;
	using std::vector;
	using std::map;
	using std::deque;
	using std::cerr;
	using std::endl;

	/** All functions must be called from the same thread. */
	template<class T>
	class PipeQueueConsumer {
	public:
		PipeQueueConsumer(int fd_in, int fd_out, const std::string&name):
			fd_in(fd_in), fd_out(fd_out), name(name), lastObject(0) {}
		~PipeQueueConsumer() { close(fd_in); close(fd_out); }

		vector<const T*> freeze();
		vector<const T*> content();
		const T* last();
		void keep(const T*t);

		vector<const T*> toKeep;
	private:
		int fd_in, fd_out;
		std::string name;
		vector<const T*> current;

		void flushOld(const vector<const T*>&v);
		vector<const T*> downloadNew();
		const T* lastObject;

		void flushPipeBuffer();
		deque<const T*> pipe_buffer;

		void debug_vector(const char*, const vector<const T*>&v);
		void debug_deque(const char*, const deque<const T*>&v);
		void debug_pointer(const char*, const T*);
	};

	/**

	All functions are thread safe.

	Garbage collection gc() must be called once in a while, for
	example after each push().

	Pushing does not block consumer from reading.
	*/
	template<class T> class PipeQueue {
	public:
		PipeQueue(const std::string& name): name(name) {}
		virtual ~PipeQueue() { }

		/// Creates a new consumer
		PipeQueueConsumer<T> *createConsumer(const std::string&name, Stack w);

		/// ob is now property of this queue
		size_t push(T*ob, Stack w);

		/// Return the size of queue
		size_t size();

		/// Runs garbage collection; to be called once in a while, for
		/// example after push
		void gc(Stack w);

		/// c is deleted and no longer valid
		void deleteConsumer(PipeQueueConsumer<T>*c, Stack w);

		/// This is needed to make a thread capable of waiting on
		/// multiple queues. The caller continues to own the pointer.
		/// If semaphore = 0 (default), no signal happens.
		void setSignalSemaphore(PipeQueueConsumer<T>*c, PosixConstructs::PosixSemaphore* semaphore);

		/// ...
		bool hasSignalSemaphore(PipeQueueConsumer<T>*c);

		/// Set filter on objects. The caller continues to own the pointer.
		void setInterests(PipeQueueConsumer<T>*c, MyInterests<T>*interests);

		virtual void push_callback(PipeQueueConsumer<T>*) { }

	private:
		// This mutex is for the following data structures
		PosixConstructs::PosixMutex mutex;
		// Reference counting
		typedef std::map<T*,int> Ptr2ref;
		Ptr2ref ptr2ref;
		// Subscriptions
		struct Subscription {
			int fd_in; int fd_out;
			MyInterests<T>*interests;
			PosixConstructs::PosixSemaphore *semaphore;
			// If the pipe is full, objects are accumulated here
			std::deque<const T*> buffer;
			string name;
		};
		typedef std::map<PipeQueueConsumer<T>*,Subscription> Con2sub;
		Con2sub subscriptions;
		std::string name;
	};

	bool writePointer(int fd, void*);
	bool readPointer(int fd, void**);
	bool setNonBlocking(int fd);

	void logIt(const std::string& s);

#include "pipequeue.hpp"

}

#endif
