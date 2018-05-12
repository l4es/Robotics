/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi (<first_name>.<last_name>@dis.uniroma1.it)
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

#ifndef RDK2_SIMPLETHREADS_H
#define RDK2_SIMPLETHREADS_H

#include <pthread.h>

namespace SimpleThreads {

class Thread {
public:
	/// *structors
	Thread() : exiting(false) { }
	virtual ~Thread() { }
	
	/// this will start the thread, the body of the thread is the exec function
	void start();
	
	/// this will stop the thread, i.e. set exiting to true, call exitRequested(), join the thread
	void stop();
	
protected:
	volatile bool exiting;
	virtual void exec() = 0;
	virtual void exitRequested() { }

private:
	static void* thread_exec(Thread* m);
	pthread_t threadId;
};

} // namespace

#endif
