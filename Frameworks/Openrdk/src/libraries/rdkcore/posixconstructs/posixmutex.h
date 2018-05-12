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

#ifndef H_POSIXMUTEX
#define H_POSIXMUTEX

#include <pthread.h>
#include "stack.h"

namespace PosixConstructs {

/** Note: a mutex locked by a thread MUST be unlocked by the same thread **/

class PosixMutex {
	private:
		mutable pthread_mutex_t m_mutex;
		mutable bool inited;
		mutable char reason[1000];
	
		mutable Stack stack;

	public:
		PosixMutex(bool initMutexOnDemand = true);
		PosixMutex(const PosixMutex&);
		const PosixMutex& operator=(const PosixMutex & rhs);
		~PosixMutex();
			
		void lock(const char* why, const char* file, int line) const;
		void lock(Stack) const;
		void unlock() const;

	private:
		void initMutex() const;
};

#define LOCK(lockable, why) (lockable)->lock(why, __FILE__, __LINE__)
#define UNLOCK(lockable) lockable->unlock()

#define LOCKR(lockable, why) (lockable).lock(why, __FILE__, __LINE__)
#define UNLOCKR(lockable) lockable.unlock()

} // end namespace

#endif

