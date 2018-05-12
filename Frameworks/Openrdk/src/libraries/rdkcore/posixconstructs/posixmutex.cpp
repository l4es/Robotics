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

#include "posixmutex.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "PosixConstructs"

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <cstdio>

namespace PosixConstructs {
	
PosixMutex::PosixMutex(bool initMutexOnDemand): inited(false) {
	sprintf(reason, "%s", "first time");
	if(!initMutexOnDemand) 
		initMutex();
}

void PosixMutex::initMutex() const {
	if(inited) {
		RDK_ERROR_STREAM("Mutex already inited.");
		return;
	}
	inited = true;
	 
	pthread_mutexattr_t mta;
	int r = pthread_mutexattr_init(&mta);
	if(r) {
		RDK_ERROR_PRINTF("Could not pthread_mutexattr_init() (%s)", strerror(errno));
	}
	
	if(pthread_mutexattr_settype(&mta, PTHREAD_MUTEX_RECURSIVE)) {
		RDK_ERROR_STREAM("Could not pthread_mutexattr_settype()");
	}
	
	if(pthread_mutex_init(&m_mutex, &mta)) {
		RDK_ERROR_STREAM("Could not pthread_mutex_init");
	}
}

/// Quando il sake e' l'unica speranza, definire OSAKE
//#define OSAKE

#define PAZIENZA_USLEEP  50000
#define PAZIENZA_COUNT  20

#ifdef OSAKE

void PosixMutex::lock(Stack nextStack) const {
	if(!inited) initMutex();

	for(int count=0;1;count++) {
		int res = pthread_mutex_trylock(&m_mutex);
		
		if(EBUSY == res ) {
/*			if(count==0) {
				RDK_TRACE_STREAM("Locking at \n"<<nextStack.toString()<<
					" blocked because of \n"<< stack.toString() << ".");	
			}
			*/
			if(count>PAZIENZA_COUNT) {
				RDK_ERROR_STREAM("Locking for \n"<< nextStack.toString() <<
					" blocked WAY TOO LONG because of \n"<< stack.toString() << ".");
			}
			
		} else if (0 == res) break;
		else {
			RDK_ERROR_STREAM("Could not trylock()");
		}
		
 		usleep(PAZIENZA_USLEEP);
	}

	stack = nextStack;
}
#else
void PosixMutex::lock(Stack nextStack) const {
	if(!inited) initMutex();
	//RDK_DEBUG_PRINTF("Trylocking %p", &m_mutex);
	int res = pthread_mutex_trylock(&m_mutex);
	//RDK_DEBUG_PRINTF("Out of trylocking %p", &m_mutex);
	if(EBUSY == res ) {
/*				RDK_TRACE_STREAM("Locking at \n"<<nextStack.toString()<<
					" blocked because of \n"<< stack.toString() << ".");	*/
		//RDK_DEBUG_PRINTF("Locking %p", &m_mutex);
		res = pthread_mutex_lock(&m_mutex);
		//RDK_DEBUG_PRINTF("Out of locking %p", &m_mutex);
		
		if(EDEADLK == res){
				RDK_ERROR_STREAM("DEADLOCK !! - Locking at \n"<<nextStack.toString()<<
					" blocked because of \n"<< /*stack.toString() <<*/ ".");	
		} else if(res) {
			RDK_ERROR_STREAM("Could not lock() mutex");	
		}
	} else if(0 != res) {
		RDK_ERROR_STREAM("Could not trylock() ("<<res<<")");
	}

	//RDK_DEBUG_PRINTF("Out of lock! %p", &m_mutex);
	stack = nextStack;
}
#endif

void PosixMutex::lock(const char*reason, const char*file, int line) const {
	lock(Stack(file, "(unknown function)", line, reason));
}

void PosixMutex::unlock()  const {
	if(!inited) {
		RDK_ERROR_STREAM("Unlocking not existing mutex.");
		return;
	}
	
	pthread_mutex_unlock(&m_mutex);
}

PosixMutex::~PosixMutex() {
	if(inited) 
		pthread_mutex_destroy(&m_mutex);

}

PosixMutex::PosixMutex(const PosixMutex&) { // occhio
	inited = false;
}

const PosixMutex& PosixMutex::operator= (const PosixMutex & ) { // occhio
	return *this;
}

} // end namespace
