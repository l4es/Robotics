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

#include "posixsem.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "PosixSem"

#include <errno.h>
#include <string.h>

namespace PosixConstructs {

#if defined(LINUX) || defined(CYGWIN)
	
LinuxSemaphore::LinuxSemaphore()
{
	//RDK_DEBUG_PRINTF("Initializing semaphore %p", &sem, strerror(errno));
	int r = sem_init(&sem, 0, 0);
	if(r) {
		RDK_ERROR_PRINTF("Could not init semaphore (%s)", r, strerror(errno));
	}
}

LinuxSemaphore::~LinuxSemaphore()
{
	//RDK_DEBUG_PRINTF("Destroying %p", &sem);
	sem_destroy(&sem);
}

void LinuxSemaphore::wait()
{
	//RDK_DEBUG_PRINTF("Sem %p waiting", &sem);
	sem_wait(&sem);
	//RDK_DEBUG_PRINTF("Sem %p awake", &sem);
}

void LinuxSemaphore::signal()
{
	//RDK_DEBUG_PRINTF("Signalling sem %p", &sem);
	sem_post(&sem);
	//RDK_DEBUG_PRINTF("Signaled sem %p", &sem);
}

#endif

// #ifdef CYGWIN
//
// WindowsSemaphore::WindowsSemaphore() {
//	sem = CreateSemaphore(NULL, 0, MAX_SEM_COUNT, NULL);
//	if (sem == NULL) {
//		RDK_ERROR_PRINTF("Cannot create semaphore (%d)", GetLastError());
//	}
// }
//
// void WindowsSemaphore::wait() {
//	WaitForSingleObject(sem, 0L);
// }
//
// void WindowsSemaphore::signal() {
//	ReleaseSemaphore(sem, 1, NULL);
// }
//
// #endif

#ifdef MACOSX
	
DarwinSemaphore::DarwinSemaphore() {
	semaphore_create(mach_task_self(), &sem, SYNC_POLICY_FIFO, 0);
}

void DarwinSemaphore::wait() {
	int count = 4;
	while (semaphore_wait(sem) && count-->0) {
		RDK_ERROR_STREAM("Could not wait() on semaphore: "<<strerror(errno)<<", waiting other " << count << "times");
	}
}

void DarwinSemaphore::signal() {
	RDK_TRACE_STREAM("Signaling");
	semaphore_signal(sem);
}

#endif
	
};
