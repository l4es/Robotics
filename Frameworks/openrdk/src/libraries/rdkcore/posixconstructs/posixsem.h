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

#ifndef H_POSIXSEM
#define H_POSIXSEM

#if defined(LINUX) || defined(CYGWIN)
#include <semaphore.h>
#endif

// #ifdef CYGWIN
// #include <windows.h>
// #endif

#ifdef MACOSX
#include <mach/semaphore.h>
#include <mach/task.h>
#include <mach/mach.h>
#endif

namespace PosixConstructs {

#if defined(LINUX) || defined(CYGWIN)
	class LinuxSemaphore {
		sem_t sem;
	public:
		LinuxSemaphore();
		~LinuxSemaphore();
		void wait();
		void signal();
	};
	
	typedef LinuxSemaphore PosixSemaphore;
#endif
	
#ifdef MACOSX
	class DarwinSemaphore {
		static int count;
		semaphore_t sem;
	public:
		DarwinSemaphore();
		void wait();
		void signal();
	};
	
	typedef DarwinSemaphore PosixSemaphore;
#endif

// #ifdef CYGWIN
//	class WindowsSemaphore {
//		HANDLE sem;
//	public:
//		WindowsSemaphore();
//		void wait();
//		void signal();
//	};
//	
//	typedef WindowsSemaphore PosixSemaphore;
// #endif

} // end namespace

#endif
