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

#include <errno.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Threads"

#include "threads.h"

namespace RDK2 { namespace Common {

bool createThread(pthread_t* threadId, void*(*threadFunction)(void*),
	void* functionParam, const std::string& threadName)
{
	int error_code;
	error_code = pthread_create(threadId, 0, threadFunction, functionParam);
	if (error_code != 0) {
		if (error_code == EAGAIN) {
			RDK_ERROR_PRINTF("Failed to launch '%s' (too many active threads)", threadName.c_str());
			return false;
		}
		else {
			RDK_ERROR_PRINTF("Failed to launch '%s' (cause unknown)", threadName.c_str());
			return false;
		}
	}
	else return true;
}

}} // namespaces
