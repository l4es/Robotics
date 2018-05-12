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

#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>
#include <iostream>
#include <sys/select.h>
#include <cstdio>

#include <fcntl.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "PosixQueues"

using namespace std;

namespace PosixQueues {

	bool fd_ready(int fd, int microsec) {
//#ifndef CYGWIN
		int res;
		fd_set to_read;
		struct timeval timeout;

		FD_ZERO(&to_read);
		FD_SET(fd, &to_read);
		timeout.tv_sec  = 0;
		timeout.tv_usec = microsec;

		res = select(fd+1, &to_read, NULL, NULL, &timeout);
		if(res==-1) {
			// PRINT(PANIC, "select() failed.");
			 return false;
		}
		else if(res==0) {
			 /* PRINT(ERROR, "TIMEOUT. no available packets from dummypic.");*/
			 return false; //return 0;
		}
		else return true;
//#else
//	return true;
//#endif
	}

	bool writePointer(int fd, void*x) {
		size_t len = sizeof(void*);
		int res = write(fd, &x, len);
		if(res!=(int)len) {
			cerr << "Could not write to pipe: " << strerror(errno) << ". fd="<<fd<<" res="<<res<<endl;
			return false;
		}
		return true;
	}

	bool readPointer(int fd, void**x) {
		if(!fd_ready(fd,0)) return false;
		// ipotesi � che 4 byte siano atomici
		void *p;
		int res = read(fd, &p, sizeof(void*));
		if(res==-1) {
			cerr << "Could not read: " << strerror(errno) << ". fd="<<fd<<endl;
			return false;
		}
		if(res==0) {
			cerr << "Could not read from pipe: " << strerror(errno) << ". fd="<<fd<<" res="<<res<<endl;
			return false;
		}
		if(res==sizeof(void*)) {
			*x=p;
			return true;
		}
		cerr << "Read partial " << res << ". fd="<<fd<<endl;
		return false;
	}


	bool setNonBlocking(int fd) {
		int flags  = fcntl(fd, F_GETFL);
		if(flags == -1) {
			fprintf(stderr, "Could not get FLAGS for pipe: %s \n", strerror(errno));
			return false;
		}

		//fprintf(stderr, "GET FLAGS : %d - %X\n", err, flags);

		flags |= O_NONBLOCK;
		int err = fcntl(fd, F_SETFL, flags);

		if(err == -1) {
			fprintf(stderr, "Could not set O_NONBLOCK for pipe: %s \n", strerror(errno));
			return false;
		}

		return true;
	}


	void logIt(const std::string& /*s*/) {
		//RDK_DEBUG_STREAM(s);
	}
}
