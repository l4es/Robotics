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

#include <stdarg.h>
using namespace std;

#include <rdkcore/time/time.h>
#include "logfile.h"

namespace RDK2 {

using namespace RDK2::Time;

LogFile::LogFile(const string& filenamePrefix, bool appendTimeStamp)
{
	filename = filenamePrefix;
	if (appendTimeStamp) {
		Timestamp ts;
		filename = filename + ts.getStringRepresentation();
	}
	filename += ".log";
	open(filename.c_str(), ios::trunc);
}

LogFile::~LogFile()
{
	close();
}

void LogFile::printf(const char* printfFormat, ...)
{
	#define BUFSIZE 1024
	char buf[BUFSIZE + 1];
	
	va_list ap;
	va_start(ap, printfFormat);
	vsnprintf(buf, BUFSIZE, printfFormat, ap);
	va_end(ap);

	*this << string(buf);
}

} // namespace
