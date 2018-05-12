/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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

#include "logfile.h"

namespace Logging {

enum LogFileOptions { NORMAL = 0, TIMESTAMP_ON_FILENAME = 1, APPEND = 2 };

LogFile::LogFile(const string& fileName, LogFileOptions options)
{
	fs.open(fileName.c_str(), (options & APPEND ? ios::app : ios::trunc));
}

LogFile::~LogFile()
{
	fs.close();
}

void LogFile::logPrintf(const char* printfFormat, ...)
{
	#define BUFSIZE 1024
	char buf[BUFSIZE + 1];
	
	va_list ap;
	va_start(ap, printfFormat);
	vsnprintf(buf, BUFSIZE, printfFormat, ap);
	va_end(ap);

	fs << string(buf) << endl;
}

} // namespace
