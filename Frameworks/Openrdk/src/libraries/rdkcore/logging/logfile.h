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

#ifndef H_LOGFILE
#define H_LOGFILE

#include <string>
#include <fstream>

namespace Logging {

using namespace std;

class LogFile {
public:
	enum LogFileOptions { NORMAL = 0, TIMESTAMP_ON_FILENAME = 1, APPEND = 2 };
	LogFile(const string& fileName, LogFileOptions options = NORMAL);
	~LogFile();

	void logPrintf(const char* printfFormat, ...);

protected:
	fstream fs;
};

#define LOGFILE_OPEN(logFile, name, options) \
	logFile = new Logging::LogFile(std::string(RDK_ROOT) + "/logs/" \
	+ std::string(LOGGING_MODULE) + "-" + name, options)

#define LOGFILE_CLOSE(logFile) \
	delete logFile

#define LOGFILE_PRINTF(logFile, format, args...) \
	{ \
	logFile->log_printf(format, ## args); \
	}

#define LOGFILE_STREAM(logFile, stream_expr) \
	{ \
	std::ostringstream s; \
	s << stream_expr; \
	logFile->log_printf(s.str().c_str()); \
	}

#define GLOBALLOG_ENABLE(enable)

#define GLOBALLOG_OPENFILE

#define GLOBALLOG_EVENT_BEGIN(eventName)

#define GLOBALLOG_EVENT_END(eventName)

#define GLOBALLOG_CLOSEFILE

} // namespace

#endif
