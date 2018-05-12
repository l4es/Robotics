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

#ifndef H_LOGGING
#define H_LOGGING

#include <string>
#include <sstream>

namespace Logging {

/**
   Clients of this logging library should define LOGGING_MODULE and use
   one of the following macros.

   Note that each macro includes a { } block. So

	if(...)
		RDK_LOG(...);  // note the ';'
	else
	...

	will be expanded to

	if(...)
		{
			// logging code here
		};  // note the ';'!!!
	else
	...

	which is not legal. You should write
	
	if(...) { // additional { } block
		RDK_LOG(...);  // note the ';'
	} else
	...
*/

using std::string;

enum LogLevel { Trace=0, Debug=1, Info=2, Warning=3, Error=4, Fatal=5 };

void log_message(
	int           level,
	const string& module,
	const string& file,
	int           line,
	const string& function,
	const string& prettyFunction,
	const string& message);

void log_message(
	int           level,
	const string& module,
	const string& file,
	int           line,
	const string& function,
	const string& prettyFunction,
	const char*   printf_format,
	... );


/* logging with std::string */
#define RDK_LOG(level, message) \
	{ Logging::log_message(     \
		level,                  \
		LOGGING_MODULE,         \
		__FILE__,               \
		__LINE__,               \
		__FUNCTION__,           \
		__PRETTY_FUNCTION__,    \
		message);               \
	}

/* printf-style macros */
#define RDK_PRINTF(level, format, args...) \
	{ Logging::log_message(     \
		level,                  \
		LOGGING_MODULE,         \
		__FILE__,               \
		__LINE__,               \
		__FUNCTION__,           \
		__PRETTY_FUNCTION__,    \
		format , ## args  );    \
	}

/* stream oriented macros */
#define RDK_STREAM(level, stream_expr) \
	{                           \
		std::ostringstream oss; \
		oss << stream_expr;     \
		Logging::log_message(   \
		level,                  \
		LOGGING_MODULE,         \
		__FILE__,               \
		__LINE__,               \
		__FUNCTION__,           \
		__PRETTY_FUNCTION__,    \
		oss.str() );            \
	}

#define RDK_TRACE_PRINTF(format, args...)   RDK_PRINTF(Logging::Trace, format , ## args)
#define RDK_DEBUG_PRINTF(format, args...)   RDK_PRINTF(Logging::Debug, format , ## args)
#define RDK_INFO_PRINTF(format, args...)    RDK_PRINTF(Logging::Info, format , ## args)
#define RDK_WARNING_PRINTF(format, args...) RDK_PRINTF(Logging::Warning, format , ## args)
#define RDK_ERROR_PRINTF(format, args...)   RDK_PRINTF(Logging::Error, format , ## args)

#define RDK_TRACE_STREAM(expr)   RDK_STREAM(Logging::Trace, expr)
#define RDK_DEBUG_STREAM(expr)   RDK_STREAM(Logging::Debug, expr)
#define RDK_INFO_STREAM(expr)    RDK_STREAM(Logging::Info, expr)
#define RDK_WARNING_STREAM(expr) RDK_STREAM(Logging::Warning, expr)
#define RDK_ERROR_STREAM(expr)   RDK_STREAM(Logging::Error, expr)

} // namespace Logging

#endif
