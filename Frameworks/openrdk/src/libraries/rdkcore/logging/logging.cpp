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

#include "logging_imp.h"
#include <stdarg.h>
#include <stdio.h>
#include <strings.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <pthread.h>
#include "streamsink.h"
#include "levelfilter.h"

#define RDK_LOG_ENV   "RDK_LOG"
#define RDK_LOG_COLOR "RDK_LOG_COLOR"

pthread_mutex_t logging_mutex;

namespace Logging {

void log_message(
	int           level,
	const string& module,
	const string& file,
	int           line,
	const string& function,
	const string& prettyFunction,
	const string& message)
{
	LogMessage l;
		l.level          = level;
		l.module         = module;
		l.file           = file;
		l.line           = line;
		l.function       = function;
		l.prettyFunction = prettyFunction;
		l.message        = message;

	log_message(l);
}

void log_message(
	int           level,
	const string& module,
	const string& file,
	int           line,
	const string& function,
	const string& prettyFunction,
	const char*   printf_format,
	... )
{
	#define BUFSIZE 1024
	char buf[BUFSIZE+1];

	va_list ap;
	va_start(ap, printf_format);	
	vsnprintf(buf, BUFSIZE, printf_format, ap);
	va_end(ap);

	log_message(level,module,file,line,function,prettyFunction,string(buf));
}


typedef std::vector<LogHandler*> Handlers;
Handlers handlers;

void addLogHandler(LogHandler *h) {
	pthread_mutex_lock(&logging_mutex);
	handlers.push_back(h); // XXX non thread safe	
	pthread_mutex_unlock(&logging_mutex);
}

void removeLogHandler(LogHandler*) {
	// XXX
}

void log_message(const LogMessage&l) {
	static bool first_time = true;
	if(first_time) {
		pthread_mutex_init(&logging_mutex,NULL);
		first_time = false;
		/* Using color? */
		char * color_env = getenv(RDK_LOG_COLOR);
		bool color = NULL!=color_env && atoi(color_env);
		LogHandler * sink = new StreamSink(&(std::cerr), color);
		/* Getting log level from environment. */		
		char * l = getenv(RDK_LOG_ENV);
		
		LogLevel level =
		!l                     ? Logging::Info :
		!strcasecmp(l,"trace") ? Logging::Trace :
		!strcasecmp(l,"debug") ? Logging::Debug :
		!strcasecmp(l,"info" ) ? Logging::Info  :
		!strcasecmp(l,"warning" ) ? Logging::Warning :
		!strcasecmp(l,"error") ? Logging::Error : Logging::Info;
		
		LogHandler * filter = new LevelFilter(sink, level, true);
		addLogHandler(filter);
	}

	pthread_mutex_lock(&logging_mutex);
	Handlers::iterator i; // XXX non thread safe
	for(i=handlers.begin(); i!=handlers.end(); ++i) {
		(*i)->handle(l);
	}
	pthread_mutex_unlock(&logging_mutex);
}

} // namespace Logging
