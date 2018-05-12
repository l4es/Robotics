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

#ifndef H_LOGGING_IMP
#define H_LOGGING_IMP

#include <vector>
#include "logging.h"

namespace Logging {

struct LogMessage {
	int    level;
	string module;
	string file;
	int    line;
	string function;
	string prettyFunction;
	string message;
	// long timestamp;
	// string host;
};

struct LogHandler {
	virtual ~LogHandler() {};
	virtual void handle(const LogMessage&) = 0;
};

void addLogHandler(LogHandler *);
void removeLogHandler(LogHandler*);

/** Distributes log to the registered handlers. */
void log_message(const LogMessage&);

} // namespace Logging

#endif
