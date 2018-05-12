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

#ifndef DEBUGLEVEL_H
#define DEBUGLEVEL_H

// the less the LOGGING_DEBUG_LEVEL, the less debug information you will receive: use 0 for no debug information at all
// when you put debug messages, using RDK_DEBUGL_PRINTF, use levels from 1 to whatever you want:
// 1 means a very important debug info, 2 is less important, etc

#define RDK_DEBUGL_PRINTF(l, s, args...) if (l <= LOGGING_DEBUG_LEVEL) RDK_DEBUG_PRINTF(s, ##args)
#define LOGGING_DEBUG_LEVEL 0

#endif
