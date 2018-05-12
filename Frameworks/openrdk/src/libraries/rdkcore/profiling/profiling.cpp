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

#include <sys/time.h>
#include <time.h>
#include <fstream>
#include <iostream>
using namespace std;

#include <rdkcore/posixconstructs/posixmutex.h>
using namespace PosixConstructs;

#include "profiling.h"

double cpu_time( void );

namespace RDK2 { namespace Profiling {

ofstream ofs;
bool doProfile = false;

PosixConstructs::PosixMutex mutex;

clock_t startClock;
struct timeval startTimestamp;

int getTimestamp()
{
	static struct timeval a;
	gettimeofday(&a, 0);
	a.tv_sec -= startTimestamp.tv_sec;
	a.tv_usec -= startTimestamp.tv_usec;
	return a.tv_sec * 1000000 + a.tv_usec;
}

// Returns cpu time in microseconds
int getClock()
{
	return (clock() - startClock) * 1000000 / CLOCKS_PER_SEC;
}

int getClockBetter()
{
	return (int)(cpu_time() * 1000*1000);
}

void Profiler::open(const char* filename)
{
	ofs.open(filename, ios::trunc | ios::out);
	doProfile = true;
	gettimeofday(&startTimestamp, 0);
	startClock = clock();
}

void Profiler::start(const char* name)
{
	addCustomLine("START", name);
}

void Profiler::end(const char* name)
{
	addCustomLine("END", name);
}

void Profiler::lock(const char* name)
{
	addCustomLine("LOCK", name);
}

void Profiler::unlock(const char* name)
{
	addCustomLine("UNLOCK", name);
}

void Profiler::addCustomLine(const char* what, const char* name, const char* otherInfo)
{
	if (!doProfile) return;
	mutex.lock(HERE);
	ofs << getTimestamp() << " " << getClock() << " " << getClockBetter() << " " << pthread_self() << " ";
	ofs << what << " " << name << " " << otherInfo << endl;
	mutex.unlock();
}

}} // namespaces
