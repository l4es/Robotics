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

#include <stdio.h>
#include <unistd.h>

#include "profiling.h"
using namespace RDK2::Profiling;

int main(int, char**)
{
	Profiler::open("test_profiling.log");
	Profiler::start("uno");
	Profiler::end("uno");
	Profiler::start("due");
	usleep(100);
	Profiler::end("due");
	Profiler::start("tre");
	sleep(1);
	Profiler::start("quattro");
	Profiler::start("cinque");
	usleep(1234);
	Profiler::end("cinque");
	usleep(3240);
	Profiler::end("quattro");
	return 0;
}
