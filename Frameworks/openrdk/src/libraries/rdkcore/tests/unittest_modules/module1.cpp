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

#include <string>
using namespace std;

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "Module1"

#include <modules/module.h>
using RDK2::Modules::Module;

#include "module1.h"

Module1::Module1(const string name, int behaviour) : Module("Module1", name), behaviour(behaviour) { }

Module1::~Module1() { }

void Module1::init()
{
	RDK_DEBUG_PRINTF("Initializing (behaviour %d)", behaviour);
	switch (behaviour) {
	case 0: break;
	case 1: sleep(1); break;
	case 2: sleep(2); break;
	}
}

void Module1::exec()
{
	if (behaviour == 2) sleep(2);
	while (!exiting) {
		RDK_DEBUG_PRINTF("Executing behaviour %d", behaviour);
		sleep(1);
	}
	if (behaviour == 0) sleep(2);
}

void Module1::exitRequested()
{
	if (behaviour == 1) sleep(2);
}

void Module1::cleanup()
{
	RDK_DEBUG_PRINTF("Cleaning up module");
	if (behaviour == 2) sleep(1);
}

