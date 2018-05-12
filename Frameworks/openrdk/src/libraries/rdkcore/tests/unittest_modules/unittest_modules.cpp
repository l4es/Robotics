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

#include <modules/modulemanager.h>
#include "module1.h"

using namespace RDK2;
using namespace RDK2::Modules;

int main(int, char**)
{
	ModuleManager mm;
	Module1 m0("Zero", 0), m1("Uno", 1), m2("Due", 2);

	mm.addModule(&m0);
	mm.addModule(&m1);
	mm.addModule(&m2);

	mm.initAllModules();
	mm.startAllModules();
	sleep(3);
	mm.endAllModules();

	return 0;
}

