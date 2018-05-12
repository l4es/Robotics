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

#include "stack.h"
#include <sys/types.h>
#include <sstream>

namespace PosixConstructs {

string Context::toString()
{
	ostringstream oss;
	oss << file << ":" << line << ", " << function << "(), reason '" << why << "'";
	return oss.str();
}

Stack operator+(const Stack&a,const Stack&b) {
	std::vector<Context> stack;
	stack.insert(stack.end(),a.stack.begin(), a.stack.end());
	stack.insert(stack.end(),b.stack.begin(), b.stack.end());
	return Stack(stack);
}

string Stack::toString()
{
	std::ostringstream oss; 
	
	oss << "\t Stack with " << stack.size() << " contexts: \n";
	
	for (size_t i = 0; i < stack.size(); i++) {
		oss << "\t " << i << "  - " << 
		 stack[i].toString() + "\n";
	}
	return oss.str();
}

} // namespace

