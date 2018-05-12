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

#ifndef H_POSIXCON_STACK
#define H_POSIXCON_STACK

#include <vector>
#include <string>
#include <rdkcore/textutils/textutils.h>
#include <sstream>

namespace PosixConstructs {

	using std::string;
	using RDK2::TextUtils::cstr;
	using std::ostringstream;

struct Context {
	string file, function; int line; string why;

	Context(cstr file, cstr function, int line, cstr why):
		file(file), function(function), line(line), why(why) { }

	string toString();
};

struct Stack {
	std::vector<Context> stack;

	Stack(cstr file, cstr function, int line, cstr why) {
		stack.push_back(Context(file,function,line,why));
	}
	Stack() { }

	Stack(std::vector<Context> stack): stack(stack) {}

	string toString();
};

Stack operator+(const Stack&a, const Stack&b);

#define WHERE(why) PosixConstructs::Stack(__FILE__, __FUNCTION__, __LINE__, why)
#define HERE PosixConstructs::Stack(__FILE__, __FUNCTION__, __LINE__, "(?)")

} // namespace

#endif
