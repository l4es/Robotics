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

#ifndef H_POSIXSET
#define H_POSIXSET

#include <set>
#include "posixmutex.h"

namespace PosixConstructs {

/** Simple thread-safe set class. All methods are thread-safe. */
template<class Y>
class PosixSet {
	std::set<Y> s;
	PosixMutex mutex;
	public:
		/** Toggles an element in the set. If it is present, it is removed and 
		vice-versa. Returns true if the element is now present in the set.
		Example:
		
			SimplePosixSet<int> set;
			set.toggle(42); // = true
			set.toggle(42); // = false
			set.toggle(42); // = true
		*/
		bool toggle(const Y&);
};

#include "posixset.cpp"

} // end namespace

#endif
