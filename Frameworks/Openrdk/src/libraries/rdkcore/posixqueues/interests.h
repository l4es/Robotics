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

#ifndef H_INTERESTS
#define H_INTERESTS

namespace PosixQueues {
	
template<class X> 
struct MyInterests {
	virtual bool areYouInterested(const X*) = 0;
	virtual ~MyInterests() {}
};

template<class X, class Y>
struct InterestedInClass : public MyInterests<X> {
	virtual bool areYouInterested(const X*x) {
		return 0 != dynamic_cast<Y*>(x);	
	}	
};

template<class X>
struct LikeEverything: public MyInterests<X> {
	bool areYouInterested(const X*) {
		return true;	
	}
};

template<class X>
struct SedicenneMetallaro: public MyInterests<X> {
	bool areYouInterested(const X*) {
		return false;	
	}
};

} // end namespace

#endif
