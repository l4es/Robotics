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

#include <iostream>

#include <cppunit/ui/text/TestRunner.h> 
#include <cppunit/TestCase.h>

#include <rdkcore/geometry/utils.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "test"
#include <rdkcore/ns.h>

using namespace RDK2;
using namespace RDK2::Geometry;

template<class T> 
bool string2val2string(const std::string& s) {
	T v;
	if(!string2float(s, v)) {
		std::cout << "error: Cannot parse " << s << std::endl;
		return false;
	}
	std::string s2 = float2string(v);
	if(s != s2) {
		std::cout << "error: " << s << " != " << s2 << std::endl;
		return false;
	}
	std::cout << "s2v2s: " << s << " -> " << v << " -> " << s2 << std::endl;
	return true;
}

template<class T> 
bool val2string2val(T v, T epsilon=1e-10) {
	std::string s = float2string(v);
	T v2;
	if(!string2float(s, v2)) {
		std::cout  << "error: " << "Cannot parse " << s << std::endl;
		return false;
	}
	if(!sameFloat(v,v2,epsilon)) {
		std::cout  << "error: " << v << " -> " << s << " -> " << v2 << std::endl;
		return false;
	}
	std::cout << "v2s2v: " << v << " -> " << s << " -> " << v2 << std::endl;
	return true;
}

template<class T> 
class INFNANTest : public CppUnit::TestCase {
	void runTest() {
		CPPUNIT_ASSERT(true);
		
		sameFloat<T>(.0, .0);
		
		CPPUNIT_ASSERT(sameFloat<T>(.0, .0));
		CPPUNIT_ASSERT(sameFloat<T>(.0, .0, 0.1));
		CPPUNIT_ASSERT(sameFloat<T>(.0, .1, 0.11));
		CPPUNIT_ASSERT(!sameFloat<T>(.0, .1, 0.09));

		CPPUNIT_ASSERT(sameFloat<T>(RDK_NAN, RDK_NAN));
		CPPUNIT_ASSERT(sameFloat<T>(RDK_NAN, RDK_NAN, 0.0));
		CPPUNIT_ASSERT(!sameFloat<T>(RDK_NAN, 2.0));
		CPPUNIT_ASSERT(!sameFloat<T>(2.0, RDK_NAN));
		
		T inf = (sizeof(T)==4) ? (T) RDK_INF32 : (T) RDK_INF64;
			
		CPPUNIT_ASSERT(!sameFloat<T>(inf, 2.0));
		CPPUNIT_ASSERT(!sameFloat<T>(2.0, inf));
		CPPUNIT_ASSERT(!sameFloat<T>(RDK_NAN, inf));
		CPPUNIT_ASSERT(!sameFloat<T>(inf, RDK_NAN));
		CPPUNIT_ASSERT(sameFloat<T>(inf, inf));
	
		CPPUNIT_ASSERT(val2string2val<T>(RDK_NAN));
		CPPUNIT_ASSERT(val2string2val<T>(inf));
		CPPUNIT_ASSERT(val2string2val<T>(42));

		CPPUNIT_ASSERT(string2val2string<T>("31"));
		CPPUNIT_ASSERT(string2val2string<T>("0"));
		CPPUNIT_ASSERT(string2val2string<T>(RDK_INF_STRING));
		CPPUNIT_ASSERT(string2val2string<T>(RDK_NAN_STRING));
		CPPUNIT_ASSERT(!string2val2string<T>("okok"));
		CPPUNIT_ASSERT(!string2val2string<T>(""));
		CPPUNIT_ASSERT(RDK_ISNAN(RDK_NAN));
		CPPUNIT_ASSERT(!RDK_ISNAN(inf));
		CPPUNIT_ASSERT(!RDK_ISINF(RDK_NAN));
		CPPUNIT_ASSERT(!RDK_ISNAN(12.0));
		CPPUNIT_ASSERT(!RDK_ISINF(13.0));
	}
};

int main() {	
	CppUnit::TextUi::TestRunner runner;
	runner.addTest( new INFNANTest<double>() );
	runner.addTest( new INFNANTest<float>() );
	bool wasSuccessful = runner.run( "", false );
	return !wasSuccessful;
}
