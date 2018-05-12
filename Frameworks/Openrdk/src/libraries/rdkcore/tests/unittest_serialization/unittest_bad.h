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

#ifndef H_UNITTEST_BAD
#define H_UNITTEST_BAD

#include <iostream>
#include <rdkcore/serialization/read.h>
#include <rdkcore/serialization/write.h>
#include <fstream>
#include <cppunit/TestCase.h>

namespace RDK2 { namespace UnitTest { namespace Serialization {

	using namespace std;
	using namespace RDK2::Serialization;

	/**
	 * Asserts if there is no exception.   
	 *
	 */
	template<class X>
	class SerializationTestBad : public CppUnit::TestCase { 
		public: 
			X*x; Writer*w; Reader*r; 
			
			SerializationTestBad( std::string name,
			X*x, Writer*w, Reader*r) : CppUnit::TestCase( name ),
				x(x), w(w), r(r) {}
	
			void runTest() {
				try {
					// nota: true perch� abbiamo dei test che usano il nome
					// della classe
					 std::string buffer1 = w->serialize(true,x);	 
					 X x2; r->deserialize(buffer1, &x2);
					 string buffer2 = w->serialize(true,&x2);
					 
					 CPPUNIT_ASSERT_MESSAGE("I expected an assertion to be generated", false);
				} catch(SerializationException e) {
					cerr << "ok: " << e.what() << endl;	
				}
			}
	};
	
}}} // namespace RDK2::UnitTest::Serialization

#endif
