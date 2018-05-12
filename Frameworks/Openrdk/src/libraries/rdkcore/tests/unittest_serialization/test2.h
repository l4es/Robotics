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
#include <string>
#include <cppunit/TestCase.h>

#include <rdkcore/serialization/read.h>
#include <rdkcore/serialization/write.h>
#include <rdkcore/object/object.h>

#include "utils.h"

namespace RDK2 { namespace UnitTest { namespace Serialization {
	
	using namespace RDK2::Serialization;
	using namespace std;
	
	
	/**
	 * - Serialize instance
	 * - Compare to file (create if not exists)
	 * - Create another instance with forName() 
	 * - Deserialize, serialize again
	 */
	 
	class SerializationTest2 : public CppUnit::TestCase { 
		public: 
			RDK2::Object*o; 
			Writer*w; 
			Reader*r; string file; 
			
			SerializationTest2(
			std::string name,
			RDK2::Object*o, Writer*w, Reader*r, cstr file) : CppUnit::TestCase( name ),
				o(o), w(w), r(r), file(file) {}
	
			void runTest() {
				string buffer1, buffer2; 
				
				try {
					 buffer1 = w->serialize(true,o);	 
				} catch(SerializationException e) {
					cerr << e.what() << endl;	
					CPPUNIT_ASSERT_MESSAGE("Exception during serialization", false);
				}
				
				CPPUNIT_ASSERT_MESSAGE("Output is different from file", 
					compareToFile(buffer1, file));
				
				RDK2::Object * o2;
				try {
					 Readable *readable = r->deserialize(buffer1);
					 CPPUNIT_ASSERT_MESSAGE("Null deserialization", readable);
					 o2 = dynamic_cast<RDK2::Object*>(readable);
					 CPPUNIT_ASSERT_MESSAGE("deserialization not an object?", o2);
				} catch(SerializationException e) {
					cerr << e.what() << endl;	
					CPPUNIT_ASSERT_MESSAGE("Exception during deserialization", false);
				}
				
				CPPUNIT_ASSERT_MESSAGE("not same class", 
					o2->getClassName() == o->getClassName()); 
				
				try {
					buffer2 = w->serialize(true,o2);
					CPPUNIT_ASSERT_MESSAGE("deserialization+serialization!=identity",
					 	buffer1==buffer2);
				} catch(SerializationException e) {
					cerr << e.what() << endl;	
					CPPUNIT_ASSERT_MESSAGE("Exception during second serialization", false);
				}
					
			}
			
	};

}}} // namespace
