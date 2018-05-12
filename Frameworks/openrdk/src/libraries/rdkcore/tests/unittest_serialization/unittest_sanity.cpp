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

#include <rdkcore/serialization/write.h> 
#include <rdkcore/serialization/read.h> 

#include <cppunit/TestCase.h>
#include <cppunit/ui/text/TestRunner.h>

namespace RDK2 { namespace UnitTest { namespace Serialization {

	class TestSizes : public CppUnit::TestCase { 
		public: 
			TestSizes( std::string name) : CppUnit::TestCase(name) {}
		
			void runTest() {
				
				const char * MESSAGE = "Bad sizes";
				
				CPPUNIT_ASSERT_MESSAGE(MESSAGE, sizeof(int8_t) == 1);
				CPPUNIT_ASSERT_MESSAGE(MESSAGE, sizeof(uint8_t) == 1);
				CPPUNIT_ASSERT_MESSAGE(MESSAGE, sizeof(int16_t) == 2);
				CPPUNIT_ASSERT_MESSAGE(MESSAGE, sizeof(int32_t) == 4);
				CPPUNIT_ASSERT_MESSAGE(MESSAGE, sizeof(float32_t) == 4);
				CPPUNIT_ASSERT_MESSAGE(MESSAGE, sizeof(float64_t) == 8);
				
			}
	};
	
}}}
	
int main()
{
	CppUnit::TextUi::TestRunner runner;
	
	runner.addTest(new RDK2::UnitTest::Serialization::TestSizes("Size test"));	
	return !runner.run();
}
