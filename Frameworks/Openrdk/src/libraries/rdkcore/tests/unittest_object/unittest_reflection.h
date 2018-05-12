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
#include <fstream>
#include <typeinfo>
#include <cppunit/TestCase.h>

#include <rdkcore/object/object.h>
#include <rdkcore/rprimitive/rint.h>

class RMia {
	int a;	
};

namespace RDK2 { namespace UnitTest { namespace Object {
	using RDK2::RPrimitive::RInt;
	
	class TestReflection1 : public CppUnit::TestCase { 
		public: 
			TestReflection1( std::string name) : CppUnit::TestCase(name) {}
		
			void runTest() {
				RInt rint;
				RMia rmia;
				
				cout << "RInt == " << rint.getClassName() << endl;
				cout << "RMia == " << RDK2::Demangle::demangle(typeid(rmia).name()) << endl;
			
				CPPUNIT_ASSERT_MESSAGE("Reflection does not work", 
					RDK2::Demangle::demangle(typeid(rmia).name()) == "RMia");
				
				CPPUNIT_ASSERT_MESSAGE("Reflection does not work", 
					rint.getClassName() == "RInt");
					
			}
	};

}}}
