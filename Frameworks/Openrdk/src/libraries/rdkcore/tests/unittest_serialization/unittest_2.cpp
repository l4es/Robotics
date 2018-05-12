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

#include <rdkcore/serialization_binary/binaryreader.h> 
#include <rdkcore/serialization_binary/binarywriter.h> 

#include <rdkcore/rprimitive/rdouble.h>
#include <rdkcore/rprimitive/rbool.h>
#include <rdkcore/rprimitive/rint.h>
#include <rdkcore/rprimitive/rstring.h>

#include <cppunit/ui/text/TestRunner.h> 
#include "test2.h"

using namespace RDK2;
using namespace RDK2::UnitTest::Serialization;

using namespace RDK2::Serialization;
using namespace RDK2::Serialization::Binary;
using namespace RDK2::RPrimitive;

int main()
{
	BinaryWriter bw(true);
	BinaryReader br;
	
	CppUnit::TextUi::TestRunner runner;

	#define ADDTEST(class, constructorParams, file) \
	runner.addTest(\
		new SerializationTest2(#class , new class(constructorParams), \
		&bw, &br, prefix+file+".bin") \
	);
	
	std::string prefix("unittest_2_output/");
	
	ADDTEST(RDouble, 22, "rdouble");
	ADDTEST(RInt, 22,"rint");
	ADDTEST(RBool, false, "rbool0");
	ADDTEST(RBool, true, "rbool1");
	ADDTEST(RString, "hello", "rstring")

	// con caratteri nulli
	std::string s("hello"); s.at(2) = 0;
	ADDTEST(RString, s, "rstringnull")
	
	
	return !runner.run();
}
