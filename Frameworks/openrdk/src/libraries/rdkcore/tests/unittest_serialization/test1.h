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
#include <cppunit/TestCase.h>

#include <rdkcore/serialization/read.h>
#include <rdkcore/serialization/write.h>

#include "utils.h"
	
#include <cppunit/ui/text/TestRunner.h> 

#include "good_tests.h"

namespace RDK2 { namespace UnitTest { namespace Serialization {
	
	using namespace RDK2::Serialization;
	using namespace std;

	/**
	 * - Serialize
	 * - Compare to file (create if not exists)
	 * - Create another instance, deserialize, serialize, compare
	 * X should have empty constructor
	 */
	template<class X>
	class SerializationTest1 : public CppUnit::TestCase { 
		public: 
			X*x; Writer*w; Reader*r; string file; 
			
			SerializationTest1( std::string name,
			X*x, Writer*w, Reader*r, cstr file) : CppUnit::TestCase( name ),
				x(x), w(w), r(r), file(file) {}
	
			void runTest() {
				string buffer1, buffer2; X x2;
				
				try {
					 buffer1 = w->serialize(true,x);	 
				} catch(SerializationException e) {
					cerr << e.what() << endl;	
					CPPUNIT_ASSERT_MESSAGE("Exception during serialization", false);
				}
				
				 CPPUNIT_ASSERT_MESSAGE("Output is different from file",
					 	compareToFile(buffer1, file));
				
				try {
					 r->deserialize(buffer1, &x2);
				} catch(SerializationException e) {
					cerr << e.what() << endl;	
					CPPUNIT_ASSERT_MESSAGE("Exception during deserialization", false);
				}
				
				
				try {
					buffer2 = w->serialize(true,&x2);
					if(buffer1!=buffer2) {
						printf("First buffer (first time): '%s'\nSecond Buffer: '%s' (second time)", buffer1.c_str(), buffer2.c_str());
						CPPUNIT_ASSERT_MESSAGE("deserialization+serialization!=identity",
					 	buffer1==buffer2);
					}
				} catch(SerializationException e) {
					cerr << e.what() << endl;	
					CPPUNIT_ASSERT_MESSAGE("Exception during second serialization", false);
				}
			}
	};

	bool doAllTests(Writer &bw, Reader &br,cstr prefix,cstr suffix) {
		CppUnit::TextUi::TestRunner runner;

		runner.addTest(
			new SerializationTest1<EmptyClass>
			("case1", new EmptyClass(), &bw, &br, prefix+"emptyclass"+suffix)
		);

		runner.addTest(
			new SerializationTest1<IntWrapper>
			("case2", new IntWrapper(42), &bw, &br, prefix+"intWrapper"+suffix)
		);

		runner.addTest(
			new SerializationTest1<Complex1>
			("oneObject-withoutClassname", new Complex1(), &bw, &br, prefix+"complex1"+suffix)
		);

		runner.addTest(
			new SerializationTest1<Complex2>
			("oneObject-withClasname", new Complex2(), &bw, &br, prefix+"complex2"+suffix)
		);

		runner.addTest(
			new SerializationTest1<Complex4>
			("Complex4", new Complex4(), &bw, &br, prefix+"complex4"+suffix)
		);

		runner.addTest(
			new SerializationTest1<Complex6>
			("Complex6", new Complex6(43,44), &bw, &br, prefix+"complex6"+suffix)
		);

		runner.addTest(
			new SerializationTest1<Complex5>
			("Complex5", new Complex5(), &bw, &br, prefix+"complex5"+suffix)
		);

		runner.addTest(
			new SerializationTest1<Complex3>
			("Complex3", new Complex3(43,44), &bw, &br, prefix+"complex3"+suffix)
		);

		runner.addTest(
			new SerializationTest1<Complex7>
			("Complex7", new Complex7(43,44), &bw, &br, prefix+"complex7"+suffix)
		);

		runner.addTest(
			new SerializationTest1<EmptyArray>
			("EmptyArray", new EmptyArray(), &bw, &br, prefix+"emptyarray"+suffix)
		);

		runner.addTest(
			new SerializationTest1<VersionTest>
			("VersionTest", new VersionTest(), &bw, &br, prefix+"versiontest"+suffix)
		);

		runner.addTest(
			new SerializationTest1<EmptyVersion>
			("EmptyVersion", new EmptyVersion(), &bw, &br, prefix+"emptyversion"+suffix)
		);

		return runner.run();
	}
	

}}} // namespace 
