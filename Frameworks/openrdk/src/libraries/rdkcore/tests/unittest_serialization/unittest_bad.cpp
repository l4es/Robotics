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

#include <cppunit/ui/text/TestRunner.h> 
#include "test1.h"
#include "bad_tests.h"
#include "unittest_bad.h"

using namespace RDK2;
using namespace RDK2::UnitTest::Serialization;
using namespace RDK2::Serialization;
using namespace RDK2::Serialization::Binary;

int main()
{
	BinaryWriter bw(true);
	BinaryReader br;
	
	CppUnit::TextUi::TestRunner runner;

	/// Questi sono per il writer
/*	runner.addTest(
		new SerializationTestBad<ErrorNoStartWriting>
		("ErrorNoStartWriting", new ErrorNoStartWriting(), &bw, &br)
	);
	
	runner.addTest(
		new SerializationTestBad<ErrorNoDoneWriting>
		("ErrorNoDoneWriting", new ErrorNoDoneWriting(), &bw, &br)
	);*/
	
	runner.addTest(
		new SerializationTestBad<ErrorNoStartReading>
		("ErrorNoStartReading", new ErrorNoStartReading(), &bw, &br)
	);
	
	runner.addTest(
		new SerializationTestBad<ErrorNoDoneReading>
		("ErrorNoDoneReading", new ErrorNoDoneReading(), &bw, &br)
	);
	
	runner.addTest(
		new SerializationTestBad<ErrorWrongClass>
		("ErrorWrongClass", new ErrorWrongClass(), &bw, &br)
	);
	
	runner.addTest(
		new SerializationTestBad<ErrorWrongType1>
		("ErrorWrongType1", new ErrorWrongType1(), &bw, &br)
	);
	
	runner.addTest(
		new SerializationTestBad<ErrorWrongName1>
		("ErrorWrongName1", new ErrorWrongName1(), &bw, &br)
	);
	
	//FIXME: colpa di Giorgio
	if(false)
	runner.addTest(
		new SerializationTestBad<ErrorIncompleteRead>
		("ErrorIncompleteRead", new ErrorIncompleteRead(), &bw, &br)
	);
	
	runner.addTest(
		new SerializationTestBad<ErrorTooMuchRead>
		("ErrorTooMuchRead", new ErrorTooMuchRead(), &bw, &br)
	);
	
	runner.addTest(
		new SerializationTestBad<ErrorWrongMemberClass>
		("ErrorWrongMemberClass", new ErrorWrongMemberClass(), &bw, &br)
	);
	
	return !runner.run();
}
