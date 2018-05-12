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

#include <rdkcore/serialization_xml/xmlreader.h>
#include <rdkcore/serialization_xml/xmlwriter.h>
#include <rdkcore/serialization_binary/binaryreader.h>
#include <rdkcore/serialization_binary/binarywriter.h>

#include <rdkcore/object/objectmanager.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "test_all"
#include <map>
#include <string>

using namespace std;
using namespace RDK2;
using namespace RDK2::Meta;
using namespace RDK2::Serialization;
using namespace RDK2::Serialization::Xml;
using namespace RDK2::Serialization::Binary;

int main()
{
	XmlWriter xw(true); XmlReader xr;
	BinaryReader br; BinaryWriter bw(true);
	
	bool failed = false;

	Class2prot& class2prot = getClass2prot();	
	Class2prot::iterator it;
	for(it=class2prot.begin();it!=class2prot.end();++it) {
		cstr className = it->first;
		RDK_INFO_STREAM("Name " << className);
		
		Object * o = forName(className);
		
		std::vector<Object*> v = o->getTestCases();
		if(v.size()==0) {
			RDK_DEBUG_STREAM("No test cases for " << className);
			continue;
		}
		std::vector<Object*>::iterator it;
		
		if(v.size()>=2) {
			Object * o1 = v[0];
			Object * o2 = v[1];
			if(o1->equals(o2)) {
				RDK_ERROR_STREAM(className << ": equal test-cases??");
				failed = true;				
			}
		}
		
		for(it=v.begin();it!=v.end();++it) {
			Object * o = *it;
			
			if(!o->equals(o)) {
				RDK_ERROR_STREAM(className << ": relation equals() is not reflective");
				failed = true;
			}

			string xml;
			try {
				xml = xw.serialize(true,o);
				Object * o2 = (Object*) xr.deserialize(xml);
				if(!o->equals(o2)) {
					RDK_ERROR_STREAM(className << ": could not serialize/deserialize XML");
					failed = true;
				}
				
				if(o->equals(o2) && !o2->equals(o)) {
					RDK_ERROR_STREAM(className << ": equals is not symmetric");
					failed = true;
				}
			} catch(exception &e) {
				RDK_ERROR_STREAM(className << ": bad xml serialization"
					<< ": " << e.what());
				failed = true;
			} catch(...) {
				RDK_ERROR_STREAM(className << ": Unknown exception");
				failed = true;
			}

			try {
				string bin = bw.serialize(true,o);
				Object * o2 = (Object*) br.deserialize(bin);
				if(!o->equals(o2)) {
					string xml2 = xw.serialize(true, o2);
					RDK_ERROR_STREAM(className << ": could not serialize/deserialize binary. XML1: \n" << xml << "XML2:\n" << xml2);
					failed = true;
				}
				
				if(o->equals(o2) && !o2->equals(o)) {
					RDK_ERROR_STREAM(className << ": equals is not symmetric: \n" << xml);
					failed = true;
				}
				
			} catch(exception &e) {
				RDK_ERROR_STREAM(className << ": bad xml serialization"
					<< ": " << e.what());
				failed = true;
			} catch(...) {
				RDK_ERROR_STREAM(className << ": Unknown exception");
				failed = true;
			}

		}
		
	}
	
	return failed ? -1 : 0;
}








