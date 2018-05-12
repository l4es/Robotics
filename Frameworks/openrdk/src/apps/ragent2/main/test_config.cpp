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

#include <rdkcore/serialization_xml/xmlwriter.h>
#include <rdkcore/serialization_xml/xmlreader.h>
#include <rdkcore/rprimitive/rstring.h>
#include <rdkcore/rprimitive/rint.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "test_config"

using namespace RDK2::RPrimitive;
using namespace RDK2::Serialization::Xml;

#include <rdkcore/modules_config/ragentconfig.h>

using namespace RDK2::RAgent;
using namespace std;

int main() {
	try{

	RAgentConfig config;
	
	config.agentName = "pippo";
	config.ypFilename = "ypconfig.conf";
	
	ModuleConfig* m1 = new ModuleConfig;
		m1->library = "libBo";
		m1->moduleName = "bo1";
		
		m1->objConfigs.push_back(new Pair("prop1", new RInt(42)));
		m1->objConfigs.push_back(new Pair("prop2", new RString("stringa")));
		
	config.moduleConfigs.push_back(m1);
		
	
	ModuleConfig * m2 = new ModuleConfig;
		m2->library = "libBo";
		m2->moduleName = "bo2";
		//m2->configFile = "bo.config";
	config.moduleConfigs.push_back(m2);

	
	XmlWriter xmlWriter(true); 
	string xml = xmlWriter.serialize(true,&config);
	
	cout << xml << endl;
	
		XmlReader reader;
		RDK2::Object * o = (RDK2::Object*) reader.deserialize(xml);
	XmlWriter xmlWriter2(true); 
	string xml2 = xmlWriter2.serialize(true,o);

	cout << xml2  << endl;
	}
	catch(RDK2::ReadingException&e) {
		RDK_ERROR_STREAM(e.what());
	}
	catch(...) {
		RDK_ERROR_STREAM("Unexpected");
	}
}
