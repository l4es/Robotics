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

#include <map>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "ObjectManager"

#include "objectmanager.h"


namespace RDK2 { namespace Meta {

	Class2prot& getClass2prot() {
		static Class2prot c;
		return c;
	}
}}

namespace RDK2 { namespace Meta {
	using namespace std;
	

	Factory::Factory(const RDK2::Object*prototype) {
		Class2prot& class2prot = getClass2prot();
		string name = prototype->getClassName();
		
		Class2prot::iterator i = class2prot.find(name);
		if(i!=class2prot.end()) {
			RDK_ERROR_STREAM("Class "<<name<<" already present in database");
			return;
		}
		
		RDK2::Object * clone =  prototype->clone();
		if(!clone) {
			RDK_ERROR_STREAM("Class " << name << " does not support clone().\n"<<
				"Not added to database");
			return;
		} 
		
		string name2 = clone->getClassName();
		if(name2 != name) {
			RDK_ERROR_STREAM("getClassName() on class " << name << " returns '"
				<< name2 <<"'. \nNot added to database.");
			return;
		}
		delete clone;
		
		class2prot[name] = prototype;
//		 RDK_INFO_STREAM("Class " << name << " initialized.");
	}
	
	RDK2::Object * forName(cstr name) {
		Class2prot& class2prot = getClass2prot();
		Class2prot::iterator i = class2prot.find(name);
		if(i==class2prot.end()) {
			RDK_ERROR_STREAM("Class "<<name<<" is not present in database");
			return 0;
		}
		RDK2::Object * clone = i->second->clone();
		
		if(!clone) {
			RDK_ERROR_STREAM("Could not clone class "<<name);
			return 0;
		}
		
		return clone;
	}
	
	void freePrototypeDb()
	{
		Class2prot& c2p = getClass2prot();
		for (Class2prot::iterator it = c2p.begin(); it != c2p.end(); ++it) {
			delete it->second;
		}
		c2p.clear();
	}

}} // namespace RDK2::Object

