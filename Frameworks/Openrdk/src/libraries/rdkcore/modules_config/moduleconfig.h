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

#ifndef RDK2_RAGENT_MODULE_CONFIG
#define RDK2_RAGENT_MODULE_CONFIG

#include <map>

#include <rdkcore/rgeometry/rpoint2i.h>
#include <rdkcore/repository_struct/property.h>

#include "pair.h"

namespace RDK2 { namespace RAgent {

using namespace RDK2::Containers;
using namespace RDK2::RGeometry;
using namespace std;
using namespace RDK2::RepositoryNS;

class ModuleConfig: public RDK2::Object  {
public:
	string moduleName;
	string library;
	RPoint2i visualConfigPosition;

protected:
	void parseTextConfig(cstr textConfig);	// called from read to fill the following structures
	string prepareTextConfigForSave() const;	// called from write to create the text config to save

public:// FIXME should be protected
	// read() fills these, and you need to fill these before calling write()
	map<string, string> textObjConfigs;		// objects loaded from and to save to textual representation
	PropertyList objConfigs;				// objects loaded from and to save to xml
	map<string, set<string> > userOptions;	// user options loaded from and to save to textual config
	map<string, string> descriptions;		// descriptions (to be set only before save)
	
public:
	ModuleConfig() { }
	void setLibrary(string s){library=s;}
	void setModuleName(string s){moduleName=s;}
	void read(Reader*r) throw (ReadingException);
	void write(Writer*w) const throw (WritingException);
	RDK2_DEFAULT_CLONE(ModuleConfig);
};

typedef RDK2::Containers::Vector<ModuleConfig> ModuleConfigVector;

}} // namespace RDK2::RAgent

#endif
