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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "ModuleConfig"

#include <rdkcore/filesystem/filesystem.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/serialization_xml/xmlreader.h>
#include <rdkcore/serialization_binary/binaryreader.h>

#include "moduleconfig.h"

namespace RDK2 { namespace RAgent {

using namespace RDK2::TextUtils;
using namespace RDK2::Serialization::Xml;
using namespace RDK2::Serialization::Binary;
using namespace RDK2::Filesystem;

RDK2_FACTORY(ModuleConfig);

void ModuleConfig::read(Reader*r) throw (ReadingException)
{
	unsigned char vers = r->startReading("ModuleConfig");
	moduleName = r->readString("moduleName");
	library = r->readString("library");
	if (vers == 1) r->readString("configFile");
	if (vers > 1) r->readObject(&visualConfigPosition, "visualConfigPosition");
	else visualConfigPosition = RPoint2i(0, 0);
	string textConfig = r->readString("textConfig");
	parseTextConfig(textConfig);
	if (vers == 1) r->readObject(&objConfigs, "config");
	else r->readObject(&objConfigs, "objConfigs");
	r->doneReading();
}

void ModuleConfig::write(Writer*w) const throw (WritingException)
{
	w->startWriting("ModuleConfig", 2);
	w->writeString(moduleName, "moduleName");
	w->writeString(library, "library");
	w->writeObject(true, &visualConfigPosition, "visualConfigPosition");
	string textConfig = prepareTextConfigForSave();
	w->writeString(textConfig, "textConfig");
	w->writeObject(true, &objConfigs, "objConfigs");
	w->doneWriting();
}

vector<string> stripOptionsFromTextConfigValue(string& val)
{
	string newVal = val;
	size_t a = val.find_first_of("[");
	size_t b = val.find_last_of("]");
	if (a == string::npos) return vector<string>();
	else {
		if (b == string::npos) {
			RDK_ERROR_PRINTF("Malformed text configuration value '%s'", val.c_str());
			return vector<string>();
		}
		else {
			string z = val.substr(a+1, b-a-1);
			newVal = trim(val.substr(0, a));
			vector<string> v = tokenize(z, ",");
			for (size_t i = 0; i < v.size(); i++) v[i] = trim(v[i]);
			val = newVal;
			return v;
		}
	}
}

// *** OPTIONS ***
//
// all:
// KEEP[_THIS]      keep this value, don't save a new value (i.e.: load but not save)
// VOLATILE         do not save anything of this object (it will be the default, i.e.: don't load nor save)
// PERSISTENT       default (load and save)
//
// for remote edit:
// NOT_EDITABLE     this property can be modified only by the owner module
// EDITABLE		    this property can be modified by remote and rconsole
//
// external links (value begins with '@'):
// TCP[_IP], UDP[_IP], SHM
//                  how to do the external link (if external)
// ON_CHANGE[(x)], PERIODIC[(x)]
//                  broadcast on change (maximum times per seconds) or periodically (seconds)
// DIFF[S], VALUE   send the diff or the whole value
//
// external files (value begins with '$'):
// BIN[ARY]         the file is binary saved
// XML              the file is xml saved (default)

void ModuleConfig::parseTextConfig(cstr textConfig)
{
	// load text configurations in the textObjConfigs map
	textObjConfigs.clear();
	istringstream ifs(textConfig);
	string line;
	while (getline(ifs, line)) {
		vector<string> vv = tokenize(line, "#");
		if (vv.size() > 0) {
			line = vv[0];
			vv = tokenize(line, "=");
			if (vv.size() > 0) {
				string key = trim(vv[0]);
				if (key != "") {
					string val = (vv.size() == 1 ? "" : trim(vv[1]));
					textObjConfigs.insert(make_pair(key, val));
				}
			}
		}
	}

	for (map<string, string>::iterator it = textObjConfigs.begin();
	it != textObjConfigs.end(); ++it) {
		// get options between square brackets and strip them from property value
		vector<string> opts = stripOptionsFromTextConfigValue(it->second);
		//RDK_DEBUG_PRINTF("value stripped: '%s'", it->second.c_str());
		// put options in userOptions map
		for (vector<string>::iterator optIt = opts.begin(); optIt != opts.end(); ++optIt) {
			string opt = *optIt;
			if (opt == "KEEP") opt = "KEEP_THIS";
			else if (opt == "BIN") opt = "BINARY";
			//RDK_DEBUG_PRINTF("  option: '%s'", opt.c_str());
			userOptions[it->first].insert(opt);
		}
	}
}

#define COMMENT_COLUMN 45

string ModuleConfig::prepareTextConfigForSave() const
{
	string tc;
	for (map<string, string>::const_iterator it = textObjConfigs.begin();
	it != textObjConfigs.end(); ++it) {
		string pname = it->first, opts, val = it->second;
		bool commented = false;
		if (pname[0] == '#') { commented = true; pname = pname.substr(1); }
		
		map<string, set<string> >::const_iterator uoIt = userOptions.find(pname);
		if (uoIt != userOptions.end()) {
			const set<string>& propUserOpts = uoIt->second;
			for (set<string>::iterator it = propUserOpts.begin();
			it != propUserOpts.end(); ++it) {
				opts += *it + ",";
			}
			
			if (opts.size() > 0) {
				opts = "[" + opts.substr(0, opts.size() - 1) + "]";
				commented = false;
			}
		}
		
		string line = (commented? "#" : "") + pname + "=" + val + opts;
		int spc = COMMENT_COLUMN - line.size();
		if (spc < 1) spc = 1;
		string desc;
		map<string, string>::const_iterator descIt = descriptions.find(pname);
		if (descIt != descriptions.end()) desc = descIt->second;
		tc += string(8, ' ') + line + string(spc, ' ') + "# " + desc + "\n";
	}
	return "\n" + tc + string(6, ' ');
}

}} // namespaces

