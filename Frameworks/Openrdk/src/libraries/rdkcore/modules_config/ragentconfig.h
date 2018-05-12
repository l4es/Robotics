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

#ifndef RDK2_RAGENT_CONFIG
#define RDK2_RAGENT_CONFIG

#include "moduleconfig.h"

namespace RDK2 { namespace RAgent {

struct RAgentConfig: public RDK2::Object  {
	string agentName;
	string defaultAgentName;
	string ypFilename;
	
	ModuleConfigVector moduleConfigs;
	
	void read(Reader*r) throw (ReadingException) {
		r->startReading("RAgentConfig");
		defaultAgentName = r->readString("agentName");
		if (agentName == "")
			agentName = defaultAgentName;
		ypFilename = r->readString("ypFilename");
		r->readObject(&moduleConfigs, "modules");
		r->doneReading();
	}
	
	void write(Writer*w) const throw (WritingException) {
		w->startWriting("RAgentConfig");
		w->writeString(defaultAgentName, "agentName");
		w->writeString(ypFilename, "ypFilename");
		w->writeObject(true, &moduleConfigs, "modules");
		w->doneWriting();
	}
	
	RDK2::Object * clone() const { return new RAgentConfig(*this); }
};

}} // ns

#endif
