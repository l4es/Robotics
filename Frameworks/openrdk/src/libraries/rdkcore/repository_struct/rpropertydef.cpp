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

#include <fstream>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RPropertyDef"
using namespace std;

#include <rdkcore/object/objectmanager.h>

#include "rpropertydef.h"

namespace RDK2 { namespace RepositoryNS {

	RDK2_FACTORY(RPropertyDef);
	
	using namespace RDK2::RPrimitive;
	
	
	RPropertyDef::RPropertyDef() : Property("", ""), completeUrl("") { }
	
	void RPropertyDef::read(Reader* r) throw (RDK2::ReadingException)
	{
		r->startReading(getClassName());
			completeUrl = Url(r->readString());
			description = r->readString();
			options = (PropertyOptions) r->read_i32();
			isEnumB = (bool) r->read_i8();
			isLinkB = (bool) r->read_i8();
			linkTo = r->readString();
			defaultDoubleUnit = (RDouble::Unit) r->read_i32();
			string cn = r->readString();
			if (getObjectClassName() == "") setObjectClassName(cn);
		r->doneReading();
	}
	
	void RPropertyDef::write(Writer* w) const throw (RDK2::WritingException)
	{
		w->startWriting(getClassName());
			w->writeString(completeUrl);
			w->writeString(description);
			w->write_i32(options);
			w->write_i8(isEnumB);
			w->write_i8(isLinkB);
			w->writeString(linkTo);
			w->write_i32(defaultDoubleUnit);
			w->writeString(getObjectClassName());
		w->doneWriting();
	}

}} // namespaces

