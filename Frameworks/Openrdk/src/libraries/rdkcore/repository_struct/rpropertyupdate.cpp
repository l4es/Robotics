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
#define LOGGING_MODULE "RPropertyUpdate"
using namespace std;

#include <rdkcore/object/objectmanager.h>

#include "rpropertyupdate.h"

namespace RDK2 { namespace RepositoryNS {

		
RDK2_FACTORY(RPropertyUpdate);
	
RDK2::Object* RPropertyUpdate::clone() const
{
	RPropertyUpdate* p = new RPropertyUpdate;
	p->completeUrl = this->completeUrl;
	p->type = this->type;
	p->object = object ? object->clone() : 0;
	return p;
}

RPropertyUpdate::RPropertyUpdate() : completeUrl(""), type(VALUE), object(0) { }

RPropertyUpdate::~RPropertyUpdate()
{
	delete object;
}

void RPropertyUpdate::read(Reader* r) throw (RDK2::ReadingException)
{
	r->startReading(getClassName());
		completeUrl = r->readString();
		type = (Type) r->read_i32();
		object = (RDK2::Object*) r->readObject();
	r->doneReading();
}

void RPropertyUpdate::write(Writer* w) const throw (RDK2::WritingException)
{
	if(!object)
		throw WritingException("Cannot serialize RPropertyUpdate with null payload.");
		
	w->startWriting(getClassName());
		w->writeString(completeUrl);
		w->write_i32(type);
		w->writeObject(true, object);
	w->doneWriting();
}

}} // namespaces

