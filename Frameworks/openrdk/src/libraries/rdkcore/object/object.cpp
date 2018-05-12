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
#include "object.h"
#define LOGGING_MODULE "Object"

namespace RDK2 {

void Object::read(Reader*) throw (ReadingException)
{
	RDK_ERROR_STREAM("Could not deserialize object of class: "<< getClassName());
	throw ReadingException(string()+"Could not deserialize object of class: " + getClassName());
}

void Object::write(Writer*) const throw (WritingException)
{
	RDK_ERROR_STREAM("Could not serialize object of class: "<< getClassName());
	throw WritingException(string()+"Could not serialize object of class: " + getClassName());
}

void Object::readObjectBody(Reader*r) throw (ReadingException)
{
	unsigned long ul = r->read_i32();
	creationTimestamp.setMsFromMidnight(ul);
	ul = r->read_i32();
	upTimestamp.setMsFromMidnight(ul);
}

void Object::writeObjectBody(Writer*w) const throw (WritingException)
{
	w->write_i32(creationTimestamp.getMsFromMidnight());
	w->write_i32(upTimestamp.getMsFromMidnight());
}

Object* Object::clone() const
{
	RDK_ERROR_STREAM("Cannot clone a Object of class "<< getClassName());
	return 0;
}

std::string Object::getStringRepresentation() const
{
	return std::string("[")+getClassName()+"]";
}

std::string Object::getStringForVisualization() const
{
	return hasStringRepresentation() ? getStringRepresentation() : "(cannot visualize)";
}

} // namespace RDK2
