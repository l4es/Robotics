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

#include "rlandmark.h"
#include <iomanip>
#include <sstream>

RDK2_FACTORY(RLandmark2D);

void RLandmark2D::write(RDK2::Serialization::Writer* w) const throw (RDK2::WritingException)
{
	w->startWriting(getClassName());
	RDK2::Object::writeObjectBody ( w );
	w->writeString(tag);
	w->write_f64(x);
	w->write_f64(y);
	w->doneWriting();
}

void RLandmark2D::read(RDK2::Serialization::Reader* r) throw (RDK2::ReadingException)
{
	r->startReading(getClassName());
	RDK2::Object::readObjectBody ( r );
	tag = r->readString();
	x = r->read_f64();
	y = r->read_f64();
	r->doneReading();
}


std::string RLandmark2D::getStringRepresentation() const
{
	std::ostringstream oss;
	oss << tag << " " << x << " " << y;
	return oss.str();
}

bool RLandmark2D::loadFromStringRepresentation(const std::string&)
{
	std::istringstream iss;
	iss >> tag >> x >> y;
	return iss;
}

std::string RLandmark2D::getStringForVisualization() const
{
	std::ostringstream oss;
	oss << std::setiosflags(std::ios::fixed) << std::setprecision(2);

	oss << tag << " " << x << " " << y;
	return oss.str();
}

