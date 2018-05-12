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

#ifndef RDK2_RPOSEVECTOR
#define RDK2_RPOSEVECTOR

#include <rdkcore/object/object.h>
//#include <rdkcore/object/robjectupdate.h>
#include <rdkcore/geometry/point.h>

namespace RDK2 { namespace RGraphics {//namespace RForeignProperties {

using namespace RDK2::Geometry;
using namespace RDK2::Serialization;

//using RDK2::RepositoryNS::Url;
//using RDK2::Meta::ObjectUpdate;

struct RPoseVector: public RDK2::Object {
	vector < Point2od > data;

	RPoseVector(vector < Point2od > data) : data(data) { }
	RPoseVector() {}
	
	void read(Reader* r) throw (ReadingException);
	void write(Writer* w) const throw (WritingException);
	RDK2_DEFAULT_CLONE(RPoseVector);

};

}} // namespaces

#endif
