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

#ifndef H_RDATA_RGRAPHICS_RQUATERNION
#define H_RDATA_RGRAPHICS_RQUATERNION

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/quaternion.h>

namespace RDK2 { namespace RGeometry {

	struct RQuaterniond : public RDK2::Object, public RDK2::Geometry::Quaterniond {

		RQuaterniond(double x=0., double y=0., double z=0., double w=1.): RDK2::Geometry::Quaterniond(x,y,z,w){ }
	
		RDK2::Object* clone() const;
		
		void read(RDK2::Reader*r) throw (RDK2::ReadingException);
		void write(RDK2::Writer*w) const  throw (RDK2::WritingException);
	};

}} // namespace RDK2::RGeometry

#endif
