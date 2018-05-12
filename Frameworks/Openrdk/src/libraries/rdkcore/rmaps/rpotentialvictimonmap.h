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

#ifndef RDK2_RMAPS_RPOTENTIALVICTIMONMAP
#define RDK2_RMAPS_RPOTENTIALVICTIMONMAP

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/container/container.h>
#include <rdkcore/rgraphics/color.h>

#include "ritemonmapvector.h"

namespace RDK2 { namespace RMaps {
	
	using namespace RDK2::RGraphics;
	
	struct RPotentialVictimOnMap : public RItemOnMap {
		RDK2::Geometry::Point2d position;
		int id;
		bool visited;
	
		RPotentialVictimOnMap(double x = 0, double y = 0, int id = 0) :
			position(x, y), id(id), visited(false) { }
		
		void read(Reader* r) throw (ReadingException);
		void write(Writer* w) const throw (WritingException);
		RDK2_DEFAULT_CLONE(RPotentialVictimOnMap);	
	};

}} // namespace

#endif
