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

#ifndef RDK2_RMAPS_RLINE2DONMAP
#define RDK2_RMAPS_RLINE2DONMAP

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/rgraphics/color.h>

#include "ritemonmapvector.h"

namespace RDK2 { namespace RMaps {

	using RDK2::RGraphics::RgbColor;
	
	struct RLine2dOnMap : public RItemOnMap {
		RDK2::Geometry::Point2d p0, p1;
		RgbColor color;
		double drawWidth;

		RLine2dOnMap(
			double x0 = 0, 
			double y0 = 0, 
			double x1 = 0, 
			double y1 = 0, 
			RgbColor color = 0x000000,
			double drawWidth = 2) 
			: p0(x0, y0), p1(x1, y1), color(color), drawWidth(drawWidth) { }
	
		void read(Reader* r) throw (ReadingException);
		void write(Writer* w) const throw (WritingException);
		virtual bool hasStringRepresentation() const;
		virtual std::string getStringRepresentation() const;
		virtual bool loadFromStringRepresentation(const std::string&);

		RDK2_DEFAULT_CLONE(RLine2dOnMap);		
	};

}} // namespace

#endif
