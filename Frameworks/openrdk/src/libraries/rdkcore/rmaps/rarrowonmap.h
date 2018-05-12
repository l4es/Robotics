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

#ifndef RDK2_RMAPS_RARROWTODRAW
#define RDK2_RMAPS_RARROWTODRAW

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/container/container.h>
#include <rdkcore/rgraphics/color.h>

#include "ritemonmapvector.h"

namespace RDK2 { namespace RMaps {

using RDK2::RGraphics::RgbColor;

class RArrowOnMap : public RItemOnMap, public RDK2::Geometry::Point2od
{
	public:
	enum ArrowType { OPEN=0, SOLID, DIAMOND, CIRCLE };

	double delta;
	double headAngle;
	double headLength;
	bool filled;
	ArrowType type;
	RgbColor color;
	double drawWidth;

	RArrowOnMap(double x = 0, double y = 0, double theta = 0,
			double d = 1, double ha      = 1, double hl = 0.3,
			 bool f = false, ArrowType t = OPEN,
			RgbColor color = 0x000000, double drawWidth = 1.0)
	:
		RDK2::Geometry::Point2od(x, y, theta), delta(d), headAngle(ha), headLength(hl), filled(f), type(t), color(color), drawWidth(drawWidth) { }
	
	void read(Reader* r) throw (ReadingException);
	void write(Writer* w) const throw (WritingException);

	static void calcVertexes(const RArrowOnMap& arrow, double& x1, double& y1, double& x2, double& y2);
	
	virtual bool hasStringRepresentation() const;
	virtual std::string getStringRepresentation() const;
	virtual bool loadFromStringRepresentation(const std::string&);

	RDK2_DEFAULT_CLONE(RArrowOnMap);
};

}} // namespace

#endif
