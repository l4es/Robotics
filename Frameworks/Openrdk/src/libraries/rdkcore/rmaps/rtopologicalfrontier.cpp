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

/**
 * @file
 *
 * @brief This file contains methods of class RTopologicalFrontier
 */

#include "rtopologicalfrontier.h"

namespace RDK2 { namespace RMaps {

RDK2_FACTORY(RTopologicalFrontier);
RDK2_FACTORY(RTopologicalFrontierVector);
	
using namespace std;

RTopologicalFrontier::RTopologicalFrontier(double x0, double y0, double x1, 
					   double y1): point0(x0, y0), 
						       point1(x1, y1)
{
	double deltaX, deltaY, l;
	midpoint.x = (x0 + x1) / 2;
	midpoint.y = (y0 + y1) / 2;
	deltaX = x1 - x0;
	deltaY = y1 - y0;
	l = sqrt(deltaX * deltaX + deltaY * deltaY);
	midpoint.theta = asin(deltaY / l) + M_PI / 2;
}

void RTopologicalFrontier::write(Writer* w) const throw (WritingException)
{
	w->startWriting(getClassName());
		w->write_f32(point0.x);	w->write_f32(point0.y);
		w->write_f32(point1.x);	w->write_f32(point1.y);
		w->write_f32(midpoint.x);	w->write_f32(midpoint.y);	w->write_f32(midpoint.theta);
	w->doneWriting();
}
	
void RTopologicalFrontier::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
		point0.x = r->read_f32();	point0.y = r->read_f32();
		point1.x = r->read_f32();	point1.y = r->read_f32();
		midpoint.x = r->read_f32();	midpoint.y = r->read_f32();	midpoint.theta = r->read_f32();
	r->doneReading();
}

std::string RTopologicalFrontier::getStringForVisualization() const {
	ostringstream oss;
	oss << "(" << point0.toString() << ") - (" << midpoint.toString() << 
		") - (" << point1.toString() << ")";
	return oss.str();
}
}} // namespace

