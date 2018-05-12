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

#include "walk_line.h"
#include <cstdlib>

namespace RDK2 { namespace Geometry {

LineWalk::LineWalk(Point2i p0, Point2i p1)
{
//	if (p0.x < p1.x || (p0.x == p1.x && p0.y < p1.y)) {
		this->p0 = p0;
		this->p1 = p1;
/*	}
	else {
		this->p0 = p1;
		this->p1 = p0;
	}*/
	vx = (this->p0.x < this->p1.x ? 1 : -1);
	vy = (this->p0.y < this->p1.y ? 1 : -1);
	unsigned int stepsx = std::abs(this->p0.x - this->p1.x);
	unsigned int stepsy = std::abs(this->p0.y - this->p1.y);
	steps = (stepsx > stepsy ? stepsx : stepsy);
	// AC: meglio
	// steps = max(stepsx, stepsy);
	if (steps > 0) {
		dx = (double) stepsx / steps;
		dy = (double) stepsy / steps;
	}
	else dx = dy = 0.;
	currentStep = 0;
}
	
bool LineWalk::next() {
	if (currentStep < steps) {
		currentStep++;
		return true;
	}
	else return false;
}
	
void LineWalk::rewind() 
{
	currentStep = 0; 
}

Point2i LineWalk::getPoint() const
{
	return Point2i((int) ((double)p0.x + dx * vx * currentStep), 
		       (int) ((double)p0.y + vy * dy * currentStep));
}
	
}} // ns
