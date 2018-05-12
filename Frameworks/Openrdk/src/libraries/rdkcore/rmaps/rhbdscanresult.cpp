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

#include <sstream>

#include "rhbdscanresult.h"

namespace RDK2 { namespace RMaps {

using namespace std;

void RHbdScanResult::write(Writer* w) const throw (WritingException)
{
	w->startWriting(getClassName());
		w->write_f32(robotPose.x);
		w->write_f32(robotPose.y);
                w->write_f32(robotPose.theta);
		w->write_f32(distance);
		w->write_f32(fovx);
		w->write_i8(victimResult);
	w->doneWriting();
}

void RHbdScanResult::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
		robotPose.x = r->read_f32();
		robotPose.y = r->read_f32();
		robotPose.theta = r->read_f32();
		distance = r->read_f32();
		fovx = r->read_f32();
		victimResult = (VictimResult) r->read_i8();
	r->doneReading();
}

RDK2::Object* RHbdScanResult::clone() const
{
	return new RHbdScanResult(*this);
}

}} // namespace
