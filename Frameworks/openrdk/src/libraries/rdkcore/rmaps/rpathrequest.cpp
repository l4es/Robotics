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

#include "rpathrequest.h"

namespace RDK2 { namespace RMaps {

	RDK2_FACTORY(RPathRequest);
	
	using namespace std;
	
	void RPathRequest::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->writeString(id);
			w->write_f32(startPose.x);
			w->write_f32(startPose.y);
			w->write_f32(startPose.theta);
			w->write_f32(targetPose.x);
			w->write_f32(targetPose.y);
			w->write_f32(targetPose.theta);
		w->doneWriting();
	}
	
	void RPathRequest::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			id = r->readString();
			startPose.x = r->read_f32();
			startPose.y = r->read_f32();
			startPose.theta = r->read_f32();
			targetPose.x = r->read_f32();
			targetPose.y = r->read_f32();
			targetPose.theta = r->read_f32();
		r->doneReading();
	}

}} // namespace
