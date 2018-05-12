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

#ifndef RDK2_RSENSORS_RUSARVICTIMRFIDDATA
#define RDK2_RSENSORS_RUSARVICTIMRFIDDATA

#include <rdkcore/container/container.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/time/time.h>
#include <rdkcore/object/object.h>
#include <string>
#include <rdkcore/container/container.h>

namespace RDK2 { namespace RSensorData {

	struct RUsarVictimRfidData : public RDK2::Object {
		std::string ipc_hostname;
		RDK2::Time::Timestamp timestamp;

		std::string id;
		std::string name;
		std::string status;
		
		double reliability;
		
		RDK2::Geometry::Point3d location;
	
		void read(Reader* r) throw (ReadingException);
		void write(Writer* w) const throw (WritingException);
		RDK2_DEFAULT_CLONE(RUsarVictimRfidData);
	};

typedef RDK2::Containers::Vector<RUsarVictimRfidData> RUsarVictimRfidDataVector;

}} // namespaces

#endif
