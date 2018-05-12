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

#ifndef RDK2_RSENSORS_RUSARINUDATA
#define RDK2_RSENSORS_RUSARINUDATA

#include <rdkcore/geometry/point.h>
#include <rdkcore/object/object.h>
#include <rdkcore/time/time.h>
#include <string>

namespace RDK2 { namespace RSensorData {
/**
 * This class contains the data read from an USARSim inertial sensor.
 */	
	struct RUsarInuData : public RDK2::Object {	
		RDK2::Time::Timestamp timestamp;
		std::string name;
		/// This was a Point2od, now it's a Point3od
		RDK2::Geometry::Point3od location;
		
		RDK2_DEFAULT_CLONE(RUsarInuData);
		///@name Serialization
		//@{
		void read(Reader* r) throw (ReadingException);
		void write(Writer* w) const throw (WritingException);
                //@}
	};

}} // namespaces

#endif
