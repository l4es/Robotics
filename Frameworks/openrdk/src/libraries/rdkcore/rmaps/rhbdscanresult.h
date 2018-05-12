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

#ifndef RDK2_RMAPS_RHBDSCANRESULT
#define RDK2_RMAPS_RHBDSCANRESULT

#include <float.h>

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/container/container.h>
#include <rdkcore/rgraphics/color.h>

namespace RDK2 { namespace RMaps {
	
	struct RHbdScanResult : public RDK2::Object {
		enum VictimResult { NO_VICTIM, POTENTIAL_VICTIM, CONFIRMED_VICTIM };
	
		RHbdScanResult() : 
			robotPose(0, 0, DBL_MAX), distance(0), 
			fovx(0), victimResult(NO_VICTIM) { }
		
		RHbdScanResult(const RDK2::Geometry::Point2od& robotPose, 
			double distance, double fovx, VictimResult victimResult) 
		:	robotPose(robotPose), distance(distance), 
			fovx(fovx), victimResult(victimResult) { }
		
		void read(Reader* r) throw (ReadingException);
		void write(Writer* w) const throw (WritingException);
		RDK2::Object* clone() const;
	
		RDK2::Geometry::Point2od robotPose;
		double distance;
		double fovx;
		/// 0 -> no victim; 1 -> potential victim; 2 -> victim confirmed
		VictimResult victimResult;
	};
}} // namespace

#endif
