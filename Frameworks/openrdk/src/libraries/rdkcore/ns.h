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

#ifndef H_RDK2_NS
#define H_RDK2_NS

namespace RDK2 { 
	
	namespace Serialization {
		namespace Xml {}
		namespace Binary {}
	} 
	using namespace RDK2::Serialization::Xml;
	using namespace RDK2::Serialization::Binary;
	using namespace RDK2::Serialization;
	
	#define INCLUDE_NS(ns) \
		namespace ns {} \
		using namespace RDK2::ns;

	INCLUDE_NS(Profiling)
	INCLUDE_NS(Meta)
	INCLUDE_NS(RepositoryNS)
	INCLUDE_NS(RGeometry)
	INCLUDE_NS(RPrimitive)
	INCLUDE_NS(RGraphics)
	INCLUDE_NS(RMaps)
	INCLUDE_NS(RNetObjects)
	INCLUDE_NS(RSensorData)
	INCLUDE_NS(RSvs)
	INCLUDE_NS(Ruby)
	INCLUDE_NS(SensorData)
	INCLUDE_NS(TextUtils)
	INCLUDE_NS(Time)
	INCLUDE_NS(Geometry)
	INCLUDE_NS(Filesystem)
}

#endif
