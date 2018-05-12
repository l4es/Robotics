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

#ifndef RDK2_RPRIMITIVE_RDOUBLE
#define RDK2_RPRIMITIVE_RDOUBLE

#include "rprimitive.h"

namespace RDK2 { namespace RPrimitive {
	
	struct RDouble: public RPrimitive<double> {
		RDouble() : unit(REAL) { }
		RDouble(double a) : RPrimitive<double>(a), unit(REAL) { }

		enum Unit {
			REAL = 0,        ///< Unitless
			PERC = 1,        ///< Percentual
			M = 3,           ///< Meters
			M_SEC = 4,       ///< Meters/second
			M_SEC2 = 5,      ///< Meters/second^2
			SEC = 7,         ///< Seconds
			RAD = 8,         ///< Radians (NOTE: radiants are always saved, loaded and displayed as degrees)
			RAD_SEC = 9,     ///< Radians/second (same as above)
			RAD_SEC2 = 10,   ///< Radians/second^2 (same as above)
			CELSIUS = 12,    ///< Celsius degrees (temperature)
			PIXELS = 13,     ///< Pixels
			V = 14,          ///< Volts
			OTHER = 99,      ///< Other (use this ONLY if no other unit fits)

			MS = 999         ///< DEPRECATED
		} unit;

		RDouble(double a, Unit u) : RPrimitive<double>(a), unit(u) { }

		const char * myClassName() const;
		virtual Object* clone() const;
		
		std::string getStringForVisualization() const;
		std::string getStringRepresentation() const;
		bool loadFromStringRepresentation(const std::string&s);

		static std::string getUnitString(Unit unit);
		
		void read(RDK2::Reader*r) throw (RDK2::ReadingException);
		void write(RDK2::Writer*w) const  throw (RDK2::WritingException);
		
	};

}}

#endif
