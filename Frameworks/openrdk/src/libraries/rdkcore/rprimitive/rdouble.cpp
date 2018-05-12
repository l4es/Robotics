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

#include <string>
#include <sstream>
#include <iomanip>

#include <rdkcore/geometry/utils.h>

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RDouble"

#include "rdouble.h"

namespace RDK2 { namespace RPrimitive {
	using namespace std;
	using namespace RDK2::Geometry;
	using RDK2::Object;

	RDK2_FACTORY(RDouble);
	
	static const char* d = "RDouble";
	const char * RDouble::myClassName() const {
		return d;
	}
	
	Object* RDouble::clone() const {
		RDouble * d = new RDouble(value);
		d->unit = unit;
		return d;
	}
	
	
	string RDouble::getUnitString(Unit unit)
	{
		switch (unit) {
		case REAL: return "";
		case PERC: return "%";
		case M: return "m";
		case M_SEC: return "m/s";
		case M_SEC2: return "m/s^2";
		case SEC: return "sec";
		case MS: return "ms";
		case RAD: return "deg";			// these are always saved, showed and loaded as degrees
		case RAD_SEC: return "deg/s";
		case RAD_SEC2: return "deg/s^2";
		case CELSIUS: return "C";
		case PIXELS: return "pixels";
		case V: return "V";
		case OTHER: return "?";
		default: {
			ostringstream oss;
			oss << "(" << unit << ")";
			return oss.str();
		}
		}
	}
	
	string RDouble::getStringRepresentation() const
	{
		std::ostringstream oss;
		oss << setiosflags(ios::fixed) << setprecision(unit == SEC ? 3 : (unit == REAL ? 5 : (unit == M ? 4 : 2)));
		switch (unit) {
			case RAD_SEC: case RAD_SEC2: case RAD: oss << rad2deg(value); break;
			case PERC: oss << value * 100.; break;
			default: oss << value; break;
		}
		return oss.str();
	}

	bool RDouble::loadFromStringRepresentation(const string& s)
	{
		std::istringstream iss(s);
		iss >> value;
		if (unit == RAD_SEC || unit == RAD_SEC2 || unit == RAD) value = deg2rad(value);
		else if (unit == PERC) value = value / 100.;
		return !iss.fail();
	}

	string RDouble::getStringForVisualization() const
	{
		std::ostringstream oss;
		oss << setiosflags(ios::fixed) << setprecision(unit != SEC && unit != RAD_SEC2 && unit != M_SEC2 ? 2 : 3);
		switch (unit) {
			case PERC: oss << value * 100. << " " << getUnitString(PERC); break;
			case RAD_SEC: oss << rad2deg(value) << " " << getUnitString(RAD_SEC); break;
			case RAD: oss << rad2deg(value) << " " << getUnitString(RAD); break;
			case RAD_SEC2: oss << rad2deg(value) << " " << getUnitString(RAD_SEC2); break;
			default: 
				oss << value; 
				string unitStr = getUnitString(unit);
				if (unitStr != "") oss << " " << unitStr;
				break;
		}
		return oss.str();
	}

#if 0	
	bool RDouble::updateObject(ObjectUpdate* objupd)
	{
		istringstream iss(objupd->updateDescription);
		int u;
		iss >> value >> u;
		unit = (RDouble::Unit) u;
		return iss;
	}
	
	ObjectUpdate* RDouble::getObjectUpdate() 
	{
		ostringstream oss;
		oss << value << " " << (int) unit;
		ObjectUpdate* objupd = new ObjectUpdate;
		objupd->updateDescription = oss.str();
		return objupd;
	}
#endif	
	void RDouble::read(RDK2::Serialization::Reader*r) throw (ReadingException) {
		r->startReading(getClassName());
			value = r->read_f64();
			unit = (Unit) r->read_i8();
		r->doneReading();
	}

	void RDouble::write(RDK2::Serialization::Writer*w) const throw (WritingException)  {
		w->startWriting(getClassName());
			w->write_f64(value);
			w->write_i8((int8_t) unit);
		w->doneWriting();
	}

	
}} 
