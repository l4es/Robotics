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

#include "rpoint2donmap.h"

namespace RDK2 { namespace RMaps {

	RDK2_FACTORY(RPoint2dOnMap);
	
	void RPoint2dOnMap::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
			w->write_f64(x);
			w->write_f64(y);
			w->write_i32(color);
			w->write_f64(drawWidth);
		w->doneWriting();
	}
	
	void RPoint2dOnMap::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
			x = r->read_f64();
			y = r->read_f64();
			color = (RgbColor) r->read_i32();
			drawWidth = r->read_f64();
		r->doneReading();
	}

	bool RPoint2dOnMap::hasStringRepresentation() const
	{
		return true;
	}

	std::string RPoint2dOnMap::getStringRepresentation() const
	{
		std::ostringstream oss;
		oss << x << " "
				<< y << " "
				<< RDK2::RGraphics::getColorName(color) << " "
				<< drawWidth;
		return oss.str();
	}

	bool RPoint2dOnMap::loadFromStringRepresentation(const std::string&)
	{
		std::istringstream iss;
		string scolor;
		iss >> x
		    >> y
		    >> scolor
		    >> drawWidth;
		color = RDK2::RGraphics::getColorByName(scolor);
		return true;
	}

}} // namespace
