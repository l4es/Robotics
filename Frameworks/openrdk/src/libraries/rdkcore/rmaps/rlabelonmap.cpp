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

#include "rlabelonmap.h"

namespace RDK2 { namespace RMaps {

	RDK2_FACTORY(RLabelOnMap);

	void RLabelOnMap::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
		w->write_f64(x);
		w->write_f64(y);
		w->write_i32(color);
		w->write_f64(size);
		w->writeString(text);
		w->doneWriting();
	}

	void RLabelOnMap::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
		x = r->read_f64();
		y = r->read_f64();
		color = (RgbColor) r->read_i32();
		size = r->read_f64();
		text = r->readString();
		r->doneReading();
	}

	bool RLabelOnMap::hasStringRepresentation() const
	{
		return true;
	}

	std::string RLabelOnMap::getStringRepresentation() const
	{
		std::ostringstream oss;
		oss << x << " "
			<< y << " "
			<< RDK2::RGraphics::getColorName(color) << " "
			<< size << " "
			<< text;
		return oss.str();
	}

	bool RLabelOnMap::loadFromStringRepresentation(const std::string&)
	{
		std::istringstream iss;
		string scolor;
		iss >> x
			>> y
			>> scolor
			>> size
			>> text;
		color = RDK2::RGraphics::getColorByName(scolor);
		return true;
	}

}} // namespace
