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

#include "rarrowonmap.h"

namespace RDK2 { namespace RMaps {

	RDK2_FACTORY(RArrowOnMap);

	void RArrowOnMap::write(Writer* w) const throw (WritingException)
	{
		w->startWriting(getClassName());
		w->write_f64(x);
		w->write_f64(y);
		w->write_f64(theta);
		w->write_f64(delta);
		w->write_f64(headAngle);
		w->write_f64(headLength);
		w->write_u8(static_cast<uint8_t>(filled));
		w->write_u8(static_cast<uint8_t>(type));
		w->write_i32(color);
		w->write_f64(drawWidth);
		w->doneWriting();
	}

	void RArrowOnMap::read(Reader* r) throw (ReadingException)
	{
		r->startReading(getClassName());
		x = r->read_f64();
		y = r->read_f64();
		theta = r->read_f64();
		delta = r->read_f64();
		headAngle = r->read_f64();
		headLength = r->read_f64();
		filled    =  static_cast<bool>(r->read_u8());
		type      =  static_cast<ArrowType>(r->read_u8());
		color     = (RgbColor) r->read_i32();
		drawWidth = r->read_f64();
		r->doneReading();
	}

	void RArrowOnMap::calcVertexes(const RArrowOnMap& arrow, double& x1, double& y1, double& x2, double& y2)
	{
		double end_x = arrow.x + arrow.delta*cos(arrow.theta),
					 end_y = arrow.y + arrow.delta*sin(arrow.theta);

		x1 = end_x - arrow.headLength * cos(arrow.theta - arrow.headAngle/2);
		y1 = end_y - arrow.headLength * sin(arrow.theta - arrow.headAngle/2);
		x2 = end_x - arrow.headLength * cos(arrow.theta + arrow.headAngle/2);
		y2 = end_y - arrow.headLength * sin(arrow.theta + arrow.headAngle/2);
	}

	bool RArrowOnMap::hasStringRepresentation() const
	{
		return true;
	}

	std::string RArrowOnMap::getStringRepresentation() const
	{
		std::ostringstream oss;
		oss << x                                    << " "
		    << y                                    << " "
		    << theta                                << " "
		    << delta                                << " "
		    << headAngle                            << " "
		    << headLength                           << " "
		    << filled                               << " "
		    << type                                 << " "
		    << RDK2::RGraphics::getColorName(color) << " "
		    << drawWidth;
		return oss.str();
	}

	bool RArrowOnMap::loadFromStringRepresentation(const std::string&)
	{
		std::istringstream iss;
		string scolor;
		int itype;
		iss >> x
				>> y
				>> theta
				>> delta
				>> headAngle
				>> headLength
				>> filled
				>> itype
				>> scolor
				>> drawWidth;
		type = ArrowType(itype);
		color = RDK2::RGraphics::getColorByName(scolor);
		return true;
	}


}} // namespace
