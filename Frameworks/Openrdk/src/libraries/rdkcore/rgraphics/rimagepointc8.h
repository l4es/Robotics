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

#ifndef H_RDATA_RGRAPHICS_RIMAGEPOINTC8
#define H_RDATA_RGRAPHICS_RIMAGEPOINTC8

#include <rdkcore/object/object.h>
#include <rdkcore/geometry/point.h>
#include "rimage.h"

namespace RDK2 { namespace RGraphics {

class RImagePointC8 : public RDK2::Object, public RDK2::Geometry::Point2i {
public:
	RImagePointC8(int x = 0, int y = 0, RImage::C8Color color = RImage::C8White) 
	: RDK2::Geometry::Point2i(x, y), color(color) { }
	
	RImage::C8Color color;

	virtual void read(Reader* r) throw (ReadingException);
	virtual void write(Writer* w) const throw (WritingException);
	
	virtual RDK2::Object* clone() const;
};

}} // namespace

#endif
