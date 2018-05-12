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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RImageDiffPoints"

#include "rimagediffpoints.h"

namespace RDK2 { namespace RGraphics {

RDK2_FACTORY(RImageDiffPoints);

RImageDiffPoints::RImageDiffPoints(RImage* image, vector<Point2i> points) :
	ObjectDiff("RImage"),
	refImageWidth(image->getWidth()), refImageHeight(image->getHeight()), refImageType(image->getType())
{
	this->points = points;
	if (image->getType() == RImage::C8) {
		for (size_t i = 0; i < points.size(); i++) {
			//colorsC8.push_back(image->getPixels()[points[i].y][points[i].x]);
			colorsC8.push_back(image->getBuffer()[points[i].y * image->getWidth() + points[i].x]);
		}
	}
	else {
		RDK_ERROR_PRINTF("Other kind of points not yet implemented");
	}
}

RDK2::Object* RImageDiffPoints::clone() const
{
	return new RImageDiffPoints(*this);
}

void RImageDiffPoints::read(Reader* r) throw (ReadingException)
{
	// FIXME
	r->startReading(getClassName());
	r->doneReading();
}

void RImageDiffPoints::write(Writer* w) const throw (WritingException)
{
	// FIXME
	w->startWriting(getClassName());
	w->doneWriting();
}

vector<ObjectDiff*> RImageDiffPoints::split(size_t /*maxSize*/)
{
	// FIXME
	vector<ObjectDiff*> ret;
	return ret;
}

}} // namespaces
