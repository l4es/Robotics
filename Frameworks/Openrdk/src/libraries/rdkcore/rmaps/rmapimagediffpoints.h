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

#ifndef RDK2_RMAPS_RMAPIMAGEDIFFPOINTS
#define RDK2_RMAPS_RMAPIMAGEDIFFPOINTS

#include <vector>
#include <rdkcore/object/object.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/object/objectdiff.h>
#include <rdkcore/rgraphics/rimagediffpoints.h>
#include "rmapimage.h"

namespace RDK2 { namespace RMaps {

using namespace RDK2::Geometry;

struct RMapImageDiffPoints: public RDK2::Meta::ObjectDiff {
	RMapImageDiffPoints();
	RMapImageDiffPoints(RMapImage* mapImage, const vector<Point2i>& points);
	virtual ~RMapImageDiffPoints() { delete imageDiffPoints; }
	
	virtual void read(RDK2::Reader*r) throw (RDK2::ReadingException);
	virtual void write(RDK2::Writer*w) const throw (RDK2::WritingException);

	virtual std::vector<ObjectDiff*> split(size_t maxSize);

	virtual RDK2::Object* clone() const;

	double refMapImageX, refMapImageY, refMapImageTheta;
	double refMapImageRealWidth;
	RImageDiffPoints* imageDiffPoints;
};

}}

#endif
