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

#ifndef RDK2_RGRAPHICS_RMAPIMAGEDIFFRECT
#define RDK2_RGRAPHICS_RMAPIMAGEDIFFRECT

#include <rdkcore/object/object.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/object/objectdiff.h>
#include <rdkcore/rgraphics/rimagediffrect.h>

#include "rmapimage.h"

namespace RDK2 { namespace RMaps {

struct RMapImageDiffRect: public RDK2::Meta::ObjectDiff {
	RMapImageDiffRect();
	RMapImageDiffRect(RMapImage* mapImage);
	RMapImageDiffRect(RMapImage* mapImage, int left, int top, int width, int height);
	virtual ~RMapImageDiffRect() { delete imageDiffRect; }
	
	void read(RDK2::Reader*r) throw (RDK2::ReadingException);
	void write(RDK2::Writer*w) const throw (RDK2::WritingException);

	RDK2::Object* clone() const;

	virtual vector<ObjectDiff*> split(size_t maxSize);

	double refMapImageX, refMapImageY, refMapImageTheta;
	double refMapImageRealWidth;
	RImageDiffRect* imageDiffRect;
};

}}

#endif
