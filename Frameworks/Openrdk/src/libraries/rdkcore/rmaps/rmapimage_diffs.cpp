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
#define LOGGING_MODULE "RMapImage"

#include "rmapimagediffrect.h"
#include "rmapimagediffpoints.h"

namespace RDK2 { namespace RMaps {

vector<ObjectDiff*> RMapImage::splitInDiffs(size_t maxSize)
{
	RMapImageDiffRect* bigDiff = new RMapImageDiffRect(this, 0, 0, image->getWidth(), image->getHeight());
	vector<ObjectDiff*> ret = bigDiff->split(maxSize);
	delete bigDiff;
	return ret;
}

bool RMapImage::applyDiff(const ObjectDiff* diff)
{
	const RMapImageDiffRect* rectDiff = dynamic_cast<const RMapImageDiffRect*>(diff);
	if (rectDiff) {
		x = rectDiff->refMapImageX;
		y = rectDiff->refMapImageY;
		theta = rectDiff->refMapImageTheta;
		realWidth = rectDiff->refMapImageRealWidth;
		image->applyDiff(rectDiff->imageDiffRect);
		return true;
	}
	const RMapImageDiffPoints* pointsDiff = dynamic_cast<const RMapImageDiffPoints*>(diff);
	if (pointsDiff) {
		x = rectDiff->refMapImageX;
		y = rectDiff->refMapImageY;
		theta = rectDiff->refMapImageTheta;
		realWidth = rectDiff->refMapImageRealWidth;
		image->applyDiff(pointsDiff->imageDiffPoints);
		return true;
	}
	return false;
}

}} // namespaces
