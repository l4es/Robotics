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

#include "rmapimagediffrect.h"

namespace RDK2 { namespace RMaps {

	RDK2_FACTORY(RMapImageDiffRect);


RMapImageDiffRect::RMapImageDiffRect(RMapImage* mapImage, int left, int top, int width, int height) :
	ObjectDiff("RMapImage"),
	refMapImageX(mapImage->x), refMapImageY(mapImage->y), refMapImageTheta(mapImage->theta),
	refMapImageRealWidth(mapImage->realWidth)
{
	imageDiffRect = new RImageDiffRect(mapImage->image, left, top, width, height);
}

RMapImageDiffRect::RMapImageDiffRect() :
	ObjectDiff("RMapImage"),
	refMapImageX(0.), refMapImageY(0.), refMapImageTheta(0.),
	refMapImageRealWidth(0.), imageDiffRect(0)
{ }

RDK2::Object* RMapImageDiffRect::clone() const
{
	RMapImageDiffRect* r = new RMapImageDiffRect(*this);
	r->imageDiffRect = imageDiffRect ? (RImageDiffRect*) imageDiffRect->clone() : 0;
	return r;
}

void RMapImageDiffRect::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
		objectClassName = r->readString();
		refMapImageX = r->read_f64();
		refMapImageY = r->read_f64();
		refMapImageTheta = r->read_f64();
		refMapImageRealWidth = r->read_f64();
		imageDiffRect = (RImageDiffRect*) r->readObject();
	r->doneReading();
}

void RMapImageDiffRect::write(Writer* w) const throw (WritingException)
{
	if(!imageDiffRect)
		throw WritingException("Cannot serialize if imageDiffRect is 0.");
		
	w->startWriting(getClassName());
		w->writeString(objectClassName);
		w->write_f64(refMapImageX);
		w->write_f64(refMapImageY);
		w->write_f64(refMapImageTheta);
		w->write_f64(refMapImageRealWidth);
		w->writeObject(true, imageDiffRect);
	w->doneWriting();
}

vector<ObjectDiff*> RMapImageDiffRect::split(size_t maxSize)
{
	vector<ObjectDiff*> ret;
	vector<ObjectDiff*> v = imageDiffRect->split(maxSize);
	for (size_t i = 0; i < v.size(); i++) {
		RMapImageDiffRect* curDiff = new RMapImageDiffRect(*this);
		curDiff->imageDiffRect = dynamic_cast<RImageDiffRect*>(v[i]);
		ret.push_back(curDiff);
	}
	return ret;
}

}} // namespaces
