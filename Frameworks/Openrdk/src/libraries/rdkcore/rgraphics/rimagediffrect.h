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

#ifndef RDK2_RGRAPHICS_RIMAGEDIFFRECT
#define RDK2_RGRAPHICS_RIMAGEDIFFRECT

#include <rdkcore/object/object.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/object/objectdiff.h>
#include <string>

namespace RDK2 { namespace RGraphics {

using namespace std;

class RImageDiffRect: public ObjectDiff  {
public:
	RImageDiffRect();
	RImageDiffRect(RImage* image, int left, int top, int width, int height);
	RImageDiffRect(RImageDiffRect* imagediffRect, int left, int top, int width, int height);
	virtual ~RImageDiffRect();
	
	void read(RDK2::Reader*r) throw (RDK2::ReadingException);
	void write(RDK2::Writer*w) const throw (RDK2::WritingException);

	RDK2::Object* clone() const;

	virtual vector<ObjectDiff*> split(size_t maxBufferSize);

	int rectLeft, rectTop, rectWidth, rectHeight;
	int refImageWidth, refImageHeight;
	RImage::Type refImageType;

	uint8_t** getPixels();
	uint8_t* getBuffer();

	size_t getCBufSize() { return cBuf.size(); }

private:
	void initBuffer(int width, int height, RImage::Type type);
	void copyRect(uint8_t* rows, int top, int left, int width, int height, RImage::Type type);

	uint8_t* buffer;
	uint8_t** rows;
	mutable vector<uint8_t> cBuf;	// compressed buffer

friend class RImage;
};

}}

#endif
