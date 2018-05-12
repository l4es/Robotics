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

#include <ctype.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string>
#include <cstring>
#include <list>

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RImage"
 
#include "rimage.h"

namespace RDK2 { namespace RGraphics {
	
using namespace std;

RDK2_FACTORY(RImage);
	
void RImage::initBuffer(int w, int h, Type t, void* buf, unsigned char fillValue, bool freeOldBuffer)
{
	this->type = t;
	this->width = w;
	this->height = h;
	this->bpp = getBytesPerPixel(t);
	
	if (freeOldBuffer && buffer) {
		delete[] buffer;
		delete[] rows;
	}
	
	this->bufferSize = width * height * bpp;
	this->dataSize = this->bufferSize;
	this->buffer = new unsigned char[bufferSize];

	if (buf) memcpy(buffer, buf, bufferSize);
	else memset(buffer, fillValue, bufferSize);
	
	size_t rowSize = w * bpp;
	this->rows = new unsigned char*[height];
	for (size_t y = 0; y < height; y++) this->rows[y] = buffer + rowSize * y;
}

RImage::RImage(): buffer(0)
{
	initBuffer(1, 1, RImage::C8, 0, 0);
}

RImage::RImage(int w, int h, Type type, unsigned char* buf): buffer(0)
{
	initBuffer(w, h, type, buf, 0);
}

RImage::RImage(const RImage& img): buffer(0)
{
	initBuffer(img.width, img.height, img.type, img.buffer, 0);
}

size_t RImage::getBytesPerPixel(RImage::Type t)
{
	switch (t) {
		case RGB32: return 4;
		case RGB24: return 3;
		case GREY: return 1;
		case C8: return 1;
		case YUYV: return 2;
		case JPEG: return 3;	// XXX
	}
	return 0;
}

string RImage::typeToString(RImage::Type t)
{
	switch (t) {
		case RGB32: return "RGB32";
		case RGB24: return "RGB24";
		case GREY: return "GREY";
		case C8: return "C8";
		case YUYV: return "YUYV";
		case JPEG: return "JPEG";
		default: return "?";
	}
}

RDK2::Object* RImage::clone() const 
{
	RImage* ri = new RImage(*this);
	return ri;
}

RImage& RImage::operator=(const RImage& img)
{
	initBuffer(img.width, img.height, img.type, img.buffer, 0);
	return *this;
}

RImage::~RImage()
{
	delete[] buffer;
	delete[] rows;
}

void RImage::setSizeAndType(size_t w, size_t h, Type t)
{
	if (w == width && h == height && t == type) return;
	initBuffer(w, h, t, 0, 0);
}

void RImage::fillBuffer(unsigned char fillValue)
{
	memset(buffer, fillValue, bufferSize);
}

void RImage::canvasResize(size_t w, size_t h, int x, int y, unsigned char fillValue)
{
	//printf("Canvas resize from %d x %d to %d x %d (%d, %d)\n", width, height, w, h, x, y);

	// the area of the original image to be copied
	int begin_u = std::min((int) width, std::max(0, x));
	int end_u = std::max(0, std::min((int) width, (int) (x + w)));
	int begin_v = std::min((int) height, std::max(0, y));
	int end_v = std::max(0, std::min((int) height, (int) (y + h)));

	//printf("Begin u = %d, end u = %d\n", begin_u, end_u);
	//printf("Begin v = %d, end v = %d\n", begin_v, end_v);
	
	// the area in the new canvas onto which the old image must be copied
	int begin_nu = begin_u - x;
	// int end_nu = end_u - x; // unused 
	int begin_nv = begin_v - y;
	int end_nv = end_v - y;

	//printf("begin nu = %d, end nu = %d\n", begin_nu, end_nu);
	//printf("begin nv = %d, end nv = %d\n", begin_nv, end_nv);

	unsigned char* oldbuf = this->buffer;
	unsigned char** oldrows = this->rows;

	//printf("initializing the new buffer to %d x %d\n", w, h);	

	initBuffer(w, h, this->type, 0, fillValue, false); 

	//printf("initialized\n");

	// LM: changing size_t (comaprison between signed and unsigned int)
	// considering changing into size_t
	for (int i = begin_nv; i < end_nv; i++) {
		//printf("Copying row %d + %d * %d to row %d + %d * %d; size %d\n", i+y, begin_u, bpp, i, begin_nu, bpp, (end_u - begin_u) * bpp);
		memcpy(rows[i] + begin_nu * bpp, oldrows[i + y] + begin_u * bpp, (end_u - begin_u) * bpp);
		//memcpy((void*) (buffer[i * width * bpp] + begin_nu * bpp),
		//	(void*) (oldbuf[i * y * bpp] + begin_u * bpp),
		//	(end_u - begin_u) * bpp);
	}
	delete[] oldbuf;
	delete[] oldrows;
}

void RImage::imageResize(size_t w, size_t h)
{
	if (w == width && h == height) return;
	unsigned char* oldbuf = buffer;
	unsigned char** oldrows = rows;
	size_t oldw = width, oldh = height;

	initBuffer(w, h, this->type, 0, 0, false);

	for (size_t y = 0; y < height; y++) {
		if (y*(double)(oldh/height) > oldh) continue;
		for (size_t x = 0; x < width; x++) {
			if (x*(double)(oldw/width) > oldw) continue;
			for (size_t i = 0; i < bpp; i++) {
				rows[y][x * bpp + i] = oldrows[(int)(y*((double)oldh/height))][(int)(x*bpp*((double)oldw/width))+i];

			}
		}
	}

	delete[] oldbuf;
	delete[] oldrows;
}

}} // namespaces
