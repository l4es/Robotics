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

#include <cstring>
#include <string>
#include <list>
using namespace std;

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RImage"

#include "rimage.h"

namespace RDK2 { namespace RGraphics {

uint8_t reduceC8(uint8_t color)
{
	switch (color) {
		case RImage::C8Black:   return 0;
		case RImage::C8Blue:    return 1;
		case RImage::C8Green:   return 2;
		case RImage::C8Cyan:    return 3;
		case RImage::C8Red:     return 4;
		case RImage::C8Magenta: return 5;
		case RImage::C8White:   return 6;
		case RImage::C8Grey:    return 7;
	}
	return 0;
}

uint8_t expandC8(uint8_t color)
{
	switch (color) {
		case 0: return RImage::C8Black;
		case 1: return RImage::C8Blue;
		case 2: return RImage::C8Green;
		case 3: return RImage::C8Cyan;
		case 4: return RImage::C8Red;
		case 5: return RImage::C8Magenta;
		case 6: return RImage::C8White;
		case 7: return RImage::C8Grey;
	}
	return RImage::C8Blue;
}

void RImage::writeImpl(Writer* w) const throw (WritingException)
{
	w->write_i32(width, "width");
	w->write_i32(height, "height");
	w->write_i32(type, "type");
	if (this->type == RImage::C8) {
		unsigned char rlebuf[bufferSize];
		int p = 0;
		unsigned char c = buffer[0];
		int n = 0;
		for (size_t i = 1; i < bufferSize; i++) {
			if (buffer[i] == c && n < 31) {
				n++;
			}
			else {
				rlebuf[p++] = (reduceC8(c) & 7) | (n << 3);
				n = 0;
				c = buffer[i];
			}
		}
		rlebuf[p++] = (c & 7) | (n << 3);
		w->writeBytes(rlebuf, p);
	}
	else {
		w->writeBytes(buffer, bufferSize);
	}
}

void RImage::write(Writer* w) const throw (WritingException)
{
	w->startWriting(getClassName());
	writeImpl(w);
	w->doneWriting();
}

void RImage::readImpl(Reader* r) throw (ReadingException) {

	int w = r->read_i32("width");
	int h = r->read_i32("height");
	Type t = (Type) r->read_i32("type");
	setSizeAndType(w, h, t);
	if (type == RImage::C8) {
		size_t readBytes;
		void* voidrlebuf;
		r->readBytes(&voidrlebuf, &readBytes);
		unsigned char* rlebuf = reinterpret_cast<unsigned char*>(voidrlebuf);
		size_t n = 0;
		unsigned char c;
		size_t s = 0;
		for (size_t i = 0; i < readBytes; i++) {
			c = expandC8(rlebuf[i] & 7);
			n = rlebuf[i] >> 3;
			if ((s + n) >= bufferSize) throw ReadingException("Something wrong in RLE decoding");
			for (int j = 0; j <= (int)n; j++) buffer[s++] = c;
		}
		delete[] rlebuf;
	}
	else {
		size_t readBytes;
		unsigned char* readbuffer;
		r->readBytes((void**) (&readbuffer), &readBytes);
		memcpy(buffer, readbuffer, readBytes);
		delete[] readbuffer;
	}
}

void RImage::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
	readImpl(r);
	r->doneReading();
}

}} // namespaces
