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

#include "rimage.h"
#include "colorspaceconversions.h"

#include <rdkcore/interop/magick++.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RImage_Conversions"

#ifdef MagickPP_FOUND
#include <Magick++.h>
#endif

#include <cstring>

/*
 * YUV <-> YCbCr
 */
//#define CLIP(x) ( x < 0 ? 0 : ( x > 255 ? 255 : x) )
//#define YUV422toR(y,u,v) ( y + ((1436*(v - 128)) >> 10) )
//#define YUV422toG(y,u,v) ( y - ((732*(v - 128) - 354*(u - 128)) >> 10) )
//#define YUV422toB(y,u,v) ( y + ((1814*(u - 128)) >> 10) )
//#define YUV422toR(y,u,v) CLIP( y + 1.370705*(v - 128) )
//#define YUV422toG(y,u,v) CLIP( y - 0.698001*(v - 128) - 0.337633*(u - 128) )
//#define YUV422toB(y,u,v) CLIP( y                     + 1.732446*(u - 128) )

namespace RDK2 { namespace RGraphics {

using namespace std;

RImage* RImage::convertTo(Type t) const
{
	RImage* img = 0;
	if (type == t) return new RImage(*this);
	switch (t) {
		case RGB32: img = toRGB32(); break;
		case RGB24: img = toRGB24(); break;
		case GREY: img = toGrey(); break;
		case C8: img = toC8(); break;
		case JPEG: img = toJPEG(); break;
		default: break;
	}
	if (!img) {
		RDK_ERROR_PRINTF("Conversion not implemented (%s to %s)",
			RImage::typeToString(type).c_str(),
			RImage::typeToString(t).c_str());
	}
	return img;
}

RImage* RImage::toRGB32() const
{
	RImage* img = new RImage(this->width, this->height, RImage::RGB32);
	switch (type) {
		case RImage::RGB24:
			for (size_t s = 0, d = 0; s < this->bufferSize; ) {
				img->buffer[d++] = this->buffer[s++];
				img->buffer[d++] = this->buffer[s++];
				img->buffer[d++] = this->buffer[s++];
				img->buffer[d++] = 255;
			}
			break;
		case RImage::GREY:
			for (size_t s = 0, d = 0; s < this->bufferSize; ) {
				// LM this could be a bug
				// unsigned char c = (unsigned int) (this->buffer[s] + this->buffer[s+1] + this->buffer[s+2]) / 3;
				// s += 3;
				unsigned char c = this->buffer[s];
				s++;
				img->buffer[d++] = c;
				img->buffer[d++] = c;
				img->buffer[d++] = c;
				img->buffer[d++] = 255;
			}
			break;
		case RImage::YUYV:
			for (size_t s = 0, d = 0; s < this->bufferSize; ) {
				unsigned char y1 = this->buffer[s++];
				unsigned char u = this->buffer[s++];
				unsigned char y2 = this->buffer[s++];
				unsigned char v = this->buffer[s++];
				unsigned char r, g, b;
				ColorSpaceUtils::convertFromYUVToRGB(y1,u,v,r,g,b);
				img->buffer[d++] = r;
				img->buffer[d++] = g;
				img->buffer[d++] = b;
				img->buffer[d++] = 255;
				ColorSpaceUtils::convertFromYUVToRGB(y2,u,v,r,g,b);
				img->buffer[d++] = r;
				img->buffer[d++] = g;
				img->buffer[d++] = b;
				img->buffer[d++] = 255;
			}
			break;
		case RImage::C8:
			for (size_t s = 0, d = 0; s < this->bufferSize; ) {
				switch (buffer[s++]) {
					case C8White:   img->buffer[d++] = 255; img->buffer[d++] = 255; img->buffer[d++] = 255; break;
					case C8Black:   img->buffer[d++] = 0;   img->buffer[d++] = 0;   img->buffer[d++] = 0;   break;
					case C8Blue:    img->buffer[d++] = 0;   img->buffer[d++] = 0;   img->buffer[d++] = 255; break;
					case C8Red:     img->buffer[d++] = 255; img->buffer[d++] = 0;   img->buffer[d++] = 0;   break;
					case C8Green:   img->buffer[d++] = 0;   img->buffer[d++] = 255; img->buffer[d++] = 0;   break;
					case C8Magenta: img->buffer[d++] = 255; img->buffer[d++] = 0;   img->buffer[d++] = 255; break;
					case C8Cyan:    img->buffer[d++] = 0;   img->buffer[d++] = 255; img->buffer[d++] = 255; break;
					case C8Grey:    img->buffer[d++] = 128; img->buffer[d++] = 128; img->buffer[d++] = 128; break;
					case C8Unset:   img->buffer[d++] = 128; img->buffer[d++] = 128; img->buffer[d++] = 128; break;
				}
				img->buffer[d++] = 255;
			}
			break;
		case RImage::JPEG: {
#ifdef MagickPP_FOUND
			Magick::Blob blob(buffer, bufferSize);
			Magick::Image mimg(blob);
			delete img;
			img = convertToRImage(mimg);
#endif
			} break;
		default: delete img; img = 0; break;
	}
	return img;
}

RImage* RImage::toRGB24() const
{
	RImage* img = 0;
	switch (type) {
		case RImage::RGB32:
			img = new RImage(this->width, this->height, RImage::RGB24);
			for (size_t s = 0, d = 0; s < this->bufferSize; ) {
				img->buffer[d++] = this->buffer[s++];
				img->buffer[d++] = this->buffer[s++];
				img->buffer[d++] = this->buffer[s++];
				s++;
			}
			break;
		case RImage::GREY:
			img = new RImage(this->width, this->height, RImage::RGB24);
			for (size_t s = 0, d = 0; s < this->bufferSize; ) {
				unsigned char c = this->buffer[s];
				s++;
				img->buffer[d++] = c;
				img->buffer[d++] = c;
				img->buffer[d++] = c;
			}
			break;
		case RImage::YUYV:
			img = new RImage(this->width, this->height, RImage::RGB24);
			for (size_t s = 0, d = 0; s < this->bufferSize; ) {
				unsigned char y1 = this->buffer[s++];
				unsigned char u = this->buffer[s++];
				unsigned char y2 = this->buffer[s++];
				unsigned char v = this->buffer[s++];
				unsigned char r, g, b;
				ColorSpaceUtils::convertFromYUVToRGB(y1,u,v,r,g,b);
				img->buffer[d++] = r;
				img->buffer[d++] = g;
				img->buffer[d++] = b;
				ColorSpaceUtils::convertFromYUVToRGB(y2,u,v,r,g,b);
				img->buffer[d++] = r;
				img->buffer[d++] = g;
				img->buffer[d++] = b;
			}
			break;
		case RImage::C8:
			for (size_t s = 0, d = 0; s < this->bufferSize; ) {
				switch (buffer[s++]) {
					case C8White:   img->buffer[d++] = 255; img->buffer[d++] = 255; img->buffer[d++] = 255; break;
					case C8Black:   img->buffer[d++] = 0;   img->buffer[d++] = 0;   img->buffer[d++] = 0;   break;
					case C8Blue:    img->buffer[d++] = 0;   img->buffer[d++] = 0;   img->buffer[d++] = 255; break;
					case C8Red:     img->buffer[d++] = 255; img->buffer[d++] = 0;   img->buffer[d++] = 0;   break;
					case C8Green:   img->buffer[d++] = 0;   img->buffer[d++] = 255; img->buffer[d++] = 0;   break;
					case C8Magenta: img->buffer[d++] = 255; img->buffer[d++] = 0;   img->buffer[d++] = 255; break;
					case C8Cyan:    img->buffer[d++] = 0;   img->buffer[d++] = 255; img->buffer[d++] = 255; break;
					case C8Grey:    img->buffer[d++] = 128; img->buffer[d++] = 128; img->buffer[d++] = 128; break;
					case C8Unset:   img->buffer[d++] = 128; img->buffer[d++] = 128; img->buffer[d++] = 128; break;
				}
			}
			break;
		case RImage::JPEG: {
#ifdef MagickPP_FOUND
			Magick::Blob blob(buffer, bufferSize);
			Magick::Image mimg(blob);
			delete img;
			img = convertToRImage(mimg);
#endif
			} break;
		default: delete img; img = 0; break;
	}
	return img;
}

RImage* RImage::toGrey() const
{
	RImage* img = 0;
	switch (type) {
		case RImage::RGB32:
			break;
		case RImage::RGB24:
			break;
		case RImage::C8:
			break;
		case RImage::JPEG:
			break;
		default: delete img; img = 0; break;
	}
	return img;
}

RImage* RImage::toC8() const
{
	RImage* img = 0;
	switch (type) {
		case RImage::RGB32:
			break;
		case RImage::GREY:
			break;
		case RImage::RGB24:
			break;
		default: delete img; img = 0; break;
	}
	return img;
}

RImage* RImage::toJPEG() const
{
#ifdef MagickPP_FOUND
	Magick::Image img;
	if (this->type == RImage::RGB32 || this->type == RImage::RGB24) {
		img = Magick::Image(this->width, this->height,
							(this->type == RImage::RGB32 ? "RGBA" : "RGB"), Magick::CharPixel, this->buffer);
	}
	else {
		RImage* r24 = this->convertTo(RImage::RGB24);
		if (!r24) return 0;
		img = Magick::Image(r24->width, r24->height, "RGB", Magick::CharPixel, r24->buffer);
		delete r24;
	}
	Magick::Blob blob;
	img.write(&blob, "jpeg");

	RImage* rimg = new RImage(this->width, this->height, RImage::JPEG);
	memcpy(rimg->buffer, blob.data(), blob.length());
	rimg->dataSize = blob.length();
	return rimg;
#else
	return 0;
#endif
}

}} // namespaces
