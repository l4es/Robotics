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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RImage_Magick"

#include <rdkcore/config.h>

#ifdef MagickPP_FOUND
#include <Magick++.h>
#endif

#include <cmath>
#include <cstring>
#include <sys/types.h>

namespace RDK2 { namespace RGraphics {

#ifdef MagickPP_FOUND
using namespace Magick;
#endif

bool RImage::loadMagick(std::string filename, double occupied, double equal, C8Color fillColor)
{
#ifndef MagickPP_FOUND
	RDK_ERROR_PRINTF("OpenRDK compiled without ImageMagick support, cannot load images using Magick++ library");
	return false;
#else
	Image image;
	try {
		image.read(filename);
		image.modifyImage();
		image.type(TrueColorType);
		image.colorSpace(RGBColorspace);
		if (type == RImage::C8 || type == RImage::GREY) {
			setSizeAndType(image.columns(), image.rows(), type);
			for (size_t y = 0; y < image.rows(); y++) {
				for (size_t x = 0; x < image.columns(); x++) {
					ColorRGB c = image.pixelColor(x, y);
					if (fabs(c.red() - c.blue()) < equal && fabs(c.green() - c.blue()) < equal) {
						if (1 - c.red() > occupied && 1 - c.blue() > occupied && 1 - c.green() > occupied ) {
							buffer[y * width + x] = C8Black;
						}
						else buffer[y * width + x] = C8White;
					}
					else buffer[y * width + x] = fillColor;
				}
			}
		}
		else {
			setSizeAndType(image.columns(), image.rows(), RImage::RGB32);
			for (size_t y = 0; y < image.rows(); y++) {
				for (size_t x = 0; x < image.columns(); x++) {
					ColorRGB c = image.pixelColor(x,y);
					buffer[(y * width + x) * 4] =     (unsigned char) (c.blue()  * 0xff);
					buffer[(y * width + x) * 4 + 1] = (unsigned char) (c.green() * 0xff);
					buffer[(y * width + x) * 4 + 2] = (unsigned char) (c.red()   * 0xff);
				}
			}
		}
		return true;
	}
	catch (const exception& e) {
		return false;
	}
#endif
}


bool RImage::saveMagick(std::string filename, bool trimImage)
{
#ifndef MagickPP_FOUND
	RDK_ERROR_PRINTF("OpenRDK compiled without ImageMagick support, cannot load images using Magick++ library");
	return false;
#else
	RImage* img = this->convertTo(RImage::RGB32);
	try {
		char* magickdimmerda = new char[img->width * img->height * getBytesPerPixel(RImage::RGB32)];
		memcpy(magickdimmerda, img->buffer, img->width * img->height * getBytesPerPixel(RImage::RGB32));
		Image image(img->width, img->height, "RGBA", CharPixel, magickdimmerda);
		image.backgroundColor(image.pixelColor(0, 0));
		image.matte(false);
		if (trimImage) image.trim();	// FIXME it doesn't work
		image.page(Magick::Geometry(0, 0));
		image.write(filename);
		delete img;
		return true;
	}
	catch (const exception& e) {
		delete img;
		return false;
	}
#endif
}

bool RImage::magickSaveToFile(const std::string& /*filename*/)
{
#ifndef MagickPP_FOUND
	RDK_ERROR_PRINTF("OpenRDK compiled without ImageMagick support, cannot save images using Magick++ library");
	return false;
#else
	RDK_WARNING_PRINTF("magickSaveToFile not implemented yet");
	return false;
#endif
}

bool RImage::magickSaveToMemory(const std::string& /*format*/, vector<unsigned char>& /*buffer*/)
{
#ifndef MagickPP_FOUND
	RDK_ERROR_PRINTF("OpenRDK compiled without ImageMagick support, cannot save images using Magick++ library");
	return false;
#else
	RDK_WARNING_PRINTF("magickSaveToMemory not implemented yet");
	return false;
#endif
}

bool RImage::magickLoadFromFile(const std::string& /*filename*/)
{
#ifndef MagickPP_FOUND
	RDK_ERROR_PRINTF("OpenRDK compiled without ImageMagick support, cannot load images using Magick++ library");
	return false;
#else
	RDK_WARNING_PRINTF("magickLoadFromFile not implemented yet");
	return false;
#endif
}

bool RImage::magickLoadFromMemory(const std::string& /*format*/, const vector<unsigned char>& /*img*/)
{
#ifndef MagickPP_FOUND
	RDK_ERROR_PRINTF("OpenRDK compiled without ImageMagick support, cannot load images using Magick++ library");
	return false;
#else
	RDK_WARNING_PRINTF("magickLoadFromMemory not implemented yet");
	return false;
#endif
}

}}
