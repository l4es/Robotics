#include "magick++.h"

#include <cstring>

namespace RDK2 {

using namespace RDK2::RGraphics;
using namespace std;

#ifdef MagickPP_FOUND

Magick::Image convertToMagickImage(const RImage* rimg)
{
	string m;
	bool converted = false;
	RImage* rimgc = 0;
	switch (rimg->getType()) {
		case RImage::GREY: m = "K"; break;
		case RImage::RGB24: m = "RGB"; break;
		case RImage::RGB32: m = "RGBA"; break;
		default: rimgc = rimg->convertTo(RImage::RGB24); converted = true; m = "RGB"; break;
	}
	if (converted) {
		Magick::Image mimg(rimgc->getWidth(), rimgc->getHeight(), m, Magick::CharPixel, rimgc->getBuffer());
		delete rimgc;
		return mimg;
	}
	else {
		Magick::Image mimg(rimg->getWidth(), rimg->getHeight(), m, Magick::CharPixel, rimg->getBuffer());
		return mimg;
	}
}

RImage* convertToRImage(const Magick::Image& mmimg)
{
	Magick::Image mimg = mmimg;
	RImage::Type t;
	switch (mimg.type()) {
		case Magick::GrayscaleType: t = RImage::GREY; break;
		case Magick::TrueColorType: t = RImage::RGB24; break;
		case Magick::TrueColorMatteType: t = RImage::RGB32; break;
		default: printf("ERROR rdkcore/interop/magick++.cpp:42: Magick::type=%d\n",mimg.type()); return 0;
	}
	RImage* rimg = new RImage(mimg.columns(), mimg.rows(), t);
	unsigned char* buffer = rimg->getBuffer();
	size_t i = 0;
	for (size_t y = 0; y < mimg.rows(); y++) {
		for (size_t x = 0; x < mimg.columns(); x++) {
			Magick::ColorRGB c = mimg.pixelColor(x, y);
			if (t == RImage::GREY) {
				buffer[i++] = c.red() / 3 + c.blue() / 3 + c.green() / 3;
			}
			else if (t == RImage::RGB24 || t == RImage::RGB32) {
				buffer[i++] = (unsigned char) (c.red() * 255);
				buffer[i++] = (unsigned char) (c.green() * 255);
				buffer[i++] = (unsigned char) (c.blue() * 255);
				if (t == RImage::RGB32) buffer[i++] = 255;
			}
		}
	}
	//memcpy(rimg->getBuffer(), mimg.getConstPixels(0, 0, mimg.columns(), mimg.rows()), rimg->getBufferSize());
	//(void) mimg.getPixels(0, 0, mimg.columns(), mimg.rows());
	/*switch (t) {
		case RImage::GREY: mimg.writePixels(Magick::GrayQuantum, rimg->getBuffer()); break;
		case RImage::RGB24: mimg.writePixels(Magick::RGBQuantum, rimg->getBuffer()); break;
		case RImage::RGB32: mimg.writePixels(Magick::RGBAQuantum, rimg->getBuffer()); break;
	}*/
	return rimg;
}

#endif

}
