#include "opencv.h"

namespace RDK2 {

using namespace RDK2::RGraphics;

#ifdef OpenCV_FOUND

IplImage* createIplImageHeader(RImage* rimg)
{
        IplImage* cvimg = cvCreateImageHeader(cvSize(rimg->getWidth(), rimg->getHeight()), IPL_DEPTH_8U, rimg->getBytesPerPixel());
        cvimg->imageData = (char*) rimg->getBuffer();
        return cvimg;
}

IplImage* convertToIplImage(const RImage* rimg)
{
	size_t channels = 0;
	if (rimg->getType() == RImage::RGB24) channels = 3;
	else if (rimg->getType() == RImage::RGB32) channels = 4;
	else return 0;

	IplImage* iimg = cvCreateImage(cvSize(rimg->getWidth(), rimg->getHeight()), IPL_DEPTH_8U, channels);
	memcpy(iimg->imageData, rimg->getBuffer(), rimg->getBufferSize());
	return iimg;
}

RImage* convertToRImage(const IplImage* iimg)
{
	RImage* rimg = new RImage(iimg->width, iimg->height, RImage::RGB24);
	IplImage* iimg_out = createIplImageHeader(rimg);
	cvConvert(iimg, iimg_out);
	cvReleaseImageHeader(&iimg_out);
	return rimg;
}

#endif

}
