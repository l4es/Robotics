#define MODULE_NAME "Video4LinuxModule"

#include "video4linuxmodule.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/time/time.h>
#include <rdkcore/config.h>
#define LOGGING_MODULE MODULE_NAME

#ifdef libv4l2_FOUND
#include <libv4l2.h>
#endif

#include <cstring>
#include <algorithm>

#define PROPERTY_OUT_IMAGE  "out/image"

#define PROPERTY_FORMAT      "params/imageFormat"
#define PROPERTY_WIDTH       "params/imageWidth"
#define PROPERTY_HEIGHT      "params/imageHeight"
#define PROPERTY_DEVICE_NAME "params/deviceName"
#define PROPERTY_FPS         "params/fps"
#define PROPERTY_IOMETHOD    "params/iomethod"
#define PROPERTY_LIBV4L2     "params/libv4l2"

namespace RDK2 { namespace RAgent {

bool Video4LinuxModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)

	Common::createDefaultProperties(session, true);
	session->createString(PROPERTY_FORMAT, "Image format (YUYV, RGB24, RGB32)", "RGB24");
	session->createInt(PROPERTY_WIDTH, "Image width", 320);
	session->createInt(PROPERTY_HEIGHT, "Image height", 240);
	session->createBool(PROPERTY_LIBV4L2, "Use libv4l2 conversions", false);
	session->createInt(PROPERTY_FPS, "Camera FPS", 10);
	session->createString(PROPERTY_DEVICE_NAME, "Device name", "/dev/video0");
	session->createString(PROPERTY_IOMETHOD, "IO Method (read, mmap)", "read");
	session->createEmptyImage(PROPERTY_OUT_IMAGE, "Current Image");

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool Video4LinuxModule::init()
{
	SESSION_TRY_START(session)

	int fps = session->getInt(PROPERTY_FPS);
	double tim = 1000.0 / fps;
	cout << "V4L2Module: timer (ms) " << tim << endl;
	session->listenToTimer(tim);

	if (session->getBool(PROPERTY_LIBV4L2)) {
#ifndef libv4l2_FOUND
		RDK_ERROR_PRINTF("You asked for libv4l2 conversions, but there was no libv4l2 "
			"library in your system when you compiled this module");
		session->end();
		return false;
#else
		fd = v4l2_open(session->getString(PROPERTY_DEVICE_NAME).c_str(), O_RDWR);
		if (fd == -1) {
			RDK_ERROR_PRINTF("Cannot open device '%s'", session->getString(PROPERTY_DEVICE_NAME).c_str());
			return false;
		}
		fd = v4l2_fd_open(fd, V4L2_ENABLE_ENUM_FMT_EMULATION);
		if (fd == -1) {
			RDK_ERROR_PRINTF("Cannot set extended libv4l flags on device '%s'", session->getString(PROPERTY_DEVICE_NAME).c_str());
			return false;
		}
		
		v4l2_fmtdesc fmtdesc;
		fmtdesc.index = 0;
		fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		printf("Supported formats:\n");
		while (v4l2_ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
			printf("  %s\n", fmtdesc.description);
			fmtdesc.index++;
		}
		
		struct v4l2_format format;
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		format.fmt.pix.width = 320;
		format.fmt.pix.height = 240;
		string imgfmt = session->getString(PROPERTY_FORMAT);
		if (imgfmt == "RGB24") format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
		else if (imgfmt == "RGB32") format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB32;
		else if (imgfmt == "YUYV") format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		else {
			RDK_ERROR_PRINTF("Unknown output image format '%s'", imgfmt.c_str());
			return false;
		}
		format.fmt.pix.field = V4L2_FIELD_NONE;
		format.fmt.pix.bytesperline = 320 * 3;
		int r = v4l2_ioctl(fd, VIDIOC_S_FMT, &format);
		if (r != 0) {
			RDK_ERROR_PRINTF("Cannot set capture format on device '%s'", session->getString(PROPERTY_DEVICE_NAME).c_str());
			return false;
		}
#endif
	}
	else {
		camera = new V4L2CameraDevice(session->getString(PROPERTY_DEVICE_NAME).c_str());
		IOMethod iom = IO_METHOD_READ;
		if (session->getString(PROPERTY_IOMETHOD) == "mmap") iom = IO_METHOD_MMAP;

		string format = session->getString(PROPERTY_FORMAT);
		int w = session->getInt(PROPERTY_WIDTH), h = session->getInt(PROPERTY_HEIGHT);
		if (format == "YUYV") camera->init(V4L2_PIX_FMT_YUYV, w, h, fps, iom);
		else if (format == "RGB24") camera->init(V4L2_PIX_FMT_RGB24, w, h, fps, iom);
		else if (format == "RGB32") camera->init(V4L2_PIX_FMT_RGB32, w, h, fps, iom);
		else {
			RDK_ERROR_PRINTF("Unsupported RImage format: '%s'", format.c_str());
			session->end();
			return false;
		}
	}

	string format = session->getString(PROPERTY_FORMAT);
	int w = session->getInt(PROPERTY_WIDTH), h = session->getInt(PROPERTY_HEIGHT);
	if (format == "YUYV") session->setObject(PROPERTY_OUT_IMAGE, new RImage(w, h, RImage::YUYV));
	else if (format == "RGB24") session->setObject(PROPERTY_OUT_IMAGE, new RImage(w, h, RImage::RGB24));
	else if (format == "RGB32") session->setObject(PROPERTY_OUT_IMAGE, new RImage(w, h, RImage::RGB32));
	else {
		RDK_ERROR_PRINTF("Unsupported RImage format: '%s'", format.c_str());
		session->end();
		return false;
	}
	
	SESSION_END(session) 
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void Video4LinuxModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)

		bool uselibv4l2 = false;
		if (session->getBool(PROPERTY_LIBV4L2)) {
#ifdef libv4l2_FOUND
			uselibv4l2 = true;
#else
			RDK_ERROR_PRINTF("No libv4l2 support");
#endif
		}

		if (!uselibv4l2) {
			if (camera->ready()) {
				camera->getFrame();

				session->lock(PROPERTY_OUT_IMAGE, HERE);
				RImage *image = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE);
				size_t sz = std::min((size_t) image->getBufferSize(),
					(size_t) camera->image.width * camera->image.height * camera->image.bpp);
				memcpy(image->getBuffer(), camera->image.data, sz);
				session->unlock(PROPERTY_OUT_IMAGE);
				session->valueChanged(PROPERTY_OUT_IMAGE);
			}
			else {
				session->lock(PROPERTY_OUT_IMAGE, HERE);
				RImage *image = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE);

				if (session->getString(PROPERTY_FORMAT)!="RGB24") {
					image->setSizeAndType(image->getWidth(),image->getHeight(),RDK2::RGraphics::RImage::RGB24);
				}

				unsigned char *buf = image->getBuffer();
				unsigned int k=0;
				while (k<DEFAULT_HEIGHT) {
					unsigned char r,g,b;
					if (k<DEFAULT_HEIGHT/4) { r=255; g=0; b=0; }
					else if (k<DEFAULT_HEIGHT/2) { r=0; g=255; b=0; }
					else if (k<DEFAULT_HEIGHT*3/4) { r=0; g=0; b=255; }
					else { r=255; g=255; b=0; }
					for (unsigned int i=0; i<DEFAULT_WIDTH; i++) { *buf=r; *(buf+1)=g; *(buf+2)=b; buf+=3; }
					k++;
				}

				session->unlock(PROPERTY_OUT_IMAGE);
				session->valueChanged(PROPERTY_OUT_IMAGE);
			}
		}
		else {
			size_t bpp = 1;
			string format = session->getString(PROPERTY_FORMAT);
			if (format == "YUYV") bpp = 2;
			else if (format == "RGB24") bpp = 3;
			else if (format == "RGB32") bpp = 4;
			else SESSION_CONTINUE(session);
			size_t readsize = session->getInt(PROPERTY_WIDTH) * session->getInt(PROPERTY_HEIGHT) * bpp;
			unsigned char* buf = new unsigned char[readsize];
#ifdef libv4l2_FOUND
			v4l2_read(fd, buf, readsize);
#endif

			session->lock(PROPERTY_OUT_IMAGE, HERE);
			RImage *image = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE);
			memcpy(image->getBuffer(), buf, readsize);
			session->unlock(PROPERTY_OUT_IMAGE);
			session->valueChanged(PROPERTY_OUT_IMAGE);
			delete[] buf;
		}

		SESSION_END_CATCH_TERMINATE(session)
	}
}

void Video4LinuxModule::cleanup()
{ 
	if (camera) {
		camera->done();
		delete camera;
	}
	else if (fd != -1) {
#ifdef libv4l2_FOUND
		v4l2_close(fd);
#endif
	}
}

MODULE_FACTORY(Video4LinuxModule);

}} // namespace
