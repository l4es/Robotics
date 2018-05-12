#define MODULE_NAME "V4LModule"

#include "v4lmodule.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/rgraphics/rimage.h>
#define LOGGING_MODULE MODULE_NAME

#include <cstring>

#include <libv4l2.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/videodev2.h>
// LM: please DC fix this ugliness
#ifdef OpenRDK_ARM9_ARCH
#include <linux/../asm-generic/ioctl.h>
#endif

#define PROPERTY_DEVICE_NAME "params/deviceName"
#define PROPERTY_OUT_IMAGE "out/image"
#define PROPERTY_IMAGE_FORMAT "params/imageFormat"

namespace RDK2 { namespace RAgent {

bool V4LModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		session->createString(PROPERTY_DEVICE_NAME, "Device name", "/dev/video0");
		session->createEmptyImage(PROPERTY_OUT_IMAGE, "Current image");
		session->createString(PROPERTY_IMAGE_FORMAT, "Image format (can be RGB24, RGB32 or YUYV)", "RGB24");
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool V4LModule::init()
{
	SESSION_TRY_START(session)
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
		string imgfmt = session->getString(PROPERTY_IMAGE_FORMAT);
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
		
		session->listenToTimer(500.);
	SESSION_END(session) 
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void V4LModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
		
			RImage::Type t = RImage::RGB24;
			string format = session->getString(PROPERTY_IMAGE_FORMAT);
			if (format == "RGB24") t = RImage::RGB24;
			else if (format == "RGB32") t = RImage::RGB32;
			else if (format == "YUYV") t = RImage::YUYV;
			else {
				RDK_ERROR_PRINTF("Unknown image format '%s'", format.c_str());
				SESSION_CONTINUE(session);
			}

			if (!session->isSet(PROPERTY_OUT_IMAGE)) session->setObject(PROPERTY_OUT_IMAGE, new RImage(320, 240, t));
			
			session->lock(PROPERTY_OUT_IMAGE, HERE);
			RImage* img = session->getObjectAsL<RImage>(PROPERTY_OUT_IMAGE);
			v4l2_read(fd, img->getBuffer(), img->getBufferSize());
			session->unlock(PROPERTY_OUT_IMAGE);
			session->valueChanged(PROPERTY_OUT_IMAGE);
 
		SESSION_END_CATCH_TERMINATE(session)
	}
}

void V4LModule::cleanup()
{
	if (fd != -1) v4l2_close(fd);
}

MODULE_FACTORY(V4LModule);

}} // namespace
