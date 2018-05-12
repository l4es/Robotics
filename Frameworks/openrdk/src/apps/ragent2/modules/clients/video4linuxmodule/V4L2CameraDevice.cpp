/**
 * @file
 *
 * @brief This file contains methods and other stuff of V4L2CameraDevice class
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <memory.h>

#include "V4L2CameraDevice.h"
#include "v4l_utils.h"

using namespace std;

V4L2CameraDevice::~V4L2CameraDevice()
{
    if (deviceReady) { // User should have called done() !
	done();
    }
}

void V4L2CameraDevice::done()
{
    if (!deviceReady) {
	fprintf(stderr, "V4L2CameraDevice::done() called without doing init()"
		"\n");
	return;
    }
    v4lu_stop_capturing(fd,io_method);
    v4lu_uninit_device(fd,io_method);
    v4lu_close_device(fd);
    deviceReady = false;
}

void V4L2CameraDevice::init(uint type, int width, int height, int fps, IOMethod io, bool do_setting)
{
    this->width=width; this->height=height; this->io_method=io;
    if (deviceReady) {
		fprintf(stderr, "V4L2CameraDevice::init() already called\n");
		return;
    }
    fd = v4lu_open_device(deviceName.c_str());
    if (fd == -1) { // Error!
	fprintf(stderr, "V4L2CameraDevice::init(): unable to open device %s: "
		"%s\n", deviceName.c_str(), strerror(errno));
	return;
    }
    v4lu_init_device(fd,type,width,height,fps,io_method,do_setting);
    image.alloc_mem(type,width, height);

    v4lu_start_capturing(fd,io_method);

	printf("Device ready.\n");
	
	deviceReady = true;
}


bool V4L2CameraDevice::ready()
{
    return deviceReady;
}


bool V4L2CameraDevice::getFrame()
{ 
    if (!deviceReady) {
		fprintf(stdout, "V4L2CameraDevice::getFrame(): device not ready\n");
		return false;
    }

    image.timestamp = getTimeStamp();

    bool r = v4lu_get_frame(fd,io_method);
    void * buf = v4lu_get_buffer(io_method);
    if (!r || (buf==NULL)) return false;

    size_t imageSize;
    imageSize = image.width * image.height * image.bpp;
    memcpy(image.data, buf, imageSize);

    return true;
}
