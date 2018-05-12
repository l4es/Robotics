/**
 * @file
 *
 * @brief This file contains methods and other stuff of V4L2CameraDevice class
 */

#ifndef V4L_UTILS
#define V4L_UTILS

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <memory.h>

typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
} IOMethod;

int v4lu_open_device(const char* dev_name, bool uselibv4l = false);
bool v4lu_init_device(int fd, unsigned int desired_format, int width, int height, int fps, IOMethod io, bool do_setting, bool uselibv4l = false);
void v4lu_start_capturing(int fd, IOMethod io);
bool v4lu_get_frame(int fd, IOMethod io, bool uselibv4l = false);
void *v4lu_get_buffer(IOMethod io);
void v4lu_stop_capturing(int fd, IOMethod io);
void v4lu_uninit_device(int fd, IOMethod io);
void v4lu_close_device(int fd, bool uselibv4l = false);

#endif
