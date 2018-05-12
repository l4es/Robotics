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

#include <libv4l2.h>

#include "V4L2CameraDevice.h"

// defined below in this file
int open_device(const char* dev_name);
bool init_device(int fd, unsigned int desired_format, int width, int height, int fps, IOMethod io, bool do_setting);
void start_capturing(int fd, IOMethod io);
bool get_frame(int fd, IOMethod io);
void *get_buffer(IOMethod io);
void stop_capturing(int fd, IOMethod io);
void uninit_device(int fd, IOMethod io);
void close_device(int fd);


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
    stop_capturing(fd,io_method);
    uninit_device(fd,io_method);
    close_device(fd);
    deviceReady = false;
}

void V4L2CameraDevice::init(uint type, int width, int height, int fps, IOMethod io, bool do_setting)
{
    this->width=width; this->height=height; this->io_method=io;
    if (deviceReady) {
		fprintf(stderr, "V4L2CameraDevice::init() already called\n");
		return;
    }
    fd = open_device(deviceName.c_str());
    if (fd == -1) { // Error!
	fprintf(stderr, "V4L2CameraDevice::init(): unable to open device %s: "
		"%s\n", deviceName.c_str(), strerror(errno));
	return;
    }
    init_device(fd,type,width,height,fps,io_method,do_setting);

    image.alloc_mem(type,width, height);

    start_capturing(fd,io_method);

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

    image.timestamp=getTimeStamp();

    bool r = get_frame(fd,io_method);
    void * buf = get_buffer(io_method);
    if (!r || (buf==NULL)) return false;

    size_t imageSize;
    imageSize = image.width * image.height * image.bpp;
    memcpy(image.data, buf, imageSize);

    return true;
}




/* ================================================================================================= */
/* ================================================================================================= */
/* ================================================================================================= */
/* ================================================================================================= */
/* ================================================================================================= */

/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))


struct buffer {
        void *                  start;
        size_t                  length;
};

static struct buffer *         buffers         = NULL;
static unsigned int     n_buffers       = 0;
static struct v4l2_buffer      buf;


void errno_exit   (const char *s)
{
        fprintf (stderr, "%s error %d, %s\n",
                 s, errno, strerror (errno));

        exit (EXIT_FAILURE);
}

int xioctl (int fd, int request, void *arg)
{
        int r;

        do r = v4l2_ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);

        return r;
}

void process_image (const void * /*p*/, int /*len*/)
{
/*
	printf(" %d ",len);
        fputc ('.', stdout);
        fflush (stdout);
*/

/*
	FILE *f = fopen("image.jpg","w");
	if (f) {
		fwrite(p,1,len,f);
		fclose(f);
	}
*/

}

int read_frame (int fd, IOMethod io)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
		 {int rr = v4l2_read (fd, buffers[0].start, buffers[0].length);
                if (-1 == rr) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */
				break;

                                /* fall through */
		      
                        default:
                                errno_exit ("read_frame");
                        }
                }

                process_image (buffers[0].start,rr);
		}

                break;

        case IO_METHOD_MMAP:
                CLEAR (buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit ("VIDIOC_DQBUF");
                        }
                }

                assert (buf.index < n_buffers);

                process_image (buffers[buf.index].start,buffers[buf.index].length);

                if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                        errno_exit ("VIDIOC_QBUF");

                break;

        case IO_METHOD_USERPTR:
                CLEAR (buf);

                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;

                if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;

                        case EIO:
                                /* Could ignore EIO, see spec. */

                                /* fall through */

                        default:
                                errno_exit ("VIDIOC_DQBUF");
                        }
                }

                for (i = 0; i < n_buffers; ++i)
                        if (buf.m.userptr == (unsigned long) buffers[i].start
                            && buf.length == buffers[i].length)
                                break;

                assert (i < n_buffers);

                process_image ((void *) buf.m.userptr, buf.length);

                if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                        errno_exit ("VIDIOC_QBUF");

                break;
        }

        return 1;
}

void * get_buffer(IOMethod io)
{
        switch (io) {
        case IO_METHOD_READ:
	    return buffers[0].start;
        case IO_METHOD_MMAP:
	    return  buffers[buf.index].start;
        case IO_METHOD_USERPTR:
	    return NULL; 
	}

	return NULL;
}


bool get_frame(int fd, IOMethod io)
{
	for (;;) {
		fd_set fds;
		struct timeval tv;
		int r;
	
		FD_ZERO (&fds);
		FD_SET (fd, &fds);
	
		/* Timeout. */
		tv.tv_sec = 2;
		tv.tv_usec = 0;
	
		r = select (fd + 1, &fds, NULL, NULL, &tv);
	
		if (-1 == r) {
			if (EINTR == errno)
				continue;
	
			errno_exit ("select");
		}
	
		if (0 == r) {
			//fprintf (stderr, "select timeout\n");
			// return false;
		}
	
		if (read_frame (fd,io))
			break;
	
		/* EAGAIN - continue select loop. */
	} // for

	return true;
}

void stop_capturing (int fd, IOMethod io)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
                        errno_exit ("VIDIOC_STREAMOFF");

                break;
        }
}

void start_capturing (int fd, IOMethod io)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR (buf);

                        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory      = V4L2_MEMORY_MMAP;
                        buf.index       = i;

                        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                                errno_exit ("VIDIOC_QBUF");
                }

                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
                        errno_exit ("VIDIOC_STREAMON");

                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR (buf);

                        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory      = V4L2_MEMORY_USERPTR;
                        buf.index       = i;
                        buf.m.userptr   = (unsigned long) buffers[i].start;
                        buf.length      = buffers[i].length;

                        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                                errno_exit ("VIDIOC_QBUF");
                }

                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
                        errno_exit ("VIDIOC_STREAMON");

                break;
        }
}

void uninit_device(int /*fd*/, IOMethod io)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free (buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap (buffers[i].start, buffers[i].length))
                                errno_exit ("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free (buffers[i].start);
                break;
        }

        free (buffers);
}

void init_read (int /*fd*/, unsigned int buffer_size)
{
        buffers = (buffer *)calloc (1, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc (buffer_size);

        if (!buffers[0].start) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

	printf("Allocated %zu buffer size\n", buffers[0].length);
} 

void init_mmap (int fd)
{
        struct v4l2_requestbuffers req;

        CLEAR (req);

        req.count               = 4;
        req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory              = V4L2_MEMORY_MMAP;

        if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "Device does not support memory mapping\n");
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) {
                fprintf (stderr, "Insufficient buffer memory on device\n");
                exit (EXIT_FAILURE);
        }

        buffers = (buffer *)calloc (req.count, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR (buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit ("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap (NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit ("mmap");
        }
}

void init_userp  (int fd, unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;
        unsigned int page_size;

        page_size = getpagesize ();
        buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

        CLEAR (req);

        req.count               = 4;
        req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory              = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "Device does not support user pointer i/o\n");
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_REQBUFS");
                }
        }

        buffers = (buffer *)calloc (4, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = memalign (/* boundary */ page_size,
                                                     buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf (stderr, "Out of memory\n");
                        exit (EXIT_FAILURE);
                }
        }
}

void enumerate_menu (int fd, struct v4l2_queryctrl &queryctrl)
{
	struct v4l2_querymenu querymenu;
	
	printf ("  Menu items:\n");

	memset (&querymenu, 0, sizeof (querymenu));
	querymenu.id = queryctrl.id;

	for (querymenu.index = queryctrl.minimum;
			(int)querymenu.index <= queryctrl.maximum;
			querymenu.index++) {
				if (0 == v4l2_ioctl (fd, VIDIOC_QUERYMENU, &querymenu)) {
					printf ("  %s\n", querymenu.name);
				} else {
					perror ("VIDIOC_QUERYMENU");
					exit (EXIT_FAILURE);
				}
			}
}

void print_control_value (int fd, int cid)
{
	struct v4l2_control control;

	memset (&control, 0, sizeof (control));
	control.id = cid;

	if (0 == v4l2_ioctl (fd, VIDIOC_G_CTRL, &control)) {
	
		printf("%d",control.value);
	}
	else
		printf("?");
}

/* Control ID   (defined in videodev2.h)

#define V4L2_CID_USER_CLASS
#define V4L2_CID_BRIGHTNESS
#define V4L2_CID_CONTRAST
#define V4L2_CID_SATURATION
#define V4L2_CID_HUE
#define V4L2_CID_AUTO_WHITE_BALANCE
#define V4L2_CID_DO_WHITE_BALANCE
#define V4L2_CID_RED_BALANCE
#define V4L2_CID_BLUE_BALANCE
#define V4L2_CID_GAMMA
#define V4L2_CID_EXPOSURE
#define V4L2_CID_AUTOGAIN
#define V4L2_CID_GAIN
#define V4L2_CID_HFLIP
#define V4L2_CID_VFLIP
*/
void set_control_value (int fd, int cid, int value)
{
	struct v4l2_control control;

	memset (&control, 0, sizeof (control));
	control.id = cid; control.value=value;

	if (0 == v4l2_ioctl (fd, VIDIOC_S_CTRL, &control)) {
	
		//printf("%d",control.value);
	}
	else {
		// unsupported no error
		// printf("!!! VIDIOC_S_CTRL Error !!!");
	}
}


void print_controls(int fd)
{
	struct v4l2_queryctrl queryctrl;

	memset (&queryctrl, 0, sizeof (queryctrl));
	
	for (queryctrl.id = V4L2_CID_BASE;	queryctrl.id < V4L2_CID_LASTP1;	queryctrl.id++) {
		 if (0 == v4l2_ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
			 if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				 continue;

			 printf ("Control %s: ", queryctrl.name);

			 if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
				 enumerate_menu (fd,queryctrl);
			 else if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER || 
					  queryctrl.type == V4L2_CTRL_TYPE_BOOLEAN) {
				 print_control_value (fd,queryctrl.id);
				 printf(" (%d:%d)",queryctrl.minimum,queryctrl.maximum);
			 }
			 else
				 printf("undef_type");
			 printf("\n");
		 } else {
			 if (errno == EINVAL)
				 continue;

			 perror ("VIDIOC_QUERYCTRL");
			 exit (EXIT_FAILURE);
		 }
	 }

	for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;	 queryctrl.id++) {
				 if (0 == v4l2_ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
					 if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
						 continue;

					 printf ("Control %s\n", queryctrl.name);

					 if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
						 enumerate_menu (fd,queryctrl);
				 } else {
					 if (errno == EINVAL)
						 break;

					 perror ("VIDIOC_QUERYCTRL");
					 exit (EXIT_FAILURE);
				 }
	}
}

bool init_device (int fd, unsigned int desired_format, int width, int height, int fps, IOMethod io, bool do_setting)
{	
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;

	if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
			if (EINVAL == errno) {
					fprintf (stderr, "Device is not a V4L2 device\n");
					exit (EXIT_FAILURE);
			} else {
					errno_exit ("VIDIOC_QUERYCAP");
			}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
			fprintf (stderr, "Device is no video capture device\n");
			exit (EXIT_FAILURE);
	}

	switch (io) {
	case IO_METHOD_READ:
			if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
					fprintf (stderr, "Device does not support read i/o\n");
					exit (EXIT_FAILURE);
			}

			break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
			if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
					fprintf (stderr, "Device does not support streaming i/o\n");
					exit (EXIT_FAILURE);
			}

			break;
	}

	printf("Device %s found\n", cap.card);
	
	struct v4l2_fmtdesc fmtdesc;
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmtdesc.index = 0;

	printf("Format allowed:\n");
	while (0 == xioctl (fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
		printf("   - %s\n",fmtdesc.description);
		fmtdesc.index++;
	}

	/* Select video input, video standard and tune here. */


	CLEAR (cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
			crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			crop.c = cropcap.defrect; /* reset to default */

			if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
					switch (errno) {
					case EINVAL:
						printf("VIDIOC_S_CROP: Image Cropping not supported.\n");
							break;
					default:
							/* Errors ignored. */
							break;
					}
			}
	} else {
			/* Errors ignored. */
	}

	CLEAR (fmt);
	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl (fd, VIDIOC_G_FMT, &fmt))
		errno_exit ("VIDIOC_G_FMT");

	char *aa = (char *)&fmt.fmt.pix.pixelformat;
	printf("Format (pre) :\n   type: %d\n   res: %dx%d\n   format: %c%c%c%c\n   field: %d\n"
			"   bpl: %d\n   size: %d\n   priv: %d\n",
	fmt.type,fmt.fmt.pix.width,fmt.fmt.pix.height,aa[0],aa[1],aa[2],aa[3],fmt.fmt.pix.field,
	fmt.fmt.pix.bytesperline,   fmt.fmt.pix.sizeimage,  fmt.fmt.pix.priv);


	if (do_setting) {
		// GET video device information
		v4l2_std_id esid0;
		int test;
		test = v4l2_ioctl( fd, VIDIOC_G_STD, &esid0 );
		if (test != 0)
		{
			printf("!!! VIDIOC_G_STD Error !!!\n");
		}

	    // SET video device STANDARD
		if (width==640)
	    	esid0 = 0x08000000UL; /*VGA*/
		else if (width==320)
	    	esid0 = 0x04000000UL; /*QVGA*/
	    test = v4l2_ioctl(fd, VIDIOC_S_STD, &esid0 );
	    if (test != 0)
	    {
			printf("!!! VIDIOC_S_STD Error !!!\n");
		}

		fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == xioctl (fd, VIDIOC_G_FMT, &fmt))
			errno_exit ("VIDIOC_G_FMT");
	
	    int bpp=3;
	    if (desired_format==V4L2_PIX_FMT_YUYV) bpp=2;
  
		printf("bpp=%d\n",bpp);
	    // CLEAR (fmt);

	    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    fmt.fmt.pix.width       = width; 
	    fmt.fmt.pix.height      = height;
	    fmt.fmt.pix.pixelformat = desired_format; // V4L2_PIX_FMT_YUYV;

	    fmt.fmt.pix.field        = V4L2_FIELD_NONE; // ANY;
	    fmt.fmt.pix.bytesperline = fmt.fmt.pix.width * bpp;
	    fmt.fmt.pix.sizeimage    = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	    fmt.fmt.pix.priv         = 0;

	    if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
		    errno_exit ("VIDIOC_S_FMT");

	    /* Note VIDIOC_S_FMT may change width and height. */

	    if (-1 == xioctl (fd, VIDIOC_G_FMT, &fmt))
		      errno_exit ("VIDIOC_G_FMT");
    
	    aa = (char *)&fmt.fmt.pix.pixelformat;
	    printf("Format (post):\n   type: %d\n   res: %dx%d\n   format: %c%c%c%c\n   field: %d\n   bpl: %d\n"
		"   size: %d\n   priv: %d\n",
	    fmt.type,fmt.fmt.pix.width,fmt.fmt.pix.height,aa[0],aa[1],aa[2],aa[3],fmt.fmt.pix.field,
	    fmt.fmt.pix.bytesperline,   fmt.fmt.pix.sizeimage,  fmt.fmt.pix.priv);

	    if (fmt.fmt.pix.pixelformat != desired_format) {
			aa = (char *)&desired_format;
			printf("%c%c%c%c ",aa[0],aa[1],aa[2],aa[3]); fflush(stdout);
			errno_exit ("Unsupported format!\n");
	    }


	    /* Buggy driver paranoia. */
	    min = fmt.fmt.pix.width * 2;
	    if (fmt.fmt.pix.bytesperline < min)
		    fmt.fmt.pix.bytesperline = min;
	    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	    if (fmt.fmt.pix.sizeimage < min)
		    fmt.fmt.pix.sizeimage = min;

	    //set fps
		struct v4l2_streamparm parm;
		
		parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		test = v4l2_ioctl(fd, VIDIOC_G_PARM, &parm);
		if (test != 0)
		{
			printf("!!! VIDIOC_G_PARM Error !!!\n");
		}
		
		parm.parm.capture.timeperframe.numerator = 1;
		parm.parm.capture.timeperframe.denominator = fps;
		parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		v4l2_ioctl(fd, VIDIOC_S_PARM, &parm);

	}

	char *a = (char *) &fmt.fmt.pix.pixelformat;
	printf("Device %s  %c%c%c%c %dx%d %d opened\n", cap.card,
	a[0],a[1],a[2],a[3],fmt.fmt.pix.width,fmt.fmt.pix.height,fmt.fmt.pix.sizeimage);

	switch (io) {
	case IO_METHOD_READ:
			init_read (fd,fmt.fmt.pix.sizeimage);
			break;

	case IO_METHOD_MMAP:
			init_mmap (fd);
			break;

	case IO_METHOD_USERPTR:
			init_userp (fd,fmt.fmt.pix.sizeimage);
			break;
	}

	
	set_control_value (fd, V4L2_CID_AUTOGAIN, 0);
	set_control_value (fd, V4L2_CID_GAIN, 10);

	print_controls(fd);

	printf("init_device done.\n");
	
	return true;
}

void close_device (int fd)
{
        if (-1 == close (fd))
                errno_exit ("close");

}

int open_device (const char *dev_name)
{
	int fd;
	struct stat st; 

        if (-1 == stat (dev_name, &st)) {
                fprintf (stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                return -1;
        }

        if (!S_ISCHR (st.st_mode)) {
                fprintf (stderr, "%s is no device\n", dev_name);
                return -1;
        }

        fd =v4l2_open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf (stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                return -1;
        }

	return fd;
}




