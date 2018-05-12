/*
 *  V4L2 video capture example (extended with libv4l2 function calls)
 *
 *  This program can be used and distributed without restrictions.
 */

#include "v4l_utils.h"

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

#include <rdkcore/config.h>

#ifdef libv4l2_FOUND
#include <libv4l2.h>
#endif

#define CLEAR(x) memset (&(x), 0, sizeof (x))


struct buffer {
	void * start;
	size_t length;
};

static struct buffer *buffers = NULL;
//static unsigned int n_buffers = 0; //unused
static struct v4l2_buffer buf;

void v4lu_errno_exit(const char *s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror (errno));
	exit(EXIT_FAILURE);
}

int v4lu_xioctl(int fd, int request, void *arg, bool uselibv4l2)
{
	uselibv4l2 = true;
	int r = -1;
	if (uselibv4l2) {
#ifdef libv4l2_FOUND
		do r = v4l2_ioctl(fd, request, arg);
		while (-1 == r && EINTR == errno);
#else
		return -1;
#endif
	}
	else {
		do r = ioctl(fd, request, arg);
		while (-1 == r && EINTR == errno);
	}
	return r;
}

int v4lu_read_frame(int fd, IOMethod io, bool uselibv4l2)
{
	//unsigned int i; //unused

	switch (io) {
		case IO_METHOD_READ:
			{
				int rr = -1;
				if (uselibv4l2) {
#ifdef libv4l2_FOUND
					rr = v4l2_read(fd, buffers[0].start, buffers[0].length);
#endif
				}
				else rr = read(fd, buffers[0].start, buffers[0].length);

				if (-1 == rr) {
					switch (errno) {
						case EAGAIN: return 0;
						case EIO: break; /* Could ignore EIO, see spec. */
						default: v4lu_errno_exit("read_frame");
					}
				}
			} break;

		case IO_METHOD_MMAP:
		case IO_METHOD_USERPTR:
			{
				fprintf(stderr,"Unhandled value in switch");
			}
			break;
#if 0
		case IO_METHOD_MMAP:
			CLEAR (buf);

			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;

			if (-1 == v4lu_xioctl(fd, VIDIOC_DQBUF, &buf, uselibv4l2)) {
				switch (errno) {
					case EAGAIN:
						return 0;

					case EIO:
						/* Could ignore EIO, see spec. */

						/* fall through */

					default:
						v4lu_errno_exit ("VIDIOC_DQBUF");
				}
			}

			assert(buf.index < n_buffers);

			if (-1 == v4lu_xioctl(fd, VIDIOC_QBUF, &buf, uselibv4l2))
				v4lu_errno_exit ("VIDIOC_QBUF");

			break;

		case IO_METHOD_USERPTR:
			CLEAR (buf);

			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_USERPTR;

			if (-1 == v4lu_xioctl(fd, VIDIOC_DQBUF, &buf, uselibv4l2)) {
				switch (errno) {
					case EAGAIN:
						return 0;

					case EIO:
						/* Could ignore EIO, see spec. */

						/* fall through */

					default:
						v4lu_errno_exit ("VIDIOC_DQBUF");
				}
			}

			for (i = 0; i < n_buffers; ++i)
				if (buf.m.userptr == (unsigned long) buffers[i].start
						&& buf.length == buffers[i].length)
					break;

			assert (i < n_buffers);

			if (-1 == v4lu_xioctl (fd, VIDIOC_QBUF, &buf, uselibv4l2))
				v4lu_errno_exit ("VIDIOC_QBUF");

			break;
#endif
	}

	return 1;
}

void * v4lu_get_buffer(IOMethod io)
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


bool v4lu_get_frame(int fd, IOMethod io, bool uselibv4l2)
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

			v4lu_errno_exit ("select");
		}

		if (0 == r) {
			//fprintf (stderr, "select timeout\n");
			// return false;
		}

		if (v4lu_read_frame(fd, io, uselibv4l2)) break;

		/* EAGAIN - continue select loop. */
	} // for

	return true;
}

void v4lu_stop_capturing(int /*fd*/, IOMethod io)
{
	// enum v4l2_buf_type type; //unused

	switch (io) {
		case IO_METHOD_READ:
			/* Nothing to do. */
			break;

		case IO_METHOD_MMAP:
		case IO_METHOD_USERPTR:
			{
				fprintf(stderr,"Unhandled value in switch");
			}
			break;

#if 0
		case IO_METHOD_MMAP:
		case IO_METHOD_USERPTR:
			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

			if (-1 == v4lu_xioctl(fd, VIDIOC_STREAMOFF, &type, false))
				v4lu_errno_exit ("VIDIOC_STREAMOFF");

			break;
#endif
	}
}

void v4lu_start_capturing(int /*fd*/, IOMethod io)
{
	//unsigned int i; //unused
	//enum v4l2_buf_type type; //unused

	switch (io) {
		case IO_METHOD_READ:
			/* Nothing to do. */
			break;

		case IO_METHOD_MMAP:
		case IO_METHOD_USERPTR:
			{
				fprintf(stderr,"Unhandled value in switch");
			}
			break;
#if 0
		case IO_METHOD_MMAP:
			for (i = 0; i < n_buffers; ++i) {
				struct v4l2_buffer buf;

				CLEAR (buf);

				buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf.memory      = V4L2_MEMORY_MMAP;
				buf.index       = i;

				if (-1 == v4lu_xioctl(fd, VIDIOC_QBUF, &buf, false))
					v4lu_errno_exit ("VIDIOC_QBUF");
			}

			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

			if (-1 == v4lu_xioctl(fd, VIDIOC_STREAMON, &type, false))
				v4lu_errno_exit ("VIDIOC_STREAMON");

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

				if (-1 == v4lu_xioctl(fd, VIDIOC_QBUF, &buf, false))
					v4lu_errno_exit ("VIDIOC_QBUF");
			}

			type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

			if (-1 == v4lu_xioctl(fd, VIDIOC_STREAMON, &type, false))
				v4lu_errno_exit ("VIDIOC_STREAMON");

			break;
#endif
	}
}

void v4lu_uninit_device(int /*fd*/, IOMethod io)
{
	//unsigned int i; //unused

	switch (io) {
		case IO_METHOD_READ:
			free (buffers[0].start);
			break;
		case IO_METHOD_MMAP:
		case IO_METHOD_USERPTR:
			{
				fprintf(stderr,"Unhandled value in switch");
			}
			break;
#if 0
		case IO_METHOD_MMAP:
			for (i = 0; i < n_buffers; ++i)
				if (-1 == munmap (buffers[i].start, buffers[i].length))
					v4lu_errno_exit ("munmap");
			break;

		case IO_METHOD_USERPTR:
			for (i = 0; i < n_buffers; ++i)
				free (buffers[i].start);
			break;
#endif
	}

	free (buffers);
}

void v4lu_init_read(int /*fd*/, unsigned int buffer_size)
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

void v4lu_init_mmap(int /*fd*/)
{
#if 0
	struct v4l2_requestbuffers req;

	CLEAR (req);

	req.count               = 1;	// FIXME
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_MMAP;

	if (-1 == v4lu_xioctl(fd, VIDIOC_REQBUFS, &req, false)) {
		if (EINVAL == errno) {
			fprintf (stderr, "Device does not support memory mapping\n");
			exit (EXIT_FAILURE);
		} else {
			v4lu_errno_exit ("VIDIOC_REQBUFS");
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

		if (-1 == v4lu_xioctl(fd, VIDIOC_QUERYBUF, &buf, false))
			v4lu_errno_exit ("VIDIOC_QUERYBUF");

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start =
			mmap (NULL /* start anywhere */,
					buf.length,
					PROT_READ | PROT_WRITE /* required */,
					MAP_SHARED /* recommended */,
					fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start)
			v4lu_errno_exit ("mmap");
	}
#endif
}

void v4lu_init_userp(int /*fd*/, unsigned int /*buffer_size*/)
{
#if 0
	struct v4l2_requestbuffers req;
	unsigned int page_size;

	page_size = getpagesize ();
	buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

	CLEAR (req);

	req.count               = 1;	// FIXME
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_USERPTR;

	if (-1 == v4lu_xioctl(fd, VIDIOC_REQBUFS, &req, false)) {
		if (EINVAL == errno) {
			fprintf (stderr, "Device does not support user pointer i/o\n");
			exit (EXIT_FAILURE);
		} else {
			v4lu_errno_exit ("VIDIOC_REQBUFS");
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
#endif
}

void v4lu_enumerate_menu(int fd, struct v4l2_queryctrl &queryctrl)
{
	struct v4l2_querymenu querymenu;

	printf ("  Menu items:\n");

	memset (&querymenu, 0, sizeof (querymenu));
	querymenu.id = queryctrl.id;

	for (querymenu.index = queryctrl.minimum;
			(int)querymenu.index <= queryctrl.maximum;
			querymenu.index++) {
		if (0 == v4lu_xioctl (fd, VIDIOC_QUERYMENU, &querymenu, false)) {
			printf ("  %s\n", querymenu.name);
		} else {
			perror ("VIDIOC_QUERYMENU");
			exit (EXIT_FAILURE);
		}
	}
}

void v4lu_print_control_value(int fd, int cid)
{
	struct v4l2_control control;

	memset (&control, 0, sizeof (control));
	control.id = cid;

	if (0 == v4lu_xioctl (fd, VIDIOC_G_CTRL, &control, false)) {

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
void v4lu_set_control_value(int fd, int cid, int value)
{
	struct v4l2_control control;

	memset (&control, 0, sizeof (control));
	control.id = cid; control.value=value;

	if (0 == v4lu_xioctl (fd, VIDIOC_S_CTRL, &control, false)) {

		//printf("%d",control.value);
	}
	else {
		// unsupported no error
		// printf("!!! VIDIOC_S_CTRL Error !!!");
	}
}


void v4lu_print_controls(int fd)
{
	struct v4l2_queryctrl queryctrl;

	memset (&queryctrl, 0, sizeof (queryctrl));

	for (queryctrl.id = V4L2_CID_BASE;	queryctrl.id < V4L2_CID_LASTP1;	queryctrl.id++) {
		if (0 == v4lu_xioctl (fd, VIDIOC_QUERYCTRL, &queryctrl, false)) {
			if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;

			printf ("Control %s: ", queryctrl.name);

			if (queryctrl.type == V4L2_CTRL_TYPE_MENU) ;
			//enumerate_menu (fd,queryctrl);
			else if (queryctrl.type == V4L2_CTRL_TYPE_INTEGER || 
					queryctrl.type == V4L2_CTRL_TYPE_BOOLEAN) {
				//print_control_value (fd,queryctrl.id); 
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
		if (0 == v4lu_xioctl (fd, VIDIOC_QUERYCTRL, &queryctrl, false)) {
			if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;

			printf ("Control %s\n", queryctrl.name);

			if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
			}
			//enumerate_menu (fd,queryctrl);
		} else {
			if (errno == EINVAL)
				break;

			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		}
	}
}

bool v4lu_init_device(int fd, unsigned int desired_format, int width, int height, int fps, IOMethod io, bool do_setting, bool uselibv4l2)
{	
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	//struct v4l2_crop crop; //unused
	struct v4l2_format fmt;
	unsigned int min;

	if (-1 == v4lu_xioctl(fd, VIDIOC_QUERYCAP, &cap, uselibv4l2)) {
		if (EINVAL == errno) {
			fprintf (stderr, "Device is not a V4L2 device\n");
			exit (EXIT_FAILURE);
		} else {
			v4lu_errno_exit ("VIDIOC_QUERYCAP");
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
	while (0 == v4lu_xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc, uselibv4l2)) {
		printf("   - %s\n",fmtdesc.description);
		fmtdesc.index++;
	}

	/* Select video input, video standard and tune here. */


	CLEAR (cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#if 0
	if (0 == v4lu_xioctl(fd, VIDIOC_CROPCAP, &cropcap, uselibv4l2)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == v4lu_xioctl(fd, VIDIOC_S_CROP, &crop, uselibv4l2)) {
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
#endif
	CLEAR (fmt);

	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == v4lu_xioctl(fd, VIDIOC_G_FMT, &fmt, uselibv4l2))
		v4lu_errno_exit ("VIDIOC_G_FMT");

	char *aa = (char *)&fmt.fmt.pix.pixelformat;
	printf("Format (pre) :\n   type: %d\n   res: %dx%d\n   format: %c%c%c%c\n   field: %d\n"
			"   bpl: %d\n   size: %d\n   priv: %d\n",
			fmt.type,fmt.fmt.pix.width,fmt.fmt.pix.height,aa[0],aa[1],aa[2],aa[3],fmt.fmt.pix.field,
			fmt.fmt.pix.bytesperline,   fmt.fmt.pix.sizeimage,  fmt.fmt.pix.priv);

	if (do_setting) {
		// GET video device information
		//v4l2_std_id esid0; //unused
		//int test; // unused
#if 0
		test = v4lu_xioctl( fd, VIDIOC_G_STD, &esid0, uselibv4l2);
		if (test != 0)
		{
			printf("!!! VIDIOC_G_STD Error !!!\n");
		}

		// SET video device STANDARD
		if (width==640)
			esid0 = 0x08000000UL; /*VGA*/
		else if (width==320)
			esid0 = 0x04000000UL; /*QVGA*/
		test = v4lu_xioctl(fd, VIDIOC_S_STD, &esid0,uselibv4l2 );
		if (test != 0)
		{
			printf("!!! VIDIOC_S_STD Error !!!\n");
		}

		fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == v4lu_xioctl(fd, VIDIOC_G_FMT, &fmt, uselibv4l2))
			v4lu_errno_exit ("VIDIOC_G_FMT");
#endif
		int bpp = 0;
		switch (desired_format) {
			case V4L2_PIX_FMT_YUYV: bpp = 2; break;
			case V4L2_PIX_FMT_RGB24: bpp = 3; break;
			case V4L2_PIX_FMT_RGB32: bpp = 4; break;
			default: printf("Unsupported format %d\n", desired_format); return false;
		}

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

		if (-1 == v4lu_xioctl(fd, VIDIOC_S_FMT, &fmt, uselibv4l2))
			v4lu_errno_exit ("VIDIOC_S_FMT");

		/* Note VIDIOC_S_FMT may change width and height. */

		if (-1 == v4lu_xioctl(fd, VIDIOC_G_FMT, &fmt, uselibv4l2))
			v4lu_errno_exit ("VIDIOC_G_FMT");

		aa = (char *)&fmt.fmt.pix.pixelformat;
		printf("Format (post):\n   type: %d\n   res: %dx%d\n   format: %c%c%c%c\n   field: %d\n   bpl: %d\n"
				"   size: %d\n   priv: %d\n",
				fmt.type,fmt.fmt.pix.width,fmt.fmt.pix.height,aa[0],aa[1],aa[2],aa[3],fmt.fmt.pix.field,
				fmt.fmt.pix.bytesperline,   fmt.fmt.pix.sizeimage,  fmt.fmt.pix.priv);

		if (fmt.fmt.pix.pixelformat != desired_format) {
			aa = (char *)&desired_format;
			printf("%c%c%c%c ",aa[0],aa[1],aa[2],aa[3]); fflush(stdout);
			v4lu_errno_exit ("Unsupported format!\n");
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
		int test2 = v4lu_xioctl(fd, VIDIOC_G_PARM, &parm, uselibv4l2);
		if (test2 != 0)
		{
			printf("!!! VIDIOC_G_PARM Error !!!\n");
		}

		parm.parm.capture.timeperframe.numerator = 1;
		parm.parm.capture.timeperframe.denominator = fps;
		parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		v4lu_xioctl(fd, VIDIOC_S_PARM, &parm, uselibv4l2);

	}

	char *a = (char *) &fmt.fmt.pix.pixelformat;
	printf("Device %s  %c%c%c%c %dx%d %d opened\n", cap.card,
			a[0],a[1],a[2],a[3],fmt.fmt.pix.width,fmt.fmt.pix.height,fmt.fmt.pix.sizeimage);

	switch (io) {
		case IO_METHOD_READ:
			v4lu_init_read(fd, fmt.fmt.pix.sizeimage);
			break;

		case IO_METHOD_MMAP:
			v4lu_init_mmap (fd);
			break;

		case IO_METHOD_USERPTR:
			v4lu_init_userp(fd, fmt.fmt.pix.sizeimage);
			break;
	}


	v4lu_set_control_value(fd, V4L2_CID_AUTOGAIN, 0);
	v4lu_set_control_value(fd, V4L2_CID_GAIN, 10);

	v4lu_print_controls(fd);

	printf("init_device done.\n");

	return true;
}

void v4lu_close_device(int fd)
{
	if (-1 == close (fd))
		v4lu_errno_exit ("close");

}

int v4lu_open_device(const char *dev_name, bool uselibv4l2)
{
	int fd = -1;
	struct stat st; 

	if (-1 == stat(dev_name, &st)) {
		fprintf (stderr, "Cannot identify '%s': %d, %s\n",
				dev_name, errno, strerror (errno));
		return -1;
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf (stderr, "%s is no device\n", dev_name);
		return -1;
	}

	if (uselibv4l2) {
#ifdef libv4l2_FOUND
		fd = v4l2_open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
#else
		printf("You asked the libv4l2 support, but libv4l2 has not been found in your system\n");
		return -1;
#endif
	}
	else {
		fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
	}

	if (-1 == fd) {
		fprintf (stderr, "Cannot open '%s': %d, %s\n",
				dev_name, errno, strerror (errno));
		return -1;
	}

	return fd;
}
