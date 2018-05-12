#include <stdio.h>
#include <sys/timeb.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>

#include "CameraDevice.h"

// return timestamp in ms from midnight
unsigned long getTimeStamp()
{
#ifdef LINUX
	struct timeval tv;
	gettimeofday(&tv,0);
	return (unsigned long)(tv.tv_sec%86400)*1000+(unsigned long)tv.tv_usec/1000;
#else
	struct timeb timebuffer;
	ftime( &timebuffer );
	unsigned short  millitm = timebuffer.millitm;
	time_t sectm = timebuffer.time;
	return (unsigned long)(sectm%86400)*1000+(unsigned long)millitm;
#endif
}


void Image::alloc_mem(int typ, int w, int h) 
{ 
	type=typ; width=w; height=h; setFourCC(); bpp=3; compressed=0;
	switch(type) {
		case V4L2_PIX_FMT_BGR24: bpp=3; break;
		case V4L2_PIX_FMT_RGB24: bpp=3; break;
		case V4L2_PIX_FMT_BGR32: bpp=4; break;
		case V4L2_PIX_FMT_RGB32: bpp=4; break;
		case V4L2_PIX_FMT_GREY : bpp=1; break;
		case V4L2_PIX_FMT_YUYV : bpp=2; break;
		case V4L2_PIX_FMT_UYVY : bpp=2; break;
														 /* compressed formats */
		case V4L2_PIX_FMT_MJPEG : compressed=1; break;
		case V4L2_PIX_FMT_JPEG  : compressed=1; break;
		case V4L2_PIX_FMT_DV    : compressed=1; break;
		case V4L2_PIX_FMT_MPEG  : compressed=1; break;
		default: fprintf(stderr,"Camera init: Unknown format type!\n");
	}

	data=(unsigned char*)malloc(width*height*bpp);
}

void Image::savePPM(const char *filename)
{
	FILE *fp;
	size_t size;

	if (filename == NULL) return;

	if ((fp = fopen(filename, "wb")) == NULL) {
		fprintf(stderr, "Error writing the file %s in RGBImage::savePPM() \n",
				filename);
		return;
	}

	// Write the header information to the PGM file.
	fprintf(fp, "P6\n%d %d\n", width, height);
	//if(comment != NULL)
	//	if(strlen(comment) <= 70) fprintf(fp, "# %s\n", comment);
	fprintf(fp, "%d\n", 255);

	size = (size_t)width * height * bpp;

	// PPM has maximum depth 24bits. If we have 32 bit-deep images, we
	// need to convert them
	if (bpp == 4) {
		printf("Converting...");
		unsigned char *tempData = (unsigned char *)malloc(width * height * 3);
		size_t i, j;
		for ( i = 0, j = 0;
				i < size; 
				i += 4, j += 3) {
			tempData[j] = data[i];
			tempData[j + 1] = data[i + 1];
			tempData[j + 2] = data[i + 2];
		}
		free(data);
		data = tempData;
		size = (size_t)width * height * 3;
	}

	// Write the image data to the file.
	fwrite(data,size,1,fp);

	fclose(fp);
	return;
}

/////////////////AGGIUNTO DA NOI

void Image::saveJPEG(const char *filename)
{
	FILE *outfile;
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	int row_stride;              // physical row width in image buffer
	JSAMPROW row_pointer[1];     // pointer to JSAMPLE row(s)
	int jsmooth=0,jopt=0,jquality=60;
	//unsigned char pic[MAX_RGBA_IMAGE_SIZE];

	if (!(outfile=fopen(filename,"w"))) {
		printf("Errore nell'apertura del file");

	};
	//printf("Prima della compressione\n");
	cinfo.err = jpeg_std_error(&jerr);

	jpeg_create_compress(&cinfo);
	//printf("Dopo della compressione\n");  

	jpeg_stdio_dest(&cinfo, outfile);
	cinfo.image_width = width;  // image width and height, in pixels
	cinfo.image_height = height;
	cinfo.input_components = bpp;         // # of color components per pixel
	cinfo.in_color_space = JCS_RGB;     // colorspace of input image

	cinfo.smoothing_factor = jsmooth;
	cinfo.optimize_coding = jopt;

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo,jquality,TRUE);
	jpeg_start_compress(&cinfo, TRUE);
	row_stride=width*3;
	while (cinfo.next_scanline < cinfo.image_height) {
		/* jpeg_write_scanlines expects an array of pointers to scanlines.
		 * Here the array is only one element long, but you could pass
		 * more than one scanline at a time if that's more convenient.
		 */
		row_pointer[0] = &data[cinfo.next_scanline * row_stride];
		jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}
	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
	fclose(outfile);
}

////////////////FINE

bool Image::loadPPM(const char *filename)
{
	FILE *fp;
	char buf[180];
	int size;

	if (filename == NULL)
		return false;

	if((fp = fopen(filename, "rb")) == NULL){
		fprintf(stderr, "Error reading the file %s in read_ppm_image().\n",
				filename);
		return false;
	}

	char * ret = fgets(buf, 180, fp);
	if (ret != buf)
	{
		fprintf(stderr,"Somethign bad happens here in CameraDevice.cpp!!!");
	}
	if(strncmp(buf,"P6",2) != 0){
		fprintf(stderr, "The file %s is not in PPM format in ", filename);
		fprintf(stderr, "read_ppm_image().\n");
		if(fp != stdin) fclose(fp);
		return false;
	}

	do{
		ret = fgets(buf, 70, fp);
		if (ret != buf)
		{
			fprintf(stderr,"Somethign bad happens here in CameraDevice.cpp!!!");
		}
	}while(buf[0] == '#');  /* skip all comment lines */
	sscanf(buf, "%d %d", &width, &height);
	do{
		ret = fgets(buf, 70, fp);
		if (ret != buf)
		{
			fprintf(stderr,"Somethign bad happens here in CameraDevice.cpp!!!");
		}
	}while(buf[0] == '#');  /* skip all comment lines */

	if (data)
		free_mem();
	alloc_mem(V4L2_PIX_FMT_RGB24,width,height);

	size = width*height; int r;
	if ((r=fread(data,3,size,fp))<size) {
		fprintf(stderr, "Warning: RGBImage::loadPPM read only %d/%d from file %s.\n",r,size,filename);
	}

	fclose(fp);
	return true;
}


/** Converts an YCbCr pixel into an RGB pixel.
 *  @param Y The Y channel of the source pixel.
 *  @param Cb The Cb channel of the source pixel.
 *  @param Cr The Cr channel of the source pixel.
 *  @param R The R channel of the target pixel.
 *  @param G The G channel of the target pixel.
 *  @param B The B channel of the target pixel.
 */
void convertFromYCbCrToRGB(unsigned char Y,
		unsigned char Cb,
		unsigned char Cr,
		unsigned char& R,
		unsigned char& G,
		unsigned char& B)
{
	int r = (int)(Y + 1.4021 * (Cb - 128)),
	g = (int)(Y - 0.3456 * (Cr - 128) - 0.71448 * (Cb - 128)),
	b = (int)(Y + 1.7710 * (Cr - 128));
	if(r < 0) r = 0; else if(r > 255) r = 255;
	if(g < 0) g = 0; else if(g > 255) g = 255;
	if(b < 0) b = 0; else if(b > 255) b = 255;
	R = (unsigned char) r;
	G = (unsigned char) g;
	B = (unsigned char) b;
}

void convertYUYVtoRGB24(unsigned char *out, unsigned char *in, int width, int height)
{
	unsigned char *p = in, *q = out;
	unsigned char r,g,b;
	unsigned char y1,y2,u,v;
	for(int y=0; y<height; y++)
		for(int x=0; x<width; x+=2)
		{
			y1 = *p; v = *(p+1); y2 = *(p+2); u = *(p+3);  p += 4;
			convertFromYCbCrToRGB(y1,u,v,b,g,r);
			*(q) = r; *(q+1) = g;	*(q+2) = b;
			//*(q) = *(q+1) =	*(q+2) = y1;
			q+=3;
			convertFromYCbCrToRGB(y2,u,v,b,g,r);
			*(q) = r; *(q+1) = g;	*(q+2) = b;
			//*(q) = *(q+1) =	*(q+2) = y2;
			q+=3;

		}
}

/*
	 Image& Image::convertTo(uint format)
	 {
	 Image *a = new Image();

	 if (type==V4L2_PIX_FMT_YUYV && format==V4L2_PIX_FMT_RGB24) 
	 {
	 a->alloc_mem(format, width, height);
	 convertYUYVtoRGB24(a->data, data, width, height);
	 }
	 else {
	 printf("Conversion not implemented!\n");
	 }

	 return *a;
	 }
	 */

void Image::setAndConvertData(unsigned char *buf, uint format)
{
	if (type==V4L2_PIX_FMT_RGB24 && format==V4L2_PIX_FMT_RGB24) 
	{    
		size_t imageSize = width * height * bpp;
		memcpy(data, buf, imageSize);
	}
	else if (type==V4L2_PIX_FMT_RGB24 && format==V4L2_PIX_FMT_YUYV) 
	{
		convertYUYVtoRGB24(data, buf, width, height);
	}
	else printf("Image::setAndConvertData - Unsupported operation!\n");
}


