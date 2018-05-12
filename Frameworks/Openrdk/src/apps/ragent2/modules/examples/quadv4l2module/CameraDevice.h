#ifndef __CAMERADEVICE_H__
#define __CAMERADEVICE_H__

#include <stdio.h>
#include <stdlib.h>
#include <linux/videodev2.h>
#include <jpeglib.h>

struct Image {
  uint type, width, height, bpp, compressed;
  char fourcc[5];
  unsigned char* data;
  unsigned long timestamp;
  
  Image() { type = V4L2_PIX_FMT_RGB24; width=0; height=0; bpp=3; data=NULL; timestamp=0; setFourCC(); }
  ~Image() { if (data) free(data); data=NULL; } 
  void alloc_mem(int typ, int w, int h);
  void free_mem() { if (data) free(data); data=NULL; }
  void savePPM(const char *filename);
  void saveJPEG(const char *filename);
  bool loadPPM(const char *filename);
  int read(FILE *f) { return fread(data,width*height*bpp,1,f); }

  void setFourCC() {
      fourcc[0]= (char)(type & 0x000000FF);
      fourcc[1]= (char)((type & 0x0000FF00)>>8);
      fourcc[2]= (char)((type & 0x00FF0000)>>16);
      fourcc[3]= (char)((type & 0xFF000000)>>24);
      fourcc[4]='\0';
  }

  // Image convertTo(uint format);
  void setAndConvertData(unsigned char *data, uint format);
};

class CameraDevice {
public:
	char cameraid[33];
	Image image;

	CameraDevice() { }
	CameraDevice(int w, int h) { image.alloc_mem(V4L2_PIX_FMT_RGB24,w,h); }
	CameraDevice(int type, int w, int h) { image.alloc_mem(type,w,h); }
	virtual ~CameraDevice() { }
	virtual void init() { }
	virtual bool ready() = 0;
	virtual bool getFrame() = 0;
	virtual void done() { }
};

unsigned long getTimeStamp();

#endif
