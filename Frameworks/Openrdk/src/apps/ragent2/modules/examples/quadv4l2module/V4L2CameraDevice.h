/**
 * @file
 *
 * @brief This file contains the class V4LCameraDevice
 */

#ifndef __V4L2CAMERADEVICE_H__
#define __V4L2CAMERADEVICE_H__

#include <string>
#include <string.h>
#include <linux/videodev2.h> // Video4Linux vers. 2
#include "CameraDevice.h"

#define DEFAULT_WIDTH   320
#define DEFAULT_HEIGHT  240
#define DEFAULT_FPS     1

typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
} IOMethod;


/**
 * @brief Video4Linux camera
 *
 * This class represents a camera accessible through the Video For
 * Linux (V4L2) interface.
 *
 */
class V4L2CameraDevice : public CameraDevice {
private:
    /// True if we did init() succesfully
    bool deviceReady;
    /// File descriptor of opened device
    int fd;	
    /// Resolution
    int width, height;
    /// Device name
    std::string deviceName;
    /// IO Method (IO_METHOD_MMAP, IO_METHOD_READ)
    IOMethod        io_method; // IO_METHOD_MMAP, IO_METHOD_READ;

public:

    /// Every class should have a zero-arguments constructor
    V4L2CameraDevice() : deviceReady(false), deviceName("/dev/video0"){ }
    /// Please use this constructor
    V4L2CameraDevice(const char *deviceName) : deviceReady(false), 
					      deviceName(deviceName) { }
    virtual ~V4L2CameraDevice();
   
    void convertFromYCbCrToRGB(unsigned char Y,
                                    unsigned char Cb,
                                    unsigned char Cr,
                                    unsigned char& R,
                                    unsigned char& G,
                                    unsigned char& B);

    void convertFromYUV422toRGB(unsigned char * image, unsigned char * buffRGB, int width, int height);
 
    /**
     * @name initialization routines. 
     *
     * Set the image format, size and IO method.
     */
    //@{
    void init(uint type, int width, int height, int fps, IOMethod iom=IO_METHOD_MMAP, bool do_setting=true);
    void init() { init(V4L2_PIX_FMT_RGB24,DEFAULT_WIDTH,DEFAULT_HEIGHT,DEFAULT_FPS,IO_METHOD_MMAP,true); }
    //@}
    /**
     * @brief camera is ready for grabbing
     *
     * @returns true if ready to grab
     */
    bool ready();
    /**
     * @brief grab a frame and put it into image
     *
     * @returns true if the frame has been acquired succesfully
     */
    bool getFrame();
    /// Close the device
    void done();
};

#endif
