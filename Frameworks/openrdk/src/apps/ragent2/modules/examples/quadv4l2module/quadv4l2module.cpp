#define MODULE_NAME "QuadV4L2Module"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/time/time.h>
#include <cstring>
#include <string>


#define LOGGING_MODULE MODULE_NAME

#include "quadv4l2module.h"
#include "quadv4l2module_names.h"
// it is better to declare some define to be used as property names, in order to avoid misspelling in strings,
// declare here those defines for all your properties and use these; for example:

#define DEFAULT_WIDTH   320
#define DEFAULT_HEIGHT  240
#define DEFAULT_FPS     1

namespace RDK2 { namespace RAgent {
	
	unsigned int width,height;
	int counter = 0;
	int savedPhotos = 0;
	bool QuadV4L2Module::initConfigurationProperties()
	{
		SESSION_TRY_START(session)
			// put false in the second parameter if you want the module disabled for default
			// add a third parameter valued false if you don't want the state property
			Common::createDefaultProperties(session, false, false);
			// here you declare the properties of your module
			// in the repository they have the name of this module as prefix, e.g. /yourModuleName/maxSpeed
			session->createString(PARAM_DEVICE, "Device name", "/dev/video0");
			session->createString(PARAM_FORMAT, "Image format (YUYV, RGB24)", "RGB24");
			session->createInt(PARAM_FPS, "Camera FPS", DEFAULT_FPS);
			session->createInt(PARAM_HEIGHT, "Camera HEIGHT", DEFAULT_HEIGHT);
			session->createInt(PARAM_WIDTH, "Camera WIDTH", DEFAULT_WIDTH);
			session->createString(PARAM_IOMETHOD, "IO Method (read (RGB24), mmap (YUYV))", "read");
			
			session->createBool(PARAM_SAVE, "Set true to enable the save", false);
			session->createInt(PARAM_PHOTOS_NUM, "Number of photos (put -1 for unlimited", 10);
			session->createString(PARAM_PREFIX, "Photo filename prefix", "photos/");
			session->createBool(PARAM_JPEG, "Set true to save on jpeg format", true);
			
			// session->createDouble(PROPERTY_MAX_SPEED, "Maximum speed", RDouble::MM_SEC, 100.);
			// // example of enum creation
			// ENUM_CREATE(whichStalled);
			// ENUM_ITEM(whichStalled, NONE, "None", "No wheel stalled");
			// ENUM_ITEM(whichStalled, LEFT, "Left", "Left wheels stalled");
			// ENUM_ITEM(whichStalled, RIGHT, "Right", "Right wheels stalled");
			// ENUM_ITEM(whichStalled, BOTH, "Both", "Full robot stall");
			// session->createEnum(PROPERTY_ROBOT_WHICH_STALLED, "Which robot wheel stalled", whichStalled, NONE, INFO);
			// // example of image creation
			session->createImage(PROPERTY_IMAGE, "Current Image", DEFAULT_WIDTH, DEFAULT_HEIGHT, RDK2::RGraphics::RImage::RGB24);
			session->setVolatile(PROPERTY_IMAGE);
			// // example of map creation
			// session->createMap(PROPERTY_MY_MAP, "My ugly map", x, y, theta, realWidth, bitmapWidth, bitmapHeight);
			// // example of vector creation
			// session->createVector<RType>(PROPERTY_MY_VECTOR, "My vector");
			
		SESSION_END(session)
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	// bool QuadV4L2Module::initInterfaceProperties() { }

	bool QuadV4L2Module::init()
	{
		// in the init() function you should initialize everything your module needs (drivers initialization,
		// sockets, and so on); all modules wait for all init() before they start
		SESSION_TRY_START(session)
			
			int fps = session->getInt(PARAM_FPS);
			double tim = 1000.0 / fps;
			session->listenToTimer(tim);
			
			
			camera = new V4L2CameraDevice(session->getString(PARAM_DEVICE).c_str());
			
			
			IOMethod iom = IO_METHOD_READ;
			if (session->getString(PARAM_IOMETHOD)=="mmap") iom = IO_METHOD_MMAP;
			
			int width = session->getInt(PARAM_WIDTH);
			int height = session->getInt(PARAM_HEIGHT);
			string format = session->getString(PARAM_FORMAT);
			
			session->lock(PROPERTY_IMAGE, HERE);
			RImage *image = session->getObjectAsL<RImage>(PROPERTY_IMAGE);
			
			if (format=="YUYV") {
				camera->init(V4L2_PIX_FMT_YUYV,width,height,fps,iom);
				image->setSizeAndType(width, height, RDK2::RGraphics::RImage::YUYV);
			}
			else if (format=="RGB24") {
				camera->init(V4L2_PIX_FMT_RGB24,width,height,fps,iom);
				image->setSizeAndType(width, height, RDK2::RGraphics::RImage::RGB24);
			}
			else {
				cout << "V4L2Module: Format " << format << " unsupported" << endl;
			}
			
			session->unlock(PROPERTY_IMAGE);
			
			noimage=false;

		SESSION_END(session) 
			return true;
		SESSION_CATCH_TERMINATE(session)
			return false;
	}

	void QuadV4L2Module::exec()
	{
		while (session->wait(), !exiting) {
			SESSION_TRY_START(session)
				if(session->getBool(PROPERTY_MODULE_ENABLED)) {
					if (camera->ready()) {
						//Timestamp t;
						camera->getFrame();
						
						//REPOSITORY
						session->lock(PROPERTY_IMAGE, HERE);
						RImage *image = session->getObjectAsL<RImage>(PROPERTY_IMAGE);
						unsigned char *buf = image->getBuffer();
						memcpy(buf,camera->image.data,camera->image.width*camera->image.height*camera->image.bpp);
						session->unlock(PROPERTY_IMAGE);
						session->valueChanged(PROPERTY_IMAGE);
						
						if(session->getBool(PARAM_SAVE)) {
							string prefix = session->getString(PARAM_PREFIX);
							
							ostringstream off;
							int t=floor(Timestamp().getMsFromMidnight());
							int sec=floor(t/1000);
							int min=floor(sec/60);
							int h=floor(min/60);
							
							//string type = session->getString(PARAM_TYPE);
							bool jpg = session->getBool(PARAM_JPEG);
							
							if(jpg) {
								off << prefix << h << "-" << min % 60 << "-" << sec % 60 << "_" << t % 1000 << "(" << counter <<").jpg";
								camera->image.saveJPEG(off.str().c_str());
								//camera->image.saveJPEG("Prova.jpg");
							}
							else {
								off << prefix << h << "-" << min % 60 << "-" << sec % 60 << "_" << t % 1000 << "(" << counter <<").ppm";
								camera->image.savePPM(off.str().c_str());
							}
							
							printf("Saved photo n. %d\n", counter);
							counter++;
							int limit = session->getInt(PARAM_PHOTOS_NUM);
							if(limit != -1) {
								savedPhotos++;
								if(savedPhotos>=limit) {
									savedPhotos = 0;
									session->setBool(PARAM_SAVE, false);
								}
							}
						}
				} //if camera->ready()
				else {
					if (!noimage) {
						noimage=true;
						RDK_ERROR_PRINTF("No image captured."); 
					}
					session->lock(PROPERTY_IMAGE, HERE);
					RImage *image = session->getObjectAsL<RImage>(PROPERTY_IMAGE);
					
					if 	(session->getString(PARAM_FORMAT)!="RGB24")
						image->setSizeAndType(image->getWidth(),image->getHeight(),RDK2::RGraphics::RImage::RGB24);
					
					unsigned char *buf = image->getBuffer();
					// create a default image
					unsigned int k=0;
					while (k<height) {
						unsigned char r,g,b;
						if (k<height/4) {
							r=255; g=0; b=0;
						}
						else if (k<height/2) {
							r=0; g=255; b=0;
						}
						else if (k<height*3/4) {
							r=0; g=0; b=255;
						}
						else {
							r=255; g=255; b=0;
						}
						for (unsigned int i=0; i<width; i++) {
							*buf=r; *(buf+1)=g; *(buf+2)=b; buf+=3;
						}
						k++;
					}//while
					
					session->unlock(PROPERTY_IMAGE);
					session->valueChanged(PROPERTY_IMAGE);
				}//fine else
			}//fine if enabled
			SESSION_END_CATCH_TERMINATE(session)
		}
	}

	// implement this if you need to force the exec to go out of the loop
	// if the thread may be waiting not for the session->wait() semaphore:
	// on closing, the main thread will set the exiting variables, call this exitRequested()
	// function and then signal the session semaphores
	// void QuadV4L2Module::exitRequested() { }

	// implement this if you need to clean up things after the exec has exited
	void QuadV4L2Module::cleanup() { 
		camera->done();
		delete camera;
	}

	// void QuadV4L2Module::asyncAgentCmd(cstr cmd)
	// {
	//	SESSION_TRY_START(asyncSession)
	//	// here you can parse 'cmd'
	//	SESSION_END_CATCH_TERMINATE(asyncSession)
	// }

	MODULE_FACTORY(QuadV4L2Module);

}} // namespace
