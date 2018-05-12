#define MODULE_NAME "VisionDemoModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/rgraphics/rimage.h>

#define LOGGING_MODULE MODULE_NAME

#include "visiondemomodule.h"

// it is better to declare some define to be used as property names, in order to avoid misspelling in strings,
// declare here those defines for all your properties and use these; for example:

#define PROPERTY_IN_IMAGE  "in/image"
#define PROPERTY_OUT_AVGINT  "out/avg_intensity"


#define DEFAULT_WIDTH 320
#define DEFAULT_HEIGHT 240


namespace RDK2 { namespace RAgent {

bool VisionDemoModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		// here you declare the properties of your module
		// in the repository they have the name of this module as prefix, e.g. /yourModuleName/maxSpeed
		// session->createString(PROPERTY_MY_BEAUTIFUL_STRING, "My beautiful string", "Default value");
		session->createDouble(PROPERTY_OUT_AVGINT, "Avg image intensity", RDouble::REAL, 0.0);

		// // example of enum creation
		// ENUM_CREATE(whichStalled);
		// ENUM_ITEM(whichStalled, NONE, "None", "No wheel stalled");
		// ENUM_ITEM(whichStalled, LEFT, "Left", "Left wheels stalled");
		// ENUM_ITEM(whichStalled, RIGHT, "Right", "Right wheels stalled");
		// ENUM_ITEM(whichStalled, BOTH, "Both", "Full robot stall");
		// session->createEnum(PROPERTY_ROBOT_WHICH_STALLED, "Which robot wheel stalled", whichStalled, NONE, INFO);
		session->createImage(PROPERTY_IN_IMAGE, "Current Image", DEFAULT_WIDTH, DEFAULT_HEIGHT, RDK2::RGraphics::RImage::RGB24);

		// // example of vector creation
		// session->createVector<RType>(PROPERTY_MY_VECTOR, "My vector");

	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

// bool VisionDemoModule::initInterfaceProperties() { }

bool VisionDemoModule::init()
{
	// in the init() function you should initialize everything your module needs (drivers initialization,
	// sockets, and so on); all modules wait for all init() before they start
	SESSION_TRY_START(session)
		// here you can declare the events you are waiting in the main exec() loop,
		// for example:
		//session->listenToTimer(500.);
		session->listen(PROPERTY_IN_IMAGE);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void VisionDemoModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)

		session->lock(PROPERTY_IN_IMAGE, HERE);
		RImage *image = session->getObjectAsL<RImage>(PROPERTY_IN_IMAGE);

		int bpp = 3;
		if (image->getType() == RDK2::RGraphics::RImage::RGB32 ||
				image->getType() == RDK2::RGraphics::RImage::YUYV)
		{
			bpp = 4;
		}
		if (image->getType() == RDK2::RGraphics::RImage::RGB32 ||
				image->getType() == RDK2::RGraphics::RImage::RGB24
				)
	 	{
			unsigned char *buf = image->getBuffer();
			int size = image->getDataSize();
	
			unsigned char *p, *pend; double sr=0, sg=0, sb=0;
			p = buf; pend = p + image->getBufferSize();
			while (p<pend) {
				sr+=*p; sg+=*(p+1); sb+=*(p+2);
				p+=bpp;
			}
			sr /= size; sg /= size; sb /= size;
	
			RDK_DEBUG_STREAM("average color = " << sr << " " << sg << " " << sb);
			session->setDouble(PROPERTY_OUT_AVGINT,(sr+sg+sb)/3);
		}
		else if (image->getType() == RDK2::RGraphics::RImage::YUYV) {
			unsigned char *buf = image->getBuffer();
			int size = image->getDataSize();

			unsigned char *p, *pend; double sy=0;
			p = buf; pend = p + image->getBufferSize();
			while (p<pend) {
				unsigned char y1 = *p, /*v = *(p+1),*/ y2 = *(p+2)/*, u = *(p+3)*/;
				sy+=y1+y2; p+=bpp;
			}
			sy /= size;
			RDK_DEBUG_STREAM("average intensity = " << sy);
			session->setDouble(PROPERTY_OUT_AVGINT,sy);

		}
		else RDK_DEBUG_STREAM("Not a RGB24/32 or YUYV image!");

		session->unlock(PROPERTY_IN_IMAGE);

		SESSION_END_CATCH_TERMINATE(session)
	}
}

// implement this if you need to force the exec to go out of the loop
// if the thread may be waiting not for the session->wait() semaphore:
// on closing, the main thread will set the exiting variables, call this exitRequested()
// function and then signal the session semaphores
// void VisionDemoModule::exitRequested() { }

// implement this if you need to clean up things after the exec has exited
// void VisionDemoModule::cleanup() { }

// void VisionDemoModule::asyncAgentCmd(cstr cmd)
// {
//	SESSION_TRY_START(asyncSession)
//	// here you can parse 'cmd'
//	SESSION_END_CATCH_TERMINATE(asyncSession)
// }

MODULE_FACTORY(VisionDemoModule);

}} // namespace
