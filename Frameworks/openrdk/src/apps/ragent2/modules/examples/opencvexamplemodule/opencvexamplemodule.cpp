#define MODULE_NAME "OpenCVExampleModule"

#include "opencvexamplemodule.h"
#include "opencvexamplemodule_names.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/ns.h>
#include <rdkcore/interop/opencv.h>
#define LOGGING_MODULE MODULE_NAME

#include <opencv/cv.h>

namespace RDK2 { namespace RAgent {

bool OpenCVExampleModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		session->createEmptyImage(PROPERTY_IN_IMAGE, "Input image");
		session->createEmptyImage(PROPERTY_OUT_IMAGE, "Output image");
		session->createString(PROPERTY_ACTION, "Action (can be \"sobel\", \"smooth\", \"pyrup\", \"pyrdown\", "
			"\"flipx\", \"flipy\", \"flipxy\", \"not\", \"laplace\")", "sobel");
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool OpenCVExampleModule::init()
{
	SESSION_TRY_START(session)
		session->listen(PROPERTY_IN_IMAGE);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void OpenCVExampleModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
			
			RImage* rin = session->getObjectCloneAs<RImage>(PROPERTY_IN_IMAGE);
			IplImage* cvin = createIplImageHeader(rin);
			
			RImage* rout = 0;

			string action = session->getString(PROPERTY_ACTION);
			if (action == "sobel" || action == "smooth" || action == "flipx" || action == "flipy"
			|| action == "flipxy" || action == "not") {
				rout = (RImage*) rin->clone();
				IplImage* cvout = createIplImageHeader(rout);
				if (action == "sobel") cvSobel(cvin, cvout, 1, 1, 3);
				else if (action == "smooth") cvSmooth(cvin, cvout, CV_GAUSSIAN, 9);
				else if (action == "flipx") cvFlip(cvin, cvout, 1);
				else if (action == "flipy") cvFlip(cvin, cvout, 0);
				else if (action == "flipxy") cvFlip(cvin, cvout, -1);
				else if (action == "not") cvNot(cvin, cvout);
				cvReleaseImageHeader(&cvout);
			}
			else if (action == "pyrdown") {
				rout = new RImage(rin->getWidth() / 2, rin->getHeight() / 2, rin->getType());
				IplImage* cvout = createIplImageHeader(rout);
				cvPyrDown(cvin, cvout);
				cvReleaseImageHeader(&cvout);
			}
			else if (action == "pyrup") {
				rout = new RImage(rin->getWidth() * 2, rin->getHeight() * 2, rin->getType());
				IplImage* cvout = createIplImageHeader(rout);
				cvPyrUp(cvin, cvout);
				cvReleaseImageHeader(&cvout);
			}
			else if (action == "laplace") {
				IplImage* cvout = cvCreateImage(cvSize(rin->getWidth(), rin->getHeight()),
					IPL_DEPTH_16S, rin->getBytesPerPixel());
				cvLaplace(cvin, cvout);
				rout = convertToRImage(cvout);
				cvReleaseImage(&cvout);
			}

			cvReleaseImageHeader(&cvin);		
	
			if (rout) session->setObject(PROPERTY_OUT_IMAGE, rout);
			delete rin;

		SESSION_END_CATCH_TERMINATE(session)
	}
}

MODULE_FACTORY(OpenCVExampleModule);

}} // namespace
