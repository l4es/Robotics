#define MODULE_NAME "ImageMagickExampleModule"

#include "imagemagickexamplemodule.h"
#include "imagemagickexamplemodule_names.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/interop/magick++.h>
#include <rdkcore/ns.h>
#define LOGGING_MODULE MODULE_NAME

#include <Magick++.h>

namespace RDK2 { namespace RAgent {

bool ImageMagickExampleModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		session->createEmptyImage(PROPERTY_IN_IMAGE, "Input image");
		session->createEmptyImage(PROPERTY_OUT_IMAGE, "Output image");
		session->createBool(PROPERTY_ADD_NOISE, "Add noise", false);
		session->createBool(PROPERTY_BLUR, "Blur image", false);
		session->createBool(PROPERTY_CHARCOAL, "Charcoal effect", false);
		session->createBool(PROPERTY_CONTRAST, "Contrast image", false);
		session->createBool(PROPERTY_EMBOSS, "Emboss image", false);
		session->createBool(PROPERTY_EDGE, "Find edges", false);
		session->createBool(PROPERTY_ENHANCE, "Enhance image", false);
		session->createBool(PROPERTY_EQUALIZE, "Equalize image", false);
		session->createBool(PROPERTY_FLIP, "Flip image", false);
		session->createBool(PROPERTY_FLOP, "Flop image", false);
		session->createBool(PROPERTY_IMPLODE, "Implode image", false);
		session->createBool(PROPERTY_OIL_PAINT, "Oil paint effect", false);
		session->createBool(PROPERTY_SOLARIZE, "Solarize", false);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool ImageMagickExampleModule::init()
{
	// in the init() function you should initialize everything your module needs (drivers initialization,
	// sockets, and so on); all modules wait for all init() before they start
	SESSION_TRY_START(session)
		session->listen(PROPERTY_IN_IMAGE);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void ImageMagickExampleModule::exec()
{
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
			RImage* rin = session->getObjectCloneAs<RImage>(PROPERTY_IN_IMAGE);
			Magick::Image mimg = convertToMagickImage(rin);
			delete rin;

			if (session->getBool(PROPERTY_ADD_NOISE)) mimg.addNoise(Magick::GaussianNoise);
			if (session->getBool(PROPERTY_BLUR)) mimg.blur();
			if (session->getBool(PROPERTY_CHARCOAL)) mimg.charcoal();
			if (session->getBool(PROPERTY_CONTRAST)) mimg.contrast(2);
			if (session->getBool(PROPERTY_EMBOSS)) mimg.emboss();
			if (session->getBool(PROPERTY_EDGE)) mimg.edge();
			if (session->getBool(PROPERTY_ENHANCE)) mimg.enhance();
			if (session->getBool(PROPERTY_EQUALIZE)) mimg.equalize();
			if (session->getBool(PROPERTY_FLIP)) mimg.flip();
			if (session->getBool(PROPERTY_FLOP)) mimg.flop();
			if (session->getBool(PROPERTY_IMPLODE)) mimg.implode(0.5);
			if (session->getBool(PROPERTY_OIL_PAINT)) mimg.oilPaint();
			if (session->getBool(PROPERTY_SOLARIZE)) mimg.solarize();

			RImage* rout = convertToRImage(mimg);
			session->setObject(PROPERTY_OUT_IMAGE, rout);
		SESSION_END_CATCH_TERMINATE(session)
	}
}

MODULE_FACTORY(ImageMagickExampleModule);

}} // namespace
