#define MODULE_NAME "ImageTestModule"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/rgraphics/rimage.h>
#define LOGGING_MODULE MODULE_NAME

#include "imagetestmodule.h"
#include "imagetestmodule_properties.h"

#include "perlinnoise.h"

namespace RDK2 { namespace RAgent {

using namespace RDK2::RGraphics;

bool ImageTestModule::initConfigurationProperties()
{
	SESSION_TRY_START(session)
		// put false in the second parameter if you want the module disabled for default
		// add a third parameter valued false if you don't want the state property
		Common::createDefaultProperties(session, true);
		ENUM_CREATE(imageShown);
		ENUM_ITEM(imageShown, 0, "Black", "Black image");
		ENUM_ITEM(imageShown, 1, "White", "White image");
		ENUM_ITEM(imageShown, 2, "Shaded", "Shaded colored image");
		ENUM_ITEM(imageShown, 3, "Green/Red/Blue", "Image changing from green to red to blue");
		ENUM_ITEM(imageShown, 4, "Perlin noise", "Perlin noise");
		ENUM_ITEM(imageShown, 5, "Plasma", "Plasma effect (perlin noise)");
		session->createEnum(PROPERTY_IMAGE_SHOWN, "Image shown", imageShown, 2);
		session->createImage(PROPERTY_IMAGE, "Image", 320, 240, RImage::RGB32);
		
		session->createDouble(PROPERTY_PERLIN_NOISE_PERSISTENCE, "Perlin noise persistence", RDouble::REAL, 0.5);
		session->createDouble(PROPERTY_PERLIN_NOISE_TIME_SCALE, "Perlin noise time scale", RDouble::REAL, 0.05);
		session->createDouble(PROPERTY_PERLIN_NOISE_SPACE_SCALE, "Perlin noise space scale", RDouble::REAL, 0.02);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

bool ImageTestModule::init()
{
	SESSION_TRY_START(session)
		session->listenToTimer(100.);
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void ImageTestModule::exec()
{
	int iteration = 0;
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)

		if (session->getBool(PROPERTY_MODULE_ENABLED)) {
			int imageShown = session->getInt(PROPERTY_IMAGE_SHOWN);
			double perlinNoisePersistence = session->getDouble(PROPERTY_PERLIN_NOISE_PERSISTENCE);
			double perlinNoiseTimeScale = session->getDouble(PROPERTY_PERLIN_NOISE_TIME_SCALE);
			double perlinNoiseSpaceScale = session->getDouble(PROPERTY_PERLIN_NOISE_SPACE_SCALE);
			
			session->lock(PROPERTY_IMAGE, HERE);
			RImage* img = session->getObjectAsL<RImage>(PROPERTY_IMAGE);
			int w = img->getWidth(), h = img->getHeight();
			switch (imageShown) {
				case 0: {
					for (size_t i = 0; i < img->getBufferSize(); i++) img->getBuffer()[i] = 0;
				} break;
				case 1: {
					for (size_t y = 0; y < img->getHeight(); y++) {
						for (size_t x = 0; x < img->getWidth(); x++) {
							for (size_t c = 0; c < img->getBytesPerPixel(); c++) {
								img->setPixel(x, y, c, 255);
							}
						}
					}
				} break;
				case 2: {
					for (int y = 0; y < h; y++) { for (int x = 0; x < w; x++) {
						img->getBuffer()[y * img->getWidth() * 4 + x*4] = y * 255 / h;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+1] = x * 255 / w;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+2] = 0;
					}}
				} break;
				case 3: {
					for (int x = 0, y = 0; y < h; ++x >= w ? x = 0, y++ : 1-1000) {
						img->getBuffer()[y * img->getWidth() * 4 + x*4+0] = iteration % 255 + y;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+1] = iteration * 2 % 255 + x;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+2] = iteration * 3 % 255 + x * y;
					}
				} break;
				case 4: {
					for (int x = 0, y = 0; y < h; ++x >= w ? x = 0, y++: 90-60-90) {
						int c = (perlinNoise3D((double) x * perlinNoiseSpaceScale,
							(double) y * perlinNoiseSpaceScale, 
							perlinNoiseTimeScale * iteration, 
							perlinNoisePersistence, false) + 0.5) * 255;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+0] = c;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+1] = c;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+2] = c;
					}
				} break;
				case 5: {
					for (int x = 0, y = 0; y < h; ++x >= w ? x = 0, y++: 90-60-90) {
						int c1 = (originalPerlinNoise3D((double) x * perlinNoiseSpaceScale,
							(double) y * perlinNoiseSpaceScale, 
							perlinNoiseTimeScale * iteration, 
							perlinNoisePersistence * 0.05, 0.01) + 0.5) * 255;
						int c2 = (originalPerlinNoise3D((double) x * perlinNoiseSpaceScale,
							(double) y * perlinNoiseSpaceScale, 
							perlinNoiseTimeScale * iteration * 2, 
							perlinNoisePersistence * 0.05, 0.1) + 0.5) * 255;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+0] = c1;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+1] = c2;
						img->getBuffer()[y * img->getWidth() * 4 + x*4+2] = 0;
					}
				} break;
			}
			session->unlock(PROPERTY_IMAGE);
			session->valueChanged(PROPERTY_IMAGE);
		}

		iteration++;
		SESSION_END_CATCH_TERMINATE(session)
	}
}

MODULE_FACTORY(ImageTestModule);

}} // namespace
