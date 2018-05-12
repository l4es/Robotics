/**
 * @file
 * @brief This file contains the declaration of ImageMagickExampleModule
 */

#ifndef RDK_MODULE_IMAGEMAGICKEXAMPLEMODULE
#define RDK_MODULE_IMAGEMAGICKEXAMPLEMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief Please write a SHORT description of ImageMagickExampleModule here.
 *
 * Please write a description of ImageMagickExampleModule here.
 *
 * @ingroup RAgentModules
 */

#define EVENT_HANDLER_CLASS ImageMagickExampleModule

class ImageMagickExampleModule : public Module {
public:
	ImageMagickExampleModule() { }
	virtual ~ImageMagickExampleModule() { }

	bool initConfigurationProperties();
	// bool initInterfaceProperties();
	bool init();
	void exec();
	// void exitRequested();
	// void cleanup();
	// void asyncAgentCmd(cstr cmd);
};

}} // namespace

#endif
