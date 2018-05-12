/**
 * @file
 * @brief This file contains the declaration of ImageTestModule
 */

#ifndef RDK_MODULE_IMAGETESTMODULE
#define RDK_MODULE_IMAGETESTMODULE

#include <rdkcore/modules/module.h>

namespace RDK2 { namespace RAgent {

/**
 * @brief This module can be used if you need an image.
 *
 * This module contains an image that can be used as a test image for visualization, message sending, etc.
 *
 * @ingroup RAgentModules
 */

class ImageTestModule : public Module {
public:
	ImageTestModule() { }
	virtual ~ImageTestModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
};

}} // namespace

#endif
